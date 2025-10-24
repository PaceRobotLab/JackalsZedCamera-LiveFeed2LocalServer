import os, sys, time, signal, threading, queue
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import pyzed.sl as sl
import cv2
import websocket  # pip install websocket-client

# ================ Config =================
SERVER      = os.environ.get("SERVER", "ws://192.168.0.118:8050")
API_TOKEN   = os.environ.get("API_TOKEN", "supersecret123")
CAMERA_ID   = os.environ.get("CAMERA_ID", "jackal-zed2i")

RESOLUTION  = os.environ.get("ZED_RES", "HD720")     # HD2K, HD1080, HD720, VGA
FPS         = int(os.environ.get("ZED_FPS", "30"))
DEPTH_MODE  = os.environ.get("ZED_DEPTH", "ULTRA")   # NONE, PERFORMANCE, QUALITY, ULTRA (avoid NEURAL on 1050 Ti)

JPEG_Q      = int(os.environ.get("JPEG_Q", "90"))

# Local saves OFF by default to reduce I/O jitter; flip to 1 if you want.
SAVE_LOCAL  = os.environ.get("SAVE_LOCAL", "0") not in ("0","false","False","no","NO")
SAVE_DIR    = Path(os.environ.get("SAVE_DIR", "/home/administrator/zed_ws_frames"))

# single-slot queues = always keep latest frame (no lag build-up)
QLEN = 1
left_q:  "queue.Queue[bytes]" = queue.Queue(maxsize=QLEN)
right_q: "queue.Queue[bytes]" = queue.Queue(maxsize=QLEN)

stop_event = threading.Event()

# ================ Helpers =================
def parse_resolution(name: str):
    m = name.upper()
    return {
        "HD2K": sl.RESOLUTION.HD2K,
        "HD1080": sl.RESOLUTION.HD1080,
        "HD720": sl.RESOLUTION.HD720,
        "VGA": sl.RESOLUTION.VGA
    }.get(m, sl.RESOLUTION.HD720)

def parse_depth(name: str):
    m = name.upper()
    return {
        "NONE": sl.DEPTH_MODE.NONE,
        "PERFORMANCE": sl.DEPTH_MODE.PERFORMANCE,
        "QUALITY": sl.DEPTH_MODE.QUALITY,
        "ULTRA": sl.DEPTH_MODE.ULTRA,
    }.get(m, sl.DEPTH_MODE.ULTRA)

def put_latest(q: queue.Queue, data: bytes):
    try:
        q.put_nowait(data)
    except queue.Full:
        try:
            q.get_nowait()
        except Exception:
            pass
        q.put_nowait(data)

def ws_sender(name: str, eye: str, q: "queue.Queue[bytes]"):
    url = f"{SERVER}/ws/push?camera_id={CAMERA_ID}&eye={eye}&authorization=Bearer%20{API_TOKEN}"
    # reconnect loop
    while not stop_event.is_set():
        ws = None
        try:
            # create_connection blocks; add timeout for safety
            ws = websocket.create_connection(url, timeout=5)
            print(f"[ws:{eye}] connected -> {url}")
            while not stop_event.is_set():
                try:
                    data = q.get(timeout=0.5)
                except queue.Empty:
                    continue
                try:
                    ws.send_binary(data)
                except Exception as e:
                    print(f"[ws:{eye}] send error: {e}")
                    break
        except Exception as e:
            print(f"[ws:{eye}] connect error: {e}")
        finally:
            if ws:
                try:
                    ws.close()
                except Exception:
                    pass
            # small backoff before reconnect
            time.sleep(1.0)

def handle_sigint(sig, frame):
    stop_event.set()

# ================ Main =================
def main():
    signal.signal(signal.SIGINT, handle_sigint)

    if SAVE_LOCAL:
        SAVE_DIR.mkdir(parents=True, exist_ok=True)

    # start sender threads
    tl = threading.Thread(target=ws_sender, args=("left","left",left_q), daemon=True)
    tr = threading.Thread(target=ws_sender, args=("right","right",right_q), daemon=True)
    tl.start(); tr.start()

    # ZED init
    zed = sl.Camera()
    init = sl.InitParameters(
        camera_resolution=parse_resolution(RESOLUTION),
        camera_fps=FPS,
        depth_mode=parse_depth(DEPTH_MODE),
        coordinate_units=sl.UNIT.MILLIMETER
    )
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ERR] ZED open failed: {status}")
        sys.exit(1)

    runtime = sl.RuntimeParameters(enable_depth=False)
    left_mat  = sl.Mat()
    right_mat = sl.Mat()

    print(f"[INFO] Streaming LEFT+RIGHT to {SERVER} as camera_id={CAMERA_ID}. Ctrl+C to stop.")

    frame_id = 0
    t0 = time.time()
    try:
        while not stop_event.is_set():
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.002)
                continue
            # LEFT
            zed.retrieve_image(left_mat, sl.VIEW.LEFT)  # BGRA
            npL = left_mat.get_data()
            if npL is not None and npL.size > 0:
               # bgrL = npL[:, :, :3][:, :, ::-1]
               # bgrL = npL[:, :, :3][:, :, [1, 2, 0]]
                bgrL = npL[:, :, :3]
                okL, jpgL = cv2.imencode(".jpg", bgrL, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
                if okL:
                    put_latest(left_q, jpgL.tobytes())
                    if SAVE_LOCAL:
                        (SAVE_DIR / f"left_{frame_id:06d}.jpg").write_bytes(jpgL.tobytes())

            # RIGHT
            zed.retrieve_image(right_mat, sl.VIEW.RIGHT)  # BGRA
            npR = right_mat.get_data()
            if npR is not None and npR.size > 0:
               # bgrR = npR[:, :, :3][:, :, ::-1]
               # bgrR = npR[:, :, :3][:, :, [1, 2, 0]]
                bgrR = npR[:, :, :3]
                okR, jpgR = cv2.imencode(".jpg", bgrR, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
                if okR:
                    put_latest(right_q, jpgR.tobytes())
                    if SAVE_LOCAL:
                        (SAVE_DIR / f"right_{frame_id:06d}.jpg").write_bytes(jpgR.tobytes())

            frame_id += 1

    finally:
        zed.close()
        stop_event.set()
        tl.join(timeout=2); tr.join(timeout=2)
        dt = time.time() - t0
        fps_pairs = frame_id / dt if dt > 0 else 0.0
        print(f"[INFO] Done. Frames: {frame_id} pairs in {dt:.1f}s (~{fps_pairs:.1f} fps pairs).")

if __name__ == "__main__":
    main()
