"""Jackal execution bridge for APPRCA live object goals.

Runs on the robot. Owns the ZED, streams RGB/depth to the server, receives
semantic target guidance, and performs low-speed local approach using ZED depth
with a Velodyne safety check. Motion is disabled unless ENABLE_MOTION=1.
"""

import json
import math
import os
import queue
import signal
import threading
import time

import cv2
import numpy as np
import pyzed.sl as sl
import rclpy
import websocket

from mycam9 import JackalDriver, compute_corridor_min_z, depth_to_colormap, put_latest


SERVER = os.environ.get("SERVER", "ws://127.0.0.1:8050").rstrip("/")
API_TOKEN = os.environ.get("API_TOKEN", "")
CAMERA_ID = os.environ.get("CAMERA_ID", "jackal-zed2i")
ENABLE_MOTION = os.environ.get("ENABLE_MOTION", "0") in ("1", "true", "True", "yes", "YES")
STANDOFF_M = float(os.environ.get("STANDOFF_M", "1.0"))
MAX_FORWARD_MPS = float(os.environ.get("MAX_FORWARD_MPS", "0.18"))
SEARCH_RADPS = float(os.environ.get("SEARCH_RADPS", "0.18"))
MAX_TURN_RADPS = float(os.environ.get("MAX_TURN_RADPS", "0.45"))
CONTROL_TIMEOUT_S = float(os.environ.get("CONTROL_TIMEOUT_S", "1.0"))
LIDAR_STALE_S = float(os.environ.get("LIDAR_STALE_S", "0.5"))
OBSTACLE_STOP_M = float(os.environ.get("OBSTACLE_STOP_M", "0.60"))

STOP = threading.Event()
LEFT_Q, RIGHT_Q, STATUS_Q = queue.Queue(1), queue.Queue(1), queue.Queue(1)
CONTROL_LOCK = threading.Lock()
CONTROL = {"message": {"type": "stop", "state": "IDLE"}, "received": 0.0}


def headers():
    return [f"Authorization: Bearer {API_TOKEN}"] if API_TOKEN else None


def sender(url, q, binary):
    while not STOP.is_set():
        ws = None
        try:
            ws = websocket.create_connection(url, timeout=5, header=headers())
            while not STOP.is_set():
                try:
                    value = q.get(timeout=0.5)
                except queue.Empty:
                    continue
                (ws.send_binary if binary else ws.send)(value)
        except Exception as exc:
            print(f"[WS] sender reconnect: {exc}")
            STOP.wait(1.0)
        finally:
            if ws:
                ws.close()


def control_receiver():
    url = f"{SERVER}/ws/control?camera_id={CAMERA_ID}"
    while not STOP.is_set():
        ws = None
        try:
            ws = websocket.create_connection(url, timeout=5)
            ws.settimeout(1.0)
            while not STOP.is_set():
                try:
                    msg = json.loads(ws.recv())
                    with CONTROL_LOCK:
                        CONTROL["message"] = msg
                        CONTROL["received"] = time.monotonic()
                except websocket.WebSocketTimeoutException:
                    continue
        except Exception as exc:
            print(f"[WS] control reconnect: {exc}")
            STOP.wait(1.0)
        finally:
            if ws:
                ws.close()


def median_target_depth(depth, bbox):
    h, w = depth.shape
    x1, y1, x2, y2 = bbox
    # Use the central 60% to reduce background contamination around a DINO box.
    x1, x2 = x1 + .2 * (x2 - x1), x2 - .2 * (x2 - x1)
    y1, y2 = y1 + .2 * (y2 - y1), y2 - .2 * (y2 - y1)
    xa, xb = max(0, int(x1 * w)), min(w, int(x2 * w))
    ya, yb = max(0, int(y1 * h)), min(h, int(y2 * h))
    values = depth[ya:yb, xa:xb]
    valid = values[np.isfinite(values) & (values > 0.2) & (values < 10.0)]
    return float(np.median(valid)) if valid.size >= 30 else float("nan")


def main():
    signal.signal(signal.SIGINT, lambda *_: STOP.set())
    query = f"camera_id={CAMERA_ID}"
    endpoints = [
        (f"{SERVER}/ws/push?{query}&eye=left", LEFT_Q, True),
        (f"{SERVER}/ws/push?{query}&eye=right", RIGHT_Q, True),
        (f"{SERVER}/ws/status/push?{query}", STATUS_Q, False),
    ]
    for args in endpoints:
        threading.Thread(target=sender, args=args, daemon=True).start()
    threading.Thread(target=control_receiver, daemon=True).start()

    rclpy.init()
    driver = JackalDriver()
    zed = sl.Camera()
    init = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720, camera_fps=30,
                             depth_mode=sl.DEPTH_MODE.PERFORMANCE,
                             coordinate_units=sl.UNIT.METER)
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        driver.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(f"ZED open failed: {status}")
    runtime = sl.RuntimeParameters(enable_depth=True)
    image_mat, depth_mat = sl.Mat(), sl.Mat()
    info = zed.get_camera_information().camera_configuration.calibration_parameters.left_cam
    fx, cx = float(info.fx), float(info.cx)
    last_status = 0.0
    print(f"[READY] motion={'ENABLED' if ENABLE_MOTION else 'DRY-RUN'} server={SERVER}")

    try:
        while rclpy.ok() and not STOP.is_set():
            rclpy.spin_once(driver, timeout_sec=0.0)
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                driver.stop()
                STOP.wait(0.01)
                continue
            zed.retrieve_image(image_mat, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
            bgr = image_mat.get_data()[:, :, :3].copy()
            depth = depth_mat.get_data().astype(np.float32)

            ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 82])
            if ok:
                put_latest(LEFT_Q, jpg.tobytes())
            depth_vis = depth_to_colormap(depth)
            ok, jpg = cv2.imencode(".jpg", depth_vis, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                put_latest(RIGHT_Q, jpg.tobytes())

            corr_zed, left_zed, right_zed = compute_corridor_min_z(depth, fx, cx)
            corr_lid, left_lid, right_lid, lidar_age = driver.get_lidar_mins()
            if lidar_age > LIDAR_STALE_S:
                corr_lid = left_lid = right_lid = float("nan")
            ranges = [v for v in (corr_zed, corr_lid) if math.isfinite(v)]
            corridor = min(ranges) if ranges else float("nan")

            with CONTROL_LOCK:
                command = dict(CONTROL["message"])
                command_age = time.monotonic() - CONTROL["received"]
            target = command.get("target", "")
            state, message, distance = "STOPPED", "", float("nan")
            v = w = 0.0

            if command_age > CONTROL_TIMEOUT_S:
                state, message = "FAILSAFE", "control connection stale"
            elif command.get("type") == "stop":
                state, message = "STOPPED", "stop requested"
            elif not math.isfinite(corridor):
                state, message = "FAILSAFE", "no valid ZED/LiDAR range"
            elif command.get("state") == "searching":
                state, message, w = "SEARCHING", "rotating to find target", SEARCH_RADPS
            elif command.get("state") == "verified" and len(command.get("bbox_norm", [])) == 4:
                bbox = command["bbox_norm"]
                distance = median_target_depth(depth, bbox)
                center_error = ((bbox[0] + bbox[2]) * 0.5 - 0.5) * 2.0
                if not math.isfinite(distance):
                    state, message = "TARGET_LOST", "target depth unavailable"
                elif distance <= STANDOFF_M:
                    state, message = "ARRIVED", "goal reached"
                elif corridor <= OBSTACLE_STOP_M:
                    state, message = "AVOIDING", "obstacle inside safety distance"
                    left = min(v for v in (left_zed, left_lid) if math.isfinite(v)) if any(math.isfinite(v) for v in (left_zed, left_lid)) else 0.0
                    right = min(v for v in (right_zed, right_lid) if math.isfinite(v)) if any(math.isfinite(v) for v in (right_zed, right_lid)) else 0.0
                    w = SEARCH_RADPS if left >= right else -SEARCH_RADPS
                else:
                    state, message = "APPROACHING", "tracking verified target"
                    w = float(np.clip(-0.55 * center_error, -MAX_TURN_RADPS, MAX_TURN_RADPS))
                    if abs(center_error) < 0.35:
                        v = min(MAX_FORWARD_MPS, max(0.0, 0.18 * (distance - STANDOFF_M)))
            else:
                state, message = "SEARCHING", "waiting for verified target"
                w = SEARCH_RADPS

            if not ENABLE_MOTION:
                v = w = 0.0
                message = "DRY-RUN: " + message
            driver.send(v, w)

            if time.monotonic() - last_status >= 0.25:
                status_msg = {"state": state, "target": target,
                              "distance_m": round(distance, 2) if math.isfinite(distance) else None,
                              "motion_enabled": ENABLE_MOTION, "message": message,
                              "timestamp": time.time()}
                put_latest(STATUS_Q, json.dumps(status_msg))
                last_status = time.monotonic()
    finally:
        STOP.set()
        driver.stop()
        zed.close()
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
