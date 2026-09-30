#!/usr/bin/env python3
import os
import sys
import time
import math
import signal
import threading
import queue
from typing import Tuple, Optional

import numpy as np
import cv2
import pyzed.sl as sl
import websocket  # pip install websocket-client

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

# ====================================================
#              CONFIG / CONSTANTS
# ====================================================

CMD_TOPIC = os.environ.get("CMD_TOPIC", "/j100_0390/platform/cmd_vel_unstamped")

# WebSocket server (your laptop)
SERVER    = os.environ.get("SERVER", "ws://127.0.0.1:8050").rstrip("/")
API_TOKEN = os.environ.get("API_TOKEN", "")
CAMERA_ID = os.environ.get("CAMERA_ID", "jackal-zed2i")

# UGV geometry
UGV_WIDTH_M  = 0.43          # 430 mm
UGV_HALF_M   = UGV_WIDTH_M / 2.0

# Camera model (approx)
HFOV_DEG     = 90.0          # horizontal FOV (tune if you know exact)
HFOV_RAD     = math.radians(HFOV_DEG)

# Motion parameters
FWD_SPEED        = 0.30      # m/s when path is clear
TURN_SPEED       = 0.50      # rad/s turning-in-place when corridor blocked
BACK_SPEED       = -0.15     # m/s backup if everything is very close
LOOP_DT          = 0.03      # ~33 Hz control loop
BACKUP_TIME      = 0.7       # s of backup when trapped
ALLOW_BACKUP     = os.environ.get("ALLOW_BACKUP", "0") in ("1", "true", "True", "yes", "YES")

# Depth thresholds
MAX_DEPTH_CLIP   = 5.0       # m, for viz and filtering

# Corridor thresholds + hysteresis
BLOCK_ENTER_DIST = 0.90      # enter "BLOCKED" when fused corridor < this
BLOCK_EXIT_DIST  = 1.20      # exit "BLOCKED" only when fused corridor > this
STOP_DIST        = 0.45      # very near; prefer stop/turn/backup

# Region of image we treat as "ground band" in front of UGV
BOTTOM_FRACTION  = 0.4       # use bottom 40% of rows
ROW_MARGIN       = 4         # small margin from very bottom

# Velodyne config
LIDAR_TOPIC      = os.environ.get("LIDAR_TOPIC", "/velodyne_points")
LIDAR_MAX_RANGE  = 30.0      # m
LIDAR_MIN_RANGE  = 0.05      # m
LIDAR_FRONT_YAW  = math.radians(60)  # use +/-60 deg in front
LIDAR_STALE_S    = float(os.environ.get("LIDAR_STALE_S", "0.5"))

# Streaming queues: LEFT = annotated RGB, RIGHT = depth colormap
LEFT_Q  = queue.Queue(maxsize=1)
RIGHT_Q = queue.Queue(maxsize=1)

STOP_EVENT = threading.Event()


# ====================================================
#              ROS2 JACKAL DRIVER + LIDAR
# ====================================================

class JackalDriver(Node):
    """
    Node that:
      - publishes Twist to drive Jackal
      - subscribes to Velodyne PointCloud2
      - keeps latest forward corridor + side clearances from LiDAR
    """
    def __init__(self):
        super().__init__("zed_velodyne_nav_ugv")
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)

        # LiDAR fusion state
        self._lidar_corridor_min_z: float = float("nan")
        self._lidar_left_min_z: float     = float("nan")
        self._lidar_right_min_z: float    = float("nan")
        self._lidar_lock = threading.Lock()
        self._last_lidar_time: float = 0.0

        # Subscribe to Velodyne point cloud
        self.sub_lidar = self.create_subscription(
            PointCloud2,
            LIDAR_TOPIC,
            self.lidar_callback,
            qos_profile_sensor_data,
        )

    # --------------- motion ---------------
    def send(self, v: float, w: float):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def stop(self):
        self.send(0.0, 0.0)

    # --------------- LiDAR handling ---------------
    def lidar_callback(self, msg: PointCloud2):
        """
        Process Velodyne point cloud, extract:
          - nearest obstacle in 0.43m corridor in front
          - nearest left side obstacle (y > UGV_HALF_M)
          - nearest right side obstacle (y < -UGV_HALF_M)

        Assume frame where:
           x = forward, y = left, z = up.
        Only use points with x > 0 (front hemisphere).
        """
        corridor_min = float("nan")
        left_min     = float("nan")
        right_min    = float("nan")

        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])

            # Only consider points in front of robot
            if x <= 0.0:
                continue

            r = math.sqrt(x * x + y * y)
            if r < LIDAR_MIN_RANGE or r > LIDAR_MAX_RANGE:
                continue

            # Restrict to front sector (+/- LIDAR_FRONT_YAW)
            angle = math.atan2(y, x)
            if abs(angle) > LIDAR_FRONT_YAW:
                continue

            # Use x as forward distance, y as lateral (left +, right -)
            forward = x
            lateral = y

            if abs(lateral) <= UGV_HALF_M:
                if (not math.isfinite(corridor_min)) or (forward < corridor_min):
                    corridor_min = forward
            elif lateral < -UGV_HALF_M:
                if (not math.isfinite(right_min)) or (forward < right_min):
                    right_min = forward
            elif lateral > UGV_HALF_M:
                if (not math.isfinite(left_min)) or (forward < left_min):
                    left_min = forward

        with self._lidar_lock:
            self._lidar_corridor_min_z = corridor_min
            self._lidar_left_min_z     = left_min
            self._lidar_right_min_z    = right_min
            self._last_lidar_time      = time.time()

    def get_lidar_mins(self) -> Tuple[float, float, float, float]:
        """
        Return (corridor_min_z, left_min_z, right_min_z, age_s) from last LiDAR update.
        """
        with self._lidar_lock:
            age = time.time() - self._last_lidar_time if self._last_lidar_time > 0 else float("inf")
            return (
                self._lidar_corridor_min_z,
                self._lidar_left_min_z,
                self._lidar_right_min_z,
                age,
            )


# ====================================================
#              WEBSOCKET SENDER
# ====================================================

def ws_sender(eye: str, q: "queue.Queue[bytes]"):
    url = (
        f"{SERVER}/ws/push?"
        f"camera_id={CAMERA_ID}&eye={eye}"
    )
    headers = [f"Authorization: Bearer {API_TOKEN}"] if API_TOKEN else None
    ws = None
    while not STOP_EVENT.is_set():
        try:
            ws = websocket.create_connection(url, timeout=5, header=headers)
            print(f"[ws:{eye}] connected -> {SERVER} camera_id={CAMERA_ID}")
            while not STOP_EVENT.is_set():
                try:
                    jpg = q.get(timeout=0.5)
                except queue.Empty:
                    continue
                try:
                    ws.send_binary(jpg)
                except Exception as e:
                    print(f"[ws:{eye}] send error:", e)
                    break
        except Exception as e:
            print(f"[ws:{eye}] connect error:", e)
            time.sleep(1.0)
        finally:
            if ws:
                try:
                    ws.close()
                except Exception:
                    pass


def put_latest(q: "queue.Queue[bytes]", data: bytes):
    """Replace a queued stale frame so the stream always favors low latency."""
    try:
        q.put_nowait(data)
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        q.put_nowait(data)


# ====================================================
#          DEPTH -> UGV CORRIDOR ESTIMATION
# ====================================================

def depth_to_colormap(depth: np.ndarray) -> np.ndarray:
    d = depth.copy()
    invalid = ~np.isfinite(d) | (d <= 0)
    d[invalid] = 0
    d = np.clip(d, 0.0, MAX_DEPTH_CLIP)
    d_norm = (d / MAX_DEPTH_CLIP * 255.0).astype(np.uint8)
    color = cv2.applyColorMap(d_norm, cv2.COLORMAP_JET)
    color[invalid] = (0, 0, 0)
    return color


def compute_corridor_min_z(
    depth: np.ndarray,
    fx: float,
    cx: float,
) -> Tuple[float, float, float]:
    """
    Scan a bottom band of the depth image, project each valid pixel to (x_lateral, z),
    and find the minimum z for which |x_lateral| < UGV_HALF_M.

    Returns:
      corridor_min_z: nearest obstacle inside the 0.43m center corridor (NaN if none)
      left_side_min:  nearest obstacle to LEFT of corridor (x < -UGV_HALF_M)
      right_side_min: nearest obstacle to RIGHT of corridor (x > +UGV_HALF_M)
    """
    H, W = depth.shape
    row_start = int(H * (1.0 - BOTTOM_FRACTION))
    row_end   = H - ROW_MARGIN

    corridor_min_z  = float("nan")
    left_side_min   = float("nan")
    right_side_min  = float("nan")

    for y in range(row_start, row_end):
        row = depth[y, :]
        valid = np.isfinite(row) & (row > 0.10) & (row < MAX_DEPTH_CLIP)
        if not np.any(valid):
            continue

        xs = np.where(valid)[0]
        zs = row[valid]

        dxs = xs - cx
        laterals = zs * (dxs.astype(np.float32) / fx)

        for z, x_lat in zip(zs, laterals):
            if abs(x_lat) <= UGV_HALF_M:
                if (not math.isfinite(corridor_min_z)) or (z < corridor_min_z):
                    corridor_min_z = float(z)
            else:
                if x_lat < -UGV_HALF_M:
                    if (not math.isfinite(left_side_min)) or (z < left_side_min):
                        left_side_min = float(z)
                elif x_lat > UGV_HALF_M:
                    if (not math.isfinite(right_side_min)) or (z < right_side_min):
                        right_side_min = float(z)
    return corridor_min_z, left_side_min, right_side_min


def draw_corridor_overlay(
    bgr: np.ndarray,
    depth: np.ndarray,
    fx: float,
    cx: float,
    corr_z_fused: float,
    left_z_fused: float,
    right_z_fused: float,
    corr_z_depth: float,
    corr_z_lidar: float,
) -> np.ndarray:
    """
    Draw corridor band + text overlays on RGB image.
    Shows fused corridor min (ZED+LiDAR) and their individual center values.
    """
    H, W = depth.shape
    row_start = int(H * (1.0 - BOTTOM_FRACTION))
    row_end   = H - ROW_MARGIN

    out = bgr.copy()

    # Draw vertical lines representing +/-UGV_HALF_M at reference depth (1m)
    ref_z = 1.0
    dx_half = UGV_HALF_M * fx / ref_z
    x_left_px  = int(cx - dx_half)
    x_right_px = int(cx + dx_half)

    cv2.rectangle(out,
                  (x_left_px, row_start),
                  (x_right_px, row_end),
                  (0, 255, 0), 2)

    def fmt(v: float) -> str:
        return "NaN" if not math.isfinite(v) else f"{v:.2f}m"

    hud1 = (f"FUSED corridor Z: {fmt(corr_z_fused)} "
            f"(block<{BLOCK_ENTER_DIST:.2f}m, stop<{STOP_DIST:.2f}m)")
    hud2 = f"FUSED L: {fmt(left_z_fused)}   FUSED R: {fmt(right_z_fused)}"
    hud3 = f"ZED center: {fmt(corr_z_depth)}  LiDAR center: {fmt(corr_z_lidar)}"

    cv2.putText(out, hud1, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    cv2.putText(out, hud2, (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
    cv2.putText(out, hud3, (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 0), 1)

    return out


# ====================================================
#                 FUSION HELPERS
# ====================================================

def fuse_dist(a: float, b: float) -> float:
    """
    Fusion rule:
      - if both finite -> return min(a,b)  (more conservative)
      - if one finite -> return that
      - else -> NaN
    """
    a_ok = math.isfinite(a)
    b_ok = math.isfinite(b)
    if a_ok and b_ok:
        return min(a, b)
    elif a_ok:
        return a
    elif b_ok:
        return b
    else:
        return float("nan")


# ====================================================
#                 MAIN LOOP
# ====================================================

def main():
    # Ctrl+C
    def sig_handler(sig, frame):
        STOP_EVENT.set()
    signal.signal(signal.SIGINT, sig_handler)

    # Start WS streaming threads
    print("[WS] Base server:", SERVER)
    print(f"[WS] LEFT  -> {SERVER}/ws/push?camera_id={CAMERA_ID}&eye=left")
    print(f"[WS] DEPTH -> {SERVER}/ws/push?camera_id={CAMERA_ID}&eye=right")
    threading.Thread(target=ws_sender, args=("left", LEFT_Q), daemon=True).start()
    threading.Thread(target=ws_sender, args=("right", RIGHT_Q), daemon=True).start()

    # ROS2
    rclpy.init()
    driver = JackalDriver()
    print("[ROS2] Publishing cmd_vel to:", CMD_TOPIC)
    print(f"[LIDAR] Subscribed to: {LIDAR_TOPIC}")
    print(f"[UGV] width={UGV_WIDTH_M:.3f} m (half={UGV_HALF_M:.3f})")

    # ZED init WITH depth
    zed = sl.Camera()
    init = sl.InitParameters(
        camera_resolution=sl.RESOLUTION.HD720,
        camera_fps=30,
        depth_mode=sl.DEPTH_MODE.PERFORMANCE,   # depth ON
        coordinate_units=sl.UNIT.METER,
    )
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print("[ZED] open failed:", status)
        rclpy.shutdown()
        return

    runtime = sl.RuntimeParameters(enable_depth=True)
    left_mat  = sl.Mat()
    depth_mat = sl.Mat()

    # First grab to get size
    if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
        print("[ZED] first grab failed.")
        zed.close()
        rclpy.shutdown()
        return

    zed.retrieve_image(left_mat, sl.VIEW.LEFT)
    zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
    np_img   = left_mat.get_data()
    np_depth = depth_mat.get_data()

    if np_img is None or np_depth is None:
        print("[ZED] initial image/depth invalid.")
        zed.close()
        rclpy.shutdown()
        return

    H, W = np_depth.shape
    cx = W / 2.0
    fx = W / (2.0 * math.tan(HFOV_RAD / 2.0))

    print(f"[ZED] Opened HD720@30 with depth, size=({W}x{H}), fx~={fx:.1f}, cx~={cx:.1f}")
    print("[NAV] ZED+Velodyne fused corridor navigation with hysteresis engaged.")

    debug_last = 0.0

    # --- state machine for motion ---
    state = "GO"           # "GO", "TURN", "BACKUP"
    turn_dir = 0           # +1 = left, -1 = right
    state_t0 = time.time()

    try:
        while rclpy.ok() and not STOP_EVENT.is_set():
            t0 = time.time()

            # Process ROS callbacks (LiDAR)
            rclpy.spin_once(driver, timeout_sec=0.0)

            # Grab ZED frame
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.005)
                continue

            zed.retrieve_image(left_mat, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

            np_img   = left_mat.get_data()
            np_depth = depth_mat.get_data()

            if np_img is None or np_depth is None:
                continue

            bgr   = np_img[:, :, :3].copy()
            depth = np_depth.astype(np.float32)

            # --- corridor from ZED depth ---
            corr_depth, left_depth, right_depth = compute_corridor_min_z(depth, fx, cx)

            # --- corridor from LiDAR ---
            corr_lid, left_lid, right_lid, lidar_age = driver.get_lidar_mins()
            if lidar_age > LIDAR_STALE_S:
                corr_lid = left_lid = right_lid = float("nan")

            # --- fused distances ---
            corr_fused  = fuse_dist(corr_depth, corr_lid)
            left_fused  = fuse_dist(left_depth, left_lid)
            right_fused = fuse_dist(right_depth, right_lid)

            now = time.time()
            if now - debug_last > 0.7:
                print(
                    f"[DEPTH] corr={corr_depth if math.isfinite(corr_depth) else float('nan'):.2f} "
                    f"L={left_depth if math.isfinite(left_depth) else float('nan'):.2f} "
                    f"R={right_depth if math.isfinite(right_depth) else float('nan'):.2f}"
                )
                print(
                    f"[LIDAR] corr={corr_lid if math.isfinite(corr_lid) else float('nan'):.2f} "
                    f"L={left_lid if math.isfinite(left_lid) else float('nan'):.2f} "
                    f"R={right_lid if math.isfinite(right_lid) else float('nan'):.2f} "
                    f"(age={lidar_age:.2f}s)"
                )
                print(
                    f"[FUSED] corr={corr_fused if math.isfinite(corr_fused) else float('nan'):.2f} "
                    f"L={left_fused if math.isfinite(left_fused) else float('nan'):.2f} "
                    f"R={right_fused if math.isfinite(right_fused) else float('nan'):.2f} "
                    f"state={state}, turn_dir={turn_dir}"
                )
                debug_last = now

            # --- visualization ---
            annotated   = draw_corridor_overlay(
                bgr, depth, fx, cx,
                corr_fused, left_fused, right_fused,
                corr_depth, corr_lid,
            )
            depth_color = depth_to_colormap(depth)

            okL, jpgL = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if okL:
                put_latest(LEFT_Q, jpgL.tobytes())

            okR, jpgR = cv2.imencode(".jpg", depth_color, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if okR:
                put_latest(RIGHT_Q, jpgR.tobytes())

            # =====================================================
            #         FUSED DEPTH/LIDAR-BASED CONTROL (STATEFUL)
            # =====================================================

            v = 0.0
            w = 0.0

            # Helper: choose side ONCE based on fused side distances
            def choose_turn_dir(default: int = +1) -> int:
                nonlocal left_fused, right_fused
                left_ok  = math.isfinite(left_fused)
                right_ok = math.isfinite(right_fused)
                if left_ok and right_ok:
                    return +1 if left_fused >= right_fused else -1
                elif left_ok and not right_ok:
                    return +1
                elif right_ok and not left_ok:
                    return -1
                else:
                    return default

            # ---------------- state machine ----------------
            # Fail safe: loss of both ranging sources must never command motion.
            if not math.isfinite(corr_fused):
                state = "GO"
                v = 0.0
                w = 0.0

            elif state == "GO":
                # Very close obstacle -> turn, or back up only when explicitly enabled.
                if corr_fused <= STOP_DIST:
                    turn_dir = choose_turn_dir(+1)
                    if ALLOW_BACKUP:
                        state = "BACKUP"
                        state_t0 = now
                        print(f"[STATE] GO -> BACKUP (close obstacle, fused={corr_fused:.2f}) turn_dir={turn_dir}")
                    else:
                        state = "TURN"
                        state_t0 = now
                        print(f"[STATE] GO -> TURN (backup disabled, fused={corr_fused:.2f}) turn_dir={turn_dir}")
                        v = 0.0
                        w = turn_dir * TURN_SPEED

                # Blocked zone -> start turning and COMMIT direction
                elif corr_fused <= BLOCK_ENTER_DIST:
                    turn_dir = choose_turn_dir(+1)
                    state = "TURN"
                    state_t0 = now
                    print(f"[STATE] GO -> TURN (blocked, fused={corr_fused:.2f}) turn_dir={turn_dir}")
                    v = 0.0
                    w = turn_dir * TURN_SPEED

                # Clear enough -> keep going forward
                else:
                    v = FWD_SPEED
                    w = 0.0

            elif state == "BACKUP":
                # Back up for BACKUP_TIME then go into TURN using same turn_dir
                if now - state_t0 < BACKUP_TIME:
                    v = BACK_SPEED
                    w = 0.0
                else:
                    state = "TURN"
                    state_t0 = now
                    print(f"[STATE] BACKUP -> TURN, turn_dir={turn_dir}")
                    v = 0.0
                    w = turn_dir * TURN_SPEED

            elif state == "TURN":
                # Keep turning in SAME direction; no re-choosing each frame -> kills jitter
                v = 0.0
                w = turn_dir * TURN_SPEED

                # Only exit turning when fused corridor is clearly open again
                if math.isfinite(corr_fused) and corr_fused > BLOCK_EXIT_DIST:
                    state = "GO"
                    state_t0 = now
                    print(f"[STATE] TURN -> GO (corridor cleared, fused={corr_fused:.2f})")
                    v = FWD_SPEED
                    w = 0.0

            else:
                # Failsafe
                state = "GO"
                v = 0.0
                w = 0.0

            driver.send(v, w)

            dt = time.time() - t0
            if dt < LOOP_DT:
                time.sleep(LOOP_DT - dt)

    finally:
        STOP_EVENT.set()
        driver.stop()
        zed.close()
        driver.destroy_node()
        rclpy.shutdown()
        print("[CLEANUP] ZED+Velodyne fused corridor navigation stopped.")


if __name__ == "__main__":
    main()
