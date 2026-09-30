# Jackal ZED Live Feed and Depth Navigation

Low-latency ZED camera streaming for a Clearpath Jackal, with an optional ROS 2
controller that combines ZED depth and Velodyne LiDAR for simple corridor-based
obstacle avoidance.

The project supports two robot-side modes:

1. **Live stereo streaming (`pubwebsoc.py`)** — sends the raw ZED left and right
   camera images to a browser through the FastAPI server.
2. **Depth navigation (`mycam9.py`)** — publishes Jackal velocity commands,
   combines ZED depth with Velodyne ranges, and streams an annotated camera view
   plus a depth visualization.
3. **APPRCA object goals (`apprca_robot_bridge.py`)** — receives verified object
   guidance from APPRCA, calculates metric target distance locally, approaches
   at low speed, and reports goal status to the browser.

> [!CAUTION]
> `mycam9.py` publishes real velocity commands. Test with the wheels raised, keep
> an emergency stop within reach, use a large controlled area, and begin with low
> speed limits. It is a research prototype, not a certified collision-avoidance
> or personnel-safety system. Autonomous backup is disabled by default.

## Architecture

```text
STREAM-ONLY MODE
ZED left/right -> JPEG -> WebSocket -> FastAPI -> browser

DEPTH-NAVIGATION MODE
ZED depth -----------+
                     +-> conservative range fusion -> GO/TURN/BACKUP -> cmd_vel
Velodyne PointCloud -+
                     +-> annotated RGB + depth JPEG -> FastAPI -> browser
```

The publisher keeps only the newest frame in each queue. When the network cannot
keep up, stale images are replaced instead of building an increasingly delayed
video backlog.

## Repository layout

```text
LiveFeedJackal/
├── server.py                 FastAPI receiver, fan-out server, and browser UI
├── pubwebsoc.py              ZED left/right stream-only publisher
├── mycam9.py                 ROS 2 depth/LiDAR navigation and visualization
├── requirements-server.txt   Laptop/server Python dependencies
├── requirements-robot.txt    Robot-side pip dependencies
└── requirements.txt          Convenience file containing both groups
```

## Requirements

### Server computer

- Python 3.8 or newer
- A network connection reachable from the Jackal
- A modern browser

### Jackal

- Clearpath Jackal with its ROS 2 drivers running
- ROS 2 Humble or a compatible ROS 2 distribution
- ZED 2i or compatible ZED camera
- Stereolabs ZED SDK and its matching Python API (`pyzed`)
- Velodyne publishing `sensor_msgs/msg/PointCloud2` for depth-navigation mode
- Python packages from `requirements-robot.txt`

`pyzed` must come from the ZED SDK installation. ROS modules such as `rclpy` and
`sensor_msgs_py` should come from the ROS installation; do not replace them with
unrelated PyPI packages.

## 1. Clone the repository

On the server and robot:

```bash
git clone https://github.com/ruturajdixit99/JackalsZedCamera-LiveFeed2LocalServer.git
cd JackalsZedCamera-LiveFeed2LocalServer/LiveFeedJackal
```

## 2. Start the server

On the laptop or server:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements-server.txt

export API_TOKEN="replace-with-a-long-random-token"
export PORT=8050
python3 server.py
```

Alternatively:

```bash
uvicorn server:app --host 0.0.0.0 --port 8050 --ws websockets
```

Verify the server locally:

```bash
curl http://127.0.0.1:8050/health
```

Expected response:

```text
ok
```

Determine the server's LAN address with `ip addr` or `hostname -I`. From the
robot, verify that address before starting a publisher:

```bash
ping <SERVER_IP>
curl http://<SERVER_IP>:8050/health
```

## 3A. Run stream-only mode

Use this mode when you only need the physical left and right ZED images and do
not want the script to control the Jackal.

```bash
python3 -m pip install -r requirements-robot.txt

export SERVER="ws://<SERVER_IP>:8050"
export API_TOKEN="replace-with-the-same-token-as-the-server"
export CAMERA_ID="jackal-zed2i"
export ZED_RES="HD720"
export ZED_FPS=30
export ZED_DEPTH="NONE"
export JPEG_Q=85

python3 pubwebsoc.py
```

## 3B. Run ROS 2 depth-navigation mode

### Install and verify dependencies

Source ROS and the Jackal workspace first:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 -m pip install --user -r requirements-robot.txt
```

Confirm the ZED Python API:

```bash
python3 -c "import pyzed.sl as sl; print('pyzed OK')"
```

Confirm the required ROS topics:

```bash
ros2 topic list | grep -E 'velodyne_points|cmd_vel'
ros2 topic info /velodyne_points
ros2 topic echo /velodyne_points --once
```

The defaults are specific to one Jackal. Override them if your topic names are
different:

```bash
export CMD_TOPIC="/j100_0390/platform/cmd_vel_unstamped"
export LIDAR_TOPIC="/velodyne_points"
```

### Safe first test

1. Raise the wheels or place the Jackal on a secure test stand.
2. Confirm that the hardware emergency stop works.
3. Stop any other node publishing to the same velocity topic.
4. Stop `zed_ros2_wrapper` if it already owns the camera; this script opens the
   ZED directly.
5. Keep autonomous backup disabled until rear clearance is independently proven.

Run the controller:

```bash
export SERVER="ws://<SERVER_IP>:8050"
export API_TOKEN="replace-with-the-same-token-as-the-server"
export CAMERA_ID="jackal-zed2i"
export CMD_TOPIC="/j100_0390/platform/cmd_vel_unstamped"
export LIDAR_TOPIC="/velodyne_points"
export LIDAR_STALE_S=0.5
export ALLOW_BACKUP=0

python3 mycam9.py
```

Press `Ctrl+C` to stop. The shutdown handler publishes zero velocity before
closing ROS and the camera.

## 4. Open the browser viewer

Open:

```text
http://<SERVER_IP>:8050/
```

Enter the camera ID, normally `jackal-zed2i`, and select **Connect**.

## APPRCA object-goal mode (without Nav2)

This mode joins this repository to
[`ActivePerceptionPipelineForARobotCognitiveArchitecture-APPRCA`](https://github.com/PaceRobotLab/ActivePerceptionPipelineForARobotCognitiveArchitecture-APPRCA).

Start the server as described above. On the Jackal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export SERVER="ws://<SERVER_IP>:8050"
export API_TOKEN="<same-token-as-server>"
export CAMERA_ID="jackal-zed2i"
export CMD_TOPIC="/j100_0390/platform/cmd_vel_unstamped"
export LIDAR_TOPIC="/velodyne_points"
export ENABLE_MOTION=0

python3 apprca_robot_bridge.py
```

Start `apprca_live_goal.py` on the GPU/server computer, open the browser, enter
an object name, and select **Start goal**. With `ENABLE_MOTION=0`, the entire
perception, depth, command, timeout, and status path runs without moving the
robot. The browser displays APPRCA's annotated image, the ZED depth map, and the
current goal state.

Only after a successful dry run, raised-wheel test, verified emergency stop, and
confirmation that positive/negative angular commands turn in the expected
directions should motion be enabled:

```bash
export ENABLE_MOTION=1
export STANDOFF_M=1.0
export MAX_FORWARD_MPS=0.18
export SEARCH_RADPS=0.18
python3 apprca_robot_bridge.py
```

The robot stops when control messages become stale, both range sources are
invalid, the user presses **STOP**, or the target reaches the standoff distance.
No reverse motion is used by this integration.

| Mode | LEFT pane | RIGHT pane |
|---|---|---|
| `pubwebsoc.py` | Physical left camera | Physical right camera |
| `mycam9.py` | Annotated left camera | ZED depth colormap |

## How depth navigation works

`mycam9.py` examines the lower 40% of the ZED depth image. For every valid depth
pixel between 0.1 m and 5 m, it estimates lateral position using the pinhole
camera relation:

```text
lateral = depth * (pixel_x - center_x) / focal_length_x
```

Points within half of the configured 0.43 m Jackal width form the forward
corridor. The nearest ZED and fresh LiDAR distances are fused conservatively by
selecting the smaller measurement.

The controller then uses a state machine:

| Condition | Behavior |
|---|---|
| No valid current range | Stop (fail-safe) |
| Corridor above 1.20 m | Drive forward |
| Corridor at or below 0.90 m | Turn toward the clearer side |
| Corridor at or below 0.45 m | Turn, or back up only if explicitly enabled |

The separate enter and exit thresholds provide hysteresis and prevent rapid
switching between forward and turning states.

## Configuration reference

### Shared streaming variables

| Variable | Default | Description |
|---|---:|---|
| `SERVER` | `ws://127.0.0.1:8050` in `mycam9.py` | FastAPI WebSocket base URL |
| `API_TOKEN` | empty | Must match the server token when authentication is enabled |
| `CAMERA_ID` | `jackal-zed2i` | Identifier selected in the browser |
| `PORT` | `8050` | Server listening port |
| `SAVE_HISTORY` | `0` | Save received JPEGs on the server |
| `SAVE_ROOT` | `./uploads` | Server history directory |

### Depth-navigation variables

| Variable | Default | Description |
|---|---:|---|
| `CMD_TOPIC` | `/j100_0390/platform/cmd_vel_unstamped` | Jackal velocity topic |
| `LIDAR_TOPIC` | `/velodyne_points` | Velodyne point-cloud topic |
| `LIDAR_STALE_S` | `0.5` | Maximum LiDAR reading age in seconds |
| `ALLOW_BACKUP` | `0` | Set to `1` only after validating rear safety |

Motion speeds and obstacle thresholds are constants near the top of
`mycam9.py`. Change them cautiously and retest with the wheels raised.

## WebSocket protocol

Robot publishers connect to:

```text
WS /ws/push?camera_id=<id>&eye=left
WS /ws/push?camera_id=<id>&eye=right
Authorization: Bearer <token>
```

Each WebSocket message contains one complete binary JPEG. Browser viewers use:

```text
WS /ws/view?camera_id=<id>&eye=left
WS /ws/view?camera_id=<id>&eye=right
```

## Troubleshooting

### Browser opens but has no images

- Confirm `curl http://<SERVER_IP>:8050/health` works from the robot.
- Confirm `SERVER` uses `ws://`, not `http://`.
- Confirm server and robot use the same `API_TOKEN`.
- Confirm the browser camera ID exactly matches `CAMERA_ID`.
- Check that the firewall allows TCP port 8050.

### `No module named pyzed`

Install the Stereolabs ZED SDK and its Python API for the exact Python version
used to run the script.

### No Velodyne data

- Source the correct ROS workspace.
- Check `ros2 topic info "$LIDAR_TOPIC"`.
- Verify the point cloud frame uses `x` forward and `y` left, or transform it
  before using this controller.
- The subscriber uses ROS sensor-data QoS for compatibility with LiDAR drivers.

### ZED camera cannot open

Close other applications and ROS nodes that already own the ZED, verify USB
connectivity, and test the camera with the ZED SDK diagnostic tools.

### High latency

Lower resolution, frame rate, or JPEG quality. For stream-only mode:

```bash
export ZED_RES=VGA
export ZED_FPS=15
export JPEG_Q=75
```

## Current limitations

- This is a research prototype, not a safety controller.
- The depth controller uses an approximate horizontal field of view rather than
  calibration intrinsics queried from the camera.
- ZED and LiDAR measurements are conservatively combined but not time-synchronized.
- LiDAR processing iterates through the point cloud in Python and may need
  optimization for dense clouds.
- The server stores connection state in memory and should run with one Uvicorn
  worker unless an external broker is added.
- Streaming is designed for a trusted local Wi-Fi network or wired LAN. Use TLS
  (`wss://`) and stronger access control on public or untrusted networks.

## License

No license has been selected yet. Add an appropriate open-source license before
redistributing or accepting outside contributions.
