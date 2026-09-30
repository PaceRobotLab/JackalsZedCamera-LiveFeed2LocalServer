# Jackal ZED Camera: Live Streaming and Depth Navigation

A low-latency camera and navigation toolkit for a Clearpath Jackal equipped with
a Stereolabs ZED camera and, optionally, a Velodyne LiDAR.

It streams JPEG frames from the robot to a FastAPI WebSocket server, where any
modern browser can display the live feed. An advanced ROS 2 mode also combines
ZED depth and Velodyne point-cloud ranges to perform simple corridor-based
obstacle avoidance while publishing velocity commands to the Jackal.

## Two operating modes

| Mode | Robot program | Output |
|---|---|---|
| Live stereo | `pubwebsoc.py` | Physical left and right ZED views |
| ROS 2 depth navigation | `mycam9.py` | Annotated RGB, depth map, and Jackal control |

```text
Robot                                           Server / browser

ZED left/right ──┐
                 ├── JPEG over WebSocket ──────> FastAPI ──> live page
Velodyne + depth ┘          (latest frame)         │
       │                                           └── optional history
       └── obstacle state machine ──> Jackal cmd_vel
```

## Quick start

```bash
git clone https://github.com/ruturajdixit99/JackalsZedCamera-LiveFeed2LocalServer.git
cd JackalsZedCamera-LiveFeed2LocalServer/LiveFeedJackal
```

On the server:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements-server.txt
export API_TOKEN="replace-with-a-long-random-token"
python3 server.py
```

On the robot for stream-only mode:

```bash
python3 -m pip install --user -r requirements-robot.txt
export SERVER="ws://<SERVER_IP>:8050"
export API_TOKEN="replace-with-the-same-token-as-the-server"
python3 pubwebsoc.py
```

Then open `http://<SERVER_IP>:8050/` and connect to camera ID
`jackal-zed2i`.

> [!CAUTION]
> `mycam9.py` sends physical motion commands. It is a research prototype, not a
> certified safety system. Read and complete the safety checklist before running
> it. Test first with raised wheels and a working emergency stop.

## Full documentation

See the **[complete installation, configuration, ROS 2, safety, protocol, and
troubleshooting guide](LiveFeedJackal/README.md)**.

The detailed guide covers:

- Server and robot installation
- ZED SDK and ROS 2 dependencies
- Jackal and Velodyne topic configuration
- Safe depth-navigation startup
- Environment-variable reference
- Depth-processing and sensor-fusion behavior
- WebSocket protocol
- Common failures and fixes

## Project status

The streaming path is designed to work over a trusted local Wi-Fi network or a
wired LAN. Internet-facing or untrusted-network deployments should add TLS
(`wss://`) and stronger access control. The navigation controller is experimental
and requires validation for each robot's sensor frames, mounting, topic names,
dynamics, and operating environment.

No open-source license has been selected yet.
