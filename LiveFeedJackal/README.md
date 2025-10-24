# 🧠 ZED Camera WebSocket Live Feed  
**Robot → Server → Browser (Low-Latency Streaming)**

This project enables real-time **LEFT + RIGHT camera feed streaming** from a **ZED 2i** (or similar) camera on a robot (e.g., Clearpath Jackal) to a remote **server** viewable in any browser — using **FastAPI WebSockets** for ultra-low latency.

---

## 📦 Overview

| Component | Role | Tech Stack |
|------------|------|-------------|
| 🤖 **Robot (ZED 2i)** | Captures and streams frames | `pyzed.sl`, `OpenCV`, `websocket-client` |
| 💻 **Server (Laptop/PC)** | Receives and distributes frames | `FastAPI`, `Uvicorn`, `WebSockets` |
| 🌐 **Browser** | Displays synchronized LEFT + RIGHT live feed | HTML + JS WebSocket API |

---

## 🖥️ SERVER SETUP

### 1️⃣ Install Dependencies
```bash
python3 -m pip install --upgrade "uvicorn[standard]" fastapi

2️⃣ Set Environment Variables
export API_TOKEN="supersecret123"    # Optional but recommended
export PORT=8050                     # Default port for the WebSocket feed****

3️⃣ Run the Server
Option A – Direct Python
python3 server.py

Option B – Uvicorn CLI (recommended)
uvicorn server:app --host 0.0.0.0 --port 8050 --ws websockets


4️⃣ Access the Feed

Open in your browser:

http://<SERVER_IP>:8050/


Example:

http://192.168.0.118:8050/


You’ll see a simple UI with fields for Camera ID — enter (e.g.) jackal-zed2i and click Connect to view live LEFT and RIGHT feeds.


🤖 ROBOT SETUP (ZED Camera Side)
1️⃣ Install Dependencies
python3 -m pip install websocket-client opencv-python-headless pyzed


(If ZED SDK is already installed, pyzed should already exist.)

2️⃣ Environment Configuration

Replace <SERVER_IP> with your server/laptop’s IP on the same Wi-Fi or LAN network.

export SAVE_DIR="/home/administrator/ros2_ws/images"
export API_TOKEN="supersecret123"
export CAMERA_ID="jackal-zed2i"
export SERVER="ws://192.168.0.118:8050"   # <-- your server IP (use ws://, not http://)


Optional performance tuning:

export ZED_RES="HD720"          # Options: HD2K, HD1080, HD720, VGA
export ZED_FPS=30
export ZED_DEPTH="ULTRA"        # Avoid NEURAL on GTX 1050 Ti
export JPEG_Q=90

3️⃣ Run the Streamer

From the robot terminal:

python3 pubwebsoc.py


This script:

Connects to the ZED camera

Streams LEFT and RIGHT views over WebSocket

Keeps a single frame buffer for minimal lag

Auto-reconnects if the server restarts

🌐 NETWORK CHECKS
On Server:
ifconfig | grep inet


Find your active interface (e.g. 192.168.0.118).

On Robot:

Check connection to the server:

ping 192.168.0.118
curl http://192.168.0.118:8050/health


Expected:

ok

🧰 TROUBLESHOOTING
Issue	Likely Cause	Solution
Unsupported upgrade request	Missing WS backend	Run pip install "uvicorn[standard]"
Browser shows “No feed”	Wrong protocol (http:// instead of ws://)	Use SERVER="ws://<IP>:8050"
Lag or delay	Using HTTP upload instead of WebSocket	Run pubwebsoc.py, not pubzed.py
Feed drops randomly	Network reconnect or timeout	The client auto-reconnects within 1 second
📁 Example Directory Structure
├── server.py         # WebSocket FastAPI server (runs on laptop)
├── pubwebsoc.py      # ZED camera WebSocket publisher (runs on robot)
└── README.md         # This file

🧩 How It Works
[ZED Camera] → [pubwebsoc.py] → WebSocket → [server.py] → WebSocket → [Browser]


Frames are JPEG-encoded on the robot.

Each eye (left, right) is streamed separately.

The browser renders both streams side-by-side in near real-time (<200 ms latency).

✅ Summary
Role	Machine	Script	Protocol
Camera Capture & Upload	Robot (ZED 2i)	pubwebsoc.py	WebSocket
Frame Receiver + Viewer	Laptop / Server	server.py	WebSocket
Live Stream Display	Browser	Auto via FastAPI	HTML + JS
🏁 Commands Summary
Server
export API_TOKEN="supersecret123"
export PORT=8050
python3 server.py
# OR
uvicorn server:app --host 0.0.0.0 --port 8050 --ws websockets

Robot
export SAVE_DIR="/home/administrator/ros2_ws/images"
export API_TOKEN="supersecret123"
export CAMERA_ID="jackal-zed2i"
export SERVER="ws://192.168.0.118:8050"
python3 pubwebsoc.py






