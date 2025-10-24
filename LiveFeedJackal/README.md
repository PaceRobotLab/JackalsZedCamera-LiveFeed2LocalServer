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
