#!/usr/bin/env python3
import os, html, asyncio, time, json
from typing import Dict, Tuple, Set, Optional
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query, HTTPException
from fastapi.responses import HTMLResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles

# ================= Config =================
API_TOKEN = os.environ.get("API_TOKEN")           # if set, require Bearer on /ws/push
HOST      = os.environ.get("HOST", "0.0.0.0")
PORT      = int(os.environ.get("PORT", "8050"))

# Optional: also save incoming frames (debug/history). Default OFF.
SAVE_HISTORY = os.environ.get("SAVE_HISTORY", "0") not in ("0","false","False","no","NO")
SAVE_ROOT    = Path(os.environ.get("SAVE_ROOT", "./uploads"))
if SAVE_HISTORY:
    SAVE_ROOT.mkdir(parents=True, exist_ok=True)

# ================= App =================
app = FastAPI()
if SAVE_HISTORY:
    app.mount("/uploads", StaticFiles(directory=str(SAVE_ROOT)), name="uploads")

# latest frame per (camera_id, eye): (bytes, ts)
LATEST: Dict[Tuple[str, str], Tuple[bytes, float]] = {}
# viewers per (camera_id, eye):
VIEWERS: Dict[Tuple[str, str], Set[WebSocket]] = {}

# APPRCA integration channels, keyed by camera_id.
GOAL_CLIENTS: Dict[str, Set[WebSocket]] = {}
CONTROL_CLIENTS: Dict[str, Set[WebSocket]] = {}
STATUS_CLIENTS: Dict[str, Set[WebSocket]] = {}
LATEST_GOAL: Dict[str, str] = {}
LATEST_STATUS: Dict[str, str] = {}

LATEST_LOCK = asyncio.Lock()


async def _broadcast_text(clients: Dict[str, Set[WebSocket]], key: str, payload: str):
    """Best-effort fan-out without holding the shared lock during network I/O."""
    async with LATEST_LOCK:
        targets = list(clients.get(key, set()))
    dead = set()
    for ws in targets:
        try:
            await ws.send_text(payload)
        except Exception:
            dead.add(ws)
    if dead:
        async with LATEST_LOCK:
            clients.setdefault(key, set()).difference_update(dead)

# ================= Utilities =================
def _check_token(header_val: Optional[str]):
    if not API_TOKEN:
        return
    if not header_val or not header_val.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Missing Bearer token")
    if header_val.split(" ", 1)[1] != API_TOKEN:
        raise HTTPException(status_code=403, detail="Bad token")

# ================= Health =================
@app.get("/health", response_class=PlainTextResponse)
def health():
    return "ok"

# ================= WebSocket: robot -> server (push) =================
@app.websocket("/ws/push")
async def ws_push(
    websocket: WebSocket,
    camera_id: str = Query("zed"),
    eye: str = Query("left"),
    authorization: Optional[str] = Query(default=None),
):
    # For Starlette/WS, headers are accessible via scope
    if API_TOKEN:
        # try query param first, else header
        token_val = authorization or websocket.headers.get("authorization")
        _check_token(token_val)

    eye = eye.lower().strip()
    if eye not in ("left", "right", "apprca"):
        await websocket.close(code=4000)
        return

    key = (camera_id, eye)
    await websocket.accept()

    # Ensure viewers set exists
    async with LATEST_LOCK:
        VIEWERS.setdefault(key, set())

    try:
        while True:
            # Robot sends binary JPEG frames
            data = await websocket.receive_bytes()
            now_ts = time.time()

            # Optionally save to disk (debug)
            if SAVE_HISTORY:
                camdir = SAVE_ROOT / camera_id / eye
                camdir.mkdir(parents=True, exist_ok=True)
                # quick rolling filename
                fname = f"{now_ts:.6f}.jpg"
                (camdir / fname).write_bytes(data)

            # update latest and fan-out to viewers
            async with LATEST_LOCK:
                LATEST[key] = (data, now_ts)
                # broadcast (best-effort)
                dead: Set[WebSocket] = set()
                for ws in VIEWERS.get(key, set()):
                    try:
                        await ws.send_bytes(data)
                    except Exception:
                        dead.add(ws)
                for ws in dead:
                    VIEWERS[key].discard(ws)

    except WebSocketDisconnect:
        # robot disconnected — just exit
        return
    except Exception:
        # any other error — close
        try:
            await websocket.close()
        except Exception:
            pass

# ================= WebSocket: browser viewer =================
@app.websocket("/ws/view")
async def ws_view(
    websocket: WebSocket,
    camera_id: str = Query(...),
    eye: str = Query("left"),
):
    eye = eye.lower().strip()
    if eye not in ("left", "right", "apprca"):
        await websocket.close(code=4000)
        return

    key = (camera_id, eye)
    await websocket.accept()

    # register viewer
    async with LATEST_LOCK:
        VIEWERS.setdefault(key, set()).add(websocket)
        # send latest immediately if present
        if key in LATEST:
            await websocket.send_bytes(LATEST[key][0])

    try:
        # keep the connection open; we push frames from ws_push handler
        while True:
            await asyncio.sleep(60)  # keep task alive
    except WebSocketDisconnect:
        pass
    finally:
        async with LATEST_LOCK:
            VIEWERS.get(key, set()).discard(websocket)


# ================= APPRCA goal/control/status =================
@app.post("/api/goal")
async def set_goal(camera_id: str = Query("jackal-zed2i"), target: str = Query(...)):
    target = " ".join(target.strip().split())
    if not target or len(target) > 100:
        raise HTTPException(status_code=400, detail="Target must be 1-100 characters")
    payload = json.dumps({"type": "goal", "camera_id": camera_id,
                          "target": target, "timestamp": time.time()})
    LATEST_GOAL[camera_id] = payload
    await _broadcast_text(GOAL_CLIENTS, camera_id, payload)
    return {"ok": True, "camera_id": camera_id, "target": target}


@app.post("/api/stop")
async def stop_goal(camera_id: str = Query("jackal-zed2i")):
    payload = json.dumps({"type": "stop", "camera_id": camera_id,
                          "timestamp": time.time()})
    LATEST_GOAL[camera_id] = payload
    await _broadcast_text(GOAL_CLIENTS, camera_id, payload)
    await _broadcast_text(CONTROL_CLIENTS, camera_id, payload)
    return {"ok": True, "camera_id": camera_id}


async def _receiver(websocket: WebSocket, clients, camera_id: str, initial: Optional[str] = None):
    await websocket.accept()
    async with LATEST_LOCK:
        clients.setdefault(camera_id, set()).add(websocket)
    if initial:
        await websocket.send_text(initial)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        async with LATEST_LOCK:
            clients.setdefault(camera_id, set()).discard(websocket)


@app.websocket("/ws/goal")
async def ws_goal(websocket: WebSocket, camera_id: str = Query("jackal-zed2i")):
    await _receiver(websocket, GOAL_CLIENTS, camera_id, LATEST_GOAL.get(camera_id))


@app.websocket("/ws/control")
async def ws_control(websocket: WebSocket, camera_id: str = Query("jackal-zed2i")):
    await _receiver(websocket, CONTROL_CLIENTS, camera_id)


@app.websocket("/ws/status")
async def ws_status(websocket: WebSocket, camera_id: str = Query("jackal-zed2i")):
    await _receiver(websocket, STATUS_CLIENTS, camera_id, LATEST_STATUS.get(camera_id))


@app.websocket("/ws/control/push")
async def ws_control_push(websocket: WebSocket, camera_id: str = Query("jackal-zed2i"),
                          authorization: Optional[str] = Query(default=None)):
    if API_TOKEN:
        _check_token(authorization or websocket.headers.get("authorization"))
    await websocket.accept()
    try:
        while True:
            payload = await websocket.receive_text()
            await _broadcast_text(CONTROL_CLIENTS, camera_id, payload)
    except WebSocketDisconnect:
        pass


@app.websocket("/ws/status/push")
async def ws_status_push(websocket: WebSocket, camera_id: str = Query("jackal-zed2i"),
                         authorization: Optional[str] = Query(default=None)):
    if API_TOKEN:
        _check_token(authorization or websocket.headers.get("authorization"))
    await websocket.accept()
    try:
        while True:
            payload = await websocket.receive_text()
            LATEST_STATUS[camera_id] = payload
            await _broadcast_text(STATUS_CLIENTS, camera_id, payload)
    except WebSocketDisconnect:
        pass

# ================= Integrated APPRCA live page =================
@app.get("/", response_class=HTMLResponse)
def index():
    return """<!doctype html><html><head><meta charset='utf-8'/>
<title>APPRCA Jackal Goal Console</title><style>
body{margin:0;background:#101319;color:#eef2f7;font:15px system-ui,sans-serif}
header{padding:14px 18px;background:#171c25;display:flex;gap:10px;align-items:center;flex-wrap:wrap}
input,button{font:inherit;padding:8px;border-radius:6px;border:1px solid #465063;background:#0d1117;color:#fff}
button{cursor:pointer;background:#1769aa}.stop{background:#a52b2b}.status{padding:10px 18px;color:#8ee3a2}
.row{display:flex;gap:10px;padding:10px}.pane{flex:1;background:#050607;text-align:center;padding:6px}
img{width:100%;height:auto;display:block}.label{padding:6px;color:#aeb8c7}
</style></head><body>
<header><strong>APPRCA + Jackal</strong><label>Camera</label><input id='cam' value='jackal-zed2i'/>
<label>Object goal</label><input id='target' placeholder='chair'/>
<button onclick='connect()'>Connect</button><button onclick='startGoal()'>Start goal</button>
<button class='stop' onclick='stopGoal()'>STOP</button></header>
<div id='status' class='status'>Disconnected</div><div class='row'>
<div class='pane'><div class='label'>APPRCA detection</div><img id='left'/></div>
<div class='pane'><div class='label'>ZED depth</div><img id='right'/></div></div>
<script>
let sockets=[],urls=[null,null],base=(location.protocol==='https:'?'wss://':'ws://')+location.host;
const enc=encodeURIComponent;
function stream(eye,img,n){let c=enc(cam.value),w=new WebSocket(base+'/ws/view?camera_id='+c+'&eye='+eye);
w.binaryType='blob';w.onmessage=e=>{let u=URL.createObjectURL(e.data);img.src=u;if(urls[n])URL.revokeObjectURL(urls[n]);urls[n]=u};sockets.push(w)}
function connect(){sockets.forEach(x=>x.close());sockets=[];stream('apprca',left,0);stream('right',right,1);
let w=new WebSocket(base+'/ws/status?camera_id='+enc(cam.value));w.onmessage=e=>{try{let s=JSON.parse(e.data);status.textContent=`${s.state||'UNKNOWN'} | goal: ${s.target||'-'} | distance: ${s.distance_m??'-'} m | ${s.message||''}`}catch{status.textContent=e.data}};sockets.push(w);status.textContent='Connected; waiting for robot status'}
async function startGoal(){let t=target.value.trim();if(!t)return;await fetch('/api/goal?camera_id='+enc(cam.value)+'&target='+enc(t),{method:'POST'});status.textContent='Goal submitted: '+t}
async function stopGoal(){await fetch('/api/stop?camera_id='+enc(cam.value),{method:'POST'});status.textContent='STOP requested'}
</script></body></html>"""

if __name__ == "__main__":
    import uvicorn
    if not API_TOKEN:
        print("[server] Warning: API_TOKEN not set; robot uploads unauthenticated (OK for LAN tests).")
    if SAVE_HISTORY:
        print(f"[server] History saving ENABLED at {SAVE_ROOT}")
    else:
        print("[server] History saving DISABLED (live-only).")
    uvicorn.run("server:app", host=HOST, port=PORT, reload=False)
