#!/usr/bin/env python3
import os, html, asyncio, time
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

LATEST_LOCK = asyncio.Lock()

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
    if eye not in ("left", "right"):
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
    if eye not in ("left", "right"):
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

# ================= Simple live page (LEFT & RIGHT) =================
@app.get("/", response_class=HTMLResponse)
def index():
    return (
        "<!doctype html><html><head><meta charset='utf-8' />"
        "<title>ZED WS Live</title>"
        "<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif}"
        ".row{display:flex;gap:8px;padding:8px}"
        ".pane{flex:1;background:#000;text-align:center}"
        "img{max-width:100%;height:auto;display:block;margin:0 auto}"
        "label,input{font-size:14px;margin:4px}button{margin-left:8px}</style>"
        "</head><body>"
        "<div style='padding:8px'>"
        "<label>Camera ID:</label><input id='cam' value='jackal-zed2i'/>"
        "<button onclick='connect()'>Connect</button>"
        "</div>"
        "<div class='row'>"
        "<div class='pane'><div>LEFT</div><img id='left' /></div>"
        "<div class='pane'><div>RIGHT</div><img id='right' /></div>"
        "</div>"
        "<script>"
        "let wsL, wsR, urlBase = (location.protocol==='https:'?'wss://':'ws://')+location.host;"
        "let lastURLL=null, lastURLR=null;"
        "function connect(){"
        " const cam=document.getElementById('cam').value;"
        " if(wsL){wsL.close()} if(wsR){wsR.close()}"
        " wsL=new WebSocket(urlBase+'/ws/view?camera_id='+encodeURIComponent(cam)+'&eye=left');"
        " wsR=new WebSocket(urlBase+'/ws/view?camera_id='+encodeURIComponent(cam)+'&eye=right');"
        " wsL.binaryType='blob'; wsR.binaryType='blob';"
        " wsL.onmessage=(ev)=>{const url=URL.createObjectURL(ev.data);"
        "  document.getElementById('left').src=url; if(lastURLL) URL.revokeObjectURL(lastURLL); lastURLL=url;};"
        " wsR.onmessage=(ev)=>{const url=URL.createObjectURL(ev.data);"
        "  document.getElementById('right').src=url; if(lastURLR) URL.revokeObjectURL(lastURLR); lastURLR=url;};"
        "}"
        "</script>"
        "</body></html>"
    )

if __name__ == "__main__":
    import uvicorn
    if not API_TOKEN:
        print("[server] Warning: API_TOKEN not set; robot uploads unauthenticated (OK for LAN tests).")
    if SAVE_HISTORY:
        print(f"[server] History saving ENABLED at {SAVE_ROOT}")
    else:
        print("[server] History saving DISABLED (live-only).")
    uvicorn.run("server:app", host=HOST, port=PORT, reload=False)
