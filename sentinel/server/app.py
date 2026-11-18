"""FastAPI server providing web dashboard, MJPEG live stream, and real-time WebSocket telemetry."""

import os
import asyncio
import logging
from typing import Set
import cv2
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import StreamingResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from sentinel.main import SentinelSystem

logger = logging.getLogger("sentinel.server")

app = FastAPI(title="SENTINEL Mission Control API", version="0.1.0")

# Global Sentinel instance
sentinel = SentinelSystem()

# Dashboard assets path
DASHBOARD_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../dashboard"))
if os.path.exists(DASHBOARD_DIR):
    app.mount("/static", StaticFiles(directory=DASHBOARD_DIR), name="static")

# WebSocket client registry
active_connections: Set[WebSocket] = set()


@app.on_event("startup")
def startup_event():
    logger.info("Starting SENTINEL Observation Subsystem...")
    sentinel.start()
    asyncio.create_task(telemetry_broadcaster())


@app.on_event("shutdown")
def shutdown_event():
    logger.info("Shutting down SENTINEL Subsystems...")
    sentinel.stop()


@app.get("/")
async def get_dashboard():
    index_path = os.path.join(DASHBOARD_DIR, "index.html")
    if os.path.exists(index_path):
        return FileResponse(index_path)
    return {"message": "SENTINEL Mission Control API active. Dashboard assets loading..."}


def generate_mjpeg_stream():
    """Generator yielding boundary-separated JPEG frames for live camera streaming."""
    while sentinel.is_running:
        frame = sentinel.get_annotated_frame()
        # Encode as JPEG
        ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret:
            continue
        frame_bytes = buffer.tobytes()
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )


@app.get("/video_feed")
def video_feed():
    """MJPEG live annotated camera feed endpoint."""
    return StreamingResponse(
        generate_mjpeg_stream(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


async def telemetry_broadcaster():
    """Background task pushing live telemetry to all connected WebSocket clients at 10 Hz."""
    while True:
        if active_connections:
            snapshot = sentinel.get_telemetry_snapshot()
            disconnected = []
            for ws in active_connections:
                try:
                    await ws.send_json(snapshot)
                except Exception:
                    disconnected.append(ws)
            for ws in disconnected:
                active_connections.remove(ws)
        await asyncio.sleep(0.1)


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """Real-time bi-directional telemetry and command WebSocket."""
    await websocket.accept()
    active_connections.add(websocket)
    try:
        while True:
            data = await websocket.receive_json()
            cmd_type = data.get("type")

            if cmd_type == "SET_MODE":
                sentinel.set_mode(data.get("mode", "SIMULATION"))
            elif cmd_type == "SET_TARGET":
                x = float(data.get("x_cm", 100.0))
                y = float(data.get("y_cm", 75.0))
                sentinel.set_simulated_target(x, y)
            elif cmd_type == "EMERGENCY_STOP":
                sentinel.trigger_emergency_stop()
            elif cmd_type == "RESET_EMERGENCY":
                sentinel.reset_emergency()
            elif cmd_type == "RESET_ROVER":
                rx = float(data.get("x_cm", 30.0))
                ry = float(data.get("y_cm", 30.0))
                sentinel.virtual_rover.reset_pose(rx, ry, heading_deg=0.0)
                sentinel.log_event("ROVER", f"Rover repositioned to ({rx:.1f}, {ry:.1f})cm")
    except WebSocketDisconnect:
        active_connections.discard(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        active_connections.discard(websocket)


# REST Endpoints
class TargetRequest(BaseModel):
    x_cm: float
    y_cm: float


@app.post("/api/target")
def set_target(req: TargetRequest):
    success = sentinel.plan_mission_to_target(req.x_cm, req.y_cm)
    return {"status": "OK" if success else "FAILED", "target": [req.x_cm, req.y_cm]}


@app.post("/api/emergency_stop")
def emergency_stop():
    sentinel.trigger_emergency_stop()
    return {"status": "EMERGENCY_STOP_TRIGGERED"}


@app.post("/api/reset_emergency")
def reset_emergency():
    sentinel.reset_emergency()
    return {"status": "EMERGENCY_CLEARED"}
