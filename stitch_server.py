"""
aiohttp server for the Multi-Cam Vision prototype (port 5006).

Serves the UI (viewer/stitch.html), the MJPEG streams (the combined canvas plus
one thumbnail per connected camera) and a small JSON WebSocket for the
per-camera placement controls. There is ONE screen and no detection here — the
tool exists only to lay the cameras' frames next to each other; grooves are
detected, and their parameters tuned, in the main app.

Completely separate from server.py — this tool must stay contained from
Developer/Participant Mode. Like the main app, the process exits (SIGINT) when
the last browser tab closes.
"""

from __future__ import annotations

import asyncio
import json
import os
import signal
import threading
from pathlib import Path

from aiohttp import WSMsgType, web

from config import (
    HTTP_HOST, STITCH_CALIB_FILE, STITCH_HTTP_PORT, STITCH_MAX_CAMERAS,
)
from multi_camera import MultiCameraThread, cam_key
from stitcher import StitchCalib

_VIEWER_DIR = Path(__file__).parent / "viewer"


class StitchServer:
    def __init__(self, camera: MultiCameraThread,
                 shared_state: dict, state_lock: threading.Lock) -> None:
        self._camera = camera
        self._state = shared_state
        self._lock = state_lock
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._had_client = False
        self._app = self._build_app()

    def _build_app(self) -> web.Application:
        app = web.Application()

        @web.middleware
        async def no_cache(request, handler):
            resp = await handler(request)
            # The page AND its script: a cached stitch.js against a restarted
            # server is a browser talking a protocol the server no longer speaks.
            if request.path == "/" or request.path.startswith("/static/"):
                resp.headers["Cache-Control"] = "no-store"
            return resp

        app.middlewares.append(no_cache)
        app.router.add_get("/", self._handle_index)
        app.router.add_get("/canvas", lambda r: self._mjpeg(r, "stitch_canvas_jpg"))
        # One thumbnail per camera, indexed by enumeration order — the same
        # index the placement controls edit.
        app.router.add_get("/cam/{index}", self._handle_cam)
        app.router.add_get("/ws", self._handle_ws)
        app.router.add_static("/static", _VIEWER_DIR, show_index=False)
        return app

    async def start(self) -> None:
        runner = web.AppRunner(self._app)
        await runner.setup()
        site = web.TCPSite(runner, HTTP_HOST, STITCH_HTTP_PORT)
        await site.start()
        print(f"Stitch prototype ready -> http://{HTTP_HOST}:{STITCH_HTTP_PORT}")
        await self._broadcast_loop()

    async def _handle_index(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(_VIEWER_DIR / "stitch.html")

    async def _handle_cam(self, request: web.Request) -> web.StreamResponse:
        index = _as_index(request.match_info.get("index"))
        if index is None:
            raise web.HTTPNotFound()
        return await self._mjpeg(request, cam_key(index))

    async def _mjpeg(self, request: web.Request, key: str) -> web.StreamResponse:
        response = web.StreamResponse()
        response.content_type = "multipart/x-mixed-replace; boundary=frame"
        await response.prepare(request)
        try:
            while True:
                with self._lock:
                    jpg = self._state.get(key)
                if jpg:
                    await response.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                                         + jpg + b"\r\n")
                await asyncio.sleep(0.2)   # streams refresh at the canvas cadence
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    # ── WebSocket ────────────────────────────────────────────────────────────
    async def _handle_ws(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        self._ws_clients.add(ws)
        self._had_client = True
        await ws.send_str(json.dumps({
            "type": "init",
            "calib": self._camera.get_calib().to_dict(),
            "colour": self._camera.get_colour(),
            "max_cameras": STITCH_MAX_CAMERAS,
        }))
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                await self._dispatch(ws, data.get("type"), data.get("params") or {})
        finally:
            self._ws_clients.discard(ws)
        return ws

    async def _dispatch(self, ws, mtype: str, params: dict) -> None:
        index = _as_index(params.get("index"))
        if mtype == "set_camera":
            if index is not None:
                self._camera.set_placement(
                    index, {k: v for k, v in params.items() if k != "index"})
        elif mtype == "rotate_camera":
            if index is not None:
                self._camera.rotate_camera(index, _as_steps(params.get("steps"), 1))
        elif mtype == "move_camera":
            if index is not None:
                self._camera.move_camera(index, _as_steps(params.get("steps"), 1))
        elif mtype == "nudge_height":
            if index is not None:
                self._camera.nudge_height(index, _as_steps(params.get("steps"), 1))
        elif mtype == "reset_camera":
            if index is not None:
                self._camera.reset_camera(index, bool(params.get("corners_only")))
        elif mtype == "set_grid":
            try:
                self._camera.set_grid(float(params.get("mm_per_px", 0.0)))
            except (TypeError, ValueError):
                pass
        elif mtype == "set_colour":
            self._camera.set_colour(bool(params.get("on")))
        elif mtype == "save_calib":
            calib = self._camera.get_calib().to_dict()
            STITCH_CALIB_FILE.write_text(json.dumps(calib, indent=2))
            await ws.send_str(json.dumps({
                "type": "save_result", "success": True,
                "message": f"Layout saved to {STITCH_CALIB_FILE}"}))

    # ── state broadcast + last-tab shutdown ──────────────────────────────────
    async def _broadcast_loop(self) -> None:
        empty_since = None
        while True:
            with self._lock:
                info = self._state.get("stitch_info")
                note = self._state.get("stitch_note")
                calib = self._state.get("stitch_calib")
            text = json.dumps({"type": "state", "info": info, "note": note,
                               "calib": calib})
            for ws in list(self._ws_clients):
                try:
                    await ws.send_str(text)
                except (ConnectionResetError, RuntimeError):
                    self._ws_clients.discard(ws)

            if self._had_client and not self._ws_clients:
                empty_since = empty_since or asyncio.get_event_loop().time()
                if asyncio.get_event_loop().time() - empty_since > 2.5:
                    print("Last stitch client disconnected — shutting down.")
                    os.kill(os.getpid(), signal.SIGINT)
            else:
                empty_since = None
            await asyncio.sleep(0.25)


def _as_index(value) -> int | None:
    """Camera index from the browser; anything out of range is ignored."""
    try:
        index = int(value)
    except (TypeError, ValueError):
        return None
    return index if 0 <= index < STITCH_MAX_CAMERAS else None


def _as_steps(value, default: int) -> int:
    """A quarter turn / height nudge count, bounded so one message stays small."""
    try:
        steps = int(value)
    except (TypeError, ValueError):
        return default
    return max(-8, min(8, steps))


def load_saved_calib() -> StitchCalib:
    """Read stitch_calibration.json if present, else an empty rig."""
    try:
        return StitchCalib.from_dict(json.loads(STITCH_CALIB_FILE.read_text()))
    except (OSError, json.JSONDecodeError, ValueError):
        return StitchCalib()
