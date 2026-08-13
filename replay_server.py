"""
aiohttp server for the saved-toolpath replay tool (port 5007).

Serves the replay UI (viewer/replay.html), the saved preview.png images and a
small JSON WebSocket: connect the robot, pick a bundle under paths/, run it.
Completely separate from server.py — this tool stays contained from
Developer/Participant Mode; the only robot-brand-specific code is behind the
replay_robot.ReplayBackend interface. Like the main app, the process exits
(SIGINT) when the last browser tab closes.
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
    DRAW_SPEED, HTTP_HOST, MAX_TCP_SPEED, MOVEP_BLEND_M, PATHS_DIR,
    REPLAY_HTTP_PORT, TRAVEL_Z,
)
from replay_robot import ReplayBackend
from settings import load_settings
from toolpath_loader import Toolpath, list_toolpaths, load_toolpath

_VIEWER_DIR = Path(__file__).parent / "viewer"


def _clamp(params: dict, key: str, default: float, lo: float, hi: float) -> float:
    try:
        return min(max(float(params.get(key, default)), lo), hi)
    except (TypeError, ValueError):
        return default


class ReplayServer:
    def __init__(self, backend: ReplayBackend,
                 shared_state: dict, state_lock: threading.Lock,
                 base_dir: Path | None = None) -> None:
        self._backend = backend
        self._state = shared_state
        self._lock = state_lock
        self._base = Path(base_dir) if base_dir is not None else PATHS_DIR
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._had_client = False
        self._selected: Toolpath | None = None
        self._app = self._build_app()

    def _build_app(self) -> web.Application:
        app = web.Application()

        @web.middleware
        async def no_cache(request, handler):
            resp = await handler(request)
            if request.path == "/":
                resp.headers["Cache-Control"] = "no-store"
            return resp

        app.middlewares.append(no_cache)
        app.router.add_get("/", self._handle_index)
        app.router.add_get("/preview/{name}", self._handle_preview)
        app.router.add_get("/ws", self._handle_ws)
        app.router.add_static("/static", _VIEWER_DIR, show_index=False)
        return app

    async def start(self) -> None:
        runner = web.AppRunner(self._app)
        await runner.setup()
        site = web.TCPSite(runner, HTTP_HOST, REPLAY_HTTP_PORT)
        await site.start()
        print(f"Toolpath replay ready -> http://{HTTP_HOST}:{REPLAY_HTTP_PORT}")
        await self._broadcast_loop()

    async def _handle_index(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(_VIEWER_DIR / "replay.html")

    async def _handle_preview(self, request: web.Request) -> web.StreamResponse:
        folder = self._safe_folder(request.match_info["name"])
        png = folder / "preview.png" if folder else None
        if png is None or not png.is_file():
            raise web.HTTPNotFound()
        return web.FileResponse(png)

    def _safe_folder(self, name: str) -> Path | None:
        """Bundle folder by name; rejects anything that could escape paths/."""
        if not name or any(c in name for c in "/\\") or ".." in name:
            return None
        folder = self._base / name
        return folder if folder.is_dir() else None

    # ── WebSocket ────────────────────────────────────────────────────────────
    async def _handle_ws(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        self._ws_clients.add(ws)
        self._had_client = True
        await ws.send_str(json.dumps({
            "type": "init",
            "backend": self._backend.name,
            "last_ip": load_settings().get("last_ip", ""),
            "toolpaths": list_toolpaths(self._base),
            "toolpath": self._toolpath_msg() if self._selected else None,
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
        if mtype == "connect":
            await self._on_connect(ws, str(params.get("ip", "")).strip())
        elif mtype == "disconnect":
            if self._executing():
                await self._result(ws, "connection_result", False,
                                   "Cancel the run before disconnecting.")
                return
            await asyncio.get_running_loop().run_in_executor(
                None, self._backend.disconnect)
            await self._result(ws, "connection_result", True, "Disconnected.")
        elif mtype == "refresh":
            await ws.send_str(json.dumps({
                "type": "toolpaths", "toolpaths": list_toolpaths(self._base)}))
        elif mtype == "select":
            await self._on_select(ws, str(params.get("name", "")),
                                  params.get("source"))
        elif mtype == "run":
            await self._on_run(ws, params)
        elif mtype == "cancel":
            self._backend.cancel()

    async def _on_connect(self, ws, ip: str) -> None:
        if not ip:
            await self._result(ws, "connection_result", False, "Enter a robot IP.")
            return
        try:
            await asyncio.get_running_loop().run_in_executor(
                None, self._backend.connect, ip)
        except Exception as exc:
            await self._result(ws, "connection_result", False, str(exc))
            return
        await self._result(ws, "connection_result", True, f"Connected to {ip}.")

    async def _on_select(self, ws, name: str, source) -> None:
        folder = self._safe_folder(name)
        if folder is None:
            await self._result(ws, "toolpath", False, f"Unknown toolpath {name!r}.")
            return
        prefer = source if source in ("json", "script") else None
        try:
            self._selected = load_toolpath(folder, prefer=prefer)
        except ValueError as exc:
            await self._result(ws, "toolpath", False, str(exc))
            return
        await self._broadcast(self._toolpath_msg())

    def _toolpath_msg(self) -> dict:
        tp = self._selected
        return {
            "type": "toolpath", "success": True,
            "name": tp.name, "source": tp.source, "meta": tp.meta,
            "stroke_count": tp.stroke_count, "point_count": tp.point_count,
            "has_preview": (tp.folder / "preview.png").is_file(),
        }

    async def _on_run(self, ws, params: dict) -> None:
        if not self._backend.connected:
            await self._result(ws, "run_result", False, "Robot not connected.")
            return
        if self._selected is None:
            await self._result(ws, "run_result", False, "Select a toolpath first.")
            return
        if self._executing():
            await self._result(ws, "run_result", False, "Already executing.")
            return
        meta = self._selected.meta
        speed_pct = _clamp(params, "speed_pct",
                           meta.get("speed_pct", DRAW_SPEED / MAX_TCP_SPEED * 100.0),
                           1.0, 100.0)
        safety_mm = _clamp(params, "safety_mm",
                           meta.get("safety_mm", TRAVEL_Z * 1000.0), 5.0, 300.0)
        blend_mm = _clamp(params, "blend_mm",
                          meta.get("blend_mm", MOVEP_BLEND_M * 1000.0), 0.0, 5.0)
        self._backend.run(self._selected.strokes,
                          speed_mps=speed_pct / 100.0 * MAX_TCP_SPEED,
                          safety_m=safety_mm / 1000.0,
                          blend_m=blend_mm / 1000.0)
        print(f"[replay] running {self._selected.name} ({self._selected.source}): "
              f"{self._selected.stroke_count} strokes, {speed_pct:.0f}% speed, "
              f"safety {safety_mm:.0f} mm, blend {blend_mm:.1f} mm")
        await self._result(
            ws, "run_result", True,
            f"Running {self._selected.name} ({self._selected.source}).")

    def _executing(self) -> bool:
        with self._lock:
            return bool(self._state.get("executing"))

    async def _result(self, ws, mtype: str, success: bool, message: str) -> None:
        await ws.send_str(json.dumps(
            {"type": mtype, "success": success, "message": message}))

    # ── state broadcast + last-tab shutdown ──────────────────────────────────
    async def _broadcast(self, payload: dict) -> None:
        text = json.dumps(payload)
        for ws in list(self._ws_clients):
            try:
                await ws.send_str(text)
            except (ConnectionResetError, RuntimeError):
                self._ws_clients.discard(ws)

    async def _broadcast_loop(self) -> None:
        empty_since = None
        while True:
            with self._lock:
                phase = self._state.get("phase", "idle")
                progress = self._state.get("progress", 0.0)
                error = self._state.get("exec_error")
                executing = bool(self._state.get("executing"))
            await self._broadcast({
                "type": "state",
                "connected": self._backend.connected,
                "executing": executing,
                # PathExecutor reports a cancelled run as phase "captured".
                "phase": "cancelled" if phase == "captured" else phase,
                "progress": progress,
                "exec_error": error,
                "selected": self._selected.name if self._selected else None,
            })
            if self._had_client and not self._ws_clients:
                empty_since = empty_since or asyncio.get_event_loop().time()
                if asyncio.get_event_loop().time() - empty_since > 2.5:
                    print("Last replay client disconnected — shutting down.")
                    os.kill(os.getpid(), signal.SIGINT)
            else:
                empty_since = None
            await asyncio.sleep(0.25)
