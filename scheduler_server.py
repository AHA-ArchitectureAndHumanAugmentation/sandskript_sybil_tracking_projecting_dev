"""
aiohttp server for the Scheduler tool (port 5008).

Serves the spreadsheet UI (viewer/scheduler.html), each bundle's mask.png, a CSV
of the same columns, and a small JSON WebSocket that pushes a fresh scan
whenever paths/ changes — so a run finishing in the main app appears here
without a reload.

READ-ONLY: this tool opens no camera, no robot, and writes nothing to disk. It
is therefore the one contained tool that can safely run alongside the main app.
Completely separate from server.py; never import main here. Like the other
tools, the process exits (SIGINT) when the last browser tab closes.
"""
from __future__ import annotations

import asyncio
import json
import os
import signal
from pathlib import Path

from aiohttp import WSMsgType, web

from config import (
    HTTP_HOST, PATHS_DIR, SCHEDULER_HTTP_PORT, SCHEDULER_REFRESH_S,
)
from scheduler import MASK_FILE, read_schedule, to_csv

_VIEWER_DIR = Path(__file__).parent / "viewer"


class SchedulerServer:
    def __init__(self, base_dir: Path | None = None) -> None:
        self._base = Path(base_dir) if base_dir is not None else PATHS_DIR
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._had_client = False
        self._signature: tuple | None = None
        self._app = self._build_app()

    def _build_app(self) -> web.Application:
        app = web.Application()

        @web.middleware
        async def no_cache(request, handler):
            resp = await handler(request)
            # The page AND its script: a cached scheduler.js against a restarted
            # server is a browser talking a protocol the server no longer speaks.
            if request.path == "/" or request.path.startswith("/static/"):
                resp.headers["Cache-Control"] = "no-store"
            return resp

        app.middlewares.append(no_cache)
        app.router.add_get("/", self._handle_index)
        app.router.add_get("/schedule.csv", self._handle_csv)
        app.router.add_get("/mask/{name}", self._handle_mask)
        app.router.add_get("/ws", self._handle_ws)
        app.router.add_static("/static", _VIEWER_DIR, show_index=False)
        return app

    async def start(self) -> None:
        runner = web.AppRunner(self._app)
        await runner.setup()
        site = web.TCPSite(runner, HTTP_HOST, SCHEDULER_HTTP_PORT)
        await site.start()
        print(f"Scheduler ready -> http://{HTTP_HOST}:{SCHEDULER_HTTP_PORT}")
        await self._watch_loop()

    async def _handle_index(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(_VIEWER_DIR / "scheduler.html")

    async def _handle_csv(self, request: web.Request) -> web.Response:
        rows = await self._scan()
        return web.Response(
            text=to_csv(rows), content_type="text/csv", charset="utf-8",
            headers={"Content-Disposition":
                     'attachment; filename="schedule.csv"'})

    async def _handle_mask(self, request: web.Request) -> web.FileResponse:
        """The bundle's mask.png. Deliberately cacheable: it never changes."""
        folder = self._safe_folder(request.match_info["name"])
        png = folder / MASK_FILE if folder else None
        if png is None or not png.is_file():
            raise web.HTTPNotFound()
        return web.FileResponse(png)

    def _safe_folder(self, name: str) -> Path | None:
        """Bundle folder by name; rejects anything that could escape paths/."""
        if not name or any(c in name for c in "/\\") or ".." in name:
            return None
        folder = self._base / name
        return folder if folder.is_dir() else None

    async def _scan(self):
        """Off the event loop: a big paths/ is a lot of stat() calls."""
        return await asyncio.get_running_loop().run_in_executor(
            None, read_schedule, self._base)

    async def _payload(self) -> dict:
        rows = await self._scan()
        return {"rows": [r.to_dict() for r in rows],
                "base": str(self._base.resolve()),
                "count": len(rows)}

    # ── WebSocket ────────────────────────────────────────────────────────────
    async def _handle_ws(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        self._ws_clients.add(ws)
        self._had_client = True
        await ws.send_str(json.dumps({"type": "init", **(await self._payload())}))
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                if data.get("type") == "refresh":
                    self._signature = None      # force the next tick to send
        finally:
            self._ws_clients.discard(ws)
        return ws

    # ── change watch + last-tab shutdown ─────────────────────────────────────
    async def _watch_loop(self) -> None:
        empty_since = None
        while True:
            payload = await self._payload()
            # Only push when something actually changed: this loop runs forever
            # and the table would otherwise be rebuilt every couple of seconds.
            signature = tuple((r["name"], r["executed_at"], tuple(r["files"]))
                              for r in payload["rows"])
            if signature != self._signature:
                self._signature = signature
                text = json.dumps({"type": "schedule", **payload})
                for ws in list(self._ws_clients):
                    try:
                        await ws.send_str(text)
                    except (ConnectionResetError, RuntimeError):
                        self._ws_clients.discard(ws)

            if self._had_client and not self._ws_clients:
                empty_since = empty_since or asyncio.get_event_loop().time()
                if asyncio.get_event_loop().time() - empty_since > 2.5:
                    print("Last scheduler client disconnected — shutting down.")
                    os.kill(os.getpid(), signal.SIGINT)
            else:
                empty_since = None
            await asyncio.sleep(SCHEDULER_REFRESH_S)
