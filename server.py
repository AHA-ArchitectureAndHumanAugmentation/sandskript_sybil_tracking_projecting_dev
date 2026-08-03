import asyncio
import base64
import json
import threading
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

from aiohttp import web

from config import (
    HTTP_HOST, HTTP_PORT, VIS_INTERVAL,
    DEPTH_PATH, RGB_PATH, GROOVE_PATH, MASK_PATH, WS_PATH, STATIC_PATH,
    SURFACE_UPLOAD_URL, PRESETS_DIR,
)
from settings import load_settings, save_settings

_VIEWER_DIR = Path(__file__).parent / "viewer"

def _safe_preset_path(name: str) -> Path | None:
    """
    Resolve a preset filename to a path inside PRESETS_DIR, or None if it is
    unsafe. Presets may be renamed to ANY filename (spaces, dots, unicode…),
    so instead of whitelisting characters we require a .json file and confirm
    the resolved path stays within PRESETS_DIR — which also blocks traversal
    ('..', absolute paths, embedded separators).
    """
    if not name or not name.lower().endswith(".json"):
        return None
    if "/" in name or "\\" in name or "\x00" in name:
        return None
    base = PRESETS_DIR.resolve()
    path = (base / name).resolve()
    if path.parent != base:
        return None
    return path


class _BroadcastWS:
    """
    Duck-typed stand-in for a single client WebSocket whose ``send_str`` fans
    out to every connected BROWSER client (tool sockets excluded). Lets the
    Participant-Mode automation reuse the per-ws pipeline handlers
    (capture/generate/save/run) unchanged — any open Developer-Mode window
    sees the automated run's stills/previews/results live.
    """

    def __init__(self, server: "Server"):
        self._server = server

    async def send_str(self, msg: str) -> None:
        srv = self._server
        for client in list(srv._ws_clients - srv._tool_clients):
            try:
                await client.send_str(msg)
            except Exception:
                pass


@web.middleware
async def _no_cache_static(request: web.Request, handler):
    """Serve the page and viewer assets with no-cache so code edits show up on a
    plain refresh. Covers both /static/* and the index page at '/' — otherwise a
    stale cached index.html can reference a fresh viewer.js and break the UI."""
    resp = await handler(request)
    if (request.path in ("/", "/projection", "/depths")
            or request.path.startswith(STATIC_PATH)) and not resp.prepared:
        resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    return resp


class Server:
    def __init__(
        self,
        shared_state: dict,
        state_lock: threading.Lock,
        robot,
        on_connect: Callable,
        on_disconnect: Callable,
        on_last_disconnect: Optional[Callable] = None,
        on_simulate_workspace: Optional[Callable] = None,
        on_capture_image: Optional[Callable] = None,
        on_preview_adjust: Optional[Callable] = None,
        on_generate_path: Optional[Callable] = None,
        on_retake: Optional[Callable] = None,
        on_run: Optional[Callable] = None,
        on_cancel: Optional[Callable] = None,
        on_save_path: Optional[Callable] = None,
        on_set_groove_params: Optional[Callable] = None,
        on_set_reference: Optional[Callable] = None,
        on_clear_reference: Optional[Callable] = None,
        on_surface_upload: Optional[Callable] = None,
        on_set_surface_pose: Optional[Callable] = None,
        on_clear_surface: Optional[Callable] = None,
        on_remove_surface: Optional[Callable] = None,
        on_depth_overlay_params: Optional[Callable] = None,
        on_register_freedrive: Optional[Callable] = None,
        on_register_corner: Optional[Callable] = None,
        on_set_trigger: Optional[Callable] = None,
        on_set_automation: Optional[Callable] = None,
        on_set_exec_params: Optional[Callable] = None,
    ):
        self._state = shared_state
        self._lock = state_lock
        self._robot = robot
        self._on_connect = on_connect
        self._on_disconnect = on_disconnect
        self._on_last_disconnect = on_last_disconnect
        self._on_simulate_workspace = on_simulate_workspace
        self._on_capture_image = on_capture_image
        self._on_preview_adjust = on_preview_adjust
        self._on_generate_path = on_generate_path
        self._on_retake = on_retake
        self._on_run = on_run
        self._on_cancel = on_cancel
        self._on_save_path = on_save_path
        self._on_set_groove_params = on_set_groove_params
        self._on_set_reference = on_set_reference
        self._on_clear_reference = on_clear_reference
        self._on_surface_upload = on_surface_upload
        self._on_set_surface_pose = on_set_surface_pose
        self._on_clear_surface = on_clear_surface
        self._on_remove_surface = on_remove_surface
        self._on_depth_overlay_params = on_depth_overlay_params
        self._on_register_freedrive = on_register_freedrive
        self._on_register_corner = on_register_corner
        self._on_set_trigger = on_set_trigger
        self._on_set_automation = on_set_automation
        self._on_set_exec_params = on_set_exec_params
        self._broadcast_ws = _BroadcastWS(self)
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._projection_clients: set[web.WebSocketResponse] = set()
        self._tool_clients: set[web.WebSocketResponse] = set()
        self._overlay_clients: set[web.WebSocketResponse] = set()  # /depths popups
        self._last_labels: Optional[list] = None   # last depth_labels object sent
        self._app = self._build_app()

    def _build_app(self) -> web.Application:
        app = web.Application(middlewares=[_no_cache_static])
        app.router.add_get("/", self._handle_index)
        app.router.add_get(DEPTH_PATH, self._handle_depth)
        app.router.add_get(RGB_PATH, self._handle_rgb)
        app.router.add_get(GROOVE_PATH, self._handle_grooves)
        app.router.add_get(MASK_PATH, self._handle_mask)
        app.router.add_post(SURFACE_UPLOAD_URL, self._handle_surface_upload)
        app.router.add_get("/presets", self._handle_presets_list)
        app.router.add_post("/presets", self._handle_presets_save)
        app.router.add_get("/presets/{name}", self._handle_presets_get)
        app.router.add_get("/status", self._handle_status)
        app.router.add_get("/projection", self._handle_projection_page)
        app.router.add_get("/depths", self._handle_depths_page)
        app.router.add_get("/depth/cropped", self._handle_depth_cropped)
        app.router.add_get("/depth/mask/full", self._handle_mask_full)
        app.router.add_get("/projection/corners", self._handle_corners_get)
        app.router.add_post("/projection/corners", self._handle_corners_post)
        app.router.add_get(WS_PATH, self._handle_ws)
        app.router.add_static(STATIC_PATH, _VIEWER_DIR, show_index=False)
        return app

    async def start(self) -> None:
        runner = web.AppRunner(self._app)
        await runner.setup()
        site = web.TCPSite(runner, HTTP_HOST, HTTP_PORT)
        await site.start()
        print(f"GUI ready → http://{HTTP_HOST}:{HTTP_PORT}")
        await self._broadcast_loop()

    async def _handle_index(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(_VIEWER_DIR / "index.html")

    async def _mjpeg_stream(self, request: web.Request, key: str) -> web.StreamResponse:
        response = web.StreamResponse()
        response.content_type = "multipart/x-mixed-replace; boundary=frame"
        await response.prepare(request)
        try:
            while True:
                with self._lock:
                    jpg = self._state.get(key)
                if jpg:
                    payload = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" +
                        jpg +
                        b"\r\n"
                    )
                    await response.write(payload)
                await asyncio.sleep(1 / 30)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return response

    async def _handle_depth(self, request: web.Request) -> web.StreamResponse:
        return await self._mjpeg_stream(request, "last_depth_color_jpg")

    async def _handle_rgb(self, request: web.Request) -> web.StreamResponse:
        return await self._mjpeg_stream(request, "last_rgb_jpg")

    async def _handle_grooves(self, request: web.Request) -> web.StreamResponse:
        return await self._mjpeg_stream(request, "last_groove_jpg")

    async def _handle_mask(self, request: web.Request) -> web.StreamResponse:
        return await self._mjpeg_stream(request, "last_mask_jpg")

    async def _handle_mask_full(self, request: web.Request) -> web.StreamResponse:
        return await self._mjpeg_stream(request, "last_mask_full_jpg")

    async def _handle_depth_cropped(self, request: web.Request) -> web.StreamResponse:
        """Colorized depth restricted to the Developer-Mode crop — the
        Participant popup's view (composed only while a popup is connected)."""
        return await self._mjpeg_stream(request, "last_depth_crop_jpg")

    async def _handle_status(self, request: web.Request) -> web.Response:
        """Compact app state for external tools (MCP): one JSON object."""
        with self._lock:
            s = self._state
            out = {
                "phase": s.get("phase", "idle"),
                "robot_connected": s.get("robot_connected", False),
                "camera_streaming": s.get("last_depth_color_jpg") is not None,
                "executing": s.get("executing", False),
                "progress": round(s.get("progress", 0.0), 3),
                "exec_error": s.get("exec_error"),
                "stroke_count": len(s.get("strokes", [])),
                "strokes_surface": s.get("strokes_surface", False),
                "surface": (s.get("surface_info") or {}).get("name"),
                "surface_count": (s.get("surface_info") or {}).get("count", 0),
                "reference_set": s.get("reference_depth") is not None,
                "projection_clients": s.get("projection_clients", 0),
                "participant_status": s.get("participant_status", "Off"),
                "trigger_mm": s.get("trigger_mm"),
            }
        return web.json_response(out)

    async def _handle_projection_page(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(_VIEWER_DIR / "projection.html")

    async def _handle_depths_page(self, request: web.Request) -> web.FileResponse:
        return web.FileResponse(_VIEWER_DIR / "depth_view.html")

    def broadcast_ws(self) -> "_BroadcastWS":
        """A ws-like object that broadcasts — for the automation pipeline."""
        return self._broadcast_ws

    async def _handle_corners_get(self, request: web.Request) -> web.Response:
        corners = load_settings().get("projection_corners")
        return web.json_response({"corners": corners})

    async def _handle_corners_post(self, request: web.Request) -> web.Response:
        try:
            data = await request.json()
            corners = data.get("corners")
            if (not isinstance(corners, list) or len(corners) != 4
                    or not all(isinstance(c, list) and len(c) == 2 for c in corners)):
                return web.json_response({"ok": False, "error": "need 4 [x,y] corners"},
                                         status=400)
            save_settings({"projection_corners": corners})
            return web.json_response({"ok": True})
        except Exception as exc:
            return web.json_response({"ok": False, "error": str(exc)}, status=400)

    def _set_projection_count(self) -> None:
        with self._lock:
            self._state["projection_clients"] = len(self._projection_clients)

    def _participant_snapshot(self) -> dict:
        """Participant-Mode block for state/init messages. Caller holds the lock."""
        return {
            "auto": bool(self._state.get("auto_on", False)),
            "status": self._state.get("participant_status", "Auto Off"),
            "message": self._state.get("participant_msg", ""),
            "trigger_mm": self._state.get("trigger_mm"),
            "below": self._state.get("trigger_below"),
        }

    def _set_overlay_count(self) -> None:
        # The camera thread computes depth-number labels only while > 0.
        with self._lock:
            self._state["depth_overlay_clients"] = len(self._overlay_clients)

    async def broadcast_projection_blank(self, on: bool) -> None:
        """Blank/unblank connected projection windows (used during Capture)."""
        msg = json.dumps({"type": "projection_blank", "on": on})
        for client in list(self._projection_clients):
            try:
                await client.send_str(msg)
            except Exception:
                self._projection_clients.discard(client)
        self._set_projection_count()

    async def _handle_surface_upload(self, request: web.Request) -> web.Response:
        """Receive an STL/OBJ mesh (multipart form field 'file') and load it."""
        if not self._on_surface_upload:
            return web.json_response({"ok": False, "error": "not supported"}, status=501)
        try:
            data = await request.post()
            field = data.get("file")
            if field is None or not getattr(field, "filename", None):
                return web.json_response({"ok": False, "error": "no file"}, status=400)
            result = await self._on_surface_upload(field.filename, field.file.read())
            return web.json_response({"ok": True, **result})
        except Exception as exc:
            return web.json_response({"ok": False, "error": str(exc)}, status=400)

    # ── Detection-parameter presets ──────────────────────────────────────────
    # Save/list/load the Detection Parameters sliders as small JSON files under
    # PRESETS_DIR. The slider values live in the browser, so Save just persists
    # the posted params object; Load hands one back for the browser to apply.
    async def _handle_presets_list(self, request: web.Request) -> web.Response:
        PRESETS_DIR.mkdir(parents=True, exist_ok=True)
        items = [
            {"name": f.name,
             "saved": datetime.fromtimestamp(f.stat().st_mtime)
                              .strftime("%Y-%m-%d %H:%M:%S")}
            for f in sorted(PRESETS_DIR.glob("*.json"), reverse=True)
        ]
        return web.json_response({"presets": items})

    async def _handle_presets_save(self, request: web.Request) -> web.Response:
        try:
            data = await request.json()
        except Exception:
            return web.json_response({"ok": False, "error": "bad JSON"}, status=400)
        params = data.get("params")
        if not isinstance(params, dict):
            return web.json_response({"ok": False, "error": "no params"}, status=400)
        PRESETS_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        name, path, n = f"{ts}.json", PRESETS_DIR / f"{ts}.json", 2
        while path.exists():          # guard two saves within the same second
            name = f"{ts}_{n}.json"
            path = PRESETS_DIR / name
            n += 1
        path.write_text(json.dumps(params, indent=2), encoding="utf-8")
        return web.json_response({"ok": True, "name": name})

    async def _handle_presets_get(self, request: web.Request) -> web.Response:
        name = request.match_info.get("name", "")
        path = _safe_preset_path(name)
        if path is None:
            return web.json_response({"ok": False, "error": "bad name"}, status=400)
        if not path.is_file():
            return web.json_response({"ok": False, "error": "not found"}, status=404)
        try:
            params = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            return web.json_response({"ok": False, "error": str(exc)}, status=400)
        return web.json_response({"ok": True, "name": name, "params": params})

    async def _handle_ws(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._ws_clients.add(ws)

        settings = load_settings()
        with self._lock:
            ws_cfg = self._state.get("workspace")
        await self._send_init(ws, settings.get("last_ip", ""), ws_cfg)

        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    await self._handle_ws_message(ws, msg.data)
        finally:
            was_tool = ws in self._tool_clients
            self._tool_clients.discard(ws)
            self._ws_clients.discard(ws)
            self._projection_clients.discard(ws)
            self._set_projection_count()
            self._overlay_clients.discard(ws)
            self._set_overlay_count()
            # Shutdown-on-last-disconnect tracks BROWSER clients only: an MCP
            # tool connecting and disconnecting must not kill the app.
            if (not was_tool
                    and not (self._ws_clients - self._tool_clients)
                    and self._on_last_disconnect):
                asyncio.create_task(self._on_last_disconnect())

        return ws

    async def _send_init(self, ws, last_ip: str, ws_cfg) -> None:
        with self._lock:
            surface_info = self._state.get("surface_info")
            surface_pose = self._state.get("surface_pose")
            surface_offset = self._state.get("surface_offset_mm", 0.0)
            surface_mesh = self._state.get("surface_mesh_payload")
            participant = self._participant_snapshot()
            detect = self._state.get("participant_gen_params") or {}
            exec_p = self._state.get("participant_exec_params") or {}
        try:
            await ws.send_str(json.dumps({
                "type": "init",
                "participant": participant,
                # Current session settings (crop/adjustments/spacing + exec-bar
                # values) so a reopened Developer window restores its controls
                # instead of showing — and later re-sending — the defaults.
                "detect": detect,
                "exec": exec_p,
                "last_ip": last_ip,
                "workspace": ws_cfg.to_browser_dict() if ws_cfg is not None else None,
                "surface": {
                    "loaded": surface_info is not None,
                    "info": surface_info,
                    "pose": surface_pose,
                    "offset_mm": surface_offset,
                    "mesh": surface_mesh,
                },
            }))
        except Exception:
            pass

    async def broadcast_surface_status(self, loaded: bool, info=None, pose=None,
                                       offset_mm: float = 0.0, mesh=None,
                                       message: str = "") -> None:
        """Tell every client the surface changed (mesh sent only when included)."""
        msg = json.dumps({
            "type": "surface_status",
            "loaded": loaded,
            "info": info,
            "pose": pose,
            "offset_mm": offset_mm,
            "mesh": mesh,
            "message": message,
        })
        dead = set()
        for client in list(self._ws_clients):
            try:
                await client.send_str(msg)
            except Exception:
                dead.add(client)
        self._ws_clients -= dead

    async def _handle_ws_message(self, ws, raw: str) -> None:
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return

        msg_type = data.get("type", "")

        if msg_type == "connect":
            ip = data.get("ip", "").strip()
            if ip:
                await self._on_connect(ip, ws)

        elif msg_type == "disconnect":
            await self._on_disconnect(ws)

        elif msg_type == "simulate_workspace":
            if self._on_simulate_workspace:
                asyncio.create_task(self._on_simulate_workspace())

        elif msg_type == "capture_image":
            if self._on_capture_image:
                asyncio.create_task(self._on_capture_image(ws))

        elif msg_type == "preview_adjust":
            if self._on_preview_adjust:
                asyncio.create_task(self._on_preview_adjust(ws, data.get("params", {})))

        elif msg_type == "generate_path":
            if self._on_generate_path:
                asyncio.create_task(self._on_generate_path(ws, data.get("params", {})))

        elif msg_type == "retake":
            if self._on_retake:
                asyncio.create_task(self._on_retake(ws))

        elif msg_type == "run":
            if self._on_run:
                asyncio.create_task(self._on_run(ws, data.get("params", {})))

        elif msg_type == "cancel":
            if self._on_cancel:
                asyncio.create_task(self._on_cancel(ws))

        elif msg_type == "save_path":
            if self._on_save_path:
                asyncio.create_task(self._on_save_path(ws, data.get("params", {})))

        elif msg_type == "set_groove_params":
            if self._on_set_groove_params:
                asyncio.create_task(self._on_set_groove_params(data.get("params", {})))

        elif msg_type == "set_reference":
            if self._on_set_reference:
                asyncio.create_task(self._on_set_reference(ws))

        elif msg_type == "clear_reference":
            if self._on_clear_reference:
                asyncio.create_task(self._on_clear_reference(ws))

        elif msg_type == "set_surface_pose":
            if self._on_set_surface_pose:
                asyncio.create_task(self._on_set_surface_pose(data.get("params", {})))

        elif msg_type == "clear_surface":
            if self._on_clear_surface:
                asyncio.create_task(self._on_clear_surface())

        elif msg_type == "remove_surface":
            if self._on_remove_surface:
                asyncio.create_task(self._on_remove_surface(data.get("params", {})))

        elif msg_type == "tool_hello":
            # External tool (MCP) socket: exempt from shutdown-on-last-disconnect.
            self._tool_clients.add(ws)

        elif msg_type == "register_freedrive":
            if self._on_register_freedrive:
                asyncio.create_task(self._on_register_freedrive(ws, data.get("params", {})))

        elif msg_type == "register_corner":
            if self._on_register_corner:
                asyncio.create_task(self._on_register_corner(ws, data.get("params", {})))

        elif msg_type == "depth_overlay_hello":
            # This socket is a /depths popup: the camera thread computes the
            # depth-number labels only while at least one is connected.
            self._overlay_clients.add(ws)
            self._set_overlay_count()

        elif msg_type == "set_exec_params":
            # Live sync of the exec-bar values (speed/offset/safety/spacing) so
            # Participant Mode always matches what Developer Mode shows.
            if self._on_set_exec_params:
                asyncio.create_task(self._on_set_exec_params(data.get("params", {})))

        elif msg_type == "set_trigger":
            # Participant-Mode trigger distance (mm); null/empty clears it.
            if self._on_set_trigger:
                asyncio.create_task(self._on_set_trigger(data.get("params", {})))

        elif msg_type == "set_automation":
            # Participant popup Auto toggle; ON locks the manual pipeline buttons.
            if self._on_set_automation:
                asyncio.create_task(self._on_set_automation(data.get("params", {})))

        elif msg_type == "depth_overlay_params":
            if self._on_depth_overlay_params:
                asyncio.create_task(self._on_depth_overlay_params(data.get("params", {})))

        elif msg_type == "projection_hello":
            # This socket is a projection window: full-frame mask composition
            # in the camera thread switches on while any are connected.
            self._projection_clients.add(ws)
            self._set_projection_count()

        elif msg_type == "projection_corners":
            # Corner-pin update from the calibration window: persist it and
            # mirror it to the other projection windows (e.g. the projector
            # output) so they warp live while the user drags on the laptop.
            corners = data.get("corners")
            if (isinstance(corners, list) and len(corners) == 4
                    and all(isinstance(c, list) and len(c) == 2 for c in corners)):
                save_settings({"projection_corners": corners})
                msg = json.dumps({"type": "projection_corners", "corners": corners})
                for client in list(self._projection_clients):
                    if client is ws:
                        continue          # don't echo back to the sender
                    try:
                        await client.send_str(msg)
                    except Exception:
                        self._projection_clients.discard(client)
                self._set_projection_count()

    async def _broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(VIS_INTERVAL)
            if not self._ws_clients:
                continue

            # Depth-number labels → only to /depths popups, only when the
            # camera thread produced a fresh list (identity check: it swaps
            # the whole object, ~4 Hz, so most 20 Hz ticks send nothing).
            if self._overlay_clients:
                with self._lock:
                    labels = self._state.get("depth_labels")
                    size = self._state.get("depth_labels_size")
                if labels is not None and labels is not self._last_labels:
                    self._last_labels = labels
                    # ``size`` = [w, h] px of the cropped region the labels
                    # (and the /depth/cropped stream) cover.
                    lmsg = json.dumps({"type": "depth_labels", "labels": labels,
                                       "size": size})
                    for client in list(self._overlay_clients):
                        try:
                            await client.send_str(lmsg)
                        except Exception:
                            self._overlay_clients.discard(client)

            with self._lock:
                connected  = self._state.get("robot_connected", False)
                ee         = self._state.get("ee", [0.0] * 6)
                phase      = self._state.get("phase", "idle")
                strokes    = self._state.get("strokes", [])
                executing  = self._state.get("executing", False)
                progress   = self._state.get("progress", 0.0)
                freedrive  = self._state.get("freedrive", False)
                ws_pts     = self._state.get("ws_points", {})
                ws_cfg     = self._state.get("workspace")
                exec_error = self._state.get("exec_error")
                participant = self._participant_snapshot()

            msg = json.dumps({
                "type": "state",
                "robot_connected": connected,
                "ee": list(ee[:3]),
                "phase": phase,
                "stroke_count": len(strokes),
                "executing": executing,
                "progress": round(progress, 3),
                "freedrive": freedrive,
                "ws_points": {
                    k: ([round(v, 4) for v in vals] if vals is not None else None)
                    for k, vals in ws_pts.items()
                },
                "workspace": ws_cfg.to_browser_dict() if ws_cfg is not None else None,
                "exec_error": exec_error,
                "participant": participant,
            })

            dead = set()
            for client in list(self._ws_clients):
                try:
                    await client.send_str(msg)
                except Exception:
                    dead.add(client)
            self._ws_clients -= dead

    async def send_connection_result(self, ws, success: bool, message: str) -> None:
        try:
            await ws.send_str(json.dumps({
                "type": "connection_result",
                "success": success,
                "message": message,
            }))
        except Exception:
            pass

    @staticmethod
    def _data_url(jpg: Optional[bytes]) -> Optional[str]:
        if not jpg:
            return None
        return "data:image/jpeg;base64," + base64.b64encode(jpg).decode("ascii")

    async def send_still(self, ws, depth_jpg: Optional[bytes], rgb_jpg: Optional[bytes],
                         width: int, height: int) -> None:
        """Send the frozen still (colorized depth + aligned RGB) plus its dimensions."""
        try:
            await ws.send_str(json.dumps({
                "type": "still",
                "depth": self._data_url(depth_jpg),
                "rgb": self._data_url(rgb_jpg),
                "width": width,
                "height": height,
            }))
        except Exception:
            pass

    async def send_preview(self, ws, depth_jpg: Optional[bytes],
                           grooves_jpg: Optional[bytes],
                           mask_jpg: Optional[bytes],
                           rgb_jpg: Optional[bytes] = None) -> None:
        """Send the edit preview: full colorized depth + cropped RGB/skeleton/mask."""
        try:
            await ws.send_str(json.dumps({
                "type": "preview",
                "depth": self._data_url(depth_jpg),
                "rgb": self._data_url(rgb_jpg),
                "grooves": self._data_url(grooves_jpg),
                "mask": self._data_url(mask_jpg),
            }))
        except Exception:
            pass

    async def send_save_result(self, ws, success: bool, folder: str = "",
                               error: Optional[str] = None) -> None:
        try:
            await ws.send_str(json.dumps({
                "type": "save_result",
                "success": success,
                "folder": folder,
                "error": error,
            }))
        except Exception:
            pass

    async def send_register_result(self, ws, success: bool, message: str = "",
                                   pose: Optional[dict] = None,
                                   error: Optional[str] = None) -> None:
        """Outcome of a corner→TCP registration step (freedrive or confirm)."""
        try:
            await ws.send_str(json.dumps({
                "type": "register_result",
                "success": success,
                "message": message,
                "pose": pose,
                "error": error,
            }))
        except Exception:
            pass

    async def send_reference_status(self, ws, active: bool, message: str) -> None:
        try:
            await ws.send_str(json.dumps({
                "type": "reference_status",
                "active": active,
                "message": message,
            }))
        except Exception:
            pass

    async def send_capture_result(
        self,
        ws,
        success: bool,
        stroke_count: int = 0,
        point_count: int = 0,
        strokes_data: Optional[list] = None,
        error: Optional[str] = None,
        reach_flags: Optional[list] = None,
        reach_out: int = 0,
        skeleton_data: Optional[list] = None,
        exec_viz: Optional[dict] = None,
    ) -> None:
        try:
            await ws.send_str(json.dumps({
                "type": "capture_result",
                "success": success,
                "stroke_count": stroke_count,
                "point_count": point_count,
                "strokes": strokes_data or [],
                "error": error,
                "reach_flags": reach_flags or [],
                "reach_out": reach_out,
                # Dense on-surface skeleton polylines ([x,y,z] only) — the white
                # preview line. Separate from the movep waypoint strokes above.
                "skeleton": skeleton_data or [],
                # blend_m / reach_m / min_reach_m / spacing_mm for the browser's
                # client-side toolpath rebuild (exec-bar Offset/Safety changes).
                "exec_viz": exec_viz or {},
            }))
        except Exception:
            pass

    async def broadcast_execution_update(self, phase: str, progress: float, error: Optional[str] = None) -> None:
        msg = json.dumps({
            "type": "execution_update",
            "phase": phase,
            "progress": round(progress, 3),
            "error": error,
        })
        dead = set()
        for client in list(self._ws_clients):
            try:
                await client.send_str(msg)
            except Exception:
                dead.add(client)
        self._ws_clients -= dead
