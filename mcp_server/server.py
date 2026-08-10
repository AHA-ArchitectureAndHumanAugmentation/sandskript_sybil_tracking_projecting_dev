"""
MCP server for depth-cam-to-robot — exposes the pipeline as tools.

Thin CLIENT over the running app (default http://127.0.0.1:5005): the app owns
the RealSense and the robot (one process each), so tools talk HTTP/WS instead
of importing pipeline modules — importing main.py would start hardware threads.
Start the app first (run.bat). Deliberately NO run() tool: moving the robot
stays a human action in the browser.

Every tool returns a compact JSON summary; large artefacts stay in files and
tools return their paths. Override the app URL with env DEPTH_APP_URL.
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
from pathlib import Path

import aiohttp
from mcp.server.fastmcp import FastMCP

# Repo root importable (for reach/config) WITHOUT importing main.py.
_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

APP_URL = os.environ.get("DEPTH_APP_URL", "http://127.0.0.1:5005")
WS_URL = APP_URL.replace("http", "ws", 1) + "/ws"

mcp = FastMCP("depth-cam-to-robot")

_NOT_RUNNING = ("app not reachable at " + APP_URL +
                " — start it first (run.bat / python main.py)")


def _err(msg: str) -> dict:
    return {"ok": False, "error": msg}


async def _ws_call(send: dict, reply_types: tuple[str, ...],
                   timeout: float = 60.0) -> dict:
    """Send one WS message to the app and wait for the first matching reply."""
    try:
        async with aiohttp.ClientSession() as s:
            async with s.ws_connect(WS_URL, heartbeat=20) as ws:
                await ws.send_str(json.dumps({"type": "tool_hello"}))
                await ws.send_str(json.dumps(send))
                loop = asyncio.get_event_loop()
                end = loop.time() + timeout
                while True:
                    left = end - loop.time()
                    if left <= 0:
                        return _err(f"timed out waiting for {reply_types}")
                    msg = await ws.receive(timeout=left)
                    if msg.type in (aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR):
                        return _err("app closed the connection")
                    if msg.type != aiohttp.WSMsgType.TEXT:
                        continue
                    d = json.loads(msg.data)
                    if d.get("type") in reply_types:
                        return d
    except aiohttp.ClientConnectorError:
        return _err(_NOT_RUNNING)
    except asyncio.TimeoutError:
        return _err(f"timed out waiting for {reply_types}")


async def _ws_send(send: dict) -> dict:
    """Fire-and-forget WS message (for messages the app doesn't reply to)."""
    try:
        async with aiohttp.ClientSession() as s:
            async with s.ws_connect(WS_URL) as ws:
                await ws.send_str(json.dumps({"type": "tool_hello"}))
                await ws.send_str(json.dumps(send))
                await asyncio.sleep(0.15)   # let the app consume it
        return {"ok": True}
    except aiohttp.ClientConnectorError:
        return _err(_NOT_RUNNING)


# ── tools ─────────────────────────────────────────────────────────────────────
@mcp.tool()
async def app_status() -> dict:
    """App/pipeline status: phase, camera streaming, robot connected, stroke count, surface, projector clients."""
    try:
        async with aiohttp.ClientSession() as s:
            async with s.get(APP_URL + "/status",
                             timeout=aiohttp.ClientTimeout(total=5)) as r:
                return {"ok": True, **(await r.json())}
    except Exception:
        return _err(_NOT_RUNNING)


@mcp.tool()
async def capture_image() -> dict:
    """Capture a temporally averaged depth+RGB still (needs live camera; takes ~2 s more if a projector window is open)."""
    d = await _ws_call({"type": "capture_image"}, ("still", "capture_result"),
                       timeout=20.0)
    if d.get("type") == "still":
        return {"ok": True, "width": d.get("width"), "height": d.get("height"),
                "note": "still captured — app is now in editing phase"}
    if d.get("type") == "capture_result":
        return _err(d.get("error") or "capture failed")
    return d


@mcp.tool()
async def generate_path(adjustments: dict | None = None,
                        crop: dict | None = None,
                        spacing_mm: float | None = None,
                        join_mm: float | None = None) -> dict:
    """Generate the toolpath from the captured still; returns stroke/point counts + reach violations (never raw strokes). adjustments: groove params (groove_depth_mm, detect, min_length_mm, ...); crop: {x,y,w,h} normalized; spacing_mm: waypoint spacing in mm (10-100, default 10); join_mm: Distance Threshold in mm (0-200, default 0 = off) — endpoints of different strokes closer than this are connected so the strokes merge into one toolpath (threshold doubles when another stroke crosses the connecting line)."""
    params = {"adjustments": adjustments or {}, "crop": crop or {}}
    if spacing_mm is not None:
        params["spacing_mm"] = spacing_mm
    if join_mm is not None:
        params["join_mm"] = join_mm
    d = await _ws_call({"type": "generate_path", "params": params},
                       ("capture_result",), timeout=120.0)
    if "error" in d and not d.get("success", False):
        return _err(d.get("error") or "generation failed")
    flags = d.get("reach_flags") or []
    bad_strokes = [i for i, f in enumerate(flags) if any(f)]
    return {
        "ok": bool(d.get("success")),
        "stroke_count": d.get("stroke_count", 0),
        "point_count": d.get("point_count", 0),
        "reach_out": d.get("reach_out", 0),
        "unreachable_strokes": bad_strokes[:20],
        "note": "reach check is envelope-only (no IK/collision model)",
    }


@mcp.tool()
async def load_surface(stl_path: str) -> dict:
    """Add an STL/OBJ target surface (Rhino export, millimetres) to the app's surface scene. CUMULATIVE: each call adds a part, keeping the position authored in the file, and the loaded surfaces act as one target (loading the same file name again replaces that part). Returns name/faces/bbox of the combined scene plus count/parts."""
    p = Path(stl_path)
    if not p.exists():
        return _err(f"file not found: {stl_path}")
    try:
        async with aiohttp.ClientSession() as s:
            form = aiohttp.FormData()
            form.add_field("file", p.read_bytes(), filename=p.name)
            async with s.post(APP_URL + "/surface/upload", data=form,
                              timeout=aiohttp.ClientTimeout(total=60)) as r:
                out = await r.json()
                return {"ok": bool(out.get("ok")), **out.get("info", {}),
                        "error": out.get("error")}
    except aiohttp.ClientConnectorError:
        return _err(_NOT_RUNNING)


@mcp.tool()
async def set_surface_pose(tx: float = 0.4, ty: float = 0.0, tz: float = 0.0,
                           rx: float = 0.0, ry: float = 0.0, rz: float = 0.0,
                           offset_mm: float = 0.0) -> dict:
    """Place the loaded surface in the robot base frame (metres + XYZ Euler degrees) and set the generate-time TCP offset (mm). Re-run generate_path to apply."""
    pose = {"tx": tx, "ty": ty, "tz": tz, "rx": rx, "ry": ry, "rz": rz}
    out = await _ws_send({"type": "set_surface_pose",
                          "params": {"pose": pose, "offset_mm": offset_mm}})
    if out.get("ok"):
        out["note"] = "pose set — re-run generate_path to apply it to the strokes"
    return out


@mcp.tool()
async def save_toolpath(speed_pct: float = 5.0, offset_mm: float = 0.0,
                        safety_mm: float = 50.0, blend_mm: float = 0.5) -> dict:
    """Save the generated toolpath (URScript + JSON with per-waypoint frames, plus mask.png/skeleton.png of the detection it came from) to a timestamped folder under paths/; returns the folder path. blend_mm = movep corner blend radius (0-5 mm, clamped per stroke)."""
    d = await _ws_call({"type": "save_path",
                        "params": {"speed_pct": speed_pct, "offset_mm": offset_mm,
                                   "safety_mm": safety_mm, "blend_mm": blend_mm}},
                       ("save_result",), timeout=30.0)
    if d.get("success"):
        return {"ok": True, "folder": d.get("folder"),
                "note": "mask.png + skeleton.png written; preview.png only if a "
                        "Developer window pushed one (that canvas is browser-only)"}
    return _err(d.get("error") or "save failed")


@mcp.tool()
def validate_toolpath(path_json: str) -> dict:
    """Reach-check a saved path.json offline (envelope only: 1.30 m sphere minus 0.18 m axis cylinder — no IK/collision). Returns pass/fail + first violations."""
    from reach import reach_flags   # safe: does not import main.py

    p = Path(path_json)
    if p.is_dir():
        p = p / "path.json"
    if not p.exists():
        return _err(f"file not found: {p}")
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        strokes = [[wp["pose"] for wp in s] for s in data["strokes"]]
    except Exception as exc:
        return _err(f"not a valid path.json: {exc}")

    flags, n_out, n_total = reach_flags(strokes)
    violations = [{"stroke": si, "point": pi}
                  for si, f in enumerate(flags) for pi, bad in enumerate(f) if bad]
    return {
        "ok": n_out == 0,
        "waypoints": n_total,
        "out_of_reach": n_out,
        "violations": violations[:20],
        "check": "envelope-only (no IK / joint limits / collision)",
    }


if __name__ == "__main__":
    mcp.run()
