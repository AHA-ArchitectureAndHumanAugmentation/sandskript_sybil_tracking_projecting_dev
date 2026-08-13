import asyncio
import functools
import math
import random
import os
import signal
import sys
import threading
import webbrowser
import json
import subprocess
import tempfile
import cv2
import numpy as np

# Force UTF-8 console output so Unicode in log messages (→, ×, …) never crashes
# the program on Windows consoles that default to a legacy code page (cp1252).
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8")
    except (AttributeError, ValueError):
        pass

from config import (
    HTTP_HOST, HTTP_PORT, CONTOUR_MIN_PIXELS,
    DEPTH_LABELS_INTERVAL_MM,
    DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FPS, DEPTH_AVERAGE_FRAMES, SURFACE_DIR,
    DRAW_SPEED, TRAVEL_Z, MAX_TCP_SPEED,
    RESAMPLE_SPACING_MM, RESAMPLE_SPACING_MIN_MM, RESAMPLE_SPACING_MAX_MM,
    JOIN_DISTANCE_MM, JOIN_DISTANCE_MIN_MM, JOIN_DISTANCE_MAX_MM,
    UR_REACH_M, UR_MIN_REACH_M, MOVEP_BLEND_M,
    PARTICIPANT_TICK_S, PARTICIPANT_CLEAR_S,
    PROFANITY_CHECK_ENABLED,
)
import text_guard
import module_trace
from automation import ParticipantAutomation
from camera_thread import DepthCameraThread
from depth_extractor import (
    Crop, DepthGrooveParams, colorize_depth, encode_jpeg, process_depth,
)
from path_extractor import extract_from_edges, pixels_to_robot_coords
from path_export import is_png_data_url, save_bundle
from server import Server
from settings import load_settings, save_settings
from surface import SurfaceModel, SurfacePose, SurfaceScene
from workspace import WorkspaceConfig, scene_mm_per_px
from zmq_bridge import send_path_to_charlotte, TileReceiver
from tile_viewer import show_flat_stroke_preview, show_projection_preview

# ── Shared state ──────────────────────────────────────────────────────────────
# NOTE: robot_connected / freedrive / ee / executing / progress are kept in
# this dict (always at their default, never set True) purely so server.py's
# existing UI payload shape doesn't break on a missing key. No robot logic
# writes to any of them anymore.
shared_state: dict = {
    "robot_connected":     False,
    "last_depth_color_jpg": None,    # colorized depth (live view)
    "last_rgb_jpg":        None,     # aligned colour image (live view)
    "last_groove_jpg":     None,     # detected groove skeleton (live preview)
    "last_mask_jpg":       None,     # thick detected-region mask (live preview)
    "last_mask_full_jpg":  None,     # full-frame mask for the projector (gated)
    "projection_clients":  0,        # connected projection windows
    "depth_overlay_clients": 0,      # connected /depths popups (gates the labels)
    "depth_labels":        None,     # [[u, v, mm], ...] for the depth-number overlay
    "depth_labels_size":   None,     # [w, h] px of the crop the labels cover
    "last_depth_crop_jpg": None,     # cropped colorized depth for the popup
    "workspace":           None,     # WorkspaceConfig | None — confirmed workspace
    "pending_workspace":   None,     # WorkspaceConfig | None — loaded from disk
    "ws_points":           {"p0": None, "px": None, "py": None},
    "freedrive":           False,
    "ee":                  [0.0] * 6,
    "phase":               "idle",     # idle|previewing|editing|captured|error
    "captured_still":      None,       # (depth_m, valid, rgb) — frozen averaged depth + colour
    "reference_depth":     None,       # baseline depth frame for background subtraction
    "surface_model":       None,       # SurfaceModel | None — target mesh for 3D projection
    "surface_info":        None,       # dict for the browser (name/faces/bbox)
    "surface_pose":        SurfacePose().to_dict(),   # placement in robot base frame
    "surface_offset_mm":   0.0,        # TCP offset along the surface normal
    "surface_mesh_payload": None,      # local-frame vertices/faces for the 3D preview
    "strokes_surface":     False,      # True when current strokes were surface-projected
    "still_dims":          None,       # (width, height) of the captured still
    "strokes":             [],         # projected strokes after Generate Path
    "last_mask":           None,       # thick detected-region mask, cropped
    "last_skeleton":       None,       # 1-px groove centrelines, cropped
    "last_preview_png":    None,       # PNG data URL | None
    "path_serial":         0,          # bumped by every Generate Path
    "executing":           False,
    "progress":            0.0,
    # ── Participant Mode (automated pipeline) ──
    "auto_on":             False,
    "trigger_mm":          None,
    "trigger_below":       None,
    "participant_status":  "Auto Off",
    "participant_msg":     "",
    "participant_gen_params":  {},
    "participant_exec_params": {},
    "next_tile_id": None,   # set by zmq_bridge.TileReceiver; the tile Charlotte wants next
}
state_lock = threading.Lock()

EMULATE_CAPTURE = True  # True = no camera -- auto-generate + save + send after every tile switch. Set False once the camera's reconnected.
STARTUP_TILE_ID = 1  # tile used immediately on startup, before any message arrives. None = wait for a real message like before.
SHOW_PROJECTION_VIEWER = True  # True = open a compas_viewer window (tile + projected path) before every send. False = run unattended.

# ── Singletons ────────────────────────────────────────────────────────────────
camera_thread = DepthCameraThread(shared_state, state_lock)
automation    = ParticipantAutomation(
    clear_ticks=round(PARTICIPANT_CLEAR_S / PARTICIPANT_TICK_S))


# ── Robot connection -- STUBS ONLY ────────────────────────────────────────────
# No RobotController, no RTDE, nothing physical. These exist only so
# server.py's callback slots have something to call.
async def on_robot_connect(ip: str, ws) -> None:
    await server.send_connection_result(ws, False, "Robot connection is not available in this repo -- see Charlotte's pipeline.")


async def on_robot_disconnect(ws) -> None:
    if ws is not None:
        await server.send_connection_result(ws, False, "Disconnected")


async def on_last_client_disconnect() -> None:
    print("Last client disconnected — stopping camera.")
    camera_thread.stop()
    os.kill(os.getpid(), signal.SIGINT)


# ── Workspace setup callbacks ─────────────────────────────────────────────────
async def on_simulate_workspace() -> None:
    """Set a synthetic workspace so the depth→groove→Path-Preview pipeline
    can be tested without a surface loaded."""
    ws_cfg = WorkspaceConfig.simulation()
    with state_lock:
        shared_state["workspace"]         = ws_cfg
        shared_state["pending_workspace"] = None
        shared_state["ws_points"]         = {"p0": None, "px": None, "py": None}
        shared_state["phase"]             = "previewing"
    if not camera_thread.running:
        camera_thread.start()
    module_trace.log(
        "generate",
        f"Simulation workspace active: "
        f"{ws_cfg.x_extent:.3f} m × {ws_cfg.y_extent:.3f} m — Capture enabled.",
        extra=("workspace",),
    )


# ── Capture image / Edit / Generate path callbacks ───────────────────────────
def _mm_per_px(workspace, surface_model=None) -> float | None:
    return scene_mm_per_px(workspace, surface_model)


async def on_set_groove_params(params: dict) -> None:
    gp = DepthGrooveParams.from_dict(params.get("adjustments"))
    crop = Crop.from_dict(params.get("crop"))
    with state_lock:
        workspace = shared_state.get("workspace")
        surface_model = shared_state.get("surface_model")
        shared_state["participant_gen_params"].update(
            {"crop": params.get("crop"), "adjustments": params.get("adjustments")})
    camera_thread.set_live_params(gp)
    camera_thread.set_live_crop(crop)
    camera_thread.set_scale(_mm_per_px(workspace, surface_model))


async def on_depth_overlay_params(params: dict) -> None:
    try:
        interval = float(params.get("interval_mm", DEPTH_LABELS_INTERVAL_MM))
    except (TypeError, ValueError):
        interval = DEPTH_LABELS_INTERVAL_MM
    camera_thread.set_depth_label_interval(min(max(interval, 1.0), 100.0))


async def on_set_reference(ws) -> None:
    captured = camera_thread.capture_frame()
    if captured is None:
        await server.send_reference_status(ws, False, "No depth frame to set as reference.")
        return
    depth_m, _valid, _rgb = captured
    with state_lock:
        shared_state["reference_depth"] = depth_m
    camera_thread.set_reference(depth_m)
    await server.send_reference_status(ws, True, "Reference captured — natural grooves can be subtracted.")
    module_trace.log("reference", "Reference depth captured for background subtraction.")


async def on_clear_reference(ws) -> None:
    with state_lock:
        shared_state["reference_depth"] = None
    camera_thread.set_reference(None)
    await server.send_reference_status(ws, False, "Reference cleared.")
    module_trace.log("reference", "Reference depth cleared.")


# ── Target surface (3D projection) callbacks ─────────────────────────────────
_surface_lock = asyncio.Lock()


async def on_surface_upload(filename: str, blob: bytes) -> dict:
    SURFACE_DIR.mkdir(exist_ok=True)
    safe_name = os.path.basename(filename)
    path = SURFACE_DIR / safe_name
    path.write_bytes(blob)

    loop = asyncio.get_running_loop()
    model = await loop.run_in_executor(None, SurfaceModel.load, path)
    async with _surface_lock:
        with state_lock:
            existing = shared_state.get("surface_model")
        scene = await loop.run_in_executor(None, SurfaceScene.combine, existing, model)
        info = scene.info()
        mesh_payload = await loop.run_in_executor(None, scene.mesh_payload)

        with state_lock:
            shared_state["surface_model"] = scene
            shared_state["surface_info"] = info
            shared_state["surface_mesh_payload"] = mesh_payload
            pose = shared_state["surface_pose"]
            offset = shared_state["surface_offset_mm"]
            if shared_state["phase"] == "idle":
                shared_state["phase"] = "previewing"

    if not camera_thread.running:
        camera_thread.start()

    added = f"Surface loaded: {safe_name} ({int(len(model.mesh.faces))} faces)"
    if info["count"] > 1:
        added += (f" — {info['count']} surfaces combined, "
                  f"{info['bbox']['size'][0]}×{info['bbox']['size'][1]} m total")
    await server.broadcast_surface_status(
        loaded=True, info=info, pose=pose, offset_mm=offset, mesh=mesh_payload,
        message=added,
    )
    module_trace.log("surface", f"[surface] {added}; scene bbox {info['bbox']['size']} m")
    return {"info": info}


async def switch_to_tile(tile_id: int) -> None:
    """Loads surfaces/tile_{tile_id}.obj as the ONLY active surface."""
    path = SURFACE_DIR / f"tile_{tile_id:03d}.obj"
    if not path.is_file():
        module_trace.log("surface", f"[zmq_bridge] tile {tile_id} not found: {path}")
        return

    loop = asyncio.get_running_loop()
    model = await loop.run_in_executor(None, SurfaceModel.load, path)
    async with _surface_lock:
        scene = await loop.run_in_executor(None, SurfaceScene.combine, None, model)
        info = scene.info()
        mesh_payload = await loop.run_in_executor(None, scene.mesh_payload)
        with state_lock:
            shared_state["surface_model"] = scene
            shared_state["surface_info"] = info
            shared_state["surface_mesh_payload"] = mesh_payload
            pose = shared_state["surface_pose"]
            offset = shared_state["surface_offset_mm"]

    await server.broadcast_surface_status(
        loaded=True, info=info, pose=pose, offset_mm=offset, mesh=mesh_payload,
        message=f"Switched to tile {tile_id}",
    )
    module_trace.log("surface", f"[zmq_bridge] switched to tile {tile_id}")


def _emulate_pixel_stroke(width: int, height: int) -> list:
    """Synthetic 2D drawing -- a sine wave, randomized a bit each call so
    successive emulated captures aren't identical. Stands in ONLY for
    camera capture + groove detection -- projection (project_strokes)
    is the same real raycasting code a live capture uses."""
    margin = 0.15
    amplitude = height * 0.3
    frequency = random.uniform(2.0, 4.0)
    phase = random.uniform(0, 2 * math.pi)
    n_points = 80

    points = []
    for i in range(n_points):
        t = i / (n_points - 1)
        x = width * margin + t * width * (1 - 2 * margin)
        y = height / 2.0 + amplitude * math.sin(t * frequency * math.pi + phase)
        points.append((int(x), int(y)))

    return [points]


async def _emulate_capture_and_save(tile_id: int) -> None:
    """Emulates ONLY capture + groove detection. Everything after this --
    projection, save, send, next-tile selection -- is the same real code
    a live capture would use."""
    with state_lock:
        surface_model = shared_state.get("surface_model")
        surface_pose = SurfacePose.from_dict(shared_state.get("surface_pose"))
    if surface_model is None:
        module_trace.log("surface", f"[emulate] tile {tile_id}: no surface loaded, skipping")
        return

    fake_strokes = _emulate_pixel_stroke(DEPTH_WIDTH, DEPTH_HEIGHT)
    loop = asyncio.get_running_loop()
    try:
        robot_strokes = await loop.run_in_executor(
            None, surface_model.project_strokes,
            fake_strokes, DEPTH_WIDTH, DEPTH_HEIGHT, surface_pose, 0.0,
        )
    except Exception as exc:
        module_trace.log("surface", f"[emulate] projection failed: {exc}")
        return

    if SHOW_PROJECTION_VIEWER:
        mm_per_px = surface_model.drawing_mm_per_px(DEPTH_WIDTH, DEPTH_HEIGHT)

        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            json.dump({
                "pixel_strokes": fake_strokes,
                "frame_width": DEPTH_WIDTH,
                "frame_height": DEPTH_HEIGHT,
                "mm_per_px": mm_per_px,
            }, f)
            flat_data_path = f.name

        module_trace.log("surface", f"[emulate] tile {tile_id}: opening RAW stroke viewer -- close it to continue")
        await loop.run_in_executor(None, subprocess.run, ["python", "tile_viewer.py", "flat", flat_data_path])

        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            json.dump({
                "mesh_vertices": surface_model.mesh.vertices.tolist(),
                "mesh_faces": surface_model.mesh.faces.tolist(),
                "robot_strokes": robot_strokes,
                "tile_id": tile_id,
            }, f)
            projected_data_path = f.name

        module_trace.log("surface", f"[emulate] tile {tile_id}: opening PROJECTED viewer -- close it to continue")
        await loop.run_in_executor(None, subprocess.run, ["python", "tile_viewer.py", "projected", projected_data_path])

    with state_lock:
        shared_state["strokes"] = robot_strokes
        shared_state["strokes_surface"] = True
        shared_state["path_serial"] = shared_state.get("path_serial", 0) + 1

    module_trace.log("surface", f"[emulate] tile {tile_id}: {len(robot_strokes[0])} points -- saving")
    await on_save_path(server.broadcast_ws(), {})



async def _tile_switch_watcher() -> None:
    last_seq = None

    if STARTUP_TILE_ID is not None:
        module_trace.log("surface", f"[startup] using tile {STARTUP_TILE_ID} immediately")
        with state_lock:
            shared_state["next_tile_id"] = STARTUP_TILE_ID
        await switch_to_tile(STARTUP_TILE_ID)
        if EMULATE_CAPTURE:
            await _emulate_capture_and_save(STARTUP_TILE_ID)

    while True:
        await asyncio.sleep(1.0)
        with state_lock:
            tile_id = shared_state.get("next_tile_id")
            seq = shared_state.get("next_tile_seq")
        if tile_id is not None and seq is not None and seq != last_seq:
            last_seq = seq
            await switch_to_tile(tile_id)
            if EMULATE_CAPTURE:
                await _emulate_capture_and_save(tile_id)


async def on_remove_surface(params: dict) -> None:
    try:
        idx = int((params or {}).get("index", -1))
    except (TypeError, ValueError):
        idx = -1
    async with _surface_lock:
        with state_lock:
            scene = shared_state.get("surface_model")
        if not isinstance(scene, SurfaceScene) or not 0 <= idx < len(scene.parts):
            return
        removed = scene.parts[idx].name
        new_scene = scene.without_part(idx)
        if new_scene is None:
            await on_clear_surface()
            return

        loop = asyncio.get_running_loop()
        info = new_scene.info()
        mesh_payload = await loop.run_in_executor(None, new_scene.mesh_payload)
        with state_lock:
            shared_state["surface_model"] = new_scene
            shared_state["surface_info"] = info
            shared_state["surface_mesh_payload"] = mesh_payload
            pose = shared_state["surface_pose"]
            offset = shared_state["surface_offset_mm"]

    msg = f"Removed {removed} — {info['count']} surface(s) left."
    await server.broadcast_surface_status(
        loaded=True, info=info, pose=pose, offset_mm=offset, mesh=mesh_payload,
        message=msg,
    )
    module_trace.log("surface", f"[surface] {msg}")


async def on_set_surface_pose(params: dict) -> None:
    pose = SurfacePose.from_dict(params.get("pose"))
    try:
        offset = float(params.get("offset_mm", 0.0))
    except (TypeError, ValueError):
        offset = 0.0
    offset = min(max(offset, -20.0), 100.0)
    with state_lock:
        shared_state["surface_pose"] = pose.to_dict()
        shared_state["surface_offset_mm"] = offset


async def on_clear_surface() -> None:
    with state_lock:
        shared_state["surface_model"] = None
        shared_state["surface_info"] = None
        shared_state["surface_mesh_payload"] = None
        shared_state["strokes_surface"] = False
    await server.broadcast_surface_status(loaded=False, message="Surfaces cleared.")
    module_trace.log("surface", "[surface] cleared — paths map to the flat workspace again", extra=("workspace",))


# ── Corner registration -- STUBS ONLY (needed the robot's live TCP position) ──
async def on_register_freedrive(ws, params: dict) -> None:
    await server.send_register_result(ws, False, error="Robot registration is not available in this repo.")


async def on_register_corner(ws, params: dict) -> None:
    await server.send_register_result(ws, False, error="Robot registration is not available in this repo.")


async def on_capture_image(ws) -> None:
    """Freeze a temporally averaged depth (+ colour) frame and enter editing."""
    if _manual_locked(ws):
        await server.send_capture_result(ws, False, error=_AUTO_LOCK_MSG)
        return
    with state_lock:
        proj = shared_state.get("projection_clients", 0) > 0
    if proj:
        await server.broadcast_projection_blank(True)
        await asyncio.sleep(DEPTH_AVERAGE_FRAMES / DEPTH_FPS + 0.3)

    try:
        captured = camera_thread.capture_frame()
        if captured is None:
            await server.send_capture_result(ws, False, error="No depth frame available.")
            return

        depth_m, valid, rgb = captured
        h, w = depth_m.shape[:2]
        with state_lock:
            shared_state["captured_still"] = (depth_m, valid, rgb)
            shared_state["still_dims"]     = (w, h)
            shared_state["phase"]          = "editing"
            shared_state["strokes"]        = []

        loop = asyncio.get_running_loop()
        color = await loop.run_in_executor(None, colorize_depth, depth_m, valid)
        depth_jpg = await loop.run_in_executor(None, encode_jpeg, color)
        rgb_jpg = await loop.run_in_executor(None, encode_jpeg, rgb) if rgb is not None else None
        await server.send_still(ws, depth_jpg=depth_jpg, rgb_jpg=rgb_jpg, width=w, height=h)
        module_trace.log("capture", f"Captured still: {w}×{h} (depth+colour) — ready for crop/adjust")
    finally:
        if proj:
            await server.broadcast_projection_blank(False)


async def on_preview_adjust(ws, params: dict) -> None:
    with state_lock:
        still = shared_state.get("captured_still")
    if still is None:
        return

    with state_lock:
        reference = shared_state.get("reference_depth")
        workspace = shared_state.get("workspace")
        surface_model = shared_state.get("surface_model")

    depth_m, valid, _rgb = still
    crop   = Crop.from_dict(params.get("crop"))
    gp     = DepthGrooveParams.from_dict(params.get("adjustments"))
    mmpp   = _mm_per_px(workspace, surface_model)

    loop = asyncio.get_running_loop()
    try:
        processed = await loop.run_in_executor(
            None, process_depth, depth_m, valid, crop, gp, reference, mmpp
        )
    except Exception as exc:
        module_trace.log("preview", f"[preview] processing error: {exc}")
        return

    depth_jpg   = await loop.run_in_executor(None, encode_jpeg, processed.color_full)
    grooves_jpg = await loop.run_in_executor(None, encode_jpeg, processed.grooves)
    mask_jpg    = await loop.run_in_executor(None, encode_jpeg, processed.mask)

    rgb_jpg = None
    if _rgb is not None:
        x0, y0 = processed.origin
        gh, gw = processed.grooves.shape[:2]
        rgb_crop = _rgb[y0:y0 + gh, x0:x0 + gw]
        rgb_jpg = await loop.run_in_executor(None, encode_jpeg, rgb_crop)

    await server.send_preview(
        ws, depth_jpg=depth_jpg, grooves_jpg=grooves_jpg, mask_jpg=mask_jpg, rgb_jpg=rgb_jpg
    )


async def on_generate_path(ws, params: dict) -> None:
    """Run groove extraction on the cropped depth and build the 3D path preview."""
    if _manual_locked(ws):
        await server.send_capture_result(ws, False, error=_AUTO_LOCK_MSG)
        return
    with state_lock:
        workspace     = shared_state.get("workspace")
        still         = shared_state.get("captured_still")
        dims          = shared_state.get("still_dims")
        reference     = shared_state.get("reference_depth")
        surface_model = shared_state.get("surface_model")
        surface_pose  = SurfacePose.from_dict(shared_state.get("surface_pose"))
        surface_offset = shared_state.get("surface_offset_mm", 0.0)

    if workspace is None and surface_model is None:
        await server.send_capture_result(ws, False, error="No workspace configured.")
        return
    if still is None:
        await server.send_capture_result(ws, False, error="No captured depth — press Capture first.")
        return

    depth_m, valid, _rgb = still
    width, height  = dims
    crop = Crop.from_dict(params.get("crop"))
    gp   = DepthGrooveParams.from_dict(params.get("adjustments"))
    mmpp = _mm_per_px(workspace, surface_model)

    try:
        spacing_mm = float(params.get("spacing_mm", RESAMPLE_SPACING_MM))
    except (TypeError, ValueError):
        spacing_mm = RESAMPLE_SPACING_MM
    spacing_mm = min(max(spacing_mm, RESAMPLE_SPACING_MIN_MM), RESAMPLE_SPACING_MAX_MM)

    try:
        join_mm = float(params.get("join_mm", JOIN_DISTANCE_MM))
    except (TypeError, ValueError):
        join_mm = JOIN_DISTANCE_MM
    join_mm = min(max(join_mm, JOIN_DISTANCE_MIN_MM), JOIN_DISTANCE_MAX_MM)

    with state_lock:
        shared_state["participant_gen_params"].update(
            {"crop": params.get("crop"), "adjustments": params.get("adjustments"),
             "spacing_mm": spacing_mm, "join_mm": join_mm})

    loop = asyncio.get_running_loop()
    try:
        processed = await loop.run_in_executor(
            None, process_depth, depth_m, valid, crop, gp, reference, mmpp
        )
        extracted = await loop.run_in_executor(
            None, extract_from_edges, processed.grooves, CONTOUR_MIN_PIXELS,
            processed.origin, spacing_mm, mmpp, join_mm,
        )
    except Exception as exc:
        await server.send_capture_result(ws, False, error=str(exc))
        return

    dense = extracted.strokes_dense or []
    if surface_model is not None:
        try:
            robot_strokes = await loop.run_in_executor(
                None, surface_model.project_strokes,
                extracted.strokes, width, height, surface_pose, surface_offset / 1000.0,
            )
            skeleton_strokes = await loop.run_in_executor(
                None, surface_model.project_strokes,
                dense, width, height, surface_pose, 0.0,
            )
        except Exception as exc:
            await server.send_capture_result(ws, False, error=f"Surface projection: {exc}")
            return
        surface_mode = True
    else:
        robot_strokes = pixels_to_robot_coords(
            extracted.strokes, workspace, width, height, draw_z_offset=0.0
        )
        skeleton_strokes = pixels_to_robot_coords(
            dense, workspace, width, height, draw_z_offset=0.0
        )
        surface_mode = False

    strokes_data = [
        [[round(v, 5) for v in pose] for pose in stroke]
        for stroke in robot_strokes
    ]
    skeleton_data = [
        [[round(pose[0], 4), round(pose[1], 4), round(pose[2], 4)] for pose in stroke]
        for stroke in skeleton_strokes
    ]

    # Reach checking removed (reach.py was UR-specific). Kept the same
    # payload shape so the browser doesn't choke on a missing key.
    reach_flags, reach_out, reach_total = [], 0, len(robot_strokes)

    with state_lock:
        shared_state["strokes"] = robot_strokes
        shared_state["strokes_surface"] = surface_mode
        shared_state["last_mask"] = processed.mask
        shared_state["last_skeleton"] = processed.grooves
        shared_state["last_preview_png"] = None
        path_serial = shared_state["path_serial"] + 1
        shared_state["path_serial"] = path_serial
        shared_state["phase"]   = "captured" if robot_strokes else "editing"
        session_blend_mm = (shared_state.get("participant_exec_params") or {}).get(
            "blend_mm", MOVEP_BLEND_M * 1000.0)

    await server.send_capture_result(
        ws,
        success=True,
        stroke_count=extracted.total_strokes,
        point_count=extracted.total_points,
        strokes_data=strokes_data,
        reach_flags=reach_flags,
        reach_out=reach_out,
        skeleton_data=skeleton_data,
        exec_viz={
            "blend_m": session_blend_mm / 1000.0,
            "reach_m": UR_REACH_M,
            "min_reach_m": UR_MIN_REACH_M,
            "spacing_mm": spacing_mm,
            "join_mm": join_mm,
        },
        path_serial=path_serial,
    )
    module_trace.log(
        "generate",
        f"Generated path: {extracted.total_strokes} strokes, {extracted.total_points} points",
        extra=("surface" if surface_mode else "workspace",),
    )


async def on_retake(ws) -> None:
    with state_lock:
        shared_state["captured_still"] = None
        shared_state["still_dims"]     = None
        shared_state["strokes"]        = []
        shared_state["phase"]          = "previewing"
    if not camera_thread.running:
        camera_thread.start()
    module_trace.log("capture", "Retake — back to live preview")


# ── Robot execution -- STUBS ONLY (no PathExecutor, no robot to run on) ──────
async def on_run(ws, params: dict | None = None) -> None:
    await server.send_capture_result(ws, False, error="Robot execution is not available in this repo -- see Charlotte's pipeline.")


async def on_cancel(ws) -> None:
    pass


async def on_preview_image(params: dict) -> None:
    params = params or {}
    image = params.get("image")
    try:
        serial = int(params.get("serial", -1))
    except (TypeError, ValueError):
        return
    if not is_png_data_url(image):
        return
    with state_lock:
        if serial != shared_state.get("path_serial"):
            return
        shared_state["last_preview_png"] = image
    module_trace.log("save", "[participant] 3D preview received from a Developer window",
                     extra=("server",))


async def on_save_path(ws, params: dict) -> None:
    """Save the toolpath as JSON (+ preview/mask/skeleton images), then send
    to Charlotte if a tile is set."""
    from datetime import datetime

    with state_lock:
        strokes      = shared_state.get("strokes", [])
        surface_mode = shared_state.get("strokes_surface", False)
        surface_info = shared_state.get("surface_info")
        surface_pose = shared_state.get("surface_pose")
        mask         = shared_state.get("last_mask")
        skeleton     = shared_state.get("last_skeleton")
        pushed_png   = shared_state.get("last_preview_png")

    if not strokes:
        await server.send_save_result(ws, False, error="No path to save — Generate Path first.")
        return

    params = params or {}

    def _num(key, default, lo, hi):
        try:
            return min(max(float(params.get(key, default)), lo), hi)
        except (TypeError, ValueError):
            return default

    speed_pct = _num("speed_pct", (DRAW_SPEED / MAX_TCP_SPEED) * 100.0, 1.0, 100.0)
    offset_mm = _num("offset_mm", 0.0, -20.0, 200.0)
    safety_mm = _num("safety_mm", TRAVEL_Z * 1000.0, 5.0, 300.0)
    blend_mm  = _num("blend_mm", MOVEP_BLEND_M * 1000.0, 0.0, 5.0)
    speed = (speed_pct / 100.0) * MAX_TCP_SPEED

    with state_lock:
        shared_state["participant_exec_params"] = {
            "speed_pct": speed_pct, "offset_mm": offset_mm,
            "safety_mm": safety_mm, "blend_mm": blend_mm}

    meta = {
        "saved": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "mode": "surface" if surface_mode else "planar",
        "surface_name": (surface_info or {}).get("name") if surface_info else None,
        "surface": surface_info,
        "surface_pose": surface_pose if surface_mode else None,
        "speed_mps": round(speed, 4),
        "speed_pct": round(speed_pct, 1),
        "offset_mm": round(offset_mm, 2),
        "safety_mm": round(safety_mm, 1),
        "blend_mm": round(blend_mm, 2),
        "stroke_count": len(strokes),
        "point_count": sum(len(s) for s in strokes),
    }

    preview_png = params.get("image") or pushed_png

    loop = asyncio.get_running_loop()
    try:
        folder = await loop.run_in_executor(
            None,
            functools.partial(
                save_bundle, strokes, speed, safety_mm / 1000.0, offset_mm / 1000.0,
                meta, preview_png, blend_m=blend_mm / 1000.0,
                mask=mask, skeleton=skeleton,
            ),
        )
    except Exception as exc:
        await server.send_save_result(ws, False, error=str(exc))
        return

    await server.send_save_result(ws, True, folder=str(folder))
    module_trace.log("save", f"[save] toolpath saved to {folder}")

    with state_lock:
        tile_id = shared_state.get("next_tile_id")
    if tile_id is not None:
        await loop.run_in_executor(None, functools.partial(send_path_to_charlotte, folder, tile_id))
        module_trace.log("save", f"[zmq_bridge] sent tile {tile_id} to Charlotte")
    else:
        module_trace.log("save", "[zmq_bridge] no tile_id yet -- not sent")


# ── Participant Mode (automated pipeline) ────────────────────────────────────
def _sync_participant_state() -> None:
    with state_lock:
        shared_state["participant_status"] = automation.status
        shared_state["participant_msg"] = automation.message


_TRIGGER_HINT = "Enter a trigger distance (mm) to arm."


def _update_trigger_hint() -> None:
    if automation.busy:
        return
    with state_lock:
        mm = shared_state.get("trigger_mm")
    if automation.enabled and mm is None:
        automation.message = _TRIGGER_HINT
    elif automation.message == _TRIGGER_HINT:
        automation.message = ""


def _manual_locked(ws) -> bool:
    with state_lock:
        auto = bool(shared_state.get("auto_on"))
    return auto and ws is not server.broadcast_ws()


_AUTO_LOCK_MSG = "Automation is ON — switch it off in the Participant window for manual control."


async def on_set_automation(params: dict) -> None:
    on = bool((params or {}).get("on"))
    with state_lock:
        shared_state["auto_on"] = on
    automation.set_enabled(on)
    _update_trigger_hint()
    _sync_participant_state()
    module_trace.log("participant", f"[participant] automation {'ON' if on else 'OFF'}")


async def on_set_exec_params(params: dict) -> None:
    params = params or {}

    def _num(key, default, lo, hi):
        try:
            return min(max(float(params.get(key, default)), lo), hi)
        except (TypeError, ValueError):
            return default

    speed_pct = _num("speed_pct", (DRAW_SPEED / MAX_TCP_SPEED) * 100.0, 1.0, 100.0)
    offset_mm = _num("offset_mm", 0.0, -20.0, 200.0)
    safety_mm = _num("safety_mm", TRAVEL_Z * 1000.0, 5.0, 300.0)
    blend_mm  = _num("blend_mm", MOVEP_BLEND_M * 1000.0, 0.0, 5.0)
    spacing_mm = _num("spacing_mm", RESAMPLE_SPACING_MM,
                      RESAMPLE_SPACING_MIN_MM, RESAMPLE_SPACING_MAX_MM)
    join_mm    = _num("join_mm", JOIN_DISTANCE_MM,
                      JOIN_DISTANCE_MIN_MM, JOIN_DISTANCE_MAX_MM)
    with state_lock:
        shared_state["participant_exec_params"] = {
            "speed_pct": speed_pct, "offset_mm": offset_mm,
            "safety_mm": safety_mm, "blend_mm": blend_mm}
        shared_state["participant_gen_params"]["spacing_mm"] = spacing_mm
        shared_state["participant_gen_params"]["join_mm"]    = join_mm


async def on_set_trigger(params: dict) -> None:
    raw = (params or {}).get("threshold_mm")
    mm = None
    try:
        if raw is not None and str(raw).strip() != "":
            mm = min(max(float(raw), 50.0), 5000.0)
    except (TypeError, ValueError):
        mm = None
    camera_thread.set_trigger_threshold(mm)
    with state_lock:
        shared_state["trigger_mm"] = mm
        if mm is None:
            shared_state["trigger_below"] = None
    _update_trigger_hint()
    _sync_participant_state()
    module_trace.log("participant", f"[participant] trigger {'set to %.0f mm' % mm if mm is not None else 'off'}", extra=("camera_thread",))


async def _participant_pipeline() -> None:
    """One automated run: Sensing → Generating Paths → profanity guard →
    save + send. A rejected drawing stops at the guard."""
    bws = server.broadcast_ws()
    try:
        with state_lock:
            ready = (shared_state.get("surface_model") is not None
                     or shared_state.get("workspace") is not None)
        if not ready:
            automation.finish("Not ready — load a target surface (or Test Mode) in Developer Mode.")
            return

        _sync_participant_state()
        await asyncio.sleep(DEPTH_AVERAGE_FRAMES / DEPTH_FPS + 0.3)
        await on_capture_image(bws)
        with state_lock:
            captured = shared_state.get("captured_still") is not None
        if not captured:
            automation.finish("Capture failed — no depth frame.")
            return

        automation.stage("Generating Paths")
        module_trace.log("generate", "[participant] stage: Generating Paths")
        _sync_participant_state()
        with state_lock:
            gen_params = dict(shared_state.get("participant_gen_params") or {})
        await on_generate_path(bws, gen_params)
        with state_lock:
            strokes = shared_state.get("strokes", [])
            mask = shared_state.get("last_mask")
        if not strokes:
            automation.finish("No grooves detected — nothing to draw.")
            return

        if PROFANITY_CHECK_ENABLED and mask is not None:
            loop = asyncio.get_running_loop()
            verdict = await loop.run_in_executor(None, text_guard.check_mask, mask)
            if verdict.profane:
                automation.reject("Drawing rejected — please rake it over and try again.")
                module_trace.log(
                    "guard",
                    f"[participant] REJECTED: {verdict.reason} | OCR read {verdict.text!r}",
                    extra=("automation",))
                return
            if not verdict.available:
                module_trace.log("guard",
                                 f"[participant] profanity guard skipped: {verdict.reason}")

        automation.stage("Actuating")
        module_trace.log("run", "[participant] stage: Actuating", extra=("automation",))
        _sync_participant_state()
        with state_lock:
            exec_params = dict(shared_state.get("participant_exec_params") or {})
        await on_save_path(bws, exec_params)
        automation.finish("Done — path saved and sent to Charlotte.")
    except Exception as exc:
        automation.finish(f"Automation error: {exc}")
        module_trace.log("participant", f"[participant] pipeline error: {exc}")
    finally:
        _sync_participant_state()
        module_trace.log(
            "participant",
            f"[participant] {automation.message or 'pipeline finished'}")


async def _participant_loop() -> None:
    while True:
        await asyncio.sleep(PARTICIPANT_TICK_S)
        with state_lock:
            below = shared_state.get("trigger_below")
        prev = (automation.status, automation.message)
        if automation.tick(below):
            asyncio.create_task(_participant_pipeline())
        if (automation.status, automation.message) != prev:
            _sync_participant_state()


# ── Entry point ───────────────────────────────────────────────────────────────
server = Server(
    shared_state,
    state_lock,
    None,  # was `robot` -- no RobotController in this repo; see stubs above
    on_connect=on_robot_connect,
    on_disconnect=on_robot_disconnect,
    on_last_disconnect=on_last_client_disconnect,
    on_simulate_workspace=on_simulate_workspace,
    on_capture_image=on_capture_image,
    on_preview_adjust=on_preview_adjust,
    on_generate_path=on_generate_path,
    on_retake=on_retake,
    on_run=on_run,
    on_cancel=on_cancel,
    on_save_path=on_save_path,
    on_set_groove_params=on_set_groove_params,
    on_set_reference=on_set_reference,
    on_clear_reference=on_clear_reference,
    on_surface_upload=on_surface_upload,
    on_set_surface_pose=on_set_surface_pose,
    on_clear_surface=on_clear_surface,
    on_remove_surface=on_remove_surface,
    on_depth_overlay_params=on_depth_overlay_params,
    on_register_freedrive=on_register_freedrive,
    on_register_corner=on_register_corner,
    on_set_trigger=on_set_trigger,
    on_set_automation=on_set_automation,
    on_set_exec_params=on_set_exec_params,
    on_preview_image=on_preview_image,
)

# Camera starts immediately so both MJPEG feeds are live from the moment you open the browser
camera_thread.start()

tile_receiver = TileReceiver(shared_state, state_lock)
tile_receiver.start()


async def _open_browser() -> None:
    await asyncio.sleep(1.0)
    webbrowser.open(f"http://{HTTP_HOST}:{HTTP_PORT}")


async def _main() -> None:
    module_trace.print_banner()
    asyncio.create_task(_open_browser())
    asyncio.create_task(_participant_loop())
    asyncio.create_task(_tile_switch_watcher())
    await server.start()


if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        print("\nShutting down.")
        camera_thread.stop()