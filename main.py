import asyncio
import functools
import math
import random
import os
import signal
import sys
import threading
import time
import webbrowser

# Force UTF-8 console output so Unicode in log messages (→, ×, …) never crashes
# the program on Windows consoles that default to a legacy code page (cp1252).
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8")
    except (AttributeError, ValueError):
        pass

from config import (
    HTTP_HOST, HTTP_PORT, CONTOUR_MIN_PIXELS,
    DEPTH_LABELS_INTERVAL_MM, DEPTH_WIDTH, DEPTH_HEIGHT,
    DEPTH_FPS, DEPTH_AVERAGE_FRAMES, SURFACE_DIR,
    DRAW_Z, DRAW_SPEED, TRAVEL_Z, MAX_TCP_SPEED,
    RESAMPLE_SPACING_MM, RESAMPLE_SPACING_MIN_MM, RESAMPLE_SPACING_MAX_MM,
    JOIN_DISTANCE_MM, JOIN_DISTANCE_MIN_MM, JOIN_DISTANCE_MAX_MM,
    MAX_PATH_LENGTH_MM, MAX_PATH_LENGTH_MIN_MM, MAX_PATH_LENGTH_MAX_MM,
    BLEND_ZONE_M, BLEND_ZONE_MAX_MM,
    PARTICIPANT_TICK_S, PARTICIPANT_CLEAR_S, TRIGGER_MIN_MM, TRIGGER_MAX_MM,
    PARTICIPANT_MAX_DRAW_MIN, PARTICIPANT_MAX_DRAW_MIN_MIN,
    PARTICIPANT_MAX_DRAW_MAX_MIN,
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
from path_length import exceeds_limit, total_length_mm
from server import Server
from settings import load_settings, save_settings
from surface import SurfaceModel, SurfacePose, SurfaceScene
from view_rotation import norm_deg, rotate_crop, rotate_image
from workspace import WorkspaceConfig, scene_mm_per_px
from zmq_bridge import send_path_to_charlotte, TileReceiver

# ── Shared state ────────────────────────────────────────────────────────────
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
    # True when those mm are HEIGHT ABOVE THE SAND (a reference is set) rather
    # than distance from the camera. Same switch the trigger uses, so the popup
    # can name the units the operator is typing into the trigger box.
    "depth_labels_relative": False,
    "last_depth_crop_jpg": None,     # cropped colorized depth for the popup
    # Size of the COMBINED camera canvas (every view is the stitched rig) and
    # how many cameras feed it. Published by the camera thread once the canvas
    # geometry is frozen; None until then.
    "frame_size":          None,     # [width, height] px of the combined canvas
    "camera_count":        0,        # cameras combined into it
    # Quarter-turn rotation of that canvas (Developer Mode's ⟳ button). Applied
    # in the camera thread, so every view and the captured still inherit it.
    # Persisted in settings.json — it describes how the rig is mounted.
    "view_rotation":       0,        # 0/90/180/270, clockwise
    "workspace":           None,     # WorkspaceConfig | None — confirmed workspace
    "pending_workspace":   None,     # WorkspaceConfig | None — loaded from disk
    "ws_points":           {"p0": None, "px": None, "py": None},
    "freedrive":           False,
    "ee":                  [0.0] * 6,
    "phase":               "idle",     # idle|previewing|editing|captured|executing|done|error
    "captured_still":      None,       # (depth_m, valid, rgb) — frozen averaged depth + colour
    "reference_depth":     None,       # baseline depth frame for background subtraction
    "surface_model":       None,       # SurfaceModel | None — target mesh for 3D projection
    "surface_info":        None,       # dict for the browser (name/faces/bbox)
    "surface_pose":        SurfacePose().to_dict(),   # placement in robot base frame
    "surface_offset_mm":   0.0,        # TCP offset along the surface normal
    "surface_mesh_payload": None,      # local-frame vertices/faces for the 3D preview
    "strokes_surface":     False,      # True when current strokes were surface-projected
    "still_dims":          None,       # (width, height) of the captured still
    "strokes":             [],         # robot-space strokes after Generate Path
    # Detection images from the last Generate Path, saved into the bundle and
    # (mask only) OCR'd by the Participant-Mode profanity guard.
    "last_mask":           None,       # thick detected-region mask, cropped
    "last_skeleton":       None,       # 1-px groove centrelines, cropped
    # 3D-preview screenshot pushed up by a Developer window (Participant Mode
    # has nobody to click Save). Paired with the generate that produced it.
    "last_preview_png":    None,       # PNG data URL | None
    "path_serial":         0,          # bumped by every Generate Path
    # Drawn length of the current path (mm, corner zone applied) and the ceiling
    # the Max Total Length box sets. Over it, Run/Save refuse and Participant
    # Mode calls the drawing Invalid. 0 = no limit.
    "path_length_mm":      0.0,
    "max_length_mm":       MAX_PATH_LENGTH_MM,
    "executing":           False,
    "progress":            0.0,
    # ── Participant Mode (automated pipeline) ──
    "auto_on":             False,      # Participant popup Auto toggle
    "trigger_mm":          None,       # trigger distance (mm from camera); None = off
    "trigger_below":       None,       # camera thread: something closer than trigger_mm?
    # Max Drawing Time: minutes one participant may occupy the sand (hand in →
    # hand out). None = off. remaining_s is what the popup counts down.
    "max_draw_min":        PARTICIPANT_MAX_DRAW_MIN or None,
    "participant_remaining_s": None,
    "participant_status":  "Auto Off", # Auto Off|Auto On|Alerted|Sensing|Generating Paths|Actuating|Invalid
    "participant_msg":     "",         # last automation outcome/message
    "participant_gen_params":  {},     # last crop/adjustments/spacing from Developer Mode
    "participant_exec_params": {},     # last speed_pct/offset_mm/safety_mm from Developer Mode
    "next_tile_id": None,   # set by zmq_bridge.TileReceiver; the tile Charlotte wants next
}
state_lock = threading.Lock()

EMULATE_CAPTURE = True  # True = pretend a drawing was captured (fake stroke), False = use the real RealSense camera. Manual choice — independent of whether a camera is physically attached.

# ── Singletons ──────────────────────────────────────────────────────────────
camera_thread = DepthCameraThread(shared_state, state_lock)
automation    = ParticipantAutomation(
    clear_ticks=round(PARTICIPANT_CLEAR_S / PARTICIPANT_TICK_S),
    max_draw_s=(PARTICIPANT_MAX_DRAW_MIN or 0.0) * 60.0)


async def on_last_client_disconnect() -> None:
    print("Last client disconnected — stopping camera.")
    camera_thread.stop()
    os.kill(os.getpid(), signal.SIGINT)


# ── Workspace setup callbacks ─────────────────────────────────────────────────
# The interactive 3-point (P0/Px/Py) freedrive calibration was removed: the
# drawing target is now always a surface mesh loaded from file (or the Test
# Mode synthetic workspace). WorkspaceConfig remains for the planar fallback.
async def on_simulate_workspace() -> None:
    """
    Set a synthetic workspace so the depth→groove→Path-Preview pipeline can be
    tested without a robot. Does not connect or move the robot; Run remains
    gated on a real connection.
    """
    # The synthetic plane takes the COMBINED canvas's aspect, so a Test-Mode
    # preview of a wide multi-camera view stays isotropic instead of being
    # squashed into one camera's 4:3.
    size = _frame_size()
    ws_cfg = WorkspaceConfig.simulation(size[0] / size[1] if size else None)
    with state_lock:
        shared_state["workspace"]         = ws_cfg
        shared_state["pending_workspace"] = None
        shared_state["ws_points"]         = {"p0": None, "px": None, "py": None}
        shared_state["phase"]             = "previewing"
    if not camera_thread.running:
        camera_thread.start()
    module_trace.log(
        "generate",
        f"Simulation workspace active (no robot): "
        f"{ws_cfg.x_extent:.3f} m × {ws_cfg.y_extent:.3f} m — Capture enabled.",
        extra=("workspace",),
    )


# A trigger above this many mm was almost certainly typed as a distance from the
# camera: no hand hovers half a metre over the sand. Used only to warn when a
# reference is captured and the number's meaning changes under it.
_ABSOLUTE_LOOKING_MM = 300.0


# ── Capture image / Edit / Generate path callbacks ──────────────────────────────
def _frame_size() -> tuple[int, int] | None:
    """
    Size of the frame the pipeline is working from — the COMBINED camera canvas.

    The captured still wins when there is one, so a crop/generate keeps using
    the frame it was taken on; otherwise the camera thread's live canvas. Both
    are needed because the mm scale and the surface fit depend on it, and it is
    no longer the fixed 640×480 of a single camera.
    """
    with state_lock:
        dims = shared_state.get("still_dims")
    return tuple(dims) if dims else camera_thread.frame_size


def _mm_per_px(workspace, surface_model=None, frame_size=None) -> float | None:
    """Scene scale for the mm-based filters/spacings — see workspace.scene_mm_per_px."""
    return scene_mm_per_px(workspace, surface_model, frame_size or _frame_size())


async def on_set_groove_params(params: dict) -> None:
    """Push the latest Detect-Grooves params + crop to the camera thread so the
    LIVE depth/groove feeds update in real time (used before an image is captured).
    The crop restricts the live groove/mask preview to the selected region."""
    gp = DepthGrooveParams.from_dict(params.get("adjustments"))
    crop = Crop.from_dict(params.get("crop"))
    with state_lock:
        workspace = shared_state.get("workspace")
        surface_model = shared_state.get("surface_model")
        # Participant Mode reuses the latest Developer-Mode detection settings.
        shared_state["participant_gen_params"].update(
            {"crop": params.get("crop"), "adjustments": params.get("adjustments")})
    camera_thread.set_live_params(gp)
    camera_thread.set_live_crop(crop)
    camera_thread.set_scale(_mm_per_px(workspace, surface_model))


async def on_depth_overlay_params(params: dict) -> None:
    """Region band width (mm) for the /depths depth-number overlay popup."""
    try:
        interval = float(params.get("interval_mm", DEPTH_LABELS_INTERVAL_MM))
    except (TypeError, ValueError):
        interval = DEPTH_LABELS_INTERVAL_MM
    camera_thread.set_depth_label_interval(min(max(interval, 1.0), 100.0))


async def on_set_reference(ws) -> None:
    """
    Capture the current (undrawn) sand as a baseline.

    The reference does two jobs. It cancels pre-existing natural grooves for
    detection (`ref_strength`), and — because a picture of the empty sand
    CONTAINS the camera's tilt — it turns "distance from the camera" into
    "height above the sand" for the Participant trigger and the near-object
    cutoff. That second job is what makes a tilted rig workable at all, so
    setting one silently changes what those two numbers mean; say so, and warn
    when the existing trigger value was plainly tuned as a camera distance.
    """
    captured = camera_thread.capture_frame()
    if captured is None:
        await server.send_reference_status(ws, False, "No depth frame to set as reference.")
        return
    depth_m, _valid, _rgb = captured
    with state_lock:
        shared_state["reference_depth"] = depth_m
        trigger_mm = shared_state.get("trigger_mm")
    camera_thread.set_reference(depth_m)
    msg = ("Reference captured — natural grooves can be subtracted, and the "
           "Participant trigger + near-object cutoff now measure HEIGHT ABOVE "
           "THE SAND (tilt-proof) instead of distance from the camera.")
    if trigger_mm is not None and trigger_mm > _ABSOLUTE_LOOKING_MM:
        msg += (f" Your trigger is still {trigger_mm:.0f} mm — that reads as a "
                "camera distance; as a height above the sand try something "
                "like 60–120 mm.")
    await server.send_reference_status(ws, True, msg)
    module_trace.log("reference",
                     "Reference depth captured — background subtraction on, "
                     "trigger + near-object cutoff now relative to the sand.")


async def on_clear_reference(ws) -> None:
    with state_lock:
        shared_state["reference_depth"] = None
    camera_thread.set_reference(None)
    await server.send_reference_status(
        ws, False,
        "Reference cleared — the trigger and near-object cutoff are absolute "
        "distances from the camera again.")
    module_trace.log("reference", "Reference depth cleared.")


# ── Target surface (3D projection) callbacks ─────────────────────────────────────
# Loading and removing are read-modify-write on the scene (read the current
# parts, rebuild, store) with awaits in between, so they are serialized: two
# clients uploading at once must not drop a part.
_surface_lock = asyncio.Lock()


async def on_surface_upload(filename: str, blob: bytes) -> dict:
    """
    Save an uploaded STL/OBJ, ADD it to the surface scene, and broadcast the
    combined mesh + status. Loading is cumulative: each file keeps the position
    authored in it, so surfaces exported from one Rhino document assemble
    themselves. Re-loading the same file name replaces that part in place.
    """
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
            # A surface replaces the flat workspace for mapping, so it also
            # unlocks the capture flow — no P0/Px/Py calibration in surface mode.
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
    successive emulated captures aren't identical."""
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

    with state_lock:
        shared_state["strokes"] = robot_strokes
        shared_state["strokes_surface"] = True
        shared_state["path_serial"] = shared_state.get("path_serial", 0) + 1

    module_trace.log("surface", f"[emulate] tile {tile_id}: {len(robot_strokes[0])} points -- saving")
    await on_save_path(server.broadcast_ws(), {})


async def _tile_switch_watcher() -> None:
    last_seq = None
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
    """Drop ONE loaded surface from the scene (the ✕ next to it in the list)."""
    try:
        idx = int((params or {}).get("index", -1))
    except (TypeError, ValueError):
        idx = -1
    async with _surface_lock:
        with state_lock:
            scene = shared_state.get("surface_model")
        # Explicit bounds check: a missing/garbled index must be a no-op, and
        # Python would happily read parts[-1] as "the last one".
        if not isinstance(scene, SurfaceScene) or not 0 <= idx < len(scene.parts):
            return
        removed = scene.parts[idx].name
        new_scene = scene.without_part(idx)
        if new_scene is None:                   # last one gone → back to flat
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
    """Update the surface placement (base frame) and TCP normal-offset."""
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
    """Remove every loaded surface (the whole scene)."""
    with state_lock:
        shared_state["surface_model"] = None
        shared_state["surface_info"] = None
        shared_state["surface_mesh_payload"] = None
        shared_state["strokes_surface"] = False
    await server.broadcast_surface_status(loaded=False, message="Surfaces cleared.")
    module_trace.log("surface", "[surface] cleared — paths map to the flat workspace again", extra=("workspace",))


async def on_capture_image(ws) -> None:
    """Freeze a temporally averaged depth (+ colour) frame and enter editing."""
    if _manual_locked(ws):
        await server.send_capture_result(ws, False, error=_AUTO_LOCK_MSG)
        return
    # If a projector is running, blank it and wait for the rolling depth buffer
    # to refill with projector-free frames — the average uses the PAST second,
    # so blanking without the wait would not help.
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
    """Reprocess the captured depth with the latest crop/groove params → preview."""
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

    # Crop the captured RGB to the same region so the RGB view shows only the crop.
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
    mmpp = _mm_per_px(workspace, surface_model, (width, height))

    # Waypoint spacing in mm (Path Preview slider), clamped to the allowed range.
    try:
        spacing_mm = float(params.get("spacing_mm", RESAMPLE_SPACING_MM))
    except (TypeError, ValueError):
        spacing_mm = RESAMPLE_SPACING_MM
    spacing_mm = min(max(spacing_mm, RESAMPLE_SPACING_MIN_MM), RESAMPLE_SPACING_MAX_MM)

    # Endpoint-join distance in mm (Path Preview "Distance Threshold" box);
    # 0 = off. Strokes whose ends fall within it merge into one toolpath.
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
        # Project the 2D drawing onto the 3D surface: waypoints lie on the mesh,
        # TCP perpendicular to it, offset applied along the surface normal.
        # The dense skeleton is projected at ZERO offset so the white preview
        # line lies exactly on the surface regardless of TCP offset.
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

    # Convert to serialisable list-of-lists for the browser's 3D preview
    strokes_data = [
        [[round(v, 5) for v in pose] for pose in stroke]
        for stroke in robot_strokes
    ]
    # Skeleton needs positions only (white line) — drop orientations, round 4dp.
    skeleton_data = [
        [[round(pose[0], 4), round(pose[1], 4), round(pose[2], 4)] for pose in stroke]
        for stroke in skeleton_strokes
    ]

    with state_lock:
        shared_state["strokes"] = robot_strokes
        shared_state["strokes_surface"] = surface_mode
        # Both go into the saved bundle; the mask is additionally what the
        # Participant-Mode profanity guard OCRs (never the skeleton — 1-px
        # hairlines read terribly). Replaced in lockstep with `strokes`, so a
        # save can never pick up images from a different generate.
        shared_state["last_mask"] = processed.mask
        shared_state["last_skeleton"] = processed.grooves
        # The 3D preview belongs to the OLD path until a browser sends a new
        # shot; drop it now so a stale picture can never reach the bundle.
        shared_state["last_preview_png"] = None
        path_serial = shared_state["path_serial"] + 1
        shared_state["path_serial"] = path_serial
        shared_state["phase"]   = "captured" if robot_strokes else "editing"
        session_blend_mm = (shared_state.get("participant_exec_params") or {}).get(
            "blend_mm", BLEND_ZONE_M * 1000.0)
        max_length_mm = shared_state.get("max_length_mm", MAX_PATH_LENGTH_MM)

    # How far the tool will travel while drawing. Measured on the projected
    # robot-space strokes, so spacing, joining and the surface's real scale are
    # already in the number; the corner zone is applied on top.
    length_mm = total_length_mm(robot_strokes, session_blend_mm)
    with state_lock:
        shared_state["path_length_mm"] = length_mm

    await server.send_capture_result(
        ws,
        success=True,
        stroke_count=extracted.total_strokes,
        point_count=extracted.total_points,
        strokes_data=strokes_data,
        skeleton_data=skeleton_data,
        # Everything the browser needs to rebuild the toolpath preview
        # client-side when the exec-bar Offset/Safety inputs change.
        exec_viz={
            "blend_m": session_blend_mm / 1000.0,
            "spacing_mm": spacing_mm,
            "join_mm": join_mm,
        },
        path_serial=path_serial,
        length_mm=length_mm,
        max_length_mm=max_length_mm,
    )
    # Mapping owner depends on the run: a loaded mesh uses surface.py, Test Mode
    # falls back to workspace.py — show whichever actually ran.
    module_trace.log(
        "generate",
        f"Generated path: {extracted.total_strokes} strokes, {extracted.total_points} points",
        extra=("surface" if surface_mode else "workspace",),
    )


async def on_retake(ws) -> None:
    """Discard the captured still and return to the live preview phase."""
    with state_lock:
        shared_state["captured_still"] = None
        shared_state["still_dims"]     = None
        shared_state["strokes"]        = []
        shared_state["phase"]          = "previewing"
    if not camera_thread.running:
        camera_thread.start()
    module_trace.log("capture", "Retake — back to live preview")


async def on_rotate_view(ws, params: dict | None = None) -> None:
    """
    Turn the whole combined canvas a quarter turn (Developer Mode's ⟳ button).

    The camera thread turns the canvas itself, so every view follows for free.
    What has to be re-based here is the state that was recorded in the OLD
    orientation and would otherwise silently point at the wrong sand:

      * the crop — turned with the picture, so it keeps framing the same region
        (it is the server's copy that Participant Mode and a reopened window
        read, so turning it here is what keeps every client agreeing);
      * the reference frame — a full-canvas array, turned by the same amount so
        background subtraction still lines up pixel for pixel;
      * the workspace — a quarter turn swaps the canvas's width and height, and
        Test Mode's plane takes its shape from that aspect;
      * the captured still and its path — these CANNOT be re-based (the strokes
        are already projected into robot space), so they are dropped, exactly
        like Retake. Rotation is a framing adjustment: turn first, then capture.

    Refused while Participant Auto is ON — re-aiming the camera mid-session
    would change what a participant's drawing means halfway through it.
    """
    if _manual_locked(ws):
        await server.send_capture_result(ws, False, error=_AUTO_LOCK_MSG)
        return
    try:
        steps = int((params or {}).get("steps", 1))
    except (TypeError, ValueError):
        steps = 1

    with state_lock:
        old = shared_state.get("view_rotation", 0)
        reference = shared_state.get("reference_depth")
        gen_params = shared_state.get("participant_gen_params") or {}
        crop_dict = gen_params.get("crop")
        workspace = shared_state.get("workspace")
    new = norm_deg(old + 90 * steps)
    delta = norm_deg(new - old)

    reference = rotate_image(reference, delta)
    crop_dict = rotate_crop(crop_dict, delta)

    with state_lock:
        shared_state["view_rotation"] = new
        shared_state["reference_depth"] = reference
        shared_state["participant_gen_params"]["crop"] = crop_dict
        # Same reset as Retake, plus the detection images the path was made of.
        shared_state["captured_still"] = None
        shared_state["still_dims"]     = None
        shared_state["strokes"]        = []
        shared_state["last_mask"]      = None
        shared_state["last_skeleton"]  = None
        shared_state["path_length_mm"] = 0.0
        if shared_state["phase"] in ("editing", "captured", "done"):
            shared_state["phase"] = "previewing"

    camera_thread.set_view_rotation(new)
    camera_thread.set_reference(reference)
    camera_thread.set_live_crop(Crop.from_dict(crop_dict))

    # frame_size is now the turned one, so re-derive anything shaped by it.
    size = _frame_size()
    if workspace is not None and size:
        workspace = workspace.with_frame_aspect(size[0] / size[1])
        with state_lock:
            shared_state["workspace"] = workspace
    with state_lock:
        surface_model = shared_state.get("surface_model")
    camera_thread.set_scale(_mm_per_px(workspace, surface_model, size))

    save_settings({"view_rotation": new})
    await server.broadcast_view_rotation(new, crop_dict)
    where = f" — canvas now {size[0]}×{size[1]} px" if size else ""
    module_trace.log(
        "rotate",
        f"View rotated to {new}°{where}; crop and reference turned with it, "
        "captured still dropped — capture again",
    )


def _length_check(strokes, blend_mm: float, params: dict) -> tuple[bool, str]:
    """
    Enforce the Max Total Length box. Returns ``(ok, message)``.

    The limit is read from the request when it carries one and from shared state
    otherwise, so a Developer window's box and Participant Mode's synced copy
    both land here. The length is recomputed from the strokes about to be used
    with the blend about to be used — not the value cached at Generate time —
    because Radius changes the drawn length and never triggers a regenerate.
    """
    limit = params.get("max_length_mm")
    if limit is None:
        with state_lock:
            limit = shared_state.get("max_length_mm", MAX_PATH_LENGTH_MM)
    try:
        limit = min(max(float(limit), MAX_PATH_LENGTH_MIN_MM), MAX_PATH_LENGTH_MAX_MM)
    except (TypeError, ValueError):
        limit = MAX_PATH_LENGTH_MM

    over, actual = exceeds_limit(strokes, blend_mm, limit)
    with state_lock:
        shared_state["path_length_mm"] = actual
        shared_state["max_length_mm"] = limit
    if not over:
        return True, ""
    return False, (f"Path too long: {actual:.0f} mm of drawing exceeds the "
                   f"{limit:.0f} mm Max Total Length. Rake a smaller drawing, "
                   f"or raise the limit.")


async def on_preview_image(params: dict) -> None:
    """
    A Developer window's screenshot of its 3D preview, sent unprompted after a
    Generate Path while Participant Mode is running.

    Participant Mode drives the pipeline server-side, so no Save click ever
    carries an image up and preview.png would be the one missing file in an
    automated bundle. The picture only exists in the browser (three.js draws it
    on the client's GPU), so the browser volunteers it here instead.

    Only kept if it is a real PNG of sane size AND belongs to the generate whose
    strokes are currently loaded — same lockstep rule as last_mask/last_skeleton.
    """
    params = params or {}
    image = params.get("image")
    try:
        serial = int(params.get("serial", -1))
    except (TypeError, ValueError):
        return
    if not is_png_data_url(image):
        return
    with state_lock:
        # A screenshot from an earlier generate is worse than none at all.
        if serial != shared_state.get("path_serial"):
            return
        shared_state["last_preview_png"] = image
    module_trace.log("save", "[participant] 3D preview received from a Developer window",
                     extra=("server",))


async def on_save_path(ws, params: dict) -> None:
    """Save the toolpath as JSON + preview/mask/skeleton images."""
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
    blend_mm  = _num("blend_mm", BLEND_ZONE_M * 1000.0, 0.0, BLEND_ZONE_MAX_MM)
    speed = (speed_pct / 100.0) * MAX_TCP_SPEED

    ok, why = _length_check(strokes, blend_mm, params)
    if not ok:
        await server.send_save_result(ws, False, error=why)
        module_trace.log("save", f"[save] refused — {why}")
        return

    with state_lock:
        length_mm = shared_state.get("path_length_mm", 0.0)
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
        # Drawn length of what was saved — the number the Max Total Length box
        # judges, recorded so a bundle says how much it lays down.
        "path_length_mm": round(length_mm, 1),
        "stroke_count": len(strokes),
        "point_count": sum(len(s) for s in strokes),
    }

    # Developer Mode: the Save click carries the screenshot. Participant Mode:
    # nobody clicked, so fall back to whatever a Developer window pushed up for
    # this same path. Neither available (no Developer window open) → no
    # preview.png, exactly as before; a save must never fail over a picture.
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
# The Auto toggle + trigger threshold come from the ⧉ Participant popup; the
# camera thread flags frames with anything closer than the trigger
# (shared_state["trigger_below"]). The loop below feeds that flag to the state
# machine; on an Alerted→clear edge the pipeline runs, reusing the SAME
# handlers as the Developer-Mode buttons via server.broadcast_ws() — open
# Developer windows see every step live. While Auto is ON, manual
# capture/generate/run calls are refused (_manual_locked) and greyed out.
def _sync_participant_state() -> None:
    with state_lock:
        shared_state["participant_status"] = automation.status
        shared_state["participant_msg"] = automation.message


_TRIGGER_HINT = "Enter a trigger distance (mm) to arm."


def _update_trigger_hint() -> None:
    """Auto ON without a trigger distance can never fire — say so in the popup.
    Only ever writes/clears the hint, so pipeline outcome messages survive."""
    if automation.busy:
        return
    with state_lock:
        mm = shared_state.get("trigger_mm")
    if automation.enabled and mm is None:
        automation.message = _TRIGGER_HINT
    elif automation.message == _TRIGGER_HINT:
        automation.message = ""


def _manual_locked(ws) -> bool:
    """True when Auto is ON and this pipeline call is NOT from the automation
    itself (which uses the broadcast shim) — manual controls are locked out."""
    with state_lock:
        auto = bool(shared_state.get("auto_on"))
    return auto and ws is not server.broadcast_ws()


_AUTO_LOCK_MSG = "Automation is ON — switch it off in the Participant window for manual control."


async def on_set_automation(params: dict) -> None:
    """Participant popup Auto toggle."""
    on = bool((params or {}).get("on"))
    with state_lock:
        shared_state["auto_on"] = on
    automation.set_enabled(on)
    _update_trigger_hint()
    _sync_participant_state()
    module_trace.log("participant", f"[participant] automation {'ON' if on else 'OFF'}")


async def on_set_exec_params(params: dict) -> None:
    """
    Live sync of the Developer exec-bar values (debounced by the browser) so
    Participant Mode always uses what Developer Mode currently shows — no Run
    or Save Path needed to 'commit' them. Same clamps as on_run/on_save_path.
    """
    params = params or {}

    def _num(key, default, lo, hi):
        try:
            return min(max(float(params.get(key, default)), lo), hi)
        except (TypeError, ValueError):
            return default

    speed_pct = _num("speed_pct", (DRAW_SPEED / MAX_TCP_SPEED) * 100.0, 1.0, 100.0)
    offset_mm = _num("offset_mm", 0.0, -20.0, 200.0)
    safety_mm = _num("safety_mm", TRAVEL_Z * 1000.0, 5.0, 300.0)
    blend_mm  = _num("blend_mm", BLEND_ZONE_M * 1000.0, 0.0, BLEND_ZONE_MAX_MM)
    spacing_mm = _num("spacing_mm", RESAMPLE_SPACING_MM,
                      RESAMPLE_SPACING_MIN_MM, RESAMPLE_SPACING_MAX_MM)
    join_mm    = _num("join_mm", JOIN_DISTANCE_MM,
                      JOIN_DISTANCE_MIN_MM, JOIN_DISTANCE_MAX_MM)
    max_length_mm = _num("max_length_mm", MAX_PATH_LENGTH_MM,
                         MAX_PATH_LENGTH_MIN_MM, MAX_PATH_LENGTH_MAX_MM)
    with state_lock:
        shared_state["participant_exec_params"] = {
            "speed_pct": speed_pct, "offset_mm": offset_mm,
            "safety_mm": safety_mm, "blend_mm": blend_mm}
        shared_state["participant_gen_params"]["spacing_mm"] = spacing_mm
        shared_state["participant_gen_params"]["join_mm"]    = join_mm
        shared_state["max_length_mm"] = max_length_mm
        strokes = shared_state.get("strokes", [])

    # Radius changes the drawn length but never regenerates the path, so the
    # readout would go stale on a slider drag. Recompute here — this is already
    # debounced by the browser, and it keeps ONE implementation of the maths.
    if strokes:
        length_mm = total_length_mm(strokes, blend_mm)
        with state_lock:
            shared_state["path_length_mm"] = length_mm


async def on_set_trigger(params: dict) -> None:
    """
    Set (or clear with null/empty) the Participant-Mode trigger threshold.

    The number is a height ABOVE THE SAND when a reference frame is set, and a
    raw distance from the camera when there is none — the camera thread picks
    the mode per frame, so nothing here has to know which is in force. One
    range spans both, hence the low floor.
    """
    raw = (params or {}).get("threshold_mm")
    mm = None
    try:
        if raw is not None and str(raw).strip() != "":
            mm = min(max(float(raw), TRIGGER_MIN_MM), TRIGGER_MAX_MM)
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


async def on_set_max_draw_time(params: dict) -> None:
    """
    Set (or clear with null/empty) the Participant-Mode Max Drawing Time, in
    MINUTES — how long one participant may keep the sand occupied, measured
    from the trigger tripping to the frame going clear. Running out is judged
    by the state machine itself (an Invalid verdict), not by the pipeline: the
    point is to stop a drawing that never ends, so it cannot wait for one.
    """
    raw = (params or {}).get("minutes")
    minutes = None
    try:
        if raw is not None and str(raw).strip() != "":
            value = float(raw)
            if value > 0:
                minutes = min(max(value, PARTICIPANT_MAX_DRAW_MIN_MIN),
                              PARTICIPANT_MAX_DRAW_MAX_MIN)
    except (TypeError, ValueError):
        minutes = None
    automation.set_max_draw_s(None if minutes is None else minutes * 60.0)
    with state_lock:
        shared_state["max_draw_min"] = minutes
        shared_state["participant_remaining_s"] = automation.remaining_s()
    module_trace.log(
        "participant",
        "[participant] max drawing time "
        + (f"set to {minutes:g} min" if minutes is not None else "off"),
        extra=("automation",))


async def _participant_pipeline() -> None:
    """One automated run: Sensing → Generating Paths → profanity guard →
    Actuating (save + run). A rejected drawing stops at the guard: status
    "Invalid", nothing saved, nothing sent to the robot."""
    bws = server.broadcast_ws()
    try:
        with state_lock:
            ready = (shared_state.get("surface_model") is not None
                     or shared_state.get("workspace") is not None)
        if not ready:
            automation.finish("Not ready — load a target surface (or Test Mode) in Developer Mode.")
            return

        # ── Sensing: the averaged capture uses the PAST second of frames, so
        # wait for the buffer to refill with hand-free frames first.
        _sync_participant_state()
        await asyncio.sleep(DEPTH_AVERAGE_FRAMES / DEPTH_FPS + 0.3)
        await on_capture_image(bws)
        with state_lock:
            captured = shared_state.get("captured_still") is not None
        if not captured:
            automation.finish("Capture failed — no depth frame.")
            return

        # ── Generating Paths: latest Developer-Mode detection/spacing settings.
        automation.stage("Generating Paths")
        module_trace.log("generate", "[participant] stage: Generating Paths")
        _sync_participant_state()
        with state_lock:
            gen_params = dict(shared_state.get("participant_gen_params") or {})
        await on_generate_path(bws, gen_params)
        with state_lock:
            strokes = shared_state.get("strokes", [])
            mask = shared_state.get("last_mask")
            exec_blend_mm = (shared_state.get("participant_exec_params") or {}).get(
                "blend_mm", BLEND_ZONE_M * 1000.0)
            max_length_mm = shared_state.get("max_length_mm", MAX_PATH_LENGTH_MM)
        if not strokes:
            automation.finish("No grooves detected — nothing to draw.")
            return

        # ── Max Total Length: too much drawing to lay down. Refused like a
        # profanity hit — Invalid, nothing saved, nothing run — because both
        # are "this drawing is not acceptable", not "something went wrong".
        over, actual_mm = exceeds_limit(strokes, exec_blend_mm, max_length_mm)
        with state_lock:
            shared_state["path_length_mm"] = actual_mm
        if over:
            automation.reject(
                f"Drawing too long ({actual_mm / 1000.0:.2f} m of "
                f"{max_length_mm / 1000.0:.2f} m allowed) — please rake it over "
                "and draw something smaller.")
            module_trace.log(
                "run",
                f"[participant] REJECTED: path {actual_mm:.0f} mm exceeds the "
                f"{max_length_mm:.0f} mm Max Total Length",
                extra=("automation",))
            return

        # ── Profanity guard: OCR the mask and stop here if it reads as
        # offensive text. Participant Mode only — in Developer Mode the
        # operator is present and decides. Nothing is saved and nothing is
        # sent to the robot, and the status stays "Invalid" so whoever drew
        # it sees the verdict.
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

        # ── Actuating: save the bundle — the robot-execution repo (rrc-dev)
        # picks it up from here via the tile hand-off / ZeroMQ bridge.
        automation.stage("Actuating")
        module_trace.log("run", "[participant] stage: Actuating", extra=("automation",))
        _sync_participant_state()
        with state_lock:
            exec_params = dict(shared_state.get("participant_exec_params") or {})
        await on_save_path(bws, exec_params)
        automation.finish("Done — path drawn and saved.")
    except Exception as exc:
        automation.finish(f"Automation error: {exc}")
        module_trace.log("participant", f"[participant] pipeline error: {exc}")
    finally:
        _sync_participant_state()
        module_trace.log(
            "participant",
            f"[participant] {automation.message or 'pipeline finished'}")


async def _participant_loop() -> None:
    """Poll the camera trigger flag and drive the automation state machine."""
    while True:
        await asyncio.sleep(PARTICIPANT_TICK_S)
        with state_lock:
            below = shared_state.get("trigger_below")
            executing = shared_state.get("executing", False)
            # The countdown the popup shows is the state machine's own clock,
            # republished each tick — one implementation, so what the
            # participant reads is exactly what judges the drawing.
            shared_state["participant_remaining_s"] = automation.remaining_s()
        # A manually-started run owns the robot — don't trigger on top of it.
        if executing and not automation.busy:
            continue
        prev = (automation.status, automation.message)
        if automation.tick(below):
            asyncio.create_task(_participant_pipeline())
        if (automation.status, automation.message) != prev:
            _sync_participant_state()


# ── Entry point ────────────────────────────────────────────────────────────────
server = Server(
    shared_state,
    state_lock,
    on_last_disconnect=on_last_client_disconnect,
    on_simulate_workspace=on_simulate_workspace,
    on_capture_image=on_capture_image,
    on_preview_adjust=on_preview_adjust,
    on_generate_path=on_generate_path,
    on_retake=on_retake,
    on_save_path=on_save_path,
    on_set_groove_params=on_set_groove_params,
    on_set_reference=on_set_reference,
    on_clear_reference=on_clear_reference,
    on_surface_upload=on_surface_upload,
    on_set_surface_pose=on_set_surface_pose,
    on_clear_surface=on_clear_surface,
    on_remove_surface=on_remove_surface,
    on_depth_overlay_params=on_depth_overlay_params,
    on_set_trigger=on_set_trigger,
    on_set_max_draw_time=on_set_max_draw_time,
    on_set_automation=on_set_automation,
    on_set_exec_params=on_set_exec_params,
    on_preview_image=on_preview_image,
    on_rotate_view=on_rotate_view,
)

# The view rotation describes how the rig is mounted, so it outlives a restart.
# Set BEFORE the thread starts: the canvas is published turned from the first
# frame, and frame_size is right the first time anything reads it.
_saved_rotation = norm_deg(load_settings().get("view_rotation", 0))
if _saved_rotation:
    shared_state["view_rotation"] = _saved_rotation
    camera_thread.set_view_rotation(_saved_rotation)
    print(f"[depth] view rotation {_saved_rotation}° restored from settings.json")

# Camera starts immediately so both MJPEG feeds are live from the moment you open the browser
camera_thread.start()

tile_receiver = TileReceiver(shared_state, state_lock)
tile_receiver.start()


async def _open_browser() -> None:
    await asyncio.sleep(1.0)
    webbrowser.open(f"http://{HTTP_HOST}:{HTTP_PORT}")


async def _main() -> None:
    # Printed after every module has been imported, so the ✓/· marks are real.
    module_trace.print_banner()
    asyncio.create_task(_participant_loop())
    asyncio.create_task(_tile_switch_watcher())
    await server.start()


if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        print("\nShutting down.")
        camera_thread.stop()