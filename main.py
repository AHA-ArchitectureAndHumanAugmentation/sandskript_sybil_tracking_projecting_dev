import asyncio
import functools
import os
import signal
import sys
import threading
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
    DEPTH_LABELS_INTERVAL_MM,
    DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FPS, DEPTH_AVERAGE_FRAMES, SURFACE_DIR,
    DRAW_Z, DRAW_SPEED, TRAVEL_Z, MAX_TCP_SPEED,
    RESAMPLE_SPACING_MM, RESAMPLE_SPACING_MIN_MM, RESAMPLE_SPACING_MAX_MM,
    JOIN_DISTANCE_MM, JOIN_DISTANCE_MIN_MM, JOIN_DISTANCE_MAX_MM,
    UR_REACH_M, UR_MIN_REACH_M, MOVEP_BLEND_M,
    PARTICIPANT_TICK_S, PARTICIPANT_CLEAR_S,
)
from automation import ParticipantAutomation
from reach import reach_flags as _compute_reach_flags
from registration import register_pose
from camera_thread import DepthCameraThread
from depth_extractor import (
    Crop, DepthGrooveParams, colorize_depth, encode_jpeg, process_depth,
)
from path_extractor import extract_from_edges, pixels_to_robot_coords
from path_executor import PathExecutor
from path_export import save_bundle
from robot_controller import RobotController
from server import Server
from settings import load_settings, save_settings
from surface import SurfaceModel, SurfacePose
from workspace import WorkspaceConfig

# ── Shared state ──────────────────────────────────────────────────────────────
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
    "executing":           False,
    "progress":            0.0,
    # ── Participant Mode (automated pipeline) ──
    "auto_on":             False,      # Participant popup Auto toggle
    "trigger_mm":          None,       # trigger distance (mm from camera); None = off
    "trigger_below":       None,       # camera thread: something closer than trigger_mm?
    "participant_status":  "Auto Off", # Auto Off|Auto On|Alerted|Sensing|Generating Paths|Actuating
    "participant_msg":     "",         # last automation outcome/message
    "participant_gen_params":  {},     # last crop/adjustments/spacing from Developer Mode
    "participant_exec_params": {},     # last speed_pct/offset_mm/safety_mm from Developer Mode
}
state_lock = threading.Lock()

# ── Singletons ────────────────────────────────────────────────────────────────
robot         = RobotController()
camera_thread = DepthCameraThread(shared_state, state_lock)
path_executor = PathExecutor(robot, shared_state, state_lock)
automation    = ParticipantAutomation(
    clear_ticks=round(PARTICIPANT_CLEAR_S / PARTICIPANT_TICK_S))


# ── Robot connection callbacks ────────────────────────────────────────────────
async def on_robot_connect(ip: str, ws) -> None:
    loop = asyncio.get_running_loop()
    try:
        await asyncio.wait_for(
            loop.run_in_executor(None, robot.connect, ip),
            timeout=10.0,
        )
        with state_lock:
            shared_state["robot_connected"] = True

        save_settings({"last_ip": ip})

        await server.send_connection_result(
            ws, True, f"Connected to {ip} — load a 3D surface to start."
        )
        print(f"Robot connected: {ip}")

    except asyncio.TimeoutError:
        with state_lock:
            shared_state["robot_connected"] = False
        msg = (
            f"Timeout: no RTDE response from {ip} after 10 s. "
            "Is the robot in Remote Control mode?"
        )
        await server.send_connection_result(ws, False, msg)
        print(msg)
    except Exception as exc:
        with state_lock:
            shared_state["robot_connected"] = False
        await server.send_connection_result(ws, False, str(exc))
        print(f"Robot connection failed: {exc}")


async def on_robot_disconnect(ws) -> None:
    loop = asyncio.get_running_loop()
    path_executor.cancel()
    if robot.connected:
        await loop.run_in_executor(None, robot.end_freedrive)
    await loop.run_in_executor(None, robot.disconnect)
    with state_lock:
        shared_state["robot_connected"] = False
        shared_state["freedrive"]       = False
        shared_state["pending_workspace"] = None
        shared_state["executing"]       = False
        shared_state["phase"]           = "idle"
    if ws is not None:
        await server.send_connection_result(ws, False, "Disconnected")
    print("Robot disconnected")


async def on_last_client_disconnect() -> None:
    print("Last client disconnected — stopping camera and robot.")
    loop = asyncio.get_running_loop()
    camera_thread.stop()
    path_executor.cancel()
    if robot.connected:
        await loop.run_in_executor(None, robot.end_freedrive)
        await loop.run_in_executor(None, robot.disconnect)
        with state_lock:
            shared_state["robot_connected"] = False
            shared_state["freedrive"]       = False
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
    ws_cfg = WorkspaceConfig.simulation()
    with state_lock:
        shared_state["workspace"]         = ws_cfg
        shared_state["pending_workspace"] = None
        shared_state["ws_points"]         = {"p0": None, "px": None, "py": None}
        shared_state["phase"]             = "previewing"
    if not camera_thread.running:
        camera_thread.start()
    print(
        f"Simulation workspace active (no robot): "
        f"{ws_cfg.x_extent:.3f} m × {ws_cfg.y_extent:.3f} m — Capture enabled."
    )


# ── Capture image / Edit / Generate path callbacks ───────────────────────────
def _mm_per_px(workspace, surface_model=None) -> float | None:
    """
    Millimetres per depth pixel for the mm-based groove filters. Planar mode
    derives it from the workspace calibration; surface mode from the drawing's
    fit onto the mesh — so neither strictly requires the other.
    """
    if workspace is not None:
        return (workspace.x_extent / DEPTH_WIDTH) * 1000.0
    if surface_model is not None:
        return surface_model.drawing_mm_per_px(DEPTH_WIDTH, DEPTH_HEIGHT)
    return None


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
    """Capture the current (undrawn) sand as a baseline for background subtraction."""
    captured = camera_thread.capture_frame()
    if captured is None:
        await server.send_reference_status(ws, False, "No depth frame to set as reference.")
        return
    depth_m, _valid, _rgb = captured
    with state_lock:
        shared_state["reference_depth"] = depth_m
    camera_thread.set_reference(depth_m)
    await server.send_reference_status(ws, True, "Reference captured — natural grooves can be subtracted.")
    print("Reference depth captured for background subtraction.")


async def on_clear_reference(ws) -> None:
    with state_lock:
        shared_state["reference_depth"] = None
    camera_thread.set_reference(None)
    await server.send_reference_status(ws, False, "Reference cleared.")
    print("Reference depth cleared.")


# ── Target surface (3D projection) callbacks ─────────────────────────────────
async def on_surface_upload(filename: str, blob: bytes) -> dict:
    """Save an uploaded STL/OBJ, load it, and broadcast mesh + status to clients."""
    SURFACE_DIR.mkdir(exist_ok=True)
    safe_name = os.path.basename(filename)
    path = SURFACE_DIR / safe_name
    path.write_bytes(blob)

    loop = asyncio.get_running_loop()
    model = await loop.run_in_executor(None, SurfaceModel.load, path)
    info = model.info()
    mesh_payload = await loop.run_in_executor(None, model.mesh_payload)

    with state_lock:
        shared_state["surface_model"] = model
        shared_state["surface_info"] = info
        shared_state["surface_mesh_payload"] = mesh_payload
        pose = shared_state["surface_pose"]
        offset = shared_state["surface_offset_mm"]
        # A surface replaces the flat workspace for mapping, so it also unlocks
        # the capture flow — no P0/Px/Py calibration needed in surface mode.
        if shared_state["phase"] == "idle":
            shared_state["phase"] = "previewing"

    if not camera_thread.running:
        camera_thread.start()

    await server.broadcast_surface_status(
        loaded=True, info=info, pose=pose, offset_mm=offset, mesh=mesh_payload,
        message=f"Surface loaded: {info['name']} ({info['faces']} faces, "
                f"{info['bbox']['size'][0]}×{info['bbox']['size'][1]} m)",
    )
    print(f"[surface] loaded {info['name']}: {info['faces']} faces, bbox {info['bbox']['size']} m")
    return {"info": info}


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
    with state_lock:
        shared_state["surface_model"] = None
        shared_state["surface_info"] = None
        shared_state["surface_mesh_payload"] = None
        shared_state["strokes_surface"] = False
    await server.broadcast_surface_status(loaded=False, message="Surface cleared.")
    print("[surface] cleared — paths map to the flat workspace again")


# ── Corner→TCP surface registration ──────────────────────────────────────────
# Optional touch-off placement: pick a mesh corner in the Path Preview,
# freedrive the tool tip onto the physical corner, confirm. 1-point for now
# (translation only — rotation stays on the sliders); registration.py already
# solves ≥3 points (full pose) for the future multi-point flow.
async def on_register_freedrive(ws, params: dict) -> None:
    """Toggle freedrive for the touch-off (registration popup button)."""
    on = bool((params or {}).get("on"))
    with state_lock:
        connected = shared_state.get("robot_connected", False)
        executing = shared_state.get("executing", False)
    if not connected:
        await server.send_register_result(ws, False, error="Robot not connected.")
        return
    if executing:
        await server.send_register_result(ws, False, error="Cannot freedrive while executing.")
        return
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(
        None, robot.start_freedrive if on else robot.end_freedrive)
    with state_lock:
        shared_state["freedrive"] = on
    print(f"[register] freedrive {'ON — move the tool tip to the corner' if on else 'off'}")


async def on_register_corner(ws, params: dict) -> None:
    """Confirm the touch-off: current TCP = the selected mesh corner."""
    try:
        idx = int((params or {}).get("corner_index", -1))
    except (TypeError, ValueError):
        idx = -1
    with state_lock:
        model = shared_state.get("surface_model")
        pose_d = shared_state.get("surface_pose")
        offset = shared_state.get("surface_offset_mm", 0.0)
        info = shared_state.get("surface_info")
        connected = shared_state.get("robot_connected", False)
    if model is None:
        await server.send_register_result(ws, False, error="No surface loaded.")
        return
    if not connected:
        await server.send_register_result(ws, False, error="Robot not connected.")
        return
    corners = model.corner_points()
    if not 0 <= idx < len(corners):
        await server.send_register_result(ws, False, error="No corner selected.")
        return

    loop = asyncio.get_running_loop()
    tcp = await loop.run_in_executor(None, robot.get_ee_position)
    try:
        new_pose = register_pose([corners[idx]], [tcp[:3]],
                                 SurfacePose.from_dict(pose_d))
    except ValueError as exc:
        await server.send_register_result(ws, False, error=str(exc))
        return

    # Touch-off done — leave freedrive so the robot holds position again.
    await loop.run_in_executor(None, robot.end_freedrive)
    with state_lock:
        shared_state["surface_pose"] = new_pose.to_dict()
        shared_state["freedrive"] = False

    msg = (f"Corner {idx + 1} registered at TCP "
           f"[{tcp[0]:.3f}, {tcp[1]:.3f}, {tcp[2]:.3f}] m — re-run Generate Path.")
    await server.send_register_result(ws, True, message=msg, pose=new_pose.to_dict())
    # Sliders + preview update everywhere via the normal surface_status path.
    await server.broadcast_surface_status(
        loaded=True, info=info, pose=new_pose.to_dict(), offset_mm=offset,
        mesh=None, message=msg,
    )
    print(f"[register] {msg}")


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
        print(f"Captured still: {w}×{h} (depth+colour) — ready for crop/adjust")
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
        print(f"[preview] processing error: {exc}")
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
    mmpp = _mm_per_px(workspace, surface_model)

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

    reach_flags, reach_out, reach_total = _compute_reach_flags(robot_strokes)

    with state_lock:
        shared_state["strokes"] = robot_strokes
        shared_state["strokes_surface"] = surface_mode
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
        # Everything the browser needs to rebuild the toolpath preview
        # client-side when the exec-bar Offset/Safety inputs change.
        exec_viz={
            "blend_m": session_blend_mm / 1000.0,
            "reach_m": UR_REACH_M,
            "min_reach_m": UR_MIN_REACH_M,
            "spacing_mm": spacing_mm,
            "join_mm": join_mm,
        },
    )
    print(f"Generated path: {extracted.total_strokes} strokes, {extracted.total_points} points"
          + (f" — WARNING: {reach_out}/{reach_total} waypoints outside estimated reach"
             if reach_out else ""))


async def on_retake(ws) -> None:
    """Discard the captured still and return to the live preview phase."""
    with state_lock:
        shared_state["captured_still"] = None
        shared_state["still_dims"]     = None
        shared_state["strokes"]        = []
        shared_state["phase"]          = "previewing"
    if not camera_thread.running:
        camera_thread.start()
    print("Retake — back to live preview")


async def on_run(ws, params: dict | None = None) -> None:
    if _manual_locked(ws):
        await server.send_capture_result(ws, False, error=_AUTO_LOCK_MSG)
        return
    with state_lock:
        strokes      = shared_state.get("strokes", [])
        connected    = shared_state.get("robot_connected", False)
        surface_mode = shared_state.get("strokes_surface", False)

    if not strokes:
        return
    if not connected:
        await server.send_capture_result(ws, False, error="Robot not connected.")
        return
    if path_executor.running:
        return

    # Execution settings from the preview bar: speed as % of the robot's max
    # TCP speed, plus run-time offset / safety retract distances in mm.
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
    draw_speed = (speed_pct / 100.0) * MAX_TCP_SPEED

    with state_lock:
        shared_state["participant_exec_params"] = {
            "speed_pct": speed_pct, "offset_mm": offset_mm,
            "safety_mm": safety_mm, "blend_mm": blend_mm}

    # Surface strokes already carry contact depth along the surface normal, so
    # the executor must not add the planar DRAW_Z on top.
    path_executor.start(
        strokes,
        draw_z=0.0 if surface_mode else DRAW_Z,
        draw_speed=draw_speed,
        normal_offset=offset_mm / 1000.0,
        travel_dist=safety_mm / 1000.0,
        blend_m=blend_mm / 1000.0,
    )
    print(f"[executor] starting path: {len(strokes)} strokes, "
          f"{speed_pct:.0f}% speed ({draw_speed:.3f} m/s), "
          f"offset {offset_mm:.1f} mm, safety {safety_mm:.0f} mm, "
          f"blend {blend_mm:.1f} mm"
          + (" (surface mode)" if surface_mode else ""))


async def on_cancel(ws) -> None:
    path_executor.cancel()
    print("[executor] cancel requested")


async def on_save_path(ws, params: dict) -> None:
    """Save the generated toolpath as URScript + JSON + preview image."""
    from datetime import datetime

    with state_lock:
        strokes      = shared_state.get("strokes", [])
        surface_mode = shared_state.get("strokes_surface", False)
        surface_info = shared_state.get("surface_info")
        surface_pose = shared_state.get("surface_pose")

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

    loop = asyncio.get_running_loop()
    try:
        folder = await loop.run_in_executor(
            None,
            functools.partial(
                save_bundle, strokes, speed, safety_mm / 1000.0, offset_mm / 1000.0,
                meta, params.get("image"), blend_m=blend_mm / 1000.0,
            ),
        )
    except Exception as exc:
        await server.send_save_result(ws, False, error=str(exc))
        return

    await server.send_save_result(ws, True, folder=str(folder))
    print(f"[save] toolpath saved to {folder}")


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
    print(f"[participant] automation {'ON' if on else 'OFF'}")


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
    """Set (or clear with null/empty) the Participant-Mode trigger distance."""
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
    print(f"[participant] trigger {'set to %.0f mm' % mm if mm is not None else 'off'}")


async def _participant_pipeline() -> None:
    """One automated run: Sensing → Generating Paths → Actuating (save + run)."""
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
        _sync_participant_state()
        with state_lock:
            gen_params = dict(shared_state.get("participant_gen_params") or {})
        await on_generate_path(bws, gen_params)
        with state_lock:
            strokes = shared_state.get("strokes", [])
            connected = shared_state.get("robot_connected", False)
        if not strokes:
            automation.finish("No grooves detected — nothing to draw.")
            return

        # ── Actuating: save the bundle first (always), then run on the robot.
        automation.stage("Actuating")
        _sync_participant_state()
        with state_lock:
            exec_params = dict(shared_state.get("participant_exec_params") or {})
        await on_save_path(bws, exec_params)
        if not connected:
            automation.finish("Path saved — robot not connected, run skipped.")
            return
        await on_run(bws, exec_params)
        await asyncio.sleep(0.5)                # let the executor thread spin up
        while path_executor.running:
            await asyncio.sleep(0.2)
        with state_lock:
            err = shared_state.get("exec_error")
        automation.finish(f"Run failed: {err}" if err else "Done — path drawn and saved.")
    except Exception as exc:
        automation.finish(f"Automation error: {exc}")
        print(f"[participant] pipeline error: {exc}")
    finally:
        _sync_participant_state()
        print(f"[participant] {automation.message or 'pipeline finished'}")


async def _participant_loop() -> None:
    """Poll the camera trigger flag and drive the automation state machine."""
    while True:
        await asyncio.sleep(PARTICIPANT_TICK_S)
        with state_lock:
            below = shared_state.get("trigger_below")
            executing = shared_state.get("executing", False)
        # A manually-started run owns the robot — don't trigger on top of it.
        if executing and not automation.busy:
            continue
        prev = (automation.status, automation.message)
        if automation.tick(below):
            asyncio.create_task(_participant_pipeline())
        if (automation.status, automation.message) != prev:
            _sync_participant_state()


# ── Entry point ───────────────────────────────────────────────────────────────
server = Server(
    shared_state,
    state_lock,
    robot,
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
    on_depth_overlay_params=on_depth_overlay_params,
    on_register_freedrive=on_register_freedrive,
    on_register_corner=on_register_corner,
    on_set_trigger=on_set_trigger,
    on_set_automation=on_set_automation,
    on_set_exec_params=on_set_exec_params,
)

# Camera starts immediately so both MJPEG feeds are live from the moment you open the browser
camera_thread.start()


# ── Live TCP poller ───────────────────────────────────────────────────────────
# Nothing else writes shared_state["ee"], so without this the preview's TCP dot
# would sit at the origin forever. Poll the RTDE receive buffer at 10 Hz while
# connected; the broadcast loop relays it to the browser.
def _ee_poller() -> None:
    import time
    while True:
        time.sleep(0.1)
        if not robot.connected:
            continue
        try:
            pose = robot.get_ee_position()
            with state_lock:
                shared_state["ee"] = list(pose)
        except Exception:
            pass


threading.Thread(target=_ee_poller, daemon=True, name="ee_poller").start()


async def _open_browser() -> None:
    await asyncio.sleep(1.0)
    webbrowser.open(f"http://{HTTP_HOST}:{HTTP_PORT}")


async def _main() -> None:
    asyncio.create_task(_open_browser())
    asyncio.create_task(_participant_loop())
    await server.start()


if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        print("\nShutting down.")
        camera_thread.stop()
        path_executor.cancel()
        robot.end_freedrive()
        robot.disconnect()
