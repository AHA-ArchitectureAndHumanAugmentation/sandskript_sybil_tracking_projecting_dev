"""
Opening and reading the RealSense cameras — the ONE place devices are started.

Both frame producers use this: `camera_thread.DepthCameraThread` (the main app,
which combines every camera into one canvas) and `multi_camera.MultiCameraThread`
(the Multi-Cam Vision placement tool). They must agree on which devices exist,
in which order, and with which intrinsics, or the layout the operator saved in
the tool would land differently in the app.

Cameras are enumerated by SERIAL and returned sorted, so USB port order never
shuffles a rig. Every device is opened with depth + colour at the configured
resolution, colour aligned to depth, exactly as the single-camera pipeline
always did.

One process per RealSense still applies: whoever calls `open_cameras` owns them
until `close`. That is why the main app and the placement tool cannot run at the
same time.

Contains no detection, no placement math and no server code — hardware in,
numpy out.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from config import (DEPTH_FPS, DEPTH_HEIGHT, DEPTH_WIDTH,
                    REALSENSE_DEPTH_FILTERS, STITCH_MAX_CAMERAS)
from stitcher import Intrinsics


@dataclass
class Camera:
    """One open RealSense: its pipeline plus everything needed to read metric depth."""
    pipe: object                # rs.pipeline
    align: object               # rs.align(depth)
    depth_scale: float          # metres per raw unit
    intr: Intrinsics
    serial: str
    # SDK post-processing applied to every depth frame in _unpack, in order.
    # The temporal filter keeps per-camera state inside the SDK, so these MUST
    # be one set per Camera — sharing them across devices would blend rigs.
    filters: tuple = ()


def open_cameras(max_cameras: int = STITCH_MAX_CAMERAS
                 ) -> tuple[list[Camera], str]:
    """
    Start every RealSense present, up to `max_cameras`.

    Returns ``(cameras, note)``. An empty list always comes with a note saying
    why — no SDK, no device, or a device that would not start — which callers
    print or show; a partial start is never returned, since a rig missing a
    camera would place the remaining ones against a layout that assumed it.
    """
    try:
        import pyrealsense2 as rs   # noqa: local import so this module loads without the SDK
    except ImportError:
        return [], ("pyrealsense2 not installed — run "
                    "`<env python> -m pip install pyrealsense2`")

    try:
        ctx = rs.context()
        serials = sorted(d.get_info(rs.camera_info.serial_number)
                         for d in ctx.query_devices())
    except Exception as exc:
        return [], f"RealSense enumeration failed ({exc})"
    if not serials:
        return [], "no RealSense cameras found (is another app holding them?)"

    note = ""
    if len(serials) > max_cameras:
        note = f"{len(serials)} cameras found — using the first {max_cameras}"
        serials = serials[:max_cameras]

    cameras: list[Camera] = []
    for serial in serials:
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT,
                          rs.format.z16, DEPTH_FPS)
        cfg.enable_stream(rs.stream.color, DEPTH_WIDTH, DEPTH_HEIGHT,
                          rs.format.bgr8, DEPTH_FPS)
        pipe = rs.pipeline()
        try:
            profile = pipe.start(cfg)
        except Exception as exc:
            close(cameras)
            return [], f"cannot start camera {serial} ({exc})"
        ri = (profile.get_stream(rs.stream.depth)
              .as_video_stream_profile().get_intrinsics())
        # Depth noise reduction at the source (SDK defaults): an edge-preserving
        # spatial filter, then a temporal filter. The D435i's 1-2 mm per-pixel
        # noise is the same size as the ~1.5 mm relief groove detection looks
        # for, so cutting it here helps every consumer at once. Both are cheap
        # (they run in the SDK on 640x480), and the temporal one is stateful —
        # built per camera, never shared.
        filters: tuple = ()
        if REALSENSE_DEPTH_FILTERS:
            filters = (rs.spatial_filter(), rs.temporal_filter())
        cameras.append(Camera(
            pipe=pipe,
            align=rs.align(rs.stream.depth),
            depth_scale=profile.get_device().first_depth_sensor().get_depth_scale(),
            intr=Intrinsics(fx=ri.fx, fy=ri.fy, cx=ri.ppx, cy=ri.ppy,
                            width=ri.width, height=ri.height),
            serial=serial,
            filters=filters,
        ))
    return cameras, note


def poll(camera: Camera) -> Optional[tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]]:
    """
    Non-blocking read: ``(depth_m float32, valid bool, rgb BGR|None)`` or None
    when this camera has nothing new. Polling (rather than waiting) is what lets
    one thread serve several cameras without the slowest one pacing the rest.
    """
    try:
        frames = camera.pipe.poll_for_frames()
    except Exception:
        return None
    if not frames:
        return None
    return _unpack(camera, frames)


def wait(camera: Camera, timeout_ms: int = 2000
         ) -> Optional[tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]]:
    """Blocking read of the next frame set; None on timeout or error."""
    try:
        frames = camera.pipe.wait_for_frames(timeout_ms)
    except Exception:
        return None
    return _unpack(camera, frames)


def _unpack(camera: Camera, frames):
    """Align, convert to metres, and pull the colour frame if there is one."""
    frames = camera.align.process(frames)
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        return None
    # Post-process the depth in the SDK before touching the pixels. Alignment
    # maps colour ONTO depth (rs.align(rs.stream.depth)), so filtering after
    # align changes nothing about the mapping — the depth grid is untouched.
    for f in camera.filters:
        try:
            depth_frame = f.process(depth_frame)
        except Exception:
            break                     # a failing filter must not drop the frame
    z = np.asarray(depth_frame.get_data(), dtype=np.float32) * camera.depth_scale
    color_frame = frames.get_color_frame()
    # Copy the colour: np.asarray on a frame is a VIEW of SDK memory, and the
    # frame is released when this function returns — callers keep the latest
    # colour around for whole seconds (it is only redrawn when the canvas is).
    rgb = np.asarray(color_frame.get_data()).copy() if color_frame else None
    return z, z > 0, rgb


def close(cameras: list[Camera]) -> None:
    """Stop every pipeline; safe to call on a partially-opened list."""
    for camera in cameras:
        try:
            camera.pipe.stop()
        except Exception:
            pass
