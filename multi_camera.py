"""
Capture thread for Multi-Cam Vision (stitch_main.py).

Owns EVERY RealSense it finds (up to STITCH_MAX_CAMERAS, selected by serial
number), keeps a short rolling buffer per camera for temporal averaging, and
every STITCH_EVERY_S rebuilds the combined canvas plus one thumbnail per
camera into shared_state. One camera is a perfectly valid rig — the canvas is
then just that camera's frame. With no camera at all (or pyrealsense2 missing,
or the main app holding the devices) it falls back to a SYNTHETIC scene so the
placement workflow stays testable.

This tool ONLY combines images. Groove detection and its parameters live in
the main app; nothing here imports them.

Deliberately separate from camera_thread.DepthCameraThread — this prototype
must not touch the single-camera pipeline. One process per RealSense still
applies: close the main app before running this tool.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Optional

import cv2
import numpy as np

from config import (
    DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FPS,
    STITCH_AVERAGE_FRAMES, STITCH_EVERY_S, STITCH_HEIGHT_STEP_MM,
    STITCH_MAX_CAMERAS, STITCH_SYNTHETIC_CAMERAS,
)
from depth_extractor import colorize_depth, encode_jpeg
from stitcher import (
    CameraFrame, CameraPlacement, Intrinsics, StitchCalib,
    bind_placements, crop_mask, default_quad_mm, footprint_mm, requad_for_crop,
    rotate_frame, rotate_quad, stitch, synthetic_scene,
)


def cam_key(index: int) -> str:
    """State key for camera `index`'s thumbnail stream."""
    return f"stitch_cam{index}_jpg"


class MultiCameraThread:
    def __init__(self, shared_state: dict, state_lock: threading.Lock) -> None:
        self._state = shared_state
        self._state_lock = state_lock
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._calib = StitchCalib()
        self._serials: list[str] = []
        # Per-camera measurements refreshed every cycle, because the server
        # thread edits placements between cycles and needs them: the rotated
        # frame shape (to re-cut a quad when the crop changes) and the patch of
        # sand one frame covers (to rebuild a default quad on reset).
        self._shapes: list[tuple[int, int]] = []
        self._footprints: list[tuple[float, float]] = []
        self._show_colour = False

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="multi_camera_thread")
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3.0)
            self._thread = None

    # ── setters (atomic swaps, same pattern as DepthCameraThread) ────────────
    def set_calib(self, calib: StitchCalib) -> None:
        self._calib = calib
        self._publish_calib()

    def get_calib(self) -> StitchCalib:
        return self._calib

    def set_placement(self, index: int, changes: dict) -> None:
        """
        Edit one camera. A crop change also re-cuts the placed quad through the
        old corner-pin, so trimming a frame never slides the sand it kept.
        """
        cams = self._calib.cams
        if not 0 <= index < len(cams):
            return
        p = cams[index]
        changes = dict(changes or {})
        if "crop" in changes and p.quad_mm and index < len(self._shapes):
            changes["quad_mm"] = [list(c) for c in
                                  requad_for_crop(p, changes["crop"], self._shapes[index])]
        self._calib = self._calib.with_camera(index, p.merged(changes))
        self._publish_calib()

    def rotate_camera(self, index: int, steps: int = 1) -> None:
        """Quarter-turn the feed AND its quad, so the picture turns unstretched."""
        cams = self._calib.cams
        if not 0 <= index < len(cams):
            return
        p = cams[index]
        changes = {"rot_deg": (p.rot_deg + 90 * steps) % 360,
                   "crop": _rotate_crop(p.crop, steps)}
        if p.quad_mm:
            changes["quad_mm"] = [list(c) for c in rotate_quad(p.quad_mm, steps)]
        self._calib = self._calib.with_camera(index, p.merged(changes))
        self._publish_calib()

    def nudge_height(self, index: int, steps: int = 1) -> None:
        """Raise/lower one camera's depths when its sand sits off its neighbour's."""
        cams = self._calib.cams
        if not 0 <= index < len(cams):
            return
        p = cams[index]
        self._calib = self._calib.with_camera(
            index, p.merged({"height_mm": p.height_mm + steps * STITCH_HEIGHT_STEP_MM}))
        self._publish_calib()

    def reset_camera(self, index: int, corners_only: bool = False) -> None:
        """Back to an unplaced default: corners only, or the whole camera."""
        cams = self._calib.cams
        if not 0 <= index < len(cams):
            return
        p = cams[index]
        fresh = p if corners_only else CameraPlacement(serial=p.serial)
        fresh = fresh.merged(
            {"quad_mm": [list(c) for c in self._default_quad(index, fresh)]})
        self._calib = self._calib.with_camera(index, fresh)
        self._publish_calib()

    def set_grid(self, mm_per_px: float) -> None:
        """Canvas resolution in mm per pixel; 0 = auto from the first camera."""
        self._calib = StitchCalib(cams=list(self._calib.cams),
                                  mm_per_px=max(0.0, float(mm_per_px)))
        self._publish_calib()

    def set_colour(self, on: bool) -> None:
        """Show the colour streams instead of depth (previews only)."""
        self._show_colour = bool(on)

    def get_colour(self) -> bool:
        return self._show_colour

    def _publish_calib(self) -> None:
        with self._state_lock:
            self._state["stitch_calib"] = self._calib.to_dict()

    # ── main loop ────────────────────────────────────────────────────────────
    def _run(self) -> None:
        pipes = self._try_start_cameras()
        if pipes is None:
            self._run_synthetic()
        else:
            try:
                self._run_real(pipes)
            finally:
                for pipe, _, _, _, _ in pipes:
                    try:
                        pipe.stop()
                    except Exception:
                        pass
        with self._state_lock:
            self._state["stitch_canvas_jpg"] = None
            for i in range(STITCH_MAX_CAMERAS):
                self._state[cam_key(i)] = None
        print("[stitch] stopped")

    def _try_start_cameras(self):
        """Start every RealSense pipeline; None → caller uses synthetic mode."""
        try:
            import pyrealsense2 as rs
        except ImportError:
            self._set_note("pyrealsense2 not installed — SYNTHETIC scene")
            return None
        try:
            ctx = rs.context()
            serials = sorted(d.get_info(rs.camera_info.serial_number)
                             for d in ctx.query_devices())
        except Exception as exc:
            self._set_note(f"RealSense enumeration failed ({exc}) — SYNTHETIC scene")
            return None
        if not serials:
            self._set_note("no cameras found (is the main app running?) "
                           "— SYNTHETIC scene")
            return None
        if len(serials) > STITCH_MAX_CAMERAS:
            self._set_note(f"{len(serials)} cameras found — using the first "
                           f"{STITCH_MAX_CAMERAS}")
            serials = serials[:STITCH_MAX_CAMERAS]

        pipes = []
        for serial in serials:
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT,
                              rs.format.z16, DEPTH_FPS)
            cfg.enable_stream(rs.stream.color, DEPTH_WIDTH, DEPTH_HEIGHT,
                              rs.format.bgr8, DEPTH_FPS)
            try:
                profile = pipe.start(cfg)
            except Exception as exc:
                for p, _, _, _, _ in pipes:
                    p.stop()
                self._set_note(f"cannot start camera {serial} ({exc}) — SYNTHETIC scene")
                return None
            scale = profile.get_device().first_depth_sensor().get_depth_scale()
            ri = (profile.get_stream(rs.stream.depth)
                  .as_video_stream_profile().get_intrinsics())
            intr = Intrinsics(fx=ri.fx, fy=ri.fy, cx=ri.ppx, cy=ri.ppy,
                              width=ri.width, height=ri.height)
            align = rs.align(rs.stream.depth)
            pipes.append((pipe, align, scale, intr, serial))
            print(f"[stitch] camera {serial}: fx={ri.fx:.1f} fy={ri.fy:.1f}")
        print(f"[stitch] {len(serials)} camera(s) live")
        return pipes

    def _run_real(self, pipes) -> None:
        buffers = [deque(maxlen=STITCH_AVERAGE_FRAMES) for _ in pipes]
        last_rgb: list[Optional[np.ndarray]] = [None] * len(pipes)
        last_stitch = 0.0
        while not self._stop_event.is_set():
            got = False
            for i, (pipe, align, scale, _intr, _serial) in enumerate(pipes):
                try:
                    frames = pipe.poll_for_frames()
                except Exception:
                    continue
                if not frames:
                    continue
                frames = align.process(frames)
                df = frames.get_depth_frame()
                if not df:
                    continue
                z = np.asarray(df.get_data(), dtype=np.float32) * scale
                buffers[i].append((z, z > 0))
                cf = frames.get_color_frame()
                if cf:
                    last_rgb[i] = np.asarray(cf.get_data())
                got = True

            now = time.monotonic()
            if (now - last_stitch) >= STITCH_EVERY_S and all(buffers):
                last_stitch = now
                built = []
                for i, (_p, _a, _s, intr, serial) in enumerate(pipes):
                    depth, valid = _average(buffers[i])
                    built.append(CameraFrame(depth, valid, intr, last_rgb[i], serial))
                self._process(built, synthetic=False)
            if not got:
                time.sleep(0.005)

    def _run_synthetic(self) -> None:
        i = 0
        while not self._stop_event.is_set():
            frames, _true = synthetic_scene(STITCH_SYNTHETIC_CAMERAS,
                                            overlap_frac=0.08, seed=i)
            i += 1
            self._process(frames, synthetic=True)
            # Sleep in small steps so stop() stays responsive.
            deadline = time.monotonic() + STITCH_EVERY_S
            while not self._stop_event.is_set() and time.monotonic() < deadline:
                time.sleep(0.05)

    # ── one canvas cycle ─────────────────────────────────────────────────────
    def _sync_placements(self, frames: list[CameraFrame]) -> None:
        """
        Bind the calibration to the cameras actually present. Runs only when the
        set of serials changes (start-up, or a camera unplugged), so the
        operator's live edits are never rebound underneath them.
        """
        serials = [f.serial for f in frames]
        if serials == self._serials and len(self._calib.cams) == len(frames):
            return
        self._serials = serials
        bound = bind_placements(self._calib, frames)
        # Materialize the default corners now rather than letting stitch() infer
        # them every frame: the browser drags corners RELATIVE to this list, so
        # it must never be empty once a camera is known.
        cams = []
        for i, (p, f) in enumerate(zip(bound.cams, frames)):
            if not p.quad_mm:
                p = p.merged({"quad_mm": [list(c) for c in self._default_quad(i, p)]})
            cams.append(p)
        self._calib = StitchCalib(cams=cams, mm_per_px=bound.mm_per_px)
        self._publish_calib()

    def _default_quad(self, index: int, p: CameraPlacement):
        foot = (self._footprints[index] if index < len(self._footprints)
                else (800.0, 600.0))
        return default_quad_mm(foot, p.crop, index)

    def _measure(self, frames: list[CameraFrame]) -> list[CameraFrame]:
        """Rotate every frame once, and cache what the setters will need."""
        rotated = [rotate_frame(f, self._calib.placement_for(f.serial, i).rot_deg)
                   for i, f in enumerate(frames)]
        self._shapes = [g.depth_m.shape[:2] for g in rotated]
        self._footprints = [footprint_mm(g) for g in rotated]
        return rotated

    def _thumbnails(self, rotated: list[CameraFrame], calib: StitchCalib) -> dict:
        """
        One preview per camera, as its placement rotates it, with the crop
        rectangle drawn on — this is where the operator drags the crop.
        """
        out = {}
        for i, g in enumerate(rotated):
            p = calib.placement_for(g.serial, i)
            if self._show_colour and g.rgb is not None:
                view = g.rgb.copy()
            else:
                view = colorize_depth(g.depth_m, crop_mask(g.valid, p.crop))
            ih, iw = view.shape[:2]
            x, y, w, h = p.crop
            cv2.rectangle(view, (int(x * iw), int(y * ih)),
                          (int((x + w) * iw) - 1, int((y + h) * ih) - 1),
                          (255, 220, 80), 1)
            out[cam_key(i)] = encode_jpeg(view)
        return out

    def _process(self, frames: list[CameraFrame], synthetic: bool) -> None:
        rotated = self._measure(frames)
        self._sync_placements(frames)
        calib = self._calib

        thumbs = self._thumbnails(rotated, calib)
        result = stitch(frames, calib)

        if self._show_colour:
            canvas = result.rgb.copy()
        else:
            canvas = colorize_depth(result.depth_m, result.valid)
        # Outline where cameras overlap: the operator is aiming for a thin band.
        contours, _ = cv2.findContours(result.overlap.astype(np.uint8),
                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(canvas, contours, -1, (255, 220, 80), 1)

        gh, gw = result.depth_m.shape
        n_valid = int(result.valid.sum())
        cameras = []
        for i, f in enumerate(frames):
            quad = result.quads_px[i] if i < len(result.quads_px) else None
            p = calib.placement_for(f.serial, i)
            cameras.append({
                "index": i,
                "serial": f.serial or f"cam{i + 1}",
                "quad_px": None if quad is None else [[round(u, 1), round(v, 1)]
                                                      for u, v in quad],
                "placed": bool(p.quad_mm),
            })
        info = {
            "synthetic": synthetic,
            "cameras": cameras,
            "count": len(frames),
            "colour": self._show_colour,
            "mm_per_px": round(result.mm_per_px, 3),
            "size": [gw, gh],
            "overlap_pct": round(float(100.0 * result.overlap.sum() / max(1, n_valid)), 1),
        }
        with self._state_lock:
            self._state.update(thumbs)
            self._state["stitch_canvas_jpg"] = encode_jpeg(canvas)
            self._state["stitch_info"] = info
            self._state["stitch_calib"] = calib.to_dict()

    def _set_note(self, msg: str) -> None:
        print(f"[stitch] {msg}")
        with self._state_lock:
            self._state["stitch_note"] = msg


def _rotate_crop(crop, steps: int) -> list[float]:
    """
    The crop rectangle follows the image through a quarter turn, so rotating a
    camera never re-exposes a strip the operator already trimmed away.
    """
    x, y, w, h = crop
    for _ in range(steps % 4):
        x, y, w, h = 1.0 - y - h, x, h, w      # clockwise in normalized coords
    return [x, y, w, h]


def _average(buffer: deque) -> tuple[np.ndarray, np.ndarray]:
    """Temporal average of a (depth, valid) buffer — same math as capture_frame."""
    frames = list(buffer)
    acc = np.zeros_like(frames[0][0], dtype=np.float32)
    cnt = np.zeros_like(acc, dtype=np.float32)
    for z, ok in frames:
        acc[ok] += z[ok]
        cnt += ok
    valid = cnt > 0
    depth = np.zeros_like(acc)
    depth[valid] = acc[valid] / cnt[valid]
    return depth, valid
