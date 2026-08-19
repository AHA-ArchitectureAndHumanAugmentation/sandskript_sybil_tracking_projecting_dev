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

Deliberately separate from camera_thread.DepthCameraThread — that thread builds
the same canvas for the pipeline (from the layout saved here), and the two must
stay independent: this one is where a layout is SHAPED, with per-camera
thumbnails, drag handles and no detection at all. They share only the pure
placement math (stitcher.py) and the device layer (realsense_source.py). One
process per RealSense still applies: close the main app before running this tool.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Optional

import cv2
import numpy as np

from config import (
    STITCH_AVERAGE_FRAMES, STITCH_EVERY_S, STITCH_HEIGHT_STEP_MM,
    STITCH_MAX_CAMERAS, STITCH_SEAM_EVERY_S, STITCH_SYNTHETIC_CAMERAS,
)
from depth_extractor import colorize_depth, encode_jpeg
import realsense_source
from stitcher import (
    CameraFrame, CameraPlacement, StitchCalib,
    bind_placements, common_footprint_mm, crop_mask, default_quad_mm,
    default_row_mm, footprint_mm, load_calib, quad_centre_x, requad_for_crop,
    rotate_frame, rotate_quad, seam_report, stitch, swap_quads_x, synthetic_scene,
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
        # The layout as last loaded from / written to disk. None until the
        # cameras are bound, because that is when "what was loaded" is known.
        self._saved: Optional[StitchCalib] = None
        self._serials: list[str] = []
        # Per-camera measurements refreshed every cycle, because the server
        # thread edits placements between cycles and needs them: the rotated
        # frame shape (to re-cut a quad when the crop changes) and the patch of
        # sand one frame covers (to rebuild a default quad on reset).
        self._shapes: list[tuple[int, int]] = []
        self._footprints: list[tuple[float, float]] = []
        # The rig's tile size, frozen when the camera set was last bound. Median
        # depth wobbles a few tenths of a percent frame to frame, and deriving
        # the tile afresh on every reset let that wobble into the layout —
        # resetting one camera left it a centimetre out of line with its
        # neighbours, which is exactly what a single-scale row must not do.
        self._tile_mm: Optional[tuple[float, float]] = None
        self._show_colour = False
        # Seam agreement, refreshed on a slower clock than the canvas (it warps
        # every camera a second time) and held in between.
        self._seam_cache: list = []
        self._seam_at = 0.0

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
        self._saved = None          # re-snapshot once it is bound to the cameras
        self._publish_calib()

    def get_calib(self) -> StitchCalib:
        return self._calib

    @property
    def dirty(self) -> bool:
        """
        True when the layout on screen is no longer the one on disk.

        This matters more than it looks: the MAIN APP builds its combined view
        from the saved file, so an adjustment nobody saved is a picture only
        this tool can see. Measured against the layout as it was LOADED (after
        binding to the cameras present), so simply opening the tool is never
        reported as a change.
        """
        if self._saved is None:
            return False
        return self._calib.to_dict() != self._saved.to_dict()

    def mark_saved(self) -> None:
        """The current layout is now what is on disk (call after writing it)."""
        self._saved = self._calib

    def revert_calib(self) -> None:
        """
        Throw away every unsaved adjustment and go back to the file — the same
        layout a restart would load, and the one the main app will use.
        """
        self._calib = load_calib()
        self._saved = None
        self._serials = []          # forces a re-bind on the next canvas
        self._publish_calib()

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

    def move_camera(self, index: int, steps: int = 1) -> None:
        """
        Shift this camera one place left (steps < 0) or right (steps > 0) in
        the row, swapping with whichever camera is currently there. Order is
        judged by where the quads actually sit on the canvas, not by USB
        enumeration — the operator is matching the picture to the rig, and
        which device happened to enumerate first has nothing to do with it.
        """
        cams = self._calib.cams
        if not 0 <= index < len(cams) or not steps:
            return
        placed = sorted((i for i, p in enumerate(cams) if p.quad_mm),
                        key=lambda i: quad_centre_x(cams[i].quad_mm))
        if index not in placed:
            return
        target = placed.index(index) + (1 if steps > 0 else -1)
        if not 0 <= target < len(placed):
            return
        other = placed[target]
        qa, qb = swap_quads_x(cams[index].quad_mm, cams[other].quad_mm)
        calib = self._calib.with_camera(
            index, cams[index].merged({"quad_mm": [list(c) for c in qa]}))
        calib = calib.with_camera(
            other, calib.cams[other].merged({"quad_mm": [list(c) for c in qb]}))
        self._calib = calib
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
        cameras, note = realsense_source.open_cameras(STITCH_MAX_CAMERAS)
        if not cameras:
            self._set_note(f"{note} — SYNTHETIC scene")
            self._run_synthetic()
        else:
            if note:
                self._set_note(note)
            print(f"[stitch] {len(cameras)} camera(s) live: "
                  + ", ".join(c.serial for c in cameras))
            try:
                self._run_real(cameras)
            finally:
                realsense_source.close(cameras)
        with self._state_lock:
            self._state["stitch_canvas_jpg"] = None
            for i in range(STITCH_MAX_CAMERAS):
                self._state[cam_key(i)] = None
        print("[stitch] stopped")

    def _run_real(self, cameras) -> None:
        buffers = [deque(maxlen=STITCH_AVERAGE_FRAMES) for _ in cameras]
        last_rgb: list[Optional[np.ndarray]] = [None] * len(cameras)
        last_stitch = 0.0
        while not self._stop_event.is_set():
            got = False
            for i, camera in enumerate(cameras):
                frame = realsense_source.poll(camera)
                if frame is None:
                    continue
                z, ok, rgb = frame
                buffers[i].append((z, ok))
                if rgb is not None:
                    last_rgb[i] = rgb
                got = True

            now = time.monotonic()
            if (now - last_stitch) >= STITCH_EVERY_S and all(buffers):
                last_stitch = now
                built = []
                for i, camera in enumerate(cameras):
                    depth, valid = _average(buffers[i])
                    built.append(CameraFrame(depth, valid, camera.intr,
                                             last_rgb[i], camera.serial))
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
        self._tile_mm = common_footprint_mm(self._footprints)
        bound = bind_placements(self._calib, frames)
        # Materialize the default corners now rather than letting stitch() infer
        # them every frame: the browser drags corners RELATIVE to this list, so
        # it must never be empty once a camera is known.
        row = self._default_row(bound.cams)
        cams = []
        for i, p in enumerate(bound.cams):
            if not p.quad_mm and i < len(row):
                p = p.merged({"quad_mm": [list(c) for c in row[i]]})
            cams.append(p)
        self._calib = StitchCalib(cams=cams, mm_per_px=bound.mm_per_px)
        # The layout as LOADED is the baseline "unchanged" state: binding to the
        # cameras present is not an operator edit, so it must not read as one.
        if self._saved is None:
            self._saved = self._calib
        self._publish_calib()

    def _default_row(self, cams: list[CameraPlacement]):
        """The flush, single-scale opening row for the cameras present."""
        tile = self._tile_mm or common_footprint_mm(self._footprints)
        return default_row_mm([tile] * len(cams), [p.crop for p in cams])

    def _default_quad(self, index: int, p: CameraPlacement):
        """Where camera `index` belongs in that row, with `p`'s crop applied."""
        cams = list(self._calib.cams)
        if 0 <= index < len(cams):
            cams[index] = p
            row = self._default_row(cams)
            if index < len(row):
                return row[index]
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

    def _seams(self, frames: list[CameraFrame], calib: StitchCalib) -> list:
        """
        Seam agreement, recomputed every STITCH_SEAM_EVERY_S and cached in
        between. It costs a second set of warps, and the answer only moves when
        the operator moves something — but it must NOT be frozen on the first
        frame either, or a Height nudge would appear to do nothing.
        """
        now = time.monotonic()
        if (now - self._seam_at) < STITCH_SEAM_EVERY_S:
            return self._seam_cache
        self._seam_at = now
        try:
            self._seam_cache = seam_report(frames, calib)
        except Exception as exc:                 # a diagnostic must never stop the tool
            print(f"[stitch] seam check failed: {exc}")
            self._seam_cache = []
        return self._seam_cache

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
            # Do the cameras AGREE about the sand they can both see? Overlapping
            # depths are averaged, so a per-camera bias shows up as a step at
            # each edge of the overlap band — several times a groove's depth,
            # and enough to make the detrend paint phantom grooves along the
            # seam. Throttled: it re-warps every camera separately, which is the
            # memory the live stitch deliberately avoids holding.
            "seams": [s.to_dict() for s in self._seams(frames, calib)],
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
