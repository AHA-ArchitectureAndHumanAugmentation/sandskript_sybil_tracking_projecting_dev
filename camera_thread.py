import threading
import time
from collections import deque
from typing import Optional

import cv2
import numpy as np

from config import (
    DEPTH_AVERAGE_FRAMES, DEPTH_LABELS_EVERY, DEPTH_LABELS_INTERVAL_MM,
    LIVE_GROOVE_EVERY, PROFILE_EVERY_S, PROFILE_PIPELINE,
    STITCH_MAIN_BIND_TIMEOUT_S, STITCH_MAIN_EVERY_S,
)
from depth_extractor import (
    Crop, DepthGrooveParams, colorize_depth, depth_region_labels,
    grooves_and_mask, encode_jpeg, presence_trigger,
)
from profiling import StageTimer
import realsense_source
from stitcher import (
    CameraFrame, CanvasGrid, StitchCalib, bind_placements, footprint_mm,
    load_calib, rotate_frame, stitch, with_default_row,
)
from view_rotation import norm_deg, rotate_image, rotate_size

# How often (in canvases) to recompute the live groove preview — now
# config.LIVE_GROOVE_EVERY, since it is one of the two knobs that decide how
# fresh the live views are and it belongs beside STITCH_MAIN_EVERY_S.
_LIVE_GROOVE_EVERY = max(int(LIVE_GROOVE_EVERY), 1)


class DepthCameraThread:
    """
    Captures depth + colour from EVERY Intel RealSense (D435i) on the rig, lays
    them onto one combined canvas using the layout saved by Multi-Cam Vision
    (stitch_calibration.json), and produces three JPEG streams from it:
      - depth:   the combined depth map colorized so depth reads as colour
      - rgb:     the combined aligned colour image
      - grooves: detected groove centrelines (live preview of what gets drawn)

    The canvas — not any single camera's frame — is what the whole pipeline
    sees, so a crop, a reference frame, the Participant-Mode trigger and the
    captured still all live in canvas pixels. One camera is a perfectly valid
    rig; the canvas is then that camera's frame through its placement.

    All three streams are stored in shared_state and served as MJPEG by the
    server. The colour stream is aligned to depth per camera before stitching,
    so a crop in normalized coordinates selects the same region in both. The raw
    metric depth of recent frames is buffered PER CAMERA at the full frame rate
    so Capture can average (~1 s, noise ↓√N) and stitch the averages — the
    single biggest win for resolving sub-millimetre grooves. A separate lock
    guards the buffers so capture_frame() doesn't block MJPEG delivery.

    The canvas geometry is frozen (`CanvasGrid`) once every camera has delivered
    its first frame, and never changes afterwards: a canvas that resized mid
    session would move the crop, the reference and the captured still with it.

    On top of that frozen geometry sits the VIEW ROTATION (`set_view_rotation`,
    Developer Mode's ⟳ button): a quarter-turn applied to the finished canvas on
    its way out, in `_publish` and `capture_frame` alike. Because it is applied
    at that single seam, every view and the captured still turn together — but a
    quarter turn does swap the frame's width and height, which the mm-per-pixel
    scale and the surface fit both read, so `set_view_rotation` republishes
    `frame_size` and main.py re-bases the crop and the reference frame on it.

    The live groove preview honours `set_live_params()` so the browser's Detect
    Grooves controls update the feed in real time, before any image is captured.
    """

    def __init__(self, shared_state: dict, state_lock: threading.Lock) -> None:
        self._state = shared_state
        self._state_lock = state_lock
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        # Per-camera rolling buffers of (depth_m float32, valid bool) for
        # temporal averaging, plus each camera's latest aligned colour frame.
        self._buffers: list[deque[tuple[np.ndarray, np.ndarray]]] = []
        self._last_rgb: list[Optional[np.ndarray]] = []
        self._serials: list[str] = []
        self._intrinsics: list = []
        # The saved layout, bound to the cameras actually present, and the
        # canvas geometry frozen from the first complete stitch.
        self._calib = StitchCalib()
        self._grid: Optional[CanvasGrid] = None
        # Live detection params + crop (atomically swapped by the setters).
        self._live_params = DepthGrooveParams()
        self._live_crop = Crop()
        self._reference: Optional[np.ndarray] = None   # baseline depth for subtraction
        self._mm_per_px: Optional[float] = None        # workspace scale for mm filters
        self._label_interval_mm = DEPTH_LABELS_INTERVAL_MM  # depth-number overlay band
        self._trigger_mm: Optional[float] = None       # Participant-Mode trigger threshold
        self._rotation = 0                             # view rotation, 0/90/180/270 CW
        # Last encoded groove/mask/projector JPEGs — kept between canvases so the
        # throttled previews hold their picture instead of blinking.
        self._cached: tuple = (None, None, None)
        # Diagnostic only (config.PROFILE_PIPELINE): every live view is produced
        # on THIS thread, so whichever stage is slow delays all the others. The
        # target rate is what STITCH_MAIN_EVERY_S asks for; the achieved rate is
        # what the work actually allows.
        self._timer = StageTimer("canvas", enabled=PROFILE_PIPELINE,
                                 every_s=PROFILE_EVERY_S,
                                 target_hz=1.0 / max(STITCH_MAIN_EVERY_S, 1e-6))

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def camera_count(self) -> int:
        """How many cameras are combined into the canvas (0 before start-up)."""
        with self._frame_lock:
            return len(self._serials)

    @property
    def frame_size(self) -> Optional[tuple[int, int]]:
        """(width, height) of the combined canvas AS PUBLISHED, or None before it
        is frozen. This is the frame size the whole pipeline maps from —
        mm-per-pixel and the surface fit both need it, it is NOT 640×480 any
        more, and on a quarter view rotation width and height are swapped."""
        grid = self._grid
        return (None if grid is None
                else rotate_size((grid.width, grid.height), self._rotation))

    @property
    def view_rotation(self) -> int:
        """Current view rotation in clockwise degrees (0/90/180/270)."""
        return self._rotation

    def start(self, index: Optional[int] = None) -> None:
        # `index` is accepted for API compatibility but ignored — cameras are
        # selected by the SDK (all of them), not by an OpenCV device index.
        if self.running:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="depth_camera_thread")
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3.0)
            self._thread = None

    def set_live_params(self, params: DepthGrooveParams) -> None:
        """Update the params used for the live depth colormap + groove preview."""
        self._live_params = params

    def set_live_crop(self, crop: Crop) -> None:
        """Restrict the live groove/mask preview to this normalized crop region."""
        self._live_crop = crop

    def set_reference(self, depth_m: Optional[np.ndarray]) -> None:
        """Set (or clear with None) the baseline depth frame for background subtraction."""
        self._reference = depth_m

    def set_scale(self, mm_per_px: Optional[float]) -> None:
        """Set the workspace scale so the live mm-based width/length filters work."""
        self._mm_per_px = mm_per_px

    def set_depth_label_interval(self, interval_mm: float) -> None:
        """Band width (mm) for the depth-number overlay's iso-depth regions."""
        self._label_interval_mm = interval_mm

    def set_trigger_threshold(self, mm: Optional[float]) -> None:
        """Participant-Mode trigger distance (mm from camera); None disables."""
        self._trigger_mm = mm

    def set_view_rotation(self, deg) -> int:
        """
        Turn the published canvas by a quarter-turn multiple (clockwise) and
        return the angle actually set. Every view and the captured still come
        out of the same seam, so they all turn together.

        Republishes `frame_size`, because a quarter turn swaps width and height
        and the mm-per-pixel scale divides by width. The canvas GEOMETRY is
        untouched — the frozen grid, the layout file and the stitch are all
        exactly as before; this only re-indexes the finished picture, so it is
        safe to change mid session in a way that re-freezing never would be.
        """
        self._rotation = norm_deg(deg)
        size = self.frame_size
        if size is not None:
            with self._state_lock:
                self._state["frame_size"] = [size[0], size[1]]
        return self._rotation

    def capture_frame(self) -> Optional[tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]]:
        """
        Return a temporally averaged, combined (depth_m, valid, rgb) over the
        buffered frames, or None if no camera has produced one yet. ``rgb`` is
        the stitched aligned colour, or None if no colour arrived at all.

        Each camera is averaged on its OWN buffer and the averages are stitched,
        so noise is cut per camera (~sqrt(n_frames)) before any warping — the
        opposite order would average an already-resampled canvas. The view
        rotation is applied last, exactly as it is for the live views, so a
        captured still is in the same orientation as the picture it was taken
        from.
        """
        if self._grid is None:
            return None            # canvas geometry not settled yet — not ready
        with self._frame_lock:
            buffers = [list(b) for b in self._buffers]
            rgbs = [None if r is None else r.copy() for r in self._last_rgb]
            intrinsics = list(self._intrinsics)
            serials = list(self._serials)
        frames = []
        for i, buffer in enumerate(buffers):
            if not buffer:
                continue
            depth_m, valid = _average(buffer)
            frames.append(CameraFrame(depth_m, valid, intrinsics[i], rgbs[i], serials[i]))
        if not frames:
            return None
        result = stitch(frames, self._calib, grid=self._grid)
        rgb = result.rgb if result.rgb_valid.any() else None
        rot = self._rotation          # read once: all three must agree
        return (rotate_image(result.depth_m, rot), rotate_image(result.valid, rot),
                rotate_image(rgb, rot))

    # ── acquisition ──────────────────────────────────────────────────────────
    def _latest_frames(self) -> list[CameraFrame]:
        """The most recent frame from every camera that has one, ready to stitch."""
        with self._frame_lock:
            latest = [(b[-1] if b else None) for b in self._buffers]
            rgbs = list(self._last_rgb)
            intrinsics = list(self._intrinsics)
            serials = list(self._serials)
        return [CameraFrame(pair[0], pair[1], intrinsics[i], rgbs[i], serials[i])
                for i, pair in enumerate(latest) if pair is not None]

    def _bind_calib(self, frames: list[CameraFrame]) -> None:
        """
        Match the saved layout to the cameras actually plugged in (by serial),
        and give anything unplaced its slot in the default row. Done ONCE, at
        start-up: unlike the placement tool, nothing here may re-bind mid
        session — the pipeline's frame size depends on it.
        """
        bound = bind_placements(load_calib(), frames)
        footprints = [footprint_mm(rotate_frame(f, bound.placement_for(f.serial, i).rot_deg))
                      for i, f in enumerate(frames)]
        self._calib = with_default_row(bound, footprints)

    def _run(self) -> None:
        cameras, note = realsense_source.open_cameras()
        if not cameras:
            print(f"[depth] ERROR: {note}")
            return
        if note:
            print(f"[depth] {note}")

        # A restarted thread re-reads the layout and re-freezes the canvas: the
        # rig may have changed while it was stopped.
        self._grid = None
        self._calib = StitchCalib()
        self._cached = (None, None, None)
        with self._frame_lock:
            self._buffers = [deque(maxlen=DEPTH_AVERAGE_FRAMES) for _ in cameras]
            self._last_rgb = [None] * len(cameras)
            self._serials = [c.serial for c in cameras]
            self._intrinsics = [c.intr for c in cameras]
        print(f"[depth] started {len(cameras)} RealSense camera(s) "
              f"({', '.join(c.serial for c in cameras)}) — combined view")

        canvas_i = 0
        started = time.monotonic()
        last_canvas = 0.0
        try:
            while not self._stop_event.is_set():
                got = False
                with self._timer.stage("poll_cameras"):
                    for i, camera in enumerate(cameras):
                        frame = realsense_source.poll(camera)
                        if frame is None:
                            continue
                        z, ok, rgb = frame
                        with self._frame_lock:
                            self._buffers[i].append((z, ok))
                            if rgb is not None:
                                self._last_rgb[i] = rgb
                        got = True
                if not got:
                    time.sleep(0.005)

                now = time.monotonic()
                if (now - last_canvas) < STITCH_MAIN_EVERY_S:
                    continue
                frames = self._latest_frames()
                if not frames:
                    continue
                # Freeze the canvas only once the whole rig has reported, so a
                # camera that starts a beat late still shapes the geometry.
                if self._grid is None and len(frames) < len(cameras) \
                        and (now - started) < STITCH_MAIN_BIND_TIMEOUT_S:
                    continue
                last_canvas = now

                if self._grid is None:
                    self._bind_calib(frames)
                with self._timer.stage("stitch"):
                    result = stitch(frames, self._calib, grid=self._grid)
                if self._grid is None:
                    self._grid = CanvasGrid.from_result(result)
                    gh, gw = result.depth_m.shape[:2]
                    # frame_size is what the pipeline sees, i.e. AFTER the view
                    # rotation — the stitch size is only half the story once the
                    # canvas is turned.
                    pw, ph = self.frame_size
                    turned = (f" → published {pw}×{ph} (turned {self._rotation}°)"
                              if self._rotation else "")
                    print(f"[depth] combined canvas {gw}×{gh} px "
                          f"@ {result.mm_per_px:.2f} mm/px from {len(frames)} camera(s)"
                          + turned)
                    with self._state_lock:
                        self._state["frame_size"] = [pw, ph]
                        self._state["camera_count"] = len(cameras)

                rot = self._rotation   # read once: the three views must agree
                rgb = result.rgb if result.rgb_valid.any() else None
                with self._timer.stage("view_rotation"):
                    z_pub = rotate_image(result.depth_m, rot)
                    ok_pub = rotate_image(result.valid, rot)
                    rgb_pub = rotate_image(rgb, rot)
                self._publish(z_pub, ok_pub, rgb_pub, canvas_i)
                canvas_i += 1
                # One canvas = one cycle. The report lands here rather than on a
                # clock of its own, so the numbers describe whole cycles.
                line = self._timer.cycle()
                if line:
                    print(line)
        finally:
            realsense_source.close(cameras)
            with self._state_lock:
                for key in ("last_depth_color_jpg", "last_depth_crop_jpg", "last_rgb_jpg",
                            "last_groove_jpg", "last_mask_jpg", "last_mask_full_jpg",
                            "depth_labels", "depth_labels_size",
                            "depth_labels_relative", "trigger_below"):
                    self._state[key] = None
            with self._frame_lock:
                self._buffers = []
                self._last_rgb = []
            print("[depth] stopped")

    # ── one canvas → every live view ─────────────────────────────────────────
    def _publish(self, z: np.ndarray, ok: np.ndarray, rgb: Optional[np.ndarray],
                 canvas_i: int) -> None:
        """
        Derive everything the browser sees from ONE combined canvas: the
        colorized depth view, the cropped popup view, colour, the groove/mask
        previews, the projector's full-frame mask, the depth-number labels and
        the Participant-Mode trigger flag.

        The arrays arrive already turned by the view rotation, so every view
        below inherits it — including the crop, which is normalized and
        therefore lands on the turned canvas exactly where main.py re-based it.
        """
        last_groove_jpg, last_mask_jpg, last_mask_full_jpg = self._cached
        params = self._live_params
        h, w = z.shape[:2]
        x0, y0, x1, y1 = self._live_crop.pixel_box(w, h)

        # The baseline sand, cropped to match — the ONE thing that lets the
        # trigger, the depth labels and near-object rejection all speak in
        # height above the sand instead of distance from the camera, which is
        # what a tilted rig needs. None (no reference, or a stale shape after a
        # view rotation) simply means every one of them falls back to absolute.
        ref = self._reference
        ref_sub = (ref[y0:y1, x0:x1]
                   if (ref is not None and ref.shape == z.shape) else None)

        # Colorized depth (FULL canvas — the crop box overlays it client-side).
        with self._timer.stage("colorize"):
            color = colorize_depth(z, ok, params.near_m, params.far_m)
        with self._timer.stage("encode_depth"):
            ok_color, color_jpg = cv2.imencode(
                ".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 80])

        # Participant popup: the SAME colorized depth but restricted to the
        # Developer-Mode crop, so the popup shows exactly the region paths are
        # generated from (like the skeleton/mask views). Composed only while a
        # popup is connected; the popup never changes the crop — only users
        # adjust it in Developer Mode.
        with self._state_lock:
            overlay_on = self._state.get("depth_overlay_clients", 0) > 0
        with self._timer.stage("encode_crop"):
            crop_jpg = encode_jpeg(color[y0:y1, x0:x1]) if overlay_on else None

        # Combined colour: buffer the FULL canvas for Capture, but serve the
        # live "rgb" view CROPPED so only the selected region shows.
        with self._timer.stage("encode_rgb"):
            rgb_jpg = encode_jpeg(rgb[y0:y1, x0:x1]) if rgb is not None else None

        # Live groove + mask preview, restricted to the live crop (throttled).
        if canvas_i % _LIVE_GROOVE_EVERY == 0:
            # Runs every _LIVE_GROOVE_EVERY-th cycle, so expect roughly half the
            # cycle count here — and note this is INLINE, so whatever it costs is
            # added to the canvas period and therefore delays the depth view too.
            with self._timer.stage("detect"):
                mask, skel = grooves_and_mask(
                    z[y0:y1, x0:x1], ok[y0:y1, x0:x1], params, ref_sub, self._mm_per_px
                )
            with self._timer.stage("encode_mask"):
                sj, mj = encode_jpeg(skel), encode_jpeg(mask)
            if sj is not None:
                last_groove_jpg = sj
            if mj is not None:
                last_mask_jpg = mj

            # Full-canvas mask for the projector: the cropped mask pasted back
            # at its canvas position, so the projection homography has stable
            # coordinates regardless of the crop. Composed ONLY while a
            # projection window is connected.
            with self._state_lock:
                proj_on = self._state.get("projection_clients", 0) > 0
            if proj_on:
                with self._timer.stage("mask_full"):
                    full = np.zeros(z.shape, np.uint8)
                    full[y0:y1, x0:x1] = mask
                    fj = encode_jpeg(full)
                if fj is not None:
                    last_mask_full_jpg = fj
            else:
                last_mask_full_jpg = None

        # Depth-number overlay labels: computed ONLY while a /depths popup is
        # connected (zero overhead otherwise), throttled harder than the groove
        # preview — it's a reference display. Labels cover the CROPPED region
        # only (coords relative to the crop, matching the popup's stream).
        if canvas_i % DEPTH_LABELS_EVERY == 0:
            with self._timer.stage("depth_labels"):
                labels = (depth_region_labels(z[y0:y1, x0:x1], ok[y0:y1, x0:x1],
                                              self._label_interval_mm,
                                              reference=ref_sub)
                          if overlay_on else None)
            with self._state_lock:
                self._state["depth_labels"] = labels
                self._state["depth_labels_size"] = (
                    [x1 - x0, y1 - y0] if labels is not None else None)
                # Which quantity those numbers are, so the popup can say so
                # rather than leaving the operator to guess.
                self._state["depth_labels_relative"] = ref_sub is not None

        # Participant-Mode trigger: is anything there? One vectorized compare per
        # canvas. Watches ONLY the cropped region — the popup shows just the
        # crop, so motion outside it must not arm/hold it. With a reference the
        # threshold is a height above the sand (tilt-proof); without one it is
        # the old absolute distance from the camera.
        thr = self._trigger_mm
        with self._timer.stage("trigger"):
            trigger_below = (presence_trigger(z[y0:y1, x0:x1], ok[y0:y1, x0:x1], thr,
                                              reference=ref_sub)
                             if thr is not None else None)
        with self._state_lock:
            self._state["trigger_below"] = trigger_below
            if ok_color:
                self._state["last_depth_color_jpg"] = color_jpg.tobytes()
                self._state["last_depth_crop_jpg"] = crop_jpg
                self._state["last_rgb_jpg"] = rgb_jpg
                self._state["last_groove_jpg"] = last_groove_jpg
                self._state["last_mask_jpg"] = last_mask_jpg
                self._state["last_mask_full_jpg"] = last_mask_full_jpg

        self._cached = (last_groove_jpg, last_mask_jpg, last_mask_full_jpg)


def _average(buffer: list[tuple[np.ndarray, np.ndarray]]
             ) -> tuple[np.ndarray, np.ndarray]:
    """Temporal average of one camera's (depth, valid) buffer, ignoring dropouts."""
    acc = np.zeros_like(buffer[0][0], dtype=np.float32)
    cnt = np.zeros_like(acc, dtype=np.float32)
    for z, ok in buffer:
        acc[ok] += z[ok]
        cnt += ok
    valid = cnt > 0
    depth_m = np.zeros_like(acc)
    depth_m[valid] = acc[valid] / cnt[valid]
    return depth_m, valid
