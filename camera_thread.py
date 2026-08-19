import threading
import time
from collections import deque
from typing import Optional

import cv2
import numpy as np

from config import (
    CAMERA_LIMIT, CAMERA_POLL_IDLE_S, CAPTURE_REFILL_MAX_S,
    DEPTH_AVERAGE_FRAMES, DEPTH_FPS, DEPTH_LABELS_EVERY,
    DEPTH_LABELS_INTERVAL_MM,
    LIVE_DEPTH_EMA_ALPHA, LIVE_DEPTH_HOLD_CYCLES, LIVE_GROOVE_EVERY,
    LIVE_MASK_HYSTERESIS, PROFILE_EVERY_S, PROFILE_PIPELINE,
    PROJECTION_CHANGE_PX, PROJECTION_EVERY_S, PROJECTION_KEEPALIVE_S,
    STITCH_MAIN_BIND_TIMEOUT_S, STITCH_MAIN_EVERY_S, STITCH_MAX_CAMERAS,
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
# Anti-flicker for the LIVE mask only (config.LIVE_* — see the note there).
_EMA_ALPHA = min(max(float(LIVE_DEPTH_EMA_ALPHA), 0.0), 1.0)
_HOLD_CYCLES = max(int(LIVE_DEPTH_HOLD_CYCLES), 0)
_HYSTERESIS = min(max(float(LIVE_MASK_HYSTERESIS), 0.0), 1.0)
_PROJ_EVERY_S = max(float(PROJECTION_EVERY_S), 0.0)
_PROJ_CHANGE_PX = max(int(PROJECTION_CHANGE_PX), 0)
# How many cameras to open. config.CAMERA_LIMIT is a diagnostic cap (0 = all),
# so a scaling test can run the same rig as 1, 2 or 3 cameras; the hard ceiling
# stays STITCH_MAX_CAMERAS.
_MAX_CAMERAS = min(CAMERA_LIMIT, STITCH_MAX_CAMERAS) if CAMERA_LIMIT > 0 \
    else STITCH_MAX_CAMERAS
# What the buffers were always MEANT to span (DEPTH_AVERAGE_FRAMES at the
# camera's own rate). `refill_seconds` never returns less than this, so the
# measurement can only ever lengthen a wait, never shorten one below the value
# the code was written against.
_REFILL_NOMINAL_S = DEPTH_AVERAGE_FRAMES / max(float(DEPTH_FPS), 1.0)
_REFILL_MAX_S = max(float(CAPTURE_REFILL_MAX_S), _REFILL_NOMINAL_S)
_POLL_IDLE_S = max(float(CAMERA_POLL_IDLE_S), 0.0)


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
        # `_stamps` records WHEN each buffered frame arrived, so how far back
        # the average actually reaches is measured rather than assumed — see
        # `refill_seconds`.
        self._buffers: list[deque[tuple[np.ndarray, np.ndarray]]] = []
        self._stamps: list[deque[float]] = []
        self._frames_read = 0          # written by the reader thread only
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
        # Anti-flicker memory for the LIVE detection ONLY: a running average of
        # the canvas depth (+ how many cycles each pixel has been missing) and
        # the previous mask, which the hysteresis deadband holds on to.
        # `capture_frame` shares none of it — the robot's path is judged on a
        # freshly averaged still, exactly as before.
        self._z_avg: Optional[np.ndarray] = None
        self._ok_avg: Optional[np.ndarray] = None
        self._stale: Optional[np.ndarray] = None
        self._prev_mask: Optional[np.ndarray] = None
        # The last PICTURE actually encoded for each mask-family stream, with
        # when it went out — so a view that is holding still is encoded once and
        # then left alone instead of being rebuilt every canvas.
        self._sent: dict[str, tuple[np.ndarray, float]] = {}
        # Diagnostic only (config.PROFILE_PIPELINE): every live view is produced
        # on THIS thread, so whichever stage is slow delays all the others. The
        # target rate is what STITCH_MAIN_EVERY_S asks for; the achieved rate is
        # what the work actually allows.
        self._timer = StageTimer("canvas", enabled=PROFILE_PIPELINE,
                                 every_s=PROFILE_EVERY_S,
                                 target_hz=1.0 / max(STITCH_MAIN_EVERY_S, 1e-6))
        # The reader runs on its OWN thread now, so it needs its own timer —
        # `+=` on a timer's slots is not atomic and two threads sharing one
        # would lose samples silently. Its target is the camera's frame rate,
        # which is the whole point: it must keep up with the cameras, not with
        # the canvas.
        self._read_timer = StageTimer("reader", enabled=PROFILE_PIPELINE,
                                      every_s=PROFILE_EVERY_S,
                                      target_hz=float(DEPTH_FPS))

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

    def _forget_live_mask(self) -> None:
        """
        Drop what the live mask is remembering: the hysteresis memory, and the
        record of which picture each stream last sent. Called whenever the thing
        being detected changes underneath it — new parameters, a new crop, a new
        reference — so a slider takes effect on the very next frame instead of
        being argued with by a mask holding on to the old answer, and so the
        first frame after the change is always encoded and pushed.
        """
        self._prev_mask = None
        self._sent = {}

    def set_live_params(self, params: DepthGrooveParams) -> None:
        """Update the params used for the live depth colormap + groove preview."""
        self._live_params = params
        self._forget_live_mask()

    def set_live_crop(self, crop: Crop) -> None:
        """Restrict the live groove/mask preview to this normalized crop region."""
        self._live_crop = crop
        self._forget_live_mask()

    def set_reference(self, depth_m: Optional[np.ndarray]) -> None:
        """Set (or clear with None) the baseline depth frame for background subtraction."""
        self._reference = depth_m
        self._forget_live_mask()

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
        # The live averages are canvas-shaped pictures of the OLD orientation.
        # A 180° turn keeps the shape, so the shape check alone would happily
        # blend the old picture into the new one — drop them explicitly.
        self._z_avg = self._ok_avg = self._stale = None
        self._forget_live_mask()
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

    def refill_seconds(self) -> float:
        """
        How long until every frame now in the rolling buffers has been replaced.

        Capture averages the WHOLE buffer, so this is how long a caller must
        wait before that average is guaranteed free of whatever is in the box
        right now — a participant's hand, or the projector's light on the sand.

        It is MEASURED, from the timestamps of the buffered frames, and not
        derived from DEPTH_AVERAGE_FRAMES / DEPTH_FPS. Those two agree only
        while every camera frame is being caught. They stop agreeing the moment
        one is missed, and then the derived figure is silently too short: the
        buffer still holds 30 frames, but they now reach several seconds back
        instead of one, and a wait sized for one second leaves the hand in the
        picture the robot draws from.

        Reported for the SLOWEST camera, since the average is only clean once
        every buffer is. Never below the nominal value — so this can lengthen a
        wait but never shorten one — and bounded above, so a camera that has
        stopped delivering cannot hang Participant Mode indefinitely.
        """
        now = time.monotonic()
        worst = 0.0
        with self._frame_lock:
            for stamps in self._stamps:
                if len(stamps) < 2:
                    continue
                # Mean interval between arrivals x the buffer's depth: correct
                # whether the buffer is full or still filling.
                interval = (stamps[-1] - stamps[0]) / (len(stamps) - 1)
                worst = max(worst, interval * DEPTH_AVERAGE_FRAMES)
                # A camera that has gone quiet since its last frame is not
                # described by its own history — count the silence too.
                worst = max(worst, now - stamps[-1])
        if worst <= 0.0:
            return _REFILL_NOMINAL_S       # nothing buffered yet: assume nominal
        return min(max(worst, _REFILL_NOMINAL_S), _REFILL_MAX_S)

    # ── acquisition ──────────────────────────────────────────────────────────
    def _reader(self, cameras: list) -> None:
        """
        Drain every camera at ITS OWN rate, on its own thread.

        This used to share the canvas loop, and that was fine only while the
        loop had time to spare: it asked each camera ~18 times per canvas and
        caught every frame. On a rig where the canvas work fills the whole
        period it got one ask per canvas, so the buffers filled at the CANVAS
        rate — measured at 9 fps per camera instead of 30 on a two-camera rig.
        Nothing looked broken, because the live views are only rebuilt that
        often anyway; what broke was Capture, whose 30-frame average silently
        stretched from one second of history to three.

        The buffers are what the robot's still is built from. They must not be
        paced by how long a canvas takes to draw.
        """
        timer = self._read_timer
        while not self._stop_event.is_set():
            got = False
            with timer.stage("poll"):
                for i, camera in enumerate(cameras):
                    frame = realsense_source.poll(camera)
                    if frame is None:
                        continue
                    z, ok, rgb = frame
                    stamp = time.monotonic()
                    with self._frame_lock:
                        if i >= len(self._buffers):
                            continue          # shutting down
                        self._buffers[i].append((z, ok))
                        self._stamps[i].append(stamp)
                        if rgb is not None:
                            self._last_rgb[i] = rgb
                    got = True
                    self._frames_read += 1
            if not got:
                # Nothing ready: wait briefly on the stop event rather than
                # sleeping, so shutdown stays immediate.
                self._stop_event.wait(_POLL_IDLE_S)
                continue
            line = timer.cycle()
            if line:
                print(line)

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
        cameras, note = realsense_source.open_cameras(_MAX_CAMERAS)
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
        self._z_avg = self._ok_avg = self._stale = None
        self._forget_live_mask()
        self._frames_read = 0
        with self._frame_lock:
            self._buffers = [deque(maxlen=DEPTH_AVERAGE_FRAMES) for _ in cameras]
            self._stamps = [deque(maxlen=DEPTH_AVERAGE_FRAMES) for _ in cameras]
            self._last_rgb = [None] * len(cameras)
            self._serials = [c.serial for c in cameras]
            self._intrinsics = [c.intr for c in cameras]
        print(f"[depth] started {len(cameras)} RealSense camera(s) "
              f"({', '.join(c.serial for c in cameras)}) — combined view")
        if CAMERA_LIMIT > 0:
            print(f"[depth] CAMERA_LIMIT={CAMERA_LIMIT} — the rig is capped for "
                  f"this run; unset SANDSKRIPT_CAMERAS to use every camera")
        if PROFILE_PIPELINE:
            print(f"[depth] profiling ON — one report every {PROFILE_EVERY_S:.0f} s")

        # Reading the cameras is deliberately NOT part of this loop — see
        # `_reader`. It must keep pace with the cameras, and this loop's pace is
        # set by how long a canvas takes to draw.
        reader = threading.Thread(target=self._reader, args=(cameras,),
                                  daemon=True, name="depth_camera_reader")

        canvas_i = 0
        started = time.monotonic()
        last_canvas = 0.0
        last_read = 0
        try:
            reader.start()
            while not self._stop_event.is_set():
                now = time.monotonic()
                due = last_canvas + STITCH_MAIN_EVERY_S
                if now < due:
                    # Wait to the DEADLINE, not in fixed lumps. The old loop
                    # woke every 5 ms and fired at the first wake after the
                    # deadline, which cost ~10 ms of period on every rig; and
                    # waiting on the stop event keeps shutdown immediate.
                    self._stop_event.wait(min(due - now, STITCH_MAIN_EVERY_S))
                    continue
                frames = self._latest_frames()
                if not frames:
                    self._stop_event.wait(_POLL_IDLE_S)
                    continue
                # Freeze the canvas only once the whole rig has reported, so a
                # camera that starts a beat late still shapes the geometry.
                if self._grid is None and len(frames) < len(cameras) \
                        and (now - started) < STITCH_MAIN_BIND_TIMEOUT_S:
                    self._stop_event.wait(_POLL_IDLE_S)
                    continue
                last_canvas = now

                if self._grid is None:
                    self._bind_calib(frames)
                with self._timer.stage("stitch"):
                    result = stitch(frames, self._calib, grid=self._grid,
                                    timer=self._timer)
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
                    # Name the rig in every profile report from here on: camera
                    # count and canvas size are exactly what a scaling test
                    # varies, and a report pasted out of a terminal has to say
                    # which of the runs it belongs to.
                    self._timer.set_context(
                        f"{len(cameras)} cam | {pw}x{ph} px "
                        f"| {result.mm_per_px:.2f} mm/px")
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
                # Camera frames read since the last canvas. Expected is
                # DEPTH_FPS x cameras / canvas rate; a shortfall means the
                # reader is not keeping up, which stretches the window Capture
                # averages over — the whole reason it lives on its own thread.
                # Counted HERE, on the timer's own thread, from a value the
                # reader only ever increments.
                read = self._frames_read
                self._timer.count("frames_read", read - last_read)
                last_read = read
                # One canvas = one cycle. The report lands here rather than on a
                # clock of its own, so the numbers describe whole cycles.
                line = self._timer.cycle()
                if line:
                    print(line)
        finally:
            # Stop the reader BEFORE closing the cameras it is polling — a poll
            # on a stopped pipeline is the one ordering that must not happen.
            self._stop_event.set()
            if reader.is_alive():
                reader.join(timeout=2.0)
            realsense_source.close(cameras)
            with self._state_lock:
                for key in ("last_depth_color_jpg", "last_depth_crop_jpg", "last_rgb_jpg",
                            "last_groove_jpg", "last_mask_jpg", "last_mask_full_jpg",
                            "depth_labels", "depth_labels_size",
                            "depth_labels_relative", "trigger_below"):
                    self._state[key] = None
            with self._frame_lock:
                self._buffers = []
                self._stamps = []
                self._last_rgb = []
            self._z_avg = self._ok_avg = self._stale = None
            self._forget_live_mask()
            print("[depth] stopped")

    def _steady_depth(self, z: np.ndarray, ok: np.ndarray
                      ) -> tuple[np.ndarray, np.ndarray]:
        """
        A running average of the canvas depth, for DETECTION only.

        Capture already averages DEPTH_AVERAGE_FRAMES before the robot draws,
        but the live mask was thresholding a single raw frame whose per-pixel
        noise is the same size as the relief it looks for — so pixels near a
        groove edge flipped on and off every cycle and the projected mask
        twitched over sand nobody was touching. Averaging is the cure because
        the noise is random and cancels; the sand is not and does not.

        The depth VIEW, the labels and the Participant trigger deliberately keep
        the raw frame: the first is the operator's ground truth and the other
        two must react to a hand the moment it appears, not half a second later.

        A pixel the sensor drops keeps its last value for a few cycles instead
        of punching a hole in the mask — but only a few, so a camera that dies
        still blanks its part of the canvas rather than showing a fossil.
        """
        # 1.0 = off. 0 would mean "never take a new reading", which is not a
        # smoother but a freeze, so it is treated as off too.
        if not (0.0 < _EMA_ALPHA < 1.0):
            return z, ok
        prev, prev_ok, stale = self._z_avg, self._ok_avg, self._stale
        if prev is None or prev.shape != z.shape:
            self._z_avg = z.astype(np.float32, copy=True)
            self._ok_avg = ok.copy()
            self._stale = np.zeros(z.shape, np.uint8)
            return self._z_avg, self._ok_avg

        both = ok & prev_ok
        prev[both] += _EMA_ALPHA * (z[both] - prev[both])
        fresh = ok & ~prev_ok
        prev[fresh] = z[fresh]                 # nothing to blend with yet

        # Count how long each pixel has been missing, saturating rather than
        # wrapping — a uint8 rolling 255 → 0 would resurrect a dead camera.
        stale[ok] = 0
        grew = ~ok & (stale < 255)
        stale[grew] += 1
        self._ok_avg = (prev_ok | ok) & (stale <= _HOLD_CYCLES)
        return prev, self._ok_avg

    def _needs_resend(self, key: str, picture: np.ndarray, now: float,
                      min_interval: float = 0.0, tol_px: int = 0) -> bool:
        """
        Has this stream's picture changed enough to be worth encoding again?

        `_mjpeg_stream` skips writing a JPEG it has already written, but it
        compares the OBJECT — and a fresh encode is always a fresh object, so a
        picture standing perfectly still was re-sent every canvas anyway. Every
        one of those costs the browser a decode, and the projection window a
        re-composite of a full-screen warped layer. Comparing the pixels lets a
        still picture be sent once; leaving the cached JPEG in place is what
        makes the stream go quiet, because the object then stays identical.

        Only worthwhile because the live mask is steady now — a shimmering one
        would differ every frame and never match.

        `tol_px` allows a few pixels to differ without counting as a change —
        for the projector, where that much is invisible on the sand. It cannot
        let the picture drift, because the comparison is always against the last
        picture SENT: small differences pile up against a fixed reference until
        they cross, rather than being forgiven one cycle at a time.
        """
        prev = self._sent.get(key)
        if prev is None:
            return True
        last, at = prev
        if (now - at) >= PROJECTION_KEEPALIVE_S:
            return True          # a silent stream must not look like a dead one
        if last.shape == picture.shape:
            if tol_px > 0:
                if int(np.count_nonzero(last != picture)) <= tol_px:
                    return False
            elif np.array_equal(last, picture):
                return False     # same picture — the browser already has it
        return (now - at) >= min_interval

    def _mark_sent(self, key: str, picture: np.ndarray, now: float) -> None:
        self._sent[key] = (picture, now)

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
            #
            # Detection alone runs on the STEADIED depth and remembers the last
            # mask, the two dampers that stop the projected picture shimmering.
            # Both are live-only: nothing here reaches `capture_frame`.
            with self._timer.stage("steady"):
                z_det, ok_det = self._steady_depth(z, ok)
            with self._timer.stage("detect"):
                mask, skel = grooves_and_mask(
                    z_det[y0:y1, x0:x1], ok_det[y0:y1, x0:x1], params, ref_sub,
                    self._mm_per_px, self._prev_mask, _HYSTERESIS,
                    timer=self._timer,
                )
            self._prev_mask = mask

            # Encode only what actually changed. Leaving the cached JPEG in
            # place is the whole mechanism: `_mjpeg_stream` compares the object,
            # so an unchanged one makes the stream go quiet — which is what
            # stops the browser decoding and re-compositing the same picture ten
            # times a second.
            now = time.monotonic()
            with self._timer.stage("encode_mask"):
                if self._needs_resend("skel", skel, now):
                    sj = encode_jpeg(skel)
                    if sj is not None:
                        last_groove_jpg = sj
                        self._mark_sent("skel", skel, now)
                else:
                    self._timer.count("skel.held")
                if self._needs_resend("mask", mask, now):
                    mj = encode_jpeg(mask)
                    if mj is not None:
                        last_mask_jpg = mj
                        self._mark_sent("mask", mask, now)
                else:
                    self._timer.count("mask.held")

            # Full-canvas mask for the projector: the cropped mask pasted back
            # at its canvas position, so the projection homography has stable
            # coordinates regardless of the crop. Composed ONLY while a
            # projection window is connected.
            #
            # The comparison is on the ASSEMBLED canvas, not the cropped mask,
            # because moving the crop moves where an otherwise identical mask
            # lands — and the projector would keep showing it in the old place.
            # Building it is cheap (~0.05 ms); the encode is what we are saving.
            with self._state_lock:
                proj_on = self._state.get("projection_clients", 0) > 0
            if proj_on:
                with self._timer.stage("mask_full"):
                    full = np.zeros(z.shape, np.uint8)
                    full[y0:y1, x0:x1] = mask
                    if self._needs_resend("mask_full", full, now, _PROJ_EVERY_S,
                                          _PROJ_CHANGE_PX):
                        fj = encode_jpeg(full)
                        if fj is not None:
                            last_mask_full_jpg = fj
                            self._mark_sent("mask_full", full, now)
                    else:
                        self._timer.count("mask_full.held")
            else:
                # A window that reconnects must get a picture at once, not at
                # the next change — it has nothing on screen to hold.
                last_mask_full_jpg = None
                self._sent.pop("mask_full", None)

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
