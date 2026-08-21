"""
Unit tests for the live mask's two anti-flicker dampers.

A D435i's per-pixel depth noise is about the size of the relief a raked groove
leaves, so thresholding ONE raw frame makes every pixel near a groove edge a
coin flip — which on the projector reads as a mask twitching in width and
length over sand nobody is touching. Two fixes, both LIVE ONLY:

  1. `DepthCameraThread._steady_depth` — a running average of the canvas depth
     used for detection. Random noise cancels; the sand does not.
  2. `grooves_and_mask(prev_mask=, hysteresis=)` — a thermostat-style deadband,
     so a lit pixel is not dropped the instant its relief dips a hair.

The property that matters most is the last one in this file: neither damper may
reach the captured still, because that is what the robot actually draws from.
No hardware — synthetic sand throughout.
"""
import threading

import numpy as np
import pytest

from camera_thread import DepthCameraThread
from config import GROOVE_DEPTH_MM
from depth_extractor import (
    Crop, DepthGrooveParams, _threshold_relief, grooves_and_mask,
)

HYST = 0.7          # release threshold as a fraction of the entry one
# A groove the detector rejects on its own but which still clears the release
# threshold — i.e. exactly the pixel that used to flicker. The smoothing pass
# costs a groove ~11% of its amplitude, so this reads ~1.2 mm of relief against
# an entry of GROOVE_DEPTH_MM (1.5) and a release of 1.05.
FAINT_MM = 1.4
DEEP_MM = 4.0


def _sand(depth_mm: float, size: int = 200, width: int = 14) -> np.ndarray:
    """Flat sand at 1 m with one straight groove `depth_mm` deeper."""
    d = np.full((size, size), 1.0, np.float32)
    lo, hi = size // 2 - width // 2, size // 2 + width // 2
    d[lo:hi, 20:size - 20] += depth_mm / 1000.0    # farther = a depression
    return d


def _ref(size: int = 200) -> np.ndarray:
    """Flat empty sand at 1 m — the reference frame for relative detection."""
    return np.full((size, size), 1.0, np.float32)


def _lit(mask: np.ndarray) -> int:
    return int((mask > 0).sum())


class TestThresholdStrictness:
    """The one primitive hysteresis is built on: the same relief, judged twice."""

    def test_a_lenient_test_accepts_a_shallower_groove(self):
        relief = np.array([[GROOVE_DEPTH_MM * 0.8]], np.float32)
        p = DepthGrooveParams()
        assert not _threshold_relief(relief, p).any()          # entry: too shallow
        assert _threshold_relief(relief, p, HYST).any()        # release: still counts

    def test_strictness_one_is_the_plain_threshold(self):
        relief = np.linspace(-5, 5, 64, dtype=np.float32).reshape(8, 8)
        p = DepthGrooveParams()
        assert np.array_equal(_threshold_relief(relief, p, 1.0),
                              relief > p.groove_depth_mm)


class TestHysteresisHolds:

    def test_a_fading_groove_is_held_by_the_previous_mask(self):
        """
        The flicker itself: a groove whose measured relief wobbles across the
        threshold. Alone it drops out; remembered, it stays lit.
        """
        deep = grooves_and_mask(_sand(DEEP_MM), reference=_ref())[0]
        assert _lit(deep) > 0, "the deep groove must be detected in the first place"

        faint = _sand(FAINT_MM)
        alone = grooves_and_mask(faint, reference=_ref())[0]
        held = grooves_and_mask(faint, prev_mask=deep, hysteresis=HYST,
                                reference=_ref())[0]

        assert _lit(alone) == 0, "a faint groove should not clear entry on its own"
        assert _lit(held) > 0, "memory should hold what entry rejects"

    def test_memory_cannot_invent_a_groove_on_flat_sand(self):
        """
        Hysteresis lowers the bar; it must not remove it. Sand that has been
        smoothed over releases, however lit it was a moment ago.
        """
        everything = np.full((200, 200), 255, np.uint8)
        flat = np.full((200, 200), 1.0, np.float32)
        held = grooves_and_mask(flat, prev_mask=everything, hysteresis=HYST,
                                reference=_ref())[0]
        assert _lit(held) == 0

    def test_it_is_off_unless_asked_for(self):
        """Both arguments default to off, so every existing caller is unchanged."""
        deep = grooves_and_mask(_sand(DEEP_MM), reference=_ref())[0]
        faint = _sand(FAINT_MM)
        assert np.array_equal(grooves_and_mask(faint, reference=_ref())[0],
                              grooves_and_mask(faint, prev_mask=deep,
                                               reference=_ref())[0])

    def test_a_stale_mask_of_the_wrong_shape_is_ignored(self):
        """A crop resize must not crash the live view."""
        faint = _sand(FAINT_MM)
        wrong = np.full((50, 50), 255, np.uint8)
        out = grooves_and_mask(faint, prev_mask=wrong, hysteresis=HYST,
                               reference=_ref())[0]
        assert out.shape == faint.shape


class TestSteadyDepth:

    def _cam(self):
        return DepthCameraThread({}, threading.Lock())

    def test_the_first_frame_passes_straight_through(self):
        cam = self._cam()
        z = np.full((8, 8), 1.0, np.float32)
        ok = np.ones((8, 8), bool)
        out, out_ok = cam._steady_depth(z, ok)
        assert np.allclose(out, z) and out_ok.all()

    def test_averaging_cuts_the_noise(self):
        """The whole point: feed the same sand plus noise, get flatter sand."""
        rng = np.random.default_rng(0)
        truth = np.full((64, 64), 1.0, np.float32)
        ok = np.ones((64, 64), bool)
        cam = self._cam()
        noisy = None
        for _ in range(40):
            noisy = truth + rng.normal(0, 0.002, truth.shape).astype(np.float32)
            out, _ = cam._steady_depth(noisy, ok)
        assert out.std() < noisy.std() / 2.0

    def test_it_converges_on_a_change_rather_than_ignoring_it(self):
        """Smoothing must not mean deaf: raked sand still has to show up."""
        cam = self._cam()
        ok = np.ones((16, 16), bool)
        for _ in range(30):
            cam._steady_depth(np.full((16, 16), 1.0, np.float32), ok)
        for _ in range(30):
            out, _ = cam._steady_depth(np.full((16, 16), 1.01, np.float32), ok)
        assert out == pytest.approx(np.full((16, 16), 1.01), abs=1e-3)

    def test_a_dropped_pixel_keeps_its_value_briefly(self):
        """A momentary dropout should not punch a hole in the mask."""
        cam = self._cam()
        z = np.full((8, 8), 1.0, np.float32)
        ok = np.ones((8, 8), bool)
        cam._steady_depth(z, ok)

        gone = ok.copy()
        gone[0, 0] = False
        out, out_ok = cam._steady_depth(np.zeros_like(z), gone)
        assert out_ok[0, 0], "one missing frame should be ridden out"
        assert out[0, 0] == pytest.approx(1.0)

    def test_a_pixel_missing_for_long_enough_goes_blank(self):
        """
        Bounded on purpose: a camera that dies must blank its part of the
        canvas, or a dead feed is indistinguishable from a live one.
        """
        from camera_thread import _HOLD_CYCLES
        cam = self._cam()
        z = np.full((8, 8), 1.0, np.float32)
        cam._steady_depth(z, np.ones((8, 8), bool))
        dead = np.zeros((8, 8), bool)
        for _ in range(_HOLD_CYCLES + 1):
            _, out_ok = cam._steady_depth(z, dead)
        assert not out_ok.any()

    def test_a_changed_canvas_shape_restarts_the_average(self):
        cam = self._cam()
        cam._steady_depth(np.full((8, 8), 1.0, np.float32), np.ones((8, 8), bool))
        z = np.full((4, 12), 2.0, np.float32)
        out, _ = cam._steady_depth(z, np.ones((4, 12), bool))
        assert out.shape == (4, 12) and np.allclose(out, 2.0)


class TestMemoryIsDroppedWhenItWouldBeWrong:
    """A held mask that argues with a slider is worse than a flickering one."""

    def _cam_with_memory(self):
        cam = DepthCameraThread({}, threading.Lock())
        cam._prev_mask = np.full((8, 8), 255, np.uint8)
        return cam

    def test_new_detection_params_clear_it(self):
        cam = self._cam_with_memory()
        cam.set_live_params(DepthGrooveParams(groove_depth_mm=3.0))
        assert cam._prev_mask is None

    def test_a_new_crop_clears_it(self):
        cam = self._cam_with_memory()
        cam.set_live_crop(Crop(0.1, 0.1, 0.5, 0.5))
        assert cam._prev_mask is None

    def test_a_new_reference_clears_it(self):
        cam = self._cam_with_memory()
        cam.set_reference(np.full((8, 8), 1.0, np.float32))
        assert cam._prev_mask is None

    def test_a_view_rotation_clears_both_memories(self):
        """
        180° keeps the canvas SHAPE, so the shape check alone would blend the
        old orientation into the new one.
        """
        cam = self._cam_with_memory()
        cam._z_avg = np.full((8, 8), 1.0, np.float32)
        cam._ok_avg = np.ones((8, 8), bool)
        cam._stale = np.zeros((8, 8), np.uint8)
        cam.set_view_rotation(180)
        assert cam._prev_mask is None and cam._z_avg is None


class TestOnlyChangedPicturesAreSent:
    """
    A steady mask still cost a JPEG encode and a browser decode every canvas,
    because `_mjpeg_stream` compares the JPEG OBJECT and an encode is always a
    new one. On the projection window each of those is a re-composite of a
    full-screen warped layer, which is what put the projector behind the Mask
    view. Every method takes `now`, so none of this needs to sleep.
    """

    def _cam(self):
        return DepthCameraThread({}, threading.Lock())

    def _pic(self, lit=500):
        p = np.zeros((100, 100), np.uint8)
        p.ravel()[:lit] = 255
        return p

    def test_the_first_picture_always_goes_out(self):
        cam = self._cam()
        assert cam._needs_resend("mask", self._pic(), 0.0)

    def test_an_identical_picture_is_held(self):
        cam = self._cam()
        pic = self._pic()
        cam._mark_sent("mask", pic, 0.0)
        assert not cam._needs_resend("mask", pic.copy(), 0.1)

    def test_a_changed_picture_goes_out(self):
        cam = self._cam()
        cam._mark_sent("mask", self._pic(500), 0.0)
        assert cam._needs_resend("mask", self._pic(900), 0.1)

    def test_a_few_pixels_are_forgiven_when_a_tolerance_is_given(self):
        """Invisible once the mask is warped across a sandbox — not worth a frame."""
        cam = self._cam()
        cam._mark_sent("mask_full", self._pic(500), 0.0)
        assert not cam._needs_resend("mask_full", self._pic(510), 0.1, tol_px=32)
        assert cam._needs_resend("mask_full", self._pic(600), 0.1, tol_px=32)

    def test_the_dev_views_are_exact(self):
        """No tolerance by default: the Mask/Skeleton views are diagnostic."""
        cam = self._cam()
        cam._mark_sent("mask", self._pic(500), 0.0)
        assert cam._needs_resend("mask", self._pic(501), 0.1)

    def test_small_changes_cannot_drift_the_picture_away_unnoticed(self):
        """
        The comparison is against the last picture SENT, so ten forgivable
        changes in a row still add up to one that is not.
        """
        cam = self._cam()
        cam._mark_sent("mask_full", self._pic(500), 0.0)
        sent = False
        for i in range(1, 11):
            grown = self._pic(500 + i * 10)
            if cam._needs_resend("mask_full", grown, 0.1 * i, tol_px=32):
                sent = True
                break
        assert sent, "accumulated drift must eventually be pushed"

    def test_an_unchanged_stream_still_speaks_up_eventually(self):
        """A stream that goes silent forever is indistinguishable from a dead one."""
        from camera_thread import PROJECTION_KEEPALIVE_S
        cam = self._cam()
        pic = self._pic()
        cam._mark_sent("mask", pic, 0.0)
        assert not cam._needs_resend("mask", pic.copy(), PROJECTION_KEEPALIVE_S / 2)
        assert cam._needs_resend("mask", pic.copy(), PROJECTION_KEEPALIVE_S + 0.1)

    def test_a_rate_floor_delays_a_changed_picture(self):
        cam = self._cam()
        cam._mark_sent("mask_full", self._pic(500), 0.0)
        assert not cam._needs_resend("mask_full", self._pic(900), 0.05, min_interval=0.2)
        assert cam._needs_resend("mask_full", self._pic(900), 0.3, min_interval=0.2)

    def test_a_resized_picture_goes_out(self):
        """A view rotation or a crop resize must not be compared away."""
        cam = self._cam()
        cam._mark_sent("mask", self._pic(), 0.0)
        assert cam._needs_resend("mask", np.zeros((50, 200), np.uint8), 0.1)

    def test_changing_the_detection_forces_the_next_frame_out(self):
        cam = self._cam()
        cam._mark_sent("mask", self._pic(), 0.0)
        cam.set_live_params(DepthGrooveParams(groove_depth_mm=3.0))
        assert cam._needs_resend("mask", self._pic(), 0.1)


class TestTheProjectorStreamBehaves:
    """End to end through `_publish`, which is where the wiring can go wrong."""

    def _frames(self, cam, state, n, truth, rng, start=0):
        """Run n canvases and count how many NEW projector JPEGs came out."""
        sent, last = 0, state.get("last_mask_full_jpg")
        for i in range(start, start + n):
            z = truth + rng.normal(0, 0.0015, truth.shape).astype(np.float32)
            cam._publish(z, np.ones(truth.shape, bool), None, i)
            cur = state.get("last_mask_full_jpg")
            if cur is not last:
                sent += 1
                last = cur
        return sent

    def _sand(self):
        t = np.full((120, 200), 1.0, np.float32)
        t[50:60, 20:180] += 0.004
        return t

    def test_still_sand_stops_feeding_the_projector(self):
        import camera_thread as ct
        state = {"projection_clients": 1}
        cam = DepthCameraThread(state, threading.Lock())
        rng = np.random.default_rng(4)
        truth = self._sand()
        ref = np.full(truth.shape, 1.0, np.float32)
        cam.set_reference(ref)
        keep = ct.PROJECTION_KEEPALIVE_S
        ct.PROJECTION_KEEPALIVE_S = 1e9      # isolate from the keepalive
        try:
            self._frames(cam, state, 80, truth, rng)          # let it settle
            quiet = self._frames(cam, state, 60, truth, rng, start=80)
        finally:
            ct.PROJECTION_KEEPALIVE_S = keep
        assert quiet <= 2, f"still sand should go quiet, sent {quiet}/60"

    def test_a_real_change_still_reaches_the_projector_at_once(self):
        """Quiet must not mean deaf — this is what a participant watches."""
        state = {"projection_clients": 1}
        cam = DepthCameraThread(state, threading.Lock())
        rng = np.random.default_rng(4)
        truth = self._sand()
        ref = np.full(truth.shape, 1.0, np.float32)
        cam.set_reference(ref)
        self._frames(cam, state, 80, truth, rng)
        truth[80:90, 20:180] += 0.006                          # a groove is raked
        assert self._frames(cam, state, 10, truth, rng, start=80) > 0

    def test_a_reconnecting_window_is_given_a_picture_immediately(self):
        """
        It has nothing on screen to hold, so it must not wait for the next
        change — which on finished sand might never come.
        """
        state = {"projection_clients": 1}
        cam = DepthCameraThread(state, threading.Lock())
        rng = np.random.default_rng(4)
        truth = self._sand()
        ref = np.full(truth.shape, 1.0, np.float32)
        cam.set_reference(ref)
        self._frames(cam, state, 60, truth, rng)

        state["projection_clients"] = 0
        cam._publish(truth, np.ones(truth.shape, bool), None, 60)
        assert state.get("last_mask_full_jpg") is None
        assert "mask_full" not in cam._sent

        state["projection_clients"] = 1
        cam._publish(truth, np.ones(truth.shape, bool), None, 61)
        assert state.get("last_mask_full_jpg") is not None


class TestTheCapturedStillIsUntouched:
    """
    The dampers buy a steady PICTURE. They must never reach the path the robot
    draws, which is judged on its own freshly averaged frame.
    """

    def test_capture_path_takes_no_memory(self):
        """`process_depth` is what Capture/Generate run — no damper arguments."""
        import inspect
        from depth_extractor import process_depth
        assert "prev_mask" not in inspect.signature(process_depth).parameters

    def test_capture_frame_does_not_read_the_live_average(self):
        """
        `capture_frame` averages the raw per-camera buffers itself. If it ever
        started reading `_z_avg`, a Capture would inherit the live smoothing —
        including the held-over pixels of a camera that had stopped.
        """
        import inspect
        src = inspect.getsource(DepthCameraThread.capture_frame)
        assert "_z_avg" not in src and "_prev_mask" not in src
