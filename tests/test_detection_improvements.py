"""
Unit tests for the 2026-08 live-view improvements:

  1. `surface_open_px` — a grayscale opening removes the grooves from what the
     detrend blur sees, so the bare-sand surface estimate no longer sinks into
     densely raked areas. Before it, parts of a groove at the SAME physical
     depth as detected grooves elsewhere fell under threshold whenever the
     raking around them was dense — the exact complaint from the sandbox.
  2. `DepthCameraThread._latch_mask` — third steadiness damper: a pixel
     detected for LATCH_ON consecutive canvases is held lit until undetected
     for LATCH_OFF consecutive ones. Raked grooves cannot un-rake themselves,
     so settled regions hold perfectly still on the projector.
  3. `DepthCameraThread._smoothed_range` — the auto colormap range is smoothed
     across canvases instead of recomputed per frame, so the depth view stops
     breathing with sensor noise.
  4. `stitch(want_rgb=False)` — the colour warp (the most expensive step of
     the live loop) is skipped when nothing is watching the colour view.

No hardware — synthetic sand throughout.
"""
import threading

import numpy as np
import pytest

from camera_thread import DepthCameraThread, _LATCH_OFF, _LATCH_ON
from depth_extractor import (
    DepthGrooveParams, grooves_and_mask, near_object_mask,
)
from stitcher import CameraFrame, CameraPlacement, Intrinsics, StitchCalib, stitch

SIZE = 240
GROOVE_MM = 3.0     # well above the 1.5 mm entry threshold — same depth everywhere
WIDTH = 10          # groove width, px
GAP = 8             # sand left between grooves in the dense field, px


def _field(n_grooves: int, size: int = SIZE) -> np.ndarray:
    """Flat sand at 1 m with `n_grooves` parallel grooves, all GROOVE_MM deep."""
    d = np.full((size, size), 1.0, np.float32)
    pitch = WIDTH + GAP
    span = n_grooves * pitch - GAP
    top = size // 2 - span // 2
    for i in range(n_grooves):
        lo = top + i * pitch
        d[lo:lo + WIDTH, 30:size - 30] += GROOVE_MM / 1000.0
    return d


def _groove_rows(n_grooves: int, size: int = SIZE) -> list[int]:
    """The centre row of each groove in `_field(n_grooves)`."""
    pitch = WIDTH + GAP
    span = n_grooves * pitch - GAP
    top = size // 2 - span // 2
    return [top + i * pitch + WIDTH // 2 for i in range(n_grooves)]


class TestSurfaceOpening:
    """The detrend must measure relief against BARE sand, not raked sand."""

    def test_dense_raking_no_longer_swallows_grooves(self):
        """
        The regression this exists for: five same-depth grooves packed
        together. The plain-Gaussian surface estimate sinks toward their
        bottoms, so the middle ones lose relief; the opening removes the
        grooves from the estimate first, so every one keeps its full depth.
        """
        field = _field(5)
        with_open = grooves_and_mask(field, params=DepthGrooveParams())[0]
        for row in _groove_rows(5):
            assert (with_open[row, 60:SIZE - 60] > 0).mean() > 0.9, \
                f"groove at row {row} should be detected end to end"

    def test_detection_matches_an_isolated_groove(self):
        """
        The user-visible property: a groove detects the SAME whether it is
        alone in clean sand or surrounded by other grooves at the same depth.
        """
        p = DepthGrooveParams()
        alone = grooves_and_mask(_field(1), params=p)[0]
        dense = grooves_and_mask(_field(5), params=p)[0]
        row_alone = _groove_rows(1)[0]
        row_mid = _groove_rows(5)[2]                      # middle of the pack
        cov_alone = (alone[row_alone, 60:SIZE - 60] > 0).mean()
        cov_mid = (dense[row_mid, 60:SIZE - 60] > 0).mean()
        assert cov_mid == pytest.approx(cov_alone, abs=0.05)

    def test_zero_disables_it(self):
        """surface_open_px=0 must reproduce the old plain-Gaussian behaviour,
        so the two can still be A/B compared from the browser."""
        field = _field(5)
        old = grooves_and_mask(field, params=DepthGrooveParams(surface_open_px=0))[0]
        new = grooves_and_mask(field, params=DepthGrooveParams())[0]
        # The old estimate loses coverage in the dense field; the new one must
        # strictly improve on it (this doubles as proof the flag does something).
        assert (new > 0).sum() > (old > 0).sum()

    def test_flat_sand_stays_empty(self):
        """The opening's bias on untouched sand must stay under threshold."""
        flat = np.full((SIZE, SIZE), 1.0, np.float32)
        assert (grooves_and_mask(flat, params=DepthGrooveParams())[0] > 0).sum() == 0

    def test_ridge_mode_mirrors_it(self):
        """Ridges are near-side excursions, removed by a CLOSING instead."""
        d = np.full((SIZE, SIZE), 1.0, np.float32)
        for row in _groove_rows(5):
            d[row - WIDTH // 2:row + WIDTH // 2, 30:SIZE - 30] -= GROOVE_MM / 1000.0
        mask = grooves_and_mask(d, params=DepthGrooveParams(detect="ridge"))[0]
        for row in _groove_rows(5):
            assert (mask[row, 60:SIZE - 60] > 0).mean() > 0.9

    def test_from_dict_round_trip(self):
        assert DepthGrooveParams.from_dict({"surface_open_px": 21}).surface_open_px == 21
        assert DepthGrooveParams.from_dict({}).surface_open_px == \
            DepthGrooveParams().surface_open_px


class TestMaskLatch:
    """Settled grooves must hold perfectly still; smoothed sand must clear."""

    def _cam(self):
        return DepthCameraThread({}, threading.Lock())

    def _on(self):
        m = np.zeros((20, 20), np.uint8)
        m[8:12, 2:18] = 255
        return m

    def _off(self):
        return np.zeros((20, 20), np.uint8)

    def test_a_settled_pixel_survives_a_flicker(self):
        cam = self._cam()
        for _ in range(_LATCH_ON):
            cam._latch_mask(self._on())
        held = cam._latch_mask(self._off())     # one dark canvas — the flicker
        assert (held[8:12, 2:18] > 0).all(), "a latched groove must ride out a dropout"

    def test_an_unsettled_pixel_does_not_latch(self):
        cam = self._cam()
        for _ in range(_LATCH_ON - 1):
            cam._latch_mask(self._on())
        assert (cam._latch_mask(self._off()) > 0).sum() == 0

    def test_smoothed_sand_releases(self):
        cam = self._cam()
        for _ in range(_LATCH_ON):
            cam._latch_mask(self._on())
        out = None
        for _ in range(_LATCH_OFF):
            out = cam._latch_mask(self._off())
        assert (out > 0).sum() == 0, "sand smoothed over must clear the latch"

    def test_intermittent_detection_keeps_the_latch(self):
        """The real signal: a groove edge flipping every other canvas."""
        cam = self._cam()
        for _ in range(_LATCH_ON):
            cam._latch_mask(self._on())
        for i in range(_LATCH_OFF * 2):
            out = cam._latch_mask(self._on() if i % 2 else self._off())
        assert (out[8:12, 2:18] > 0).all()

    def test_forget_live_mask_drops_it(self):
        cam = self._cam()
        for _ in range(_LATCH_ON):
            cam._latch_mask(self._on())
        cam._forget_live_mask()
        assert cam._lit is None and cam._latched is None
        assert (cam._latch_mask(self._off()) > 0).sum() == 0

    def test_a_crop_resize_restarts_the_counts(self):
        cam = self._cam()
        for _ in range(_LATCH_ON):
            cam._latch_mask(self._on())
        small = np.zeros((10, 10), np.uint8)
        out = cam._latch_mask(small)
        assert out.shape == (10, 10) and (out > 0).sum() == 0


class TestSmoothedColormapRange:
    def _cam(self):
        return DepthCameraThread({}, threading.Lock())

    def test_first_frame_sets_the_range(self):
        cam = self._cam()
        z = np.linspace(0.8, 1.2, 100, dtype=np.float32).reshape(10, 10)
        near, far = cam._smoothed_range(z, np.ones((10, 10), bool))
        assert 0.8 <= near < far <= 1.2

    def test_noise_no_longer_moves_the_range_much(self):
        """The breathing itself: per-frame percentiles vs the smoothed ones."""
        rng = np.random.default_rng(1)
        cam = self._cam()
        ok = np.ones((50, 50), bool)
        raw_far, smooth_far = [], []
        for _ in range(30):
            z = (1.0 + rng.normal(0, 0.002, (50, 50))).astype(np.float32)
            raw_far.append(float(np.percentile(z[ok], 98.0)))
            smooth_far.append(cam._smoothed_range(z, ok)[1])
        assert np.std(smooth_far[10:]) < np.std(raw_far[10:]) / 2.0

    def test_a_real_change_still_gets_through(self):
        cam = self._cam()
        ok = np.ones((10, 10), bool)
        for _ in range(5):
            cam._smoothed_range(np.full((10, 10), 1.0, np.float32), ok)
        far = 0.0
        for _ in range(60):
            _, far = cam._smoothed_range(np.full((10, 10), 1.5, np.float32), ok)
        assert far == pytest.approx(1.5, abs=0.05)

    def test_an_empty_frame_keeps_the_last_range(self):
        cam = self._cam()
        ok = np.ones((10, 10), bool)
        first = cam._smoothed_range(np.full((10, 10), 1.0, np.float32), ok)
        empty = cam._smoothed_range(np.zeros((10, 10), np.float32),
                                    np.zeros((10, 10), bool))
        assert empty == first


class TestMovingObjectsAreRejected:
    """
    The near-object filter must find a hand in the RAW frame.

    Judging it on the steadied (time-averaged) canvas worked for a STILL
    object — it settles into the average and reads its true height — but not
    for a MOVING one, which only drags each pixel part of the way toward
    itself before it has moved on, so it never clears the cutoff. A hand
    raking is exactly the moving case, i.e. the one the filter exists for.
    """

    SAND_M = 0.30
    HAND_MM = 60.0          # how far the hand hovers above the sand
    CUTOFF_MM = 40.0

    def _cam(self):
        return DepthCameraThread({}, threading.Lock())

    def _sand(self):
        return np.full((80, 80), self.SAND_M, np.float32)

    def _with_hand(self, col: int):
        d = self._sand()
        d[20:60, col:col + 12] = self.SAND_M - self.HAND_MM / 1000.0
        return d

    def test_the_raw_frame_sees_a_hand_the_average_misses(self):
        """The bug in one assertion, stated on the two frames themselves."""
        cam = self._cam()
        ok = np.ones((80, 80), bool)
        sand = self._sand()
        for _ in range(30):
            cam._steady_depth(sand, ok)          # settle on empty sand

        moving = self._with_hand(30)             # hand arrives, one canvas
        z_avg, ok_avg = cam._steady_depth(moving, ok)

        raw_hit = near_object_mask(moving, ok, sand, self.CUTOFF_MM)
        avg_hit = near_object_mask(z_avg, ok_avg, sand, self.CUTOFF_MM)
        assert raw_hit.any(), "the raw frame must see the hand immediately"
        assert not avg_hit.any(), "the average has barely moved — this was the bug"

    def test_a_sweeping_hand_never_enters_the_average(self):
        """
        Freezing held pixels is what stops a moving hand painting phantom
        relief along its path and trailing behind itself afterwards.
        """
        cam = self._cam()
        ok = np.ones((80, 80), bool)
        sand = self._sand()
        for _ in range(30):
            cam._steady_depth(sand, ok)

        for col in range(10, 60, 2):             # sweep across the box
            frame = self._with_hand(col)
            near = near_object_mask(frame, ok, sand, self.CUTOFF_MM)
            z_avg, _ = cam._steady_depth(frame, ok, near)

        assert z_avg == pytest.approx(sand, abs=1e-4), \
            "the averaged canvas must still be clean sand after the sweep"

    def test_a_hand_cannot_latch(self):
        cam = self._cam()
        mask = np.zeros((40, 40), np.uint8)
        mask[10:30, 10:30] = 255                 # detection under the hand
        near = np.zeros((40, 40), bool)
        near[10:30, 10:30] = True
        out = None
        for _ in range(_LATCH_ON + 5):
            out = cam._latch_mask(mask, near)
        assert (out > 0).sum() == 0, "nothing under a near object may be published"
        assert not cam._latched.any(), "and nothing under it may latch"

    def test_a_hand_crossing_a_groove_does_not_unlatch_it(self):
        """
        Counters freeze under a hand rather than counting toward release, so a
        groove the hand sweeps over comes straight back instead of blinking
        out and having to earn its latch again.
        """
        cam = self._cam()
        groove = np.zeros((40, 40), np.uint8)
        groove[18:22, :] = 255
        for _ in range(_LATCH_ON):
            cam._latch_mask(groove)

        near = np.zeros((40, 40), bool)
        near[15:25, 10:20] = True                # hand parked on the groove
        occluded = groove.copy()
        occluded[15:25, 10:20] = 0               # blob removal under the hand
        for _ in range(_LATCH_OFF * 2):
            out = cam._latch_mask(occluded, near)
            assert (out[18:22, 10:20] > 0).sum() == 0, "not shown under the hand"

        back = cam._latch_mask(occluded)         # hand lifts, sand still unread
        assert (back[18:22, 10:20] > 0).all(), "the groove must return at once"

    def test_detection_accepts_a_ready_made_near_mask(self):
        """`near=` overrides the frame the test would otherwise be judged on."""
        d = self._sand()
        d[38:42, 10:70] += 3.0 / 1000.0          # a groove
        p = DepthGrooveParams(ignore_closer_mm=self.CUTOFF_MM)
        free = grooves_and_mask(d, params=p, reference=self._sand())[0]
        assert (free > 0).any(), "the groove is detected when nothing is near"

        near = np.zeros(d.shape, bool)
        near[36:44, 30:40] = True                # a hand across it
        blocked = grooves_and_mask(d, params=p, reference=self._sand(),
                                   near=near)[0]
        assert (blocked > 0).sum() == 0, "the touched blob must be dropped"

    def test_off_by_default_so_capture_is_unchanged(self):
        d = self._sand()
        d[38:42, 10:70] += 3.0 / 1000.0
        p = DepthGrooveParams()                  # ignore_closer_mm = 0
        assert np.array_equal(
            grooves_and_mask(d, params=p)[0],
            grooves_and_mask(d, params=p, near=None)[0])

    def test_no_reference_means_no_near_mask(self):
        assert near_object_mask(self._with_hand(30), np.ones((80, 80), bool),
                                None, self.CUTOFF_MM) is None

    def test_zero_cutoff_disables_it(self):
        assert near_object_mask(self._with_hand(30), np.ones((80, 80), bool),
                                self._sand(), 0.0) is None


class TestColourWarpGating:
    def _frame(self):
        w, h = 160, 120
        return CameraFrame(
            depth_m=np.full((h, w), 0.8, np.float32),
            valid=np.ones((h, w), bool),
            intr=Intrinsics.nominal(w, h),
            rgb=np.full((h, w, 3), 120, np.uint8),
            serial="cam0",
        )

    def test_want_rgb_false_skips_the_colour_warp(self):
        calib = StitchCalib(cams=[CameraPlacement()])
        result = stitch([self._frame()], calib, want_rgb=False)
        assert not result.rgb_valid.any()
        assert result.valid.any(), "depth must be unaffected"

    def test_want_rgb_defaults_on(self):
        """Capture and the placement tool never pass the flag — they keep colour."""
        calib = StitchCalib(cams=[CameraPlacement()])
        result = stitch([self._frame()], calib)
        assert result.rgb_valid.any()
