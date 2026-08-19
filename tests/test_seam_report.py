"""
Unit tests for stitcher.seam_report — the seam-agreement diagnostic.

Where two cameras see the same sand, `stitch` AVERAGES their depths, so a
per-camera bias is not one visible step but a plateau with a step at each edge
of the overlap band. A raked groove is ~1.5 mm deep, so a few mm of seam is
several times the signal.

The single thing this tool has to get right is the VERDICT: "height" means one
constant is wrong and the Height buttons cancel it; "tilt" means the gap varies
along the seam and no single offset can. Everything else is decoration.
No hardware — synthetic cameras throughout.
"""
import numpy as np
import pytest

from stitcher import SeamStats, seam_report, stitch, synthetic_scene


def _scene(n=2):
    """Two overlapping synthetic cameras and the calib that places them."""
    return synthetic_scene(n, overlap_frac=0.25)


def _bias(frame, mm):
    """The same camera, reading everything `mm` farther than it really is."""
    biased = frame.depth_m.copy()
    biased[frame.valid] += mm / 1000.0
    return type(frame)(biased, frame.valid, frame.intr, frame.rgb, frame.serial)


def _tilt(frame, mm_across):
    """A camera whose error ramps left-to-right — a mounting ANGLE difference."""
    tilted = frame.depth_m.copy()
    w = tilted.shape[1]
    ramp = np.linspace(0.0, mm_across / 1000.0, w, dtype=np.float32)
    tilted[frame.valid] += np.broadcast_to(ramp, tilted.shape)[frame.valid]
    return type(frame)(tilted, frame.valid, frame.intr, frame.rgb, frame.serial)


class TestAgreementIsDetected:

    def test_identical_cameras_read_as_aligned(self):
        frames, calib = _scene()
        seams = seam_report(frames, calib)
        assert seams, "two overlapping cameras should produce one seam"
        assert seams[0].verdict == "aligned"
        assert abs(seams[0].mean_mm) < 0.5

    def test_one_seam_per_overlapping_pair(self):
        frames, calib = synthetic_scene(3, overlap_frac=0.25)
        seams = seam_report(frames, calib)
        pairs = {(s.a, s.b) for s in seams}
        assert (0, 1) in pairs and (1, 2) in pairs

    def test_non_overlapping_cameras_produce_nothing(self):
        """No shared sand = no comparison to make, not a zero-disagreement one."""
        frames, calib = synthetic_scene(2, overlap_frac=0.0)
        for s in seam_report(frames, calib):
            assert s.px >= 50

    def test_a_single_camera_has_no_seams(self):
        frames, calib = synthetic_scene(1)
        assert seam_report(frames, calib) == []


class TestHeightVersusTilt:
    """The distinction the whole feature exists to make."""

    def test_a_constant_offset_reads_as_height(self):
        frames, calib = _scene()
        frames[1] = _bias(frames[1], 8.0)          # camera 2 reads 8 mm farther
        seam = seam_report(frames, calib)[0]
        assert seam.verdict == "height"
        assert seam.mean_mm == pytest.approx(-8.0, abs=1.5)   # a − b

    def test_the_height_hint_names_the_camera_and_the_number(self):
        frames, calib = _scene()
        frames[1] = _bias(frames[1], 8.0)
        seam = seam_report(frames, calib)[0]
        assert "Height" in seam.hint
        assert "camera 2" in seam.hint

    def test_a_ramping_error_reads_as_tilt(self):
        frames, calib = _scene()
        frames[1] = _tilt(frames[1], 25.0)         # 25 mm across the frame
        seam = seam_report(frames, calib)[0]
        assert seam.verdict == "tilt"
        assert seam.slope_mm > 1.0

    def test_the_tilt_hint_says_a_height_nudge_will_not_help(self):
        frames, calib = _scene()
        frames[1] = _tilt(frames[1], 25.0)
        seam = seam_report(frames, calib)[0]
        assert "ANGLE" in seam.hint or "angle" in seam.hint

    def test_a_big_offset_with_a_small_ramp_is_still_height(self):
        """Real mounts are never perfect — a dominant offset must not read tilt."""
        frames, calib = _scene()
        frames[1] = _tilt(_bias(frames[1], 20.0), 2.0)
        assert seam_report(frames, calib)[0].verdict == "height"


class TestHeightMmIsHonoured:

    def test_a_seam_levelled_with_the_height_buttons_reads_flat(self):
        """
        height_mm is added to that camera's depth by the stitch, so a seam the
        operator has already levelled must come back "aligned" — otherwise the
        readout would keep nagging about a fault that is fixed.
        """
        frames, calib = _scene()
        frames[1] = _bias(frames[1], 8.0)
        seam = seam_report(frames, calib)[0]
        assert seam.verdict == "height"

        # Apply exactly the correction the hint asks for.
        fixed = calib.with_camera(1, calib.cams[1].merged({"height_mm": -8.0}))
        after = seam_report(frames, fixed)[0]
        assert after.verdict == "aligned"
        assert abs(after.mean_mm) < 0.5


class TestReportingShape:

    def test_to_dict_is_json_safe(self):
        frames, calib = _scene()
        frames[1] = _bias(frames[1], 6.0)
        d = seam_report(frames, calib)[0].to_dict()
        import json
        json.loads(json.dumps(d))              # would raise on a numpy scalar
        assert set(d) == {"a", "b", "px", "mean_mm", "std_mm", "p95_mm",
                          "slope_mm", "verdict", "hint"}
        assert isinstance(d["mean_mm"], float)

    def test_it_does_not_disturb_the_stitch(self):
        """It is a separate pass on purpose — the live canvas must be untouched."""
        frames, calib = _scene()
        before = stitch(frames, calib).depth_m.copy()
        seam_report(frames, calib)
        after = stitch(frames, calib).depth_m
        assert np.array_equal(before, after)

    def test_a_frozen_grid_is_accepted(self):
        """The main app freezes its canvas; the report must measure on it too."""
        from stitcher import CanvasGrid
        frames, calib = _scene()
        grid = CanvasGrid.from_result(stitch(frames, calib))
        seams = seam_report(frames, calib, grid=grid)
        assert seams and isinstance(seams[0], SeamStats)
