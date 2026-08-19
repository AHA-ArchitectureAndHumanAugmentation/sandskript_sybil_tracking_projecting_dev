"""
Unit tests for path_length.py — how far the tool travels while drawing, and the
Max Total Length gate. Pure geometry, no hardware.
"""
import math

import pytest

from path_export import stroke_blend
from path_length import (
    blended_length, exceeds_limit, polyline_length, total_length_mm,
)

_PI = math.pi


def _line(points):
    """Waypoints from [x, y, z] triples, tool-down orientation."""
    return [[x, y, z, 0.0, _PI, 0.0] for x, y, z in points]


# A 100 mm right angle: 50 mm east, then 50 mm north.
CORNER = _line([(0.0, 0.0, 0.0), (0.05, 0.0, 0.0), (0.05, 0.05, 0.0)])
# A straight 100 mm run through a middle waypoint.
STRAIGHT = _line([(0.0, 0.0, 0.0), (0.05, 0.0, 0.0), (0.1, 0.0, 0.0)])


class TestPolylineLength:

    def test_sums_the_segments(self):
        assert polyline_length(CORNER) == pytest.approx(0.1)

    def test_measures_in_three_dimensions(self):
        # The strokes are projected onto the surface, so a path running up a
        # slope is genuinely longer than its footprint. Measuring in 2D would
        # under-report exactly the case the limit exists for.
        slope = _line([(0.0, 0.0, 0.0), (0.03, 0.04, 0.0)])
        climb = _line([(0.0, 0.0, 0.0), (0.03, 0.0, 0.04)])
        assert polyline_length(slope) == pytest.approx(0.05)
        assert polyline_length(climb) == pytest.approx(0.05)

    def test_single_point_is_zero(self):
        assert polyline_length(_line([(0.0, 0.0, 0.0)])) == 0.0


class TestBlendedLength:

    def test_zero_radius_is_the_plain_polyline(self):
        assert blended_length([CORNER], 0.0) == pytest.approx(0.1)

    def test_a_straight_run_is_unchanged_by_blending(self):
        # No corner to round: a waypoint mid-line must cost nothing.
        assert blended_length([STRAIGHT], 0.005) == pytest.approx(0.1)

    def test_rounding_a_corner_shortens_the_path(self):
        assert blended_length([CORNER], 0.005) < polyline_length(CORNER)

    def test_the_saving_matches_the_arc_geometry(self):
        # 90° corner, 5 mm zone: 2r of straight line replaced by an arc of
        # radius r·tan(45°) swept through 90°.
        r = stroke_blend(CORNER, 0.005)
        expected = 0.1 - (2 * r - r * math.tan(_PI / 4) * (_PI / 2))
        assert blended_length([CORNER], 0.005) == pytest.approx(expected)

    def test_a_bigger_radius_shortens_it_further(self):
        small = blended_length([CORNER], 0.001)
        big = blended_length([CORNER], 0.005)
        assert big < small < polyline_length(CORNER)

    def test_the_radius_is_clamped_like_the_executor(self):
        # stroke_blend caps at 45% of the shortest segment; asking for more must
        # not let the length run away, or the number stops describing the robot.
        tight = _line([(0.0, 0.0, 0.0), (0.01, 0.0, 0.0), (0.01, 0.004, 0.0)])
        huge = blended_length([tight], 1.0)
        clamped = blended_length([tight], stroke_blend(tight, 1.0))
        assert huge == pytest.approx(clamped)
        assert huge >= 0.0

    def test_strokes_are_summed(self):
        assert blended_length([CORNER, CORNER], 0.0) == pytest.approx(0.2)

    def test_travels_between_strokes_are_not_counted(self):
        # Two strokes a metre apart still measure 200 mm: the pen-up move
        # between them is positioning, not drawing.
        far = [[x + 1.0, y, z, rx, ry, rz] for x, y, z, rx, ry, rz in CORNER]
        assert blended_length([CORNER, far], 0.0) == pytest.approx(0.2)

    def test_empty_and_degenerate_input(self):
        assert blended_length([], 0.005) == 0.0
        assert blended_length([[]], 0.005) == 0.0
        assert blended_length([_line([(0.0, 0.0, 0.0)])], 0.005) == 0.0

    def test_duplicate_waypoints_do_not_break_it(self):
        dup = _line([(0.0, 0.0, 0.0), (0.05, 0.0, 0.0), (0.05, 0.0, 0.0),
                     (0.05, 0.05, 0.0)])
        assert blended_length([dup], 0.005) == pytest.approx(
            blended_length([dup], 0.005))          # finite, no divide-by-zero
        assert blended_length([dup], 0.005) > 0.0

    def test_a_doubling_back_corner_never_goes_negative(self):
        back = _line([(0.0, 0.0, 0.0), (0.05, 0.0, 0.0), (0.0, 0.0, 0.0)])
        assert blended_length([back], 0.005) >= 0.0


class TestSpacingAndJoining:
    """
    Spacing and Distance Threshold are 'considered' by construction — they
    change the waypoints before this ever sees them. These pin that down.
    """

    def test_coarser_spacing_shortens_a_curve(self):
        # Sampling a quarter circle finely follows it; sampling it coarsely
        # cuts across, and the length must reflect that.
        fine = _line([(0.1 * math.cos(t * _PI / 40), 0.1 * math.sin(t * _PI / 40), 0.0)
                      for t in range(21)])
        coarse = _line([(0.1 * math.cos(t * _PI / 8), 0.1 * math.sin(t * _PI / 8), 0.0)
                        for t in range(5)])
        assert polyline_length(coarse) < polyline_length(fine)

    def test_a_join_adds_the_gap_it_closed(self):
        # join_strokes concatenates the point lists, so the closed gap becomes
        # a real segment — two 50 mm strokes 20 mm apart measure 120 mm.
        a = _line([(0.0, 0.0, 0.0), (0.05, 0.0, 0.0)])
        b = _line([(0.07, 0.0, 0.0), (0.12, 0.0, 0.0)])
        assert blended_length([a, b], 0.0) == pytest.approx(0.1)      # separate
        assert blended_length([a + b], 0.0) == pytest.approx(0.12)    # joined


class TestTotalLengthMm:

    def test_converts_to_millimetres(self):
        assert total_length_mm([CORNER], 0.0) == pytest.approx(100.0)

    def test_blend_argument_is_in_millimetres(self):
        assert total_length_mm([CORNER], 5.0) == pytest.approx(
            blended_length([CORNER], 0.005) * 1000.0)


class TestExceedsLimit:

    def test_under_the_limit_passes(self):
        over, actual = exceeds_limit([CORNER], 0.0, 200.0)
        assert over is False
        assert actual == pytest.approx(100.0)

    def test_over_the_limit_fails(self):
        over, actual = exceeds_limit([CORNER], 0.0, 50.0)
        assert over is True
        assert actual == pytest.approx(100.0)

    def test_exactly_at_the_limit_passes(self):
        over, _ = exceeds_limit([CORNER], 0.0, 100.0)
        assert over is False

    def test_zero_means_no_limit(self):
        # Same "0 = off" convention as the Distance Threshold box.
        over, actual = exceeds_limit([CORNER] * 100, 0.0, 0.0)
        assert over is False
        assert actual == pytest.approx(10000.0)

    def test_none_means_no_limit(self):
        assert exceeds_limit([CORNER], 0.0, None)[0] is False

    def test_the_radius_can_bring_a_path_under_the_limit(self):
        # Rounding corners genuinely shortens the path, so Radius belongs in
        # the judgement — a path just over the line can pass with blending on.
        zig = _line([(0.0, 0.0, 0.0), (0.05, 0.0, 0.0), (0.05, 0.05, 0.0),
                     (0.1, 0.05, 0.0), (0.1, 0.1, 0.0)])
        limit = total_length_mm([zig], 0.0) - 1.0
        assert exceeds_limit([zig], 0.0, limit)[0] is True
        assert exceeds_limit([zig], 5.0, limit)[0] is False

    def test_no_strokes_is_never_over(self):
        assert exceeds_limit([], 0.0, 1.0) == (False, 0.0)


class TestContainment:
    def test_importing_does_not_drag_in_the_app(self):
        """A limit check must not be able to start a camera or a robot."""
        import subprocess
        import sys
        from pathlib import Path

        code = (
            "import sys, path_length;"
            "bad=[m for m in ('main','camera_thread','robot_controller',"
            "'pyrealsense2','compas_rrc') if m in sys.modules];"
            "print(','.join(bad))"
        )
        out = subprocess.run([sys.executable, "-c", code],
                             cwd=str(Path(__file__).resolve().parent.parent),
                             capture_output=True, text=True, timeout=60)
        assert out.returncode == 0, out.stderr
        assert out.stdout.strip() == "", f"path_length pulled in {out.stdout.strip()}"
