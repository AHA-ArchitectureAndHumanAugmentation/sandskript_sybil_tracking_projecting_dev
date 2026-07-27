"""
Unit tests for path_extractor.py — pure functions, no hardware required.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from path_extractor import (
    ExtractedPath,
    _order_strokes,
    extract_from_edges,
    join_strokes,
    pixels_to_robot_coords,
    resample_stroke,
)
from config import TOOL_ORIENTATION


# ─────────────────────────────────────────────────────────────────────────────
# resample_stroke
# ─────────────────────────────────────────────────────────────────────────────

class TestResampleStroke:

    def test_single_point_returned_unchanged(self):
        stroke = [(10, 20)]
        assert resample_stroke(stroke, spacing_px=10.0) == [(10, 20)]

    def test_short_segment_returns_only_endpoints(self):
        # arc length = 5, spacing = 10 → total < spacing → [start, end]
        stroke = [(0, 0), (3, 4)]
        result = resample_stroke(stroke, spacing_px=10.0)
        assert result == [(0, 0), (3, 4)]

    def test_horizontal_line_spacing(self):
        stroke = [(0, 0), (100, 0)]
        result = resample_stroke(stroke, spacing_px=10.0)
        assert result[0] == (0, 0)
        assert result[-1] == (100, 0)
        # all y == 0
        assert all(y == 0 for _, y in result)
        # consecutive x-differences ≈ 10 (except the last gap)
        for i in range(1, len(result) - 1):
            dx = result[i][0] - result[i - 1][0]
            assert 8 <= dx <= 12, f"spacing {dx} not ≈10 at index {i}"

    def test_diagonal_preserves_endpoints(self):
        # arc length = 50, spacing = 10 → 6 points
        stroke = [(0, 0), (30, 40)]
        result = resample_stroke(stroke, spacing_px=10.0)
        assert result[0] == (0, 0)
        assert result[-1] == (30, 40)
        assert len(result) == 6

    def test_spacing_larger_than_arc_length_returns_endpoints(self):
        stroke = [(0, 0), (5, 0), (8, 0)]  # arc = 8
        result = resample_stroke(stroke, spacing_px=20.0)
        assert result == [(0, 0), (8, 0)]

    def test_polyline_corner_not_skipped(self):
        # L-shape: right 50 then up 50, total arc = 100, spacing = 25 → 5 pts
        stroke = [(0, 0), (50, 0), (50, 50)]
        result = resample_stroke(stroke, spacing_px=25.0)
        assert result[0] == (0, 0)
        assert result[-1] == (50, 50)
        assert len(result) == 5


# ─────────────────────────────────────────────────────────────────────────────
# _order_strokes
# ─────────────────────────────────────────────────────────────────────────────

class TestOrderStrokes:

    def test_empty_returns_empty(self):
        assert _order_strokes([]) == []

    def test_single_stroke_unchanged(self):
        s = [(0, 0), (10, 0)]
        result = _order_strokes([s])
        assert result == [s]

    def test_prefers_nearest_start(self):
        # After A (ends at (0,0)): B starts at (1,0) — closer than C at (100,100)
        A = [(10, 10), (0, 0)]
        B = [(1, 0), (5, 0)]
        C = [(100, 100), (110, 100)]
        result = _order_strokes([A, B, C])
        assert result[0] == A
        assert result[1] == B
        assert result[2] == C

    def test_reverses_when_end_is_closer(self):
        # A ends at (0,0). B = [(100,0),(1,0)] — B's end (1,0) is closer to (0,0)
        A = [(10, 10), (0, 0)]
        B = [(100, 0), (1, 0)]
        result = _order_strokes([A, B])
        assert result[0] == A
        assert result[1] == list(reversed(B))

    def test_two_strokes_efficient_order(self):
        # s1 ends at (50, 0). s2 starts at (51, 0) — 1px away vs end at (200, 0) — 150px away.
        # Start wins: s2 must not be reversed.
        s1 = [(0, 0), (50, 0)]
        s2 = [(51, 0), (200, 0)]
        result = _order_strokes([s1, s2])
        assert result[0] == s1
        assert result[1] == s2


# ─────────────────────────────────────────────────────────────────────────────
# extract_from_edges (groove mask → strokes)
# ─────────────────────────────────────────────────────────────────────────────

class TestExtractFromEdges:

    def test_blank_mask_returns_empty(self, mask_blank):
        result = extract_from_edges(mask_blank)
        assert isinstance(result, ExtractedPath)
        assert result.total_strokes == 0
        assert result.strokes == []
        assert result.total_points == 0

    def test_rectangle_returns_nonzero_strokes(self, mask_rectangle):
        result = extract_from_edges(mask_rectangle)
        assert result.total_strokes >= 1
        assert result.total_points > 0

    def test_min_contour_filter_rejects_tiny_dot(self, mask_tiny_dot):
        result = extract_from_edges(mask_tiny_dot, min_contour_pixels=20)
        assert result.total_strokes == 0

    def test_min_contour_filter_passes_with_lower_threshold(self, mask_long_line):
        result = extract_from_edges(mask_long_line, min_contour_pixels=3)
        assert result.total_strokes >= 1

    def test_fallback_10px_spacing_without_scale(self, mask_long_line):
        """Without a mm-per-pixel scale (Test Mode), spacing falls back to 10 px."""
        result = extract_from_edges(mask_long_line)
        assert result.total_strokes >= 1
        for stroke in result.strokes:
            for i in range(1, len(stroke)):
                dx = stroke[i][0] - stroke[i - 1][0]
                dy = stroke[i][1] - stroke[i - 1][1]
                dist = math.sqrt(dx * dx + dy * dy)
                assert dist <= 15, f"point spacing {dist:.1f}px unexpectedly large"

    def test_mm_spacing_converts_via_scale(self, mask_long_line):
        """spacing_mm=20 at 1 mm/px must give ~20 px between waypoints."""
        result = extract_from_edges(mask_long_line, spacing_mm=20.0, mm_per_px=1.0)
        assert result.total_strokes >= 1
        for stroke in result.strokes:
            # all gaps except the closing one should be ≈20 px
            for i in range(1, len(stroke) - 1):
                dx = stroke[i][0] - stroke[i - 1][0]
                dy = stroke[i][1] - stroke[i - 1][1]
                dist = math.sqrt(dx * dx + dy * dy)
                assert 16 <= dist <= 24, f"spacing {dist:.1f}px not ≈20px"

    def test_wider_spacing_gives_fewer_waypoints(self, mask_long_line):
        near = extract_from_edges(mask_long_line, spacing_mm=10.0, mm_per_px=1.0)
        far  = extract_from_edges(mask_long_line, spacing_mm=100.0, mm_per_px=1.0)
        assert far.total_points < near.total_points

    def test_dense_skeleton_returned_alongside(self, mask_long_line):
        """strokes_dense (white preview line) is denser than the waypoint strokes."""
        result = extract_from_edges(mask_long_line, spacing_mm=50.0, mm_per_px=1.0)
        assert result.strokes_dense
        dense_pts = sum(len(s) for s in result.strokes_dense)
        assert dense_pts > result.total_points

    def test_single_pixel_does_not_crash(self, mask_blank):
        """A mask with a single bright pixel must not crash the chain extractor."""
        single_pixel = mask_blank.copy()
        single_pixel[60, 50] = 255
        result = extract_from_edges(single_pixel, min_contour_pixels=1)
        assert isinstance(result, ExtractedPath)

    def test_two_lines_return_multiple_strokes(self, mask_two_lines):
        result = extract_from_edges(mask_two_lines)
        assert result.total_strokes >= 2

    def test_offset_shifts_into_full_frame(self, mask_long_line):
        result = extract_from_edges(mask_long_line, offset=(100, 50))
        assert result.total_strokes >= 1
        # Every point should be shifted by the crop origin.
        assert all(pt[0] >= 100 and pt[1] >= 50 for s in result.strokes for pt in s)

    def test_strokes_contain_valid_pixel_coords(self, mask_rectangle):
        result = extract_from_edges(mask_rectangle)
        for stroke in result.strokes:
            for pt in stroke:
                assert len(pt) == 2
                assert 0 <= pt[0] <= 640
                assert 0 <= pt[1] <= 480

# ─────────────────────────────────────────────────────────────────────────────
# join_strokes (Distance Threshold — merge strokes whose endpoints nearly touch)
# ─────────────────────────────────────────────────────────────────────────────

class TestJoinStrokes:

    def test_zero_threshold_is_a_no_op(self):
        strokes = [[(0, 0), (10, 0)], [(12, 0), (22, 0)]]
        assert join_strokes(strokes, join_px=0) == strokes

    def test_single_stroke_unchanged(self):
        strokes = [[(0, 0), (10, 0)]]
        assert join_strokes(strokes, join_px=50) == strokes

    def test_gap_below_threshold_merges(self):
        """End of A is 2 px from start of B → one stroke of all four points."""
        merged = join_strokes([[(0, 0), (10, 0)], [(12, 0), (22, 0)]], join_px=5)
        assert len(merged) == 1
        assert merged[0] == [(0, 0), (10, 0), (12, 0), (22, 0)]

    def test_gap_above_threshold_stays_separate(self):
        strokes = [[(0, 0), (10, 0)], [(60, 0), (70, 0)]]
        assert len(join_strokes(strokes, join_px=5)) == 2

    def test_direction_ignored_end_to_end(self):
        """B runs backwards; its END is the near endpoint, so B is flipped."""
        merged = join_strokes([[(0, 0), (10, 0)], [(22, 0), (12, 0)]], join_px=5)
        assert len(merged) == 1
        assert merged[0] == [(0, 0), (10, 0), (12, 0), (22, 0)]

    def test_start_to_start_join_flips_the_first(self):
        """Both strokes start at the meeting point — one must be reversed."""
        merged = join_strokes([[(10, 0), (0, 0)], [(12, 0), (22, 0)]], join_px=5)
        assert len(merged) == 1
        assert merged[0] == [(0, 0), (10, 0), (12, 0), (22, 0)]

    def test_nearest_endpoint_wins(self):
        """A's end has two candidates in range; only the nearer one connects."""
        strokes = [
            [(0, 0), (10, 0)],      # A — end at (10, 0)
            [(14, 0), (24, 0)],     # B — 4 px away
            [(12, 0), (12, 20)],    # C — 2 px away, the nearer one
        ]
        merged = join_strokes(strokes, join_px=8)
        assert len(merged) == 2                      # A+C merged, B alone
        joined = max(merged, key=len)
        assert (0, 0) in joined and (12, 20) in joined
        assert (24, 0) not in joined

    def test_crossing_stroke_doubles_the_threshold(self):
        """A 15 px gap is rejected at threshold 10, accepted when a third
        stroke crosses the connecting line (10 → 20 effective)."""
        a = [(0, 0), (10, 0)]
        b = [(25, 0), (35, 0)]
        crosser = [(17, -10), (17, 10)]              # cuts the A→B gap
        assert len(join_strokes([a, b], join_px=10)) == 2
        merged = join_strokes([a, b, crosser], join_px=10)
        lengths = sorted(len(s) for s in merged)
        assert lengths == [2, 4]                     # crosser alone + A joined to B

    def test_crossing_does_not_extend_beyond_double(self):
        """Even with a crossing stroke, a gap past 2x the threshold is refused."""
        a = [(0, 0), (10, 0)]
        b = [(45, 0), (55, 0)]                       # 35 px gap, 2x threshold = 20
        crosser = [(27, -10), (27, 10)]
        assert len(join_strokes([a, b, crosser], join_px=10)) == 3

    def test_chain_of_three_merges_into_one(self):
        strokes = [[(0, 0), (10, 0)], [(12, 0), (22, 0)], [(24, 0), (34, 0)]]
        merged = join_strokes(strokes, join_px=5)
        assert len(merged) == 1
        assert len(merged[0]) == 6

    def test_loop_is_refused(self):
        """Three strokes in a triangle: two joins accepted, the closing one
        refused, so the result is one open polyline (never a closed ring)."""
        strokes = [
            [(0, 0), (100, 0)],
            [(102, 2), (50, 90)],
            [(48, 88), (-2, 2)],
        ]
        merged = join_strokes(strokes, join_px=6)
        assert len(merged) == 1
        assert len(merged[0]) == 6                   # all points, no duplication

    def test_every_point_survives(self):
        strokes = [[(0, 0), (10, 0)], [(12, 0), (22, 0)], [(80, 80), (90, 80)]]
        merged = join_strokes(strokes, join_px=5)
        assert sum(len(s) for s in merged) == sum(len(s) for s in strokes)

    def test_no_stroke_joins_to_itself(self):
        """A stroke whose own two ends nearly meet must stay one stroke."""
        strokes = [[(0, 0), (10, 0), (10, 10), (1, 1)]]
        merged = join_strokes(strokes, join_px=5)
        assert len(merged) == 1
        assert len(merged[0]) == 4


# ─────────────────────────────────────────────────────────────────────────────
# extract_from_edges + join_mm (the wired-up Distance Threshold)
# ─────────────────────────────────────────────────────────────────────────────

class TestExtractFromEdgesJoining:

    @staticmethod
    def _broken_line(mask_blank):
        """Two collinear segments with an 8-px gap — one interrupted groove."""
        mask = mask_blank.copy()
        mask[100, 40:120] = 255
        mask[100, 128:210] = 255
        return mask

    def test_join_off_by_default(self, mask_blank):
        result = extract_from_edges(self._broken_line(mask_blank), mm_per_px=1.0)
        assert result.total_strokes == 2

    def test_join_mm_merges_the_broken_line(self, mask_blank):
        result = extract_from_edges(self._broken_line(mask_blank),
                                    mm_per_px=1.0, join_mm=20.0)
        assert result.total_strokes == 1

    def test_join_below_gap_leaves_it_split(self, mask_blank):
        result = extract_from_edges(self._broken_line(mask_blank),
                                    mm_per_px=1.0, join_mm=2.0)
        assert result.total_strokes == 2

    def test_dense_skeleton_reflects_the_join(self, mask_blank):
        """The white preview line must show the same merge as the waypoints."""
        result = extract_from_edges(self._broken_line(mask_blank),
                                    mm_per_px=1.0, join_mm=20.0)
        assert len(result.strokes_dense) == result.total_strokes == 1

    def test_join_scales_with_mm_per_px(self, mask_blank):
        """8 px gap at 4 mm/px = 32 mm — 20 mm is too small to bridge it."""
        mask = self._broken_line(mask_blank)
        assert extract_from_edges(mask, mm_per_px=4.0, join_mm=20.0).total_strokes == 2
        assert extract_from_edges(mask, mm_per_px=4.0, join_mm=40.0).total_strokes == 1

    def test_waypoints_are_continuous_across_the_join(self, mask_blank):
        """Resampling runs after joining, so no oversized step at the seam."""
        result = extract_from_edges(self._broken_line(mask_blank), spacing_mm=10.0,
                                    mm_per_px=1.0, join_mm=20.0)
        stroke = result.strokes[0]
        gaps = [math.dist(stroke[i - 1], stroke[i]) for i in range(1, len(stroke))]
        assert max(gaps) <= 15, f"seam step {max(gaps):.1f}px — resample ran before the join"


# ─────────────────────────────────────────────────────────────────────────────
# pixels_to_robot_coords
# ─────────────────────────────────────────────────────────────────────────────

class TestPixelsToRobotCoords:

    def test_bottom_left_pixel_maps_to_robot_origin(self, simple_workspace):
        # Image rows grow downward, world Y grows upward, so the bottom-left
        # pixel (0, frame_height) is the workspace origin after the vertical flip.
        result = pixels_to_robot_coords([[(0, 480)]], simple_workspace, 640, 480)
        pose = result[0][0]
        assert abs(pose[0]) < 1e-9  # x
        assert abs(pose[1]) < 1e-9  # y
        assert abs(pose[2]) < 1e-9  # z (draw_z_offset=0)

    def test_top_left_pixel_maps_to_y_extent(self, simple_workspace):
        # Top-left pixel (0, 0) maps to the +Y edge after the vertical flip.
        result = pixels_to_robot_coords([[(0, 0)]], simple_workspace, 640, 480)
        pose = result[0][0]
        assert abs(pose[0]) < 1e-9          # x
        assert abs(pose[1] - 0.225) < 1e-6  # y_extent

    def test_far_corner_maps_to_workspace_extent(self, simple_workspace):
        # Top-right pixel (640, 0) is the far corner (x_extent, y_extent).
        result = pixels_to_robot_coords([[(640, 0)]], simple_workspace, 640, 480)
        pose = result[0][0]
        assert abs(pose[0] - 0.3) < 1e-6    # x_extent
        assert abs(pose[1] - 0.225) < 1e-6  # y_extent

    def test_center_pixel_maps_to_workspace_center(self, simple_workspace):
        result = pixels_to_robot_coords([[(320, 240)]], simple_workspace, 640, 480)
        pose = result[0][0]
        assert abs(pose[0] - 0.15) < 1e-6
        assert abs(pose[1] - 0.1125) < 1e-6

    def test_draw_z_offset_applied(self, simple_workspace):
        result = pixels_to_robot_coords([[(0, 0)]], simple_workspace, 640, 480, draw_z_offset=-0.01)
        pose = result[0][0]
        # z_axis=[0,0,1], so z component = -0.01 * 1 = -0.01
        assert abs(pose[2] - (-0.01)) < 1e-9

    def test_tool_orientation_appended(self, simple_workspace):
        result = pixels_to_robot_coords([[(100, 100)]], simple_workspace, 640, 480)
        pose = result[0][0]
        rx, ry, rz = TOOL_ORIENTATION
        assert abs(pose[3] - rx) < 1e-9
        assert abs(pose[4] - ry) < 1e-9
        assert abs(pose[5] - rz) < 1e-9

    def test_empty_strokes_returns_empty(self, simple_workspace):
        result = pixels_to_robot_coords([], simple_workspace, 640, 480)
        assert result == []

    def test_zero_frame_width_raises(self, simple_workspace):
        """BUG DOCUMENTATION: no guard against division by zero."""
        with pytest.raises(ZeroDivisionError):
            pixels_to_robot_coords([[(0, 0)]], simple_workspace, 0, 480)

    def test_missing_workspace_field_raises(self):
        """BUG DOCUMENTATION: no validation before attribute access."""
        bad_ws = SimpleNamespace(origin=[0, 0, 0], x_axis=[1, 0, 0], y_axis=[0, 1, 0],
                                 x_extent=0.3, y_extent=0.225)
        # z_axis deliberately missing
        with pytest.raises(AttributeError):
            pixels_to_robot_coords([[(0, 0)]], bad_ws, 640, 480)

    def test_multiple_strokes_preserved(self, simple_workspace):
        strokes = [[(0, 0), (100, 0)], [(200, 200), (300, 200)]]
        result = pixels_to_robot_coords(strokes, simple_workspace, 640, 480)
        assert len(result) == 2
        assert len(result[0]) == 2
        assert len(result[1]) == 2

    def test_each_pose_has_six_elements(self, simple_workspace):
        result = pixels_to_robot_coords([[(50, 50), (100, 100)]], simple_workspace, 640, 480)
        for pose in result[0]:
            assert len(pose) == 6
