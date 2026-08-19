"""
Unit tests for workspace.scene_mm_per_px — the single mm→pixel scale behind the
mm-based groove filters and the mm spacings (Spacing, Distance Threshold).

Regression guard: with BOTH a Test-Mode workspace and a loaded surface, the
scale must come from the surface, because stroke mapping projects onto the
surface whenever one is loaded. Using the 0.30 m synthetic workspace instead
made every mm label lie by the ratio of surface width to 0.30 m.
"""
from config import DEPTH_HEIGHT, DEPTH_WIDTH
from workspace import WorkspaceConfig, scene_mm_per_px


class FakeSurface:
    """Duck-typed stand-in: scene_mm_per_px only calls drawing_mm_per_px."""

    def __init__(self, mm_per_px: float):
        self._mm_per_px = mm_per_px
        self.calls: list[tuple[int, int]] = []

    def drawing_mm_per_px(self, frame_width: int, frame_height: int) -> float:
        self.calls.append((frame_width, frame_height))
        return self._mm_per_px


class TestSceneMmPerPx:

    def test_neither_returns_none(self):
        assert scene_mm_per_px(None, None) is None

    def test_workspace_only(self):
        ws = WorkspaceConfig.simulation()          # 0.30 m across the frame
        expected = (ws.x_extent / DEPTH_WIDTH) * 1000.0
        assert abs(scene_mm_per_px(ws, None) - expected) < 1e-9

    def test_surface_only(self):
        surf = FakeSurface(1.875)
        assert scene_mm_per_px(None, surf) == 1.875
        assert surf.calls == [(DEPTH_WIDTH, DEPTH_HEIGHT)]

    def test_surface_wins_over_workspace(self):
        # The regression: Test Mode active AND a surface loaded — mapping uses
        # the surface, so the mm scale must too, not the 0.30 m workspace.
        ws = WorkspaceConfig.simulation()
        surf = FakeSurface(1.875)
        assert scene_mm_per_px(ws, surf) == 1.875


class TestCombinedCanvasSize:
    """Every view is the COMBINED canvas of the whole camera rig, so the frame
    the scale is measured over is no longer one camera's 640×480."""

    def test_the_surface_is_fitted_to_the_canvas_not_to_one_camera(self):
        surf = FakeSurface(1.0)
        scene_mm_per_px(None, surf, frame_size=(1800, 620))
        assert surf.calls == [(1800, 620)]

    def test_a_wider_canvas_makes_each_pixel_smaller(self):
        ws = WorkspaceConfig.simulation()
        one = scene_mm_per_px(ws, None, frame_size=(DEPTH_WIDTH, DEPTH_HEIGHT))
        wide = scene_mm_per_px(ws, None, frame_size=(2 * DEPTH_WIDTH, DEPTH_HEIGHT))
        assert wide == one / 2

    def test_a_missing_or_nonsense_size_falls_back_to_one_camera(self):
        surf = FakeSurface(1.0)
        scene_mm_per_px(None, surf, frame_size=None)
        scene_mm_per_px(None, surf, frame_size=(0, 0))
        assert surf.calls == [(DEPTH_WIDTH, DEPTH_HEIGHT)] * 2


class TestSimulationAspect:
    """Test Mode's synthetic plane follows the frame's shape, so a wide
    multi-camera canvas is not squashed into a single camera's 4:3."""

    def test_default_follows_one_camera(self):
        ws = WorkspaceConfig.simulation()
        assert ws.y_extent / ws.x_extent == DEPTH_HEIGHT / DEPTH_WIDTH

    def test_a_wide_canvas_gives_a_wide_plane(self):
        ws = WorkspaceConfig.simulation(frame_aspect=3.0)
        assert ws.x_extent / ws.y_extent == 3.0

    def test_the_plane_stays_isotropic(self):
        """mm across == mm down: the whole point of deriving y from the aspect."""
        aspect = 1800 / 620
        ws = WorkspaceConfig.simulation(frame_aspect=aspect)
        assert (ws.x_extent / 1800) == (ws.y_extent / 620)
