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
