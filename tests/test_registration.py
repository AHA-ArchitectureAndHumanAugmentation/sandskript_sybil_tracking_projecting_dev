"""
Unit tests for registration.py — corner→TCP surface placement. Pure numpy/scipy,
no robot or trimesh needed.
"""
import numpy as np
import pytest

from registration import corner_points, register_pose
from surface import SurfacePose


def _cube_vertices():
    return np.array([[x, y, z]
                     for z in (0.0, 0.3)
                     for y in (0.0, 0.5)
                     for x in (0.0, 1.0)])


class TestCornerPoints:

    def test_cube_gives_eight(self):
        corners = corner_points(_cube_vertices())
        assert len(corners) == 8
        # Every bbox corner is an actual vertex of the cube.
        vs = {tuple(v) for v in _cube_vertices().tolist()}
        assert all(tuple(c) in vs for c in corners)

    def test_flat_sheet_dedupes_to_four(self):
        # A flat plate: top/bottom bbox corners hit the same vertices.
        v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0.5, 0.5, 0]])
        corners = corner_points(v)
        assert len(corners) == 4

    def test_empty_returns_empty(self):
        assert corner_points(np.zeros((0, 3))) == []

    def test_order_is_stable(self):
        # First corner = (−x, −y, z-min) ring per the documented ordering.
        corners = corner_points(_cube_vertices())
        assert corners[0] == [0.0, 0.0, 0.0]
        assert corners[1] == [1.0, 0.0, 0.0]


class TestRegisterPose:

    def test_one_point_translates_corner_to_tcp(self):
        current = SurfacePose(tx=0.4, ty=0.1, tz=0.0, rx=0.0, ry=0.0, rz=30.0)
        corner = [0.2, 0.1, 0.05]
        tcp = [0.75, -0.20, 0.30]
        new = register_pose([corner], [tcp], current)
        # The corner must land exactly on the TCP under the new pose…
        m = new.matrix()
        moved = m[:3, :3] @ np.array(corner) + m[:3, 3]
        assert np.allclose(moved, tcp, atol=1e-9)
        # …and rotation must be untouched (1-point = translation only).
        assert np.allclose([new.rx, new.ry, new.rz],
                           [current.rx, current.ry, current.rz], atol=1e-9)

    def test_three_points_recover_full_pose(self):
        truth = SurfacePose(tx=0.5, ty=-0.2, tz=0.15, rx=10.0, ry=-20.0, rz=45.0)
        m = truth.matrix()
        corners = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.5, 0.0], [1.0, 0.5, 0.2]]
        points = [(m[:3, :3] @ np.array(c) + m[:3, 3]).tolist() for c in corners]
        new = register_pose(corners, points, SurfacePose())   # current pose ignored
        assert np.allclose(new.matrix(), m, atol=1e-6)

    def test_two_points_rejected(self):
        with pytest.raises(ValueError):
            register_pose([[0, 0, 0], [1, 0, 0]], [[0, 0, 0], [1, 0, 0]], SurfacePose())

    def test_mismatched_lengths_rejected(self):
        with pytest.raises(ValueError):
            register_pose([[0, 0, 0]], [], SurfacePose())

    def test_kabsch_never_returns_a_reflection(self):
        # Noisy/degenerate-ish input must still yield det(R) = +1.
        rng = np.random.default_rng(3)
        corners = rng.normal(size=(4, 3)).tolist()
        points = rng.normal(size=(4, 3)).tolist()
        new = register_pose(corners, points, SurfacePose())
        assert np.linalg.det(new.matrix()[:3, :3]) > 0
