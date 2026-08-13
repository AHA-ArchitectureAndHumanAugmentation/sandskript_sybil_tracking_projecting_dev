"""
registration.py — place the target surface by touching its corners with the TCP.

The surface pose normally comes from the UI sliders (a guess). Registration
measures it instead: pick a corner of the mesh in the Path Preview, freedrive
the robot so the tool tip touches that physical corner, confirm — and the pose
is updated so the mesh corner coincides with the measured TCP point.

Two solvers, chosen by how many corner↔TCP pairs are given:
  • 1 pair  — translation only: rotation is kept from the current pose (set the
    rotation sliders first). This is the current UI flow.
  • ≥3 pairs — full rigid pose via Kabsch/SVD (standard 3-point workpiece
    calibration). Already implemented so the future multi-point UI only needs
    to collect more pairs.
(2 pairs under-determine rotation and are rejected.)

Corner candidates are the mesh vertices nearest the local bounding-box corners,
deduplicated — a sheet-like Rhino surface yields its 4 outline corners. The
numbering here is the numbering shown in the browser (markers + popup list).
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from surface import SurfacePose


def corner_points(vertices: np.ndarray) -> list[list[float]]:
    """
    Candidate touch-off corners: for each of the 8 local-frame bbox corners,
    the nearest actual mesh vertex (a real, physically touchable point).
    Duplicates collapse (order preserved), so a flat sheet gives 4 corners.
    Order: z-min ring then z-max ring, each (−x−y, +x−y, −x+y, +x+y).
    """
    v = np.asarray(vertices, dtype=np.float64).reshape(-1, 3)
    if v.size == 0:
        return []
    lo, hi = v.min(axis=0), v.max(axis=0)
    corners_bbox = [
        [x, y, z]
        for z in (lo[2], hi[2])
        for y in (lo[1], hi[1])
        for x in (lo[0], hi[0])
    ]
    out: list[list[float]] = []
    seen: set[int] = set()
    for c in corners_bbox:
        i = int(np.argmin(((v - c) ** 2).sum(axis=1)))
        if i in seen:
            continue
        seen.add(i)
        out.append([round(float(x), 6) for x in v[i]])
    return out


def register_pose(
    corners_local: list[list[float]],
    points_base: list[list[float]],
    current_pose: SurfacePose,
) -> SurfacePose:
    """
    Solve the surface pose from corner↔TCP pairs.

    ``corners_local`` are mesh corners in the LOCAL frame; ``points_base`` the
    measured TCP positions (robot base frame, metres) touching them, same order.
    Returns a new SurfacePose; ``current_pose`` supplies the rotation in the
    1-point case and is otherwise untouched.
    """
    c = np.asarray(corners_local, dtype=np.float64).reshape(-1, 3)
    p = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
    if len(c) != len(p) or len(c) == 0:
        raise ValueError("need equal, non-zero numbers of corners and points")

    if len(c) == 1:
        # Translation only: keep the current rotation R, move the mesh so
        # R @ corner + t = tcp  →  t = tcp − R @ corner.
        R = current_pose.matrix()[:3, :3]
        t = p[0] - R @ c[0]
        rot = Rotation.from_matrix(R)
    elif len(c) == 2:
        raise ValueError("2 points under-determine rotation — use 1 or ≥3")
    else:
        # Kabsch: best-fit rigid transform local → base over all pairs.
        cc, pc = c.mean(axis=0), p.mean(axis=0)
        H = (c - cc).T @ (p - pc)
        U, _S, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))     # avoid a reflection
        R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
        t = pc - R @ cc
        rot = Rotation.from_matrix(R)

    rx, ry, rz = rot.as_euler("xyz", degrees=True)  # matches SurfacePose.matrix()
    return SurfacePose(
        tx=float(t[0]), ty=float(t[1]), tz=float(t[2]),
        rx=float(rx), ry=float(ry), rz=float(rz),
    )
