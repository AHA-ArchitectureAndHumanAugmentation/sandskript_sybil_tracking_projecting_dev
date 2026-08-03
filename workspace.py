from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

from config import DEPTH_HEIGHT, DEPTH_WIDTH


@dataclass
class WorkspaceConfig:
    origin:   list[float]  # P0 [x, y, z] in robot base frame
    x_axis:   list[float]  # unit vector
    y_axis:   list[float]  # unit vector (re-orthogonalized)
    z_axis:   list[float]  # x_axis × y_axis (normal to work surface, points away from surface)
    x_extent: float         # metres: distance P0 → Px
    y_extent: float         # metres: x_extent × (DEPTH_HEIGHT / DEPTH_WIDTH) — isotropic scaling

    @classmethod
    def from_points(
        cls,
        p0: list[float],
        px: list[float],
        py: list[float],
    ) -> WorkspaceConfig:
        x_vec = [px[i] - p0[i] for i in range(3)]
        y_vec = [py[i] - p0[i] for i in range(3)]

        x_extent = _norm(x_vec)
        y_raw_len = _norm(y_vec)
        if x_extent < 1e-6 or y_raw_len < 1e-6:
            raise ValueError("Points are too close together to define a workspace frame.")

        x_axis = _scale(x_vec, 1.0 / x_extent)
        y_raw  = _scale(y_vec, 1.0 / y_raw_len)

        z_axis = _cross(x_axis, y_raw)
        z_len  = _norm(z_axis)
        if z_len < 1e-6:
            raise ValueError("Px and Py are collinear — cannot define a work plane.")
        z_axis = _scale(z_axis, 1.0 / z_len)

        y_axis = _cross(z_axis, x_axis)  # re-orthogonalize
        y_len  = _norm(y_axis)
        y_axis = _scale(y_axis, 1.0 / y_len)

        # y_extent derived from depth-frame aspect ratio — ensures isotropic scaling
        y_extent = x_extent * (DEPTH_HEIGHT / DEPTH_WIDTH)

        return cls(
            origin=list(p0[:3]),
            x_axis=x_axis,
            y_axis=y_axis,
            z_axis=z_axis,
            x_extent=x_extent,
            y_extent=y_extent,
        )

    @classmethod
    def simulation(cls) -> WorkspaceConfig:
        """
        Build a synthetic, axis-aligned workspace for testing the vision pipeline
        without a physical robot. The work plane is a 0.30 m wide rectangle on the
        robot base XY plane (z up); extents follow the camera aspect ratio so the
        preview is isotropic, exactly like a real freedrive-defined workspace.
        """
        p0 = [0.40, -0.15, 0.10]   # origin
        px = [0.70, -0.15, 0.10]   # +X, 0.30 m extent
        py = [0.40,  0.15, 0.10]   # +Y direction
        return cls.from_points(p0, px, py)

    def save(self, path: Path) -> None:
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)

    @classmethod
    def load(cls, path: Path) -> WorkspaceConfig:
        with open(path) as f:
            data = json.load(f)
        return cls(**data)

    def to_browser_dict(self) -> dict:
        return {
            "origin":   self.origin,
            "x_axis":   self.x_axis,
            "y_axis":   self.y_axis,
            "z_axis":   self.z_axis,
            "x_extent": round(self.x_extent, 4),
            "y_extent": round(self.y_extent, 4),
        }


def scene_mm_per_px(workspace, surface_model=None) -> Optional[float]:
    """
    Millimetres per depth pixel for the mm-based groove filters and mm spacings
    (Spacing, Distance Threshold). A loaded surface takes precedence over the
    workspace — stroke mapping does the same (a surface replaces the flat
    workspace) — so a mm entered in the UI stays a mm on whatever the strokes
    actually land on. Duck-typed: ``surface_model`` only needs
    ``drawing_mm_per_px``, ``workspace`` only ``x_extent``.
    """
    if surface_model is not None:
        return surface_model.drawing_mm_per_px(DEPTH_WIDTH, DEPTH_HEIGHT)
    if workspace is not None:
        return (workspace.x_extent / DEPTH_WIDTH) * 1000.0
    return None


# ── Vector helpers (avoid heavy numpy import) ─────────────────────────────────

def _dot(a: list[float], b: list[float]) -> float:
    return sum(a[i] * b[i] for i in range(3))


def _cross(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _norm(v: list[float]) -> float:
    return math.sqrt(sum(x * x for x in v))


def _scale(v: list[float], s: float) -> list[float]:
    return [x * s for x in v]
