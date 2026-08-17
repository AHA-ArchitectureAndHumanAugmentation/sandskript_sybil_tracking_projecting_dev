"""
toolpath_loader.py — read saved toolpath bundles back into executable strokes.

Used by the CONTAINED replay tool (replay_main.py). A bundle is one timestamped
folder under paths/ written by path_export.save_bundle, and path.json is what
carries the motion: poses read directly, with the offset already baked in on
save, so replay executes them literally (draw_z = 0, offset = 0).

Bundles also contain a path.script (URScript), which this module deliberately
IGNORES: a GoFa cannot run it, and parsing UR motion back would invite someone
to trust it as executable. It is a readable record beside path.json, nothing
more — so a folder holding ONLY a path.script does not count as a bundle.

Brand-neutral: no robot imports here.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from config import PATHS_DIR

Pose = list[float]                    # [x, y, z, rx, ry, rz] — metres / rad
Strokes = list[list[Pose]]


@dataclass
class Toolpath:
    """One loaded bundle, ready for a ReplayBackend to execute literally."""
    name: str                          # folder name (the timestamp)
    folder: Path
    strokes: Strokes
    meta: dict = field(default_factory=dict)
    source: str = ""                   # always "path.json"

    @property
    def stroke_count(self) -> int:
        return len(self.strokes)

    @property
    def point_count(self) -> int:
        return sum(len(s) for s in self.strokes)


# ── listing ───────────────────────────────────────────────────────────────────
def list_toolpaths(base_dir: Path | None = None) -> list[dict]:
    """Bundles under ``base_dir`` (default paths/), newest first, no parsing."""
    base = Path(base_dir) if base_dir is not None else PATHS_DIR
    if not base.is_dir():
        return []
    entries = []
    for folder in base.iterdir():
        if not folder.is_dir():
            continue
        if not (folder / "path.json").is_file():
            continue
        entries.append({
            "name": folder.name,
            "has_json": True,
            "has_preview": (folder / "preview.png").is_file(),
            "mtime": folder.stat().st_mtime,
        })
    entries.sort(key=lambda e: e["mtime"], reverse=True)
    for e in entries:
        del e["mtime"]
    return entries


# ── loading ───────────────────────────────────────────────────────────────────
def load_toolpath(folder: Path, prefer: str | None = None) -> Toolpath:
    """
    Load a bundle folder from its path.json. ``prefer`` is accepted so existing
    callers keep working, but "json" is the only format left. Raises ValueError
    on a missing or malformed file.
    """
    folder = Path(folder)
    json_file = folder / "path.json"

    if prefer not in (None, "json"):
        raise ValueError(f"unknown format {prefer!r} — only 'json' is supported")
    if not json_file.is_file():
        raise ValueError(f"no path.json in {folder}")

    strokes, meta = parse_json(json_file.read_text(encoding="utf-8"))
    source = "path.json"

    if not strokes:
        raise ValueError(f"{folder / source}: no strokes found")
    return Toolpath(name=folder.name, folder=folder, strokes=strokes,
                    meta=meta, source=source)


def parse_json(text: str) -> tuple[Strokes, dict]:
    """path.json → (strokes, meta). Poses are taken verbatim."""
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON: {exc}") from exc
    raw = data.get("strokes")
    if not isinstance(raw, list):
        raise ValueError("path.json has no 'strokes' list")
    strokes: Strokes = []
    for si, s in enumerate(raw):
        stroke = []
        for wi, wp in enumerate(s):
            pose = wp.get("pose") if isinstance(wp, dict) else None
            _check_pose(pose, f"stroke {si} waypoint {wi}")
            stroke.append([float(v) for v in pose])
        if stroke:
            strokes.append(stroke)
    meta = data.get("meta") or {}
    return strokes, dict(meta) if isinstance(meta, dict) else {}


def _check_pose(pose, where: str) -> None:
    if (not isinstance(pose, (list, tuple)) or len(pose) != 6
            or not all(isinstance(v, (int, float)) and math.isfinite(v)
                       for v in pose)):
        raise ValueError(f"bad pose at {where}")
