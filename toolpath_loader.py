"""
toolpath_loader.py — read saved toolpath bundles back into executable strokes.

Used by the CONTAINED replay tool (replay_main.py). A bundle is one timestamped
folder under paths/ written by path_export.save_bundle; either file in it can be
loaded:

  - path.json    poses read directly (offset already baked in on save).
  - path.script  the URScript is parsed back: the exporter's fixed pattern per
                 stroke is  "# stroke N (M pts)" → movel(approach) →
                 movel(start) → movep(...)×(M−1) → movel(retract), so the
                 stroke = the second movel + the movep poses. Scripts without
                 the "# stroke" markers fall back to a movep-run heuristic
                 (single-point strokes cannot be recovered there).

Both loaders return the SAME poses the file would actuate, so replay executes
them literally (draw_z = 0, offset = 0). Brand-neutral: no robot imports here.
"""
from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path

from config import MAX_TCP_SPEED, PATHS_DIR

Pose = list[float]                    # [x, y, z, rx, ry, rz] — metres / rad
Strokes = list[list[Pose]]

_POSE_RE   = re.compile(r"p\[([^\]]+)\]")
_V_RE      = re.compile(r"v=([0-9.]+)")
_R_RE      = re.compile(r"r=([0-9.]+)")
_STROKE_RE = re.compile(r"#\s*stroke\s+\d+\s+\((\d+)\s+pts?\)")
_MODE_RE   = re.compile(r"mode:\s*(\w+)")


@dataclass
class Toolpath:
    """One loaded bundle, ready for a ReplayBackend to execute literally."""
    name: str                          # folder name (the timestamp)
    folder: Path
    strokes: Strokes
    meta: dict = field(default_factory=dict)
    source: str = ""                   # "path.json" | "path.script"

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
        has_json = (folder / "path.json").is_file()
        has_script = (folder / "path.script").is_file()
        if not (has_json or has_script):
            continue
        entries.append({
            "name": folder.name,
            "has_json": has_json,
            "has_script": has_script,
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
    Load a bundle folder. ``prefer`` = "json" | "script" | None (None = json if
    present, else script). Raises ValueError on missing/malformed files.
    """
    folder = Path(folder)
    json_file = folder / "path.json"
    script_file = folder / "path.script"

    if prefer == "json" or (prefer is None and json_file.is_file()):
        if not json_file.is_file():
            raise ValueError(f"{json_file} not found")
        strokes, meta = parse_json(json_file.read_text(encoding="utf-8"))
        source = "path.json"
    elif prefer in (None, "script"):
        if not script_file.is_file():
            raise ValueError(f"no path.json or path.script in {folder}")
        strokes, meta = parse_urscript(script_file.read_text(encoding="utf-8"))
        source = "path.script"
    else:
        raise ValueError(f"unknown format {prefer!r} (use 'json' or 'script')")

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


def parse_urscript(text: str) -> tuple[Strokes, dict]:
    """
    path.script → (strokes, meta). Meta is reconstructed from the script:
    speed from v=, blend from the largest r= (per-stroke clamping only shrinks
    it), safety from the approach→start distance, mode from the header comment.
    """
    moves: list[tuple[str, Pose, float | None, float | None]] = []
    markers: list[int] = []            # index into `moves` where a stroke starts
    mode = None
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith("#"):
            if _STROKE_RE.search(line):
                markers.append(len(moves))
            elif mode is None:
                m = _MODE_RE.search(line)
                if m:
                    mode = m.group(1)
            continue
        kind = ("movel" if line.startswith("movel(")
                else "movep" if line.startswith("movep(") else None)
        if kind is None:
            continue
        pm = _POSE_RE.search(line)
        if pm is None:
            raise ValueError(f"{kind} without p[...] pose: {line}")
        pose = [float(v) for v in pm.group(1).split(",")]
        _check_pose(pose, line)
        vm, rm = _V_RE.search(line), _R_RE.search(line)
        moves.append((kind, pose,
                      float(vm.group(1)) if vm else None,
                      float(rm.group(1)) if rm else None))

    strokes = (_strokes_from_blocks(moves, markers) if markers
               else _strokes_from_movep_runs(moves))

    meta: dict = {"mode": mode} if mode else {}
    speeds = [v for _, _, v, _ in moves if v is not None]
    if speeds:
        meta["speed_mps"] = speeds[0]
        meta["speed_pct"] = round(speeds[0] / MAX_TCP_SPEED * 100.0, 1)
    blends = [r for k, _, _, r in moves if k == "movep" and r is not None]
    if blends:
        meta["blend_mm"] = round(max(blends) * 1000.0, 2)
    if markers and len(moves) >= markers[0] + 2:
        a, b = moves[markers[0]][1], moves[markers[0] + 1][1]
        meta["safety_mm"] = round(math.dist(a[:3], b[:3]) * 1000.0, 1)
    return strokes, meta


def _strokes_from_blocks(moves, markers) -> Strokes:
    """Exporter layout per block: movel approach, movel start, movep×, movel lift."""
    strokes: Strokes = []
    bounds = list(markers) + [len(moves)]
    for lo, hi in zip(bounds, bounds[1:]):
        block = moves[lo:hi]
        if len(block) < 3 or block[0][0] != "movel" or block[1][0] != "movel":
            raise ValueError("malformed stroke block in path.script")
        strokes.append([block[1][1]] + [p for k, p, _, _ in block[2:-1]
                                        if k == "movep"])
    return strokes


def _strokes_from_movep_runs(moves) -> Strokes:
    """No markers: each maximal movep run + the movel just before it."""
    strokes: Strokes = []
    current: list[Pose] | None = None
    prev_movel: Pose | None = None
    for kind, pose, _, _ in moves:
        if kind == "movep":
            if current is None:
                if prev_movel is None:
                    raise ValueError("movep before any movel in path.script")
                current = [prev_movel]
            current.append(pose)
        else:
            if current is not None:
                strokes.append(current)
                current = None
            prev_movel = pose
    if current is not None:
        strokes.append(current)
    return strokes


def _check_pose(pose, where: str) -> None:
    if (not isinstance(pose, (list, tuple)) or len(pose) != 6
            or not all(isinstance(v, (int, float)) and math.isfinite(v)
                       for v in pose)):
        raise ValueError(f"bad pose at {where}")
