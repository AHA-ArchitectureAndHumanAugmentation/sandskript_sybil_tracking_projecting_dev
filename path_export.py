"""
path_export.py — save a generated toolpath to disk in an open, robot-neutral form.

Each save produces a timestamped subfolder under ``paths/`` containing:
  - path.json    the strokes as 6-DOF poses PLUS a full plane/frame per
                 waypoint (origin + x/y/z axes), for frame-guided workflows.
                 This is the EXECUTABLE file: the replay tool reads it back and
                 the ABB frames the robot moves through come straight from it.
  - path.script  URScript (movel travels + movep drawing) of the same strokes.
  - preview.png  the 3D Path Preview image, so an operator can identify the file.
  - mask.png     the thick detected-region mask the path came from.
  - skeleton.png the 1-px groove centrelines the strokes were traced along.

The last two are the detection stage as it stood at save time, cropped to the
same region the strokes were extracted from — what the sand actually looked
like to the detector, kept alongside the path it produced.

⚠ path.script is a UR-format file kept for continuity — a GoFa cannot run it,
and nothing in this program reads it back any more. It records the same poses
as path.json in a second, human-readable form; treat it as a record, not as
something to load onto a controller. Motion on the GoFa is sent live over RRC
from path.json, whose per-waypoint plane is exactly what an ABB frame needs.

Strokes are the robot-space poses from Generate Path: a list of strokes, each a
list of ``[x, y, z, rx, ry, rz]`` (metres + axis-angle rotation vector). The
run-time normal offset is baked in on save so the saved path matches what would
execute.
"""
from __future__ import annotations

import base64
import json
import math
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from config import (
    BLEND_ZONE_M, DRAW_ACCEL, PATHS_DIR, PREVIEW_MAX_BYTES, TRAVEL_ACCEL,
)

Pose = list           # [x, y, z, rx, ry, rz]
Strokes = list        # list[list[Pose]]

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


# ── geometry helpers (mirror path_executor) ──────────────────────────────────
def _tool_axis(pose: Pose) -> np.ndarray:
    """Outward surface normal = opposite the tool approach (−tool Z)."""
    return -Rotation.from_rotvec(pose[3:6]).apply([0.0, 0.0, 1.0])


def _offset_pose(pose: Pose, dist: float) -> Pose:
    n = _tool_axis(pose)
    return [pose[0] + dist * n[0], pose[1] + dist * n[1], pose[2] + dist * n[2],
            pose[3], pose[4], pose[5]]


def _retract(pose: Pose, dist: float) -> Pose:
    return _offset_pose(pose, dist)   # retract = move outward along the normal


def stroke_blend(waypoints: Strokes, blend_m: float) -> float:
    """
    Clamp a requested corner zone radius for one stroke. A zone that reaches
    half the distance to the next waypoint has nothing left to round — the
    controller starts cutting the corner off rather than following it — and
    resampling can leave one short tail segment, so cap at 45% of the stroke's
    SHORTEST segment.
    """
    blend = max(float(blend_m), 0.0)
    if blend <= 0.0 or len(waypoints) < 2:
        return blend
    min_seg = min(
        math.dist(a[:3], b[:3]) for a, b in zip(waypoints, waypoints[1:])
    )
    return min(blend, 0.45 * min_seg)


# ── URScript (record only — see the module docstring) ─────────────────────────
def build_urscript(strokes: Strokes, speed: float, safety: float,
                   header: str = "", name: str = "draw_path",
                   blend_m: float = BLEND_ZONE_M) -> str:
    """
    URScript program: for each stroke, retract-approach → start → movep through
    the waypoints → retract. ``speed`` m/s is used for travels and drawing so the
    motion is uniform; ``safety`` m is the retract distance along the tool axis;
    ``blend_m`` is the movep corner radius (clamped per stroke by stroke_blend).

    A UR-format record of the same poses path.json carries — kept for continuity
    and readability, NOT what the GoFa executes. The accelerations here are the
    config values; on the ABB side acceleration is a controller setting, so the
    two are not expected to match instruction for instruction.
    """
    def ps(p: Pose) -> str:
        return "p[" + ", ".join(f"{v:.5f}" for v in p) + "]"

    ta, da = TRAVEL_ACCEL, DRAW_ACCEL
    lines = []
    for hl in header.splitlines():
        lines.append(f"# {hl}")
    lines.append(f"def {name}():")
    for si, s in enumerate(strokes):
        if len(s) < 1:
            continue
        r = stroke_blend(s, blend_m)
        lines.append(f"  # stroke {si + 1} ({len(s)} pts)")
        lines.append(f"  movel({ps(_retract(s[0], safety))}, a={ta:.3f}, v={speed:.4f})")
        lines.append(f"  movel({ps(s[0])}, a={ta:.3f}, v={speed:.4f})")
        for p in s[1:]:
            lines.append(f"  movep({ps(p)}, a={da:.3f}, v={speed:.4f}, r={r:.4f})")
        lines.append(f"  movel({ps(_retract(s[-1], safety))}, a={ta:.3f}, v={speed:.4f})")
    lines.append("end")
    lines.append(f"{name}()")
    return "\n".join(lines) + "\n"


# ── JSON (poses + plane/frame per waypoint) ───────────────────────────────────
def _plane(pose: Pose) -> dict:
    """Tool frame at a waypoint: origin + orthonormal x/y/z axes (z = approach)."""
    R = Rotation.from_rotvec(np.asarray(pose[3:6])).as_matrix()
    return {
        "origin": [round(float(pose[i]), 5) for i in range(3)],
        "xaxis": [round(float(v), 6) for v in R[:, 0]],
        "yaxis": [round(float(v), 6) for v in R[:, 1]],
        "zaxis": [round(float(v), 6) for v in R[:, 2]],
    }


def build_json(strokes: Strokes, meta: dict) -> dict:
    strokes_out = [
        [{"pose": [round(float(v), 5) for v in p], "plane": _plane(p)} for p in s]
        for s in strokes
    ]
    return {"meta": meta, "units": "metres, radians (UR pose = rotation vector)",
            "strokes": strokes_out}


# ── bundle ────────────────────────────────────────────────────────────────────
def _decode_png(data_url: str | None) -> bytes | None:
    if not data_url or "," not in data_url:
        return None
    try:
        return base64.b64decode(data_url.split(",", 1)[1])
    except Exception:
        return None


def is_png_data_url(value, max_bytes: int = PREVIEW_MAX_BYTES) -> bool:
    """
    True for a browser ``canvas.toDataURL("image/png")`` payload worth keeping.

    Participant Mode takes its preview image from whatever a Developer window
    pushes up rather than from a Save click, so this is the gate on a
    client-supplied blob: right prefix, real base64, a genuine PNG signature,
    and small enough to sit in memory until the save.
    """
    if not isinstance(value, str) or not value.startswith("data:image/png;base64,"):
        return False
    if len(value) > max_bytes * 2:       # base64 is ~4/3 of the bytes; cheap pre-check
        return False
    png = _decode_png(value)
    return bool(png) and len(png) <= max_bytes and png[:8] == PNG_SIGNATURE


def save_bundle(
    strokes: Strokes,
    speed: float,
    safety_m: float,
    offset_m: float,
    meta: dict,
    preview_png_data_url: str | None = None,
    base_dir: Path | None = None,
    blend_m: float = BLEND_ZONE_M,
    mask=None,
    skeleton=None,
) -> Path:
    """Write path.json + path.script (+ preview/mask/skeleton .png) into a
    timestamped folder. `mask` and `skeleton` are the cropped uint8 detection
    images from Generate Path; either may be None (nothing is written for it)."""
    base = Path(base_dir) if base_dir is not None else PATHS_DIR
    ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    folder = base / ts
    # Guard against two saves within the same second.
    n = 2
    while folder.exists():
        folder = base / f"{ts}_{n}"
        n += 1
    folder.mkdir(parents=True, exist_ok=True)

    final = ([[_offset_pose(p, offset_m) for p in s] for s in strokes]
             if abs(offset_m) > 1e-9 else strokes)

    # The run parameters go in meta as well as the script header, so path.json
    # alone is enough to reproduce the run — the replay tool reads them there.
    meta = dict(meta)
    meta.setdefault("saved", ts)
    meta["speed_mps"] = round(float(speed), 4)
    meta["safety_mm"] = round(safety_m * 1000.0, 1)
    meta["blend_mm"] = round(blend_m * 1000.0, 2)
    (folder / "path.json").write_text(
        json.dumps(build_json(final, meta), indent=2), encoding="utf-8")

    header = (
        f"depth-cam-to-robot toolpath — {meta.get('saved', ts)}\n"
        f"mode: {meta.get('mode', '?')}  surface: {meta.get('surface_name', '-')}\n"
        f"speed: {speed:.3f} m/s  offset: {meta.get('offset_mm', 0):.1f} mm  "
        f"safety: {meta.get('safety_mm', 0):.0f} mm  "
        f"blend: {blend_m * 1000:.1f} mm  "
        f"strokes: {meta.get('stroke_count', len(final))}\n"
        f"UR-format record of the poses in path.json — NOT what the GoFa runs."
    )
    (folder / "path.script").write_text(
        build_urscript(final, speed, safety_m, header=header, blend_m=blend_m),
        encoding="utf-8")

    png = _decode_png(preview_png_data_url)
    if png:
        (folder / "preview.png").write_bytes(png)

    # The detection stage that produced this path, kept beside it.
    _write_png(folder / "mask.png", mask)
    _write_png(folder / "skeleton.png", skeleton)
    return folder


def _write_png(path: Path, image) -> None:
    """
    Save one detection image. Encoded in memory rather than through
    cv2.imwrite, which resolves the filename itself and silently writes
    nothing when the path is not representable in the system codepage.
    """
    if image is None:
        return
    arr = np.asarray(image)
    if arr.size == 0:
        return
    ok, buf = cv2.imencode(".png", arr)
    if ok:
        path.write_bytes(buf.tobytes())
