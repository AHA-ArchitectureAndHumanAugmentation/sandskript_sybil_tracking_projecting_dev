"""
path_length.py — how far the tool actually travels while drawing.

Lives in its own module (not main.py) so external tools can import it without
side effects, same reason as reach.py.

This measures the DRAWING motion only — the green (or red) line in the Path
Preview. Pen-up travels, retracts and the approach onto each stroke are not
counted: they are positioning moves, not deposited material, and the limit this
feeds exists to bound how much gets laid down.

Everything upstream is already baked into the numbers it is handed:

  * **spacing** — the waypoints ARE the resampled path, so a coarser Spacing
    genuinely shortens the polyline (it cuts corners).
  * **connecting distance** — joining concatenates two strokes' point lists, so
    the gap that was closed becomes a real segment and is counted.
  * **surface projection + scale** — the strokes passed in are the robot-space
    poses in metres, already ray-cast onto the loaded mesh. Summing distances
    between them is therefore a length on the actual surface, not on the flat
    camera image.
  * **radius** — the corner zone, handled here (see ``blended_length``).
"""
from __future__ import annotations

import math

Pose = list           # [x, y, z, rx, ry, rz] — metres
Strokes = list        # list[list[Pose]]


def polyline_length(stroke: Strokes) -> float:
    """Straight-line length through every waypoint, in metres."""
    return sum(math.dist(a[:3], b[:3]) for a, b in zip(stroke, stroke[1:]))


def _corner_saving(prev: Pose, vertex: Pose, nxt: Pose, r: float) -> float:
    """
    Metres saved at one corner by rounding it with zone radius ``r``.

    The controller leaves the incoming segment ``r`` before the waypoint and
    rejoins the outgoing one ``r`` after it, following a circular arc tangent to
    both. So ``2r`` of straight line is replaced by that arc:

        interior angle θ  →  arc radius r·tan(θ/2), swept through (π − θ)
        saving = 2r − r·tan(θ/2)·(π − θ)

    Degenerate cases fall out correctly: a straight-through waypoint (θ = π)
    saves nothing, and a full doubling-back (θ = 0) saves the whole 2r.
    """
    if r <= 0.0:
        return 0.0
    ax, ay, az = (vertex[0] - prev[0], vertex[1] - prev[1], vertex[2] - prev[2])
    bx, by, bz = (nxt[0] - vertex[0], nxt[1] - vertex[1], nxt[2] - vertex[2])
    la = math.sqrt(ax * ax + ay * ay + az * az)
    lb = math.sqrt(bx * bx + by * by + bz * bz)
    if la <= 1e-12 or lb <= 1e-12:
        return 0.0
    # Turn angle between the two segments; θ (interior) = π − turn.
    cos_turn = (ax * bx + ay * by + az * bz) / (la * lb)
    turn = math.acos(max(-1.0, min(1.0, cos_turn)))
    theta = math.pi - turn
    if turn <= 1e-9:
        return 0.0                       # straight through: nothing to round
    arc = r * math.tan(theta / 2.0) * turn
    return max(2.0 * r - arc, 0.0)


def blended_length(strokes: Strokes, blend_m: float = 0.0) -> float:
    """
    Total drawn length in metres, with corner rounding taken into account.

    ``blend_m`` is the exec-bar Radius. It is clamped per stroke exactly as the
    executor clamps it (``path_export.stroke_blend`` — 45% of that stroke's
    shortest segment), so this measures the path the robot is actually driven
    through, not the un-blended polyline.

    Note the Path Preview draws each rounded corner as a quadratic curve rather
    than a true circular arc; that is a drawing shortcut, and the difference in
    length is far below the millimetre this is reported in.
    """
    from path_export import stroke_blend   # local: keeps this module import-light

    total = 0.0
    for stroke in strokes:
        if len(stroke) < 2:
            continue
        total += polyline_length(stroke)
        r = stroke_blend(stroke, blend_m)
        if r <= 0.0:
            continue
        for i in range(1, len(stroke) - 1):
            total -= _corner_saving(stroke[i - 1], stroke[i], stroke[i + 1], r)
    return max(total, 0.0)


def total_length_mm(strokes: Strokes, blend_mm: float = 0.0) -> float:
    """``blended_length`` in the millimetres the UI works in."""
    return blended_length(strokes, blend_mm / 1000.0) * 1000.0


def exceeds_limit(strokes: Strokes, blend_mm: float, max_mm: float) -> tuple[bool, float]:
    """
    ``(over_limit, actual_mm)`` for the Max Total Length box.

    ``max_mm <= 0`` means no limit — the same "0 = off" convention the Distance
    Threshold box uses — and never reports over.
    """
    actual = total_length_mm(strokes, blend_mm)
    if max_mm is None or max_mm <= 0.0:
        return False, actual
    return actual > max_mm, actual
