from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from config import (
    CONTOUR_MIN_PIXELS, JOIN_CROSSING_FACTOR, JOIN_DISTANCE_MM,
    RESAMPLE_SPACING_MM, TOOL_ORIENTATION,
)

# Fallback spacing (pixels) used only when no mm-per-pixel scale is available
# (e.g. Test Mode before a workspace/surface is set). Normally spacing is mm.
_FALLBACK_SPACING_PX = 10.0
# Same fallback applied to the join distance: keep it in the same px-per-mm
# proportion as the spacing fallback so both scale together in Test Mode.
_FALLBACK_PX_PER_MM = _FALLBACK_SPACING_PX / RESAMPLE_SPACING_MM

# Dense resample spacing for the skeleton preview line (the white on-surface
# curve in the 3D view). Much finer than the waypoint spacing so it hugs the
# surface; never used for robot motion.
_SKELETON_SPACING_MM = 2.0
_SKELETON_FALLBACK_PX = 2.0


@dataclass
class ExtractedPath:
    strokes: list[list[tuple[float, float]]]  # pixel-coordinate strokes, ordered for efficient travel
    total_strokes: int
    total_points: int
    # Densely resampled copy of the same strokes (~2 mm spacing) for the white
    # on-surface skeleton line in the 3D preview. Not used for robot motion.
    strokes_dense: list[list[tuple[float, float]]] = None


def extract_from_edges(
    edges: np.ndarray,
    min_contour_pixels: int = CONTOUR_MIN_PIXELS,
    offset: tuple[int, int] = (0, 0),
    spacing_mm: float = RESAMPLE_SPACING_MM,
    mm_per_px: Optional[float] = None,
    join_mm: float = JOIN_DISTANCE_MM,
) -> ExtractedPath:
    """
    Turn a binary groove image (1-px-wide centrelines, white on black) into
    ordered, resampled pixel strokes.

    The live groove preview and the final captured path come from identical
    processing. ``offset`` (x0, y0) shifts every point back into full-frame pixel
    coordinates when the grooves were computed on a cropped sub-image, so the
    workspace mapping in pixels_to_robot_coords stays correct.

    ``spacing_mm`` is the target spacing between waypoints in millimetres;
    ``mm_per_px`` converts it into the pixel spacing used for resampling. When no
    scale is available (mm_per_px is None or non-positive) it falls back to a
    fixed pixel spacing so Test Mode still produces a path.

    ``join_mm`` (0 = off) merges strokes whose endpoints nearly touch — see
    ``join_strokes``. Joining runs on the smoothed chains BEFORE resampling and
    ordering, so a merged stroke is resampled as one continuous run (waypoints
    land evenly across the join) and the TSP sees the merged set. The dense
    skeleton comes from the same merged chains, so the white preview line and
    the waypoints always agree about what got connected.
    """
    strokes = _chains_from_edges(edges, min_contour_pixels)

    if not strokes:
        return ExtractedPath(strokes=[], total_strokes=0, total_points=0,
                             strokes_dense=[])

    # Convert the mm spacings into pixels using the scene scale; fall back to
    # fixed pixel spacings when no mm-per-pixel is known.
    if mm_per_px and mm_per_px > 0:
        spacing_px = max(spacing_mm / mm_per_px, 1.0)
        dense_px   = max(_SKELETON_SPACING_MM / mm_per_px, 1.0)
        join_px    = max(join_mm, 0.0) / mm_per_px
    else:
        spacing_px = _FALLBACK_SPACING_PX
        dense_px   = _SKELETON_FALLBACK_PX
        join_px    = max(join_mm, 0.0) * _FALLBACK_PX_PER_MM
    strokes_smoothed  = [smooth_stroke(s) for s in strokes]
    strokes_smoothed  = join_strokes(strokes_smoothed, join_px)
    strokes_resampled = [resample_stroke(s, spacing_px) for s in strokes_smoothed]
    strokes_ordered   = _order_strokes(strokes_resampled)
    strokes_dense     = [resample_stroke(s, dense_px) for s in strokes_smoothed]

    ox, oy = offset
    if ox or oy:
        strokes_ordered = [[(x + ox, y + oy) for x, y in s] for s in strokes_ordered]
        strokes_dense   = [[(x + ox, y + oy) for x, y in s] for s in strokes_dense]

    total_pts = sum(len(s) for s in strokes_ordered)
    return ExtractedPath(
        strokes=strokes_ordered,
        total_strokes=len(strokes_ordered),
        total_points=total_pts,
        strokes_dense=strokes_dense,
    )


def pixels_to_robot_coords(
    strokes: list[list[tuple[int, int]]],
    workspace,
    frame_width: int,
    frame_height: int,
    draw_z_offset: float = 0.0,
) -> list[list[list[float]]]:
    """
    Convert pixel strokes to robot base-frame poses.

    Assumes the camera looks straight down and covers the full workspace rectangle.
    Image rows (v) increase downward, but the workspace Y axis increases upward, so v
    is flipped to keep the preview the same way up as the camera image:
      wx = (u / frame_width)          * workspace.x_extent
      wy = (1 - v / frame_height)     * workspace.y_extent
      p  = origin + wx*x_axis + wy*y_axis + draw_z_offset*z_axis

    Returns list of strokes, each stroke is a list of [x, y, z, rx, ry, rz].
    """
    o  = workspace.origin
    xa = workspace.x_axis
    ya = workspace.y_axis
    za = workspace.z_axis
    xe = workspace.x_extent
    ye = workspace.y_extent
    rx, ry, rz = TOOL_ORIENTATION

    robot_strokes: list[list[list[float]]] = []
    for stroke in strokes:
        robot_stroke: list[list[float]] = []
        for u, v in stroke:
            wx = (u / frame_width)        * xe
            wy = (1.0 - v / frame_height) * ye   # flip: image row grows down, world Y grows up
            px = o[0] + wx * xa[0] + wy * ya[0] + draw_z_offset * za[0]
            py = o[1] + wx * xa[1] + wy * ya[1] + draw_z_offset * za[1]
            pz = o[2] + wx * xa[2] + wy * ya[2] + draw_z_offset * za[2]
            robot_stroke.append([px, py, pz, rx, ry, rz])
        if robot_stroke:
            robot_strokes.append(robot_stroke)

    return robot_strokes


def _chains_from_edges(
    edge_img: np.ndarray,
    min_len: int,
) -> list[list[tuple[int, int]]]:
    """
    Extract ordered pixel chains from a binary groove image via 8-connectivity
    chain-following. Each edge pixel is visited once, giving the true centerline
    without the double-tracing artefact that cv2.findContours produces on thin edges.
    """
    ys, xs = np.where(edge_img > 0)
    if len(xs) == 0:
        return []
    h, w = edge_img.shape
    remaining: set[tuple[int, int]] = set(zip(xs.tolist(), ys.tolist()))

    def nbrs(x: int, y: int) -> list[tuple[int, int]]:
        return [
            (x + dx, y + dy)
            for dx in (-1, 0, 1) for dy in (-1, 0, 1)
            if (dx, dy) != (0, 0)
            and 0 <= x + dx < w and 0 <= y + dy < h
            and (x + dx, y + dy) in remaining
        ]

    # Pre-find endpoints (≤1 neighbour) to start chains from tips rather than middles.
    endpoint_q = [px for px in remaining if len(nbrs(*px)) <= 1]
    ep_idx = 0
    chains: list[list[tuple[int, int]]] = []

    while remaining:
        start: tuple[int, int] | None = None
        while ep_idx < len(endpoint_q):
            cand = endpoint_q[ep_idx]
            ep_idx += 1
            if cand in remaining:
                start = cand
                break
        if start is None:
            start = next(iter(remaining))

        chain: list[tuple[int, int]] = []
        x, y = start
        while True:
            chain.append((x, y))
            remaining.discard((x, y))
            nn = nbrs(x, y)
            if not nn:
                break
            x, y = nn[0]

        if len(chain) >= min_len:
            chains.append(chain)

    return chains


def smooth_stroke(
    pts: list[tuple[float, float]],
    iterations: int = 2,
) -> list[tuple[float, float]]:
    """Chaikin corner-cutting: each iteration replaces segments with two interior points."""
    if len(pts) < 3:
        return pts
    for _ in range(iterations):
        new_pts: list[tuple[float, float]] = [pts[0]]
        for i in range(len(pts) - 1):
            x0, y0 = pts[i]
            x1, y1 = pts[i + 1]
            new_pts.append((0.75 * x0 + 0.25 * x1, 0.75 * y0 + 0.25 * y1))
            new_pts.append((0.25 * x0 + 0.75 * x1, 0.25 * y0 + 0.75 * y1))
        new_pts.append(pts[-1])
        pts = new_pts
    return pts


def resample_stroke(
    stroke: list[tuple[float, float]],
    spacing_px: float,
) -> list[tuple[float, float]]:
    """Resample a stroke to approximately `spacing_px` pixel intervals."""
    if len(stroke) < 2:
        return stroke

    cum = [0.0]
    for i in range(1, len(stroke)):
        dx = stroke[i][0] - stroke[i - 1][0]
        dy = stroke[i][1] - stroke[i - 1][1]
        cum.append(cum[-1] + math.sqrt(dx * dx + dy * dy))

    total = cum[-1]
    if total < spacing_px:
        return [stroke[0], stroke[-1]]

    result: list[tuple[float, float]] = []
    target = 0.0
    j = 0

    while target <= total + 1e-9:
        while j + 1 < len(cum) and cum[j + 1] < target:
            j += 1
        if j + 1 >= len(stroke):
            break
        seg_len = cum[j + 1] - cum[j]
        t = (target - cum[j]) / seg_len if seg_len > 1e-9 else 0.0
        x = stroke[j][0] + t * (stroke[j + 1][0] - stroke[j][0])
        y = stroke[j][1] + t * (stroke[j + 1][1] - stroke[j][1])
        result.append((x, y))
        target += spacing_px

    if not result or result[-1] != stroke[-1]:
        result.append(stroke[-1])

    return result


def join_strokes(
    strokes: list[list[tuple[float, float]]],
    join_px: float,
    crossing_factor: float = JOIN_CROSSING_FACTOR,
) -> list[list[tuple[float, float]]]:
    """
    Merge strokes whose endpoints nearly touch, so an interrupted groove becomes
    one continuous toolpath instead of several short ones.

    Rules (all distances in pixels; the caller converts the mm box value):
      * Only endpoints count — the start or the end of a stroke, direction
        irrelevant. A stroke never joins to itself.
      * A pair qualifies when the gap between the two endpoints is below
        ``join_px``, OR below ``crossing_factor * join_px`` when the straight
        line closing that gap is crossed by a THIRD stroke. A crossing means the
        two ends were interrupted by another groove, which is exactly the case
        where they most likely belong together — so the threshold is relaxed.
      * Each endpoint takes at most ONE partner. Candidates are accepted
        shortest-gap-first, so every endpoint ends up joined to its nearest
        eligible neighbour rather than to whichever was examined first.
      * A join that would close a loop (both ends already in the same merged
        chain) is refused, so the result is always a set of open polylines.

    ``join_px <= 0`` disables joining and returns the strokes unchanged.
    """
    n = len(strokes)
    if join_px <= 0 or n < 2:
        return [list(s) for s in strokes]

    near = float(join_px)
    far  = near * max(crossing_factor, 1.0)

    # Candidate endpoint pairs. Anything beyond the relaxed threshold can never
    # qualify, so the crossing test only runs on the near..far band.
    cands: list[tuple[float, int, int, int, int]] = []
    for i in range(n):
        for ei, a in ((0, strokes[i][0]), (1, strokes[i][-1])):
            for j in range(i + 1, n):
                for ej, b in ((0, strokes[j][0]), (1, strokes[j][-1])):
                    d = math.hypot(a[0] - b[0], a[1] - b[1])
                    if d >= far:
                        continue
                    if d >= near and not any(
                        _crosses_gap(strokes[k], a, b)
                        for k in range(n) if k != i and k != j
                    ):
                        continue
                    cands.append((d, i, ei, j, ej))

    cands.sort(key=lambda c: c[0])

    # Greedy shortest-first matching. Union-find tracks which strokes are already
    # in the same chain so a join can never close a loop.
    parent = list(range(n))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    link: dict[tuple[int, int], tuple[int, int]] = {}
    for _d, i, ei, j, ej in cands:
        if (i, ei) in link or (j, ej) in link:
            continue                       # endpoint already took a nearer partner
        ri, rj = find(i), find(j)
        if ri == rj:
            continue                       # same chain already — would close a loop
        parent[ri] = rj
        link[(i, ei)] = (j, ej)
        link[(j, ej)] = (i, ei)

    if not link:
        return [list(s) for s in strokes]

    # Walk each chain from a free end, flipping strokes so the joined endpoints
    # meet. Loops are impossible (refused above), so every chain has two free ends.
    merged: list[list[tuple[float, float]]] = []
    visited: set[int] = set()
    for s in range(n):
        if s in visited or ((s, 0) in link and (s, 1) in link):
            continue                       # mid-chain stroke; reached from an end
        i, e_in = s, (1 if (s, 0) in link else 0)
        pts: list[tuple[float, float]] = []
        while True:
            visited.add(i)
            pts.extend(strokes[i] if e_in == 0 else reversed(strokes[i]))
            nxt = link.get((i, 1 - e_in))
            if nxt is None or nxt[0] in visited:
                break
            i, e_in = nxt
        merged.append(pts)

    # Defensive: anything not reached (cannot happen while loops are refused).
    merged.extend(list(strokes[i]) for i in range(n) if i not in visited)
    return merged


def _crosses_gap(
    stroke: list[tuple[float, float]],
    a: tuple[float, float],
    b: tuple[float, float],
) -> bool:
    """True when any segment of `stroke` crosses the straight line a→b."""
    # Bounding-box reject first — most strokes are nowhere near the gap.
    lo_x, hi_x = (a[0], b[0]) if a[0] <= b[0] else (b[0], a[0])
    lo_y, hi_y = (a[1], b[1]) if a[1] <= b[1] else (b[1], a[1])
    xs = [p[0] for p in stroke]
    ys = [p[1] for p in stroke]
    if max(xs) < lo_x or min(xs) > hi_x or max(ys) < lo_y or min(ys) > hi_y:
        return False
    for i in range(len(stroke) - 1):
        if _segments_cross(a, b, stroke[i], stroke[i + 1]):
            return True
    return False


def _segments_cross(a, b, c, d) -> bool:
    """
    True when segment a→b and segment c→d properly cross.

    Both orientation products must be strictly negative, which requires all four
    signs to be non-zero: a stroke that merely TOUCHES or ends on the line (a T
    junction, or a groove running collinear with it) does not count as passing
    through it, so it never earns the doubled threshold.
    """
    def orient(p, q, r) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    return (orient(a, b, c) * orient(a, b, d) < 0
            and orient(c, d, a) * orient(c, d, b) < 0)


def _order_strokes(
    strokes: list[list[tuple[float, float]]],
) -> list[list[tuple[float, float]]]:
    """
    Nearest-neighbour TSP: order strokes to minimise total pen-up travel.
    Each stroke can be reversed if its end is closer to the current position.
    """
    if not strokes:
        return []

    remaining = list(range(len(strokes)))
    ordered: list[list[tuple[float, float]]] = []

    first = strokes[remaining.pop(0)]
    ordered.append(first)
    cx, cy = first[-1]

    while remaining:
        best_i     = None
        best_dist  = float("inf")
        best_rev   = False

        for idx in remaining:
            s = strokes[idx]
            d_fwd = _dist2(cx, cy, s[0][0],  s[0][1])
            d_rev = _dist2(cx, cy, s[-1][0], s[-1][1])
            if d_fwd <= d_rev and d_fwd < best_dist:
                best_dist = d_fwd
                best_i    = idx
                best_rev  = False
            elif d_rev < d_fwd and d_rev < best_dist:
                best_dist = d_rev
                best_i    = idx
                best_rev  = True

        chosen = strokes[best_i]
        if best_rev:
            chosen = list(reversed(chosen))
        ordered.append(chosen)
        remaining.remove(best_i)
        cx, cy = chosen[-1]

    return ordered


def _dist2(x1: float, y1: float, x2: float, y2: float) -> float:
    dx = x1 - x2
    dy = y1 - y2
    return dx * dx + dy * dy
