"""
Pure math for Multi-Cam Vision (no hardware, no server).

N RealSense cameras (1 … STITCH_MAX_CAMERAS) look down at the sand from FIXED
mounts. This module lays their frames side by side on one canvas — nothing
more. Per camera: rotate the image to match how it hangs, crop away what is
not sand, then pin the four corners of what is left onto the canvas. The four
corners ARE the placement: sliding all of them moves the camera, turning them
rotates it, pulling one skews it — which is exactly the keystone a camera
mounted at an angle produces. `cv2.warpPerspective` then drops each camera's
depth (and colour) onto the shared grid, and overlapping pixels are averaged.

Same corner-pin convention as the projector calibration in
viewer/projection.html, so the two feel identical: corner order is
**TL, TR, BL, BR** — handles 1, 2, 3, 4.

There is deliberately NO overlap search and no groove detection here. The
cameras are bolted down, so the layout is dialled in once by hand (or loaded
from stitch_calibration.json) and then stays put; detection lives in the main
app, which is the only place its parameters are tuned.

Canvas conventions: millimetres, x right and y down, so v grows down exactly
like the single-camera pipeline. Heightmap values are metres of depth below
the cameras — valleys are larger values, same as raw depth — which keeps the
result a drop-in input for `depth_extractor.grooves_and_mask`.

RGB: the colour stream is aligned to depth per camera, so each depth pixel
carries a colour sample where the (narrower-FOV) RGB lens covers it. Those
ride the same warp; the strips the RGB lenses miss stay black (expected — the
depth image is the product, RGB is reference only).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Optional

import cv2
import numpy as np

from config import (
    DEPTH_WIDTH, DEPTH_HEIGHT,
    STITCH_MM_PER_PX, STITCH_MAX_GRID_W, STITCH_MAX_GRID_H,
    STITCH_NOMINAL_HFOV_DEG, STITCH_NOMINAL_VFOV_DEG,
)

# Corner order shared with the projector calibration: handles 1-4 on screen.
CORNER_NAMES = ("top-left", "top-right", "bottom-left", "bottom-right")
# Index of each corner when walked around the perimeter (for area/drawing).
_PERIMETER = (0, 1, 3, 2)


# ── data types ────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Intrinsics:
    """
    Pinhole intrinsics of one depth stream (pixels). Only used to guess a
    sensible STARTING size for a camera's canvas footprint — the placement
    itself is whatever the operator drags it to.
    """
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = DEPTH_WIDTH
    height: int = DEPTH_HEIGHT

    @classmethod
    def nominal(cls, width: int = DEPTH_WIDTH, height: int = DEPTH_HEIGHT) -> "Intrinsics":
        """D435 depth intrinsics estimated from the datasheet FOV (synthetic mode)."""
        fx = (width / 2) / math.tan(math.radians(STITCH_NOMINAL_HFOV_DEG) / 2)
        fy = (height / 2) / math.tan(math.radians(STITCH_NOMINAL_VFOV_DEG) / 2)
        return cls(fx=fx, fy=fy, cx=width / 2, cy=height / 2, width=width, height=height)


def _norm_rot(value) -> int:
    """Snap any angle to the four mounting orientations (clockwise degrees)."""
    try:
        k = int(round(float(value) / 90.0)) % 4
    except (TypeError, ValueError):
        k = 0
    return k * 90


def _norm_crop(value) -> tuple[float, float, float, float]:
    """Clamp a normalized crop rect into the frame, never zero-sized."""
    try:
        x, y, w, h = (float(v) for v in value)
    except (TypeError, ValueError):
        return (0.0, 0.0, 1.0, 1.0)
    x = min(max(x, 0.0), 0.95)
    y = min(max(y, 0.0), 0.95)
    w = min(max(w, 0.05), 1.0 - x)
    h = min(max(h, 0.05), 1.0 - y)
    return (x, y, w, h)


def _norm_quad(value) -> tuple[tuple[float, float], ...]:
    """Four canvas corners in mm (TL, TR, BL, BR); () when never placed."""
    if not value:
        return ()
    try:
        pts = [(float(p[0]), float(p[1])) for p in value]
    except (TypeError, ValueError, IndexError):
        return ()
    return tuple(pts) if len(pts) == 4 else ()


def quad_area_mm2(quad) -> float:
    """Shoelace area walked around the perimeter; ≤ 0 means the corners crossed."""
    q = np.asarray(quad, np.float64)[list(_PERIMETER)]
    x, y = q[:, 0], q[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


@dataclass
class CameraPlacement:
    """
    Where ONE camera's frame lands on the canvas.

    `quad_mm` is the whole placement — the four canvas corners the cropped,
    rotated frame is pinned to. There are no separate move/rotate/skew numbers
    on purpose: every one of those is just a different way of moving corners,
    and one representation means the UI can be four drag handles.
    """
    serial: str = ""
    rot_deg: int = 0                                    # 0/90/180/270, clockwise
    crop: tuple[float, float, float, float] = (0.0, 0.0, 1.0, 1.0)   # x,y,w,h normalized
    quad_mm: tuple[tuple[float, float], ...] = ()       # TL,TR,BL,BR; () = not placed yet
    height_mm: float = 0.0                              # shifts this camera's depths
    enabled: bool = True

    @classmethod
    def from_dict(cls, d: dict | None) -> "CameraPlacement":
        d = d or {}
        p = cls(serial=str(d.get("serial", "")))
        p.rot_deg = _norm_rot(d.get("rot_deg", 0))
        p.crop = _norm_crop(d.get("crop"))
        p.quad_mm = _norm_quad(d.get("quad_mm"))
        if d.get("height_mm") is not None:
            try:
                p.height_mm = float(d["height_mm"])
            except (TypeError, ValueError):
                pass
        p.enabled = bool(d.get("enabled", True))
        return p

    def to_dict(self) -> dict:
        return {"serial": self.serial, "rot_deg": self.rot_deg,
                "crop": list(self.crop),
                "quad_mm": [list(c) for c in self.quad_mm],
                "height_mm": self.height_mm, "enabled": self.enabled}

    def merged(self, changes: dict) -> "CameraPlacement":
        """A NEW placement with `changes` applied — the UI edits one thing at a time."""
        return CameraPlacement.from_dict({**self.to_dict(), **(changes or {})})


@dataclass
class StitchCalib:
    """The whole rig: one placement per camera plus the canvas resolution."""
    cams: list[CameraPlacement] = field(default_factory=list)
    mm_per_px: float = STITCH_MM_PER_PX     # 0 = auto from the first camera

    @classmethod
    def from_dict(cls, d: dict | None) -> "StitchCalib":
        d = d or {}
        cams = [CameraPlacement.from_dict(c) for c in (d.get("cams") or [])]
        try:
            mm = float(d.get("mm_per_px", STITCH_MM_PER_PX))
        except (TypeError, ValueError):
            mm = STITCH_MM_PER_PX
        return cls(cams=cams, mm_per_px=max(0.0, mm))

    def to_dict(self) -> dict:
        return {"cams": [c.to_dict() for c in self.cams],
                "mm_per_px": self.mm_per_px}

    def placement_for(self, serial: str, index: int) -> CameraPlacement:
        """Serial wins when there is one; otherwise fall back to position."""
        if serial:
            for p in self.cams:
                if p.serial == serial:
                    return p
        if not serial and 0 <= index < len(self.cams):
            return self.cams[index]
        return CameraPlacement(serial=serial)

    def with_camera(self, index: int, placement: CameraPlacement) -> "StitchCalib":
        """A NEW calib with one camera replaced (worker threads may hold the old one)."""
        if not 0 <= index < len(self.cams):
            return self
        cams = list(self.cams)
        cams[index] = placement
        return StitchCalib(cams=cams, mm_per_px=self.mm_per_px)


@dataclass
class CameraFrame:
    """One camera's averaged capture: metric depth + valid mask (+ aligned RGB)."""
    depth_m: np.ndarray                 # float32 HxW, metres
    valid: np.ndarray                   # bool HxW
    intr: Intrinsics
    rgb: Optional[np.ndarray] = None    # uint8 HxWx3 BGR aligned to depth, or None
    serial: str = ""


@dataclass
class StitchResult:
    depth_m: np.ndarray                 # float32 HxW — depth below the cameras (m)
    valid: np.ndarray                   # bool HxW
    rgb: np.ndarray                     # uint8 HxWx3 BGR (black where no colour)
    rgb_valid: np.ndarray               # bool HxW
    coverage: np.ndarray                # uint8 HxW — how many cameras hit each pixel
    overlap: np.ndarray                 # bool HxW — two or more cameras contributed
    mm_per_px: float
    origin_mm: tuple[float, float]      # canvas mm at grid pixel (0,0)
    # Each camera's placed quad in GRID PIXELS (TL,TR,BL,BR), None when the
    # camera is disabled or had nothing to show. These are the drag handles.
    quads_px: list[Optional[list[tuple[float, float]]]] = field(default_factory=list)


# ── per-camera preparation ────────────────────────────────────────────────────

def _rot90_intr(i: Intrinsics) -> Intrinsics:
    """Intrinsics after rotating the image 90° CLOCKWISE (w/h and fx/fy swap)."""
    return Intrinsics(fx=i.fy, fy=i.fx,
                      cx=(i.height - 1) - i.cy, cy=i.cx,
                      width=i.height, height=i.width)


def rotate_frame(frame: CameraFrame, deg: int) -> CameraFrame:
    """
    Undo the mounting orientation: a camera bolted on its side or upside-down
    delivers the scene rotated about its optical axis. `deg` is clockwise and
    snaps to 0/90/180/270.
    """
    k = _norm_rot(deg) // 90
    if k == 0:
        return frame

    def rot(a):
        return None if a is None else np.rot90(a, -k).copy()

    intr = frame.intr
    for _ in range(k):
        intr = _rot90_intr(intr)
    return CameraFrame(rot(frame.depth_m), rot(frame.valid), intr,
                       rot(frame.rgb), frame.serial)


def crop_mask(valid: np.ndarray, crop) -> np.ndarray:
    """
    Keep only the crop rectangle. Done by clearing the valid mask rather than
    slicing, so the pixel coordinates the corner-pin is built on stay exact.
    """
    x, y, w, h = _norm_crop(crop)
    ih, iw = valid.shape
    u0 = max(0, min(iw - 1, int(round(x * iw))))
    u1 = max(u0 + 1, min(iw, int(round((x + w) * iw))))
    v0 = max(0, min(ih - 1, int(round(y * ih))))
    v1 = max(v0 + 1, min(ih, int(round((y + h) * ih))))
    out = np.zeros_like(valid)
    out[v0:v1, u0:u1] = valid[v0:v1, u0:u1]
    return out


def crop_corners_px(shape, crop) -> np.ndarray:
    """The crop rectangle's corners in image pixels, TL/TR/BL/BR."""
    ih, iw = shape[:2]
    x, y, w, h = _norm_crop(crop)
    u0, u1 = x * iw, (x + w) * iw
    v0, v1 = y * ih, (y + h) * ih
    return np.array([[u0, v0], [u1, v0], [u0, v1], [u1, v1]], np.float32)


def median_depth_m(depth_m: np.ndarray, valid: np.ndarray) -> Optional[float]:
    z = depth_m[valid]
    return float(np.median(z)) if z.size else None


def footprint_mm(frame: CameraFrame) -> tuple[float, float]:
    """How wide and tall a patch of sand one full frame covers, at median depth."""
    med = median_depth_m(frame.depth_m, frame.valid) or 0.8
    return (med * 1000.0 * frame.intr.width / frame.intr.fx,
            med * 1000.0 * frame.intr.height / frame.intr.fy)


def common_footprint_mm(footprints) -> tuple[float, float]:
    """
    ONE patch size for the whole rig — the median of what the cameras report.
    The cameras hang at nominally the same height, so their footprints should
    agree; the spread that is left is median-depth noise, and letting each
    camera set its own scale is what used to open the rig as a row of
    mismatched tiles. The median keeps one odd reading from sizing everybody.
    """
    fs = [f for f in footprints if f and f[0] > 0 and f[1] > 0]
    if not fs:
        return (800.0, 600.0)
    return (float(np.median([f[0] for f in fs])),
            float(np.median([f[1] for f in fs])))


def default_quad_mm(footprint: tuple[float, float], crop, index: int
                    ) -> tuple[tuple[float, float], ...]:
    """
    Fallback placement for ONE camera in isolation (a folded quad being parked
    somewhere visible). `default_row_mm` is what lays out a rig; this only has
    to put a readable rectangle on the canvas.
    """
    fw, fh = footprint
    _, _, cw, ch = _norm_crop(crop)
    w, h = fw * cw, fh * ch
    x0 = index * fw
    return ((x0, 0.0), (x0 + w, 0.0), (x0, h), (x0 + w, h))


def default_row_mm(footprints, crops) -> list[tuple[tuple[float, float], ...]]:
    """
    The opening layout: every camera at ONE common scale, laid flush left to
    right in enumeration order, tops aligned.

    Each tile is the common footprint scaled by that camera's own crop, and the
    next tile starts at the previous one's right edge — so trimming a camera
    narrows its tile and the row closes up behind it instead of leaving the
    hole a per-camera offset used to leave. Nothing here claims to match the
    real rig; it is a clean strip for the operator to reorder and nudge.
    """
    n = max(len(footprints), len(crops))
    if n <= 0:
        return []
    tw, th = common_footprint_mm(footprints)
    out: list[tuple[tuple[float, float], ...]] = []
    x = 0.0
    for i in range(n):
        _, _, cw, ch = _norm_crop(crops[i] if i < len(crops) else None)
        w, h = tw * cw, th * ch
        out.append(((x, 0.0), (x + w, 0.0), (x, h), (x + w, h)))
        x += w
    return out


def quad_centre_x(quad) -> float:
    """Where a placed quad sits along the row — what left/right order means."""
    return sum(p[0] for p in quad) / 4.0 if len(quad) == 4 else 0.0


def swap_quads_x(qa, qb) -> tuple[tuple[tuple[float, float], ...],
                                  tuple[tuple[float, float], ...]]:
    """
    Trade two cameras' places in the row without touching their shapes: each
    quad slides in x until the two centres have swapped. Reordering is how the
    operator makes the canvas match which camera is physically on the left, so
    the keystone they already dialled into each one has to survive it — which
    rules out simply exchanging the two quads.
    """
    if len(qa) != 4 or len(qb) != 4:
        return tuple(tuple(p) for p in qa), tuple(tuple(p) for p in qb)
    d = quad_centre_x(qb) - quad_centre_x(qa)
    return (tuple((float(p[0] + d), float(p[1])) for p in qa),
            tuple((float(p[0] - d), float(p[1])) for p in qb))


def rotate_quad(quad, steps: int = 1) -> tuple[tuple[float, float], ...]:
    """
    Turn a placed quad a quarter turn (positive = clockwise) about its own
    centre, re-labelling the corners so the picture inside visibly rotates
    while keeping its proportions. Pairs with `rot_deg`: rotate the image and
    its quad together, or the frame ends up stretched into the old shape.
    """
    q = [list(p) for p in quad]
    if len(q) != 4:
        return tuple(tuple(p) for p in q)
    for _ in range(steps % 4):
        cx = sum(p[0] for p in q) / 4.0
        cy = sum(p[1] for p in q) / 4.0
        # Clockwise on screen, where y grows down: (dx, dy) → (-dy, dx).
        spun = [[cx - (p[1] - cy), cy + (p[0] - cx)] for p in q]
        # After a clockwise image turn the new TL is the old BL, and so on.
        q = [spun[2], spun[0], spun[3], spun[1]]
    return tuple((float(p[0]), float(p[1])) for p in q)


def requad_for_crop(placement: CameraPlacement, new_crop, shape
                    ) -> tuple[tuple[float, float], ...]:
    """
    Re-cut the placed quad for a new crop so the sand does not move: the new
    crop rectangle is pushed through the corner-pin the OLD crop defined. Without
    this, cropping would stretch what is left over the same canvas area and slide
    the camera out of alignment.
    """
    if not placement.quad_mm:
        return ()
    src = crop_corners_px(shape, placement.crop)
    dst = np.asarray(placement.quad_mm, np.float32)
    h = cv2.getPerspectiveTransform(src, dst)
    pts = crop_corners_px(shape, new_crop).reshape(-1, 1, 2)
    out = cv2.perspectiveTransform(pts, h).reshape(-1, 2)
    return tuple((float(x), float(y)) for x, y in out)


@dataclass
class _Placed:
    """One camera, ready to warp onto the canvas."""
    frame: CameraFrame          # rotated
    valid: np.ndarray           # rotated AND cropped
    src: np.ndarray             # crop corners in image px (TL,TR,BL,BR)
    quad: np.ndarray            # canvas corners in mm (TL,TR,BL,BR)
    height_m: float


def _prepare(frame: CameraFrame, p: CameraPlacement, index: int) -> Optional[_Placed]:
    if not p.enabled:
        return None
    g = rotate_frame(frame, p.rot_deg)
    valid = crop_mask(g.valid, p.crop)
    if not valid.any():
        return None
    quad = p.quad_mm or default_quad_mm(footprint_mm(g), p.crop, index)
    # A corner dragged past its neighbours folds the quad over, and the warp
    # would smear that camera across the canvas. Park it back on its default
    # rather than leaving the operator a blank view to fix.
    if quad_area_mm2(quad) <= 1.0:
        quad = default_quad_mm(footprint_mm(g), p.crop, index)
    return _Placed(g, valid, crop_corners_px(g.depth_m.shape, p.crop),
                   np.asarray(quad, np.float32), p.height_mm / 1000.0)


def bind_placements(calib: StitchCalib, frames: list[CameraFrame]) -> StitchCalib:
    """
    Match saved placements to the cameras actually plugged in: by serial first,
    then positionally for placements saved without one (or from a rig whose
    cameras were swapped out), then defaults for anything left over. Returns a
    NEW calib whose `cams` is index-aligned with `frames`.
    """
    taken: set[int] = set()
    out: list[Optional[CameraPlacement]] = [None] * len(frames)
    for k, f in enumerate(frames):
        for j, p in enumerate(calib.cams):
            if j not in taken and p.serial and p.serial == f.serial:
                out[k] = replace(p, serial=f.serial)
                taken.add(j)
                break
    spare = [j for j in range(len(calib.cams)) if j not in taken]
    for k, f in enumerate(frames):
        if out[k] is None and spare:
            out[k] = replace(calib.cams[spare.pop(0)], serial=f.serial)
    for k, f in enumerate(frames):
        if out[k] is None:
            out[k] = CameraPlacement(serial=f.serial)
    return StitchCalib(cams=[p for p in out if p is not None],
                       mm_per_px=calib.mm_per_px)


# ── the canvas ────────────────────────────────────────────────────────────────

def _auto_mm_per_px(placed: list[_Placed]) -> float:
    """Resolution that keeps the first camera at roughly its native detail."""
    p = placed[0]
    w_mm = float(np.hypot(*(p.quad[1] - p.quad[0])))
    w_px = float(np.hypot(*(p.src[1] - p.src[0])))
    return max(0.1, w_mm / max(1.0, w_px))


def fill_small_holes(height: np.ndarray, valid: np.ndarray, iters: int = 2
                     ) -> tuple[np.ndarray, np.ndarray]:
    """
    Fill 1-2 px speckle holes with the mean of valid 3×3 neighbours. Large gaps
    (outside every footprint, or a real dropout) stay invalid.
    """
    h = height.astype(np.float32).copy()
    v = valid.copy()
    for _ in range(iters):
        vf = cv2.blur(v.astype(np.float32), (3, 3))
        hf = cv2.blur(np.where(v, h, 0.0).astype(np.float32), (3, 3))
        fill = (vf > 0) & ~v
        if not fill.any():
            break
        h[fill] = hf[fill] / vf[fill]
        v = v | fill
    return h, v


def _empty_result(n_cams: int, mm_per_px: float) -> StitchResult:
    empty = np.zeros((2, 2), np.float32)
    return StitchResult(empty, empty.astype(bool), np.zeros((2, 2, 3), np.uint8),
                        empty.astype(bool), empty.astype(np.uint8),
                        empty.astype(bool), max(mm_per_px, 1.0), (0.0, 0.0),
                        [None] * n_cams)


def stitch(frames: list[CameraFrame], calib: StitchCalib,
           mm_per_px: Optional[float] = None) -> StitchResult:
    """
    Lay every camera's frame onto the shared canvas through its corner-pin.
    `frames` are the raw per-device captures in enumeration order; the rotation,
    crop and placement in `calib` are applied here.
    """
    if mm_per_px is None:
        mm_per_px = calib.mm_per_px

    prepared = [_prepare(f, calib.placement_for(f.serial, i), i)
                for i, f in enumerate(frames)]
    live = [q for q in prepared if q is not None]
    if not live:
        return _empty_result(len(frames), mm_per_px if mm_per_px > 0 else 2.0)

    if mm_per_px <= 0:
        mm_per_px = _auto_mm_per_px(live)

    corners = np.concatenate([q.quad for q in live])
    xmin, ymin = float(corners[:, 0].min()), float(corners[:, 1].min())
    xmax, ymax = float(corners[:, 0].max()), float(corners[:, 1].max())
    # Quantize the origin so the canvas does not shuffle by a pixel every frame.
    step = mm_per_px * 8
    xmin = math.floor(xmin / step) * step
    ymin = math.floor(ymin / step) * step
    gw = int(math.ceil((xmax - xmin) / mm_per_px))
    gh = int(math.ceil((ymax - ymin) / mm_per_px))
    # Cap the grid by coarsening resolution, never by cropping coverage.
    scale = max(gw / STITCH_MAX_GRID_W, gh / STITCH_MAX_GRID_H, 1.0)
    if scale > 1.0:
        mm_per_px *= scale
        gw = max(1, int(math.ceil((xmax - xmin) / mm_per_px)))
        gh = max(1, int(math.ceil((ymax - ymin) / mm_per_px)))
    gw, gh = max(gw, 16), max(gh, 16)

    depth_sum = np.zeros((gh, gw), np.float32)
    coverage = np.zeros((gh, gw), np.uint8)
    rgb_sum = np.zeros((gh, gw, 3), np.float32)
    rgb_cnt = np.zeros((gh, gw), np.float32)
    quads_px: list[Optional[list[tuple[float, float]]]] = []

    for q in prepared:
        if q is None:
            quads_px.append(None)
            continue
        dst = (q.quad - np.array([xmin, ymin], np.float32)) / mm_per_px
        quads_px.append([(float(u), float(v)) for u, v in dst])
        h = cv2.getPerspectiveTransform(q.src, dst.astype(np.float32))
        # NEAREST everywhere: interpolating depth across a dropout would invent
        # surface that is not there, and the mask must stay strictly binary.
        src_depth = np.where(q.valid, q.frame.depth_m, 0.0).astype(np.float32)
        warped = cv2.warpPerspective(src_depth, h, (gw, gh), flags=cv2.INTER_NEAREST)
        hit = cv2.warpPerspective(q.valid.astype(np.uint8), h, (gw, gh),
                                  flags=cv2.INTER_NEAREST) > 0
        depth_sum[hit] += warped[hit] + q.height_m
        coverage += hit.astype(np.uint8)
        if q.frame.rgb is not None:
            colour = np.where(q.valid[..., None], q.frame.rgb, 0)
            cw = cv2.warpPerspective(colour, h, (gw, gh), flags=cv2.INTER_NEAREST)
            has = hit & (cw.sum(axis=2) > 0)   # aligned-RGB black = outside colour FOV
            rgb_sum[has] += cw[has]
            rgb_cnt += has.astype(np.float32)

    valid = coverage > 0
    depth = np.zeros((gh, gw), np.float32)
    depth[valid] = depth_sum[valid] / coverage[valid]
    overlap = coverage >= 2
    depth, valid = fill_small_holes(depth, valid)

    rgb = np.zeros((gh, gw, 3), np.uint8)
    rgb_valid = rgb_cnt > 0
    if rgb_valid.any():
        rgb[rgb_valid] = np.clip(
            rgb_sum[rgb_valid] / rgb_cnt[rgb_valid, None], 0, 255).astype(np.uint8)

    return StitchResult(depth, valid, rgb, rgb_valid, coverage, overlap,
                        mm_per_px, (xmin, ymin), quads_px)


# ── synthetic scene (no-hardware fallback + tests) ────────────────────────────

def _world_relief_mm(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """
    Groove pattern carved into the synthetic sand, in mm below the flat surface.
    Spans a wide footprint so strokes cross every seam. x, y in metres.
    """
    relief = np.zeros_like(x)
    # A long wavy groove running left-right across all footprints.
    cy = 0.05 * np.sin(x * 7.0)
    relief += 2.5 * np.exp(-((y - cy) ** 2) / (2 * 0.006 ** 2))
    # A diagonal straight groove.
    d = (y - 0.12) - 0.35 * (x - 0.2)
    relief += 2.0 * np.exp(-(d ** 2) / (2 * 0.005 ** 2))
    return relief


def synthetic_frame(intr: Intrinsics, tx_mm: float = 0.0, ty_mm: float = 0.0,
                    plane_z_m: float = 0.8, noise_mm: float = 0.35,
                    rng: np.random.Generator | None = None,
                    serial: str = "") -> CameraFrame:
    """
    Render what one camera sees of the synthetic sand plane, given where it
    hangs over the canvas (tx/ty in mm). The surface lies plane_z_m below.
    """
    rng = rng or np.random.default_rng(0)
    h, w = intr.height, intr.width
    us, vs = np.meshgrid(np.arange(w, dtype=np.float64),
                         np.arange(h, dtype=np.float64))
    xw = (us - intr.cx) / intr.fx * plane_z_m + tx_mm / 1000.0
    yw = (vs - intr.cy) / intr.fy * plane_z_m + ty_mm / 1000.0
    depth = plane_z_m + _world_relief_mm(xw, yw) / 1000.0
    depth += rng.normal(0.0, noise_mm / 1000.0, size=depth.shape)
    valid = rng.random(depth.shape) > 0.01         # ~1% dropout speckle
    depth = depth.astype(np.float32)
    depth[~valid] = 0.0

    # Fake aligned RGB: sand colour, black band at the frame edges imitating
    # the narrower colour FOV (so the stitched RGB shows the expected gaps).
    rgb = np.full((h, w, 3), (96, 130, 168), np.uint8)   # BGR sand tone
    shade = np.clip(1.0 - 0.12 * _world_relief_mm(xw, yw), 0.0, 1.0)
    rgb = (rgb.astype(np.float32) * shade[..., None]).astype(np.uint8)
    edge = int(w * 0.10)
    rgb[:, :edge] = 0
    rgb[:, -edge:] = 0
    return CameraFrame(depth_m=depth, valid=valid, intr=intr, rgb=rgb, serial=serial)


def synthetic_scene(n: int = 2, overlap_frac: float = 0.08,
                    plane_z_m: float = 0.8, seed: int = 0
                    ) -> tuple[list[CameraFrame], StitchCalib]:
    """
    `n` synthetic cameras in a row with the requested footprint overlap.
    Returns (frames, true_calib) — feed true_calib to stitch() for a perfect
    merge, or move a quad to exercise the placement controls.
    """
    intr = Intrinsics.nominal()
    rng = np.random.default_rng(seed)
    fw = plane_z_m * intr.width / intr.fx * 1000.0     # footprint, mm
    fh = plane_z_m * intr.height / intr.fy * 1000.0
    baseline = fw * (1.0 - overlap_frac)
    # Where each camera's full frame lands on the canvas, exactly as rendered.
    x0 = -intr.cx / intr.fx * plane_z_m * 1000.0
    y0 = -intr.cy / intr.fy * plane_z_m * 1000.0
    frames, cams = [], []
    for i in range(max(1, n)):
        serial = f"SYN-{i + 1}"
        tx = i * baseline
        frames.append(synthetic_frame(intr, tx_mm=tx, plane_z_m=plane_z_m,
                                      rng=rng, serial=serial))
        left, top = x0 + tx, y0
        cams.append(CameraPlacement(serial=serial, quad_mm=(
            (left, top), (left + fw, top), (left, top + fh), (left + fw, top + fh))))
    return frames, StitchCalib(cams=cams)
