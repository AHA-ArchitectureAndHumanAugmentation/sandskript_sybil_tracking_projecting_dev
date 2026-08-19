"""
Whole-canvas view rotation — the ⟳ button on Developer Mode's Depth viewport.

The rig is bolted down and the sand box is not always square to it, so the
combined canvas can come out sideways. This turns the WHOLE canvas in quarter
turns, and it does so at ONE place: where the canvas leaves the camera thread.
Everything downstream — the RGB / skeleton / mask views, the projector's
full-frame mask, the depth-number labels, the Participant popup's cropped
stream, the captured still and therefore the generated path — is derived from
that canvas, so all of it turns together and none of it needs to know why.
Turning each view separately is what this module exists to avoid: the crop, the
reference frame and the strokes would then be in four different orientations.

Quarter turns only, and `np.rot90`: the picture is re-indexed, never resampled,
so a rotated depth map holds exactly the same millimetres. Angles are CLOCKWISE
degrees, matching what the button does to what the operator sees.

Note this is NOT the per-camera `rot_deg` in Multi-Cam Vision. That one says how
a single camera is mounted and is baked into the layout before stitching; this
one turns the finished canvas, live, without touching the layout file.

Pure: no state, no I/O, safe to import from any thread.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

# The only rotations that re-index without resampling.
VALID_DEG = (0, 90, 180, 270)


def norm_deg(deg) -> int:
    """Any angle → one of 0/90/180/270. Junk (None, text, 37°) → 0."""
    try:
        turns = int(round(float(deg) / 90.0))
    except (TypeError, ValueError):
        return 0
    return (turns % 4) * 90


def turns(deg) -> int:
    """Clockwise quarter turns, 0-3."""
    return norm_deg(deg) // 90


def rotate_image(arr: Optional[np.ndarray], deg) -> Optional[np.ndarray]:
    """
    Rotate an image/array clockwise by `deg`. Works for any dtype and for both
    2-D (depth, valid, mask) and 3-D (BGR) arrays — only the first two axes move.

    The result is made contiguous: `np.rot90` returns a view with negative
    strides, and cv2 (colorize, imencode, warp) rejects or silently copies those.
    """
    if arr is None:
        return None
    k = turns(deg)
    if k == 0:
        return arr
    return np.ascontiguousarray(np.rot90(arr, -k))    # -k = clockwise


def rotate_size(size, deg) -> Optional[tuple[int, int]]:
    """(width, height) after the turn — swapped on a quarter turn."""
    if size is None:
        return None
    w, h = int(size[0]), int(size[1])
    return (h, w) if turns(deg) % 2 else (w, h)


def rotate_crop(crop: Optional[dict], deg) -> Optional[dict]:
    """
    Carry a normalized crop {x, y, w, h} through the same turn, so the region the
    operator framed keeps covering the same sand instead of jumping to another
    corner. None stays None (no crop set).

    One clockwise quarter turn sends a point (x, y) to (1 - y, x) — the old top-
    left corner ends up top-RIGHT, which is what turning a picture clockwise
    does — so the rectangle [x, x+w] x [y, y+h] becomes
    [1-y-h, 1-y] x [x, x+w].
    """
    if not crop:
        return crop
    try:
        x = float(crop.get("x", 0.0))
        y = float(crop.get("y", 0.0))
        w = float(crop.get("w", 1.0))
        h = float(crop.get("h", 1.0))
    except (AttributeError, TypeError, ValueError):
        return crop
    for _ in range(turns(deg)):
        x, y, w, h = 1.0 - y - h, x, h, w
    return {"x": round(x, 6), "y": round(y, 6),
            "w": round(w, 6), "h": round(h, 6)}
