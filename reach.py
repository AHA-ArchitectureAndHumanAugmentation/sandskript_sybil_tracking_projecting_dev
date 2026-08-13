"""
reach.py — reach-envelope estimate for generated toolpaths.

Lives in its own module (not main.py) so external tools can import it without
side effects: importing main starts the camera thread and pollers.

The check is an ENVELOPE ONLY: a point is "reachable" if it lies inside a
UR_REACH_M sphere around the robot base and outside a thin UR_MIN_REACH_M
cylinder around the base axis. Joint limits, wrist configuration and collisions
are NOT modelled — red flags are certain failures, but all-green near the
boundary can still fault on the real arm.
"""
from __future__ import annotations

import math

from config import UR_REACH_M, UR_MIN_REACH_M


def reach_flags(strokes: list[list[list[float]]]) -> tuple[list[list[int]], int, int]:
    """Per-stroke 0/1 flags (1 = outside the estimated envelope), n_out, n_total."""
    flags: list[list[int]] = []
    n_out = n_total = 0
    for stroke in strokes:
        f = []
        for p in stroke:
            r = math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])
            r_xy = math.hypot(p[0], p[1])
            bad = int(r > UR_REACH_M or r_xy < UR_MIN_REACH_M)
            f.append(bad)
            n_out += bad
            n_total += 1
        flags.append(f)
    return flags, n_out, n_total
