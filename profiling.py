"""
Where the live-view time actually goes — a measuring tool, not a feature.

Both live views are produced on ONE thread: the camera thread polls every
RealSense, stitches the combined canvas, colorizes it, encodes JPEGs, and (every
other cycle) runs groove detection. Anything slow in there delays everything
after it, so "the mask lags" and "the depth view lags" can have the same cause.
This tells you which stage owns the time, and whether the canvas is actually
hitting its intended rate or falling short of it.

Off by default and free when off: `stage()` hands back a shared do-nothing
context manager, so an instrumented loop costs one attribute read per stage.
Switch it on with PROFILE_PIPELINE in config.py.

Reading the report:

    [profile] canvas — 18 cycles in 5.0 s = 3.6 Hz (target 5.0 Hz)
        detect        1420.0 ms total   157.8 ms x 9    28.4% of window
        stitch         890.2 ms total    49.5 ms x 18   17.8% of window
        ...

  * "x N" is how many cycles ran that stage — detection runs every 2nd canvas,
    so half the cycle count is expected, not a bug.
  * The percentages are of WALL time, not of each other. They will not add up to
    100%: the loop also sleeps and polls cameras between canvases. What matters
    is whether the achieved Hz is near the target. Near it → the cadence is the
    limit and the work has headroom. Well under it → the work is the limit, and
    the biggest stage is where to look.

Counters (`count()`) ride along for things that are not durations — the MJPEG
streams use them to report how many frames they sent versus how many were
actually new pictures.

Pure: no config import, no I/O, no threads of its own. The caller decides
whether it is enabled and where the report goes.
"""
from __future__ import annotations

import time
from contextlib import contextmanager


class _NullStage:
    """What `stage()` returns when profiling is off — costs nothing to enter."""

    __slots__ = ()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


_NULL = _NullStage()


class StageTimer:
    """
    Accumulate per-stage wall time (and plain counters) over a window, then hand
    back one formatted report per window.

    Thread-safety: intended for one owning thread. The camera thread writes its
    own timer, the server writes another. `+=` on a list slot is not atomic, so
    don't share one instance across threads — a lost sample would be silent.
    """

    def __init__(self, label: str, enabled: bool = False, every_s: float = 5.0,
                 target_hz: float | None = None, report_rate: bool = True,
                 clock=time.perf_counter) -> None:
        self.label = label
        self.enabled = bool(enabled)
        self.every_s = max(float(every_s), 0.1)
        self.target_hz = target_hz
        # False for a counters-only timer whose cycle() is driven by some
        # unrelated heartbeat — reporting that loop's rate would read like a
        # measurement of the thing being profiled, which it is not.
        self.report_rate = bool(report_rate)
        self._clock = clock
        self._totals: dict[str, list] = {}     # name -> [seconds, calls]
        self._counts: dict[str, float] = {}
        self._cycles = 0
        self._window_start = clock()

    # ── recording ────────────────────────────────────────────────────────────
    def stage(self, name: str):
        """`with timer.stage("stitch"):` — times the block, or does nothing."""
        if not self.enabled:
            return _NULL
        return self._timed(name)

    @contextmanager
    def _timed(self, name: str):
        t0 = self._clock()
        try:
            yield
        finally:
            elapsed = self._clock() - t0
            rec = self._totals.get(name)
            if rec is None:
                self._totals[name] = [elapsed, 1]
            else:
                rec[0] += elapsed
                rec[1] += 1

    def count(self, name: str, n: float = 1.0) -> None:
        """Record something that is not a duration (frames sent, bytes, drops)."""
        if not self.enabled:
            return
        self._counts[name] = self._counts.get(name, 0.0) + n

    def cycle(self) -> str | None:
        """
        Mark one completed cycle. Returns the report string when the window has
        elapsed (and starts a new window), else None — so the caller can simply
        `if (line := timer.cycle()): print(line)` with no clock of its own.
        """
        if not self.enabled:
            return None
        self._cycles += 1
        if (self._clock() - self._window_start) < self.every_s:
            return None
        return self.report(reset=True)

    # ── reporting ────────────────────────────────────────────────────────────
    def report(self, reset: bool = False) -> str:
        window = max(self._clock() - self._window_start, 1e-9)
        hz = self._cycles / window
        if self.report_rate:
            head = (f"[profile] {self.label} — {self._cycles} cycles in "
                    f"{window:.1f} s = {hz:.1f} Hz")
            if self.target_hz:
                head += f" (target {self.target_hz:.1f} Hz)"
                if hz < self.target_hz * 0.8:
                    head += "  ← BELOW TARGET: the work is the limit, not the cadence"
        else:
            head = f"[profile] {self.label} — over {window:.1f} s"
        lines = [head]

        for name, (secs, calls) in sorted(
                self._totals.items(), key=lambda kv: kv[1][0], reverse=True):
            per = (secs / calls * 1000.0) if calls else 0.0
            lines.append(f"    {name:<16}{secs * 1000.0:9.1f} ms total"
                         f"{per:8.1f} ms x {calls:<5d}"
                         f"{secs / window * 100.0:6.1f}% of window")
        for name, total in sorted(self._counts.items()):
            lines.append(f"    {name:<24}{total:12,.0f} total"
                         f"{total / window:12,.1f} /s")
        if reset:
            self.reset()
        return "\n".join(lines)

    def reset(self) -> None:
        self._totals.clear()
        self._counts.clear()
        self._cycles = 0
        self._window_start = self._clock()
