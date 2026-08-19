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
Switch it on with PROFILE_PIPELINE in config.py (or SANDSKRIPT_PROFILE=1 in the
environment, which leaves the file alone).

Reading the report:

    [profile] canvas | 2 cam | 1216x480 px | 2.11 mm/px
              - 47 cycles in 5.0 s = 9.4 Hz (target 10.0 Hz)
        work 62.1 ms/cycle of a 100.0 ms budget = 62% busy
        stage             total ms   ms/cyc  ms/call      max  calls
        stitch              1620.0    34.47    34.47    41.20     47
          warp_depth         880.1    18.73    18.73    22.10     47
          warp_rgb           520.0    11.06    11.06    14.90     47
          blend              180.0     3.83     3.83     5.10     47
          prepare             40.0     0.85     0.85     1.30     47
        detect               610.0    12.98    12.98    15.40     47
        ...

  * The header carries the RIG — camera count, canvas size, mm/px — because
    those are what a scaling test changes, and a pasted report has to say which
    run it came from.
  * **ms/cyc is the column to read.** It is total ÷ CANVASES, so a stage that
    runs every second canvas shows what it truly costs per canvas; ms/call
    shows what it costs when it does run. `calls` below the cycle count is
    throttling, not a fault.
  * Indented rows are a BREAKDOWN of the stage above them (recorded as
    "parent.child"). They are inside the parent's time, so they are excluded
    from the `work` line — no double counting.
  * `work` is the sum of the top-level stages per canvas against the budget the
    target rate allows. It is the headroom number: near 100% busy means the work
    is the limit; well under means the cadence is, and shaving a stage will not
    make the views any fresher.
  * `max` is the worst single call in the window. A stage averaging 5 ms with a
    40 ms max is a stutter, which reads on screen quite differently from a stage
    that is evenly slow.

Counters (`count()`) ride along for things that are not durations — the MJPEG
streams use them to report how many frames they sent versus how many were
actually new pictures.

Pure: no config import, no I/O, no threads of its own. The caller decides
whether it is enabled and where the report goes. Pure modules that want to
report their internals take an optional timer and default it to NULL_TIMER, so
they stay independent of whether anything is being measured.
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
                 context: str = "", clock=time.perf_counter) -> None:
        self.label = label
        self.enabled = bool(enabled)
        self.every_s = max(float(every_s), 0.1)
        self.target_hz = target_hz
        # False for a counters-only timer whose cycle() is driven by some
        # unrelated heartbeat — reporting that loop's rate would read like a
        # measurement of the thing being profiled, which it is not.
        self.report_rate = bool(report_rate)
        # What was being measured (rig size, canvas, resolution). Printed in the
        # header so a report pasted out of a terminal still says which run it is.
        self.context = str(context)
        self._clock = clock
        self._totals: dict[str, list] = {}     # name -> [seconds, calls, max_s]
        self._counts: dict[str, float] = {}
        self._cycles = 0
        self._window_start = clock()

    def set_context(self, context: str) -> None:
        """Name the thing being measured — usually known only once it has started."""
        self.context = str(context)

    # ── recording ────────────────────────────────────────────────────────────
    def stage(self, name: str):
        """
        `with timer.stage("stitch"):` — times the block, or does nothing.

        A dotted name ("stitch.warp_depth") records a BREAKDOWN of its parent:
        reported indented under it and left out of the per-cycle work total,
        since its time is already inside the parent's.
        """
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
                self._totals[name] = [elapsed, 1, elapsed]
            else:
                rec[0] += elapsed
                rec[1] += 1
                if elapsed > rec[2]:
                    rec[2] = elapsed

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
    def _tree(self) -> list[tuple[str, str, list]]:
        """
        Stages in print order: each top-level stage by total time, with its
        dotted children right under it. Returns (indent, label, record) rows.
        """
        parents: dict[str, list] = {}
        children: dict[str, list[tuple[str, list]]] = {}
        for name, rec in self._totals.items():
            parent, _, child = name.partition(".")
            if child:
                children.setdefault(parent, []).append((child, rec))
            else:
                parents[name] = rec
        # A breakdown whose parent is not itself timed still has to appear.
        for parent in children:
            parents.setdefault(parent, [0.0, 0, 0.0])

        rows: list[tuple[str, str, list]] = []
        for name, rec in sorted(parents.items(), key=lambda kv: kv[1][0],
                                reverse=True):
            rows.append(("", name, rec))
            for child, crec in sorted(children.get(name, []),
                                      key=lambda kv: kv[1][0], reverse=True):
                rows.append(("  ", child, crec))
        return rows

    def _work_ms_per_cycle(self) -> float:
        """Measured work per cycle, top-level stages only (children are inside them)."""
        if not self._cycles:
            return 0.0
        total = sum(rec[0] for name, rec in self._totals.items()
                    if "." not in name)
        return total / self._cycles * 1000.0

    def report(self, reset: bool = False) -> str:
        window = max(self._clock() - self._window_start, 1e-9)
        hz = self._cycles / window
        # ASCII only, deliberately. A report exists to be pasted into a terminal,
        # an issue or a chat, and a mangled separator is noise in all three — but
        # more than that, print() encodes with the console's codepage, and the
        # arrow that used to sit in front of BELOW TARGET is absent from cp1252.
        # A diagnostic that raises UnicodeEncodeError inside the camera thread —
        # only ever on the reports that carry bad news — is worse than useless.
        label = f"{self.label} | {self.context}" if self.context else self.label
        if self.report_rate:
            head = (f"[profile] {label} - {self._cycles} cycles in "
                    f"{window:.1f} s = {hz:.1f} Hz")
            if self.target_hz:
                head += f" (target {self.target_hz:.1f} Hz)"
                if hz < self.target_hz * 0.8:
                    head += "  <- BELOW TARGET: the work is the limit, not the cadence"
        else:
            head = f"[profile] {label} - over {window:.1f} s"
        lines = [head]

        # The headroom line: what the cycle costs against what the target rate
        # allows it to cost. This is the number a scaling test is really after.
        work = self._work_ms_per_cycle()
        if work > 0.0:
            if self.report_rate and self.target_hz:
                budget = 1000.0 / self.target_hz
                lines.append(f"    work {work:.1f} ms/cycle of a {budget:.1f} ms "
                             f"budget = {work / budget * 100.0:.0f}% busy")
            else:
                lines.append(f"    work {work:.1f} ms/cycle")

        rows = self._tree()
        if rows:
            lines.append(f"    {'stage':<18}{'total ms':>9}{'ms/cyc':>9}"
                         f"{'ms/call':>9}{'max':>9}{'calls':>7}")
        for indent, name, (secs, calls, mx) in rows:
            per_call = (secs / calls * 1000.0) if calls else 0.0
            per_cycle = (secs / self._cycles * 1000.0) if self._cycles else 0.0
            lines.append(f"    {indent}{name:<{18 - len(indent)}}"
                         f"{secs * 1000.0:9.1f}{per_cycle:9.2f}{per_call:9.2f}"
                         f"{mx * 1000.0:9.2f}{calls:7d}")
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


# The default for pure modules that accept an optional timer (stitcher,
# depth_extractor). Disabled, so `stage()` returns the shared no-op and nothing
# accumulates — which is also why one shared instance is safe across threads.
NULL_TIMER = StageTimer("null", enabled=False)
