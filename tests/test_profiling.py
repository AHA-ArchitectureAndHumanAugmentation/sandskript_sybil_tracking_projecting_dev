"""
Unit tests for profiling.StageTimer — the live-view diagnostic.

Two properties matter and both are easy to get wrong: it must cost nothing and
change nothing while disabled (it ships disabled, in the hot loop that produces
every live view), and its report must be readable enough to actually diagnose
the "is the cadence the limit, or is the work?" question it exists for. The
clock is injected so none of this needs real time to pass.
"""
import pytest

from profiling import StageTimer


class FakeClock:
    """A hand-cranked clock, so tests never sleep."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, seconds):
        self.t += seconds


def _stage_rows(report):
    """The per-stage table rows, in print order (header and prose dropped)."""
    out = []
    for line in report.splitlines():
        if not line.startswith("    ") or line.lstrip().startswith("stage "):
            continue
        if line.lstrip().startswith("work "):
            continue
        out.append(line)
    return out


def _row(report, name):
    """The one table row for `name`, so a test can assert on its numbers."""
    for line in _stage_rows(report):
        if line.split()[0] == name:
            return line
    raise AssertionError(f"no row for {name!r} in:\n{report}")


class TestDisabledCostsNothing:

    def test_stage_is_a_no_op_context_manager(self):
        t = StageTimer("x", enabled=False)
        with t.stage("anything"):
            pass
        assert "anything" not in t.report()

    def test_disabled_records_nothing(self):
        clock = FakeClock()
        t = StageTimer("x", enabled=False, clock=clock)
        with t.stage("stitch"):
            clock.advance(1.0)
        t.count("frames", 10)
        assert "stitch" not in t.report()
        assert "frames" not in t.report()

    def test_disabled_never_reports(self):
        """The hot loop calls cycle() every canvas — it must stay silent."""
        clock = FakeClock()
        t = StageTimer("x", enabled=False, every_s=1.0, clock=clock)
        for _ in range(100):
            clock.advance(1.0)
            assert t.cycle() is None

    def test_reused_null_context_is_shared(self):
        """No per-call allocation in the disabled path."""
        t = StageTimer("x", enabled=False)
        assert t.stage("a") is t.stage("b")


class TestTiming:

    def test_stage_totals_and_call_counts(self):
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, clock=clock)
        for _ in range(3):
            with t.stage("stitch"):
                clock.advance(0.010)
        row = _row(t.report(), "stitch")
        assert "30.0" in row                    # total: 3 x 10 ms
        assert "10.00" in row                   # per call
        assert row.split()[-1] == "3"           # calls

    def test_a_raising_stage_is_still_recorded(self):
        """A stage that throws must not silently vanish from the numbers."""
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, clock=clock)
        with pytest.raises(ValueError):
            with t.stage("detect"):
                clock.advance(0.005)
                raise ValueError("boom")
        assert "detect" in t.report()

    def test_stages_are_ordered_by_total_time(self):
        """The point of the report is 'what is slowest' — so it leads."""
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, clock=clock)
        with t.stage("cheap"):
            clock.advance(0.001)
        with t.stage("expensive"):
            clock.advance(0.100)
        rows = _stage_rows(t.report())
        assert "expensive" in rows[0]
        assert "cheap" in rows[1]

    def test_the_worst_single_call_is_reported(self):
        """A stage that averages 5 ms with a 40 ms max is a stutter, not a cost."""
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, clock=clock)
        for ms in (2, 2, 40, 2):
            with t.stage("stitch"):
                clock.advance(ms / 1000.0)
        row = _row(t.report(), "stitch")
        assert "40.00" in row                   # the spike survives the average

    def test_counters_report_a_rate(self):
        clock = FakeClock()
        t = StageTimer("mjpeg", enabled=True, clock=clock)
        t.count("depth.sent", 300)
        t.count("depth.new", 50)
        clock.advance(10.0)
        report = t.report()
        assert "depth.sent" in report and "300" in report
        assert "depth.new" in report


class TestCycleAndWindow:

    def test_no_report_before_the_window_elapses(self):
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, every_s=5.0, clock=clock)
        for _ in range(10):
            clock.advance(0.2)
            if clock.t < 5.0:
                assert t.cycle() is None

    def test_report_arrives_and_resets(self):
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, every_s=1.0, clock=clock)
        with t.stage("stitch"):
            clock.advance(0.5)
        clock.advance(0.6)
        line = t.cycle()
        assert line is not None and "stitch" in line
        # Window reset: the next report must not carry the old numbers.
        clock.advance(1.1)
        assert "stitch" not in t.cycle()

    def test_achieved_rate_is_reported(self):
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, every_s=1.0, clock=clock)
        for _ in range(4):
            clock.advance(0.25)
            line = t.cycle()
        assert "4 cycles" in line
        assert "4.0 Hz" in line

    def test_falling_short_of_target_is_called_out(self):
        """The whole diagnosis hinges on this one comparison."""
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, every_s=1.0, target_hz=5.0,
                       clock=clock)
        for _ in range(2):                      # 2 Hz against a 5 Hz target
            clock.advance(0.5)
            line = t.cycle()
        assert "target 5.0 Hz" in line
        assert "BELOW TARGET" in line

    def test_hitting_target_is_not_flagged(self):
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, every_s=1.0, target_hz=5.0,
                       clock=clock)
        for _ in range(5):
            clock.advance(0.2)
            line = t.cycle()
        assert "target 5.0 Hz" in line
        assert "BELOW TARGET" not in line


class TestBreakdownsNestUnderTheirStage:
    """
    A dotted name is a breakdown of the stage above it, recorded INSIDE that
    stage's own timing. Getting this wrong would double-count the expensive
    stages — which are exactly the ones a scaling test is looking at.
    """

    def test_a_child_prints_under_its_parent(self):
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, clock=clock)
        with t.stage("colorize"):
            clock.advance(0.002)
        with t.stage("stitch"):
            with t.stage("stitch.warp_depth"):
                clock.advance(0.020)
            clock.advance(0.005)
        rows = _stage_rows(t.report())
        assert rows[0].split()[0] == "stitch"
        assert rows[1].split()[0] == "warp_depth"
        assert rows[1].startswith("      ")      # indented = a breakdown
        assert rows[2].split()[0] == "colorize"

    def test_children_are_left_out_of_the_work_total(self):
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, every_s=1.0, target_hz=10.0,
                       clock=clock)
        with t.stage("stitch"):
            with t.stage("stitch.warp_depth"):
                clock.advance(0.030)
            with t.stage("stitch.blend"):
                clock.advance(0.010)
        clock.advance(1.0)
        line = t.cycle()
        # 40 ms of stitch, not 80 ms of stitch-plus-its-own-parts.
        assert "work 40.0 ms/cycle" in line
        assert "of a 100.0 ms budget = 40% busy" in line

    def test_a_breakdown_without_a_timed_parent_still_shows(self):
        """Half-instrumented is a normal state mid-investigation, not an error."""
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, clock=clock)
        with t.stage("detect.blur"):
            clock.advance(0.006)
        rows = _stage_rows(t.report())
        assert rows[0].split()[0] == "detect"    # a placeholder parent
        assert rows[1].split()[0] == "blur"

    def test_work_is_per_cycle_not_per_window(self):
        """Two canvases of 20 ms each is 20 ms/cycle, however long the window."""
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, every_s=1.0, target_hz=10.0,
                       clock=clock)
        for _ in range(2):
            with t.stage("stitch"):
                clock.advance(0.020)
            clock.advance(1.0)          # a long, mostly idle window
            line = t.cycle()
        assert "work 20.0 ms/cycle" in line
        assert "= 20% busy" in line     # of the 100 ms the target rate allows


class TestTheReportSaysWhatItMeasured:
    """
    Three runs of a scaling test produce three reports that differ only in their
    numbers. Without the rig in the header there is no way to tell them apart
    once they are pasted somewhere.
    """

    def test_the_context_appears_in_the_header(self):
        t = StageTimer("canvas", enabled=True, context="2 cam | 1216x480 px")
        assert "canvas | 2 cam | 1216x480 px" in t.report()

    def test_the_context_can_be_set_after_construction(self):
        """The canvas size is only known once the cameras have delivered a frame."""
        t = StageTimer("canvas", enabled=True)
        assert "canvas -" in t.report()
        t.set_context("3 cam | 1824x480 px")
        assert "canvas | 3 cam | 1824x480 px" in t.report()

    def test_the_context_survives_a_window_reset(self):
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, every_s=1.0, clock=clock)
        t.set_context("1 cam")
        for _ in range(2):
            clock.advance(1.1)
            assert "1 cam" in t.cycle()

    def test_the_report_is_ascii(self):
        """
        Not cosmetic: print() encodes with the console's codepage, and cp1252 has
        no arrow. A report that raised UnicodeEncodeError would take the camera
        thread down — and only on the reports that carry bad news.
        """
        clock = FakeClock()
        t = StageTimer("canvas", enabled=True, every_s=1.0, target_hz=10.0,
                       context="2 cam | 1216x480 px | 2.11 mm/px", clock=clock)
        with t.stage("stitch"):
            clock.advance(0.02)
        with t.stage("stitch.warp_depth"):
            clock.advance(0.01)
        t.count("mask.held", 3)
        clock.advance(2.0)                  # slow enough to earn the warning
        line = t.cycle()
        assert "BELOW TARGET" in line
        line.encode("ascii")


class TestTheHotStagesReportTheirParts:
    """
    The two stages that own the live cycle are opaque blobs without this, and
    "stitch is 46% of the time" is not something you can act on. Both take an
    optional timer and default to measuring nothing.
    """

    def _timer(self):
        return StageTimer("canvas", enabled=True)

    def test_stitch_breaks_itself_down(self):
        from stitcher import stitch, synthetic_scene

        frames, calib = synthetic_scene(2)
        t = self._timer()
        stitch(frames, calib, timer=t)
        report = t.report()
        for part in ("prepare", "alloc", "warp_depth", "blend"):
            assert _row(report, part), part

    def test_stitch_measures_the_warp_once_per_camera(self):
        """
        ms/call is then one camera's cost and ms/cyc the whole rig's — the pair
        that separates "more cameras" from "bigger canvas".
        """
        from stitcher import stitch, synthetic_scene

        for n in (1, 3):
            frames, calib = synthetic_scene(n)
            t = self._timer()
            stitch(frames, calib, timer=t)
            assert _row(t.report(), "warp_depth").split()[-1] == str(n)

    def test_detection_breaks_itself_down(self):
        import numpy as np
        from depth_extractor import DepthGrooveParams, grooves_and_mask

        depth = np.full((120, 160), 1.0, np.float32)
        depth[60:64, 20:140] += 0.003
        sand = np.full((120, 160), 1.0, np.float32)
        t = self._timer()
        grooves_and_mask(depth, None, DepthGrooveParams(), reference=sand, timer=t)
        report = t.report()
        for part in ("prep", "blur", "threshold", "morph", "skeleton"):
            assert _row(report, part), part

    def test_the_parts_nest_inside_the_callers_own_stage(self):
        """
        The camera thread times `stitch`/`detect` itself; the parts must land
        under those names or the report would grow a second top-level row for
        the same work.
        """
        import numpy as np
        from depth_extractor import DepthGrooveParams, grooves_and_mask
        from stitcher import stitch, synthetic_scene

        frames, calib = synthetic_scene(1)
        t = self._timer()
        with t.stage("stitch"):
            stitch(frames, calib, timer=t)
        with t.stage("detect"):
            sand = np.full((80, 80), 1.0, np.float32)
            grooves_and_mask(sand, None, DepthGrooveParams(),
                             reference=sand, timer=t)
        tops = [r.split()[0] for r in _stage_rows(t.report())
                if not r.startswith("      ")]
        assert sorted(tops) == ["detect", "stitch"]

    def test_measuring_nothing_is_the_default(self):
        """
        The capture path calls both of these. It must not pay for, or be
        perturbed by, a diagnostic nobody asked for.
        """
        import inspect

        import numpy as np
        from depth_extractor import DepthGrooveParams, grooves_and_mask
        from profiling import NULL_TIMER
        from stitcher import stitch, synthetic_scene

        for fn in (stitch, grooves_and_mask):
            assert inspect.signature(fn).parameters["timer"].default is NULL_TIMER
        frames, calib = synthetic_scene(1)
        stitch(frames, calib)
        grooves_and_mask(np.full((60, 60), 1.0, np.float32), None,
                         DepthGrooveParams())
        assert NULL_TIMER.report().strip().startswith("[profile]")
        assert "warp_depth" not in NULL_TIMER.report()


class TestWiredIntoTheCameraThread:

    def test_camera_thread_ships_with_profiling_off(self):
        """
        It lives in the hot loop, so shipping it enabled would be a regression.

        Switching it on locally to take a reading is the whole point of the
        flag, though, and a red suite is the wrong way to react to someone
        using a tool correctly — so that case skips with a reminder instead.
        A skip still shows up in the run, which is the nudge that matters.
        """
        import config
        if config.PROFILE_PIPELINE:
            pytest.skip("PROFILE_PIPELINE is ON — if that came from "
                        "SANDSKRIPT_PROFILE, fine, it clears itself; if it was "
                        "edited into config.py, set it back before committing. "
                        "It runs in the live-view hot loop")
        assert config.PROFILE_PIPELINE is False

    def test_the_timer_targets_the_configured_canvas_rate(self):
        import threading

        import config
        from camera_thread import DepthCameraThread

        cam = DepthCameraThread({}, threading.Lock())
        assert cam._timer.target_hz == pytest.approx(
            1.0 / config.STITCH_MAIN_EVERY_S)
        assert cam._timer.enabled is config.PROFILE_PIPELINE


class TestTheRigCanBeCappedForAScalingTest:
    """
    Measuring 1, then 2, then 3 cameras means opening a different number of them.
    An unopened camera streams nothing, so a cap is equivalent to unplugging —
    and it takes the same first N by serial every time, which is what makes the
    three runs comparable.
    """

    def test_the_cap_is_off_by_default(self):
        import config
        assert config.CAMERA_LIMIT == 0, (
            "CAMERA_LIMIT is committed as 0 = every camera; a leftover cap "
            "would silently shrink the rig on the next run")

    def test_the_cap_is_read_from_the_environment(self, monkeypatch):
        import importlib

        import config
        monkeypatch.setenv("SANDSKRIPT_CAMERAS", "2")
        monkeypatch.setenv("SANDSKRIPT_PROFILE", "1")
        reloaded = importlib.reload(config)
        try:
            assert reloaded.CAMERA_LIMIT == 2
            assert reloaded.PROFILE_PIPELINE is True
        finally:
            monkeypatch.undo()
            importlib.reload(config)

    def test_nonsense_in_the_environment_is_ignored(self, monkeypatch):
        """A typo must fall back to the committed value, not crash a run."""
        import importlib

        import config
        monkeypatch.setenv("SANDSKRIPT_CAMERAS", "two")
        reloaded = importlib.reload(config)
        try:
            assert reloaded.CAMERA_LIMIT == 0
        finally:
            monkeypatch.undo()
            importlib.reload(config)

    def test_the_cap_never_exceeds_the_hard_ceiling(self):
        import camera_thread
        import config

        assert camera_thread._MAX_CAMERAS <= config.STITCH_MAX_CAMERAS
        if config.CAMERA_LIMIT == 0:
            assert camera_thread._MAX_CAMERAS == config.STITCH_MAX_CAMERAS
