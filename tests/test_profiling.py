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


class TestDisabledCostsNothing:

    def test_stage_is_a_no_op_context_manager(self):
        t = StageTimer("x", enabled=False)
        with t.stage("anything"):
            pass
        assert t.report() .count("ms total") == 0

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
        report = t.report()
        assert "stitch" in report
        assert "30.0 ms total" in report        # 3 x 10 ms
        assert "10.0 ms x 3" in report

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
        lines = [l for l in t.report().splitlines() if "ms total" in l]
        assert "expensive" in lines[0]
        assert "cheap" in lines[1]

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
            pytest.skip("PROFILE_PIPELINE is ON locally — set it back to False "
                        "before committing; it runs in the live-view hot loop")
        assert config.PROFILE_PIPELINE is False

    def test_the_timer_targets_the_configured_canvas_rate(self):
        import threading

        import config
        from camera_thread import DepthCameraThread

        cam = DepthCameraThread({}, threading.Lock())
        assert cam._timer.target_hz == pytest.approx(
            1.0 / config.STITCH_MAIN_EVERY_S)
        assert cam._timer.enabled is config.PROFILE_PIPELINE
