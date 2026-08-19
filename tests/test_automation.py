"""
Unit tests for automation.py — the Participant-Mode state machine. Pure logic,
no hardware: main.py feeds it the camera's below-threshold flag via tick().
"""
from automation import ParticipantAutomation, format_duration


def make(clear_ticks: int = 3, max_draw_s=None) -> ParticipantAutomation:
    a = ParticipantAutomation(clear_ticks=clear_ticks, max_draw_s=max_draw_s)
    a.set_enabled(True)
    return a


class TestArming:

    def test_starts_off(self):
        a = ParticipantAutomation()
        assert a.status == "Auto Off"
        assert a.tick(True) is False          # disabled: below is ignored

    def test_enable_arms_watching(self):
        a = make()
        assert a.status == "Auto On"

    def test_disable_returns_to_off(self):
        a = make()
        a.tick(True)                          # Alerted
        a.set_enabled(False)
        assert a.status == "Auto Off"
        assert a.tick(False) is False         # no trigger after disarm


class TestTriggerEdge:

    def test_below_alerts(self):
        a = make()
        a.tick(True)
        assert a.status == "Alerted"

    def test_sustained_clear_triggers_once(self):
        a = make(clear_ticks=3)
        a.tick(True)
        assert a.tick(False) is False
        assert a.tick(False) is False
        assert a.tick(False) is True          # 3rd clear tick fires
        assert a.status == "Sensing" and a.busy
        assert a.tick(False) is False         # never twice per edge

    def test_flicker_resets_the_debounce(self):
        a = make(clear_ticks=3)
        a.tick(True)
        a.tick(False); a.tick(False)
        a.tick(True)                          # hand back in → restart count
        assert a.tick(False) is False
        assert a.tick(False) is False
        assert a.tick(False) is True

    def test_watching_clear_never_triggers(self):
        a = make(clear_ticks=1)
        for _ in range(5):
            assert a.tick(False) is False     # edge-triggered: needs Alerted first

    def test_none_means_no_data(self):
        a = make(clear_ticks=1)
        a.tick(True)                          # Alerted
        assert a.tick(None) is False          # camera gap must not fire the robot
        assert a.status == "Alerted"


class TestPipelineLifecycle:

    def test_busy_ignores_trigger(self):
        a = make(clear_ticks=1)
        a.tick(True)
        assert a.tick(False) is True          # pipeline starts
        a.tick(True)
        assert a.tick(False) is False         # busy: no re-trigger
        assert a.status == "Sensing"

    def test_stages_and_finish_rearm(self):
        a = make(clear_ticks=1)
        a.tick(True); a.tick(False)
        a.stage("Generating Paths")
        assert a.status == "Generating Paths"
        a.stage("Actuating")
        a.finish("Done.")
        assert a.status == "Auto On" and not a.busy
        assert a.message == "Done."
        a.tick(True)
        assert a.tick(False) is True          # ready for the next participant

    def test_finish_after_disable_goes_off(self):
        a = make(clear_ticks=1)
        a.tick(True); a.tick(False)           # busy
        a.set_enabled(False)                  # Auto toggled off mid-run
        assert a.status == "Sensing"          # pipeline keeps its stage...
        a.finish("Done.")
        assert a.status == "Auto Off"              # ...but re-arms disabled


class TestProfanityRejection:
    """reject() = the profanity guard refused the drawing (Participant Mode)."""

    def test_reject_sets_invalid_and_clears_busy(self):
        a = make(clear_ticks=1)
        a.tick(True); a.tick(False)           # busy, Sensing
        a.stage("Generating Paths")
        a.reject("Drawing rejected.")
        assert a.status == "Invalid"
        assert a.busy is False
        assert a.message == "Drawing rejected."

    def test_invalid_is_sticky_until_the_next_trigger(self):
        """The verdict has to stay readable — it must not revert to Auto On."""
        a = make(clear_ticks=1)
        a.tick(True); a.tick(False)
        a.reject("nope")
        assert a.tick(False) is False         # quiet frames leave it alone
        assert a.status == "Invalid"

    def test_still_armed_after_reject(self):
        a = make(clear_ticks=1)
        a.tick(True); a.tick(False)
        a.reject("nope")
        a.tick(True)                          # next participant steps in
        assert a.status == "Alerted"
        assert a.tick(False) is True          # pipeline runs again normally

    def test_reject_while_disabled_goes_off(self):
        a = make(clear_ticks=1)
        a.tick(True); a.tick(False)
        a.set_enabled(False)
        a.reject("nope")
        assert a.status == "Auto Off"

    def test_toggling_auto_clears_the_invalid_verdict(self):
        a = make(clear_ticks=1)
        a.tick(True); a.tick(False)
        a.reject("nope")
        a.set_enabled(False)
        a.set_enabled(True)
        assert a.status == "Auto On"
        assert a.message == ""

    def test_finish_still_returns_to_auto_on(self):
        """A normal run is unaffected by the new Invalid status."""
        a = make(clear_ticks=1)
        a.tick(True); a.tick(False)
        a.finish("Done.")
        assert a.status == "Auto On"


class TestMaxDrawingTime:
    """The Max Drawing Time box: hand-in → hand-out, or the drawing is Invalid.

    Every tick takes an explicit ``now`` (monotonic seconds), so the clock is
    tested without sleeping.
    """

    def test_no_limit_never_times_out(self):
        a = make(clear_ticks=1)                       # box empty = no limit
        a.tick(True, now=0)
        assert a.tick(False, now=10_000) is True      # an hour of drawing is fine
        assert a.remaining_s(now=10_000) is None

    def test_remaining_is_the_full_allowance_while_nobody_draws(self):
        a = make(clear_ticks=1, max_draw_s=300)
        assert a.remaining_s(now=0) == 300            # armed: shows what you get

    def test_remaining_counts_down_while_drawing(self):
        a = make(clear_ticks=3, max_draw_s=300)
        a.tick(True, now=0)
        a.tick(True, now=60)
        assert a.remaining_s(now=60) == 240

    def test_countdown_is_off_when_disabled(self):
        a = ParticipantAutomation(clear_ticks=1, max_draw_s=300)
        assert a.remaining_s(now=0) is None           # Auto Off shows no clock

    def test_overrun_is_invalid_and_saves_nothing(self):
        a = make(clear_ticks=2, max_draw_s=10)
        a.tick(True, now=0)
        assert a.tick(True, now=11) is False          # never starts the pipeline
        assert a.status == "Invalid"
        assert a.busy is False
        assert "0:10" in a.message                    # says what the limit was

    def test_overrun_holds_the_verdict_until_the_sand_is_clear(self):
        """The hand is still in frame — re-Alerting on it would hide the verdict."""
        a = make(clear_ticks=2, max_draw_s=10)
        a.tick(True, now=0)
        a.tick(True, now=11)                          # timed out → Invalid
        a.tick(True, now=12)
        assert a.status == "Invalid"
        a.tick(False, now=13); a.tick(False, now=14)  # sand clear again
        assert a.status == "Invalid"                  # still sticky, still armed

    def test_next_participant_gets_a_fresh_clock(self):
        a = make(clear_ticks=2, max_draw_s=10)
        a.tick(True, now=0)
        a.tick(True, now=11)                          # timed out
        a.tick(False, now=12); a.tick(False, now=13)  # clear
        a.tick(True, now=20)                          # next participant
        assert a.status == "Alerted"
        assert a.remaining_s(now=20) == 10
        assert a.tick(False, now=21) is False
        assert a.tick(False, now=22) is True          # runs normally

    def test_finishing_just_in_time_is_accepted(self):
        """The clock stops when the hand LEAVES, not when the debounce expires."""
        a = make(clear_ticks=3, max_draw_s=10)
        a.tick(True, now=0)
        a.tick(False, now=9)                          # hand out with 1 s to spare
        a.tick(False, now=10.5)                       # debounce runs past the limit
        assert a.tick(False, now=12) is True          # still a valid drawing
        assert a.status == "Sensing"

    def test_hand_returning_resumes_the_clock(self):
        a = make(clear_ticks=3, max_draw_s=10)
        a.tick(True, now=0)
        a.tick(False, now=5)                          # paused at 5 s
        assert a.remaining_s(now=8) == 5
        a.tick(True, now=8)                           # back in → counting again
        assert a.remaining_s(now=9) == 1

    def test_limit_can_be_changed_or_cleared_live(self):
        a = make(clear_ticks=1, max_draw_s=10)
        a.set_max_draw_s(None)
        a.tick(True, now=0)
        assert a.tick(True, now=999) is False and a.status == "Alerted"
        a.set_max_draw_s(0)                           # 0 = off, same as empty
        assert a.max_draw_s is None
        a.set_max_draw_s(60)
        assert a.max_draw_s == 60

    def test_toggling_auto_resets_the_clock(self):
        a = make(clear_ticks=1, max_draw_s=10)
        a.tick(True, now=0)
        a.set_enabled(False)
        a.set_enabled(True)
        assert a.remaining_s(now=100) == 10           # not "9:50 overdue"


def test_format_duration():
    assert format_duration(0) == "0:00"
    assert format_duration(9.4) == "0:09"
    assert format_duration(300) == "5:00"
    assert format_duration(3661) == "61:01"
