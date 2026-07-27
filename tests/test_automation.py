"""
Unit tests for automation.py — the Participant-Mode state machine. Pure logic,
no hardware: main.py feeds it the camera's below-threshold flag via tick().
"""
from automation import ParticipantAutomation


def make(clear_ticks: int = 3) -> ParticipantAutomation:
    a = ParticipantAutomation(clear_ticks=clear_ticks)
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
