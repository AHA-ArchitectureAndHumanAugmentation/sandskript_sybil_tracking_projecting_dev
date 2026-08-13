"""
Participant-Mode automation state machine.

Participant Mode lives in the ⧉ popup on the Depth viewport: an Auto toggle
arms a depth trigger (a distance in mm from the camera). When anything closer
than the trigger appears in frame the status becomes "Alerted", and once the
frame stays clear for a debounce period the pipeline runs automatically —
Sensing (capture) → Generating Paths → Actuating (save + run). While Auto is
ON the manual Capture/Generate/Run buttons in Developer Mode are locked out.

This module is ONLY the transition logic, so it stays unit-testable: main.py
polls the camera trigger flag, calls ``tick()``, and drives the actual pipeline
(reusing the same handlers as the Developer-Mode buttons) when ``tick`` fires.

Statuses (shown big in the popup, top-right):
  Auto Off          toggle off — popup is just the depth-number viewport
  Auto On           armed, frame clear, watching for something below trigger
  Alerted           something is closer than the trigger
  Sensing           frame cleared — capturing the averaged depth still
  Generating Paths  extracting strokes + projecting the toolpath
  Actuating         saving the bundle and running it on the robot
  Invalid           the profanity guard rejected the drawing — nothing was
                    saved or run. Sticky: it stays on screen (still armed) until
                    the next participant trips the trigger.
"""

# Statuses that mean "armed and watching the trigger". "Invalid" is included so
# a rejected drawing keeps its verdict visible without disarming the machine.
_ARMED = ("Auto On", "Invalid")


class ParticipantAutomation:
    """Edge-triggered (Alerted → clear) pipeline arming with a clear-debounce."""

    def __init__(self, clear_ticks: int = 10):
        # How many consecutive clear ticks are needed before triggering — the
        # capture averages the PAST second, so the hand must be fully gone.
        self._clear_ticks_needed = max(1, int(clear_ticks))
        self._clear_count = 0
        self.enabled = False    # the Auto toggle is on
        self.busy = False       # pipeline currently running
        self.status = "Auto Off"
        self.message = ""

    def set_enabled(self, enabled: bool) -> None:
        """Auto toggle switched on/off in the Participant popup."""
        self.enabled = enabled
        self._clear_count = 0
        if not self.busy:
            self.status = "Auto On" if enabled else "Auto Off"
            self.message = ""

    def tick(self, below: bool | None) -> bool:
        """
        Feed one trigger sample (True = something below threshold, False =
        clear, None = no camera data). Returns True exactly once per
        Alerted → sustained-clear edge: the pipeline should start (status is
        already "Sensing" and the machine is busy until ``finish()``).
        """
        if not self.enabled or self.busy or below is None:
            return False

        if self.status in _ARMED:
            if below:
                self.status = "Alerted"
                self.message = ""
                self._clear_count = 0
        elif self.status == "Alerted":
            if below:
                self._clear_count = 0
            else:
                self._clear_count += 1
                if self._clear_count >= self._clear_ticks_needed:
                    self.busy = True
                    self.status = "Sensing"
                    self.message = ""
                    return True
        return False

    def stage(self, status: str, message: str = "") -> None:
        """Advance the busy pipeline: "Generating Paths", then "Actuating"."""
        self.status = status
        self.message = message

    def finish(self, message: str = "") -> None:
        """Pipeline done (or aborted): re-arm, keeping the outcome message."""
        self.busy = False
        self._clear_count = 0
        self.status = "Auto On" if self.enabled else "Auto Off"
        self.message = message

    def reject(self, message: str = "") -> None:
        """
        The profanity guard refused the drawing: stop before Actuating, so
        nothing is saved and nothing is sent to the robot.

        Unlike ``finish`` the status stays "Invalid" rather than reverting to
        "Auto On" — the verdict has to be readable by whoever just drew it. The
        machine is still armed (see ``_ARMED``), so the next trigger proceeds
        normally. Toggling Auto off and on also clears it.
        """
        self.busy = False
        self._clear_count = 0
        self.status = "Invalid" if self.enabled else "Auto Off"
        self.message = message
