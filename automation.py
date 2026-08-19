"""
Participant-Mode automation state machine.

Participant Mode lives in the ⧉ popup on the Depth viewport: an Auto toggle
arms a depth trigger (a distance in mm from the camera). When anything closer
than the trigger appears in frame the status becomes "Alerted", and once the
frame stays clear for a debounce period the pipeline runs automatically —
Sensing (capture) → Generating Paths → Actuating (save + run). While Auto is
ON the manual Capture/Generate/Run buttons in Developer Mode are locked out.

A second threshold, the **Max Drawing Time**, bounds how long one participant
may keep the sand occupied: the clock starts the moment the trigger is tripped
(hand enters) and stops when the frame goes clear (hand leaves). Running out is
a verdict, not an error — the status becomes "Invalid" exactly like a profanity
hit, so nothing is saved and nothing is sent to the robot.

This module is ONLY the transition logic, so it stays unit-testable: main.py
polls the camera trigger flag, calls ``tick()``, and drives the actual pipeline
(reusing the same handlers as the Developer-Mode buttons) when ``tick`` fires.
The clock is likewise injectable — every method that needs the time takes an
optional ``now`` (monotonic seconds), defaulting to ``time.monotonic()``.

Statuses (shown big in the popup, top-right):
  Auto Off          toggle off — popup is just the depth-number viewport
  Auto On           armed, frame clear, watching for something below trigger
  Alerted           something is closer than the trigger (the drawing clock runs)
  Sensing           frame cleared — capturing the averaged depth still
  Generating Paths  extracting strokes + projecting the toolpath
  Actuating         saving the bundle and running it on the robot
  Invalid           the drawing was refused — by the profanity guard, by the
                    Max Total Length, or by running out of drawing time.
                    Nothing was saved or run. Sticky: it stays on screen (still
                    armed) until the next participant trips the trigger.
"""
import time

# Statuses that mean "armed and watching the trigger". "Invalid" is included so
# a rejected drawing keeps its verdict visible without disarming the machine.
_ARMED = ("Auto On", "Invalid")


def format_duration(seconds: float) -> str:
    """Seconds → "M:SS" (the countdown's format, also used in messages)."""
    total = max(0, int(round(seconds)))
    return f"{total // 60}:{total % 60:02d}"


class ParticipantAutomation:
    """Edge-triggered (Alerted → clear) pipeline arming with a clear-debounce."""

    def __init__(self, clear_ticks: int = 10, max_draw_s: float | None = None):
        # How many consecutive clear ticks are needed before triggering — the
        # capture averages the PAST second, so the hand must be fully gone.
        self._clear_ticks_needed = max(1, int(clear_ticks))
        self._clear_count = 0
        # When the current clear run began (monotonic s) — the drawing clock is
        # frozen at that instant, since "hand leaves" is when drawing stopped,
        # not when the debounce finally expires.
        self._clear_start: float | None = None
        self._draw_t0: float | None = None    # hand entered (Alerted) at
        self._max_draw_s: float | None = None
        # Timed out mid-drawing: hold the Invalid verdict until the sand is
        # clear again, so the participant reads it instead of being re-Alerted
        # by the hand that is still in frame.
        self._await_clear = False
        self.enabled = False    # the Auto toggle is on
        self.busy = False       # pipeline currently running
        self.status = "Auto Off"
        self.message = ""
        self.set_max_draw_s(max_draw_s)

    # ── Thresholds ───────────────────────────────────────────────────────────
    def set_max_draw_s(self, seconds: float | None) -> None:
        """Set (or clear with None/0) the Max Drawing Time, in seconds."""
        try:
            value = float(seconds) if seconds is not None else 0.0
        except (TypeError, ValueError):
            value = 0.0
        self._max_draw_s = value if value > 0 else None

    @property
    def max_draw_s(self) -> float | None:
        return self._max_draw_s

    def set_enabled(self, enabled: bool) -> None:
        """Auto toggle switched on/off in the Participant popup."""
        self.enabled = enabled
        self._reset_clock()
        if not self.busy:
            self.status = "Auto On" if enabled else "Auto Off"
            self.message = ""

    # ── The drawing clock ────────────────────────────────────────────────────
    def _reset_clock(self) -> None:
        self._clear_count = 0
        self._clear_start = None
        self._draw_t0 = None
        self._await_clear = False

    def elapsed_s(self, now: float | None = None) -> float | None:
        """Seconds this participant has been drawing, or None if none is."""
        if self._draw_t0 is None:
            return None
        if now is None:
            now = time.monotonic()
        # Freeze while the frame is clear: the hand left at _clear_start.
        end = self._clear_start if self._clear_count else now
        return max(0.0, end - self._draw_t0)

    def remaining_s(self, now: float | None = None) -> float | None:
        """
        Seconds left on the Max Drawing Time — what the popup counts down.
        None when there is no limit or the machine is not armed. While nobody
        is drawing this is the full allowance, so the participant can see how
        long they will get before they start.
        """
        if self._max_draw_s is None or not self.enabled:
            return None
        elapsed = self.elapsed_s(now)
        if elapsed is None or self.busy:
            return self._max_draw_s
        return max(0.0, self._max_draw_s - elapsed)

    def _timed_out(self, now: float) -> bool:
        if self._max_draw_s is None:
            return False
        elapsed = self.elapsed_s(now)
        return elapsed is not None and elapsed >= self._max_draw_s

    def _time_up(self) -> None:
        """Max Drawing Time exhausted: same verdict as the profanity guard."""
        limit = format_duration(self._max_draw_s or 0.0)
        self.reject(
            f"Time's up — {limit} is the limit for one drawing. Nothing was "
            "saved; please rake it over so the next person can start.")
        # The hand is still in frame; don't re-arm on it.
        self._await_clear = True

    # ── Trigger ──────────────────────────────────────────────────────────────
    def tick(self, below: bool | None, now: float | None = None) -> bool:
        """
        Feed one trigger sample (True = something below threshold, False =
        clear, None = no camera data). Returns True exactly once per
        Alerted → sustained-clear edge: the pipeline should start (status is
        already "Sensing" and the machine is busy until ``finish()``).
        """
        if not self.enabled or self.busy or below is None:
            return False
        if now is None:
            now = time.monotonic()

        if self._await_clear:
            # Timed out: the verdict stands until the sand is clear again.
            if below:
                self._clear_count = 0
            elif self._count_clear(now) >= self._clear_ticks_needed:
                self._reset_clock()
            return False

        if self.status in _ARMED:
            if below:
                self.status = "Alerted"
                self.message = ""
                self._clear_count = 0
                self._clear_start = None
                self._draw_t0 = now          # the drawing clock starts here
        elif self.status == "Alerted":
            if below:
                self._clear_count = 0
                self._clear_start = None
            else:
                self._count_clear(now)
            if self._timed_out(now):
                self._time_up()
                return False
            if self._clear_count >= self._clear_ticks_needed:
                self.busy = True
                self.status = "Sensing"
                self.message = ""
                self._draw_t0 = None
                return True
        return False

    def _count_clear(self, now: float) -> int:
        if self._clear_count == 0:
            self._clear_start = now
        self._clear_count += 1
        return self._clear_count

    # ── Pipeline lifecycle ───────────────────────────────────────────────────
    def stage(self, status: str, message: str = "") -> None:
        """Advance the busy pipeline: "Generating Paths", then "Actuating"."""
        self.status = status
        self.message = message

    def finish(self, message: str = "") -> None:
        """Pipeline done (or aborted): re-arm, keeping the outcome message."""
        self.busy = False
        self._reset_clock()
        self.status = "Auto On" if self.enabled else "Auto Off"
        self.message = message

    def reject(self, message: str = "") -> None:
        """
        The drawing was refused — by the profanity guard, by the Max Total
        Length, or by the Max Drawing Time running out. Stop before Actuating,
        so nothing is saved and nothing is sent to the robot.

        Unlike ``finish`` the status stays "Invalid" rather than reverting to
        "Auto On" — the verdict has to be readable by whoever just drew it. The
        machine is still armed (see ``_ARMED``), so the next trigger proceeds
        normally. Toggling Auto off and on also clears it.
        """
        self.busy = False
        self._reset_clock()
        self.status = "Invalid" if self.enabled else "Auto Off"
        self.message = message
