import threading
from typing import Optional

from scipy.spatial.transform import Rotation

from config import (
    DRAW_Z, TRAVEL_Z, DRAW_SPEED, TRAVEL_ACCEL, DRAW_ACCEL, MOVEP_BLEND_M,
)
from path_export import stroke_blend


def _offset_pose(pose: list[float], dist: float) -> list[float]:
    """Shift a waypoint by ``dist`` along its outward normal (anti tool-Z)."""
    n = -Rotation.from_rotvec(pose[3:6]).apply([0.0, 0.0, 1.0])
    return [pose[0] + dist * n[0],
            pose[1] + dist * n[1],
            pose[2] + dist * n[2]] + list(pose[3:6])


def _retract(pose: list[float], dist: float) -> list[float]:
    """
    Offset ``pose`` by ``dist`` along the tool's retreat direction (opposite the
    tool Z / approach axis). For the classic tool-down orientation [0, π, 0]
    this is exactly base +Z — but on a tilted or vertical target surface it pulls
    AWAY from the surface instead of sliding along it.
    """
    n = -Rotation.from_rotvec(pose[3:6]).apply([0.0, 0.0, 1.0])
    return [pose[0] + dist * n[0],
            pose[1] + dist * n[1],
            pose[2] + dist * n[2]] + list(pose[3:6])


class PathExecutor:
    """
    Executes an ordered list of robot-space strokes via blocking moveL calls.
    Runs in a daemon thread so the async event loop is never blocked.
    Progress and phase are written back to shared_state as execution proceeds.
    """

    def __init__(self, robot, shared_state: dict, state_lock: threading.Lock) -> None:
        self._robot = robot
        self._state = shared_state
        self._state_lock = state_lock
        self._cancel_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._draw_z: float = DRAW_Z
        self._draw_speed: float = DRAW_SPEED
        self._travel_dist: float = TRAVEL_Z
        self._blend_m: float = MOVEP_BLEND_M

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, strokes: list[list[list[float]]], draw_z: float = DRAW_Z,
              draw_speed: float = DRAW_SPEED, normal_offset: float = 0.0,
              travel_dist: float = TRAVEL_Z, blend_m: float = MOVEP_BLEND_M) -> None:
        """
        draw_z        base-frame Z offset while drawing. Planar mode uses config
                      DRAW_Z; surface mode passes 0.0 (depth baked into waypoints).
        draw_speed    m/s along the toolpath (the UI Speed slider, % of MAX_TCP_SPEED).
        normal_offset m added at run time along each waypoint's tool axis — lifts
                      the TCP off the surface without regenerating the path.
        travel_dist   m retracted along the tool axis before/between/after strokes.
        blend_m       movep corner blend radius (the UI Radius slider); clamped
                      per stroke so it never reaches half a segment length.
        """
        if self.running:
            return
        self._draw_z = draw_z
        self._draw_speed = max(draw_speed, 0.005)
        self._travel_dist = max(travel_dist, 0.005)
        self._blend_m = max(blend_m, 0.0)
        if normal_offset:
            strokes = [[_offset_pose(p, normal_offset) for p in s] for s in strokes]
        self._cancel_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            args=(strokes,),
            daemon=True,
            name="path_executor",
        )
        self._thread.start()

    def cancel(self) -> None:
        self._cancel_event.set()
        if self._robot.connected:
            self._robot.stop_motion()

    def _run(self, strokes: list[list[list[float]]]) -> None:
        with self._state_lock:
            self._state["executing"]  = True
            self._state["phase"]      = "executing"
            self._state["progress"]   = 0.0
            self._state["exec_error"] = None

        total_moves = sum(len(s) for s in strokes) + len(strokes) * 2
        moves_done  = 0
        cancelled   = False

        try:
            current_ee = self._robot.get_ee_position()
            cx, cy, cz = current_ee[0], current_ee[1], current_ee[2]
            rx, ry, rz = current_ee[3], current_ee[4], current_ee[5]

            for stroke in strokes:
                if self._cancel_event.is_set():
                    cancelled = True
                    break

                # Retract from the current position along the tool axis (pulls
                # away from the surface even when it is tilted or vertical).
                lift_pose = _retract([cx, cy, cz, rx, ry, rz], self._travel_dist)
                self._robot.move_to(lift_pose, self._draw_speed, TRAVEL_ACCEL)
                moves_done += 1
                self._update_progress(moves_done, total_moves)

                if self._cancel_event.is_set():
                    cancelled = True
                    break

                # Travel to a point retracted off the stroke start, in the
                # stroke's own approach orientation.
                travel_pose = _retract(stroke[0], self._travel_dist)
                self._robot.move_to(travel_pose, self._draw_speed, TRAVEL_ACCEL)
                moves_done += 1
                self._update_progress(moves_done, total_moves)

                if self._cancel_event.is_set():
                    cancelled = True
                    break

                # Draw the stroke as a blended movep process move — identical
                # actuation to the saved path.script.
                self._movep_draw_stroke(stroke)

                moves_done += len(stroke)
                self._update_progress(moves_done, total_moves)

                if self._cancel_event.is_set():
                    cancelled = True
                    break

                cx, cy, cz = stroke[-1][0], stroke[-1][1], stroke[-1][2] + self._draw_z
                rx, ry, rz = stroke[-1][3], stroke[-1][4], stroke[-1][5]

            # Final pen-up — retract along the tool axis. All non-drawing moves
            # use the same speed as drawing, so the Speed slider governs the
            # entire actuation uniformly.
            if self._robot.connected:
                final_ee = self._robot.get_ee_position()
                self._robot.move_to(_retract(list(final_ee), self._travel_dist),
                                    self._draw_speed, TRAVEL_ACCEL)

        except Exception as exc:
            print(f"[executor] error during path execution: {exc}")
            with self._state_lock:
                self._state["executing"]  = False
                self._state["phase"]      = "error"
                self._state["progress"]   = 0.0
                self._state["exec_error"] = str(exc)   # surfaced in the browser header
            return

        with self._state_lock:
            self._state["executing"] = False
            self._state["phase"]     = "captured" if cancelled else "done"
            self._state["progress"]  = 0.0 if cancelled else 1.0

        print(f"[executor] {'cancelled' if cancelled else 'done'}")

    def _movep_draw_stroke(self, stroke: list[list[float]]) -> None:
        """
        Draw one stroke as a blended ``movep`` process move — the same actuation
        the saved path.script produces. Mirrors the export sequence: moveL onto
        the first waypoint, then movep through the rest at DRAW_SPEED with the
        run's corner blend radius. draw_z is baked into every waypoint's Z.
        """
        waypoints = [[p[0], p[1], p[2] + self._draw_z] + p[3:] for p in stroke]
        if not waypoints:
            return

        # Land on the stroke start (export does movel to s[0] before movep).
        self._robot.move_to(waypoints[0], self._draw_speed, TRAVEL_ACCEL)

        if len(waypoints) > 1:
            self._robot.move_process_path(
                waypoints[1:], self._draw_speed, DRAW_ACCEL,
                stroke_blend(waypoints, self._blend_m),
                self._cancel_event,
            )

    def _update_progress(self, done: int, total: int) -> None:
        if total <= 0:
            return
        with self._state_lock:
            self._state["progress"] = min(done / total, 1.0)
