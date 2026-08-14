"""
robot_controller.py — thread-safe ABB GoFa 10 link via compas_rrc.

compas_rrc never touches the controller directly. The chain is:

    this module  →  rosbridge websocket  →  RRC driver  →  RRC RAPID task  →  arm

so the host this connects to is the machine running the ROS bridge (normally
this PC, via Docker — see the README), NOT the robot's own IP. The RRC task
must already be running on the controller.

Units are the seam. Everything above this module is metres + an axis-angle
rotation vector (the pose convention the whole pipeline uses); RAPID wants
millimetres + a frame. Both conversions live here and nowhere else.
"""
import threading
import time
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from config import (
    DISPENSER_ENABLED, DISPENSER_OFF_DELAY_S, DISPENSER_ON_DELAY_S,
    DISPENSER_SIGNAL, RRC_ACCEL_PCT, RRC_ACCEL_RAMP_PCT, RRC_CONNECT_TIMEOUT_S,
    RRC_NAMESPACE, RRC_ROS_PORT, RRC_TOOL, RRC_WORK_OBJECT, START_JOINT_ANGLES,
    START_SPEED,
)

# RAPID zone data is a rounding radius in mm; -1 means "stop exactly here".
_ZONE_FINE = -1


def _deg(radians: float) -> float:
    """RAPID joint targets are in degrees; the rest of the app is in radians."""
    return float(np.degrees(radians))


def pose_to_frame(pose: list[float]):
    """[x, y, z, rx, ry, rz] in metres/rad → a compas Frame in millimetres."""
    from compas.geometry import Frame

    R = Rotation.from_rotvec(pose[3:6]).as_matrix()
    return Frame([pose[0] * 1000.0, pose[1] * 1000.0, pose[2] * 1000.0],
                 R[:, 0].tolist(), R[:, 1].tolist())


def frame_to_pose(frame) -> list[float]:
    """A compas Frame in millimetres → [x, y, z, rx, ry, rz] in metres/rad."""
    R = np.column_stack([list(frame.xaxis), list(frame.yaxis), list(frame.zaxis)])
    rv = Rotation.from_matrix(R).as_rotvec()
    p = list(frame.point)
    return [p[0] / 1000.0, p[1] / 1000.0, p[2] / 1000.0,
            float(rv[0]), float(rv[1]), float(rv[2])]


class RobotController:
    """
    Thin wrapper around compas_rrc's AbbClient.
    All public methods are thread-safe via an internal lock.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._ros = None
        self._abb = None
        self._host: Optional[str] = None

    @property
    def connected(self) -> bool:
        return self._abb is not None

    def connect(self, host: str) -> None:
        """
        Open the bridge and prove the RRC task answers.

        A reachable rosbridge is not the same as a working robot link: the
        websocket connects even when the RAPID task is not running. The Noop
        round trip is what actually tests the whole chain, so it is part of
        connecting rather than a later surprise mid-stroke.
        """
        import compas_rrc as rrc
        import roslibpy

        ros = roslibpy.Ros(host=host, port=RRC_ROS_PORT)
        try:
            ros.run(timeout=RRC_CONNECT_TIMEOUT_S)
        except Exception as exc:
            raise ConnectionError(
                f"No ROS bridge at {host}:{RRC_ROS_PORT} — {exc}. "
                "Start it with `docker compose up` (see the README), and check "
                "nothing else is already using the port."
            ) from exc
        if not ros.is_connected:
            raise ConnectionError(
                f"No ROS bridge at {host}:{RRC_ROS_PORT}. Start it with "
                "`docker compose up` (see the README)."
            )

        abb = rrc.AbbClient(ros, RRC_NAMESPACE)
        try:
            abb.send_and_wait(rrc.Noop(), timeout=RRC_CONNECT_TIMEOUT_S)
        except Exception as exc:
            try:
                ros.close()
            except Exception:
                pass
            raise ConnectionError(
                f"ROS bridge reached, but the RRC task on {RRC_NAMESPACE} did not "
                f"answer — {exc}. On the pendant: check the RRC RAPID program is "
                "loaded and running, and the controller is in Auto mode."
            ) from exc

        # Poses are authored in the robot base frame, so the work object must be
        # the base one; the tool is what the TCP offset was taught on.
        abb.send(rrc.SetTool(RRC_TOOL))
        abb.send(rrc.SetWorkObject(RRC_WORK_OBJECT))

        # Acceleration is a controller-wide setting (RAPID AccSet), not a
        # per-move argument — this is the only place it can be set, and it
        # applies to travel and drawing alike. Sent on every connect even at
        # 100%, because it persists on the controller: without this, a session
        # that turned it down would quietly govern the next one.
        abb.send(rrc.SetAcceleration(RRC_ACCEL_PCT, RRC_ACCEL_RAMP_PCT))

        with self._lock:
            if self._abb is not None:
                self._disconnect_unlocked()
            self._ros = ros
            self._abb = abb
            self._host = host

    def disconnect(self) -> None:
        with self._lock:
            self._disconnect_unlocked()

    def _disconnect_unlocked(self) -> None:
        if self._abb is not None:
            try:
                import compas_rrc as rrc
                # Close the valve before dropping the link, not after: once the
                # client is gone there is no way left to switch it off, and an
                # open dispenser over the sand outlasts the session.
                if DISPENSER_ENABLED and DISPENSER_SIGNAL:
                    self._abb.send(rrc.SetDigital(DISPENSER_SIGNAL, 0))
                self._abb.send(rrc.Stop())
            except Exception:
                pass
            self._abb = None
        if self._ros is not None:
            try:
                self._ros.close()
            except Exception:
                pass
            self._ros = None
        self._host = None

    def move_to_start(self) -> None:
        with self._lock:
            if self._abb is None:
                return
            import compas_rrc as rrc
            joints = [_deg(a) for a in START_JOINT_ANGLES]
            self._abb.send_and_wait(rrc.MoveToJoints(
                joints, [], START_SPEED * 1000.0, rrc.Zone.FINE))

    def move_to(self, pose_vec: list[float], speed: float, accel: float) -> None:
        """
        Blocking linear move to one pose. Called from the path executor thread.

        ``accel`` is accepted for signature compatibility with the rest of the
        pipeline but is not sent per-move: RAPID acceleration is a controller
        setting (AccSet), not a MoveL argument.
        """
        with self._lock:
            if self._abb is None:
                return
            import compas_rrc as rrc
            self._abb.send_and_wait(rrc.MoveToFrame(
                pose_to_frame(pose_vec), speed * 1000.0, rrc.Zone.FINE,
                rrc.Motion.LINEAR))

    def move_process_path(self, waypoints: list[list[float]], speed: float,
                          accel: float, blend: float,
                          cancel_event: Optional[threading.Event] = None,
                          poll_dt: float = 0.02) -> None:
        """
        Draw a stroke as one continuous blended run of linear moves.

        Every waypoint but the last carries zone data (``blend``, m → mm), which
        is what makes the controller round the corner and keep moving instead of
        decelerating to a stop on each point. The last one is FINE so the stroke
        ends exactly where it should.

        The whole stroke is queued at once and only the final move is waited on:
        RRC executes the queue on the controller, so waiting per point would
        stall the arm between waypoints and defeat the blending. Cancel is
        checked while waiting — Stop() empties the queue.
        """
        if not waypoints:
            return

        import compas_rrc as rrc

        zone_mm = max(int(round(blend * 1000.0)), 0)
        with self._lock:
            if self._abb is None:
                return
            last = len(waypoints) - 1
            done = None
            for i, p in enumerate(waypoints):
                zone = _ZONE_FINE if i == last else zone_mm
                instruction = rrc.MoveToFrame(
                    pose_to_frame(p), speed * 1000.0, zone, rrc.Motion.LINEAR,
                    feedback_level=(rrc.FeedbackLevel.DONE if i == last
                                    else rrc.FeedbackLevel.NONE))
                result = self._abb.send(instruction)
                if i == last:
                    done = result

        # Wait outside the lock so the EE poller keeps updating and cancel stays
        # responsive: this is the only long wait in a stroke. Waiting on the
        # future's own event (rather than sleeping) returns the instant the
        # controller reports the stroke finished.
        while done is not None and not done.done:
            if cancel_event is not None and cancel_event.is_set():
                self.stop_motion()
                return
            done.event.wait(poll_dt)

    def stop_motion(self) -> None:
        """Stop the current motion and drop whatever is still queued."""
        with self._lock:
            if self._abb is None:
                return
            try:
                import compas_rrc as rrc
                self._abb.send(rrc.Stop())
            except Exception:
                pass

    # ── Material dispenser ───────────────────────────────────────────────────
    def set_dispenser(self, on: bool) -> None:
        """
        Open or close the substrate dispenser's digital output.

        A no-op unless a signal is configured, so a rig without the valve wired
        sends nothing at all — which is the state the code ships in. Errors are
        NOT swallowed here: material failing to flow is a real failure of the
        drawing and belongs in the error phase. The executor is what makes
        closing safe, by doing it in a ``finally``.

        The optional delay is the time a pump needs to build (or drop) pressure
        before the arm moves; it is slept OUTSIDE the lock so the EE poller and
        cancel keep working through it.
        """
        if not DISPENSER_ENABLED or not DISPENSER_SIGNAL:
            return
        with self._lock:
            if self._abb is None:
                return
            import compas_rrc as rrc
            self._abb.send_and_wait(
                rrc.SetDigital(DISPENSER_SIGNAL, 1 if on else 0))
        delay = DISPENSER_ON_DELAY_S if on else DISPENSER_OFF_DELAY_S
        if delay > 0:
            time.sleep(delay)

    def get_ee_position(self) -> list[float]:
        with self._lock:
            if self._abb is None:
                return [0.0] * 6
            try:
                import compas_rrc as rrc
                frame = self._abb.send_and_wait(rrc.GetFrame(), timeout=5.0)
            except Exception:
                return [0.0] * 6
        return frame_to_pose(frame)

    # ── Lead-through ─────────────────────────────────────────────────────────
    # RRC has no freedrive instruction: on a GoFa the arm is hand-guided with
    # the lead-through button ON THE ARM itself, which the controller owns. So
    # these report "not software-controllable" rather than pretending. Corner
    # registration still works — the operator guides the tool by hand and the
    # touched position is read back with GetFrame like before.
    FREEDRIVE_IS_SOFTWARE_CONTROLLED = False

    def start_freedrive(self) -> None:
        return

    def end_freedrive(self) -> None:
        return
