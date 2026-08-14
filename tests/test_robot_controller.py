"""
Unit tests for robot_controller.py — the ABB GoFa link, with compas_rrc and
roslibpy mocked. No robot, no ROS bridge, no Docker required.

The interesting part of this module is the unit seam: everything above it is
metres + an axis-angle rotation vector, RAPID wants millimetres + a frame, and
getting that wrong is a robot moving to the wrong place by a factor of 1000.
"""
import math
import threading
import time
from contextlib import contextmanager
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from config import RRC_ACCEL_PCT, RRC_ACCEL_RAMP_PCT
from robot_controller import RobotController, frame_to_pose, pose_to_frame

_POSE = [0.1, 0.2, 0.3, 0.0, math.pi, 0.0]
_WAYPOINTS = [[0.1, 0.2, 0.3, 0.0, math.pi, 0.0],
              [0.15, 0.2, 0.3, 0.0, math.pi, 0.0]]


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _future(done=True):
    """Stand-in for compas_rrc.FutureResult."""
    f = MagicMock()
    f.done = done
    f.event = threading.Event()
    if done:
        f.event.set()
    return f


def _make_rrc():
    """A mock compas_rrc module that records the instructions sent."""
    rrc = MagicMock()
    rrc.Zone.FINE = -1
    rrc.Motion.LINEAR = "L"
    rrc.FeedbackLevel.DONE = 1
    rrc.FeedbackLevel.NONE = 0
    # Instruction constructors just record their arguments.
    for name in ("Noop", "SetTool", "SetWorkObject", "SetAcceleration",
                 "SetDigital", "MoveToFrame", "MoveToJoints", "GetFrame",
                 "Stop"):
        getattr(rrc, name).side_effect = (
            lambda *a, _n=name, **k: {"instruction": _n, "args": a, "kwargs": k})
    return rrc


@contextmanager
def _connected_robot(host="127.0.0.1"):
    """Yield a connected RobotController plus its mocked rrc module and client."""
    rrc = _make_rrc()
    abb = MagicMock()
    abb.send.return_value = _future()
    abb.send_and_wait.return_value = True
    rrc.AbbClient.return_value = abb

    ros = MagicMock()
    ros.is_connected = True
    roslibpy = MagicMock()
    roslibpy.Ros.return_value = ros

    with patch.dict("sys.modules", {"compas_rrc": rrc, "roslibpy": roslibpy}):
        rc = RobotController()
        rc.connect(host)
        yield rc, rrc, abb, ros


def _sent_instructions(abb, kind):
    """Every instruction of one kind that reached the client, in order."""
    out = []
    for mock_call in list(abb.send.call_args_list) + list(abb.send_and_wait.call_args_list):
        arg = mock_call[0][0] if mock_call[0] else None
        if isinstance(arg, dict) and arg.get("instruction") == kind:
            out.append(arg)
    return out


# ─────────────────────────────────────────────────────────────────────────────
# The unit seam: metres/rotation-vector ⇄ millimetres/frame
# ─────────────────────────────────────────────────────────────────────────────

class TestPoseFrameConversion:

    def test_metres_become_millimetres(self):
        frame = pose_to_frame([0.42, -0.13, 0.255, 0.0, math.pi, 0.0])
        assert [round(v, 6) for v in frame.point] == [420.0, -130.0, 255.0]

    def test_round_trip_is_exact(self):
        pose = [0.42, -0.13, 0.255, 0.31, 2.88, -0.42]
        back = frame_to_pose(pose_to_frame(pose))
        assert back == pytest.approx(pose, abs=1e-12)

    def test_frame_axes_are_orthonormal(self):
        frame = pose_to_frame([0.1, 0.2, 0.3, 0.31, 2.88, -0.42])
        x, y, z = np.array(list(frame.xaxis)), np.array(list(frame.yaxis)), np.array(list(frame.zaxis))
        for v in (x, y, z):
            assert np.linalg.norm(v) == pytest.approx(1.0)
        assert np.dot(x, y) == pytest.approx(0.0, abs=1e-12)
        assert np.cross(x, y) == pytest.approx(z, abs=1e-12)

    def test_tool_down_pose_points_z_down(self):
        # [0, π, 0] is the classic tool-down orientation the pipeline uses.
        frame = pose_to_frame([0.0, 0.0, 0.0, 0.0, math.pi, 0.0])
        assert list(frame.zaxis) == pytest.approx([0.0, 0.0, -1.0], abs=1e-12)


# ─────────────────────────────────────────────────────────────────────────────
# Connection state
# ─────────────────────────────────────────────────────────────────────────────

class TestConnectionState:

    def test_not_connected_before_connect(self):
        assert RobotController().connected is False

    def test_connected_after_connect(self):
        with _connected_robot() as (rc, _, __, ___):
            assert rc.connected is True

    def test_not_connected_after_disconnect(self):
        with _connected_robot() as (rc, _, __, ___):
            rc.disconnect()
            assert rc.connected is False

    def test_connect_pings_the_rrc_task(self):
        # A live websocket does not prove the RAPID task is running; the Noop
        # round trip is what actually tests the chain.
        with _connected_robot() as (rc, rrc, abb, _):
            assert _sent_instructions(abb, "Noop")

    def test_connect_sets_tool_and_work_object(self):
        with _connected_robot() as (rc, rrc, abb, _):
            assert _sent_instructions(abb, "SetTool")
            assert _sent_instructions(abb, "SetWorkObject")

    def test_connect_sets_acceleration(self):
        # RAPID acceleration is a controller-wide setting, so connect is the
        # only place it can be applied — and it must be applied every time,
        # because it persists on the controller between sessions.
        with _connected_robot() as (rc, rrc, abb, _):
            (acc,) = _sent_instructions(abb, "SetAcceleration")
            assert acc["args"] == (RRC_ACCEL_PCT, RRC_ACCEL_RAMP_PCT)

    def test_acceleration_is_a_percentage_not_metres_per_second_squared(self):
        # Guards against someone "helpfully" passing DRAW_ACCEL (0.3 m/s²)
        # through: 0.3 would be read as 0.3% and the arm would barely move.
        assert 1.0 <= RRC_ACCEL_PCT <= 100.0
        assert 1.0 <= RRC_ACCEL_RAMP_PCT <= 100.0

    def test_no_bridge_raises_with_a_docker_hint(self):
        roslibpy = MagicMock()
        ros = MagicMock()
        ros.is_connected = False
        ros.run.side_effect = Exception("connection refused")
        roslibpy.Ros.return_value = ros
        with patch.dict("sys.modules", {"compas_rrc": _make_rrc(), "roslibpy": roslibpy}):
            with pytest.raises(ConnectionError) as exc:
                RobotController().connect("127.0.0.1")
        assert "docker" in str(exc.value).lower()

    def test_silent_bridge_still_raises(self):
        # ros.run() returning without error but never connecting.
        roslibpy = MagicMock()
        ros = MagicMock()
        ros.is_connected = False
        roslibpy.Ros.return_value = ros
        with patch.dict("sys.modules", {"compas_rrc": _make_rrc(), "roslibpy": roslibpy}):
            with pytest.raises(ConnectionError):
                RobotController().connect("127.0.0.1")

    def test_dead_rrc_task_raises_and_closes_the_bridge(self):
        rrc = _make_rrc()
        abb = MagicMock()
        abb.send_and_wait.side_effect = Exception("no feedback")
        rrc.AbbClient.return_value = abb
        ros = MagicMock()
        ros.is_connected = True
        roslibpy = MagicMock()
        roslibpy.Ros.return_value = ros
        with patch.dict("sys.modules", {"compas_rrc": rrc, "roslibpy": roslibpy}):
            with pytest.raises(ConnectionError) as exc:
                RobotController().connect("127.0.0.1")
        assert "RRC" in str(exc.value)
        assert ros.close.called          # no dangling websocket

    def test_reconnect_closes_the_previous_bridge(self):
        with _connected_robot() as (rc, _, __, first_ros):
            rrc2 = _make_rrc()
            abb2 = MagicMock()
            abb2.send.return_value = _future()
            rrc2.AbbClient.return_value = abb2
            ros2 = MagicMock()
            ros2.is_connected = True
            roslibpy2 = MagicMock()
            roslibpy2.Ros.return_value = ros2
            with patch.dict("sys.modules", {"compas_rrc": rrc2, "roslibpy": roslibpy2}):
                rc.connect("10.0.0.2")
            assert rc.connected is True
            assert first_ros.close.called


# ─────────────────────────────────────────────────────────────────────────────
# move_to
# ─────────────────────────────────────────────────────────────────────────────

class TestMoveTo:

    def test_sends_a_linear_move_in_mm_per_second(self):
        with _connected_robot() as (rc, rrc, abb, _):
            rc.move_to(_POSE, 0.05, 0.3)
            (move,) = _sent_instructions(abb, "MoveToFrame")
            frame, speed, zone, motion = move["args"]
            assert [round(v, 6) for v in frame.point] == [100.0, 200.0, 300.0]
            assert speed == pytest.approx(50.0)      # 0.05 m/s → 50 mm/s
            assert zone == rrc.Zone.FINE             # a single move lands exactly
            assert motion == rrc.Motion.LINEAR

    def test_silent_when_not_connected(self):
        RobotController().move_to(_POSE, 0.05, 0.3)   # must not raise


# ─────────────────────────────────────────────────────────────────────────────
# move_process_path — one stroke as a zone-blended run
# ─────────────────────────────────────────────────────────────────────────────

class TestMoveProcessPath:

    def test_blends_every_corner_but_lands_exactly_on_the_last_point(self):
        with _connected_robot() as (rc, rrc, abb, _):
            rc.move_process_path(_WAYPOINTS * 2, 0.05, 0.3, 0.0005)
            moves = _sent_instructions(abb, "MoveToFrame")
            zones = [m["args"][2] for m in moves]
            assert zones[-1] == rrc.Zone.FINE
            assert all(z != rrc.Zone.FINE for z in zones[:-1])

    def test_blend_metres_become_zone_millimetres(self):
        with _connected_robot() as (rc, rrc, abb, _):
            rc.move_process_path(_WAYPOINTS * 2, 0.05, 0.3, 0.005)   # 5 mm
            moves = _sent_instructions(abb, "MoveToFrame")
            assert moves[0]["args"][2] == 5

    def test_only_the_last_move_asks_for_feedback(self):
        # Waiting on every point would stop the arm between waypoints and
        # defeat the blending the zone data exists to produce.
        with _connected_robot() as (rc, rrc, abb, _):
            rc.move_process_path(_WAYPOINTS * 2, 0.05, 0.3, 0.0005)
            levels = [m["kwargs"]["feedback_level"]
                      for m in _sent_instructions(abb, "MoveToFrame")]
            assert levels[-1] == rrc.FeedbackLevel.DONE
            assert all(lv == rrc.FeedbackLevel.NONE for lv in levels[:-1])

    def test_cancel_event_stops_the_robot(self):
        with _connected_robot() as (rc, rrc, abb, _):
            abb.send.return_value = _future(done=False)   # never finishes
            cancel = threading.Event()
            cancel.set()
            rc.move_process_path(_WAYPOINTS, 0.05, 0.3, 0.0005, cancel)
            assert _sent_instructions(abb, "Stop")

    def test_silent_when_not_connected(self):
        RobotController().move_process_path(_WAYPOINTS, 0.05, 0.3, 0.0005)

    def test_empty_waypoints_is_a_noop(self):
        with _connected_robot() as (rc, rrc, abb, _):
            abb.send.reset_mock()
            rc.move_process_path([], 0.05, 0.3, 0.0005)
            assert not _sent_instructions(abb, "MoveToFrame")


# ─────────────────────────────────────────────────────────────────────────────
# stop_motion / disconnect
# ─────────────────────────────────────────────────────────────────────────────

class TestStopMotion:

    def test_sends_stop(self):
        with _connected_robot() as (rc, rrc, abb, _):
            rc.stop_motion()
            assert _sent_instructions(abb, "Stop")

    def test_silent_when_not_connected(self):
        RobotController().stop_motion()

    def test_tolerates_a_failing_stop(self):
        with _connected_robot() as (rc, rrc, abb, _):
            abb.send.side_effect = RuntimeError("bridge gone")
            rc.stop_motion()          # must not raise


class TestDisconnect:

    def test_stops_before_closing_the_bridge(self):
        with _connected_robot() as (rc, rrc, abb, ros):
            rc.disconnect()
            assert _sent_instructions(abb, "Stop")
            assert ros.close.called

    def test_tolerates_a_failing_stop(self):
        with _connected_robot() as (rc, rrc, abb, ros):
            abb.send.side_effect = RuntimeError("bridge gone")
            rc.disconnect()           # must not raise
            assert rc.connected is False


# ─────────────────────────────────────────────────────────────────────────────
# Material dispenser (RAPID digital output)
# ─────────────────────────────────────────────────────────────────────────────

@contextmanager
def _dispenser_on(signal="doDispenser"):
    """Pretend the valve is wired and its output named."""
    with patch("robot_controller.DISPENSER_ENABLED", True), \
         patch("robot_controller.DISPENSER_SIGNAL", signal):
        yield


class TestDispenser:

    def test_disabled_by_default_sends_nothing(self):
        # The valve does not exist on the rig yet; until DISPENSER_ENABLED is
        # turned on, not one instruction may reach the controller.
        with _connected_robot() as (rc, rrc, abb, _):
            rc.set_dispenser(True)
            assert not _sent_instructions(abb, "SetDigital")

    def test_open_sets_the_output_high(self):
        with _dispenser_on(), _connected_robot() as (rc, rrc, abb, _):
            rc.set_dispenser(True)
            (sig,) = _sent_instructions(abb, "SetDigital")
            assert sig["args"] == ("doDispenser", 1)

    def test_close_sets_the_output_low(self):
        with _dispenser_on(), _connected_robot() as (rc, rrc, abb, _):
            rc.set_dispenser(False)
            (sig,) = _sent_instructions(abb, "SetDigital")
            assert sig["args"] == ("doDispenser", 0)

    def test_blank_signal_name_sends_nothing(self):
        with _dispenser_on(signal=""), _connected_robot() as (rc, rrc, abb, _):
            rc.set_dispenser(True)
            assert not _sent_instructions(abb, "SetDigital")

    def test_silent_when_not_connected(self):
        with _dispenser_on():
            RobotController().set_dispenser(True)      # must not raise

    def test_a_failed_switch_raises(self):
        # Material failing to flow is a real failure of the drawing, so it must
        # reach the executor's error phase rather than be swallowed here.
        with _dispenser_on(), _connected_robot() as (rc, rrc, abb, _):
            abb.send_and_wait.side_effect = RuntimeError("no such signal")
            with pytest.raises(RuntimeError):
                rc.set_dispenser(True)

    def test_prime_delay_is_slept_outside_the_lock(self):
        # A pump needing a moment to build pressure must not freeze the EE
        # poller or cancel for that moment.
        with _dispenser_on(), patch("robot_controller.DISPENSER_ON_DELAY_S", 0.3), \
             _connected_robot() as (rc, rrc, abb, _):
            t = threading.Thread(target=rc.set_dispenser, args=(True,))
            t.start()
            time.sleep(0.05)                       # now inside the prime delay
            acquired = rc._lock.acquire(timeout=0.15)
            if acquired:
                rc._lock.release()
            t.join(timeout=2.0)
            assert acquired, "the prime delay must not hold the client lock"

    def test_disconnect_closes_the_valve_before_dropping_the_link(self):
        # Once the client is gone there is no way left to switch it off.
        with _dispenser_on(), _connected_robot() as (rc, rrc, abb, ros):
            rc.disconnect()
            assert _sent_instructions(abb, "SetDigital")[-1]["args"] == ("doDispenser", 0)

    def test_disconnect_sends_nothing_when_disabled(self):
        with _connected_robot() as (rc, rrc, abb, ros):
            rc.disconnect()
            assert not _sent_instructions(abb, "SetDigital")


# ─────────────────────────────────────────────────────────────────────────────
# get_ee_position
# ─────────────────────────────────────────────────────────────────────────────

class TestGetEePosition:

    def test_reads_the_frame_back_in_metres(self):
        with _connected_robot() as (rc, rrc, abb, _):
            abb.send_and_wait.return_value = pose_to_frame(_POSE)
            assert rc.get_ee_position() == pytest.approx(_POSE, abs=1e-12)

    def test_returns_zeros_when_not_connected(self):
        assert RobotController().get_ee_position() == [0.0] * 6

    def test_a_failed_read_returns_zeros_rather_than_raising(self):
        # The EE poller runs at 10 Hz; one dropped reply must not kill it.
        with _connected_robot() as (rc, rrc, abb, _):
            abb.send_and_wait.side_effect = Exception("timeout")
            assert rc.get_ee_position() == [0.0] * 6

    def test_returns_plain_floats(self):
        with _connected_robot() as (rc, rrc, abb, _):
            abb.send_and_wait.return_value = pose_to_frame(_POSE)
            result = rc.get_ee_position()
            assert isinstance(result, list)
            assert all(isinstance(v, float) for v in result)


# ─────────────────────────────────────────────────────────────────────────────
# Lead-through
# ─────────────────────────────────────────────────────────────────────────────

class TestLeadThrough:

    def test_freedrive_is_declared_not_software_controlled(self):
        # RRC has no freedrive instruction — the GoFa is hand-guided with the
        # button on the arm. The UI reads this flag to say so rather than
        # offering a toggle that does nothing.
        assert RobotController.FREEDRIVE_IS_SOFTWARE_CONTROLLED is False

    def test_the_calls_are_harmless_no_ops(self):
        with _connected_robot() as (rc, rrc, abb, _):
            abb.send.reset_mock()
            abb.send_and_wait.reset_mock()
            rc.start_freedrive()
            rc.end_freedrive()
            assert not abb.send.called and not abb.send_and_wait.called

    def test_no_ops_are_safe_while_disconnected(self):
        rc = RobotController()
        rc.start_freedrive()
        rc.end_freedrive()          # must not raise


# ─────────────────────────────────────────────────────────────────────────────
# Thread safety
# ─────────────────────────────────────────────────────────────────────────────

class TestThreadSafety:

    def test_concurrent_move_to_no_deadlock(self):
        with _connected_robot() as (rc, rrc, abb, _):
            threads = [threading.Thread(target=rc.move_to, args=(_POSE, 0.05, 0.3))
                       for _ in range(10)]
            for t in threads:
                t.start()
            for t in threads:
                t.join(timeout=2.0)
                assert not t.is_alive(), "Thread did not complete — possible deadlock"
            assert len(_sent_instructions(abb, "MoveToFrame")) == 10

    def test_concurrent_get_and_move_no_deadlock(self):
        with _connected_robot() as (rc, rrc, abb, _):
            abb.send_and_wait.return_value = pose_to_frame(_POSE)
            errors = []

            def getter():
                for _ in range(20):
                    try:
                        rc.get_ee_position()
                    except Exception as e:      # pragma: no cover
                        errors.append(e)

            def mover():
                for _ in range(20):
                    try:
                        rc.move_to(_POSE, 0.05, 0.3)
                    except Exception as e:      # pragma: no cover
                        errors.append(e)

            threads = ([threading.Thread(target=getter) for _ in range(5)]
                       + [threading.Thread(target=mover) for _ in range(5)])
            for t in threads:
                t.start()
            for t in threads:
                t.join(timeout=3.0)
                assert not t.is_alive()
            assert errors == []
