"""
Unit tests for robot_controller.py — RobotController with mocked RTDE interfaces.
No physical robot required.
"""
import socket
import threading
from contextlib import contextmanager
from unittest.mock import MagicMock, patch, call

import pytest

from robot_controller import RobotController, _check_rtde_port


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _make_mock_rtde():
    rtde_c = MagicMock()
    rtde_r = MagicMock()
    rtde_r.getActualTCPPose.return_value = [0.1, 0.2, 0.3, 0.0, 3.14159, 0.0]
    return rtde_c, rtde_r


@contextmanager
def _connected_robot(ip="192.168.1.100"):
    """Context manager: yield a connected RobotController with mocked RTDE."""
    rtde_c, rtde_r = _make_mock_rtde()
    rtde_control_mod = MagicMock()
    rtde_receive_mod = MagicMock()
    rtde_control_mod.RTDEControlInterface.return_value = rtde_c
    rtde_receive_mod.RTDEReceiveInterface.return_value = rtde_r

    with patch("robot_controller._check_rtde_port"), \
         patch.dict("sys.modules", {
             "rtde_control": rtde_control_mod,
             "rtde_receive": rtde_receive_mod,
         }):
        rc = RobotController()
        rc.connect(ip)
        yield rc, rtde_c, rtde_r


# ─────────────────────────────────────────────────────────────────────────────
# Connection state
# ─────────────────────────────────────────────────────────────────────────────

class TestConnectionState:

    def test_not_connected_before_connect(self):
        rc = RobotController()
        assert rc.connected is False

    def test_connected_after_connect(self):
        with _connected_robot() as (rc, _, __):
            assert rc.connected is True

    def test_not_connected_after_disconnect(self):
        with _connected_robot() as (rc, _, __):
            rc.disconnect()
            assert rc.connected is False

    def test_connect_calls_port_check(self):
        rtde_c, rtde_r = _make_mock_rtde()
        rtde_control_mod = MagicMock(RTDEControlInterface=MagicMock(return_value=rtde_c))
        rtde_receive_mod = MagicMock(RTDEReceiveInterface=MagicMock(return_value=rtde_r))
        with patch("robot_controller._check_rtde_port") as mock_check, \
             patch.dict("sys.modules", {
                 "rtde_control": rtde_control_mod,
                 "rtde_receive": rtde_receive_mod,
             }):
            rc = RobotController()
            rc.connect("10.0.0.1")
            mock_check.assert_called_once_with("10.0.0.1")

    def test_reconnect_disconnects_existing(self):
        with _connected_robot() as (rc, rtde_c, rtde_r):
            first_rtde_c = rtde_c
            # Connect again — should close the first connection
            rtde_c2, rtde_r2 = _make_mock_rtde()
            rtde_control_mod2 = MagicMock(RTDEControlInterface=MagicMock(return_value=rtde_c2))
            rtde_receive_mod2 = MagicMock(RTDEReceiveInterface=MagicMock(return_value=rtde_r2))
            with patch("robot_controller._check_rtde_port"), \
                 patch.dict("sys.modules", {
                     "rtde_control": rtde_control_mod2,
                     "rtde_receive": rtde_receive_mod2,
                 }):
                rc.connect("10.0.0.2")
            assert rc.connected is True
            # Original RTDE was disconnected
            assert first_rtde_c.disconnect.called


# ─────────────────────────────────────────────────────────────────────────────
# move_to
# ─────────────────────────────────────────────────────────────────────────────

class TestMoveTo:

    def test_calls_moveL_with_correct_args(self):
        with _connected_robot() as (rc, rtde_c, _):
            pose = [0.1, 0.2, 0.3, 0.0, 3.14, 0.0]
            rc.move_to(pose, 0.05, 0.3)
            rtde_c.moveL.assert_called_once_with(pose, 0.05, 0.3)

    def test_silent_when_not_connected(self):
        rc = RobotController()
        rc.move_to([0, 0, 0, 0, 0, 0], 0.05, 0.3)  # must not raise


# ─────────────────────────────────────────────────────────────────────────────
# move_process_path (movep drawing)
# ─────────────────────────────────────────────────────────────────────────────

_WAYPOINTS = [[0.1, 0.2, 0.3, 0.0, 3.14, 0.0],
              [0.15, 0.2, 0.3, 0.0, 3.14, 0.0]]


class TestMoveProcessPath:

    def test_launches_async_move_path_and_polls_to_completion(self):
        with _connected_robot() as (rc, rtde_c, _):
            rtde_c.getAsyncOperationProgress.return_value = -1   # finished
            rc.move_process_path(_WAYPOINTS, 0.05, 0.3, 0.0005)
            rtde_c.movePath.assert_called_once()
            # asynchronous=True so the lock is released while polling
            assert rtde_c.movePath.call_args[0][1] is True
            assert rtde_c.getAsyncOperationProgress.called

    def test_cancel_event_stops_motion(self):
        with _connected_robot() as (rc, rtde_c, _):
            rtde_c.getAsyncOperationProgress.return_value = 10   # still running
            cancel = threading.Event()
            cancel.set()
            rc.move_process_path(_WAYPOINTS, 0.05, 0.3, 0.0005, cancel)
            rtde_c.stopL.assert_called_once_with(2.0)

    def test_silent_when_not_connected(self):
        rc = RobotController()
        rc.move_process_path(_WAYPOINTS, 0.05, 0.3, 0.0005)  # must not raise

    def test_empty_waypoints_is_noop(self):
        with _connected_robot() as (rc, rtde_c, _):
            rc.move_process_path([], 0.05, 0.3, 0.0005)
            rtde_c.movePath.assert_not_called()


# ─────────────────────────────────────────────────────────────────────────────
# stop_motion
# ─────────────────────────────────────────────────────────────────────────────

class TestStopMotion:

    def test_calls_stopL_with_deceleration_2s(self):
        with _connected_robot() as (rc, rtde_c, _):
            rc.stop_motion()
            rtde_c.stopL.assert_called_once_with(2.0)

    def test_silent_when_not_connected(self):
        rc = RobotController()
        rc.stop_motion()  # must not raise

    def test_tolerates_stopL_exception(self):
        with _connected_robot() as (rc, rtde_c, _):
            rtde_c.stopL.side_effect = RuntimeError("already stopped")
            rc.stop_motion()  # must not raise


# ─────────────────────────────────────────────────────────────────────────────
# get_ee_position
# ─────────────────────────────────────────────────────────────────────────────

class TestGetEePosition:

    def test_returns_list_of_six_floats(self):
        with _connected_robot() as (rc, _, rtde_r):
            result = rc.get_ee_position()
            assert isinstance(result, list)
            assert len(result) == 6

    def test_returns_zeros_when_not_connected(self):
        rc = RobotController()
        assert rc.get_ee_position() == [0.0] * 6

    def test_converts_to_python_list(self):
        with _connected_robot() as (rc, _, rtde_r):
            import numpy as np
            rtde_r.getActualTCPPose.return_value = np.array([0.1, 0.2, 0.3, 0.0, 3.14, 0.0])
            result = rc.get_ee_position()
            assert isinstance(result, list)  # must not be numpy array


# ─────────────────────────────────────────────────────────────────────────────
# disconnect
# ─────────────────────────────────────────────────────────────────────────────

class TestDisconnect:

    def test_calls_stopL_before_disconnect(self):
        with _connected_robot() as (rc, rtde_c, _):
            rc.disconnect()
            calls = [c[0] for c in rtde_c.method_calls]
            stopL_idx = next(i for i, c in enumerate(calls) if "stopL" in str(c))
            disc_idx  = next(i for i, c in enumerate(calls) if c == "disconnect")
            assert stopL_idx < disc_idx

    def test_tolerates_stopL_exception_on_disconnect(self):
        with _connected_robot() as (rc, rtde_c, _):
            rtde_c.stopL.side_effect = RuntimeError("already stopped")
            rc.disconnect()  # must not raise
            assert rc.connected is False


# ─────────────────────────────────────────────────────────────────────────────
# freedrive
# ─────────────────────────────────────────────────────────────────────────────

class TestFreedrive:

    def test_start_freedrive_calls_freedrive_mode(self):
        with _connected_robot() as (rc, rtde_c, _):
            rc.start_freedrive()
            rtde_c.freedriveMode.assert_called_once()

    def test_end_freedrive_calls_end_and_reupload(self):
        with _connected_robot() as (rc, rtde_c, _):
            rc.end_freedrive()
            rtde_c.endFreedrive.assert_called_once()
            rtde_c.reuploadScript.assert_called_once()

    def test_end_freedrive_tolerates_endFreedrive_exception(self):
        with _connected_robot() as (rc, rtde_c, _):
            rtde_c.endFreedrive.side_effect = RuntimeError("not in freedrive")
            rc.end_freedrive()  # must not raise; reuploadScript still called
            rtde_c.reuploadScript.assert_called_once()


# ─────────────────────────────────────────────────────────────────────────────
# Thread safety
# ─────────────────────────────────────────────────────────────────────────────

class TestThreadSafety:

    def test_concurrent_move_to_no_deadlock(self):
        with _connected_robot() as (rc, rtde_c, _):
            threads = [
                threading.Thread(target=rc.move_to, args=([0, 0, 0, 0, 0, 0], 0.05, 0.3))
                for _ in range(10)
            ]
            for t in threads:
                t.start()
            for t in threads:
                t.join(timeout=2.0)
                assert not t.is_alive(), "Thread did not complete — possible deadlock"
            assert rtde_c.moveL.call_count == 10

    def test_concurrent_get_and_move_no_deadlock(self):
        with _connected_robot() as (rc, rtde_c, _):
            errors = []

            def getter():
                for _ in range(20):
                    try:
                        rc.get_ee_position()
                    except Exception as e:
                        errors.append(e)

            def mover():
                for _ in range(20):
                    try:
                        rc.move_to([0, 0, 0, 0, 0, 0], 0.05, 0.3)
                    except Exception as e:
                        errors.append(e)

            threads = [threading.Thread(target=getter) for _ in range(5)] + \
                      [threading.Thread(target=mover) for _ in range(5)]
            for t in threads:
                t.start()
            for t in threads:
                t.join(timeout=3.0)
                assert not t.is_alive()
            assert errors == []


# ─────────────────────────────────────────────────────────────────────────────
# _check_rtde_port
# ─────────────────────────────────────────────────────────────────────────────

class TestCheckRtdePort:

    def test_timeout_raises_timeout_error_with_hint(self):
        with patch("robot_controller.socket.create_connection",
                   side_effect=socket.timeout()):
            with pytest.raises(TimeoutError) as exc_info:
                _check_rtde_port("10.0.0.1")
            assert "Remote Control" in str(exc_info.value)

    def test_refused_raises_connection_refused_with_hint(self):
        with patch("robot_controller.socket.create_connection",
                   side_effect=ConnectionRefusedError()):
            with pytest.raises(ConnectionRefusedError) as exc_info:
                _check_rtde_port("10.0.0.1")
            assert "Remote Control" in str(exc_info.value)

    def test_os_error_raises_os_error(self):
        with patch("robot_controller.socket.create_connection",
                   side_effect=OSError("Network unreachable")):
            with pytest.raises(OSError):
                _check_rtde_port("10.0.0.1")

    def test_success_returns_none(self):
        mock_conn = MagicMock()
        mock_conn.__enter__ = MagicMock(return_value=mock_conn)
        mock_conn.__exit__ = MagicMock(return_value=False)
        with patch("robot_controller.socket.create_connection", return_value=mock_conn):
            result = _check_rtde_port("10.0.0.1")
            assert result is None
