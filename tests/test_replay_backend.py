"""
Unit tests for replay_robot.py — the brand abstraction of the replay tool.
Robot is mocked; verifies saved waypoints are executed literally. No hardware.
"""
import math
import threading
import time
from unittest.mock import Mock

import pytest

from replay_robot import URReplayBackend, make_backend

_PI = math.pi

STROKES = [
    [[0.4, 0.0, 0.2, 0.0, _PI, 0.0], [0.45, 0.0, 0.2, 0.0, _PI, 0.0]],
    [[0.4, 0.1, 0.2, 0.0, _PI, 0.0], [0.4, 0.15, 0.2, 0.0, _PI, 0.0],
     [0.4, 0.2, 0.2, 0.0, _PI, 0.0]],
]


def _backend():
    state = {"executing": False, "phase": "idle", "progress": 0.0,
             "exec_error": None}
    robot = Mock()
    robot.connected = True
    robot.get_ee_position.return_value = [0.0] * 6
    return URReplayBackend(state, threading.Lock(), robot=robot), robot, state


def _wait_done(backend, timeout=3.0):
    t0 = time.monotonic()
    while backend.running and time.monotonic() - t0 < timeout:
        time.sleep(0.01)
    assert not backend.running, "replay run did not finish in time"


class TestURReplayBackend:

    def test_run_executes_saved_waypoints_literally(self):
        backend, robot, state = _backend()
        backend.run(STROKES, speed_mps=0.1, safety_m=0.05, blend_m=0.003)
        _wait_done(backend)
        assert state["phase"] == "done"

        calls = robot.move_process_path.call_args_list
        assert len(calls) == len(STROKES)
        # Waypoints verbatim (no draw-Z, no offset) — draw part = stroke[1:].
        assert calls[0][0][0][0] == pytest.approx(STROKES[0][1])
        assert calls[1][0][0] == [pytest.approx(p) for p in STROKES[1][1:]]
        # Blend forwarded (50 mm segments — no clamp at 3 mm).
        assert calls[0][0][3] == pytest.approx(0.003)

    def test_connected_and_disconnect_delegate(self):
        backend, robot, _ = _backend()
        assert backend.connected is True
        robot.connected = False
        assert backend.connected is False
        backend.disconnect()
        robot.disconnect.assert_called_once()

    def test_cancel_stops_motion(self):
        backend, robot, _ = _backend()
        backend.cancel()
        robot.stop_motion.assert_called_once()


class TestMakeBackend:

    def test_ur(self):
        be = make_backend("ur", {}, threading.Lock())
        assert isinstance(be, URReplayBackend)

    def test_abb_gofa_documented_stub(self):
        with pytest.raises(NotImplementedError, match="ReplayBackend"):
            make_backend("abb_gofa", {}, threading.Lock())

    def test_unknown_raises(self):
        with pytest.raises(ValueError, match="REPLAY_BACKEND"):
            make_backend("kuka", {}, threading.Lock())
