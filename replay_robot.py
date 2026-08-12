"""
replay_robot.py — robot-brand abstraction for the saved-toolpath replay tool.

The replay server/UI only ever talk to the ReplayBackend interface, so porting
to another robot brand touches nothing else. The shipped backend is the ABB
GoFa 10 over compas_rrc; it reuses the main app's RobotController and
PathExecutor, so a replayed path.json actuates exactly like a live run.

To add another brand:

  1. Subclass ReplayBackend and implement connect/disconnect/run/cancel plus
     the connected/running properties.
  2. Convert poses at the boundary. Everything above this module is metres +
     an axis-angle rotation vector; RobotController.pose_to_frame shows the
     conversion the ABB backend uses, and path.json's per-waypoint ``plane``
     (origin + x/y/z axes) is already a frame if that suits the target better.
  3. Register the class in ``make_backend()`` and set REPLAY_BACKEND to your
     key in config.py.

Progress reporting: a backend writes ``executing`` (bool), ``phase``
("executing" | "done" | "captured"=cancelled | "error"), ``progress`` (0..1)
and ``exec_error`` into the shared_state dict it was constructed with — the
replay server broadcasts those keys to the browser.
"""
from __future__ import annotations

import threading
from abc import ABC, abstractmethod

from path_executor import PathExecutor
from robot_controller import RobotController
from toolpath_loader import Strokes


class ReplayBackend(ABC):
    """What the replay server needs from any robot brand."""

    name: str = "?"

    @abstractmethod
    def connect(self, ip: str) -> None:
        """Blocking connect; raise with a readable message on failure."""

    @abstractmethod
    def disconnect(self) -> None: ...

    @property
    @abstractmethod
    def connected(self) -> bool: ...

    @abstractmethod
    def run(self, strokes: Strokes, speed_mps: float, safety_m: float,
            blend_m: float) -> None:
        """
        Execute the saved waypoints LITERALLY (no draw-Z, no normal offset —
        both were baked in when the bundle was saved), retracting ``safety_m``
        along the tool axis around each stroke. Non-blocking: progress goes to
        shared_state (see module docstring).
        """

    @property
    @abstractmethod
    def running(self) -> bool: ...

    @abstractmethod
    def cancel(self) -> None: ...


class ABBGofaBackend(ReplayBackend):
    """ABB GoFa 10 via compas_rrc — reuses the main app's RobotController +
    PathExecutor, so a replayed path.json actuates exactly like a live run."""

    name = "ABB GoFa 10 (compas_rrc)"

    def __init__(self, shared_state: dict, state_lock: threading.Lock,
                 robot: RobotController | None = None) -> None:
        self._robot = robot if robot is not None else RobotController()
        self._executor = PathExecutor(self._robot, shared_state, state_lock)

    def connect(self, ip: str) -> None:
        """``ip`` is the ROS bridge host, not the robot — see RobotController."""
        self._robot.connect(ip)

    def disconnect(self) -> None:
        if self.running:
            self.cancel()
        self._robot.disconnect()

    @property
    def connected(self) -> bool:
        return self._robot.connected

    def run(self, strokes: Strokes, speed_mps: float, safety_m: float,
            blend_m: float) -> None:
        self._executor.start(
            strokes,
            draw_z=0.0,               # saved poses are final — execute verbatim
            draw_speed=speed_mps,
            normal_offset=0.0,
            travel_dist=safety_m,
            blend_m=blend_m,
        )

    @property
    def running(self) -> bool:
        return self._executor.running

    def cancel(self) -> None:
        self._executor.cancel()


def make_backend(kind: str, shared_state: dict,
                 state_lock: threading.Lock) -> ReplayBackend:
    """Backend factory — config.REPLAY_BACKEND selects the brand."""
    if kind == "abb_gofa":
        return ABBGofaBackend(shared_state, state_lock)
    raise ValueError(f"unknown REPLAY_BACKEND {kind!r} (known: 'abb_gofa')")
