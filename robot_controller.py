import socket
import threading
import time
from typing import Optional

from config import (
    START_JOINT_ANGLES, START_SPEED, START_ACCEL,
)

_RTDE_PORT = 30004


def _check_rtde_port(ip: str, timeout: float = 3.0) -> None:
    """Fast TCP probe — raises OSError/TimeoutError if port 30004 is unreachable."""
    try:
        with socket.create_connection((ip, _RTDE_PORT), timeout=timeout):
            pass
    except socket.timeout:
        raise TimeoutError(
            f"No response from {ip}:{_RTDE_PORT} after {timeout}s. "
            "Check the robot is powered on and Remote Control is enabled "
            "(Teach pendant → Settings → System → Remote Control)."
        )
    except ConnectionRefusedError:
        raise ConnectionRefusedError(
            f"Port {_RTDE_PORT} refused on {ip}. "
            "The RTDE interface may be disabled or the robot is not in Remote Control mode."
        )
    except OSError as exc:
        raise OSError(f"Cannot reach {ip}:{_RTDE_PORT} — {exc}") from exc


class RobotController:
    """
    Thin wrapper around ur_rtde.
    All public methods are thread-safe via an internal lock.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._rtde_c = None
        self._rtde_r = None
        self._ip: Optional[str] = None

    @property
    def connected(self) -> bool:
        return self._rtde_c is not None

    def connect(self, ip: str) -> None:
        from rtde_control import RTDEControlInterface
        from rtde_receive import RTDEReceiveInterface

        _check_rtde_port(ip)

        rtde_c = RTDEControlInterface(ip)
        rtde_r = RTDEReceiveInterface(ip)

        with self._lock:
            if self._rtde_c is not None:
                self._disconnect_unlocked()
            self._rtde_c = rtde_c
            self._rtde_r = rtde_r
            self._ip = ip

    def disconnect(self) -> None:
        with self._lock:
            self._disconnect_unlocked()

    def _disconnect_unlocked(self) -> None:
        if self._rtde_c:
            try:
                self._rtde_c.stopL(2.0)
            except Exception:
                pass
            try:
                self._rtde_c.disconnect()
            except Exception:
                pass
            self._rtde_c = None
        if self._rtde_r:
            try:
                self._rtde_r.disconnect()
            except Exception:
                pass
            self._rtde_r = None
        self._ip = None

    def move_to_start(self) -> None:
        with self._lock:
            if self._rtde_c is None:
                return
            self._rtde_c.moveJ(START_JOINT_ANGLES, START_SPEED, START_ACCEL)

    def move_to(self, pose_vec: list[float], speed: float, accel: float) -> None:
        """Blocking moveL. Called from the path executor thread."""
        with self._lock:
            if self._rtde_c is None:
                return
            self._rtde_c.moveL(pose_vec, speed, accel)

    def move_process_path(self, waypoints: list[list[float]], speed: float,
                          accel: float, blend: float,
                          cancel_event: Optional[threading.Event] = None,
                          poll_dt: float = 0.02) -> None:
        """
        Draw a stroke as a blended process move — URScript ``movep`` — matching
        the saved path.script actuation exactly (linear, constant tool speed,
        corner-blended). The path is launched asynchronously on the controller
        and polled to completion so the lock is released between checks: the EE
        poller keeps updating and cancel() stays responsive mid-stroke.

        ``blend`` (m) is applied at every waypoint except the last (0.0) and must
        be smaller than half the spacing between neighbouring waypoints, or the
        controller rejects the path. Each waypoint is [x, y, z, rx, ry, rz].
        """
        from rtde_control import Path, PathEntry

        if not waypoints:
            return

        with self._lock:
            if self._rtde_c is None:
                return
            path = Path()
            last = len(waypoints) - 1
            for i, p in enumerate(waypoints):
                r = 0.0 if i == last else blend
                path.addEntry(PathEntry(
                    PathEntry.MoveP, PathEntry.PositionTcpPose,
                    [p[0], p[1], p[2], p[3], p[4], p[5], speed, accel, r],
                ))
            self._rtde_c.movePath(path, True)   # asynchronous — returns at once

        # Poll to completion outside the lock. getAsyncOperationProgress() is
        # >= 0 while the move runs and negative once it finishes — but it can
        # still read negative for a moment right after launch, so only trust a
        # negative reading once the op has been seen running (or after a short
        # grace period, which also covers very short paths finishing between
        # polls).
        started = time.monotonic()
        seen_running = False
        while True:
            if cancel_event is not None and cancel_event.is_set():
                self.stop_motion()
                return
            with self._lock:
                if self._rtde_c is None:
                    return
                progress = self._rtde_c.getAsyncOperationProgress()
            if progress >= 0:
                seen_running = True
            elif seen_running or (time.monotonic() - started) > 0.5:
                return
            time.sleep(poll_dt)

    def stop_motion(self) -> None:
        """Stop any in-progress moveL or movePath (movep) motion."""
        with self._lock:
            if self._rtde_c is None:
                return
            try:
                self._rtde_c.stopL(2.0)
            except Exception:
                pass

    def get_ee_position(self) -> list[float]:
        with self._lock:
            if self._rtde_r is None:
                return [0.0] * 6
            return list(self._rtde_r.getActualTCPPose())

    def start_freedrive(self) -> None:
        with self._lock:
            if self._rtde_c is None:
                return
            self._rtde_c.freedriveMode()

    def end_freedrive(self) -> None:
        with self._lock:
            if self._rtde_c is None:
                return
            try:
                self._rtde_c.endFreedrive()
            except Exception:
                pass
            try:
                self._rtde_c.reuploadScript()
            except Exception:
                pass
