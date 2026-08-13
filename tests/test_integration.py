"""
Integration tests — require physical hardware.
Run only when camera and/or robot are connected:
  pytest -m integration -v

Skip in CI:
  pytest -m "not integration" -v
"""
import os

import pytest


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _robot_ip():
    ip = os.environ.get("TEST_ROBOT_IP", "")
    if not ip:
        pytest.skip("TEST_ROBOT_IP env var not set")
    return ip


# ─────────────────────────────────────────────────────────────────────────────
# Camera
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.integration
def test_camera_live_feed(shared_state_and_lock):
    """
    Prerequisite: RealSense D435i connected.
    Asserts:
    - Both JPEG streams are non-None bytes within 2 seconds.
    - The colorized-depth JPEG decodes to an image of the configured dimensions.
    """
    import time
    import cv2
    import numpy as np
    from camera_thread import DepthCameraThread
    from config import DEPTH_WIDTH, DEPTH_HEIGHT

    state, lock = shared_state_and_lock
    ct = DepthCameraThread(state, lock)
    ct.start()
    time.sleep(2.0)
    # Assert before stop() — stop() clears both JPEG keys to None.
    depth_jpg  = state["last_depth_color_jpg"]
    groove_jpg = state["last_groove_jpg"]
    ct.stop()

    assert depth_jpg is not None, "colorized-depth JPEG stream never populated"
    assert groove_jpg is not None, "groove JPEG stream never populated"

    color = cv2.imdecode(np.frombuffer(depth_jpg, np.uint8), cv2.IMREAD_COLOR)
    assert color is not None
    assert color.shape[1] == DEPTH_WIDTH
    assert color.shape[0] == DEPTH_HEIGHT


@pytest.mark.integration
def test_extract_from_live_depth(shared_state_and_lock):
    """
    Prerequisite: RealSense present, grooves raked into sand in the field of view.
    Asserts:
    - grooves_from_depth + extract_from_edges produce at least one stroke.
    - All stroke points are within valid frame bounds.
    """
    import time
    from camera_thread import DepthCameraThread
    from depth_extractor import grooves_from_depth
    from path_extractor import extract_from_edges
    from config import DEPTH_WIDTH, DEPTH_HEIGHT

    state, lock = shared_state_and_lock
    ct = DepthCameraThread(state, lock)
    ct.start()
    time.sleep(2.0)
    captured = ct.capture_frame()
    ct.stop()

    assert captured is not None, "No depth captured — is the RealSense connected?"
    depth_m, valid, _rgb = captured
    grooves = grooves_from_depth(depth_m, valid)
    result = extract_from_edges(grooves)
    assert result.total_strokes > 0, "No grooves found — rake some marks into the sand"
    for stroke in result.strokes:
        for pt in stroke:
            assert 0 <= pt[0] <= DEPTH_WIDTH
            assert 0 <= pt[1] <= DEPTH_HEIGHT


# ─────────────────────────────────────────────────────────────────────────────
# Robot
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.integration
def test_robot_connect_disconnect():
    """
    Prerequisite: UR robot reachable at TEST_ROBOT_IP env var.
    Asserts:
    - connect() succeeds.
    - get_ee_position() returns 6 finite floats in plausible meter ranges.
    - disconnect() leaves connected=False.
    """
    import math
    from robot_controller import RobotController

    ip = _robot_ip()
    rc = RobotController()
    rc.connect(ip)
    assert rc.connected is True

    ee = rc.get_ee_position()
    assert isinstance(ee, list) and len(ee) == 6
    for v in ee:
        assert math.isfinite(v)
    # XYZ in meters — UR typically within [-2, 2]
    for v in ee[:3]:
        assert -2.0 <= v <= 2.0, f"EE position {v} out of expected range"

    rc.disconnect()
    assert rc.connected is False


@pytest.mark.integration
def test_robot_move_to_safe_pose():
    """
    Prerequisite: robot connected, clear workspace, operator present.
    Moves to a known safe pose and verifies EE reaches it within 1mm.
    CAUTION: only run with a clear workspace and emergency stop within reach.
    """
    import math
    from robot_controller import RobotController
    from config import TRAVEL_SPEED, TRAVEL_ACCEL

    ip = _robot_ip()
    rc = RobotController()
    rc.connect(ip)

    # Read current pose, raise only Z by TRAVEL_Z to lift the pen
    current = rc.get_ee_position()
    target = list(current)
    target[2] += 0.05  # lift 50mm

    rc.move_to(target, TRAVEL_SPEED, TRAVEL_ACCEL)
    actual = rc.get_ee_position()
    rc.disconnect()

    for i in range(3):  # XYZ tolerance
        assert abs(actual[i] - target[i]) < 0.001, (
            f"EE axis {i}: expected {target[i]:.4f}, got {actual[i]:.4f}"
        )


# ─────────────────────────────────────────────────────────────────────────────
# Full pipeline
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.integration
def test_full_pipeline_flat_sand(shared_state_and_lock):
    """
    Prerequisite: RealSense + robot connected. Flat, unmarked sand in view.
    Asserts:
    - grooves_from_depth returns 0 strokes on flat sand.
    - PathExecutor completes with phase="done" and only the final pen-up move fires.
    """
    import threading
    import time
    from camera_thread import DepthCameraThread
    from depth_extractor import grooves_from_depth
    from path_extractor import extract_from_edges
    from path_executor import PathExecutor
    from robot_controller import RobotController

    state, lock = shared_state_and_lock
    ip = _robot_ip()

    # Depth capture
    ct = DepthCameraThread(state, lock)
    ct.start()
    time.sleep(2.0)
    captured = ct.capture_frame()
    ct.stop()
    assert captured is not None

    # Extraction — expects flat sand
    depth_m, valid, _rgb = captured
    extracted = extract_from_edges(grooves_from_depth(depth_m, valid))
    assert extracted.total_strokes == 0, "Expected flat sand — smooth out any grooves"

    # Execution — empty strokes
    rc = RobotController()
    rc.connect(ip)
    try:
        ex = PathExecutor(rc, state, lock)
        ex.start(extracted.strokes)
        ex._thread.join(timeout=10.0)
        with lock:
            assert state["phase"] == "done"
    finally:
        rc.disconnect()


@pytest.mark.integration
def test_full_pipeline_raked_groove(shared_state_and_lock):
    """
    Prerequisite: RealSense + robot. Grooves raked into the sand in view.
    Asserts:
    - At least one stroke extracted.
    - Robot executes all strokes, phase becomes "done", no robot fault.
    CAUTION: robot will move. Ensure workspace is clear before running.
    """
    import time
    from camera_thread import DepthCameraThread
    from depth_extractor import grooves_from_depth
    from path_extractor import extract_from_edges, pixels_to_robot_coords
    from path_executor import PathExecutor
    from robot_controller import RobotController
    from config import DEPTH_WIDTH, DEPTH_HEIGHT

    state, lock = shared_state_and_lock
    ip = _robot_ip()

    rc = RobotController()
    rc.connect(ip)

    ct = DepthCameraThread(state, lock)
    ct.start()
    time.sleep(2.0)
    captured = ct.capture_frame()
    ct.stop()
    assert captured is not None

    depth_m, valid, _rgb = captured
    extracted = extract_from_edges(grooves_from_depth(depth_m, valid))
    assert extracted.total_strokes > 0, "No grooves detected — rake some marks into the sand"

    with lock:
        workspace = state.get("workspace")
    assert workspace is not None, "Workspace must be set before running full pipeline"

    robot_strokes = pixels_to_robot_coords(
        extracted.strokes, workspace, DEPTH_WIDTH, DEPTH_HEIGHT
    )

    try:
        ex = PathExecutor(rc, state, lock)
        ex.start(robot_strokes)
        ex._thread.join(timeout=60.0)
        with lock:
            assert state["phase"] == "done", f"Unexpected phase: {state['phase']}"
    finally:
        rc.disconnect()
