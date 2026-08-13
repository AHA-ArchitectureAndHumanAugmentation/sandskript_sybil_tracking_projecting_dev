"""
Shared fixtures, mock factories, and synthetic image generators.
Lives at project root so pytest discovers it for all tests/ files.
"""
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest

from workspace import WorkspaceConfig


# ── Synthetic groove masks (binary, white-on-black) ───────────────────────────
# These stand in for the output of depth_extractor.grooves_from_depth — the
# 1-px-wide groove centrelines that path_extractor.extract_from_edges consumes.

@pytest.fixture
def mask_blank():
    """480×640 all-black binary mask. No grooves."""
    return np.zeros((480, 640), dtype=np.uint8)


@pytest.fixture
def mask_rectangle():
    """Thin white rectangle outline — a single long closed chain."""
    mask = np.zeros((480, 640), dtype=np.uint8)
    cv2.rectangle(mask, (100, 100), (300, 300), 255, 1)
    return mask


@pytest.fixture
def mask_tiny_dot():
    """A 2-pixel dot — below CONTOUR_MIN_PIXELS."""
    mask = np.zeros((480, 640), dtype=np.uint8)
    cv2.circle(mask, (50, 50), 1, 255, -1)
    return mask


@pytest.fixture
def mask_two_lines():
    """Two separate lines in opposite quadrants — used for TSP ordering tests."""
    mask = np.zeros((480, 640), dtype=np.uint8)
    cv2.line(mask, (50, 100), (150, 100), 255, 1)     # top-left
    cv2.line(mask, (490, 380), (590, 380), 255, 1)    # bottom-right
    return mask


@pytest.fixture
def mask_long_line():
    """A single long horizontal line (400px wide)."""
    mask = np.zeros((480, 640), dtype=np.uint8)
    cv2.line(mask, (100, 240), (500, 240), 255, 1)
    return mask


# ── Synthetic depth frames (metric, metres) ──────────────────────────────────

@pytest.fixture
def flat_depth():
    """480×640 flat surface at 0.30 m — no grooves."""
    return np.full((480, 640), 0.30, dtype=np.float32)


@pytest.fixture
def depth_with_groove():
    """
    Flat 0.30 m surface with a horizontal groove carved 3 mm deeper (0.303 m).
    Deeper = farther from the top-down camera = a positive 'valley' relief.
    """
    d = np.full((480, 640), 0.30, dtype=np.float32)
    cv2.line(d, (100, 240), (500, 240), 0.303, 3)
    return d


# ── Workspace fixture ─────────────────────────────────────────────────────────

@pytest.fixture
def simple_workspace():
    """
    Axis-aligned workspace for deterministic coordinate mapping:
      origin=[0,0,0], x_axis=[1,0,0], y_axis=[0,1,0], z_axis=[0,0,1]
      x_extent=0.3, y_extent=0.225

    Mapping: pixel (u,v) → robot (u/640*0.3, (1 - v/480)*0.225, 0)
    (v is flipped: image rows grow downward, world Y grows upward)
    """
    return WorkspaceConfig(
        origin=[0.0, 0.0, 0.0],
        x_axis=[1.0, 0.0, 0.0],
        y_axis=[0.0, 1.0, 0.0],
        z_axis=[0.0, 0.0, 1.0],
        x_extent=0.3,
        y_extent=0.225,
    )


# ── Mock robot factory ────────────────────────────────────────────────────────

@pytest.fixture
def mock_robot():
    """
    MagicMock that mirrors RobotController's public interface.
    - connected: True by default
    - get_ee_position(): returns [0.1, 0.2, 0.3, 0.0, 3.14159, 0.0]
    - move_to, stop_motion, start_freedrive, end_freedrive: no-op, call-recording mocks
    """
    robot = MagicMock()
    robot.connected = True
    robot.get_ee_position.return_value = [0.1, 0.2, 0.3, 0.0, 3.14159, 0.0]
    return robot


# ── Shared state factory ──────────────────────────────────────────────────────

@pytest.fixture
def shared_state_and_lock():
    """Returns (state_dict, lock) matching main.py's shared_state structure."""
    state = {
        "robot_connected":      False,
        "last_depth_color_jpg": None,
        "last_rgb_jpg":         None,
        "last_groove_jpg":      None,
        "workspace":            None,
        "pending_workspace":    None,
        "ws_points":            {"p0": None, "px": None, "py": None},
        "freedrive":            False,
        "ee":                   [0.0] * 6,
        "phase":                "idle",
        "captured_still":       None,
        "still_dims":           None,
        "strokes":              [],
        "executing":            False,
        "progress":             0.0,
    }
    return state, threading.Lock()
