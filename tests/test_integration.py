"""
Integration tests — require physical hardware (camera only; this repo never
connects to a robot — see main.py's _NO_ROBOT_MSG).
Run only when a camera is connected:
  pytest -m integration -v

Skip in CI:
  pytest -m "not integration" -v
"""
import pytest


# ─────────────────────────────────────────────────────────────────────────────
# Camera
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.integration
def test_camera_live_feed(shared_state_and_lock):
    """
    Prerequisite: RealSense D435i connected.
    Asserts:
    - Both JPEG streams are non-None bytes within 2 seconds.
    - The colorized-depth JPEG decodes to an image the size of the CANVAS.

    Not the camera's 640×480: the view is a stitched canvas whose size comes
    from the layout and config.STITCH_MM_PER_PX, and equals the camera's
    native resolution only by coincidence (which is what it used to assert,
    back when the resolution was on automatic). `frame_size` is what the whole
    pipeline maps from, so agreeing with THAT is the property that matters.
    """
    import time
    import cv2
    import numpy as np
    from camera_thread import DepthCameraThread

    state, lock = shared_state_and_lock
    ct = DepthCameraThread(state, lock)
    ct.start()
    time.sleep(2.0)
    # Assert before stop() — stop() clears both JPEG keys to None.
    depth_jpg  = state["last_depth_color_jpg"]
    groove_jpg = state["last_groove_jpg"]
    size = ct.frame_size
    ct.stop()

    assert depth_jpg is not None, "colorized-depth JPEG stream never populated"
    assert groove_jpg is not None, "groove JPEG stream never populated"
    assert size is not None, "canvas geometry never froze"

    color = cv2.imdecode(np.frombuffer(depth_jpg, np.uint8), cv2.IMREAD_COLOR)
    assert color is not None
    assert (color.shape[1], color.shape[0]) == size


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
    # Bounds are the CANVAS's, not the camera's — see test_camera_live_feed.
    height, width = depth_m.shape[:2]
    for stroke in result.strokes:
        for pt in stroke:
            assert 0 <= pt[0] <= width
            assert 0 <= pt[1] <= height
