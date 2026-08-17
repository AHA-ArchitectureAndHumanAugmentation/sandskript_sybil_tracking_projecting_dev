"""
Unit tests for the COMBINED camera view — the canvas every Developer- and
Participant-Mode view is built from. No hardware: the RealSense layer
(realsense_source) is never touched; frames are handed to the thread's buffers
directly, which is exactly what the device loop does.

Two things are under test:
  * stitcher's frozen `CanvasGrid` — the guarantee that the pipeline's frame
    size cannot move once the app is running;
  * camera_thread.DepthCameraThread deriving every live view (crop, colour,
    grooves, projector mask, depth labels, participant trigger) from that
    canvas instead of from one camera's 640×480 frame.
"""

import json
import threading

import numpy as np
import pytest

from camera_thread import DepthCameraThread
from depth_extractor import Crop, DepthGrooveParams
from stitcher import (
    CameraFrame, CameraPlacement, CanvasGrid, Intrinsics, StitchCalib,
    load_calib, stitch, with_default_row,
)

CAM_W, CAM_H = 160, 120
TILE = 400.0, 300.0     # mm covered by one camera on the canvas


def _frame(depth=0.80, serial="A", rgb_level=None):
    """One camera's flat capture, optionally with aligned colour."""
    color = (None if rgb_level is None
             else np.full((CAM_H, CAM_W, 3), rgb_level, np.uint8))
    return CameraFrame(depth_m=np.full((CAM_H, CAM_W), depth, np.float32),
                       valid=np.ones((CAM_H, CAM_W), bool),
                       intr=Intrinsics.nominal(CAM_W, CAM_H),
                       rgb=color, serial=serial)


def _quad(x0):
    w, h = TILE
    return ((x0, 0.0), (x0 + w, 0.0), (x0, h), (x0 + w, h))


def _row_calib(serials=("A", "B")):
    """A saved two-camera layout: tiles flush left to right, like the tool writes."""
    return StitchCalib(
        cams=[CameraPlacement(serial=s, quad_mm=_quad(i * TILE[0]))
              for i, s in enumerate(serials)],
        mm_per_px=2.0)


class TestCanvasGrid:
    def test_grid_is_taken_from_a_result_and_reproduces_it(self):
        calib = _row_calib()
        first = stitch([_frame(serial="A"), _frame(serial="B")], calib)
        grid = CanvasGrid.from_result(first)
        again = stitch([_frame(serial="A"), _frame(serial="B")], calib, grid=grid)
        assert again.depth_m.shape == first.depth_m.shape
        assert again.origin_mm == pytest.approx(first.origin_mm)
        assert again.mm_per_px == pytest.approx(first.mm_per_px)

    def test_a_camera_dropping_out_does_not_resize_the_canvas(self):
        """The invariant the whole pipeline leans on: crop, reference frame and
        captured still are all in canvas pixels, so the canvas may not move."""
        calib = _row_calib()
        grid = CanvasGrid.from_result(
            stitch([_frame(serial="A"), _frame(serial="B")], calib))

        alone = stitch([_frame(serial="A")], calib, grid=grid)
        assert alone.depth_m.shape == (grid.height, grid.width)
        # The missing camera's half is simply blank, not cropped away.
        assert alone.valid[:, :grid.width // 3].any()
        assert not alone.valid[:, -grid.width // 3:].any()

    def test_without_a_grid_the_canvas_follows_the_cameras(self):
        calib = _row_calib()
        both = stitch([_frame(serial="A"), _frame(serial="B")], calib)
        one = stitch([_frame(serial="A")], calib)
        assert one.depth_m.shape[1] < both.depth_m.shape[1]

    def test_a_grid_survives_a_cycle_with_no_camera_data_at_all(self):
        calib = _row_calib()
        grid = CanvasGrid.from_result(
            stitch([_frame(serial="A"), _frame(serial="B")], calib))
        empty = stitch([], calib, grid=grid)
        assert empty.depth_m.shape == (grid.height, grid.width)
        assert not empty.valid.any()

    def test_the_canvas_is_wider_than_one_camera_frame(self):
        result = stitch([_frame(serial="A"), _frame(serial="B")], _row_calib())
        assert result.depth_m.shape[1] > CAM_W
        assert result.depth_m.shape[1] > result.depth_m.shape[0]


class TestDefaultRow:
    def test_unplaced_cameras_are_laid_out_placed_ones_are_left_alone(self):
        calib = StitchCalib(cams=[CameraPlacement(serial="A", quad_mm=_quad(999.0)),
                                  CameraPlacement(serial="B")])
        out = with_default_row(calib, [TILE, TILE])
        assert out.cams[0].quad_mm == _quad(999.0)     # operator's placement kept
        assert len(out.cams[1].quad_mm) == 4           # the other one got a slot

    def test_an_empty_rig_is_returned_unchanged(self):
        assert with_default_row(StitchCalib(), []).cams == []


class TestLoadCalib:
    def test_reads_a_saved_layout(self, tmp_path):
        path = tmp_path / "stitch_calibration.json"
        path.write_text(json.dumps(_row_calib().to_dict()))
        calib = load_calib(path)
        assert [c.serial for c in calib.cams] == ["A", "B"]
        assert calib.cams[1].quad_mm[0][0] == pytest.approx(TILE[0])
        assert calib.mm_per_px == pytest.approx(2.0)

    def test_missing_or_broken_file_is_an_empty_rig_not_a_crash(self, tmp_path):
        assert load_calib(tmp_path / "nope.json").cams == []
        bad = tmp_path / "bad.json"
        bad.write_text("{ not json")
        assert load_calib(bad).cams == []


# ── the camera thread, fed frames by hand ─────────────────────────────────────

def _thread_with_rig(state=None, serials=("A", "B"), depth=0.80, rgb_level=None):
    """A DepthCameraThread wired to a two-camera rig, canvas already frozen."""
    state = state if state is not None else {}
    thread = DepthCameraThread(state, threading.Lock())
    frames = [_frame(depth=depth, serial=s, rgb_level=rgb_level) for s in serials]
    thread._calib = _row_calib(serials)
    thread._serials = list(serials)
    thread._intrinsics = [f.intr for f in frames]
    thread._buffers = [[(f.depth_m, f.valid)] for f in frames]
    thread._last_rgb = [f.rgb for f in frames]
    thread._grid = CanvasGrid.from_result(stitch(frames, thread._calib))
    return thread, state


class TestCaptureFrame:
    def test_nothing_is_captured_before_the_canvas_is_frozen(self):
        thread, _ = _thread_with_rig()
        thread._grid = None
        assert thread.capture_frame() is None

    def test_capture_returns_the_combined_canvas(self):
        thread, _ = _thread_with_rig()
        depth, valid, rgb = thread.capture_frame()
        assert depth.shape == (thread._grid.height, thread._grid.width)
        assert depth.shape[1] > CAM_W          # both cameras, not just one
        assert valid.any()
        assert rgb is None                     # no colour was delivered

    def test_colour_is_combined_too(self):
        thread, _ = _thread_with_rig(rgb_level=90)
        _depth, _valid, rgb = thread.capture_frame()
        assert rgb is not None
        assert rgb.shape[:2] == (thread._grid.height, thread._grid.width)
        assert (rgb[..., 0] == 90).any()

    def test_each_camera_is_averaged_on_its_own_buffer(self):
        """Averaging before the warp is the point — it cuts sensor noise on the
        raw frame instead of on an already-resampled canvas."""
        thread, _ = _thread_with_rig()
        for i, extra in enumerate((0.82, 0.86)):
            f = _frame(depth=extra)
            thread._buffers[i].append((f.depth_m, f.valid))
        depth, valid, _rgb = thread.capture_frame()
        left = depth[valid][:10]
        assert depth[valid].min() == pytest.approx(0.81, abs=1e-3)   # (0.80+0.82)/2
        assert depth[valid].max() == pytest.approx(0.83, abs=1e-3)   # (0.80+0.86)/2
        assert left.size

    def test_a_camera_with_no_frames_yet_is_skipped_not_fatal(self):
        thread, _ = _thread_with_rig()
        thread._buffers[1] = []
        depth, valid, _rgb = thread.capture_frame()
        assert depth.shape == (thread._grid.height, thread._grid.width)
        assert valid.any()

    def test_no_frames_at_all_is_none(self):
        thread, _ = _thread_with_rig()
        thread._buffers = [[], []]
        assert thread.capture_frame() is None

    def test_frame_size_reports_the_canvas(self):
        thread, _ = _thread_with_rig()
        assert thread.frame_size == (thread._grid.width, thread._grid.height)
        assert thread.camera_count == 2


def _publish_once(thread, state, crop=None, trigger_mm=None,
                  overlay=0, projection=0, depth=0.80, hand=None):
    """Run one canvas through the live-view derivation, as the loop does."""
    state["depth_overlay_clients"] = overlay
    state["projection_clients"] = projection
    if crop is not None:
        thread.set_live_crop(crop)
    thread.set_trigger_threshold(trigger_mm)
    frames = [_frame(depth=depth, serial=s) for s in thread._serials]
    if hand is not None:
        # A hand over the LEFT camera only: a near patch big enough to trip
        # the trigger's minimum-area guard.
        frames[0].depth_m[40:100, 20:100] = hand
    result = stitch(frames, thread._calib, grid=thread._grid)
    thread._publish(result.depth_m, result.valid, None, 0)
    return result


class TestLiveViews:
    def test_every_live_stream_is_published_from_the_canvas(self):
        thread, state = _thread_with_rig()
        _publish_once(thread, state)
        assert state["last_depth_color_jpg"]
        assert state["last_groove_jpg"]
        assert state["last_mask_jpg"]

    def test_the_popup_stream_and_labels_follow_the_developer_crop(self):
        thread, state = _thread_with_rig()
        crop = Crop(0.25, 0.0, 0.5, 1.0)
        _publish_once(thread, state, crop=crop, overlay=1)
        assert state["last_depth_crop_jpg"]
        # The popup re-fits its stage from this size, so it must be the crop of
        # the CANVAS — not of one camera's frame.
        expected_w = int(round(0.5 * thread._grid.width))
        assert state["depth_labels_size"][0] == pytest.approx(expected_w, abs=2)
        assert state["depth_labels_size"][1] == thread._grid.height

    def test_no_popup_connected_means_no_crop_stream_and_no_labels(self):
        thread, state = _thread_with_rig()
        _publish_once(thread, state, overlay=0)
        assert state["last_depth_crop_jpg"] is None
        assert state["depth_labels"] is None

    def test_projector_mask_is_composed_at_full_canvas_size(self):
        import cv2
        thread, state = _thread_with_rig()
        _publish_once(thread, state, crop=Crop(0.1, 0.1, 0.5, 0.5), projection=1)
        full = cv2.imdecode(np.frombuffer(state["last_mask_full_jpg"], np.uint8),
                            cv2.IMREAD_GRAYSCALE)
        assert full.shape == (thread._grid.height, thread._grid.width)

    def test_projector_mask_is_not_composed_without_a_projection_window(self):
        thread, state = _thread_with_rig()
        _publish_once(thread, state, projection=0)
        assert state["last_mask_full_jpg"] is None

    def test_trigger_fires_on_something_close_inside_the_crop(self):
        thread, state = _thread_with_rig()
        _publish_once(thread, state, trigger_mm=700.0, hand=0.40)
        assert state["trigger_below"] is True

    def test_trigger_ignores_something_close_outside_the_crop(self):
        """Motion beside the sand must not arm Participant Mode — the crop is
        the visible region, and it is now a crop of the whole rig's canvas."""
        thread, state = _thread_with_rig()
        # Crop the RIGHT half; the hand is over the left camera.
        _publish_once(thread, state, crop=Crop(0.55, 0.0, 0.45, 1.0),
                      trigger_mm=700.0, hand=0.40)
        assert state["trigger_below"] is False

    def test_no_trigger_distance_means_no_flag(self):
        thread, state = _thread_with_rig()
        _publish_once(thread, state, trigger_mm=None, hand=0.40)
        assert state["trigger_below"] is None

    def test_live_params_reach_the_groove_preview(self):
        thread, state = _thread_with_rig()
        thread.set_live_params(DepthGrooveParams.from_dict({"groove_depth_mm": 0.5}))
        _publish_once(thread, state)
        assert state["last_mask_jpg"]
