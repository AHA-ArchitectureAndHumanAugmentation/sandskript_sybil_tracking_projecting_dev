"""
Unit tests for the whole-canvas view rotation (view_rotation.py) and the seam it
is applied at (camera_thread.set_view_rotation, workspace.with_frame_aspect).

The property that matters is that ONE setting turns everything together: the
picture, the frame size the mm scale divides by, and the crop that says which
part of the picture the pipeline looks at. No hardware.
"""
import asyncio
import json
import threading

import numpy as np
import pytest

import server as server_mod
import view_rotation as vr
from depth_extractor import Crop
from workspace import WorkspaceConfig


class TestNormalise:

    def test_quarter_turns_pass_through(self):
        for deg in (0, 90, 180, 270):
            assert vr.norm_deg(deg) == deg

    def test_wraps_past_a_full_turn(self):
        assert vr.norm_deg(360) == 0
        assert vr.norm_deg(450) == 90
        assert vr.norm_deg(-90) == 270

    def test_junk_is_no_rotation(self):
        """A bad settings.json must not stop the app — it must just not turn."""
        for bad in (None, "", "sideways", float("nan")):
            assert vr.norm_deg(bad) == 0

    def test_off_angles_snap_to_the_nearest_quarter(self):
        assert vr.norm_deg(80) == 90
        assert vr.norm_deg(10) == 0

    def test_turns_counts_quarters(self):
        assert [vr.turns(d) for d in (0, 90, 180, 270)] == [0, 1, 2, 3]


class TestRotateImage:

    @staticmethod
    def _ramp(h=4, w=6):
        return np.arange(h * w, dtype=np.float32).reshape(h, w)

    def test_quarter_turn_swaps_the_axes(self):
        out = vr.rotate_image(self._ramp(4, 6), 90)
        assert out.shape == (6, 4)

    def test_half_turn_keeps_the_shape(self):
        out = vr.rotate_image(self._ramp(4, 6), 180)
        assert out.shape == (4, 6)

    def test_turning_clockwise_sends_top_left_to_top_right(self):
        """The direction the button promises: ⟳ moves the corner rightwards."""
        img = self._ramp(4, 6)
        out = vr.rotate_image(img, 90)
        assert out[0, -1] == img[0, 0]

    def test_four_turns_are_the_identity(self):
        img = self._ramp()
        out = img
        for _ in range(4):
            out = vr.rotate_image(out, 90)
        assert np.array_equal(out, img)

    def test_values_are_untouched(self):
        """Re-indexed, never resampled — a rotated depth map is the same mm."""
        img = self._ramp()
        out = vr.rotate_image(img, 90)
        assert sorted(out.ravel().tolist()) == sorted(img.ravel().tolist())

    def test_result_is_contiguous(self):
        """np.rot90 hands back negative strides, which cv2 will not take."""
        out = vr.rotate_image(self._ramp(), 90)
        assert out.flags["C_CONTIGUOUS"]

    def test_colour_images_keep_their_channels(self):
        rgb = np.zeros((4, 6, 3), np.uint8)
        rgb[0, 0] = (10, 20, 30)
        out = vr.rotate_image(rgb, 90)
        assert out.shape == (6, 4, 3)
        assert tuple(out[0, -1]) == (10, 20, 30)

    def test_boolean_valid_masks_survive(self):
        ok = np.zeros((4, 6), bool)
        ok[0, 0] = True
        out = vr.rotate_image(ok, 90)
        assert out.dtype == bool and out[0, -1]

    def test_none_stays_none(self):
        """No colour frame / no reference is a normal state, not an error."""
        assert vr.rotate_image(None, 90) is None

    def test_zero_returns_the_same_object(self):
        img = self._ramp()
        assert vr.rotate_image(img, 0) is img


class TestRotateSize:

    def test_quarter_turn_swaps_width_and_height(self):
        assert vr.rotate_size((1280, 480), 90) == (480, 1280)
        assert vr.rotate_size((1280, 480), 270) == (480, 1280)

    def test_half_turn_keeps_it(self):
        assert vr.rotate_size((1280, 480), 180) == (1280, 480)

    def test_none_stays_none(self):
        assert vr.rotate_size(None, 90) is None


class TestRotateCrop:

    def test_full_frame_stays_full_frame(self):
        full = {"x": 0.0, "y": 0.0, "w": 1.0, "h": 1.0}
        assert vr.rotate_crop(full, 90) == full

    def test_a_corner_crop_follows_the_picture(self):
        """Top-left quarter, turned clockwise, is the top-RIGHT quarter."""
        out = vr.rotate_crop({"x": 0.0, "y": 0.0, "w": 0.5, "h": 0.5}, 90)
        assert out == {"x": 0.5, "y": 0.0, "w": 0.5, "h": 0.5}

    def test_four_turns_return_the_original(self):
        crop = {"x": 0.1, "y": 0.25, "w": 0.4, "h": 0.3}
        out = crop
        for _ in range(4):
            out = vr.rotate_crop(out, 90)
        for key in crop:
            assert out[key] == pytest.approx(crop[key])

    def test_the_rotated_crop_covers_the_rotated_pixels(self):
        """
        The whole point: crop the canvas then turn it, or turn it then crop with
        the turned crop — same pixels. This is what keeps the framed sand framed.
        """
        img = np.arange(40 * 60, dtype=np.float32).reshape(40, 60)
        crop = {"x": 0.0, "y": 0.0, "w": 0.5, "h": 0.5}

        x0, y0, x1, y1 = Crop.from_dict(crop).pixel_box(60, 40)
        crop_then_turn = vr.rotate_image(img[y0:y1, x0:x1], 90)

        turned = vr.rotate_image(img, 90)
        th, tw = turned.shape
        rx0, ry0, rx1, ry1 = Crop.from_dict(
            vr.rotate_crop(crop, 90)).pixel_box(tw, th)
        turn_then_crop = turned[ry0:ry1, rx0:rx1]

        assert np.array_equal(crop_then_turn, turn_then_crop)

    def test_none_and_junk_pass_through_untouched(self):
        assert vr.rotate_crop(None, 90) is None
        assert vr.rotate_crop({}, 90) == {}
        assert vr.rotate_crop({"x": "left"}, 90) == {"x": "left"}


class TestWorkspaceFollowsTheAspect:

    @staticmethod
    def _ws(aspect):
        return WorkspaceConfig.simulation(aspect)

    def test_y_extent_tracks_the_frame_aspect(self):
        ws = self._ws(2.0)
        assert ws.y_extent == pytest.approx(ws.x_extent / 2.0)
        turned = ws.with_frame_aspect(0.5)
        assert turned.y_extent == pytest.approx(ws.x_extent / 0.5)

    def test_the_plane_itself_does_not_move(self):
        """Only the isotropy correction changes — not the touched-off frame."""
        ws = self._ws(2.0)
        turned = ws.with_frame_aspect(0.5)
        assert turned.origin == ws.origin
        assert turned.x_axis == ws.x_axis
        assert turned.x_extent == ws.x_extent

    def test_a_useless_aspect_is_ignored(self):
        ws = self._ws(2.0)
        assert ws.with_frame_aspect(0) is ws
        assert ws.with_frame_aspect(None) is ws


class TestCameraThreadSeam:
    """
    The camera thread is where the rotation is applied, so it is also where
    `frame_size` has to start telling the truth — everything mm-based divides
    by that width.
    """

    @staticmethod
    def _thread():
        import threading

        from camera_thread import DepthCameraThread
        from stitcher import CanvasGrid

        state: dict = {}
        cam = DepthCameraThread(state, threading.Lock())
        cam._grid = CanvasGrid(origin_mm=(0.0, 0.0), width=1280, height=480,
                               mm_per_px=1.0)
        return cam, state

    def test_frame_size_swaps_on_a_quarter_turn(self):
        cam, _ = self._thread()
        assert cam.frame_size == (1280, 480)
        cam.set_view_rotation(90)
        assert cam.frame_size == (480, 1280)
        cam.set_view_rotation(180)
        assert cam.frame_size == (1280, 480)

    def test_setting_it_republishes_frame_size(self):
        """Stale frame_size = every mm in the UI wrong by the aspect ratio."""
        cam, state = self._thread()
        cam.set_view_rotation(90)
        assert state["frame_size"] == [480, 1280]

    def test_junk_angles_do_not_turn_the_canvas(self):
        cam, _ = self._thread()
        assert cam.set_view_rotation("sideways") == 0
        assert cam.frame_size == (1280, 480)

    def test_no_canvas_yet_is_not_an_error(self):
        """The button can be pressed before the cameras have reported."""
        from camera_thread import DepthCameraThread

        state: dict = {}
        cam = DepthCameraThread(state, threading.Lock())
        assert cam.set_view_rotation(90) == 90
        assert cam.frame_size is None
        assert "frame_size" not in state


class TestCapturedStillIsTurnedToo:
    """
    The seam end to end, on a synthetic two-camera rig: what `capture_frame`
    hands the pipeline must be turned the same way the live views are, and must
    match the `frame_size` every mm-per-pixel calculation divides by. A still in
    one orientation and a frame_size in another is the failure this guards.
    """

    @staticmethod
    def _thread_with_frames():
        from collections import deque

        from camera_thread import DepthCameraThread
        from stitcher import CanvasGrid, stitch, synthetic_scene

        frames, calib = synthetic_scene(2)
        cam = DepthCameraThread({}, threading.Lock())
        cam._calib = calib
        cam._grid = CanvasGrid.from_result(stitch(frames, calib))
        cam._serials = [f.serial for f in frames]
        cam._intrinsics = [f.intr for f in frames]
        cam._last_rgb = [f.rgb for f in frames]
        cam._buffers = [deque([(f.depth_m, f.valid)]) for f in frames]
        return cam

    def test_unturned_capture_matches_frame_size(self):
        cam = self._thread_with_frames()
        depth, valid, _rgb = cam.capture_frame()
        assert (depth.shape[1], depth.shape[0]) == cam.frame_size
        assert valid.shape == depth.shape

    def test_a_quarter_turn_turns_the_still_with_the_view(self):
        cam = self._thread_with_frames()
        straight, _, _ = cam.capture_frame()
        cam.set_view_rotation(90)
        turned, turned_valid, _ = cam.capture_frame()

        assert turned.shape == straight.shape[::-1]
        # …and it is the SAME picture, turned — not a re-stitch at another size.
        assert np.array_equal(turned, vr.rotate_image(straight, 90))
        assert turned_valid.shape == turned.shape
        # The size the mm scale divides by agrees with the still it came from.
        assert (turned.shape[1], turned.shape[0]) == cam.frame_size

    def test_colour_turns_with_the_depth(self):
        """A still whose colour and depth disagreed would misalign every crop."""
        cam = self._thread_with_frames()
        cam.set_view_rotation(270)
        depth, _valid, rgb = cam.capture_frame()
        assert rgb is not None
        assert rgb.shape[:2] == depth.shape[:2]


class FakeWS:
    """Collects what the server sends, in place of a websocket."""

    def __init__(self):
        self.sent = []

    async def send_str(self, msg):
        self.sent.append(json.loads(msg))


class TestServerWiring:

    @staticmethod
    def _server(state=None, **kwargs):
        return server_mod.Server(state if state is not None else {},
                                 threading.Lock(),
                                 on_connect=None, on_disconnect=None, **kwargs)

    def test_rotate_view_reaches_the_handler(self):
        seen = []

        async def handler(ws, params):
            seen.append(params)

        srv = self._server(on_rotate_view=handler)

        async def go():
            await srv._handle_ws_message(
                FakeWS(), json.dumps({"type": "rotate_view",
                                      "params": {"steps": 1}}))
            await asyncio.sleep(0)      # the dispatcher fires a task

        asyncio.run(go())
        assert seen == [{"steps": 1}]

    def test_broadcast_carries_the_angle_and_the_turned_crop(self):
        """
        The crop travels WITH the angle: a client that got one without the other
        would draw its box over the wrong part of the turned canvas.
        """
        srv = self._server()
        ws = FakeWS()
        srv._ws_clients.add(ws)
        crop = {"x": 0.5, "y": 0.0, "w": 0.5, "h": 0.5}
        asyncio.run(srv.broadcast_view_rotation(90, crop))
        assert ws.sent == [{"type": "view_rotation", "deg": 90, "crop": crop}]

    def test_init_reports_the_current_rotation(self):
        """A reopened window must show the angle the pipeline is using."""
        srv = self._server({"view_rotation": 270})
        ws = FakeWS()
        asyncio.run(srv._send_init(ws, "", None))
        assert ws.sent[0]["view_rotation"] == 270
