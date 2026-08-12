"""
Unit tests for path_export.py — JSON + URScript record + bundle saving.
No hardware.
"""
import base64
import json
import math

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from path_export import (
    build_json, build_urscript, is_png_data_url, save_bundle, _offset_pose,
    stroke_blend,
)

_PI = math.pi

# Two short strokes, tool-down orientation ([0, π, 0] → tool Z points −Z).
STROKES = [
    [[0.4, 0.0, 0.2, 0.0, _PI, 0.0], [0.45, 0.0, 0.2, 0.0, _PI, 0.0]],
    [[0.4, 0.1, 0.2, 0.0, _PI, 0.0], [0.4, 0.15, 0.2, 0.0, _PI, 0.0], [0.4, 0.2, 0.2, 0.0, _PI, 0.0]],
]


# ─────────────────────────────────────────────────────────────────────────────
# URScript — a record of the same poses, not what the GoFa executes
# ─────────────────────────────────────────────────────────────────────────────

class TestUrscript:

    def test_structure(self):
        s = build_urscript(STROKES, speed=0.3, safety=0.05)
        assert "def draw_path():" in s
        assert s.strip().endswith("draw_path()")
        assert "end" in s

    def test_travels_use_movel_draws_use_movep(self):
        s = build_urscript(STROKES, speed=0.3, safety=0.05)
        assert s.count("movel(") == len(STROKES) * 3   # approach + start + lift each
        assert s.count("movep(") == sum(len(st) - 1 for st in STROKES)

    def test_poses_formatted(self):
        s = build_urscript(STROKES, speed=0.25, safety=0.05)
        assert "p[0.40000, 0.00000, 0.20000, 0.00000, 3.14159, 0.00000]" in s
        assert "v=0.2500" in s

    def test_safety_retract_is_above_for_tool_down(self):
        # tool-down retract adds +Z; first movel target should be z = 0.2 + 0.05
        s = build_urscript([STROKES[0]], speed=0.3, safety=0.05)
        first = s.split("movel(")[1]
        assert "0.25000" in first

    def test_it_carries_the_same_poses_as_the_json(self, tmp_path):
        # The two files are one record in two forms; if they ever disagree the
        # .script stops being a usable reference for what was drawn.
        folder = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path)
        script = (folder / "path.script").read_text(encoding="utf-8")
        data = json.loads((folder / "path.json").read_text(encoding="utf-8"))
        for stroke in data["strokes"]:
            for wp in stroke:
                x, y, z = wp["pose"][:3]
                assert f"{x:.5f}, {y:.5f}, {z:.5f}" in script

    def test_the_header_says_it_is_not_executable(self, tmp_path):
        # Nothing here runs this file; the header is what stops someone
        # loading it onto a controller and expecting the GoFa to obey.
        folder = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path)
        head = (folder / "path.script").read_text(encoding="utf-8")[:400]
        assert "NOT what the GoFa runs" in head


# ─────────────────────────────────────────────────────────────────────────────
# Blend radius (the exec-bar Radius slider)
# ─────────────────────────────────────────────────────────────────────────────

class TestBlendRadius:

    def test_blend_is_recorded_in_the_saved_meta(self, tmp_path):
        # The Radius the run used has to survive into the bundle, or a replay
        # cannot reproduce the same corners.
        folder = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path,
                             blend_m=0.003)
        meta = json.loads((folder / "path.json").read_text())["meta"]
        assert meta["blend_mm"] == pytest.approx(3.0)

    def test_stroke_blend_passthrough_when_small(self):
        # 50 mm segments: a 3 mm request is far below the 45% cap.
        assert stroke_blend(STROKES[1], 0.003) == pytest.approx(0.003)

    def test_stroke_blend_clamped_to_shortest_segment(self):
        # 10 mm then 4 mm tail segment: 5 mm request must clamp to 0.45 × 4 mm.
        stroke = [[0.4, 0.0, 0.2, 0.0, _PI, 0.0],
                  [0.41, 0.0, 0.2, 0.0, _PI, 0.0],
                  [0.414, 0.0, 0.2, 0.0, _PI, 0.0]]
        assert stroke_blend(stroke, 0.005) == pytest.approx(0.45 * 0.004)

    def test_stroke_blend_zero_and_degenerate(self):
        assert stroke_blend(STROKES[0], 0.0) == 0.0
        assert stroke_blend(STROKES[0], -1.0) == 0.0
        assert stroke_blend([STROKES[0][0]], 0.005) == 0.005  # 1 pt: nothing to clamp

    def test_clamp_survives_a_stroke_with_one_short_tail_segment(self):
        # Resampling regularly leaves a short final segment; the clamp has to
        # follow the SHORTEST one, not the average.
        stroke = [[0.4, 0.0, 0.2, 0.0, _PI, 0.0],
                  [0.45, 0.0, 0.2, 0.0, _PI, 0.0],
                  [0.50, 0.0, 0.2, 0.0, _PI, 0.0],
                  [0.504, 0.0, 0.2, 0.0, _PI, 0.0]]
        assert stroke_blend(stroke, 0.005) == pytest.approx(0.45 * 0.004)


# ─────────────────────────────────────────────────────────────────────────────
# JSON
# ─────────────────────────────────────────────────────────────────────────────

class TestJson:

    def test_shape_and_meta(self):
        j = build_json(STROKES, {"mode": "surface"})
        assert j["meta"]["mode"] == "surface"
        assert len(j["strokes"]) == 2
        assert len(j["strokes"][1]) == 3
        assert "pose" in j["strokes"][0][0]
        assert "plane" in j["strokes"][0][0]

    def test_plane_is_orthonormal_frame(self):
        j = build_json(STROKES, {})
        pl = j["strokes"][0][0]["plane"]
        x, y, z = np.array(pl["xaxis"]), np.array(pl["yaxis"]), np.array(pl["zaxis"])
        assert abs(np.linalg.norm(x) - 1) < 1e-4
        assert abs(np.dot(x, y)) < 1e-4               # orthogonal
        assert np.allclose(np.cross(x, y), z, atol=1e-4)   # right-handed
        # tool-down: approach axis (z) points down
        assert np.allclose(z, [0, 0, -1], atol=1e-4)

    def test_plane_origin_matches_pose(self):
        j = build_json(STROKES, {})
        wp = j["strokes"][0][0]
        assert wp["plane"]["origin"] == wp["pose"][:3]


# ─────────────────────────────────────────────────────────────────────────────
# Offset + bundle
# ─────────────────────────────────────────────────────────────────────────────

class TestOffset:

    def test_offset_lifts_along_normal(self):
        p = [0.4, 0.0, 0.2, 0.0, _PI, 0.0]
        out = _offset_pose(p, 0.01)     # tool-down → +Z
        assert abs(out[2] - 0.21) < 1e-9
        assert out[:2] == p[:2] and out[3:] == p[3:]


class TestSaveBundle:

    def test_creates_the_bundle_files(self, tmp_path):
        # 1×1 transparent PNG
        png_b64 = ("data:image/png;base64,"
                   "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==")
        folder = save_bundle(STROKES, speed=0.3, safety_m=0.05, offset_m=0.0,
                             meta={"mode": "surface"}, preview_png_data_url=png_b64,
                             base_dir=tmp_path)
        assert (folder / "path.json").exists()
        assert (folder / "path.script").exists()
        assert (folder / "preview.png").exists()
        assert (folder / "preview.png").stat().st_size > 0

    def test_json_is_valid_and_offset_applied(self, tmp_path):
        folder = save_bundle(STROKES, speed=0.3, safety_m=0.05, offset_m=0.005,
                             meta={"mode": "planar"}, base_dir=tmp_path)
        data = json.loads((folder / "path.json").read_text())
        # first waypoint z lifted by 5 mm (tool-down → +Z)
        assert abs(data["strokes"][0][0]["pose"][2] - 0.205) < 1e-6

    def test_no_image_skips_png(self, tmp_path):
        folder = save_bundle(STROKES, speed=0.3, safety_m=0.05, offset_m=0.0,
                             meta={}, preview_png_data_url=None, base_dir=tmp_path)
        assert not (folder / "preview.png").exists()

    def test_collision_makes_distinct_folder(self, tmp_path):
        a = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path)
        b = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path)
        assert a != b        # same-second saves don't overwrite


class TestPngDataUrl:
    """
    The gate on a preview screenshot pushed up by a browser. Participant Mode
    saves whatever a Developer window volunteers, with no Save click behind it,
    so this is the only thing standing between a client blob and the bundle.
    """

    # 1×1 transparent PNG, exactly what canvas.toDataURL("image/png") produces.
    PNG = ("data:image/png;base64,"
           "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==")

    def test_accepts_a_real_canvas_shot(self):
        assert is_png_data_url(self.PNG) is True

    def test_rejects_junk(self):
        for value in (None, "", 42, [], "not a data url", "data:image/png;base64,",
                      "data:image/png;base64,!!!not base64!!!"):
            assert is_png_data_url(value) is False

    def test_rejects_another_format_wearing_the_png_label(self):
        # Right prefix, decodable, but the bytes are not a PNG — the signature
        # is what stops a JPEG (or anything else) landing in preview.png.
        jpeg = "data:image/png;base64," + base64.b64encode(b"\xff\xd8\xff\xe0junk").decode()
        assert is_png_data_url(jpeg) is False

    def test_rejects_a_different_mime(self):
        assert is_png_data_url(self.PNG.replace("image/png", "image/jpeg")) is False

    def test_rejects_an_oversized_image(self):
        # A cap on what one message may park in memory until the save.
        assert is_png_data_url(self.PNG, max_bytes=10) is False
        assert is_png_data_url(self.PNG, max_bytes=10_000) is True

    def test_what_it_accepts_is_what_save_bundle_writes(self, tmp_path):
        # The validator and the writer must agree, or a "valid" preview is
        # silently dropped at save time.
        assert is_png_data_url(self.PNG)
        folder = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, preview_png_data_url=self.PNG,
                             base_dir=tmp_path)
        assert (folder / "preview.png").stat().st_size > 0


class TestSaveDetectionImages:
    """mask.png + skeleton.png — the detection stage kept beside its path."""

    @staticmethod
    def _mask(w=32, h=24):
        m = np.zeros((h, w), np.uint8)
        m[10:14, 4:28] = 255            # one thick horizontal groove
        return m

    @staticmethod
    def _skeleton(w=32, h=24):
        s = np.zeros((h, w), np.uint8)
        s[12, 4:28] = 255               # its 1-px centreline
        return s

    def test_both_images_are_written(self, tmp_path):
        folder = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path,
                             mask=self._mask(), skeleton=self._skeleton())
        assert (folder / "mask.png").stat().st_size > 0
        assert (folder / "skeleton.png").stat().st_size > 0

    def test_the_pixels_survive_the_round_trip(self, tmp_path):
        mask = self._mask()
        folder = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path,
                             mask=mask, skeleton=self._skeleton())
        # PNG is lossless: the saved mask must be the mask, not a re-rendering.
        back = cv2.imread(str(folder / "mask.png"), cv2.IMREAD_GRAYSCALE)
        assert back.shape == mask.shape
        assert (back == mask).all()

    def test_the_two_images_are_not_the_same_picture(self, tmp_path):
        folder = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path,
                             mask=self._mask(), skeleton=self._skeleton())
        m = cv2.imread(str(folder / "mask.png"), cv2.IMREAD_GRAYSCALE)
        s = cv2.imread(str(folder / "skeleton.png"), cv2.IMREAD_GRAYSCALE)
        assert s.sum() < m.sum()        # the centreline is thinner than the region

    def test_missing_images_are_simply_absent(self, tmp_path):
        # A save must never fail because a generate left nothing behind.
        folder = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path)
        assert (folder / "path.json").exists()
        assert not (folder / "mask.png").exists()
        assert not (folder / "skeleton.png").exists()

    def test_an_empty_array_is_not_written(self, tmp_path):
        folder = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path,
                             mask=np.zeros((0, 0), np.uint8))
        assert not (folder / "mask.png").exists()
