"""Unit tests for the multi-camera stitching math (stitcher.py). No hardware."""

import cv2
import numpy as np
import pytest

from stitcher import (
    CameraFrame, CameraPlacement, Intrinsics, StitchCalib,
    bind_placements, crop_corners_px, crop_mask, default_quad_mm,
    fill_small_holes, footprint_mm, quad_area_mm2, requad_for_crop,
    rotate_frame, rotate_quad, stitch, synthetic_scene,
)

UNIT_QUAD = ((0.0, 0.0), (400.0, 0.0), (0.0, 300.0), (400.0, 300.0))   # TL TR BL BR


def _flat_frame(depth=0.8, w=160, h=120, rgb=False, serial=""):
    intr = Intrinsics.nominal(w, h)
    d = np.full((h, w), depth, np.float32)
    v = np.ones((h, w), bool)
    color = np.full((h, w, 3), 120, np.uint8) if rgb else None
    return CameraFrame(depth_m=d, valid=v, intr=intr, rgb=color, serial=serial)


def _one(frame, placement=None):
    return stitch([frame], StitchCalib(cams=[placement or CameraPlacement()]))


def _covered_width_mm(result):
    return float(result.valid.any(axis=0).sum()) * result.mm_per_px


def _groove_contrast(result, mask):
    """How much relief survives in a region — misplaced cameras average it away."""
    d = result.depth_m.astype(np.float32)
    return float((d - cv2.blur(d, (15, 15)))[mask].std())


class TestFrameOps:
    def test_rotate_180_flips_both_ways(self):
        f = _flat_frame()
        f.depth_m[10, 20] = 0.9
        r = rotate_frame(f, 180)
        assert r.depth_m[f.intr.height - 11, f.intr.width - 21] == pytest.approx(0.9)

    def test_rotate_90_swaps_axes_and_intrinsics(self):
        f = _flat_frame(w=160, h=120)
        f.depth_m[10, 20] = 0.9
        r = rotate_frame(f, 90)
        assert (r.intr.width, r.intr.height) == (120, 160)
        assert (r.intr.fx, r.intr.fy) == (f.intr.fy, f.intr.fx)
        # Clockwise: pixel (u,v) lands at (H-1-v, u).
        assert r.depth_m[20, f.intr.height - 1 - 10] == pytest.approx(0.9)

    def test_four_quarter_turns_are_identity(self):
        f = _flat_frame()
        f.depth_m[7, 33] = 0.95
        r = f
        for _ in range(4):
            r = rotate_frame(r, 90)
        assert np.array_equal(r.depth_m, f.depth_m)
        assert r.intr == f.intr

    def test_rotation_snaps_to_quarter_turns(self):
        assert CameraPlacement.from_dict({"rot_deg": 271}).rot_deg == 270
        assert CameraPlacement.from_dict({"rot_deg": -90}).rot_deg == 270
        assert CameraPlacement.from_dict({"rot_deg": "junk"}).rot_deg == 0

    def test_crop_mask_keeps_only_the_rectangle(self):
        v = np.ones((100, 200), bool)
        out = crop_mask(v, (0.25, 0.5, 0.5, 0.5))
        assert out[:50].sum() == 0                 # top half dropped
        assert out.sum() == pytest.approx(100 * 50, rel=0.02)
        assert out[75, 50] and not out[75, 40]

    def test_crop_corners_are_in_shared_order(self):
        c = crop_corners_px((120, 160), (0.25, 0.5, 0.5, 0.5))
        assert c.shape == (4, 2)
        assert list(c[0]) == [40.0, 60.0]      # TL
        assert list(c[1]) == [120.0, 60.0]     # TR
        assert list(c[2]) == [40.0, 120.0]     # BL
        assert list(c[3]) == [120.0, 120.0]    # BR

    def test_bad_crop_falls_back_to_the_full_frame(self):
        assert CameraPlacement.from_dict({"crop": [0, 0]}).crop == (0.0, 0.0, 1.0, 1.0)
        assert CameraPlacement.from_dict({"crop": None}).crop == (0.0, 0.0, 1.0, 1.0)
        c = CameraPlacement.from_dict({"crop": [0.8, 0.0, 0.9, 1.0]}).crop
        assert c[0] + c[2] == pytest.approx(1.0)


class TestPlacement:
    def test_dict_roundtrip(self):
        p = CameraPlacement(serial="X1", rot_deg=90, crop=(0.1, 0.2, 0.5, 0.6),
                            quad_mm=UNIT_QUAD, height_mm=3.5, enabled=False)
        assert CameraPlacement.from_dict(p.to_dict()) == p
        c = StitchCalib(cams=[p], mm_per_px=1.5)
        assert StitchCalib.from_dict(c.to_dict()) == c

    def test_malformed_quad_is_treated_as_unplaced(self):
        assert CameraPlacement.from_dict({"quad_mm": [[0, 0], [1, 1]]}).quad_mm == ()
        assert CameraPlacement.from_dict({"quad_mm": "nope"}).quad_mm == ()
        assert CameraPlacement.from_dict({}).quad_mm == ()

    def test_merged_leaves_the_original_alone(self):
        p = CameraPlacement(serial="X1", height_mm=1.0)
        q = p.merged({"height_mm": 9.0})
        assert p.height_mm == 1.0 and q.height_mm == 9.0 and q.serial == "X1"

    def test_quad_area_detects_folded_corners(self):
        assert quad_area_mm2(UNIT_QUAD) == pytest.approx(400 * 300)
        folded = (UNIT_QUAD[1], UNIT_QUAD[0], UNIT_QUAD[2], UNIT_QUAD[3])
        assert quad_area_mm2(folded) <= 0

    def test_default_quads_sit_side_by_side(self):
        a = default_quad_mm((800.0, 600.0), (0, 0, 1, 1), 0)
        b = default_quad_mm((800.0, 600.0), (0, 0, 1, 1), 1)
        assert a[0] == (0.0, 0.0) and a[3] == (800.0, 600.0)
        assert b[0][0] == pytest.approx(800.0)      # parked one footprint right

    def test_default_quad_shrinks_with_the_crop(self):
        q = default_quad_mm((800.0, 600.0), (0.0, 0.0, 0.5, 0.5), 0)
        assert quad_area_mm2(q) == pytest.approx(400 * 300)

    def test_footprint_scales_with_distance(self):
        near, far = _flat_frame(depth=0.5), _flat_frame(depth=1.0)
        assert footprint_mm(far)[0] == pytest.approx(2.0 * footprint_mm(near)[0], rel=1e-6)


class TestRotateQuad:
    def test_four_turns_return_the_original(self):
        q = UNIT_QUAD
        for _ in range(4):
            q = rotate_quad(q, 1)
        assert np.allclose(np.asarray(q), np.asarray(UNIT_QUAD), atol=1e-6)

    def test_one_turn_swaps_the_proportions_in_place(self):
        q = np.asarray(rotate_quad(UNIT_QUAD, 1))
        width = np.hypot(*(q[1] - q[0]))
        height = np.hypot(*(q[2] - q[0]))
        assert width == pytest.approx(300.0)     # the 400x300 quad now stands up
        assert height == pytest.approx(400.0)
        # Turning about its own centre keeps the quad where it was.
        assert q.mean(axis=0) == pytest.approx(np.asarray(UNIT_QUAD).mean(axis=0))

    def test_turning_image_and_quad_together_keeps_the_picture_square(self):
        # A frame twice as wide as it is tall, placed on a matching quad.
        frames, calib = synthetic_scene(1)
        p = calib.cams[0].merged({"quad_mm": [list(c) for c in UNIT_QUAD]})
        turned = p.merged({"rot_deg": 90,
                           "quad_mm": [list(c) for c in rotate_quad(p.quad_mm, 1)]})
        a = stitch(frames, StitchCalib(cams=[p]))
        b = stitch(frames, StitchCalib(cams=[turned]))
        # Same patch of canvas covered, just stood on its side.
        assert a.valid.sum() == pytest.approx(b.valid.sum(), rel=0.05)
        assert a.depth_m[a.valid].mean() == pytest.approx(
            b.depth_m[b.valid].mean(), abs=0.002)

    def test_negative_steps_turn_the_other_way(self):
        assert np.allclose(np.asarray(rotate_quad(UNIT_QUAD, -1)),
                           np.asarray(rotate_quad(UNIT_QUAD, 3)), atol=1e-6)


class TestRequadForCrop:
    def test_cropping_leaves_the_sand_where_it_was(self):
        shape = (480, 640)
        p = CameraPlacement(crop=(0.0, 0.0, 1.0, 1.0), quad_mm=UNIT_QUAD)
        new_crop = (0.2, 0.1, 0.5, 0.6)
        new_quad = requad_for_crop(p, new_crop, shape)

        def canvas_of(pixel, crop, quad):
            h = cv2.getPerspectiveTransform(crop_corners_px(shape, crop),
                                            np.asarray(quad, np.float32))
            pt = np.asarray([[pixel]], np.float32)
            return cv2.perspectiveTransform(pt, h).reshape(2)

        # A pixel inside the new crop must land on the same millimetre as before.
        for pixel in ([200.0, 100.0], [400.0, 300.0]):
            assert canvas_of(pixel, new_crop, new_quad) == pytest.approx(
                canvas_of(pixel, p.crop, p.quad_mm), abs=1e-3)

    def test_unplaced_camera_has_nothing_to_recut(self):
        assert requad_for_crop(CameraPlacement(), (0.1, 0.1, 0.5, 0.5), (480, 640)) == ()


class TestStitch:
    def test_single_camera_roundtrip(self):
        r = _one(_flat_frame())
        assert r.valid.any()
        assert r.depth_m[r.valid].mean() == pytest.approx(0.8, abs=0.002)
        assert r.coverage.max() == 1 and not r.overlap.any()
        assert len(r.quads_px) == 1 and r.quads_px[0] is not None

    def test_more_cameras_widen_the_canvas(self):
        frames, calib = synthetic_scene(3, overlap_frac=0.08)
        one = stitch(frames[:1], StitchCalib(cams=calib.cams[:1]))
        three = stitch(frames, calib)
        assert _covered_width_mm(three) > 2.5 * _covered_width_mm(one)
        assert three.coverage.max() == 2       # neighbours only, never all three
        assert 0.02 < three.overlap.sum() / three.valid.sum() < 0.30

    def test_correct_placement_keeps_the_grooves_sharp(self):
        # The real test of a placement: two cameras that agree reinforce the
        # relief in the overlap; misaligned ones average it into mush.
        frames, calib = synthetic_scene(2, overlap_frac=0.30)
        good = stitch(frames, calib)
        off = calib.cams[1].merged(
            {"quad_mm": [[x, y + 60.0] for x, y in calib.cams[1].quad_mm]})
        bad = stitch(frames, StitchCalib(cams=[calib.cams[0], off]))
        assert good.overlap.sum() > 1000 and bad.overlap.sum() > 1000
        assert _groove_contrast(good, good.overlap) > \
            1.3 * _groove_contrast(bad, bad.overlap)

    def test_rgb_merges_with_gaps(self):
        frames, calib = synthetic_scene(2)
        r = stitch(frames, calib)
        assert r.rgb_valid.any()
        assert r.rgb[r.rgb_valid].mean() > 20      # colour actually landed
        assert not r.rgb_valid.all()               # the RGB-FOV gaps exist
        assert (r.rgb[~r.rgb_valid] == 0).all()

    def test_rotation_undoes_an_upside_down_mount(self):
        frames, calib = synthetic_scene(2, overlap_frac=0.15)
        f2 = frames[1]
        flipped = CameraFrame(f2.depth_m[::-1, ::-1].copy(),
                              f2.valid[::-1, ::-1].copy(), f2.intr,
                              f2.rgb[::-1, ::-1].copy(), f2.serial)
        fixed = StitchCalib(cams=[calib.cams[0], calib.cams[1].merged({"rot_deg": 180})])
        a = stitch(frames, calib)
        b = stitch([frames[0], flipped], fixed)
        assert abs(int(a.valid.sum()) - int(b.valid.sum())) / a.valid.sum() < 0.02
        assert _groove_contrast(b, b.overlap) == pytest.approx(
            _groove_contrast(a, a.overlap), rel=0.35)

    def test_cropping_shrinks_that_camera_only(self):
        frames, calib = synthetic_scene(2, overlap_frac=0.08)
        full = stitch(frames, calib)
        p = calib.cams[1]
        narrow = p.merged({"crop": [0.0, 0.0, 0.5, 1.0],
                           "quad_mm": [list(c) for c in requad_for_crop(
                               p, (0.0, 0.0, 0.5, 1.0), frames[1].depth_m.shape)]})
        cropped = stitch(frames, StitchCalib(cams=[calib.cams[0], narrow]))
        assert _covered_width_mm(cropped) < _covered_width_mm(full) * 0.85
        assert cropped.quads_px[0] is not None and cropped.quads_px[1] is not None

    def test_disabled_camera_leaves_the_canvas(self):
        frames, calib = synthetic_scene(2, overlap_frac=0.08)
        off = StitchCalib(cams=[calib.cams[0], calib.cams[1].merged({"enabled": False})])
        r = stitch(frames, off)
        assert r.quads_px[1] is None
        assert not r.overlap.any()
        assert _covered_width_mm(r) < _covered_width_mm(stitch(frames, calib)) * 0.7

    def test_all_cameras_disabled_is_an_empty_canvas(self):
        frames, calib = synthetic_scene(2)
        off = StitchCalib(cams=[c.merged({"enabled": False}) for c in calib.cams])
        r = stitch(frames, off)
        assert not r.valid.any()
        assert r.quads_px == [None, None]

    def test_height_nudge_shifts_only_that_camera(self):
        frames, calib = synthetic_scene(2, overlap_frac=0.0)
        base = stitch(frames, calib)
        raised = StitchCalib(cams=[calib.cams[0],
                                   calib.cams[1].merged({"height_mm": 20.0})])
        r = stitch(frames, raised)
        assert r.depth_m[r.valid].mean() > base.depth_m[base.valid].mean()

    def test_moving_a_quad_moves_its_footprint(self):
        frames, calib = synthetic_scene(2, overlap_frac=0.08)
        base = stitch(frames, calib)
        moved = calib.cams[1].merged(
            {"quad_mm": [[x, y + 100.0] for x, y in calib.cams[1].quad_mm]})
        r = stitch(frames, StitchCalib(cams=[calib.cams[0], moved]))
        dv = (np.mean([q[1] for q in r.quads_px[1]])
              - np.mean([q[1] for q in base.quads_px[1]]))
        assert dv * r.mm_per_px == pytest.approx(100.0, abs=25.0)

    def test_skewing_a_corner_keystones_the_footprint(self):
        frames, calib = synthetic_scene(1)
        quad = [list(c) for c in calib.cams[0].quad_mm]
        quad[1][1] -= 120.0                        # pull the top-right corner up
        r = stitch(frames, StitchCalib(cams=[calib.cams[0].merged({"quad_mm": quad})]))
        q = r.quads_px[0]
        assert (q[2][1] - q[0][1]) < (q[3][1] - q[1][1])   # right edge now taller
        assert r.valid.any()

    def test_folded_quad_falls_back_to_the_default(self):
        frames, calib = synthetic_scene(1)
        good = calib.cams[0].quad_mm
        # Swap left and right corners: the quad turns inside out.
        folded = calib.cams[0].merged(
            {"quad_mm": [list(good[1]), list(good[0]), list(good[3]), list(good[2])]})
        r = stitch(frames, StitchCalib(cams=[folded]))
        assert r.valid.any()
        assert _covered_width_mm(r) > 0.5 * _covered_width_mm(
            stitch(frames, StitchCalib(cams=[calib.cams[0]])))

    def test_explicit_grid_resolution_is_honoured(self):
        frames, calib = synthetic_scene(1)
        r = stitch(frames, StitchCalib(cams=calib.cams, mm_per_px=4.0))
        assert r.mm_per_px == pytest.approx(4.0)

    def test_auto_resolution_lands_near_native_detail(self):
        frames, calib = synthetic_scene(1)
        r = stitch(frames, calib)
        fw = footprint_mm(frames[0])[0]
        assert r.mm_per_px == pytest.approx(fw / frames[0].intr.width, rel=0.05)


class TestBindPlacements:
    def test_matches_saved_placements_by_serial(self):
        frames, _ = synthetic_scene(2)
        saved = StitchCalib(cams=[CameraPlacement(serial="SYN-2", height_mm=7.0),
                                  CameraPlacement(serial="SYN-1", height_mm=1.0)])
        bound = bind_placements(saved, frames)
        assert [c.serial for c in bound.cams] == ["SYN-1", "SYN-2"]
        assert [c.height_mm for c in bound.cams] == [1.0, 7.0]

    def test_falls_back_to_position_for_unlabelled_placements(self):
        frames, _ = synthetic_scene(2)
        saved = StitchCalib(cams=[CameraPlacement(height_mm=5.0),
                                  CameraPlacement(height_mm=6.0)])
        bound = bind_placements(saved, frames)
        assert [c.height_mm for c in bound.cams] == [5.0, 6.0]
        assert [c.serial for c in bound.cams] == ["SYN-1", "SYN-2"]

    def test_new_camera_arrives_unplaced(self):
        frames, _ = synthetic_scene(3)
        saved = StitchCalib(cams=[CameraPlacement(serial="SYN-1", height_mm=2.0)])
        bound = bind_placements(saved, frames)
        assert len(bound.cams) == 3
        assert bound.cams[0].height_mm == 2.0
        assert bound.cams[2].quad_mm == ()      # caller fills in a default

    def test_unplugged_camera_drops_out(self):
        frames, calib = synthetic_scene(3)
        bound = bind_placements(calib, frames[:2])
        assert [c.serial for c in bound.cams] == ["SYN-1", "SYN-2"]

    def test_placement_lookup_prefers_serial_over_order(self):
        calib = StitchCalib(cams=[CameraPlacement(serial="A", height_mm=1.0),
                                  CameraPlacement(serial="B", height_mm=2.0)])
        assert calib.placement_for("B", 0).height_mm == 2.0
        # An unknown serial must not borrow another camera's placement.
        assert calib.placement_for("C", 0).height_mm == 0.0

    def test_with_camera_returns_a_new_calib(self):
        calib = StitchCalib(cams=[CameraPlacement(serial="A")])
        other = calib.with_camera(0, calib.cams[0].merged({"height_mm": 4.0}))
        assert calib.cams[0].height_mm == 0.0 and other.cams[0].height_mm == 4.0
        assert calib.with_camera(9, CameraPlacement()) is calib


class TestFillHoles:
    def test_fills_isolated_hole_keeps_large_gap(self):
        h = np.full((20, 20), 5.0, np.float32)
        v = np.ones((20, 20), bool)
        v[10, 10] = False                    # isolated pinhole
        v[0:20, 0:3] = False                 # wide gap
        hf, vf = fill_small_holes(h, v, iters=2)
        assert vf[10, 10] and hf[10, 10] == pytest.approx(5.0)
        assert not vf[5, 0]                  # interior of the wide gap stays invalid
