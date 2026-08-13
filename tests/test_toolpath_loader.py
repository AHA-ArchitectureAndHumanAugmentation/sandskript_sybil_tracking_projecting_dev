"""
Unit tests for toolpath_loader.py — reading saved bundles back (replay tool).
Round-trips through path_export so the parsers track the real writer. No hardware.
"""
import math
import os

import pytest

from path_export import build_urscript, save_bundle
from toolpath_loader import list_toolpaths, load_toolpath, parse_json, parse_urscript

_PI = math.pi

STROKES = [
    [[0.4, 0.0, 0.2, 0.0, _PI, 0.0], [0.45, 0.0, 0.2, 0.0, _PI, 0.0]],
    [[0.4, 0.1, 0.2, 0.0, _PI, 0.0], [0.4, 0.15, 0.2, 0.0, _PI, 0.0],
     [0.4, 0.2, 0.2, 0.0, _PI, 0.0]],
]


def _assert_strokes_equal(a, b, tol=1e-5):
    assert len(a) == len(b)
    for sa, sb in zip(a, b):
        assert len(sa) == len(sb)
        for pa, pb in zip(sa, sb):
            assert pa == pytest.approx(pb, abs=tol)


# ─────────────────────────────────────────────────────────────────────────────
# URScript parsing (round-trip through the exporter)
# ─────────────────────────────────────────────────────────────────────────────

class TestParseUrscript:

    def test_roundtrip_strokes(self):
        script = build_urscript(STROKES, speed=0.3, safety=0.05, blend_m=0.003)
        strokes, _ = parse_urscript(script)
        _assert_strokes_equal(strokes, STROKES)

    def test_meta_reconstructed(self):
        script = build_urscript(STROKES, speed=0.3, safety=0.05, blend_m=0.003,
                                header="mode: surface  surface: dune.stl")
        _, meta = parse_urscript(script)
        assert meta["speed_pct"] == pytest.approx(30.0)
        assert meta["safety_mm"] == pytest.approx(50.0, abs=0.1)
        assert meta["blend_mm"] == pytest.approx(3.0)
        assert meta["mode"] == "surface"

    def test_single_point_stroke(self):
        # 1-pt stroke: movel approach + movel start + movel lift, no movep.
        script = build_urscript([[STROKES[0][0]]], speed=0.3, safety=0.05)
        strokes, _ = parse_urscript(script)
        assert len(strokes) == 1 and len(strokes[0]) == 1
        assert strokes[0][0] == pytest.approx(STROKES[0][0], abs=1e-5)

    def test_heuristic_without_stroke_markers(self):
        # Hand-edited scripts may lose the "# stroke" comments; multi-point
        # strokes are still recovered from the movep runs.
        script = build_urscript(STROKES, speed=0.3, safety=0.05)
        stripped = "\n".join(l for l in script.splitlines()
                             if not l.strip().startswith("#"))
        strokes, _ = parse_urscript(stripped)
        _assert_strokes_equal(strokes, STROKES)

    def test_malformed_pose_raises(self):
        with pytest.raises(ValueError):
            parse_urscript("def f():\n  movel(a=0.5, v=0.3)\nend\nf()\n")


# ─────────────────────────────────────────────────────────────────────────────
# JSON parsing
# ─────────────────────────────────────────────────────────────────────────────

class TestParseJson:

    def test_invalid_json_raises(self):
        with pytest.raises(ValueError):
            parse_json("{not json")

    def test_missing_strokes_raises(self):
        with pytest.raises(ValueError):
            parse_json('{"meta": {}}')

    def test_bad_pose_raises(self):
        with pytest.raises(ValueError):
            parse_json('{"strokes": [[{"pose": [1, 2, 3]}]]}')


# ─────────────────────────────────────────────────────────────────────────────
# Bundle loading (through save_bundle output on disk)
# ─────────────────────────────────────────────────────────────────────────────

class TestLoadToolpath:

    def _bundle(self, tmp_path, **kw):
        return save_bundle(STROKES, speed=0.3, safety_m=0.05, offset_m=0.0,
                           meta={"mode": "surface", "speed_pct": 30.0,
                                 "safety_mm": 50.0, "blend_mm": 0.5},
                           base_dir=tmp_path, **kw)

    def test_json_preferred_by_default(self, tmp_path):
        folder = self._bundle(tmp_path)
        tp = load_toolpath(folder)
        assert tp.source == "path.json"
        assert tp.meta["mode"] == "surface"
        assert tp.name == folder.name
        assert tp.stroke_count == 2 and tp.point_count == 5
        _assert_strokes_equal(tp.strokes, STROKES)

    def test_script_on_request_matches_json(self, tmp_path):
        folder = self._bundle(tmp_path)
        tp = load_toolpath(folder, prefer="script")
        assert tp.source == "path.script"
        _assert_strokes_equal(tp.strokes, STROKES)

    def test_script_only_bundle_falls_back(self, tmp_path):
        folder = self._bundle(tmp_path)
        (folder / "path.json").unlink()
        tp = load_toolpath(folder)
        assert tp.source == "path.script"

    def test_missing_files_raise(self, tmp_path):
        empty = tmp_path / "empty"
        empty.mkdir()
        with pytest.raises(ValueError):
            load_toolpath(empty)
        folder = self._bundle(tmp_path)
        (folder / "path.script").unlink()
        with pytest.raises(ValueError):
            load_toolpath(folder, prefer="script")


# ─────────────────────────────────────────────────────────────────────────────
# Listing
# ─────────────────────────────────────────────────────────────────────────────

class TestListToolpaths:

    def test_lists_newest_first_with_flags(self, tmp_path):
        older = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path)
        newer = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path)
        (newer / "path.json").unlink()
        (tmp_path / "not_a_bundle").mkdir()          # no path files → skipped
        (tmp_path / "loose.txt").write_text("x")     # plain file → skipped
        os.utime(older, (1_000_000, 1_000_000))
        os.utime(newer, (2_000_000, 2_000_000))

        entries = list_toolpaths(tmp_path)
        assert [e["name"] for e in entries] == [newer.name, older.name]
        assert entries[0] == {"name": newer.name, "has_json": False,
                              "has_script": True, "has_preview": False}
        assert entries[1]["has_json"] and entries[1]["has_script"]

    def test_missing_base_dir(self, tmp_path):
        assert list_toolpaths(tmp_path / "nope") == []
