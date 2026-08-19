"""
Unit tests for toolpath_loader.py — reading saved bundles back (replay tool).
Round-trips through path_export so the parser tracks the real writer.
No hardware.
"""
import math
import os

import pytest

from path_export import save_bundle
from toolpath_loader import list_toolpaths, load_toolpath, parse_json

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
                           meta={"mode": "surface", "speed_pct": 30.0},
                           base_dir=tmp_path, **kw)

    def test_loads_the_json(self, tmp_path):
        folder = self._bundle(tmp_path)
        tp = load_toolpath(folder)
        assert tp.source == "path.json"
        assert tp.meta["mode"] == "surface"
        assert tp.name == folder.name
        assert tp.stroke_count == 2 and tp.point_count == 5
        _assert_strokes_equal(tp.strokes, STROKES)

    def test_run_parameters_survive_into_meta(self, tmp_path):
        # These are what the replay UI prefills Speed/Safety/Radius from.
        folder = self._bundle(tmp_path, blend_m=0.003)
        meta = load_toolpath(folder).meta
        assert meta["speed_mps"] == pytest.approx(0.3)
        assert meta["safety_mm"] == pytest.approx(50.0)
        assert meta["blend_mm"] == pytest.approx(3.0)

    def test_explicit_json_is_the_same(self, tmp_path):
        folder = self._bundle(tmp_path)
        assert load_toolpath(folder, prefer="json").source == "path.json"

    def test_an_unknown_format_is_refused(self, tmp_path):
        # "script" used to be valid; asking for it now is a caller bug, not a
        # silent fallback to something that might not be the same motion.
        folder = self._bundle(tmp_path)
        with pytest.raises(ValueError):
            load_toolpath(folder, prefer="script")

    def test_missing_json_raises(self, tmp_path):
        empty = tmp_path / "empty"
        empty.mkdir()
        with pytest.raises(ValueError):
            load_toolpath(empty)

    def test_a_urscript_only_folder_is_not_loadable(self, tmp_path):
        # Every bundle carries a path.script as a readable record, but it is
        # never parsed back — a GoFa cannot run UR motion. So a folder holding
        # ONLY that file is not a bundle.
        folder = tmp_path / "2026-01-01_00-00-00"
        folder.mkdir()
        (folder / "path.script").write_text("def draw_path():\nend\n", encoding="utf-8")
        with pytest.raises(ValueError):
            load_toolpath(folder)


# ─────────────────────────────────────────────────────────────────────────────
# Listing
# ─────────────────────────────────────────────────────────────────────────────

class TestListToolpaths:

    def test_lists_newest_first_with_flags(self, tmp_path):
        older = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path)
        newer = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path)
        (tmp_path / "not_a_bundle").mkdir()          # no path files → skipped
        (tmp_path / "loose.txt").write_text("x")     # plain file → skipped
        os.utime(older, (1_000_000, 1_000_000))
        os.utime(newer, (2_000_000, 2_000_000))

        entries = list_toolpaths(tmp_path)
        assert [e["name"] for e in entries] == [newer.name, older.name]
        assert entries[0] == {"name": newer.name, "has_json": True,
                              "has_preview": False}

    def test_a_urscript_only_folder_is_not_listed(self, tmp_path):
        # Same rule at listing time: path.json is what makes a bundle.
        save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path)
        legacy = tmp_path / "2026-01-01_00-00-00"
        legacy.mkdir()
        (legacy / "path.script").write_text("def draw_path():\nend\n", encoding="utf-8")
        assert [e["name"] for e in list_toolpaths(tmp_path)] != [legacy.name]
        assert legacy.name not in [e["name"] for e in list_toolpaths(tmp_path)]

    def test_missing_base_dir(self, tmp_path):
        assert list_toolpaths(tmp_path / "nope") == []
