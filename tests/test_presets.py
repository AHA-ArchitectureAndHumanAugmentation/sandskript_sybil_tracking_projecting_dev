"""
Unit tests for the Detection-Parameter presets (server.py): the filename guard
and the save→load round trip. Presets may be renamed to arbitrary filenames —
the guard must accept those while still rejecting path traversal — and a preset
now carries the Path Preview bar alongside the sliders, which the server must
persist verbatim. No hardware / no running server.
"""
import asyncio
import json
import threading
from pathlib import Path

import pytest

import server


@pytest.fixture
def presets_dir(tmp_path, monkeypatch):
    d = tmp_path / "presets"
    d.mkdir()
    monkeypatch.setattr(server, "PRESETS_DIR", d)
    return d


class TestSafePresetPath:

    def test_default_timestamp_name(self, presets_dir):
        p = server._safe_preset_path("2026-07-23_15-30-00.json")
        assert p == (presets_dir / "2026-07-23_15-30-00.json").resolve()

    def test_custom_names_accepted(self, presets_dir):
        for name in ["dune ridges.json", "My Preset (v2).json",
                     "fine.détaillé.json", "coarse_band.JSON"]:
            assert server._safe_preset_path(name) is not None, name

    def test_requires_json_suffix(self, presets_dir):
        assert server._safe_preset_path("preset.txt") is None
        assert server._safe_preset_path("preset") is None

    def test_rejects_traversal_and_separators(self, presets_dir):
        for name in ["../secret.json", "..\\secret.json", "sub/child.json",
                     "sub\\child.json", "/etc/passwd.json", "a\x00.json", ""]:
            assert server._safe_preset_path(name) is None, name

    def test_resolved_path_stays_in_presets_dir(self, presets_dir):
        p = server._safe_preset_path("valid name.json")
        assert p is not None
        assert p.parent == presets_dir.resolve()


class FakeRequest:
    """Just enough of aiohttp's Request for the two preset handlers."""

    def __init__(self, payload=None, name=""):
        self._payload = payload
        self.match_info = {"name": name}

    async def json(self):
        if self._payload is None:
            raise ValueError("no body")
        return self._payload


def _server():
    return server.Server({}, threading.Lock(),
                         on_connect=None, on_disconnect=None)


def _body(response):
    return json.loads(response.body.decode())


class TestPresetRoundTrip:
    """
    A preset holds the detection sliders (flat keys) plus the Path Preview bar
    under `exec`. The server is deliberately schema-free about that — it stores
    what the browser posted — so these tests pin the property that matters:
    what goes in comes back out unchanged, old files included.
    """

    def test_save_then_load_returns_the_same_params(self, presets_dir):
        srv = _server()
        params = {
            "min_depth": 3.5, "min_length": 40, "detect": "valley",
            "exec": {"spacing_mm": 25, "join_mm": 30, "blend_mm": 12.5,
                     "speed_pct": 40, "offset_mm": -2, "safety_mm": 60,
                     "max_length_mm": 8000},
        }
        saved = _body(asyncio.run(
            srv._handle_presets_save(FakeRequest({"params": params}))))
        assert saved["ok"]

        loaded = _body(asyncio.run(
            srv._handle_presets_get(FakeRequest(name=saved["name"]))))
        assert loaded["ok"]
        assert loaded["params"] == params

    def test_exec_block_survives_verbatim(self, presets_dir):
        """Every bar control must be in the file, not just the ones that fit."""
        srv = _server()
        bar = {"spacing_mm": 10, "join_mm": 0, "blend_mm": 50,
               "speed_pct": 5, "offset_mm": 0, "safety_mm": 50,
               "max_length_mm": 0}
        name = _body(asyncio.run(srv._handle_presets_save(
            FakeRequest({"params": {"exec": bar}}))))["name"]
        on_disk = json.loads((presets_dir / name).read_text(encoding="utf-8"))
        assert on_disk["exec"] == bar

    def test_preset_without_exec_still_loads(self, presets_dir):
        """Files written before the bar was included must not become errors."""
        (presets_dir / "legacy.json").write_text(
            json.dumps({"min_depth": 2.0, "detect": "ridge"}), encoding="utf-8")
        srv = _server()
        loaded = _body(asyncio.run(
            srv._handle_presets_get(FakeRequest(name="legacy.json"))))
        assert loaded["ok"]
        assert "exec" not in loaded["params"]
        assert loaded["params"]["detect"] == "ridge"

    def test_two_saves_in_the_same_second_do_not_collide(self, presets_dir):
        srv = _server()
        names = {_body(asyncio.run(srv._handle_presets_save(
                    FakeRequest({"params": {"exec": {"speed_pct": i}}}))))["name"]
                 for i in range(3)}
        assert len(names) == 3
        assert len(list(presets_dir.glob("*.json"))) == 3
