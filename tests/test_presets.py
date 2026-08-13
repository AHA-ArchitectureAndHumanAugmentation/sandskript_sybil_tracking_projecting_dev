"""
Unit tests for the Detection-Parameter preset filename guard (server.py).
Presets may be renamed to arbitrary filenames — the guard must accept those
while still rejecting path traversal. No hardware / no running server.
"""
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
