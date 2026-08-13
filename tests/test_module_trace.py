"""
Unit tests for module_trace.py — console module attribution. Pure formatting,
no hardware, no app import.
"""
import pytest

import module_trace
from module_trace import FEATURES, STAGES, banner, chain, log


class TestChain:

    def test_known_action_lists_its_modules(self):
        assert chain("save") == "path_export.py"

    def test_multi_module_action_is_arrow_joined(self):
        assert chain("capture") == "camera_thread.py → depth_extractor.py"

    def test_unknown_action_is_empty_not_an_error(self):
        assert chain("no-such-action") == ""

    def test_extra_modules_are_appended(self):
        out = chain("generate", extra=("surface", "reach"))
        assert out == "depth_extractor.py → path_extractor.py → surface.py → reach.py"

    def test_extra_does_not_duplicate_an_existing_module(self):
        assert chain("save", extra=("path_export",)) == "path_export.py"

    def test_unknown_action_with_extra_still_works(self):
        assert chain("no-such-action", extra=("workspace",)) == "workspace.py"

    def test_every_stage_module_appears_in_the_feature_table(self):
        """A stage naming a module absent from FEATURES means the banner lies."""
        known = {m for mods in FEATURES.values() for m in mods}
        for action, mods in STAGES.items():
            for m in mods:
                assert m in known, f"{action} names {m}.py, missing from FEATURES"


class TestLog:

    def test_prints_message_then_chain(self, capsys, monkeypatch):
        monkeypatch.setattr(module_trace, "SHOW_MODULE_TRACE", True)
        log("save", "[save] toolpath saved to paths/x")
        out = capsys.readouterr().out.splitlines()
        assert out[0] == "[save] toolpath saved to paths/x"
        assert out[1] == "  └ path_export.py"

    def test_disabled_prints_only_the_message(self, capsys, monkeypatch):
        monkeypatch.setattr(module_trace, "SHOW_MODULE_TRACE", False)
        log("save", "[save] done")
        assert capsys.readouterr().out == "[save] done\n"

    def test_unknown_action_prints_message_without_a_trail(self, capsys, monkeypatch):
        monkeypatch.setattr(module_trace, "SHOW_MODULE_TRACE", True)
        log("mystery", "something happened")
        assert capsys.readouterr().out == "something happened\n"


class TestBanner:

    def test_lists_every_feature(self):
        text = banner()
        for feature in FEATURES:
            assert feature in text

    def test_marks_loaded_modules(self):
        """module_trace itself imports config, so config.py must read as loaded."""
        assert "✓ config.py" in banner()

    def test_marks_unloaded_modules(self, monkeypatch):
        monkeypatch.setitem(module_trace.FEATURES, "Fake", ("definitely_not_imported",))
        assert "· definitely_not_imported.py" in banner()
        module_trace.FEATURES.pop("Fake", None)

    def test_print_banner_respects_the_flag(self, capsys, monkeypatch):
        monkeypatch.setattr(module_trace, "SHOW_MODULE_BANNER", False)
        module_trace.print_banner()
        assert capsys.readouterr().out == ""
