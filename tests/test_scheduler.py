"""Unit tests for scheduler.py — the paths/ ledger. No hardware, no server."""
import json
from datetime import datetime

from scheduler import (
    FILE_DATE, FOLDER_NAME, PATH_JSON, parse_folder_time, read_schedule, to_csv,
)


def _bundle(base, name, files=("path.script",), meta=None):
    """A folder that list_toolpaths will accept as a bundle."""
    folder = base / name
    folder.mkdir(parents=True, exist_ok=True)
    for f in files:
        if f == "path.json":
            folder.joinpath(f).write_text(
                json.dumps({"meta": meta or {}, "strokes": []}), encoding="utf-8")
        else:
            folder.joinpath(f).write_text("# stub", encoding="utf-8")
    return folder


class TestParseFolderTime:
    def test_reads_the_exporter_format(self):
        assert parse_folder_time("2026-08-10_14-42-31") == datetime(2026, 8, 10, 14, 42, 31)

    def test_same_second_suffix_is_not_part_of_the_time(self):
        # save_bundle appends _2, _3 … when two saves land in one second.
        assert parse_folder_time("2026-08-10_14-42-31_2") == \
               parse_folder_time("2026-08-10_14-42-31")

    def test_rejects_anything_else(self):
        for name in ("", "notes", "2026-08-10", "2026-13-40_99-99-99", "x_2026-08-10_14-42-31"):
            assert parse_folder_time(name) is None


class TestReadSchedule:
    def test_numbers_from_one_oldest_first(self, tmp_path):
        for name in ("2026-08-10_09-00-00", "2026-08-09_18-30-00", "2026-08-10_14-42-31"):
            _bundle(tmp_path, name)
        rows = read_schedule(tmp_path)
        assert [r.index for r in rows] == [1, 2, 3]
        assert [r.name for r in rows] == ["2026-08-09_18-30-00",
                                          "2026-08-10_09-00-00",
                                          "2026-08-10_14-42-31"]

    def test_the_three_columns(self, tmp_path):
        _bundle(tmp_path, "2026-08-10_14-42-31")
        (row,) = read_schedule(tmp_path)
        assert row.index == 1
        assert row.name == "2026-08-10_14-42-31"
        assert row.when == "2026-08-10 14:42:31"

    def test_lists_the_files_the_bundle_holds(self, tmp_path):
        _bundle(tmp_path, "2026-08-10_14-42-31",
                files=("path.script", "path.json", "mask.png", "skeleton.png"))
        (row,) = read_schedule(tmp_path)
        assert row.files == ["mask.png", "path.json", "path.script", "skeleton.png"]

    def test_folder_name_wins_over_path_json(self, tmp_path):
        # A bundle copied under a new name must report the name, not the meta.
        _bundle(tmp_path, "2026-08-10_14-42-31", files=("path.json",),
                meta={"saved": "1999-01-01 00:00:00"})
        (row,) = read_schedule(tmp_path)
        assert row.time_source == FOLDER_NAME
        assert row.when == "2026-08-10 14:42:31"

    def test_falls_back_to_path_json(self, tmp_path):
        _bundle(tmp_path, "renamed-by-hand", files=("path.json",),
                meta={"saved": "2026-08-10 14:42:31"})
        (row,) = read_schedule(tmp_path)
        assert row.time_source == PATH_JSON
        assert row.when == "2026-08-10 14:42:31"

    def test_falls_back_to_the_file_date(self, tmp_path):
        _bundle(tmp_path, "renamed-by-hand", files=("path.script",))
        (row,) = read_schedule(tmp_path)
        assert row.time_source == FILE_DATE
        assert row.when                     # something, from the filesystem

    def test_a_malformed_path_json_does_not_lose_the_row(self, tmp_path):
        folder = tmp_path / "renamed-by-hand"
        folder.mkdir()
        folder.joinpath("path.json").write_text("{not json", encoding="utf-8")
        (row,) = read_schedule(tmp_path)
        assert row.time_source == FILE_DATE   # the row survives, dated loosely

    def test_ignores_folders_that_are_not_bundles(self, tmp_path):
        _bundle(tmp_path, "2026-08-10_14-42-31")
        (tmp_path / "notes").mkdir()                     # no path.json/.script
        (tmp_path / "2026-08-10_15-00-00").mkdir()       # empty
        (tmp_path / "stray.txt").write_text("x", encoding="utf-8")
        rows = read_schedule(tmp_path)
        assert [r.name for r in rows] == ["2026-08-10_14-42-31"]

    def test_a_new_bundle_appears_on_the_next_scan(self, tmp_path):
        # This is what the server's watch loop diffs to decide whether to push,
        # so a path saved by the main app shows up without a browser reload.
        _bundle(tmp_path, "2026-08-10_14-42-31")
        assert len(read_schedule(tmp_path)) == 1
        _bundle(tmp_path, "2026-08-10_15-00-00")
        rows = read_schedule(tmp_path)
        assert [r.index for r in rows] == [1, 2]
        assert rows[-1].name == "2026-08-10_15-00-00"

    def test_missing_paths_folder_is_empty_not_an_error(self, tmp_path):
        assert read_schedule(tmp_path / "nope") == []

    def test_same_second_bundles_both_appear_in_a_stable_order(self, tmp_path):
        _bundle(tmp_path, "2026-08-10_14-42-31_2")
        _bundle(tmp_path, "2026-08-10_14-42-31")
        rows = read_schedule(tmp_path)
        assert [r.name for r in rows] == ["2026-08-10_14-42-31",
                                          "2026-08-10_14-42-31_2"]
        assert [r.index for r in rows] == [1, 2]


class TestMaskColumn:
    def test_a_bundle_with_a_mask_reports_one(self, tmp_path):
        _bundle(tmp_path, "2026-08-10_14-42-31",
                files=("path.script", "mask.png"))
        (row,) = read_schedule(tmp_path)
        assert row.has_mask is True
        assert row.mask_path == str(tmp_path / "2026-08-10_14-42-31" / "mask.png")
        assert row.to_dict()["has_mask"] is True

    def test_an_older_bundle_has_no_mask(self, tmp_path):
        # Everything saved before mask.png existed: a blank cell, not an error.
        _bundle(tmp_path, "2026-07-13_16-56-25",
                files=("path.script", "path.json", "preview.png"))
        (row,) = read_schedule(tmp_path)
        assert row.has_mask is False
        assert row.mask_path == ""
        assert row.to_dict()["has_mask"] is False

    def test_skeleton_alone_is_not_a_mask(self, tmp_path):
        _bundle(tmp_path, "2026-08-10_14-42-31",
                files=("path.script", "skeleton.png"))
        (row,) = read_schedule(tmp_path)
        assert row.has_mask is False


class TestMaskRoute:
    """The /mask/{name} route resolves a name under paths/ and nothing else."""

    @staticmethod
    def _server(base):
        from scheduler_server import SchedulerServer
        return SchedulerServer(base_dir=base)

    def test_finds_a_real_bundle(self, tmp_path):
        _bundle(tmp_path, "2026-08-10_14-42-31", files=("path.script", "mask.png"))
        folder = self._server(tmp_path)._safe_folder("2026-08-10_14-42-31")
        assert folder is not None and (folder / "mask.png").is_file()

    def test_refuses_anything_that_could_escape_paths(self, tmp_path):
        s = self._server(tmp_path)
        for name in ("", "..", "../..", "..\\secrets", "a/b", "a\\b",
                     "2026-08-10_14-42-31/../.."):
            assert s._safe_folder(name) is None

    def test_unknown_name_is_not_a_folder(self, tmp_path):
        assert self._server(tmp_path)._safe_folder("nope") is None


class TestCsv:
    def test_header_and_rows(self, tmp_path):
        _bundle(tmp_path, "2026-08-09_18-30-00")
        _bundle(tmp_path, "2026-08-10_14-42-31", files=("path.script", "mask.png"))
        lines = to_csv(read_schedule(tmp_path)).strip().split("\n")
        assert lines[0] == "#,Path executed,Date and time,Mask"
        assert lines[1] == "1,2026-08-09_18-30-00,2026-08-09 18:30:00,"
        mask = str(tmp_path / "2026-08-10_14-42-31" / "mask.png")
        assert lines[2] == f"2,2026-08-10_14-42-31,2026-08-10 14:42:31,{mask}"

    def test_empty_schedule_still_has_a_header(self, tmp_path):
        assert to_csv(read_schedule(tmp_path)).strip() == \
               "#,Path executed,Date and time,Mask"


class TestContainment:
    def test_scheduler_does_not_drag_in_the_app(self):
        """
        The whole point of a contained tool: importing it must not start a
        camera thread, open a robot, or pull the main app in behind it.

        Checked in a FRESH interpreter — asserting on this process's sys.modules
        would only prove that some earlier test in the session had imported
        camera_thread, which says nothing about the scheduler.
        """
        import subprocess
        import sys
        from pathlib import Path

        code = (
            "import sys, scheduler, scheduler_server;"
            "bad=[m for m in ('main','camera_thread','robot_controller',"
            "'pyrealsense2','rtde_control') if m in sys.modules];"
            "print(','.join(bad))"
        )
        out = subprocess.run([sys.executable, "-c", code],
                             cwd=str(Path(__file__).resolve().parent.parent),
                             capture_output=True, text=True, timeout=60)
        assert out.returncode == 0, out.stderr
        assert out.stdout.strip() == "", f"scheduler pulled in {out.stdout.strip()}"
