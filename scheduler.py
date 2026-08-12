"""
scheduler.py — the ledger of toolpaths that have gone out to the robot.

Pure filesystem reading: scan paths/ for saved bundles and turn each one into a
numbered row (index, path, when). No hardware, no robot, no server, and nothing
from main.py — the Scheduler is a CONTAINED tool and this module is the whole
of what it knows.

About the timestamp. A bundle carries no separate "executed at" record, because
nothing in the pipeline writes one. What every bundle DOES carry is when it was
SAVED, encoded in its folder name (`YYYY-MM-DD_HH-MM-SS`, with `_2`, `_3`, …
appended when two saves land in the same second). In Participant Mode the save
happens immediately before the robot runs, so save time is run time to within a
second; in Developer Mode it is the moment the operator pressed Save Path, which
may be before, after, or instead of a run. Three sources are tried in order of
how much they can be trusted — the folder name, then path.json's `saved` field,
then the folder's own modification date — and every row reports which one it
used, so a reader is never left guessing how solid the time is.

What counts as a bundle is NOT decided here: `toolpath_loader.list_toolpaths`
already owns that rule (a folder holding path.json), and the
Scheduler defers to it so the two tools can never disagree about what is on
disk.
"""
from __future__ import annotations

import csv
import io
import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

from config import PATHS_DIR
from toolpath_loader import list_toolpaths

# path_export names folders "%Y-%m-%d_%H-%M-%S", plus "_2", "_3", … when two
# saves land inside the same second. The suffix is a disambiguator, not a time.
_FOLDER_TIME_RE = re.compile(r"^(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})(?:_(\d+))?$")

FOLDER_NAME = "folder name"
PATH_JSON = "path.json"
FILE_DATE = "file date"
UNKNOWN = "unknown"

# The detection image save_bundle writes beside the path. Bundles saved before
# it existed simply do not have one, which is a blank cell, not an error.
MASK_FILE = "mask.png"


@dataclass
class ScheduleRow:
    """One executed path: exactly the three columns, plus how we know them."""
    index: int                              # 1-based, chronological
    name: str                               # the bundle folder's name
    folder: Path
    executed_at: Optional[datetime]
    time_source: str = UNKNOWN
    files: list[str] = field(default_factory=list)

    @property
    def when(self) -> str:
        """The date and time as shown in the table; blank when unknown."""
        return self.executed_at.strftime("%Y-%m-%d %H:%M:%S") if self.executed_at else ""

    @property
    def has_mask(self) -> bool:
        return MASK_FILE in self.files

    @property
    def mask_path(self) -> str:
        """Where the mask lives on disk; blank when this bundle has none."""
        return str(self.folder / MASK_FILE) if self.has_mask else ""

    def to_dict(self) -> dict:
        return {
            "index": self.index,
            "name": self.name,
            "path": str(self.folder),
            "executed_at": self.when,
            "time_source": self.time_source,
            "files": list(self.files),
            "has_mask": self.has_mask,
        }


def parse_folder_time(name: str) -> Optional[datetime]:
    """The save timestamp encoded in a bundle folder name, or None."""
    m = _FOLDER_TIME_RE.match(name or "")
    if not m:
        return None
    try:
        return datetime.strptime(m.group(1), "%Y-%m-%d_%H-%M-%S")
    except ValueError:
        return None


def _json_saved_time(folder: Path) -> Optional[datetime]:
    """path.json's `meta.saved` ("%Y-%m-%d %H:%M:%S"), written by save_bundle."""
    import json
    try:
        data = json.loads((folder / "path.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, UnicodeDecodeError):
        return None
    saved = (data.get("meta") or {}).get("saved") if isinstance(data, dict) else None
    if not isinstance(saved, str):
        return None
    try:
        return datetime.strptime(saved, "%Y-%m-%d %H:%M:%S")
    except ValueError:
        return None


def _when(folder: Path) -> tuple[Optional[datetime], str]:
    """Best available time for a bundle, and which source produced it."""
    stamp = parse_folder_time(folder.name)
    if stamp:
        return stamp, FOLDER_NAME
    stamp = _json_saved_time(folder)
    if stamp:
        return stamp, PATH_JSON
    try:
        return datetime.fromtimestamp(folder.stat().st_mtime), FILE_DATE
    except OSError:
        return None, UNKNOWN


def _files_in(folder: Path) -> list[str]:
    """The bundle's own files, sorted — what the row was built from."""
    try:
        return sorted(p.name for p in folder.iterdir() if p.is_file())
    except OSError:
        return []


def read_schedule(base_dir: Path | None = None) -> list[ScheduleRow]:
    """
    Every saved bundle under ``base_dir`` (default paths/) as a numbered row,
    OLDEST FIRST so row 1 is the first path that went out and the newest lands
    at the bottom — a ledger reads forwards. Bundles whose time cannot be
    established at all sort last rather than being dropped: a path that exists
    on disk belongs in the list even when its date does not.
    """
    base = Path(base_dir) if base_dir is not None else PATHS_DIR
    rows: list[ScheduleRow] = []
    for entry in list_toolpaths(base):
        folder = base / entry["name"]
        stamp, source = _when(folder)
        rows.append(ScheduleRow(index=0, name=entry["name"], folder=folder,
                                executed_at=stamp, time_source=source,
                                files=_files_in(folder)))
    rows.sort(key=lambda r: (r.executed_at is None, r.executed_at or datetime.min,
                             r.name))
    for i, row in enumerate(rows, start=1):
        row.index = i
    return rows


def to_csv(rows: list[ScheduleRow]) -> str:
    """
    The same columns as the table, for a real spreadsheet. A CSV cannot hold
    the mask picture, so it carries the file's location instead — enough to
    open or link it from wherever the sheet ends up.
    """
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(["#", "Path executed", "Date and time", "Mask"])
    for row in rows:
        w.writerow([row.index, row.name, row.when, row.mask_path])
    return buf.getvalue()
