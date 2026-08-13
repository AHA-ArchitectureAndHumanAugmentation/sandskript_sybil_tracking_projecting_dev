#!/usr/bin/env python
"""PreCompact hook: back up the transcript before compaction summarizes it.

Reads the hook payload (JSON) from stdin, copies `transcript_path` to
`.claude/transcript-backups/<timestamp>_<trigger>.jsonl`, then prunes to the
newest MAX_BACKUPS. Always exits 0 -- a backup failure must never block or
delay compaction.

Why this exists: this repo's CLAUDE.md maintenance rule makes API-change
sessions long, multi-file and reasoning-heavy (which WS/HTTP message changed,
which MCP tool wraps it, what reply field). Those are exactly the sessions that
hit compaction; the backup lets that reasoning be recovered if the summary
drops it.
"""
import sys
import os
import json
import glob
import shutil
from datetime import datetime

MAX_BACKUPS = 20


def main() -> None:
    try:
        payload = json.load(sys.stdin)
    except Exception:
        return  # no / invalid payload -> nothing to back up

    transcript = payload.get("transcript_path")
    if not transcript or not os.path.isfile(transcript):
        return

    project_dir = os.environ.get("CLAUDE_PROJECT_DIR") or os.getcwd()
    backup_dir = os.path.join(project_dir, ".claude", "transcript-backups")
    try:
        os.makedirs(backup_dir, exist_ok=True)
    except OSError:
        return

    stamp = datetime.now().strftime("%Y-%m-%dT%H-%M-%S-%f")
    trigger = str(payload.get("trigger", "compact"))
    dest = os.path.join(backup_dir, f"{stamp}_{trigger}.jsonl")
    try:
        shutil.copy2(transcript, dest)
    except Exception:
        return

    # Prune: keep only the newest MAX_BACKUPS by mtime.
    backups = sorted(
        glob.glob(os.path.join(backup_dir, "*.jsonl")),
        key=os.path.getmtime,
        reverse=True,
    )
    for old in backups[MAX_BACKUPS:]:
        try:
            os.remove(old)
        except OSError:
            pass


if __name__ == "__main__":
    main()
    sys.exit(0)
