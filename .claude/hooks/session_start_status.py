#!/usr/bin/env python
"""SessionStart hook: inject a short status of the pipeline backend.

The depth-cam-pipeline MCP server (mcp_server/server.py) is a thin client over
the app's HTTP/WS API at http://localhost:5005, so every MCP tool (app_status,
capture_image, generate_path, save_toolpath, ...) only works when that app is
running. This probes GET /status (short timeout) and prints a compact summary
to stdout, which Claude Code adds to the session context -- so the agent knows
up front whether the MCP tools will work, instead of finding out via a failed
tool call.

Deliberately does NOT re-run `git status` / `git log`: the harness already
injects those at session start. Always exits 0.
"""
import sys
import json
import urllib.request

URL = "http://localhost:5005/status"
TIMEOUT = 2.0


def main() -> None:
    try:
        with urllib.request.urlopen(URL, timeout=TIMEOUT) as resp:
            raw = resp.read().decode("utf-8", "replace")
    except Exception:
        print(
            "[pipeline] Backend NOT reachable at http://localhost:5005 -- start "
            "it (run.bat) before using depth-cam-pipeline MCP tools; they will "
            "fail until it is up."
        )
        return

    try:
        s = json.loads(raw)
    except Exception:
        print("[pipeline] Backend UP at http://localhost:5005 (/status returned "
              "non-JSON).")
        return

    surface = s.get("surface") or "none"
    print(
        "[pipeline] Backend UP at http://localhost:5005 -- depth-cam-pipeline MCP "
        "tools should work.\n"
        f"  phase={s.get('phase', '?')}  "
        f"robot_connected={s.get('robot_connected')}  "
        f"camera_streaming={s.get('camera_streaming')}  "
        f"executing={s.get('executing')}\n"
        f"  surface={surface}  strokes={s.get('stroke_count', 0)}  "
        f"reference_set={s.get('reference_set')}"
    )


if __name__ == "__main__":
    main()
    sys.exit(0)
