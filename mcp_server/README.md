# mcp_server

FastMCP server exposing the pipeline as tools (registered in `../.mcp.json`).
It is a thin client over the RUNNING app — start `run.bat` first; the app owns
the camera/robot, tools talk to it via HTTP/WS on port 5005 (`DEPTH_APP_URL` to
override). Tools: app_status, capture_image, generate_path (accepts
adjustments, crop, spacing_mm 10–100 for waypoint spacing, join_mm 0–200 =
Distance Threshold — merge strokes whose endpoints are closer than this, with
the threshold doubled when another stroke crosses the connecting line; 0 = off),
load_surface,
set_surface_pose, save_toolpath (speed_pct, offset_mm, safety_mm, blend_mm
0–5 = movep corner radius), validate_toolpath. No run() tool by design —
executing robot motion stays a human action in the browser.

Note: while the Participant-Mode **Auto toggle is ON** (the ⧉ popup in the
browser), the app refuses manual `capture_image`/`generate_path` calls — the
automation owns the pipeline. `app_status` shows `participant_status`, which
includes **`Invalid`** — the profanity guard rejected that drawing, so it was
neither saved nor run. The guard is Participant-Mode only and has no MCP tool:
`generate_path` via MCP is never gated by it.
