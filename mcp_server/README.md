# mcp_server

FastMCP server exposing the pipeline as tools (registered in `../.mcp.json`).
It is a thin client over the RUNNING app — start `run.bat` first; the app owns
the camera/robot, tools talk to it via HTTP/WS on port 5005 (`DEPTH_APP_URL` to
override). Tools: app_status, capture_image, generate_path (accepts
adjustments, crop, spacing_mm 10–100 for waypoint spacing, join_mm 0–200 =
Distance Threshold — merge strokes whose endpoints are closer than this, with
the threshold doubled when another stroke crosses the connecting line; 0 = off),
load_surface (CUMULATIVE — each call ADDS a surface to the scene, keeping the
position authored in its file; loading the same file name again replaces that
part. The loaded surfaces then act as ONE target: the drawing spans them all
and a single pose moves them together. The reply carries `count` = parts now
loaded. Removing one part / clearing them all is browser-only),
set_surface_pose, save_toolpath (speed_pct, offset_mm, safety_mm, blend_mm
0–5 = corner zone radius, max_length_mm = Max Total Length ceiling on the DRAWN
length, 0 = off, omit to use the app's current setting — a longer path is
REFUSED, and generate_path reports length_mm/max_length_mm/over_length so the
refusal is visible one step earlier; the bundle also gets mask.png + skeleton.png of the
detection the path came from. preview.png appears only when an open Developer
window has pushed a shot of its 3D canvas for this same path — that canvas is
browser-only, so a save with no browser behind it simply has no preview),
validate_toolpath. No run() tool by design —
executing robot motion stays a human action in the browser.

Note: while the Participant-Mode **Auto toggle is ON** (the ⧉ popup in the
browser), the app refuses manual `capture_image`/`generate_path` calls — the
automation owns the pipeline. `app_status` shows `participant_status`, which
includes **`Invalid`** — that drawing was refused (profanity guard, Max Total
Length, or the Max Drawing Time running out), so it was neither saved nor run.
`app_status` also reports `trigger_mm` and `max_draw_min` (the Max Drawing Time
limit in minutes, `null` = off). All three checks are Participant-Mode only and
have no MCP tool: `generate_path` via MCP is never gated by them.
