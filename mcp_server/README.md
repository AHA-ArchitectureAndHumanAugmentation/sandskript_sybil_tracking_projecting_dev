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
0–50 = corner zone radius, max_length_mm = Max Total Length ceiling on the DRAWN
length, 0 = off, omit to use the app's current setting — a longer path is
REFUSED, and generate_path reports length_mm/max_length_mm/over_length so the
refusal is visible one step earlier; the bundle also gets mask.png + skeleton.png of the
detection the path came from. preview.png appears only when an open Developer
window has pushed a shot of its 3D canvas for this same path — that canvas is
browser-only, so a save with no browser behind it simply has no preview.
path.json is COMPAS JSON: a flat `frames` list of `compas.geometry/Frame`
objects in millimetres plus an identity work object, which a compas_rrc script
reads with `compas.json_load`; the same waypoints also appear grouped under
`strokes` in metres, which is what validate_toolpath and the replay tool read,
with `stroke_starts` indexing where each stroke begins in the flat list),
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

Read `trigger_mm` together with `reference_set`, which decides what it measures.
With a reference it is a height ABOVE THE SAND (tens of mm); without one it is a
raw distance from the camera (hundreds). The rig's camera is mounted at an angle,
so the relative mode is the one in use — on a tilt the sand's own depth range is
wider than a hand's clearance above it, and no absolute cutoff separates the two.
`reference_set` governs the `ignore_closer_mm` detection parameter the same way.

Note on the camera: the app combines **every** RealSense on the rig into one
wide view using the layout saved in Multi-Cam Vision, and that view — not a
single camera's 640×480 frame — is what `capture_image` freezes and what
`generate_path`'s `crop` is relative to. Crops are normalized `{x,y,w,h}`, so
nothing about the tool call changes; `app_status` reports `camera_count` and
`frame_size` when the actual pixel size matters.

That canvas can also be **turned** in quarter turns from Developer Mode (the ⟳
button on the Depth viewport). The turn is applied where the canvas is built, so
everything these tools see is already turned: `frame_size` reports the turned
size — width and height swap on a quarter turn — and a `crop` is relative to the
turned frame. `app_status` reports the angle as `view_rotation` (0/90/180/270).
There is no tool to change it: turning the view drops the captured still, and
that is an at-the-rig framing decision, not a remote one.

The Participant-Mode **sound cues** are likewise not exposed here. They are
played by the projection window in the browser off the same `participant_status`
these tools can already read, so there is nothing for a tool to trigger — and
`app_status` remains the way to see which stage a participant is at.
