# CLAUDE.md

## Maintenance rule (apply on EVERY pipeline/feature change)
When the pipeline, WS/HTTP API, or features change: (1) update this file —
pipeline stages, WS message list, conventions, gotchas, test count; (2) check
`mcp_server/server.py` — its tools wrap the WS/HTTP API, so renamed/changed
messages, params or reply fields break them; update tools + `mcp_server/README.md`
to match; (3) update README.md user docs. Do this in the same commit as the change.

## What this is
depth-cam-to-robot: a browser-controlled pipeline that watches a sandbox with an
Intel RealSense **D435i** depth camera, detects hand-raked grooves (mm-deep — raw
metric depth, no RGB vision), converts them to strokes, projects them onto a
Rhino-authored 3D target surface, and has a **UR10e** (ur-rtde) retrace them with
the TCP perpendicular to the surface. Artistic context: gestures in sand guide a
robot depositing a living seeded substrate — the code's job ends at toolpath
execution/export. Includes a projector subsystem that shines the detected mask
back onto the sand, and a Save feature exporting URScript + JSON toolpaths.
Two modes: **Developer Mode** (`/`, all manual controls) and **Participant
Mode** (the ⧉ popup on the Depth viewport, `/depths`): an Auto toggle + depth
trigger run the whole pipeline automatically and lock the manual buttons —
including an OCR profanity guard that refuses offensive drawings before the
robot moves.

## Run / test
- Run: `run.bat` or the conda-env python (`ENVPY` below) `main.py` → Developer
  Mode at http://localhost:5005 (Participant Mode = its ⧉ popup). Closing the
  last browser tab kills the server (deliberate, via SIGINT).
- Python env = the **`sandskript` conda env** (recipe: `environment.yml`;
  recreate with `conda env create -f environment.yml`). On this machine
  ENVPY = `C:\Users\linfo\miniconda3\envs\sandskript\python.exe` — the .bat
  files and `.mcp.json` hardcode it; update those paths on a new machine.
  The recipe pulls python/pip from **conda-forge** (not `defaults`) so newer
  conda's Anaconda-ToS gate doesn't block env creation; keep the base/user
  `.condarc` on conda-forge too. If `conda` isn't found in PowerShell, run
  `conda init powershell` once (conda lives at `C:\Users\linfo\miniconda3`).
  Never bare `pip` (broken launcher risk — use `<ENVPY> -m pip`). The Intel
  RealSense USB driver is an OS-level install, outside the env. The old
  `.venv` is retired.
- Unit tests: `<ENVPY> -m pytest -q -m "not integration"` (305, no
  hardware). Integration: `-m integration`, needs RealSense/robot + TEST_ROBOT_IP.
- No CLI modes. Hardware vs no-robot is in the UI: "Test Mode (no robot)" button
  unlocks capture with a synthetic workspace; Run stays gated on a robot connection.
- Robot bring-up: UR10e in **Remote Control** mode, pendant speed slider 100%
  (or programmed speeds scale down), static link-local IP (e.g. 169.254.10.10),
  TCP+payload set on pendant. PC on same subnet.

## Pipeline (stage → owner → I/O)
1. **Capture** `camera_thread.DepthCameraThread` — RealSense depth+RGB 640×480@30,
   colour aligned to depth. Rolling buffer; `capture_frame()` → (depth_m float32
   HxW, valid bool, rgb BGR|None), temporally averaged (~30 frames ≈1 s, noise ↓√N).
   Live JPEGs into shared_state keys (`last_depth_color_jpg` etc.).
2. **Groove detection** `depth_extractor` — `grooves_and_mask(depth, valid, params,
   reference, mm_per_px)`: gap-fill → denoise → detrend (subtract blurred surface)
   → threshold (valley/ridge/band, mm relief) → morph close/min-blob →
   near-object rejection (`ignore_closer_mm` > 0: mask blobs touching anything
   ABSOLUTELY closer to the camera than that — a hand/body over the sand —
   dilated by GROOVE_NEAR_MARGIN_PX, are dropped; keeps the live projection off
   objects; UI = the "Ignore closer than (mm)" number box overlaid on the Mask
   viewport, always visible) → per-stroke filters
   (reference subtraction, min mean depth, min/max width, min length) →
   (thick mask, 1-px skeleton). `process_depth` adds crop; coords stay full-frame.
3. **Stroke extraction** `path_extractor.extract_from_edges` — 8-conn chain follow
   → Chaikin smooth → **endpoint join** (`join_strokes`, `join_mm` = exec-bar
   "Distance Threshold" box 0–200 mm, default JOIN_DISTANCE_MM=0 = off) →
   resample at `spacing_mm` (UI Spacing slider 10–100 mm,
   default RESAMPLE_SPACING_MM=10; falls back to 10 px w/o a mm scale) →
   nearest-neighbour TSP ordering → pixel strokes. Also returns `strokes_dense`
   (~2 mm) for the white on-surface skeleton line in the 3D preview.
   Joining merges two strokes when the gap between an endpoint of one and an
   endpoint of the other (start or end, direction irrelevant) is under
   `join_mm` — or under JOIN_CROSSING_FACTOR×`join_mm` (2×) when a THIRD stroke
   properly crosses the straight line closing that gap (an interruption implies
   one gesture). Each endpoint takes at most one partner, accepted
   shortest-gap-first so it lands on its nearest eligible neighbour; joins that
   would close a loop are refused, so the output is always open polylines.
   Order matters: joining runs on the smoothed chains BEFORE resample/TSP, so a
   merged stroke is resampled continuously across the seam and `strokes_dense`
   (the white line) shows the same merges as the waypoints — keep it there.
4. **Mapping** `surface.SurfaceModel.project_strokes` — STL/OBJ (Rhino, mm→m) via
   trimesh; camera frame fitted centred (aspect kept) onto the footprint ⟂ the
   mesh's dominant normal; ray-cast; TCP ⟂ surface with minimal twist; offset
   along outward normal. Draw side: authored mesh normals, EXCEPT steep
   surfaces (>~45° from horizontal) always draw on the side facing the robot
   base wherever the pose puts them (`draw_side_flip`) — so positive offset
   moves the TCP toward the robot and never behind a wall. Placement = `SurfacePose` (m + XYZ euler deg, base frame),
   set by UI sliders OR by corner→TCP touch-off (`registration.py`: pick a mesh
   corner — click its marker in the 3D preview or the dialog list, hover
   highlights it cyan (dialog is non-modal, preview stays visible) — then
   freedrive the tool tip onto it, confirm —
   1-point = translation only, keeps slider rotation; Kabsch ≥3-point solver
   already implemented for a future multi-point UI; corners = mesh vertices
   nearest the bbox corners, shipped in `mesh_payload()["corners"]`, same
   indices browser + server). No camera↔robot calibration exists. Planar fallback:
   `path_extractor.pixels_to_robot_coords` + `workspace.WorkspaceConfig` (Test Mode).
5. **Reach check** `reach.reach_flags` — envelope only (1.30 m sphere − 0.18 m axis
   cylinder). No IK/joint-limit/collision model. Red segments in preview.
6. **Execution** `path_executor.PathExecutor` — per stroke: retract along tool axis
   (Safety mm) → movel travel → movel onto the first waypoint → **movep** blended
   process path through the rest (blend = exec-bar Radius slider 0–5 mm, default
   MOVEP_BLEND_M=0.5 mm, clamped per stroke by `path_export.stroke_blend` to
   45% of the shortest segment; async movePath, polled so cancel stays
   responsive) — same actuation as the saved path.script; uniform
   speed = UI % of MAX_TCP_SPEED (1.0 m/s); run-time normal offset baked into
   waypoints. `robot_controller` = thread-safe ur-rtde wrapper.
7. **Export** `path_export.save_bundle` → `paths/<YYYY-MM-DD_HH-MM-SS>/` with
   `path.script` (URScript movel/movep), `path.json` (poses + per-waypoint plane:
   origin + orthonormal x/y/z axes, z = approach), `preview.png`.
8. **Server/UI** `server.py` (aiohttp) + `viewer/` — MJPEG: /depth /rgb
   /depth/grooves /depth/mask /depth/mask/full /depth/cropped (colorized depth
   restricted to the Developer-Mode crop; composed only while a /depths popup
   is connected); WS /ws (JSON); POST /surface/upload;
   GET /status (compact state JSON for tools); GET/POST /presets + GET
   /presets/{name} (Detection-Parameter slider presets; saved as
   `presets/<date_time>.json` but ANY .json in the folder loads — the GET
   guard (`_safe_preset_path`) allows custom-renamed files, rejecting only
   traversal via resolved-path containment; gitignored; browser-only, not
   exposed to MCP tools); /projection (+?cal);
   /depths (the Participant Mode popup: the CROPPED live depth view — the
   /depth/cropped stream, same region as the skeleton/mask views — with
   absolute mm-from-camera labels + Auto toggle + trigger box + big status
   chip; labels computed on the crop in camera_thread ONLY while a popup is
   connected, gated by `depth_overlay_clients`, throttled DEPTH_LABELS_EVERY;
   the popup never changes the crop — only users adjust it in Developer Mode). viewer.js =
   Developer-Mode single-page app w/ three.js preview; projection.html =
   corner-pin homography; depth_view.html + depth_overlay.js = the popup.
9. **Projector** — full-frame mask composed ONLY while a projection window is
   connected (`projection_clients`); corners persist in settings.json; Capture
   auto-blanks projector and waits for buffer refill before averaging.
10. **Participant Mode** `automation.ParticipantAutomation` (pure state machine)
    + `_participant_loop`/`_participant_pipeline` in main.py. Lives in the
    /depths popup: an **Auto toggle** (`set_automation{on}`) + a trigger
    distance (mm, `set_trigger`); camera thread flags frames with
    ≥TRIGGER_MIN_AREA_PX valid px closer than the trigger (`trigger_below`,
    `depth_extractor.depth_below_threshold`) — evaluated on the CROPPED
    region only, so motion outside the popup's visible area never triggers. Auto ON → **Auto On**; anything
    below trigger → **Alerted**; frame clear for PARTICIPANT_CLEAR_S →
    **Sensing** (waits buffer refill, then capture) → **Generating Paths**
    (current Dev-Mode crop/adjustments/spacing/join) → **profanity guard**
    (below) → **Actuating** (save_bundle, then run if robot connected; skipped
    otherwise) → back to **Auto On** (**Auto Off** when toggled off). While
    Auto is ON the manual
    capture/generate/run WS calls are refused server-side (`_manual_locked`,
    also blocks MCP tools) and the Dev-Mode buttons grey out; automation
    itself calls the SAME handlers via `server.broadcast_ws()` (a ws shim
    fanning out to all browser clients), so Developer windows watch it live.
    Statuses shown big top-right in the popup via `state.participant`.
11. **Profanity guard** `text_guard` — Participant Mode ONLY. Between Generating
    Paths and Actuating, OCRs the groove MASK (`shared_state["last_mask"]`,
    stashed by `on_generate_path`) and, on a wordlist hit, calls
    `automation.reject()` → status **Invalid**, red chip, nothing saved and
    nothing run. Invalid is STICKY (stays on screen, still armed — `_ARMED` in
    automation.py) so the participant reads the verdict; the next trigger, or
    toggling Auto off/on, clears it. OCR = Tesseract via `pytesseract`, both
    INSIDE the conda env; `_ensure_engine` points pytesseract at
    `sys.prefix/Library/bin/tesseract.exe`, prepends that dir to PATH (run.bat
    starts the env python WITHOUT activating, so tesseract55.dll's neighbours
    are otherwise unfindable) and sets TESSDATA_PREFIX. Mask is read at both
    polarities × PROFANITY_OCR_ROTATIONS (0°/180° — participants write from the
    far side), ~4 passes, once per capture. Matching (`find_profanity`, pure
    text, no OCR needed) = whole-token match, then substring match on the
    de-spaced text for entries ≥ PROFANITY_MIN_SUBSTRING_LEN (4) so "assist"
    survives "ass"; text normalized for case, umlauts/ß, accents and leetspeak.
    Wordlists = every `.txt` in `wordlists/` (seed en+de shipped; drop LDNOOBW
    files in to extend, no code change). Any failure — no engine, no wordlist,
    OCR error — returns `available=False, profane=False` so the pipeline still
    runs. Deliberately NOT wired into Developer Mode (the operator decides) and
    NOT exposed to MCP.

## Contained prototype: Dual-Cam Vision (NOT part of the two modes)
`run_stitch.bat` → `stitch_main.py` → http://localhost:5006. Merges TWO D435i
depth feeds (~5-10% frame overlap) into one top-down heightmap covering a
larger sand area. Two screens toggled by the Stitch button (`set_stitch{on}`):
OFF = setup (per-camera live depth+RGB left/right, ⇄ Swap and per-side ⟲
Rotate 180° buttons for upside-down mounts), ON = combined views. Starts OFF
unless `stitch_calibration.json` exists. Turning stitch ON auto-runs
`auto_align` (sweep candidate baselines tx, score overlap relief correlation,
`refine_shift` trim; fails cleanly on featureless sand — calib unchanged).
Modules: `stitcher.py` (pure math: deproject with per-device intrinsics →
cam2→cam1 rig transform `StitchCalib` (tx/ty/tz mm + yaw, swap, rot1/rot2 =
per-physical-camera 180° flags applied by `apply_orientation` before swap) →
rasterize onto a shared grid; overlap averaged; `refine_shift` = template-match
XY trim; `auto_align` = full baseline search), `dual_camera.py` (owns both
RealSense pipelines by serial; <2 cameras → SYNTHETIC scene; publishes
per-camera left/right JPEGs in both modes, stitched set only when ON),
`stitch_server.py` + `viewer/stitch.html`/`stitch.js` (MJPEG: /stitch/depth
w/ overlap outline, /stitch/rgb (middle gap expected — narrower colour FOV),
/stitch/mask, /stitch/skel + setup views /cam/{left,right}/{depth,rgb}; WS:
set_stitch, set_calib, set_params, auto_align, auto_refine, save_calib →
`stitch_calibration.json`, gitignored; state/init carry `stitch_on`).
Detection reuses `grooves_and_mask` unchanged on the stitched heightmap.
Deliberately NOT wired into Developer/Participant Mode or the MCP tools; no
main-app API change. Cannot run while the main app runs (one process per
RealSense). Never import `main` or `camera_thread` from these modules.

## Contained tool: toolpath replay (NOT part of the two modes)
`run_replay.bat` → `replay_main.py` → http://localhost:5007. Connect the robot,
pick a saved bundle under `paths/`, see its preview.png + meta, Run/Cancel with
Speed/Safety/Radius prefilled from the file. Modules: `toolpath_loader.py`
(pure parsing: `list_toolpaths`, `load_toolpath` → `Toolpath`; path.json read
verbatim, path.script parsed back via the exporter's "# stroke N (M pts)"
block layout — movep-run heuristic fallback for scripts without markers;
meta reconstructed from v=/r=/approach distance), `replay_robot.py`
(**`ReplayBackend` ABC = the robot-brand seam**: connect/disconnect/run/cancel
+ connected/running; `URReplayBackend` reuses RobotController + PathExecutor
with draw_z=0/offset=0 — saved poses execute literally; a future ABB GoFa port
= one new backend class + `make_backend` entry + `REPLAY_BACKEND` in config,
recipe in the module docstring), `replay_server.py` (WS: connect, disconnect,
refresh, select{name,source}, run{params}, cancel; GET /preview/{name}),
`viewer/replay.html`/`replay.js`. Deliberately NOT wired into the two modes or
MCP; no main-app API change. Reads settings.json `last_ip` (never writes it).
Don't run while the main app holds the robot (one RTDE controller per robot);
never import `main` from these modules.

## Conventions
- Pose = `[x, y, z, rx, ry, rz]`: metres + UR rotation vector (rad), robot base
  frame. Tool approach = tool-frame +Z; outward surface normal = −(R@[0,0,1]).
- Pixels 640×480, v grows down (flipped to world/robot Y-up). Crops normalized
  [0,1]; stroke coords always shifted back to full frame before mapping.
- Mesh files + UI depth params in mm; everything robot-side in m.
- Console output goes through `module_trace.log(action, msg, extra=())`, which
  prints the task line then `  └ a.py → b.py` naming the modules that served it;
  `module_trace.print_banner()` prints the feature→modules table at startup with
  ✓/· for actually-imported. Adding a pipeline stage means adding its chain to
  `STAGES` (a test asserts every STAGES module exists in `FEATURES`). Only
  process-lifecycle lines stay bare `print()`. Flags: SHOW_MODULE_BANNER /
  SHOW_MODULE_TRACE.
- `config.py` = every constant. `settings.json` = last robot IP + projector
  corners. `environment.yml` = the committed conda-env recipe (env = `sandskript`;
  pulls `tesseract` + `libcurl` from conda-forge for the profanity guard).
  `wordlists/*.txt` = profanity seed lists (committed, not gitignored).
  Gitignored: `surfaces/`, `paths/`, `presets/`, `settings.json`, `.venv/`
  (retired but still ignored as a safety net).
- Phases: idle → previewing → editing → captured → executing → done | error.

## Key WS messages (browser ↔ server; external tools may use these)
- in: `connect{ip}`, `disconnect`, `simulate_workspace`, `capture_image`,
  `preview_adjust{params}`,
  `generate_path{params:{crop,adjustments,spacing_mm,join_mm}}`,
  `run{params:{speed_pct,offset_mm,safety_mm,blend_mm}}`, `cancel`,
  `save_path{params:{speed_pct,offset_mm,safety_mm,blend_mm,image}}`,
  `set_groove_params{params}`, `set_reference`/`clear_reference`,
  `set_surface_pose{params:{pose,offset_mm}}`, `clear_surface`,
  `projection_hello`, `projection_corners{corners}`,
  `depth_overlay_hello`, `depth_overlay_params{params:{interval_mm}}`,
  `register_freedrive{params:{on}}`, `register_corner{params:{corner_index}}`,
  `set_trigger{params:{threshold_mm|null}}` (trigger distance; null/empty clears),
  `set_automation{params:{on}}` (Participant Auto toggle; ON locks manual
  capture/generate/run for every other client incl. MCP tools),
  `set_exec_params{params:{speed_pct,offset_mm,safety_mm,blend_mm,spacing_mm,
  join_mm}}` (live, debounced sync of the exec bar so Participant Mode +
  reopened windows match; blend_mm = movep corner Radius slider, 0–5;
  join_mm = Distance Threshold box, 0–200).
- out: `state` (20 Hz, incl. `participant{auto,status,message,trigger_mm,below}`;
  `init` carries the same block plus `detect{crop,adjustments,spacing_mm,join_mm}`
  + `exec{speed_pct,offset_mm,safety_mm,blend_mm}` — the browser restores its
  controls from these on (re)open), `capture_result{stroke_count,point_count,strokes,
  reach_flags,reach_out,skeleton,exec_viz:{blend_m,reach_m,min_reach_m,
  spacing_mm,join_mm}}`, `still`, `preview`, `surface_status`, `save_result`,
  `reference_status`, `execution_update`, `connection_result`,
  `register_result{success,message,pose,error}`,
  `depth_labels{labels:[[u,v,mm],...],size:[w,h]}` (only to /depths popups,
  ~4 Hz; coords + size are relative to the Developer-Mode crop, matching the
  /depth/cropped stream — the popup re-fits its stage from `size`).
  (`skeleton` = dense on-surface [x,y,z] polylines for the white preview line;
  `exec_viz` lets the browser rebuild the toolpath viz client-side on
  Offset/Safety edits.)

## Don't touch / gotchas
- **Never `import main` from tools/scripts** — import starts the camera thread and
  pollers (hardware side effects). Import the stage modules instead.
- One process per RealSense; one RTDE controller per robot. The running app owns
  both — external tools must go through HTTP/WS, not open hardware directly.
- Safety constants (`MAX_TCP_SPEED`, `UR_REACH_M`, speeds/accels, `DRAW_Z`) only
  change on explicit user request.
- Projection windows intentionally open on `127.0.0.1` (not localhost): Chrome
  caps 6 HTTP/1.1 connections per host and MJPEG streams hold theirs forever.
- `/`, `/projection`, `/depths`, `/static/*` are served no-cache — but Python
  changes still need an app restart.
- Participant Sensing waits DEPTH_AVERAGE_FRAMES/DEPTH_FPS before capturing:
  the averaged still uses the PAST second, which would contain the hand
  otherwise. Keep that wait ≥ the buffer length.
- `movep` orientation interp assumes neighbouring waypoints don't flip the
  wrist — surface projection chains tool-X for minimal twist; keep that property.
- Live drawing and the saved path.script both use movep with the exec-bar
  Radius blend — keep them in sync (that equivalence is the point of the movep
  executor). Both clamp via `path_export.stroke_blend` (45% of the stroke's
  shortest segment) because the UR rejects any path where a blend reaches half
  a segment; don't bypass that clamp. `MOVEP_BLEND_M` is only the default.
- The browser preview reads the Radius slider directly (`readBlendMm()` →
  `rebuildToolpathViz`); `exec_viz.blend_m` from capture_result is only the
  session echo.
- Exec-bar controls split two ways: Spacing and **Distance Threshold** change
  the path GEOMETRY, so they re-send `generate_path` (server-side rebuild);
  Offset/Safety/Radius only re-draw client-side via `rebuildToolpathViz`. Don't
  wire Distance Threshold into the client-side path — the browser has no copy
  of the pre-join chains.
- `_segments_cross` deliberately requires strictly opposite orientation signs,
  so a stroke that merely touches or ends ON the connecting line (a T junction)
  does NOT earn the doubled join threshold — only one that truly passes through.
- The profanity guard must FAIL OPEN, never closed: a missing Tesseract, a
  missing wordlist or an OCR exception all return `available=False,
  profane=False`. Blocking every drawing because an optional OCR install is
  absent would take the installation down; keep that property.
- `libcurl` is NOT optional in environment.yml — conda-forge's `tesseract`
  package does not pull it in on Windows and `tesseract55.dll` fails to load
  (exit 0xC0000135) without it. Symptom: guard silently reports "OCR engine
  unavailable" and every drawing passes.
- Never OCR the skeleton or the projected 3D strokes — 1-px hairlines read
  terribly. The guard is on the thick mask for a reason.
- Test count reference: 305 unit (+6 hardware-gated). The `text_guard` OCR
  tests skip themselves when Tesseract is absent; the text-matching ones always
  run. Keep green.
