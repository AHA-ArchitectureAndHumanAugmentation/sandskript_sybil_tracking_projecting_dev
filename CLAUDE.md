# CLAUDE.md

## Maintenance rule (apply on EVERY pipeline/feature change)
When the pipeline, WS/HTTP API, or features change: (1) update this file —
pipeline stages, WS message list, conventions, gotchas, test count; (2) check
`mcp_server/server.py` — its tools wrap the WS/HTTP API, so renamed/changed
messages, params or reply fields break them; update tools + `mcp_server/README.md`
to match; (3) update README.md user docs. Do this in the same commit as the change.

## What this is
depth-cam-to-robot: a browser-controlled pipeline that watches a sandbox with
one or more Intel RealSense **D435i** depth cameras — combined into ONE canvas
by the layout saved in Multi-Cam Vision, which is what every view in both modes
shows — detects hand-raked grooves (mm-deep — raw
metric depth, no RGB vision), converts them to strokes, projects them onto a
Rhino-authored 3D target surface, and has an **ABB GoFa 10** (compas_rrc) retrace them with
the TCP perpendicular to the surface. Artistic context: gestures in sand guide a
robot depositing a living seeded substrate — the code's job ends at toolpath
execution/export. Includes a projector subsystem that shines the detected mask
back onto the sand — and plays the participant sound cues through the
projector's speaker — and a Save feature exporting JSON toolpaths.
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
- Unit tests: `<ENVPY> -m pytest -q -m "not integration"` (611, no
  hardware). Integration: `-m integration`, needs RealSense/robot + TEST_ROBOT_IP.
- No CLI modes. Hardware vs no-robot is in the UI: "Test Mode (no robot)" button
  unlocks capture with a synthetic workspace; Run stays gated on a robot connection.
- Robot bring-up: the GoFa link is NOT a direct socket. compas_rrc talks to a
  **ROS bridge** which talks to the **RRC RAPID task** on the controller, so
  three things must be up: (1) the RRC task loaded and running on the pendant,
  controller in Auto; (2) the bridge — `docker compose up` in **`docker/`**,
  which is OUR committed `docker-compose.yml` (see README), not one cloned from
  the compas_rrc repo: the driver image is pinned to `v1.1.2` so an upstream
  release cannot change what the installation runs, and rosbridge is launched
  with `unregister_timeout:=28800` (8 h) because the ~10 s default drops a quiet
  connection — the normal state of an installation waiting between visitors.
  `robot_ip` in that file is the ONE place the robot's own address is entered;
  `namespace:=rob1` must match `RRC_NAMESPACE` (a test asserts both, plus the
  port, against config.py); (3) the
  app, where the "IP" you enter is the machine running the BRIDGE (normally
  127.0.0.1), not the robot. Set TCP + payload on the pendant first.
  `RobotController.connect` sends a `Noop` as part of connecting: a live
  websocket does not prove the RAPID task is running, and finding that out
  mid-stroke is worse.

## Pipeline (stage → owner → I/O)
1. **Capture** `camera_thread.DepthCameraThread` — **every** RealSense on the rig
   (`realsense_source.open_cameras`, enumerated by serial), depth+RGB 640×480@30
   each, colour aligned to depth, laid onto ONE **combined canvas** by
   `stitcher.stitch` using the layout saved by Multi-Cam Vision
   (`stitcher.load_calib` → stitch_calibration.json, bound by serial,
   `with_default_row` for anything unplaced). That canvas — NOT one camera's
   frame — is what the whole pipeline sees: crop, reference, still and trigger
   are all in canvas pixels, and it is not 640×480.
   On top of that canvas sits the **view rotation** (`view_rotation.py`,
   Developer Mode's ⟳ button on the Depth viewport): a quarter turn of the
   FINISHED canvas, applied at the single seam where it leaves the camera
   thread (`_publish` + `capture_frame`), so every view, the projector mask,
   the depth labels, the Participant popup, the captured still and therefore
   the path all turn together and none of them knows about it. `np.rot90`, so
   the picture is re-indexed and never resampled. It is NOT the per-camera
   `rot_deg` in Multi-Cam Vision (that is a mounting angle baked in before
   stitching); this turns the whole picture, live, without touching the layout
   file. Persisted in settings.json (`view_rotation`) — it describes the rig.
   Per-camera rolling buffers at the full frame rate; `capture_frame()` averages
   EACH camera (~30 frames ≈1 s, noise ↓√N) and stitches the averages —
   averaging before the warp, not after. Live JPEGs into shared_state keys
   (`last_depth_color_jpg` etc.) rebuilt every STITCH_MAIN_EVERY_S (~5 Hz), not
   per frame — warping several frames onto a big canvas 30×/s buys nothing on
   static sand. The canvas geometry is FROZEN (`stitcher.CanvasGrid`) once every
   camera has delivered its first frame and never changes again; `frame_size` /
   `camera_count` are published to shared_state (and `/status`).
2. **Groove detection** `depth_extractor` — `grooves_and_mask(depth, valid, params,
   reference, mm_per_px)`: gap-fill → denoise → detrend (subtract blurred surface)
   → threshold (valley/ridge/band, mm relief) → morph close/min-blob →
   near-object rejection (`ignore_closer_mm` > 0: mask blobs touching a
   hand/body over the sand, dilated by GROOVE_NEAR_MARGIN_PX, are dropped;
   keeps the live projection off objects. WITH a reference the cutoff is a
   HEIGHT ABOVE THE SAND (`surface_height_mm`), without one an absolute camera
   distance — see stage 2b; UI = the number box overlaid on the Mask viewport,
   always visible, relabelled "Ignore above sand" / "Ignore closer than" by
   `showReferenceMode`) → per-stroke filters
   (reference subtraction, min mean depth, min/max width, min length) →
   (thick mask, 1-px skeleton). `process_depth` adds crop; coords stay full-frame.
2b. **Height above the sand** `depth_extractor.surface_height_mm(depth, valid,
   reference)` → `(height_mm, ok)` or None. `(reference − depth)·1000`: positive
   = nearer the camera than the baseline (a hand), negative = a groove, ~0 =
   untouched sand. This is the ONE place the camera's mounting angle is
   cancelled, and three things read it — the Participant trigger
   (`presence_trigger`), the popup's depth labels (`depth_region_labels`) and
   near-object rejection. A TILTED camera makes distance-from-camera useless:
   the sand alone spans more depth than a hand's clearance above it, so no
   absolute cutoff separates them. The reference frame CONTAINS the tilt, so
   subtracting it removes the gradient per pixel and one number means the same
   thing everywhere in the box. It must be per-pixel: rescaling the depth axis
   as a whole (however the range is compressed) scales the sand's spread and
   the hand's clearance by the same factor, leaves their ratio alone, and
   therefore changes no threshold test at all. None (no reference, or a shape
   that no longer matches after a view rotation) = every caller falls back to
   absolute — worse on a tilt, never broken.
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
   along outward normal. **Multi-surface**: Load Surface is CUMULATIVE —
   `surface.SurfaceScene` (a SurfaceModel subclass, so drop-in everywhere) holds
   the parts and hands downstream ONE concatenated mesh. Each file keeps the
   coordinates authored in it (nothing is re-centred), so surfaces exported from
   one Rhino document assemble themselves; the drawing is fitted across the
   UNION's footprint (one drawing over the assembly, not one per part), rays hit
   whichever part is nearest the draw side, and corners come from the union bbox
   so ONE `SurfacePose` — sliders or registration — moves everything rigidly.
   Re-loading the same file NAME replaces that part in place; `with_part`/
   `without_part` return NEW scenes (worker threads may hold the old one);
   removing the last part clears the scene.
   Draw side: authored mesh normals, EXCEPT steep
   surfaces (>~45° from horizontal) always draw on the side facing the robot
   base wherever the pose puts them (`draw_side_flip`) — so positive offset
   moves the TCP toward the robot and never behind a wall. Placement = `SurfacePose` (m + XYZ euler deg, base frame),
   set by UI sliders OR by corner→TCP touch-off (`registration.py`: pick a mesh
   corner — click its marker in the 3D preview or the dialog list, hover
   highlights it cyan (dialog is non-modal, preview stays visible) — then
   hand-guide the tool tip onto it (lead-through button ON THE ARM — RRC has
   no freedrive instruction, so the UI toggle only arms the touch-off), confirm —
   1-point = translation only, keeps slider rotation; Kabsch ≥3-point solver
   already implemented for a future multi-point UI; corners = mesh vertices
   nearest the bbox corners, shipped in `mesh_payload()["corners"]`, same
   indices browser + server). No camera↔robot calibration exists. Planar fallback:
   `path_extractor.pixels_to_robot_coords` + `workspace.WorkspaceConfig` (Test Mode;
   `simulation(frame_aspect)` takes the canvas aspect so a wide rig is not squashed
   into one camera's 4:3).
   The mm→px scale for all mm-based filters/spacings = `workspace.scene_mm_per_px`:
   surface first, workspace fallback — SAME precedence as stroke mapping, so a mm
   in the UI is a mm on whatever the strokes land on (Test Mode + surface included).
   It takes `frame_size` — the COMBINED canvas, via `main._frame_size()` (captured
   still first, else the live canvas) — because that is the frame stroke mapping
   fits onto the target. Passing a single camera's 640×480 would misreport every
   mm by the ratio of canvas width to 640.
5. **Reach check** `reach.reach_flags` — envelope only (1.62 m sphere − 0.18 m axis
   cylinder). No IK/joint-limit/collision model. Red segments in preview.
5b. **Length limit** `path_length.blended_length` — how far the tool travels
   WHILE DRAWING (the green/red line; travels and retracts excluded, since the
   limit bounds material laid down, not motion). Measured on the PROJECTED
   robot-space strokes, so surface shape, real scale, Spacing and the join
   Distance Threshold are already in the number — a join's closed gap is a real
   segment because `join_strokes` concatenates point lists. The corner zone is
   then subtracted per interior vertex: the controller replaces 2r of straight
   line with an arc, so `saving = 2r − r·tan(θ/2)·(π−θ)` (θ = interior angle),
   which correctly gives 0 for a straight-through waypoint and the whole 2r for
   a doubling-back. Radius is clamped by the SAME `path_export.stroke_blend` the
   executor uses, so the number describes the robot and not an ideal path.
   Ceiling = the exec-bar **Max Total Length** box (mm, 0 = off,
   MAX_PATH_LENGTH_MM). Over it: `on_run` and `on_save_path` both refuse
   (`main._length_check`), and Participant Mode calls the drawing **Invalid**
   via `automation.reject` — the same treatment as a profanity hit, because
   both mean "this drawing is not acceptable", not "something broke".
6. **Execution** `path_executor.PathExecutor` — per stroke: retract along tool axis
   (Safety mm) → linear travel → land exactly on the first waypoint → one
   **zone-blended run** of linear moves through the rest (zone = exec-bar Radius
   slider 0–BLEND_ZONE_MAX_MM = 50 mm, default BLEND_ZONE_M=0.5 mm, clamped per
   stroke by `path_export.stroke_blend` to 45% of the shortest segment — so a
   radius past what the Spacing allows is not an error, it just rounds as much
   as the segment has room for). The whole stroke
   is queued at once and ONLY the last move asks for feedback — waiting per
   waypoint would stop the arm on each point and defeat the blending the zone
   data exists to produce; the wait happens outside the lock so cancel stays
   responsive. Uniform speed = UI % of MAX_TCP_SPEED (1.0 m/s; the GoFa 10 is
   the GoFa 10's rated tool speed — note every Speed percentage is relative
   to it, so a pre-port bundle saved at "50%" now replays twice as fast);
   run-time normal offset baked into waypoints.
   **Material dispenser**: `robot.set_dispenser(True/False)` wraps the blended
   run in `_draw_stroke` — opened once the TCP is ON the first waypoint, closed
   in a `finally` before anything retracts, so nothing is deposited during a
   travel move and a cancel/error/disconnect closes it. Config-only
   (`DISPENSER_ENABLED` False until the valve is wired + named in the
   controller's I/O); disabled means not one instruction is sent. No UI, no MCP.
   `robot_controller` = thread-safe compas_rrc wrapper, and the ONE place the
   unit seam lives: everything above it is metres + axis-angle rotation vector,
   RAPID wants mm + a frame (`pose_to_frame`/`frame_to_pose`). It is also where
   **acceleration** is set (`rrc.SetAcceleration`, in `connect`): RAPID AccSet
   is a controller-wide PERCENTAGE, not a per-move m/s², so `DRAW_ACCEL` /
   `TRAVEL_ACCEL` never reach the arm and `RRC_ACCEL_PCT`/`RRC_ACCEL_RAMP_PCT`
   (default 100 = the controller's own default) do.
7. **Export** `path_export.save_bundle` → `paths/<YYYY-MM-DD_HH-MM-SS>/` with
   `path.script` (URScript movel/movep — a readable RECORD of the same
   poses, NOT what the GoFa runs and never parsed back), `path.json` (see
   below), `preview.png`, plus the
   detection stage that produced the path: `mask.png` (thick region) and
   `skeleton.png` (1-px centrelines), both the CROPPED uint8 arrays stashed by
   `on_generate_path` as `last_mask`/`last_skeleton` — replaced in lockstep with
   `strokes`, so a save can never pair a path with another generate's images.
   Written via `_write_png` (imencode + write_bytes, not `cv2.imwrite`). Any of
   the three PNGs may be absent; only .script/.json are guaranteed.
   `preview.png` is the ONE image the server cannot make — three.js draws it on
   the client's GPU. Developer Mode: the Save click carries it up. Participant
   Mode: nobody clicks, so any open Developer window volunteers a shot on its
   own (`preview_image` WS message → `shared_state["last_preview_png"]`) and
   `on_save_path` falls back to it. Paired to its generate by
   `shared_state["path_serial"]` — bumped on every Generate Path, echoed in
   `capture_result`, sent back with the image, and a mismatch is dropped: same
   lockstep rule as mask/skeleton. Gate = `path_export.is_png_data_url`
   (prefix + real base64 + PNG signature + PREVIEW_MAX_BYTES), since this is a
   client-supplied blob. No Developer window open → no preview.png, never a
   failed save.
7b. **path.json** (`path_export.build_json`) carries the SAME waypoints twice,
   because two different readers want them:
   `frames` = a FLAT list of serialized COMPAS `Frame`s (`dtype`
   `compas.geometry/Frame`, point in **mm**, xaxis+yaxis — compas derives
   z = x×y = the approach) plus an identity work object as three
   `compas.geometry/Point`s (`wobj_origin`/`wobj_xaxis`/`wobj_yaxis`, axis
   points WOBJ_AXIS_MM out; identity because the poses are already base-frame,
   the same `wobj0` a live run uses). That is exactly what `compas.json_load`
   decodes and what a compas_rrc script consumes, so a saved bundle can be run
   from a plain RRC script with no conversion. Frames are built the SAME way as
   `robot_controller.pose_to_frame`, so the file and the live run agree.
   `strokes` = the same waypoints GROUPED per stroke, poses in metres + the
   per-waypoint plane (origin + orthonormal x/y/z) — this is what
   `toolpath_loader` reads and the replay tool executes. `stroke_starts` indexes
   `frames` at each stroke's first waypoint, since the flat list alone cannot
   say where one gesture ends and the next begins. Plus `meta` (run params) and
   `units`.
8. **Server/UI** `server.py` (aiohttp) + `viewer/` — /sounds/*.wav (cue audio,
   see stage 10c); MJPEG: /depth /rgb
   /depth/grooves /depth/mask /depth/mask/full /depth/cropped (colorized depth
   restricted to the Developer-Mode crop; composed only while a /depths popup
   is connected); WS /ws (JSON); POST /surface/upload (ADDS a part to the scene
   — see stage 4; the browser file input is `multiple` and posts them one at a
   time, since each upload is a read-modify-write of the scene, serialized
   server-side by `main._surface_lock`);
   GET /status (compact state JSON for tools; `surface` = scene name,
   `surface_count` = parts loaded, `camera_count` + `frame_size` = the combined
   canvas the crop is relative to, `view_rotation` = the quarter turn already
   applied to it); GET/POST /presets + GET
   /presets/{name} (Detection-Parameter presets — the sliders as flat keys
   PLUS the whole Path Preview bar under an **`exec`** key
   {spacing_mm,join_mm,blend_mm,speed_pct,offset_mm,safety_mm,max_length_mm},
   so a preset restores what the settings DRAW and not just how they detect;
   saved as `presets/<date_time>.json` but ANY .json in the folder loads — the
   GET guard (`_safe_preset_path`) allows custom-renamed files, rejecting only
   traversal via resolved-path containment; the server is schema-free here, it
   persists what the browser posted verbatim; gitignored; browser-only, not
   exposed to MCP tools); /projection (+?cal);
   /depths (the Participant Mode popup: the CROPPED live depth view — the
   /depth/cropped stream, same region as the skeleton/mask views — with
   absolute mm-from-camera labels + Auto toggle + trigger box + Max Drawing
   Time box + big status chip (top-right) + drawing-time countdown (top-left); labels computed on the crop in camera_thread ONLY while a popup is
   connected, gated by `depth_overlay_clients`, throttled DEPTH_LABELS_EVERY;
   the popup never changes the crop — only users adjust it in Developer Mode). viewer.js =
   Developer-Mode single-page app w/ three.js preview; projection.html =
   corner-pin homography; depth_view.html + depth_overlay.js = the popup.
9. **Projector** — full-frame mask composed ONLY while a projection window is
   connected (`projection_clients`); corners persist in settings.json; Capture
   auto-blanks projector and waits for buffer refill before averaging. The
   projection page is also the audio output — see stage 10c.
10. **Participant Mode** `automation.ParticipantAutomation` (pure state machine)
    + `_participant_loop`/`_participant_pipeline` in main.py. Lives in the
    /depths popup: an **Auto toggle** (`set_automation{on}`) + a trigger
    threshold (mm, `set_trigger`, clamped TRIGGER_MIN_MM…TRIGGER_MAX_MM); camera
    thread flags frames with ≥TRIGGER_MIN_AREA_PX valid px past the trigger
    (`trigger_below`, `depth_extractor.presence_trigger`) — evaluated on the
    CROPPED region only, so motion outside the popup's visible area never
    triggers. The threshold is a HEIGHT ABOVE THE SAND when a reference is set
    and an absolute camera distance when not (stage 2b); the popup relabels the
    box and the overlay numbers from `state.reference_set` /
    `depth_labels.relative`, because the two modes want values an order of
    magnitude apart. Auto ON → **Auto On**; anything
    below trigger → **Alerted**; frame clear for PARTICIPANT_CLEAR_S →
    **Sensing** (waits buffer refill, then capture) → **Generating Paths**
    (current Dev-Mode crop/adjustments/spacing/join) → **profanity guard**
    (below) → **Actuating** (save_bundle — including preview.png when a
    Developer window pushed one, see stage 7 — then run if robot connected;
    skipped otherwise) → back to **Auto On** (**Auto Off** when toggled off). While
    Auto is ON the manual
    capture/generate/run WS calls are refused server-side (`_manual_locked`,
    also blocks MCP tools) and the Dev-Mode buttons grey out; automation
    itself calls the SAME handlers via `server.broadcast_ws()` (a ws shim
    fanning out to all browser clients), so Developer windows watch it live.
    Statuses shown big top-right in the popup via `state.participant`.
10b. **Max Drawing Time** — the popup's second threshold, beside the trigger:
    minutes (`set_max_draw_time{minutes|null}`, empty = off, clamped
    PARTICIPANT_MAX_DRAW_MIN_MIN…MAX_MIN). Measured **Alerted → frame clear**
    (hand in → hand out), NOT including the pipeline, and the clock FREEZES at
    the first clear frame, so a drawing finished just in time is not failed by
    the PARTICIPANT_CLEAR_S debounce. Running out is judged inside
    `ParticipantAutomation.tick` (it must stop a drawing that never ends, so it
    cannot wait for the pipeline) → `reject()` → **Invalid**, nothing saved,
    nothing run, plus an internal `_await_clear` latch: the still-present hand
    must NOT re-Alert the machine or the verdict would vanish before it is read.
    Countdown = the SERVER's clock (`automation.remaining_s()`, republished by
    `_participant_loop` into `shared_state["participant_remaining_s"]`, shipped
    in `state.participant`), drawn top-LEFT in the popup — dim when idle
    (showing the full allowance), blue while counting, red+blinking for the last
    PARTICIPANT_WARN_S seconds (`warn_s` travels with the state, so the CSS and
    the config can't drift apart). The browser never runs its own timer: one
    clock shows and judges.
10c. **Sound cues** `sound_design` + `viewer/projection.html` — four synthesized
    .wav cues marking the participant-facing moments, played through the
    PROJECTOR's speaker. `sound_design.py` composes them from numpy (no
    recordings, no audio dependency) and is run by hand to render
    `sounds/*.wav`, which are COMMITTED — the app never imports the module.
    Mapping = `config.SOUND_CUES` (status → file stem): Alerted→`engaged`
    (rising A major triad = invitation), Sensing→`acknowledged` (the same notes
    falling to the root, held + pulsed = received, still working),
    Actuating→`anticipation` (accelerating pentatonic rise over a swell),
    Invalid→`alarm` (tritone sawtooth klaxon, loudest by design). Generating
    Paths is deliberately silent. Serving = `add_static(SOUNDS_URL_PATH)`, the
    ONE static route that is not no-cache. The map is shipped in `init` as
    `sounds{path,cues}` so the page holds no copy of it. Playback lives in
    projection.html: Web Audio, all cues decoded at init, fired on status ENTRY
    (`lastStatus` seeded from `init.participant` so opening the window mid-cycle
    is silent), OUTPUT window only (`!IS_CAL`), `M` mutes, and a badge asks for
    the one click browsers require before audio may start. No projection window
    → no sound, by design. NOT wired into Developer Mode or the /depths popup,
    and not exposed to MCP.
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

## Multi-Cam Vision: the placement tool (its own app; its OUTPUT feeds both modes)
`run_stitch.bat` → `stitch_main.py` → http://localhost:5006. Lays HOWEVER MANY
D435i depth feeds are plugged in (1 … STITCH_MAX_CAMERAS = 4, enumerated by
serial) onto ONE top-down canvas covering a larger sand area. The TOOL is
standalone (separate process, separate port, no main-app import), but the
LAYOUT it saves is now the main app's camera: `camera_thread` reads the same
`stitch_calibration.json` and builds the same canvas for the pipeline. This is
where the picture the robot draws from gets shaped.
It ONLY combines images: no overlap search (the cameras are bolted down), no
groove detection and no detection parameters (those live in the main app).
ONE screen, always
live, split by a drag bar into RESULT on top (the combined canvas, look-only,
`pointer-events:none`; only the selected camera's footprint is outlined so you
can tell the pictures apart) and WORKBENCH underneath (one panel per camera —
every edit happens here). Splitter height persists in localStorage.
Sidebar per camera: **Turn** (⟲ ⟳ quarter turns), **Move** (◀ ▶ = swap places
with the neighbour to that side), Height (▼ ▲), Reset corners / Reset camera,
depth-vs-colour, Save layout / **Discard changes**. Turn + Move are how the row
is made to match the physical rig BEFORE any fine dragging.
**Saved vs adjusted** (`MultiCameraThread.dirty` / `mark_saved` / `revert_calib`,
shipped as `dirty` in init+state): the main app builds its view from the FILE,
so an unsaved adjustment is a picture only this window has — the Save button
goes amber and the line under it reads "Adjusted — not saved. The app still
uses the layout in stitch_calibration.json." Nothing but an explicit Save writes,
so reopening always comes back to the last save; Discard changes (arms on the
first click, fires on the second) reloads the file without a restart. The
baseline for "dirty" is the layout AS LOADED — i.e. after `bind_placements`
matched it to the cameras present — so merely opening the tool never reads as
an edit, while a rig that no longer matches the file (a camera unplugged) does.
Per panel: green numbered handles 1-4 on the corners shape where that camera
lands, dragging inside the green outline moves it, blue EDGE bars trim the
picture (edges not corners, so the two never fight for the same hit area).
The green outline is the camera's canvas quad drawn at the panel's own scale
through a **cached** mm→px map (`panelMaps`/`fitMap`/`panelPts`): at each fit
an unskewed quad lands exactly on the crop rect, so the cropped region normally
sits inside the four handles. The map deliberately does NOT depend on the quad
— deriving scale + centre from the corners every repaint is what made dragging
ONE handle rescale and re-centre the other three. It re-fits only on a turn, a
panel resize, a `command()` button, when the shape drifts off-panel, or when
`applyCalib` sees the SERVER move corners we did not (a trim re-cut); a plain
corner drag echoes back identical, so nothing shifts under the operator's hand.
Handles live on `<body>`, positioned from the panel's client rect, so a short
workbench never clips them.
Per camera (`stitcher.CameraPlacement`): `rot_deg` 0/90/180/270 mounting
rotation, `crop` normalized x/y/w/h, **`quad_mm` = the four canvas corners the
cropped frame is pinned to**, `height_mm`, `enabled`. `quad_mm` IS the
placement — move/rotate/skew are all just different ways of moving corners, so
there are no separate offset/angle numbers and the UI is four drag handles.
Corner order is **TL, TR, BL, BR** = handles 1-4, the SAME convention (and
look) as `viewer/projection.html`'s projector calibration.
Pipeline per camera: rotate image+intrinsics → crop (clears `valid`, never
slices, so the pixel coords the pin is built on stay exact) →
`cv2.getPerspectiveTransform(crop_corners_px, quad)` → `cv2.warpPerspective`
straight onto the shared canvas; overlaps averaged, `coverage` counts
contributors, `fill_small_holes` closes speckle. Image-space, not deprojection:
that is what makes a corner drag land exactly where the operator put it, and
for a near-flat sand plane the two agree to well under a pixel.
Helpers worth knowing: `rotate_quad` (turn a quad about its centre AND
re-label its corners, so the picture turns unstretched — pair it with
`rot_deg`, `MultiCameraThread.rotate_camera` does both plus `_rotate_crop`),
`requad_for_crop` (push a new crop through the OLD pin so trimming an edge
never slides the sand that was kept), **`default_row_mm`** (the opening layout:
ONE tile size for the whole rig — `common_footprint_mm`, the MEDIAN of what the
cameras report — laid FLUSH left to right, tops aligned, each tile scaled by its
own crop so trimming closes the row up instead of leaving a hole;
`default_quad_mm` is only the single-camera fallback for a folded quad),
**`swap_quads_x`** (trade two cameras' places in x while each keeps its own
shape — the keystone must survive a reorder; `quad_centre_x` is what "left"
means), `bind_placements` (match a saved calib to the cameras present, serial
first then position).
Modules: `stitcher.py` (pure math + `synthetic_scene(n)`), `multi_camera.py`
(`MultiCameraThread` owns every RealSense pipeline; 0 cameras → SYNTHETIC
scene of STITCH_SYNTHETIC_CAMERAS), `stitch_server.py` +
`viewer/stitch.html`/`stitch.js` (MJPEG: `/canvas` = the result view with
overlap outlined, `/cam/{index}` = one workbench panel per camera carrying the
crop rectangle; WS in: set_camera{index,…}, rotate_camera{index,steps},
move_camera{index,steps} (±1 = one place left/right in the row),
nudge_height{index,steps}, reset_camera{index,corners_only},
set_grid{mm_per_px}, set_colour{on}, save_calib → `stitch_calibration.json`
(gitignored) + `mark_saved`, revert_calib (reload the file, drop unsaved edits);
out: init/state carry `calib{cams[],mm_per_px}`, `dirty` (init also
`calib_file`) and
`info.cameras[].quad_px` = each placed quad in canvas pixels, which the browser
uses ONLY to outline the selected camera in the result view — the workbench
handles are driven by `calib.cams[].quad_mm`, not by `quad_px`).
The tool's UI is NOT wired into Developer/Participant Mode or the MCP tools —
the coupling is one file, `stitch_calibration.json`, read at camera-thread
start-up. Cannot run while the main app runs (one process per RealSense).
Never import `main` or `camera_thread` from these modules; the shared code goes
the other way (both import `stitcher` for the placement math and
`realsense_source` for the devices).

## Contained tool: toolpath replay (NOT part of the two modes)
`run_replay.bat` → `replay_main.py` → http://localhost:5007. Connect the robot,
pick a saved bundle under `paths/`, see its preview.png + meta, Run/Cancel with
Speed/Safety/Radius prefilled from the file. Modules: `toolpath_loader.py`
(pure parsing: `list_toolpaths`, `load_toolpath` → `Toolpath`; path.json read
verbatim; the sibling path.script is ignored, so a folder with no
path.json is not a bundle;
meta reconstructed from v=/r=/approach distance), `replay_robot.py`
(**`ReplayBackend` ABC = the robot-brand seam**: connect/disconnect/run/cancel
+ connected/running; `URReplayBackend` reuses RobotController + PathExecutor
with draw_z=0/offset=0 — saved poses execute literally; a future ABB GoFa port
= one new backend class + `make_backend` entry + `REPLAY_BACKEND` in config,
recipe in the module docstring), `replay_server.py` (WS: connect, disconnect,
refresh, select{name,source}, run{params}, cancel; GET /preview/{name}),
`viewer/replay.html`/`replay.js`. Deliberately NOT wired into the two modes or
MCP; no main-app API change. Reads settings.json `last_ip` (never writes it).
Don't run while the main app holds the robot (one RRC client per controller task);
never import `main` from these modules.

## Contained tool: Scheduler (NOT part of the two modes)
`run_scheduler.bat` → `scheduler_main.py` → http://localhost:5008. A READ-ONLY
ledger of `paths/`: one numbered row per saved bundle, four columns —
**# / Path executed / Date and time / Mask**. Oldest first, so row 1 is the
earliest path saved. Modules: `scheduler.py` (pure filesystem logic:
`ScheduleRow`, `read_schedule`, `parse_folder_time`, `to_csv`),
`scheduler_server.py` (aiohttp; GET `/`, GET `/schedule.csv`, GET
`/mask/{name}`, WS `/ws` — in: `refresh`; out: `init`/`schedule` carrying
`rows[]`+`base`+`count`), `viewer/scheduler.html`/`scheduler.js`.
The Mask column serves each bundle's `mask.png` through `/mask/{name}`, guarded
by `_safe_folder` (same rule as `replay_server`: reject `/`, `\`, `..`) and
`loading="lazy"` in the browser, since a long ledger is a lot of 640×480 PNGs.
Bundles saved before `mask.png` existed show a dash — `has_mask` on the row,
never a broken image. The CSV cannot hold a picture, so its Mask column carries
the file's path instead.
What counts as a bundle is NOT redefined here: it defers to
`toolpath_loader.list_toolpaths` so the Scheduler and the replay tool can never
disagree about what is on disk. The row also carries `files` (the bundle's
actual contents) which the UI prints under the folder name, so a bundle missing
`preview.png` (Participant Mode) or `mask.png`/`skeleton.png` (saved before
those existed) is visible at a glance.
**The timestamp is when the bundle was SAVED, not a separate execution record**
— nothing in the pipeline writes one. Sources are tried in order of trust:
folder name (`YYYY-MM-DD_HH-MM-SS`, `_2` suffix stripped) → `path.json`'s
`meta.saved` → folder mtime; each row reports its `time_source`, and the UI
marks anything that is not the folder name. In Participant Mode save happens
immediately before the run, so save time IS run time to within a second.
The watch loop re-scans every SCHEDULER_REFRESH_S and pushes ONLY when the
scan's signature changes, so a path saved in the main app appears without a
reload. Being read-only and hardware-free, this is the ONE tool that is safe to
run beside the main app. Deliberately NOT wired into Developer/Participant Mode
and NOT exposed to MCP; no main-app API change. Never import `main` from these
modules (a test asserts a fresh interpreter importing `scheduler_server` pulls
in no `main`/`camera_thread`/`robot_controller`).

## Conventions
- Pose = `[x, y, z, rx, ry, rz]`: metres + UR rotation vector (rad), robot base
  frame. Tool approach = tool-frame +Z; outward surface normal = −(R@[0,0,1]).
- Pixels = the COMBINED canvas (640×480 only for a one-camera rig at 1:1 — never
  assume it), v grows down (flipped to world/robot Y-up). Crops normalized [0,1],
  so they survive any canvas size; stroke coords always shifted back to full
  frame before mapping. Anything needing the real size takes it from
  `shared_state["frame_size"]` / `still_dims` / `camera_thread.frame_size`, never
  from DEPTH_WIDTH/DEPTH_HEIGHT — those two now describe ONE camera's stream.
- Mesh files + UI depth params in mm; everything robot-side in m.
- Console output goes through `module_trace.log(action, msg, extra=())`, which
  prints the task line then `  └ a.py → b.py` naming the modules that served it;
  `module_trace.print_banner()` prints the feature→modules table at startup with
  ✓/· for actually-imported. Adding a pipeline stage means adding its chain to
  `STAGES` (a test asserts every STAGES module exists in `FEATURES`). Only
  process-lifecycle lines stay bare `print()`. Flags: SHOW_MODULE_BANNER /
  SHOW_MODULE_TRACE.
- `config.py` = every constant. `settings.json` = last robot IP + projector
  corners + `view_rotation` (the canvas quarter turn; read once at start-up,
  before the camera thread starts, so the first frame is already turned). `docker/docker-compose.yml` = the committed ROS bridge (pinned
  driver image, 8 h rosbridge timeout, `robot_ip`).
  `environment.yml` = the committed conda-env recipe (env = `sandskript`;
  pulls `tesseract` + `libcurl` from conda-forge for the profanity guard).
  `wordlists/*.txt` = profanity seed lists (committed, not gitignored).
  `sounds/*.wav` = the rendered participant cues (committed too — a fresh
  clone must make noise without anyone running the generator).
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
  `rotate_view{params:{steps}}` (⟳ on the Depth viewport; steps = quarter turns
  clockwise, default 1 — turns the whole canvas, re-bases the crop + reference,
  drops the captured still, lock-gated like capture/generate/run),
  `set_surface_pose{params:{pose,offset_mm}}`, `clear_surface` (ALL parts),
  `remove_surface{params:{index}}` (one part; index = `info.parts[].index`,
  out-of-range/missing is a no-op; removing the last part clears the scene),
  `projection_hello`, `projection_corners{corners}`,
  `depth_overlay_hello`, `depth_overlay_params{params:{interval_mm}}`,
  `register_freedrive{params:{on}}` (arms the touch-off; on the GoFa it does
  NOT move the robot into a compliant mode — see stage 4),
  `register_corner{params:{corner_index}}`,
  `set_trigger{params:{threshold_mm|null}}` (trigger distance; null/empty clears),
  `set_max_draw_time{params:{minutes|null}}` (Max Drawing Time; null/empty = no
  limit — see stage 10b),
  `set_automation{params:{on}}` (Participant Auto toggle; ON locks manual
  capture/generate/run for every other client incl. MCP tools),
  `preview_image{params:{image,serial}}` (a Developer window handing over a PNG
  data URL of its 3D canvas so an automated save still gets preview.png; sent
  unprompted after each `capture_result` while Auto is ON, never lock-gated —
  it is not a pipeline action),
  `set_exec_params{params:{speed_pct,offset_mm,safety_mm,blend_mm,spacing_mm,
  join_mm,max_length_mm}}` (live, debounced sync of the exec bar so Participant
  Mode + reopened windows match; blend_mm = corner zone Radius slider, 0–50;
  join_mm = Distance Threshold box, 0–200; max_length_mm = Max Total Length
  box, 0–100000, 0 = off). `run` and `save_path` also carry `max_length_mm`, so
  the click is judged against what that window currently shows.
- out: `view_rotation{deg,crop}` (broadcast on the button press only — `crop` is
  the server's crop turned onto the new canvas; it travels WITH the angle
  because a client that took one without the other would frame the wrong sand.
  Deliberately not folded into `state`: the crop is dragged by hand and
  republishing it 20×/s would fight the mouse. `deg` alone IS in `state` and
  `init`, so a window that missed the broadcast self-heals),
  `state` (20 Hz, incl. `view_rotation`, `reference_set` — which tells both UIs
  whether the trigger and the near-object cutoff are heights above the sand or
  absolute camera distances — and `path_length_mm` + `max_length_mm` — the drawn
  length of the current path and its ceiling — and
  `participant{auto,status,message,trigger_mm,below,max_draw_min,remaining_s,
  warn_s}`;
  `init` carries the same block plus `detect{crop,adjustments,spacing_mm,join_mm}`
  + `exec{speed_pct,offset_mm,safety_mm,blend_mm}` + `view_rotation`
  + `reference_set` — the browser restores its controls from these on (re)open — and `sounds{path,cues}` = where the cue
  .wav files are served and which participant status plays which, straight from
  `config.SOUND_CUES`), `capture_result{stroke_count,point_count,strokes,
  reach_flags,reach_out,skeleton,path_serial,length_mm,max_length_mm,
  exec_viz:{blend_m,reach_m,min_reach_m,spacing_mm,join_mm}}`, `still`, `preview`,
  `surface_status{loaded,info,pose,offset_mm,mesh,message}` (`info.count` +
  `info.parts[{index,name,faces,bbox}]` = the loaded parts; `mesh` = the
  COMBINED geometry + union corners), `save_result`,
  `reference_status`, `execution_update`, `connection_result`,
  `register_result{success,message,pose,error}`,
  `depth_labels{labels:[[u,v,mm],...],size:[w,h],relative}` (only to /depths
  popups, ~4 Hz; coords + size are relative to the Developer-Mode crop, matching
  the /depth/cropped stream — the popup re-fits its stage from `size`.
  `relative` = those mm are height above the sand, not distance from the
  camera — see stage 2b).
  (`skeleton` = dense on-surface [x,y,z] polylines for the white preview line;
  `exec_viz` lets the browser rebuild the toolpath viz client-side on
  Offset/Safety edits.)

## Don't touch / gotchas
- **Never `import main` from tools/scripts** — import starts the camera thread and
  pollers (hardware side effects). Import the stage modules instead.
- One process per RealSense; one RRC client per controller task. The running app owns
  both — external tools must go through HTTP/WS, not open hardware directly. The
  main app now owns EVERY camera, so Multi-Cam Vision and the app are strictly
  either/or (it always was, but with one camera you could get away with it).
- Only an explicit Save writes `stitch_calibration.json` — never a drag, a
  rotate, a reset or a shutdown. That is what makes the combined view
  reproducible: reopening the tool, and the main app reading the same file,
  both rebuild the last SAVED layout exactly (same quads → same canvas origin,
  size and mm/px; `_auto_mm_per_px` derives from saved quads, so it is
  deterministic too). Don't add an autosave — an adjustment the operator was
  still trying out would silently become what the robot draws from.
- The canvas geometry is frozen ONCE, at camera-thread start-up, and the layout
  file is read there and nowhere else. Editing the layout therefore needs an app
  restart — which is not a limitation to "fix", since the placement tool cannot
  run while the app holds the cameras anyway. Never re-bind or re-freeze mid
  session: the crop, the reference frame and the captured still are all in
  canvas pixels, so a canvas that resized would silently move all three.
- `stitch(..., grid=...)` is what enforces that. Without a grid the canvas is
  sized to whichever cameras had data THIS cycle — right for the placement tool
  (a camera being dragged should grow the picture), wrong for a running
  pipeline, where one dropped frame would resize the world.
- A camera that stops delivering leaves its part of the canvas blank; it does
  NOT shrink the canvas and does not stop the app. Deliberate — a half-blank
  view is diagnosable, a silently re-scaled drawing is not.
- Changing the layout moves the projector's picture too: the full-frame mask is
  the whole canvas, so the projection corner-pin (`/projection?cal`) must be
  re-done after a layout change. The two calibrations use the same TL,TR,BL,BR
  convention on purpose. **A view rotation does the same thing** — the mask is
  the whole (turned) canvas, so re-do the corner-pin after pressing ⟳.
- The view rotation is the ONE thing about the canvas that may change mid
  session, and it is safe precisely because it does NOT re-freeze anything: the
  grid, the layout file and the stitch are untouched, the finished picture is
  only re-indexed. What it does change is width vs height, which `mm_per_px`
  divides by — so `set_view_rotation` republishes `frame_size`, and
  `on_rotate_view` re-bases everything shaped by the old orientation: the crop
  (`rotate_crop`), the reference frame (`rotate_image` — a full-canvas array),
  and the flat workspace's `y_extent` (`WorkspaceConfig.with_frame_aspect`,
  which only exists for this). The captured still and its strokes are DROPPED,
  exactly like Retake, because strokes are already projected into robot space
  and cannot be turned back. Don't try to "keep the capture" through a rotation.
- The crop is turned SERVER-side, not in the browser, and shipped back in
  `view_rotation{crop}`. It is the server's copy that Participant Mode and a
  reopened window read, so a browser-side turn would leave those pointing at
  the old sand — and two Developer windows would disagree. The browser's job is
  to render what comes back.
- The crop box is drawn in pixels over the depth `<img>`, and after a turn the
  new frame only arrives with the next MJPEG frame (~200 ms). A `ResizeObserver`
  on the depth images redraws it then; don't replace that with a one-shot
  `renderCrop()` at click time, which measures the OLD aspect.
- `rotate_view` is lock-gated (`_manual_locked`) like capture/generate/run:
  re-aiming the camera halfway through a participant's drawing would change what
  that drawing means. It is also deliberately NOT an MCP tool — dropping the
  capture is an at-the-rig decision.
- Safety constants (`MAX_TCP_SPEED`, `GOFA_REACH_M`, speeds/accels incl.
  `RRC_ACCEL_PCT`, `DRAW_Z`) only change on explicit user request.
- `RRC_ACCEL_PCT` is a percentage of the arm's rated acceleration, NOT m/s².
  Passing `DRAW_ACCEL` (0.3) through would read as 0.3% and the arm would crawl.
  It is one global setting shared by drawing and travel, so lowering it to
  smooth corners also slows the hops between strokes — and it PERSISTS on the
  controller, which is why `connect` sends it every time even at 100.
- The dispenser must fail CLOSED, not open. `set_dispenser(True)` raises (no
  material = a failed drawing, and it belongs in the error phase), but every
  close path swallows — `_draw_stroke`'s `finally`, `PathExecutor._dispenser_off`,
  `cancel`, and `_disconnect_unlocked`, which switches the output off BEFORE
  dropping the client because afterwards there is no way left to switch it.
  Don't move the open before the landing move_to: material would come out
  during the travel.
- Projection windows intentionally open on `127.0.0.1` (not localhost): Chrome
  caps 6 HTTP/1.1 connections per host and MJPEG streams hold theirs forever.
- `/`, `/projection`, `/depths`, `/static/*` are served no-cache — but Python
  changes still need an app restart.
- The rig's camera is MOUNTED AT AN ANGLE (the installation needs it), so the
  sand's own depth spans more than a hand's clearance above it. That is why the
  trigger and `ignore_closer_mm` go through `surface_height_mm` (stage 2b) and
  not raw depth. Don't "simplify" either back to an absolute compare — it
  works on a bench with a level camera and fails on the actual rig.
- Do NOT try to fix the tilt by remapping the depth NUMBERS (normalize the
  frame's min…max into a smaller range, auto-scale the colormap, etc.). Every
  such remap is monotonic in depth, so `hit = depth < threshold` selects exactly
  the same pixels with a relabelled threshold; the sand's spread and the hand's
  clearance are scaled by the same factor and their ratio — the thing that
  decides whether any cutoff can separate them — is unchanged. Tilt is a
  SPATIAL gradient; only a per-pixel baseline removes it. A per-frame min/max
  would also move every time a hand entered, which is precisely when it matters.
- Setting or clearing a reference silently changes what the trigger and the
  near-object box MEAN (height above sand ↔ camera distance), and the two want
  values an order of magnitude apart. Hence `state.reference_set`, the relabelled
  boxes, and `on_set_reference`'s warning when the existing trigger is still a
  camera-distance-looking number (`_ABSOLUTE_LOOKING_MM`). Keep all three: a
  trigger that quietly stops firing is the worst outcome here.
- `depth_region_labels` bands SIGNED values now, so band 0 is the sand itself.
  Invalid pixels use an int32-min sentinel, not 0 — the old `+1` offset would
  merge every invalid pixel into the sand's own band.
- Participant Sensing waits DEPTH_AVERAGE_FRAMES/DEPTH_FPS before capturing:
  the averaged still uses the PAST second, which would contain the hand
  otherwise. Keep that wait ≥ the buffer length.
- Blended linear motion assumes neighbouring waypoints don't flip the wrist —
  surface projection chains tool-X for minimal twist; keep that property.
- Multi-surface parts are NEVER re-centred — preserving the authored coordinates
  is the whole point (that is what keeps a multi-part Rhino export assembled).
  Don't "helpfully" normalize a part's origin, and don't give parts individual
  poses: one scene = one `SurfacePose`, which is what makes corner registration
  move the whole assembly. Note the drawing still fits the UNION bbox aspect, so
  parts placed far apart leave the centred drawing hovering over the gap between
  them (rays miss → empty path); that is correct "contain" behaviour, not a bug.
- A live run and a replayed bundle both go through PathExecutor with the same
  exec-bar Radius, so they trace identically — keep it that way. The clamp in
  `path_export.stroke_blend` (45% of the stroke's shortest segment) stays
  because a zone reaching half a segment has nothing left to round and the
  controller starts cutting the corner off; don't bypass it. `BLEND_ZONE_M` is
  only the default.
- The browser preview reads the Radius slider directly (`readBlendMm()` →
  `rebuildToolpathViz`); `exec_viz.blend_m` from capture_result is only the
  session echo.
- A pushed `preview_image` must never be able to fail a save: bad PNG, wrong
  serial, no Developer window — all just mean no preview.png, the same as
  before the push existed. `path_serial` is what keeps it honest; drop the
  serial check and a slow window's screenshot of the PREVIOUS drawing gets
  saved beside this one, which is worse than no picture at all. Note the push
  is gated on `autoLocked` in the browser, so Developer Mode still sends its
  screenshot the old way (with the Save click) and costs no extra traffic.
- Exec-bar controls split THREE ways: Spacing and **Distance Threshold** change
  the path GEOMETRY, so they re-send `generate_path` (server-side rebuild);
  Offset/Safety/Radius only re-draw client-side via `rebuildToolpathViz`; and
  **Max Total Length** changes nothing at all — it is a limit, so it only has to
  reach the server (`set_exec_params`) to be re-judged on Run/Save. Don't wire
  Distance Threshold into the client-side path — the browser has no copy of the
  pre-join chains.
- A **preset carries the exec bar too**, under a NESTED `exec` key beside the
  flat slider keys. Nested on purpose: the flat keys keep their old meaning, so
  presets written before this still load and their absent `exec` simply leaves
  the bar alone (`applyExecSettings` applies only fields that parse as numbers —
  never reset what a file does not mention). `readExecSettings`/
  `applyExecSettings` are the ONE reader/writer pair for that bar; the reconnect
  restore (`restoreSessionSettings`) goes through the same writer, feeding it
  `init.exec` merged with `detect.spacing_mm`/`join_mm` — two blocks on the
  wire, one bar on screen. Loading a preset then does what a manual edit does:
  `syncExecParams()` so the server's session copy (and therefore Participant
  Mode) matches, and — only if a path is already on screen — one
  `generate_path` for the new Spacing/Distance Threshold plus a
  `rebuildToolpathViz` for Offset/Safety/Radius. No clamping is added in the
  browser: the range inputs bound what can be typed and `main._num` clamps every
  value again server-side, so a hand-edited preset cannot push the arm past a
  limit.
- The exec bar is TWO rows (`#preview-controls` stack: `#path-legend` above
  `#exec-bar`): PATH = Spacing / Distance Threshold / Radius, RUN = Speed /
  Offset / Safety / Max Total Length / Save. `.exec-break` forces that split;
  each control is wrapped in `.exec-group` so a label never wraps away from its
  input. The stack pulls its right edge in to 290px while the Detection
  Parameters overlay is open (`#edit-panel:not(.hidden) ~ #panel-3d
  #preview-controls`) — that overlay is 270px wide at z-index 60 and used to
  swallow the bar's right-hand controls. Keep the rule keyed on the overlay's
  own `hidden` class: a popped-out preview lives in another document where the
  selector cannot match, which is right, because the overlay stays behind in the
  main window.
- path.json holds the SAME waypoints in two shapes — `frames` (COMPAS, mm, flat,
  for compas_rrc) and `strokes` (grouped, metres, what `toolpath_loader` and the
  replay tool read). Neither is redundant: drop `frames` and an RRC script has
  to convert; drop `strokes` and replay loses both the stroke boundaries (the
  tool would drag between gestures instead of retracting) and `meta`. Anything
  added to one must be derived from the same poses as the other — they are
  written from one list in `build_json`, keep it that way.
- The drawn-length readout is the SERVER's number, pushed in `state` and
  `capture_result`, never recomputed in the browser. One implementation means
  what the box shows is exactly what Run/Save judge; a JS copy would drift the
  moment either side changed. It follows the Radius slider because
  `on_set_exec_params` recomputes on every (debounced) sync — Radius changes
  the drawn length but deliberately never regenerates the path.
- `_length_check` recomputes from the strokes and blend about to be USED, not
  the value cached at Generate time. Trusting the cached number would let a
  Radius change between Generate and Run slip an over-length path through.
- `_segments_cross` deliberately requires strictly opposite orientation signs,
  so a stroke that merely touches or ends ON the connecting line (a T junction)
  does NOT earn the doubled join threshold — only one that truly passes through.
- The Max Drawing Time clock lives in `automation.py` and nowhere else: every
  method takes an optional `now` (monotonic s) so it is testable without
  sleeping, and the popup only RENDERS `remaining_s`. Don't add a JS timer —
  a browser-side countdown would drift from the clock that actually rejects the
  drawing, which is the one number a participant is watching. Same rule as the
  drawn-length readout.
- Don't drop the `_await_clear` latch after a time-out. The hand that ran out of
  time is still over the sand, and "Invalid" is in `_ARMED`, so without the
  latch the very next tick re-Alerts and the verdict disappears before anyone
  reads it — and the clock restarts on a drawing already refused.
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
- Sound plays in the projection OUTPUT window only, never `?cal`. Both pages
  are the same file, so lifting the `!IS_CAL` gate makes the laptop and the
  projector play the same buffer a few ms apart — which sounds like a fault,
  not like stereo. Same split as `projection_blank`.
- Audio must never be able to break the experience: a blocked AudioContext, a
  404 on a .wav, a browser with no Web Audio, or nobody clicking to unlock all
  mean SILENCE and nothing else. Keep every fetch/decode `.catch()`ed and keep
  `playCue` a no-op when the context is not running — same fail-open rule as
  the profanity guard.
- Cues fire on status ENTRY, and `lastStatus` is seeded from `init.participant`
  on purpose: without that, opening the projection window mid-drawing blurts
  the cue for a stage nobody just reached. Don't drive sound off a timer or off
  `below` — the state machine's statuses are the only trigger.
- The status→cue map lives in `config.SOUND_CUES` and travels in `init`. Don't
  hardcode a second copy in projection.html; a rename would then half-apply.
- Edit `sound_design.py` → RE-RENDER (`<ENVPY> sound_design.py`) and commit the
  .wav files. The code is not what plays; the rendered files are. A test asserts
  every configured cue exists under `sounds/`.
- `sounds/` is the one static route deliberately served WITH caching. Re-fetching
  a ~200 KB wav mid-experience is an audible stutter, and unlike viewer.js the
  files don't change between restarts.
- The alarm's grit uses a SEEDED rng — `render_all` must be deterministic or
  every regeneration shows up as a binary diff.
- Multi-Cam Vision has no auto-alignment and no detection ON PURPOSE. The
  cameras are bolted down, so an overlap search only ever failed on flat sand;
  and the tool exists to combine images — groove parameters belong in the main
  app, where they are actually tuned. Don't add either back "to help".
- The whole placement is `quad_mm`. Resist adding tx/ty/yaw/skew fields
  alongside it: two representations of the same thing is what made the previous
  UI unusable, and every one of those is a corner move.
- A camera's crop is applied by clearing `valid`, NOT by slicing the array —
  slicing would move the pixel coordinates the corner-pin is built on. Same
  reason `rotate_frame` rotates the intrinsics alongside the image.
- Changing a crop MUST go through `requad_for_crop` (the thread's
  `set_placement` does it), or trimming an edge stretches what is left over the
  same canvas area and slides the camera out of alignment.
- `_prepare` falls back to the default quad when a corner is dragged past its
  neighbours: a folded quad makes `warpPerspective` smear that camera across
  the canvas, and a blank view gives the operator nothing to drag back.
- Placements are bound to cameras by SERIAL (`bind_placements`), so unplugging
  a camera or changing USB port order does not shuffle the rig. The positional
  fallback exists only for calibrations saved before serials were recorded.
  `MultiCameraThread._sync_placements` then materializes default corners: the
  browser drags corners RELATIVE to `quad_mm`, so it must never be empty.
- `stitch.js` ignores the server's `calib` echo for ~700 ms after sending an
  edit. Drags are relative, so a stale echo would not just flicker the outline,
  it would make the NEXT delta start from the wrong corner.
- The panel's mm→px map (`panelMaps`) must NOT be keyed on the quad, and must
  not be re-fit mid-gesture. Both were the same bug: dragging one handle
  changed the quad's width and centroid, the panel re-derived its scale and
  centre from them, and all four handles moved. Keyed on `rot_deg` + panel
  size only; the crop is deliberately absent because trimming does not move the
  sand (requad), so the outline should sit still and re-fit when the re-cut
  corners land.
- The opening row's tile is FROZEN in `MultiCameraThread._tile_mm` when the
  camera set is bound, not re-derived per reset. Median depth wobbles a few
  tenths of a percent frame to frame; letting that into the layout put a
  reset camera ~1 cm out of line with neighbours reset a second earlier.
- `move_camera` orders cameras by `quad_centre_x`, NOT by index. Once the
  operator has dragged things around, USB enumeration order says nothing about
  which camera is on the left, and swapping by index would move the wrong pair.
  It also swaps POSITIONS, not quads — exchanging the quads outright would
  hand each camera the other's keystone.
- Keep the result view non-interactive. Handles used to live on the combined
  canvas and it read as two competing workspaces; the split is result-on-top,
  edits-below, and the top's SVG is `pointer-events:none` to enforce it.
- `stitch_server` serves `/static/*` no-store like the main app. A cached
  `stitch.js` against a restarted server is a browser talking a protocol the
  server no longer speaks, and it corrupts placements silently.
- The Scheduler's "Date and time" is a SAVE time, not a proven execution time.
  Don't relabel the column "executed at" without first making something
  actually record a run — and don't add that recording to the main app on the
  Scheduler's behalf; the tool is read-only by design.
- `scheduler.py` must not grow its own "is this a bundle?" rule. It calls
  `toolpath_loader.list_toolpaths` precisely so the Scheduler and the replay
  tool always list the same folders.
- `realsense_source` is the ONE place a RealSense is opened, and it returns
  either every camera or none — a partial start would place the cameras that
  did come up against a layout that assumed the missing one. Keep the colour
  frame `.copy()`ed there: `np.asarray(frame.get_data())` is a view of SDK
  memory and callers hold the latest colour for seconds.
- Test count reference: 611 unit (+6 hardware-gated). The `text_guard` OCR
  tests skip themselves when Tesseract is absent; the text-matching ones always
  run. Keep green.
