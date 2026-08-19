# depth-cam-to-robot

---

> **This is `sandskript_sybil_tracking_projecting`, one half of two.** It is a
> fork of `depth-cam-to-robot` — most of this README is still the parent
> project's own, describing a single self-contained app. **This fork no
> longer connects to or runs a robot.** Robot execution was removed (the
> "Connecting to the GoFa" section and the standalone Replay tool below no
> longer apply — `robot_controller.py`, `path_executor.py`, `registration.py`,
> `reach.py`, and `replay_*` were deleted); this app now stops at capture →
> groove detection → surface projection → **save & send**. The saved bundle
> goes out over ZeroMQ (`zmq_bridge.py`, port 5557) to
> `sandskript_sybil_rrc_dev` — a separate repo — which does the actual
> 301→302→304 conversion and robot run. See that repo's
> `documentation/README_pipeline.md` for what happens after this one hands
> off. `mcp_server/`, `.mcp.json`, and `CLAUDE.md` referenced further down
> were part of the parent project too and are not present in this fork.

## Overview

`depth-cam-to-robot` is the software of the **Sandskript** project, developed for **Sybil**, an interactive installation at **Ars Electronica 2026**. It watches a sandbox with one or more depth cameras — combined into a single wide view — detects the grooves a visitor rakes into the sand, converts them into strokes on a 3D target surface, and has a robot arm retrace the strokes, depositing living, seeded biomaterial on a tensile canvas.

---

## Installation

```bash
git clone https://github.com/zhoulin-ethaha/depth-cam-to-robot.git
cd depth-cam-to-robot
conda env create -f environment.yml   # creates the "sandskript" env (Python 3.11 + all deps)
conda activate sandskript
```

Requires [Miniconda](https://docs.conda.io/en/latest/miniconda.html) (or Anaconda). The Intel RealSense **USB driver** is an OS-level install and is *not* part of the environment — install it separately from the [librealsense releases](https://github.com/IntelRealSense/librealsense/releases).

**Dependencies:**


| Package                    | Purpose                                                      |
| -------------------------- | ------------------------------------------------------------ |
| `pyrealsense2 >= 2.54`     | RealSense depth capture                                      |
| `opencv-python >= 4.8`     | Depth filtering, colorizing, JPEG encoding                   |
| `scikit-image >= 0.22`     | Fast skeletonization (a pure-numpy fallback runs without it) |
| `compas_rrc >= 2.0`        | ABB robot control through the RRC task (linear moves, TCP frame) |
| `aiohttp >= 3.9`           | Async web server, MJPEG streaming, WebSocket                 |
| `numpy >= 1.26`            | Array operations                                             |
| `trimesh >= 4.0` + `rtree` | Target-surface mesh loading and ray-casting                  |
| `scipy >= 1.11`            | Rotations (surface-normal TCP orientations, retracts)        |
| `tesseract` + `libcurl`    | OCR engine for the Participant-Mode profanity guard — conda packages, so no OS-level install (`libcurl` is required: `tesseract55.dll` will not load without it) |
| `pytesseract >= 0.3.10`    | Python wrapper around the Tesseract binary above             |


---



## Running

```bash
python main.py        # or double-click run.bat
```

The browser opens at `http://localhost:5105` in **Developer Mode** (the full manual UI); **Participant Mode** is its **⧉ popup** on the Depth viewport. Closing the last browser window stops the server.

### Reading the terminal

The console tells you **which `.py` file did what**, so you can go from a behaviour to its source without hunting. On startup it prints a map of every feature to its modules, with `✓` for those actually imported in this process and `·` for those not:

```
── Python modules by feature ──────────────────────────────────────────────
   ✓ = imported in this process    · = not loaded

   Core / server       ✓ main.py  ✓ server.py  ✓ config.py  ✓ settings.py
   Capture             ✓ camera_thread.py  ✓ depth_extractor.py
   Groove detection    ✓ depth_extractor.py
   Stroke extraction   ✓ path_extractor.py
   Surface mapping     ✓ surface.py  ✓ workspace.py  ✓ registration.py
   Reach check         ✓ reach.py
   Execution           ✓ path_executor.py  ✓ robot_controller.py
   Export              ✓ path_export.py
   Participant Mode    ✓ automation.py  ✓ text_guard.py
```

Then every task prints the module chain that served it on the line below, in call order:

```
Captured still: 640×480 (depth+colour) — ready for crop/adjust
  └ camera_thread.py → depth_extractor.py
Generated path: 12 strokes, 340 points
  └ depth_extractor.py → path_extractor.py → surface.py → reach.py
[executor] starting path: 12 strokes, 5% speed (0.050 m/s), offset 0.0 mm, ...
  └ path_export.py → path_executor.py → robot_controller.py
[participant] REJECTED: drawing reads as offensive text ('****')
  └ text_guard.py → automation.py
```

The chain is *runtime-accurate*, not a fixed label: **Generate Path** shows `surface.py` with a mesh loaded and `workspace.py` in Test Mode, because that's what actually ran. Errors are attributed too, so a failure names its own module.

Turn either off with `SHOW_MODULE_BANNER` / `SHOW_MODULE_TRACE` in `config.py`.

---



## Hardware requirements


| Component       | Requirement                                                      |
| --------------- | ---------------------------------------------------------------- |
| Camera          | Intel RealSense D435i — **one to four**; all of them are used     |
| Camera position | Top-down, fixed mounts; together covering the full sandbox        |
| Camera layout   | Set once in **Multi-Cam Vision** and saved — the app reads it     |

No robot connects from this repo — see the note at the top of this file. The
robot and ROS-bridge requirements (ABB GoFa 10, Docker, `compas_rrc`) belong
to `sandskript_sybil_rrc_dev` instead.


---



## Connecting to the GoFa

**Not applicable in this repo.** The Connect / Run / Register-Corner controls
in the UI are stubs — clicking them returns "Robot execution is not available
in this repo" without attempting anything. There is no ROS bridge, no Docker
stack, and no `docker/` folder here any more. If you need to connect to and
run the GoFa directly, that lives in `sandskript_sybil_rrc_dev`, whose README
covers the Docker/rosbridge setup this section used to describe.


---



## The flow

The pipeline turns a raw depth frame into a saved, sent toolpath in eight
steps. Robot motion is not one of them any more — see the note at the top of
this file; step 8 hands off to `sandskript_sybil_rrc_dev` instead of driving
an arm directly.

```
depth-camera frames
    │
    ▼
combined view        every camera laid onto one picture, using the layout saved
    │                in Multi-Cam Vision (one camera → just that camera)
    ▼
groove regions       detect the mm-deep marks; tuned live with the parameters
    │
    ▼
centrelines          thin each groove region to a 1-px-wide skeleton line
    │
    ▼
joined strokes       connect stroke ends that nearly touch, so an interrupted
    │                groove becomes one continuous path (Distance Threshold
    │                box; default 0 mm = off)
    ▼
resampled strokes    drop a waypoint every 10–100 mm along each line
    │                (Spacing slider; default 10 mm)
    ▼
ordered strokes      choose the drawing order that minimises pen-up travel (TSP*)
    │
    ▼
surface projection   cast the strokes onto the 3D target mesh; the tool is kept
    │                perpendicular to the surface at every waypoint
    ▼
save & send          write path.json (+ preview/mask/skeleton) under paths/,
                     then push it to sandskript_sybil_rrc_dev over ZeroMQ
                     (port 5557) if a tile has been announced
```

**TSP** = the *Travelling-Salesman Problem*: visit every stroke once by the shortest total route. Here it's solved with a fast nearest-neighbour heuristic, so the robot wastes as little time as possible lifting and moving between strokes.

The eight steps group into four phases:


| Phase                   | Steps                                                                  | What the phase does                                         |
| ----------------------- | ---------------------------------------------------------------------- | ----------------------------------------------------------- |
| **Sensing**             | depth-camera frames → combined view                                    | Capture a clean, noise-averaged depth still of the whole sandbox |
| **Interpretation**      | groove regions → centrelines                                           | Turn raw depth into 1-px groove centrelines                 |
| **Path preparation**    | resampled strokes → ordered strokes → surface projection               | Turn the centrelines into an ordered 3D toolpath             |
| **Handoff**             | save & send                                                            | Save the bundle and, if a tile is active, send it onward     |




## The structure



### Program structure (the Hierarchy)

```
depth_cam-to-robot 
│
├─ MAIN APP
│   │
│   └─────── 🟢 Developer Mode  —  full manual UI: connect, tune, capture, generate, run, save
│     │
│     │  modules
│     │  ┌───────────┐ ┌──────────────────┐ ┌────────────────────┐ ┌───────────────────┐
│     │  │ server.py │ │ camera_thread.py │ │ depth_extractor.py │ │ path_extractor.py │
│     │  └───────────┘ └──────────────────┘ └────────────────────┘ └───────────────────┘
│     │  ┌──────────────────────┐ ┌─────────────┐
│     │  │ realsense_source.py  │ │ stitcher.py │  (all cameras → one combined view)
│     │  └──────────────────────┘ └─────────────┘
│     │  ┌────────────┐ ┌─────────────────┐ ┌──────────┐ ┌──────────────────┐ ┌────────────────┐
│     │  │ surface.py │ │ registration.py │ │ reach.py │ │ path_executor.py │ │ path_export.py │
│     │  └────────────┘ └─────────────────┘ └──────────┘ └──────────────────┘ └────────────────┘
│     │  ┌─────────────────────┐ ┌──────────────┐
│     │  │ robot_controller.py │ │ workspace.py │  (Test Mode)
│     │  └─────────────────────┘ └──────────────┘
│     │  UI
│     │  ┌───────────────────┐ ┌──────────────────┐
│     │  │ viewer/index.html │ │ viewer/viewer.js │
│     │  └───────────────────┘ └──────────────────┘
│     │
│     ├─ 🟣 Participant Mode  —  ⧉ popup: Auto toggle + depth trigger run the pipeline hands-free
│     │     modules
│     │     ┌───────────────┐ ┌───────────────┐
│     │     │ automation.py │ │ text_guard.py │  + the 🟢 pipeline modules it re-drives
│     │     └───────────────┘ └───────────────┘
│     │     UI
│     │     ┌────────────────────────┐ ┌─────────────────────────┐
│     │     │ viewer/depth_view.html │ │ viewer/depth_overlay.js │
│     │     └────────────────────────┘ └─────────────────────────┘
│     │
│     └─ 🟠 Projection  —  projector shines the detected mask back onto the sand,
│                          and its speaker plays the participant sound cues
│           modules
│           ┌──────────────────┐ ┌───────────┐ ┌─────────────────┐
│           │ camera_thread.py │ │ server.py │ │ sound_design.py │
│           └──────────────────┘ └───────────┘ └─────────────────┘
│            (mask composition)                 (renders sounds/*.wav)
│           UI
│           ┌────────────────────────┐
│           │ viewer/projection.html │  (corner-pin calibration + cue playback)
│           └────────────────────────┘
│
├─ MULTI-CAM VISION  ·  separate app — its saved layout IS the main app's camera
│  └─ 🔵 Stitching  —  lay every RealSense feed onto one canvas by dragging its corners
│        modules
│        ┌─────────────┐ ┌─────────────────┐ ┌──────────────────┐ ┌────────────────────┐
│        │ stitcher.py │ │ multi_camera.py │ │ stitch_server.py │ │ depth_extractor.py │
│        └─────────────┘ └─────────────────┘ └──────────────────┘ └────────────────────┘
│        ┌─────────────────────┐
│        │ realsense_source.py │  (shared with the main app: opens the cameras)
│        └─────────────────────┘
│        UI
│        ┌────────────────────┐ ┌──────────────────┐
│        │ viewer/stitch.html │ │ viewer/stitch.js │
│        └────────────────────┘ └──────────────────┘
│
├─ SCHEDULER  ·  contained tool (read-only)
│  └─ 🟡 Ledger  —  every saved path as a numbered spreadsheet: which, and when
│        modules
│        ┌──────────────┐ ┌─────────────────────┐ ┌────────────────────┐
│        │ scheduler.py │ │ scheduler_server.py │ │ toolpath_loader.py │
│        └──────────────┘ └─────────────────────┘ └────────────────────┘
│        UI
│        ┌───────────────────────┐ ┌─────────────────────┐
│        │ viewer/scheduler.html │ │ viewer/scheduler.js │
│        └───────────────────────┘ └─────────────────────┘
│
├─ TOOLPATH REPLAY  ·  contained tool
│  └─ ⚪ Replay  —  re-run a saved bundle from paths/ without the camera
│        modules
│        ┌────────────────────┐ ┌──────────────────┐ ┌─────────────────┐
│        │ toolpath_loader.py │ │ replay_server.py │ │ replay_robot.py │  (current UR, ABB-ready)
│        └────────────────────┘ └──────────────────┘ └─────────────────┘
│        ┌──────────────────┐ ┌─────────────────────┐
│        │ path_executor.py │ │ robot_controller.py │
│        └──────────────────┘ └─────────────────────┘
│        UI
│        ┌────────────────────┐ ┌──────────────────┐
│        │ viewer/replay.html │ │ viewer/replay.js │
│        └────────────────────┘ └──────────────────┘
│
└─ MCP SERVER
   └─ 🤖 AI tools  —  drive the pipeline via the running app's HTTP/WS API
         modules
         ┌──────────────────────┐
         │ mcp_server/server.py │
         └──────────────────────┘
```



### File structure (directory style)

```
depth_cam-to-robot/
├── main.py                  🟢🟣🟠 Entry point: shared state, callbacks, startup, TCP poller
├── automation.py            🟣 Participant-Mode state machine (trigger → auto pipeline)
├── text_guard.py            🟣 Profanity guard: OCR the mask, reject offensive drawings
├── sound_design.py          🟣🟠 Composes the four participant sound cues → sounds/*.wav
├── module_trace.py          🟢🟣🟠 Console: startup feature→module table + per-task module trail
├── config.py                🟢🟣🟠🔵⚪🤖 All configurable parameters
├── server.py                🟢🟣🟠🤖 aiohttp server: MJPEG feeds, WebSocket, surface upload
├── camera_thread.py         🟢🟣🟠 DepthCameraThread: every RealSense → ONE combined view → depth/RGB/skeleton/mask streams
├── realsense_source.py      🟢🟣🟠🔵 Opens and reads the cameras (the one place devices are started)
├── view_rotation.py         🟢🟣 Quarter-turns the combined canvas (the ⟳ button) — image, frame size and crop together
├── depth_extractor.py       🟢🟣🔵 Depth → groove engine: colorize, detect, filter, skeletonize
├── path_extractor.py        🟢🟣 Grooves → pixel chains → smooth → resample → TSP
├── surface.py               🟢🟣 Target mesh: STL/OBJ load, multi-surface scene, projection, normal TCP orientations
├── registration.py          🟢 Corner→TCP touch-off placement (1-point + Kabsch ≥3-point)
├── path_export.py           🟢🟣⚪ Save toolpath → JSON (poses+frames) + preview/mask/skeleton PNGs
├── path_executor.py         🟢🟣⚪ Background thread: retract/travel/blended draw per stroke, progress
├── robot_controller.py      🟢🟣⚪ Thread-safe compas_rrc wrapper (linear moves, blended runs, TCP frame)
├── workspace.py             🟢 Planar fallback mapping (Test Mode)
├── reach.py                 🟢🟣 Reach-envelope estimate (importable without hardware)
├── stitcher.py              🔵🟢🟣 Corner-pin placement + canvas warping math (the tool AND the app's combined view)
├── multi_camera.py          🔵 Multi-Cam Vision: owns every connected RealSense pipeline
├── stitch_server.py         🔵 Multi-Cam Vision: aiohttp server (port 5106)
├── stitch_main.py           🔵 Multi-Cam Vision entry point (run_stitch.bat)
├── scheduler.py             🟡 Scheduler: read paths/ into numbered ledger rows (pure, read-only)
├── scheduler_server.py      🟡 Scheduler: aiohttp server (port 5108)
├── scheduler_main.py        🟡 Scheduler entry point (run_scheduler.bat)
├── toolpath_loader.py       ⚪🟡 Replay tool: read saved bundles (path.json OR path.script)
├── replay_robot.py          ⚪ Replay tool: robot-brand abstraction (UR now, ABB-ready)
├── replay_server.py         ⚪ Replay tool: aiohttp server (port 5107)
├── replay_main.py           ⚪ Replay tool entry point (run_replay.bat)
├── settings.py              🟢🟣🟠⚪ Persistent JSON settings (last robot IP + projector corners)
├── mcp_server/              🤖 FastMCP tools wrapping the app's HTTP/WS API
├── .mcp.json                🤖 Registers the MCP pipeline server (project scope)
├── CLAUDE.md                🤖 AI-assistant repo guide (pipeline, API, gotchas)
├── environment.yml          🟢🟣🟠🔵⚪🤖 Conda-env recipe ("sandskript": Python 3.11 + all deps)
├── requirements.txt         🟢🟣🟠🔵⚪🤖 pip dependencies (installed by environment.yml)
├── requirements-dev.txt     🟢🟣🟠🔵⚪🤖 dev extras: pytest, mcp
├── run.bat                  🟢🟣🟠 Main-app launcher (double-click)
├── run_stitch.bat           🔵 Multi-Cam launcher
├── run_scheduler.bat        🟡 Scheduler launcher
├── run_replay.bat           ⚪ Replay launcher
├── conftest.py              🟢🟣🟠🔵⚪🤖 Pytest shared fixtures
├── pytest.ini               🟢🟣🟠🔵⚪🤖 Test configuration
├── settings.json            🟢🟣🟠⚪ Auto-generated: last robot IP, projector corners, view rotation (gitignored)
├── surfaces/                🟢🟣 Uploaded target meshes (gitignored)
├── paths/                   🟢🟣⚪ Saved toolpaths: dated folders of .script/.json/.png (gitignored)
├── presets/                 🟢 Saved parameter files (detection sliders + Path Preview bar), named by date (gitignored)
├── wordlists/               🟣 Profanity wordlists (en/de seed; add LDNOOBW .txt files here)
├── sounds/                  🟣🟠 The four cue .wav files (committed; replace with your own)
├── tests/                   🟢🟣🟠🔵⚪🤖 Unit + hardware-gated integration tests
└── viewer/
    ├── index.html           🟢 Single-page app
    ├── viewer.js            🟢 WebSocket client, UI handlers, Three.js 3D path preview
    ├── projection.html      🟠🟣 Projector output / corner-pin calibration + the sound cues
    ├── depth_view.html      🟣 Participant Mode popup (depth numbers + Auto + trigger + time limit)
    ├── depth_overlay.js     🟣 Popup logic: number overlay, Auto toggle, status chip, countdown
    ├── stitch.html          🔵 Multi-Cam Vision prototype UI
    ├── stitch.js            🔵 Multi-Cam Vision logic (corner handles, crop drags)
    ├── scheduler.html       🟡 Scheduler spreadsheet UI
    ├── scheduler.js         🟡 Scheduler logic (live table from the paths folder)
    ├── replay.html          ⚪ Toolpath replay tool UI
    ├── replay.js            ⚪ Replay UI logic (connect, pick bundle, run)
    ├── style.css            🟢 Responsive layout
    └── lib/
        ├── three.min.js     🟢 Three.js (3D rendering)
        └── OrbitControls.js 🟢 Mouse/touch orbit controls
```

Feature tags:

🟢 Developer Mode · 🟣 Participant Mode · 🟠 Projection · 🔵 Multi-Cam · 🟡 Scheduler · ⚪ Replay · 🤖 MCP

---



## Guide pour les nuls

The Developer-Mode workflow, step by step.

1. **Connect** — enter the robot's IP (e.g. `192.168.1.100`) and click **Connect**.
2. **Load the drawing target** — mesh your Rhino surface, export it as **STL/OBJ in millimetres**, and load it at the prompt. Load **more than one file** to build a multi-part target: each keeps the position it was authored at, and they then behave as a single surface that moves as one (see *Loading several surfaces*). There is no manual robot-calibration step: the surface's position relative to the robot is set with the Surface X/Y/Z + rotation sliders (or corner touch-off) and verified visually in the Path Preview.
3. **Aim the RealSense** straight down so it covers the whole sandbox. The four viewports show **Depth** (near = blue → far = red), **RGB**, **Skeleton** (the 1-px centrelines that become the path) and **Mask** (the thick detected region — shows groove *width*, handy while tuning). The **⧉ Participant Mode** popup (Depth viewport) adds the live depth view with **absolute mm-from-camera** labels per iso-depth region (**Region interval** and **Text size** sliders; display-only, computed only while the popup is open) and holds the **Auto** toggle + **Trigger below** box that automate the pipeline — see *Participant Mode* below.

   If the camera view comes out sideways relative to the sandbox, press **⟳** on the Depth viewport: each press turns the whole camera view 90° clockwise, and the button shows the current angle. This is one setting for the whole pipeline, not a display trick — **RGB, Skeleton, Mask, the projection and Participant Mode all turn with it**, the crop follows the sand it was framing, and any reference frame is turned to match. Because a still captured at the old angle would no longer line up, pressing ⟳ drops it and returns you to the live view: **rotate first, then Capture**. The angle is remembered in `settings.json`, so it survives a restart. Two things to know: the projector's corner-pin must be re-done after a turn (the projected mask is the whole, now-turned view), and the button is greyed out while Participant **Auto** is on — re-aiming mid-drawing would change what that drawing means. This is separate from the per-camera rotation in Multi-Cam Vision, which describes how one camera is mounted; use ⟳ when the whole picture needs turning and you don't want to stop the app.
4. **Tune detection live** — the **Detection Parameters** panel works *before* capturing: pick a **Mode** (Valley / Ridge / Band) and adjust **Groove depth**, **Surface scale**, **Denoise**, **Min blob**; the viewports update in real time. Drag a **crop** rectangle on the Depth view to limit the region. **Save** stores the sliders to a dated file under `presets/`, **Load** restores one, **Reset** returns the sliders to defaults.

   A saved file holds **more than the sliders**: it also carries the whole Path Preview bar — **Spacing**, **Distance Threshold**, **Radius**, **Speed**, **Offset**, **Safety** and **Max Total Length** — because those decide what the same detection settings actually draw. Loading a file therefore restores a complete working setup: the sliders, the mode, and the bar. If a path is already on screen it is re-generated at the loaded Spacing and Distance Threshold, and the loaded values become the ones Participant Mode uses too. Files saved before this restore the sliders only and leave the bar untouched — nothing to re-save, they simply keep working. **Reset** is unchanged: it resets the detection sliders, not the bar.
5. **Capture Image** — freezes a temporally averaged depth (+ aligned colour) still; the crop carries over (drag inside to move, corners to resize, **Reset Crop** for full frame). Detection — and the generated path — cover only the crop.
6. **Generate Path** — the 3D viewer shows the surface, the detected skeleton as a **white** on-surface line, and the toolpath: **green** blended segments with waypoint dots (**red** = outside estimated reach), **amber** safety/retract points, **grey** pen-up travels. **Spacing** (10–100 mm) sets waypoint distance and regenerates on release; **Distance Threshold** (0–200 mm, default 0 = off) merges strokes whose ends nearly touch — see below — and also regenerates; **Radius** (0–50 mm, default 0.5) is the corner zone — how far before a waypoint the arm may start rounding into the next segment; clamped per stroke to 45 % of the shortest segment, since a zone reaching half a segment has nothing left to round and the corner gets cut off instead — so a radius larger than the Spacing allows is not an error, it simply rounds as much as each corner has room for; Offset/Safety edits update the preview live. The settings bar across the bottom of the preview is in two rows — **Path** (Spacing, Distance Threshold, Radius: what shapes the drawn line) above **Run** (Speed, Offset, Safety, Max Total Length, Save Path: what the click uses) — and it keeps clear of the Detection Parameters panel while that is open, so no control is ever hidden underneath it. **Path | Order** switches to a numbered stroke-order view; **⧉ Pop out** opens the preview in its own window. Re-tune and regenerate freely, or **Retake**.
7. **Run** — set **Speed** (% of max TCP speed, governs the *entire* motion), **Offset** (mm off the surface along the local normal), **Safety** (retract mm) and **Radius**, then Run. The blue dot tracks the live TCP; a progress bar tracks execution; **Cancel** stops mid-stroke. A live run and a replayed bundle go through the same executor with the same Radius, so they trace identically. **💾 Save Path** writes the toolpath — current settings baked in — to a timestamped folder under `paths/` (see *Saving toolpaths*).

### Distance Threshold — joining broken strokes

A single raked gesture rarely survives detection as one stroke: a shallow patch,
a crossing groove, or a fleck of shadow splits it into fragments, and the robot
then lifts, travels and re-approaches in the middle of what should be one line.
The **Distance Threshold** box (mm, in the Path Preview bar) stitches those
fragments back together before the waypoints are laid down.

- **What counts** — only stroke **endpoints**, and only across *different*
  strokes. Direction is irrelevant: a start may join a start, an end an end.
  A stroke can never join to itself, so a near-closed curve stays open.
- **The doubling rule** — if drawing the straight line that would close a gap
  means crossing *another* stroke, the threshold **doubles** for that pair. A
  crossing groove is the most common reason a gesture got cut in two, so those
  ends are given twice the benefit of the doubt. A stroke that merely touches or
  ends on that line does not count — it has to genuinely pass through.
- **One partner each** — when several ends qualify, the **nearest** wins.
  Candidates are settled shortest-gap-first, so no end is claimed by a distant
  neighbour just because it was checked first.
- **Never a loop** — a join that would close a chain back on itself is refused,
  so the output is always open polylines.

Set it to **0 to switch joining off** (the default — every stroke stays as
detected). Changing it re-generates the path, exactly like Spacing. What you see
in the preview *is* what runs: the white skeleton line, the green toolpath, the
stroke count and the saved bundle all come from the joined strokes.

Joining happens **before** resampling, so waypoints are spaced evenly straight
across a seam — a merged stroke is indistinguishable from one that was never
broken.

### Max Total Length — a ceiling on how much gets drawn

The box beside Distance Threshold caps how far the tool may travel **while drawing**. Enter a length in mm; **0 switches it off** (the default). The current path's length is shown next to the box, and turns **red** when it is over.

Over the limit, the path is simply refused:

- **Developer Mode** — **Run** and **💾 Save Path** both decline with a message saying the actual length and the limit. Nothing else changes: the path stays on screen, so you can raise the limit, raise the Spacing, or re-rake and try again.
- **Participant Mode** — the drawing is marked **Invalid**, exactly like a profanity hit: red chip, nothing saved, nothing sent to the robot, and the verdict stays on screen for whoever drew it.

**What the number actually measures.** Only the drawing motion — the green line (red where out of reach). Pen-up travels, retracts and approach moves are excluded, because the point of the limit is to bound how much material goes down, not how far the arm moves. It is measured on the strokes **after** they have been projected onto the loaded surface, so it is a real distance across that surface at true scale — a path running up a slope counts as longer than its flat footprint, which is exactly the case a flat measurement would miss.

All three of the other path controls are already inside the number:

- **Spacing** — the waypoints *are* the resampled path, so a coarser spacing genuinely shortens it (it cuts corners).
- **Distance Threshold** — a join closes a gap by connecting two strokes, and that connecting segment is drawn, so it counts.
- **Radius** — rounding a corner is shorter than going out to it and back, so more Radius means a slightly shorter path. It is clamped exactly as the executor clamps it, so the figure describes the motion the robot is actually driven through.

Because Radius changes the length without changing the path, the readout updates as you drag that slider, and **Run/Save re-measure at the moment you click** — the limit is judged against the settings in front of you, not the ones in force when the path was generated.

---



## Dry Knowledge



### Why "valley detection", not a fixed depth band

Real sand surfaces sag and tilt, so a fixed absolute depth band picks up the *slope*, not the marks. Instead the smooth bare-sand surface (a heavily blurred copy of the depth map) is estimated and subtracted, leaving only the **local relief**: a groove is simply "a few mm deeper than its immediate surroundings", regardless of tilt. (An absolute iso-depth band is still available via **Band** mode.)

### The stages (`depth_extractor.grooves_from_depth`)

**1 Gap fill** — invalid depth pixels (0 / NaN) filled from the nearest valid neighbour, so blurring doesn't bleed holes. **2 Denoise** (`smooth_sigma_px`) — small Gaussian against per-pixel noise. **3 Detrend** (`detrend_sigma_px`) — subtract the large-radius-blurred surface → relief in mm (positive = deeper). **4 Threshold** (`groove_depth_mm`, mode below). **5 Clean** — morphological close bridges 1-px gaps, blobs under `min_blob_px` dropped. **6 Skeletonize** — thin to 1-px centrelines (scikit-image, else opencv `ximgproc.thinning`, else a pure-numpy Zhang-Suen fallback).


| Mode               | Keeps                                                                        |
| ------------------ | ---------------------------------------------------------------------------- |
| `valley` (default) | relief deeper than `groove_depth_mm` — the grooves                           |
| `ridge`            | relief raised more than `groove_depth_mm` — bumps/ridges                     |
| `band`             | relief within `band_center_mm ± band_width_mm` — an absolute iso-depth slice |




### From centrelines to robot strokes

`_chains_from_edges()` walks each centreline via 8-connectivity, starting from endpoints (≤1 neighbour) and removing visited pixels — every pixel visited exactly once, each chain an ordered tip-to-tip path. (`cv2.findContours` would trace each thin line down one side and back, drawing it twice.)

### Tuning


| Goal                       | What to change                                                |
| -------------------------- | ------------------------------------------------------------- |
| Catch fainter grooves      | Lower `groove_depth_mm`                                       |
| Reject noise / grain       | Raise `groove_depth_mm`, or raise `smooth_sigma_px` (Denoise) |
| Flatten broad undulations  | Lower `detrend_sigma_px` (Surface scale)                      |
| Keep thin marks            | Lower `smooth_sigma_px`                                       |
| Discard speckle            | Raise `min_blob_px`                                           |
| Trace raised lines instead | Switch **Mode** to `ridge`                                    |


The single biggest quality win is **temporal averaging**: the sand is static, so Capture averages `DEPTH_AVERAGE_FRAMES` frames, cutting per-pixel depth noise by ~√N before any detection runs.

### Rejecting natural grooves

Pre-existing ripples/texture can look like grooves. Four independent filters (**Reject natural grooves** panel; each disabled at 0) suppress them: **Reference subtraction** (`ref_strength`) — capture the *undrawn* sand with **Set Reference**; pre-existing grooves appear in both frames and **cancel**, leaving only what was drawn (the most reliable discriminator; camera + sandbox must stay still). **Min mean depth** — drop grooves whose *average* relief is shallow (raked grooves are consistently a few mm deep, faint ripples aren't). **Min / Max width** — keep only grooves matching the raking tool's width. **Min length** — drop short fragments of natural texture. (Width/length get their mm scale from the drawing's fit onto the surface, or the Test-Mode workspace.)

### Ignoring objects above the sand

Detection is *relative* (mm below the local surface), so a hand raking or a person leaning over creates phantom relief. The box on the Mask viewport (always visible) drops any groove blob touching such an object, grown by a safety margin — from live views, projection and path generation alike. 0 or empty disables it.

What the box measures depends on whether a reference is set, and it relabels itself so you can see which:

- **Reference set → "Ignore above sand"** — a height above the sand surface. Anything standing more than this many mm proud of the sand is an object. Try **30–60 mm**. This is the one to use, and the only one that works on a tilted camera.
- **No reference → "Ignore closer than"** — an absolute distance from the camera. Set it a little nearer than the sand (read it off the Participant popup's labels).

### Tilted cameras, and why "height above the sand" matters

If the camera is mounted at an angle — as it often must be, to fit the installation — the sand itself is nearer at one end of the box than at the other. A 15° tilt across a 600 mm sandbox puts ~160 mm between the two ends. Since a hand only clears the sand by 50–150 mm, **the sand's own depth range swallows the hand's**, and no absolute cutoff can tell them apart: set it low and nothing ever fires, set it high and the near end of the *sand* trips it permanently. Both the Participant trigger and the near-object box hit this.

**Press Set Reference and the problem disappears.** A reference is a snapshot of the empty sand, so it contains the tilt; subtracting it, pixel by pixel, leaves height above the sand — 0 on untouched sand anywhere in the box, negative in a groove, strongly positive under a hand. One number then means the same thing at both ends, at any camera angle. The Participant popup's labels switch to the same units (`+90` for a hand, `−6` for a raked groove), so you can read a trigger value straight off the picture instead of guessing.

Worth knowing: it has to be a *per-pixel* baseline. Rescaling the depth numbers as a whole — compressing the frame's min–max into a narrower range, for instance — cannot help, because it shrinks the sand's spread and the hand's clearance by the same factor and leaves their ratio exactly as it was; a threshold picks out precisely the same pixels, just with a different number typed into the box. A tilt is a *positional* effect, so only something that knows the sand's depth *at each pixel* removes it.

If the sand is re-levelled deeply or the camera is bumped, capture the reference again.

### Participant Mode (automated pipeline)

The **⧉ Participant Mode popup** replaces the buttons with a **depth trigger**: a participant rakes, pulls their hand out, and the robot retraces — no clicks. Enter a trigger threshold, then switch **Auto ON**. The box carries the same unit as the depth labels beside it, and relabels itself to say which:

- **"Trigger above sand"** (a reference is set) — fires when something rises more than this many mm above the sand. Try **60–120 mm**. Unaffected by camera tilt; use this one.
- **"Trigger below"** (no reference) — fires when something comes closer to the camera than this. Sand at 900 mm → e.g. 700. Fine on a level camera; on a tilted one see *Tilted cameras* above.

Pressing **Set Reference** switches between the two, so a value tuned in one mode will be wrong in the other — the app says so when you capture a reference while a camera-distance-sized trigger is still in the box. The popup shows only the Developer-Mode crop — the labels and the trigger watch that region too; the crop itself can only be changed in Developer Mode. Statuses appear large in the popup's top-right:


| Status               | Meaning                                                           |
| -------------------- | ----------------------------------------------------------------- |
| **Auto Off**         | Toggle off — the popup is just the depth-number viewport.         |
| **Auto On**          | Armed; nothing in frame is closer than the trigger.               |
| **Alerted**          | Something closer than the trigger is in frame (a hand raking).    |
| **Sensing**          | Frame stayed clear for ~1 s → capturing the averaged depth still. |
| **Generating Paths** | Extracting strokes and building the toolpath.                     |
| **Actuating**        | Saving the bundle to `paths/` and running it on the robot.        |
| **Invalid**          | The drawing was refused — profanity guard, over the Max Total Length, or out of drawing time. Nothing saved, nothing run. |


After Actuating it returns to **Auto On**, ready for the next participant. While Auto is **ON**, the manual Capture / Retake / Generate / Run buttons grey out (the server also refuses them) — **Cancel stays active** as the emergency stop. Worth knowing: the automated run reuses the **same pipeline and current settings** as the Developer-Mode buttons (set everything up, then flip Auto ON; the Developer window shows each step live); an empty trigger box can never fire; without a robot the toolpath is still generated and **saved**, only the run is skipped; Auto stays ON server-side even if the popup closes; Sensing deliberately waits ~1 s so the averaged still doesn't contain the hand.

#### Max drawing time

Next to the trigger box is **Max drawing time (min)** — how long one participant may keep the sand to themselves. The clock starts the instant the trigger trips (a hand enters) and stops when the frame is clear again (the hand leaves); it does **not** count the pipeline afterwards, so the limit is drawing time, not cycle time. Leave the box empty for no limit.

A **countdown** in the popup's opposite corner (top-left) shows the time: dim grey with the full allowance while the sand is free, blue and counting while someone draws, and **red and blinking for the last 10 seconds** as a warning. Run out and the drawing is **Invalid** — the same verdict as a profanity hit: nothing saved, nothing sent to the robot, and a message telling the participant to rake it over so the next person can start. The verdict holds while their hand is still in frame and only re-arms once the sand is clear, so it can't be missed.

The countdown is the server's clock, not the browser's — every open popup shows the same number, and it is the same clock that judges the drawing.

#### Sound cues

Participant Mode talks back. Four short pieces mark the moments a participant needs to know about, and they come out of **the projector's speaker** — they are played by the projection window, so **no projection window open means no sound**, and the projector's audio has to be the Windows output device (in *Sound settings → Output*, pick the projector / the HDMI device).

| When                                 | Cue            | What it says                                                              |
| ------------------------------------ | -------------- | ------------------------------------------------------------------------- |
| A hand enters the sand (**Alerted**) | `engaged`      | Rising major triad — *I see you, go ahead.*                               |
| The hand leaves (**Sensing**)        | `acknowledged` | The same notes falling to the root, held and pulsing — *got it, working.* |
| Saving and running (**Actuating**)   | `anticipation` | Five notes rising and accelerating over a swell — *here it comes.*        |
| The drawing is refused (**Invalid**) | `alarm`        | A harsh two-tone klaxon a tritone apart — unmistakably a *no*.            |

The cue plays once, when the status is reached; *Generating Paths* is deliberately silent so the sequence stays legible. Everything Participant Mode already refuses — the profanity guard, **Max Total Length**, running out of **Max drawing time** — lands on the same alarm.

- ▶ **One click to start.** Browsers refuse to play audio until the page has been clicked, so the projection window shows *"🔊 click anywhere to enable sound"* until you do. Click it once while setting up (before F11) and it never asks again for that session. Sound is an added layer: if the click never happens, the experience runs exactly as it did before, silently.
- 🔇 **M** mutes and un-mutes the projection window. **C** (handles) and **B** (blank) are unchanged.
- 🔊 **Only the output window makes sound.** The `?cal` calibration window on the laptop stays silent, so the two never play the same cue a few milliseconds apart.
- 🎚 **Replace them with your own.** The files are `sounds/engaged.wav`, `acknowledged.wav`, `anticipation.wav`, `alarm.wav` — drop any WAV in under those names. To regenerate the synthesized ones (or tune them: the pitches, lengths and levels are single lines in `sound_design.py`, with the reasoning for each choice written above them), run `python sound_design.py`. Changing *which status plays what* is `SOUND_CUES` in `config.py` — the page reads that map from the server, so nothing else needs editing.

#### Profanity guard

Because Participant Mode runs unattended, it reads the sand before the robot does. Between *Generating Paths* and *Actuating*, the detected **mask** is passed through OCR (English + German); if what was raked spells something on a wordlist, the run stops: the chip turns red and reads **Invalid**, and **nothing is saved and nothing is sent to the robot**.

**Invalid is sticky** — it stays on screen so whoever drew it sees the verdict, while the trigger stays armed. The next participant clears it; so does toggling Auto off and on. The **Max Total Length** limit (above) and the **Max drawing time** use the same verdict, and for the same reason: all three mean *this drawing cannot be used*, rather than *something went wrong*.

- **Where it looks** — the mask, not the skeleton. Thick strokes are what OCR can read; 1-px centrelines are not.
- **What it reads** — the mask upright *and* upside down (participants write from the far side of the sandbox), in both black-on-white and white-on-black. Four quick passes, once per capture — no effect on the live feeds.
- **What counts** — case, umlauts/ß, accents and leetspeak (`5H1T`) are all folded before matching, and OCR's broken spacing is handled by also testing the words run together. Entries under 4 letters only match as standalone words, so `assist` and `classic` are safe.
- **Wordlists** — every `.txt` in `wordlists/`. A short English and German seed list ships with the repo; for real coverage drop in the [LDNOOBW](https://github.com/LDNOOBW/List-of-Dirty-Naughty-Obscene-and-Otherwise-Bad-Words) `en` and `de` files (rename to anything ending `.txt`). All files are merged — no code change needed.

**It fails open, on purpose.** If Tesseract is missing, the wordlist is empty, or OCR errors out, the drawing goes through and a line is printed to the console. An installation should not stop dead because an OCR dependency is absent. ⚠️ The flip side: this is a coarse filter, not a guarantee — handwriting in sand is genuinely hard to read, so expect misses. It does not detect offensive **symbols** at all, only text. Keep a human able to hit **Cancel**.

Turn it off with `PROFANITY_CHECK_ENABLED = False` in `config.py`. It never runs in Developer Mode, where an operator is present to judge for themselves.

### Test mode (no robot)

**Test Mode (no robot)** sets a synthetic workspace so the depth → groove → path-preview pipeline can be exercised without a robot. Run stays gated on a real connection.

### How the drawing maps onto the surface

**1.** In Rhino, `Mesh` the surface and **export as STL/OBJ in millimetres**. It may be flat, tilted or vertical — projection follows the mesh's dominant (area-weighted) face normal, and the drawing lands on the side the normals point (flip with `Dir` if paths appear on the back). Exception: a **steep surface** (more than ~45° from horizontal) always draws on the **side facing the robot base**, so a positive TCP offset always moves the tool *toward* the robot and never behind a wall. **2.** The full camera frame (4:3) is **fitted centred** onto the surface's footprint, aspect preserved — each stroke lands at the same relative position it has in the camera view; cropping only selects which grooves exist, it doesn't move or zoom the drawing. **3.** Every waypoint gets a tool orientation **perpendicular to the surface** with minimal wrist twist; rays that miss the mesh split the stroke. **4.** Placement is live: the **Surface X/Y/Z + Rot X/Y/Z** sliders position the mesh in the robot base frame; the **TCP offset (mm)** slider bakes a hover distance at Generate time, the execution bar's **Offset** adds more at Run time. **5.** Contact depth comes from the offsets (planar `DRAW_Z` is not applied in surface mode); retracts follow the tool axis, pulling *away* from tilted surfaces. **Clear All Surfaces** returns to the flat Test-Mode mapping. If the robot draws on a *real* surface, the virtual placement must match reality — verify with the preview and a slow, offset-first run.

### Loading several surfaces

**Load Surface** is cumulative — press it again (or pick several files at once) to build a **multi-part target**. Each file keeps **the position it was authored at**, so a Rhino document exported as several meshes reassembles itself: nothing is re-centred, and the gaps and offsets between the pieces are exactly as modelled. Export them all from the same Rhino origin and they land correctly relative to one another.

Once loaded, the pieces behave as **one surface**: the drawing is fitted across the whole assembly (one drawing spread over all parts, not a copy per part), rays land on whichever piece is nearest, and the **Surface X/Y/Z + Rot** sliders and **Register Corner → TCP** move them **together** as a rigid group — the registration corners come from the assembly's overall bounding box, so touching off one corner places everything at once and the parts can never drift apart.

Each loaded file gets a row in the panel with a **✕** to remove just that one; **Clear All Surfaces** removes them all. Loading a file whose name is already in the list **replaces** that part in place — re-export from Rhino and load it again to update one piece without touching the others.

One consequence of the aspect-preserving fit: because the 4:3 camera frame is centred inside the assembly's overall bounding box, parts placed far apart leave the drawing hovering over the empty space between them, and the strokes fall off into the gap. Keep the pieces adjacent, or expect the drawing to cover only the middle of the span.

### Register Corner → TCP (touch-off placement)

Measure the placement with the robot instead of guessing sliders. **Register Corner → TCP…** (Target surface section) opens a non-modal dialog — the Path Preview stays orbitable — with **numbered markers** on the mesh's corners (vertices nearest the bounding-box corners; a sheet shows 4). **1.** Pick a corner: click its marker in the preview or a row in the list (hover = cyan + enlarged, selected = green). **2.** **Arm touch-off**, then hold the lead-through button on the arm and touch the tool tip to that corner of the physical object. (The GoFa is hand-guided from the arm itself — there is no software freedrive over RRC, so the button only arms the step and tells you when to guide.) **3.** **Confirm** — the pose updates so the mesh corner sits exactly at the measured TCP point; the sliders jump to the solved values; re-run **Generate Path**. One corner fixes **position only** — rotation keeps the slider values, so orient first. Registration is optional (closing without confirming keeps the pose); the touch-off disarms on confirm or close. A ≥3-corner version that also solves rotation is planned — the solver already supports it.

### Saving toolpaths

**💾 Save Path** (execution bar) writes a **timestamped subfolder** under `paths/` (e.g. `paths/2026-07-13_14-32-08/`) with five files: `path.script` — a **URScript** program (movel travels + movep draws, Speed/Offset/Safety baked in). ⚠ This is a *record*, not an executable: a GoFa cannot run URScript, nothing in this program reads it back, and its header says so. It is kept so the poses stay readable in a second form; `path.json` — the waypoints in **COMPAS's own JSON format**, so a compas_rrc script can read the file with `compas.json_load` and drive the arm straight from it (see *What is inside path.json* below); `preview.png` — the 3D preview, to identify the path at a glance; and the two detection images the path was traced from — `mask.png` (the thick detected region) and `skeleton.png` (its 1-px centrelines), both cropped to the same region the strokes came from and saved lossless, so you can see later exactly what the sand looked like to the detector. `paths/` is gitignored; `path.json`'s `meta` block — and the `.script` header — record mode, surface, speed, offset, safety, radius and stroke count.

**What is inside `path.json`.** The file holds the same waypoints twice, because two different readers want them in different shapes:

- **`frames`** — a flat list of COMPAS `Frame` objects (`"dtype": "compas.geometry/Frame"`, `point` in **millimetres**, plus `xaxis` and `yaxis`; COMPAS derives the z axis as x × y, which is the tool approach direction). Alongside it sits an identity work object as three `compas.geometry/Point`s — `wobj_origin`, `wobj_xaxis`, `wobj_yaxis` — identity because the poses are already in the robot's base frame, the same `wobj0` a live run uses. This is exactly what `compas.json_load` decodes, so a saved bundle can be executed from a plain compas_rrc script with no conversion step; the frames are built the same way the live run builds them, so the file and the app agree waypoint for waypoint.
- **`stroke_starts`** — the index in `frames` where each stroke begins. The frame list is flat, so without this a script cannot tell where one gesture ends and the next starts, and would drag the tool across the surface between them instead of lifting.
- **`strokes`** — the same waypoints grouped per stroke, poses in **metres** with a full plane per waypoint (`origin` + orthonormal `xaxis`/`yaxis`/`zaxis`). This is what the replay tool reads and executes, and what suits frame-guided workflows in Grasshopper.
- **`meta`** and **`units`** — the run parameters (mode, surface, speed, offset, safety, radius, stroke count) and a note on which part is in which unit.

The images come from the last **Generate Path**, so they always match the saved path — regenerating replaces both. Participant Mode saves the same five files, since it goes through the same Save step.

One note on `preview.png`: the 3D preview is drawn by your graphics card inside the browser, so it is the one file the program cannot produce on its own. In Developer Mode the Save click sends the picture up with the request. In Participant Mode nobody clicks, so the Developer window the popup was opened from quietly hands over a screenshot after each drawing is processed, and that is what gets saved. **Leave the Developer window open during an automated session** and every bundle is complete; close it and everything else is still saved exactly as before, just without `preview.png`.

### Projecting the mask onto the sand

A projector pointed at the sandbox lights up the detected grooves in place: **⧉ Project** (Mask viewport) opens `/projection` — drag it onto the projector display and press **F11**. No extra software; a corner-pin homography in the browser does the mapping, and the projector-side stream is only computed while the window is open. **Calibrate once:** rake reference marks into the sand corners, then drag the projected handles **1–4** until the mask lands on the physical marks (arrows nudge 1 px, Shift = 10 px); saved to `settings.json`; **C** re-enters calibration, **B** blanks, **M** mutes. This window is also the one that plays Participant Mode's [sound cues](#sound-cues) — click it once to let the browser start the audio. The projection uses the **full-frame** mask (stable regardless of crop). **Capture auto-blanks** the projector and waits ~1 s for the depth buffer to refill, so projected light never contaminates the capture. Projector: keystone OFF, no digital zoom, fixed mount — recalibrate after any bump; a dimmer room gives crisper grooves.

### Multi-Cam Vision — where the camera view is shaped

A separate app that lays the feeds of **however many** D435i cameras are plugged in (one, two, three, up to four) onto one combined depth image covering a larger sand area. It does one job: **putting the pictures together**. There is no groove detection here — that stays in the main app, which is where its parameters are tuned.

> **The layout you save here is what the main app sees.** Every view in Developer Mode (depth and colour) and in the Participant window is this combined picture — cropping, the depth numbers, the trigger and the drawing all work on it exactly as before. With a single camera nothing changes in practice; the combined view of one camera is that camera.
>
> The app reads `stitch_calibration.json` **once, at start-up**, so after changing the layout: **press Save layout**, close this tool, start the main app. An adjustment you did not save changes nothing outside this window — the app, and this tool the next time you open it, both go back to the last saved layout. (You could not have them both open anyway — a camera can only belong to one program.) **The projector needs re-aiming after a layout change**, since the mask it throws is now shaped by the new canvas: open `/projection?cal` and drag the four corners again.

- Launch with `run_stitch.bat` → [http://localhost:5106](http://localhost:5106). **Close the main app first** — each RealSense can only be owned by one process. The tool **finds the cameras itself**; with none connected it runs on a **synthetic** three-camera scene (banner shows why) so the workflow can still be tried.
- **One screen, split in two.** The **top is the result** — the combined picture, always live, look-only. The **bottom is the workbench**, one panel per camera, where every adjustment is made. Drag the **bar between them** to give whichever half you need more room; the size is remembered. On the right are the remaining controls — all of them buttons.
- **Where it starts.** Every camera opens as a tile of **the same size**, laid **flush side by side** in a row, tops level. That is a tidy starting strip, not a guess at your rig — the next two steps are how you make it match reality.
- **Getting each camera the right way round and in the right place.** Click a camera panel (or its name in the list) to select it, then:
  - **⟲ / ⟳** turn the camera a quarter turn — for a camera mounted on its side or upside-down. The picture turns without stretching.
  - **◀ / ▶** move it one place left or right, swapping with the camera that is there. Use this so the on-screen order matches the physical order; which camera happens to be "Cam 1" depends on USB, not on the rig. A camera keeps its own turn, trim and corner shape when it changes places.
- **Fine placement.** A green outline sits on the selected panel with four numbered handles, **exactly like the projector calibration window**:
  - **Drag handle 1–4** to shape where that camera lands. This is the fix for a camera mounted at an **angle**: a tilted camera sees the sand as a keystone, and pulling the corners back into shape squares it up. Only the handle you grab moves — the other three stay put. The green outline shows the shape you are making; the top view shows the effect.
  - **Drag inside the green outline** to slide that camera across the combined picture until it lines up with its neighbour.
  - **Arrow keys** nudge the last handle you touched (**Shift** = faster).
  - **▲ / ▼** raise or lower just that camera, for when it hangs higher or lower than its neighbour and its sand sits at the wrong level.
  - **Reset corners** puts one camera back on its plain rectangle in the row; **Reset camera** clears its turn and crop too.
  - The **eye** next to a camera drops it out of the result without losing its placement.
- **Trimming a camera.** Drag a **blue edge bar** on that camera's panel to cut away table edges, walls, or anything that is not sand — one edge at a time. Trimming never moves the sand you kept: the placement is re-cut to match. Only the trimmed region reaches the combined view, and the four handles sit on it, so what is inside them is what you get.
- **Showing depth / colour** flips both halves between the depth image and the colour cameras. The colour view has dark strips — expected: the colour lens has a narrower field of view than the depth sensor, and depth is the product here.
- **Save layout** → `stitch_calibration.json` (gitignored), reloaded automatically next start — by this tool **and by the main app** — and matched back to each camera by **serial number**, so swapping USB ports does not shuffle the rig.
- **Saved, or only adjusted?** The line under the Save button always says which. **Saved — this is what the app uses** means the picture on screen is the picture the robot will draw from. As soon as you turn, move, trim or drag anything it becomes **Adjusted — not saved**, and the Save button turns amber: your changes are live in this window only, and the app is still on the old layout. Nothing else ever writes the file — not a drag, not closing the tool — so **reopening always comes back to your last save**, pixel for pixel.
- **Discard changes** throws away everything since that save and reloads the file, without restarting the tool. It asks twice (the button changes to *Click again to discard*) so a stray click cannot cost you an evening's alignment.
- **How it merges:** each camera's cropped, turned picture is warped onto one shared top-down canvas through the corner pin you set, at a uniform mm-per-pixel. Where two cameras overlap, the measurements are **averaged** — the seam region ends up *less* noisy than either camera alone, and the overlap is outlined so you can see it.
- Like the main app, closing the last browser tab stops the program.



### Scheduler (standalone)

A **contained, read-only** tool that turns the `paths/` folder into a spreadsheet: every toolpath that has been saved, numbered and dated. It is the record of what the machine has drawn.

- Launch with `run_scheduler.bat` → [http://localhost:5108](http://localhost:5108). It opens no camera and no robot connection and writes nothing, so — unlike the Multi-Cam and replay tools — **you can leave it running next to the main app**.
- **Four columns**: **#** (1, 2, 3 … oldest first, so row 1 is the earliest path saved), **Path executed** (the bundle folder, with the files it holds listed underneath), **Date and time**, and **Mask** — a thumbnail of the groove image that path was traced from. Click a thumbnail to open it full size. Bundles saved before mask images existed show a dash instead.
- **It updates itself.** The folder is re-scanned every couple of seconds and the table refreshes on its own, so a path saved in Developer Mode — or by Participant Mode while nobody is watching — appears without touching the browser. **Refresh** forces a scan now.
- **Download CSV** gives you the same columns as a real spreadsheet file. A CSV cannot hold a picture, so its Mask column carries the image's location on disk instead.
- **About the date.** Nothing in the pipeline currently records the moment a path was *run*, so this is the moment the bundle was **written**. In Participant Mode the save happens immediately before the robot moves, so it is the run time to within a second; in Developer Mode it is when **Save Path** was pressed, which may be before, after, or instead of a run. The time normally comes from the folder's own name; if a folder has been renamed by hand the tool falls back to `path.json` and then to the file date, and marks the value so you know it was inferred.
- The file list under each path makes gaps obvious: older bundles show `mask.png —` and `skeleton.png —` because they were saved before those images existed (so their Mask cell is a dash too), and a bundle shows `preview.png —` when no Developer window was open to supply that picture.
- Like the other tools, closing the last browser tab stops the program.

### Toolpath replay tool (standalone)

A **contained** tool — not part of Developer or Participant Mode — that re-runs a previously saved toolpath without the camera or the full app.

- Launch with `run_replay.bat` → [http://localhost:5107](http://localhost:5107). **Close the main app first if it is connected to the robot** — one controller per robot. No camera is needed.
- The left panel lists every bundle in `paths/` (newest first). Click one to load it: the saved **preview.png** is shown, along with strokes/waypoints and the metadata it was saved with. `path.json` is the executable file; clicking the row (or its `json` badge) loads it. The sibling `.script` is a UR-format record and is never loaded.
- Enter the ROS bridge host (prefilled from the last one used in the main app — normally `127.0.0.1`, see *Connecting to the GoFa*) and **Connect**, set **Speed / Safety / Radius** (prefilled from the file's own saved values), then **Run**. The saved waypoints are executed *literally* — offset and contact depth were already baked in at save time — through the same executor as the main app, so a replay traces exactly what a live run would. **Cancel** stops mid-path; a progress bar tracks the run.
- **Future robots:** everything brand-specific sits behind one small interface (`replay_robot.ReplayBackend`). Porting to e.g. an ABB GoFa means writing one backend class (compas_rrc: one MoveL per waypoint; `path.json` even carries a ready-made plane per waypoint for ABB's quaternion frames) and switching `REPLAY_BACKEND` in `config.py` — the loader, server and UI stay unchanged.
- Like the main app, closing the last browser tab stops the program.



### Configuration reference

All parameters live in `config.py`.

#### Server


| Variable                | Default       | Description                             |
| ----------------------- | ------------- | --------------------------------------- |
| `HTTP_HOST`             | `"localhost"` | Bind address                            |
| `HTTP_PORT`             | `5105`        | Web UI port (main app)                  |
| `STITCH_HTTP_PORT`      | `5106`        | Multi-Cam Vision                        |
| `REPLAY_HTTP_PORT`      | `5107`        | Toolpath replay                         |
| `SCHEDULER_HTTP_PORT`   | `5108`        | Scheduler                               |
| `SCHEDULER_REFRESH_S`   | `2.0`         | How often the Scheduler re-scans `paths/` |




#### Depth camera (RealSense)


| Variable                                   | Default | Description                           |
| ------------------------------------------ | ------- | ------------------------------------- |
| `DEPTH_WIDTH`                              | `640`   | Depth stream width of ONE camera (px)   |
| `DEPTH_HEIGHT`                             | `480`   | Depth stream height of ONE camera (px)  |
| `DEPTH_FPS`                                | `30`    | Depth stream frame rate                 |
| `DEPTH_AVERAGE_FRAMES`                     | `30`    | Frames temporally averaged per camera on Capture |
| `DEPTH_COLOR_NEAR_M` / `DEPTH_COLOR_FAR_M` | `0.0`   | Colormap range in metres (0 = auto)     |
| `STITCH_MAIN_EVERY_S`                      | `0.2`   | How often the combined live view is rebuilt (s) |
| `STITCH_MAIN_BIND_TIMEOUT_S`               | `4.0`   | Wait for every camera's first frame before fixing the view's size |

The combined view is bigger than one camera: with the cameras' own layout it keeps their native detail, so its size follows how much sand the rig covers, not these two numbers. `GET /status` reports it as `frame_size` alongside `camera_count`.




#### Groove detection


| Variable                  | Default    | Description                                  |
| ------------------------- | ---------- | -------------------------------------------- |
| `GROOVE_DETECT`           | `"valley"` | `valley` / `ridge` / `band`                  |
| `GROOVE_DEPTH_MM`         | `1.5`      | mm deeper than surface to count as a groove  |
| `GROOVE_DETREND_SIGMA_PX` | `25.0`     | blur radius estimating the bare surface      |
| `GROOVE_SMOOTH_SIGMA_PX`  | `1.5`      | depth denoise before detection               |
| `GROOVE_MIN_BLOB_PX`      | `40`       | discard detected specks smaller than this    |
| `CONTOUR_MIN_PIXELS`      | `20`       | discard chains shorter than this many pixels |




#### Path extraction


| Variable                             | Default         | Description                                                      |
| ------------------------------------ | --------------- | ---------------------------------------------------------------- |
| `RESAMPLE_SPACING_MM`                | `10.0` mm       | Default waypoint spacing (Spacing slider overrides per generate) |
| `RESAMPLE_SPACING_MIN_MM` / `MAX_MM` | `10` / `100` mm | Spacing slider range                                             |
| `JOIN_DISTANCE_MM`                   | `0.0` mm        | Default endpoint-join distance; `0` = joining off (Distance Threshold box overrides per generate) |
| `JOIN_DISTANCE_MIN_MM` / `MAX_MM`    | `0` / `200` mm  | Distance Threshold box range                                     |
| `JOIN_CROSSING_FACTOR`               | `2.0`           | Threshold multiplier when another stroke crosses the connecting line |




#### Profanity guard (Participant Mode only)


| Variable                      | Default      | Description                                                        |
| ----------------------------- | ------------ | ------------------------------------------------------------------ |
| `PROFANITY_CHECK_ENABLED`     | `True`       | Master switch; `False` skips the check entirely                    |
| `PROFANITY_LANGS`             | `"eng+deu"`  | Tesseract language packs (both ship inside the conda env)          |
| `PROFANITY_WORDLIST_DIR`      | `wordlists`  | Folder scanned for `*.txt` wordlists                               |
| `PROFANITY_MIN_SUBSTRING_LEN` | `4`          | Entries this long also match inside run-together words; shorter ones must stand alone |
| `PROFANITY_OCR_ROTATIONS`     | `(0, 180)`   | Angles the mask is re-read at (180° = written from the far side)   |




#### Sound cues (Participant Mode, played by the projection window)


| Variable            | Default                | Description                                                     |
| ------------------- | ---------------------- | --------------------------------------------------------------- |
| `SOUNDS_DIR`        | `sounds`               | Folder the cue `.wav` files are read from (and rendered into)   |
| `SOUNDS_URL_PATH`   | `/sounds`              | Static route the projection window fetches them over            |
| `SOUND_SAMPLE_RATE` | `44100`                | Sample rate `sound_design.py` renders at                        |
| `SOUND_CUES`        | 4 statuses (see below) | Participant status → cue file stem; a status left out is silent |

```python
SOUND_CUES = {
    "Alerted":   "engaged",        # hand enters
    "Sensing":   "acknowledged",   # hand leaves
    "Actuating": "anticipation",   # saving + running
    "Invalid":   "alarm",          # refused
}
```




#### Console output


| Variable             | Default | Description                                                          |
| -------------------- | ------- | -------------------------------------------------------------------- |
| `SHOW_MODULE_BANNER` | `True`  | Print the feature→modules table at startup                           |
| `SHOW_MODULE_TRACE`  | `True`  | Print the `└ a.py → b.py` module trail under each task line          |




#### Target surface


| Variable             | Default     | Description                                        |
| -------------------- | ----------- | -------------------------------------------------- |
| `SURFACE_DIR`        | `surfaces/` | Uploaded STL/OBJ meshes are stored here            |
| `SURFACE_UNITS_TO_M` | `0.001`     | File-unit scale (Rhino mm → m; set 1.0 for metres) |
| `SURFACE_MAX_FACES`  | `80000`     | Warn above this — browser preview gets heavy       |




#### Robot motion


| Variable           | Default     | Units | Description                                                            |
| ------------------ | ----------- | ----- | ---------------------------------------------------------------------- |
| `DRAW_Z`           | `-0.010`    | m     | Planar-mode pen contact offset (not used in surface mode)              |
| `TRAVEL_Z`         | `0.050`     | m     | Default safety retract (UI Safety box overrides per run)               |
| `DRAW_SPEED`       | `0.05`      | m/s   | Default speed = 5% (UI Speed slider overrides per run)                 |
| `MAX_TCP_SPEED`    | `2.0`       | m/s   | 100% on the Speed slider — the GoFa 10's rated tool speed. Every Speed percentage is relative to this, so a bundle saved before the ABB port replays twice as fast at the same % |
| `DRAW_ACCEL`       | `0.3`       | m/s²  | Drawing acceleration — used by the simulation/preview side only; RAPID takes a percentage, not m/s² (see `RRC_ACCEL_PCT`) |
| `TRAVEL_ACCEL`     | `0.5`       | m/s²  | Travel/retract acceleration (same note)                                |
| `TOOL_ORIENTATION` | `[0, π, 0]` | rad   | Planar-mode TCP orientation (surface mode derives it per waypoint)     |
| `GOFA_REACH_M`     | `1.62`      | m     | Reach-check envelope radius around the base (GoFa 10, flange variant — other GoFa 10 variants are 1.52 m) |
| `GOFA_MIN_REACH_M` | `0.18`      | m     | Reach-check inner cylinder around the base axis (carried over, not an ABB figure) |
| `MAX_PATH_LENGTH_MM` | `0.0`     | mm    | Default Max Total Length; 0 = off (UI box overrides per run)            |
| `BLEND_ZONE_M`     | `0.0005`    | m     | Default corner zone radius (UI Radius slider overrides per run) |
| `BLEND_ZONE_MAX_MM` | `50.0`     | mm    | Upper bound of the Radius slider (still clamped per stroke) |
| `RRC_ROS_HOST`     | `127.0.0.1` | —     | Default ROS bridge host offered in the connect box                     |
| `RRC_ROS_PORT`     | `9090`      | —     | rosbridge websocket port                                               |
| `RRC_NAMESPACE`    | `/rob1`     | —     | RRC task namespace on the controller (must match `namespace:=` in `docker/docker-compose.yml`) |
| `RRC_ACCEL_PCT`    | `100.0`     | %     | Acceleration, as a percentage of what the arm can do — RAPID's `AccSet`, sent once at connect. One controller-wide setting shared by drawing and travel, so lowering it (20–40 is visibly smoother) also slows the hops between strokes. 100 = the controller's own default |
| `RRC_ACCEL_RAMP_PCT` | `100.0`   | %     | How sharply the acceleration itself builds                             |


#### Material dispenser

The valve or pump that lays the seeded substrate down while a stroke is drawn. The controller switches it through a **named digital output**, so nothing here works until that signal exists in the robot's I/O configuration. Until then leave `DISPENSER_ENABLED` off and not one instruction is ever sent.

It is open **only while drawing** — it opens once the tool has landed on a stroke's first point and closes before anything retracts, so nothing comes out during a travel move, and a cancel, an error or a disconnect all close it.

| Variable                | Default         | Units | Description                                          |
| ----------------------- | --------------- | ----- | ---------------------------------------------------- |
| `DISPENSER_ENABLED`     | `False`         | —     | Turn on once the valve is wired and named below       |
| `DISPENSER_SIGNAL`      | `"doDispenser"` | —     | The RAPID digital output's name on the controller     |
| `DISPENSER_ON_DELAY_S`  | `0.0`           | s     | Priming pause after opening, before the arm moves     |
| `DISPENSER_OFF_DELAY_S` | `0.0`           | s     | Pause after closing, before the retract               |




## References

- ABB robot control: [COMPAS RRC documentation](https://compas-rrc.github.io/compas_rrc/)
- The ROS bridge images: [compas_rrc docker setup](https://github.com/compas-rrc/compas_rrc)
- Intel RealSense SDK (`pyrealsense2`): [librealsense](https://github.com/IntelRealSense/librealsense)

