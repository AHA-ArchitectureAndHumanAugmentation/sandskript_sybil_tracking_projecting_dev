# depth-cam-to-robot

---

## Overview

`depth-cam-to-robot` is the software of the **Sandskript** project, developed for **Sybil**, an interactive installation at **Ars Electronica 2026**. It watches a sandbox with a depth camera, detects the grooves a visitor rakes into the sand, converts them into strokes on a 3D target surface, and has a robot arm retrace the strokes, depositing living, seeded biomaterial on a tensile canvas.

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
| `ur-rtde >= 1.6`           | UR robot RTDE control (moveL, movep paths, TCP pose)         |
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

The browser opens at `http://localhost:5005` in **Developer Mode** (the full manual UI); **Participant Mode** is its **⧉ popup** on the Depth viewport. Closing the last browser window stops the server.

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
| Robot           | Universal Robots UR3 / UR5 / UR10 / UR16 (any with RTDE support) |
| Robot mode      | **Remote Control** (Settings → System → Remote Control)          |
| Camera          | Intel RealSense D435i (any RealSense depth camera should work)   |
| Camera position | Top-down view covering the full sandbox                          |


---



## The flow

The pipeline turns a raw depth frame into robot motion in eight steps:

```
depth-camera frame
    │   
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
reach check          flag any waypoint outside the arm's reach (shown red)
    │
    ▼
path execution       the robot retraces the strokes
                     (moveL to travel between strokes, movep to draw each one)
```

**TSP** = the *Travelling-Salesman Problem*: visit every stroke once by the shortest total route. Here it's solved with a fast nearest-neighbour heuristic, so the robot wastes as little time as possible lifting and moving between strokes.

The eight steps group into four phases:


| Phase                   | Steps                                                                  | What the phase does                                         |
| ----------------------- | ---------------------------------------------------------------------- | ----------------------------------------------------------- |
| **Sensing**             | depth-camera frame                                                     | Capture a clean, noise-averaged depth still of the sand     |
| **Interpretation**      | groove regions → centrelines                                           | Turn raw depth into 1-px groove centrelines                 |
| **Robotic preparation** | resampled strokes → ordered strokes → surface projection → reach check | Turn the centrelines into an ordered, reachable 3D toolpath |
| **Actuation**           | path execution                                                         | The robot draws the strokes on the surface                  |




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
│     └─ 🟠 Projection  —  projector shines the detected mask back onto the sand
│           modules
│           ┌──────────────────┐ ┌───────────┐
│           │ camera_thread.py │ │ server.py │  (mask composition)
│           └──────────────────┘ └───────────┘
│           UI
│           ┌────────────────────────┐
│           │ viewer/projection.html │  (corner-pin calibration)
│           └────────────────────────┘
│
├─ DUAL-CAM VISION  ·  contained prototype (in development)
│  └─ 🔵 Stitching  —  merge two RealSense feeds into one heightmap, detect grooves (same engine)
│        modules
│        ┌─────────────┐ ┌────────────────┐ ┌──────────────────┐ ┌────────────────────┐
│        │ stitcher.py │ │ dual_camera.py │ │ stitch_server.py │ │ depth_extractor.py │
│        └─────────────┘ └────────────────┘ └──────────────────┘ └────────────────────┘
│        UI
│        ┌────────────────────┐ ┌──────────────────┐
│        │ viewer/stitch.html │ │ viewer/stitch.js │
│        └────────────────────┘ └──────────────────┘
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
├── module_trace.py          🟢🟣🟠 Console: startup feature→module table + per-task module trail
├── config.py                🟢🟣🟠🔵⚪🤖 All configurable parameters
├── server.py                🟢🟣🟠🤖 aiohttp server: MJPEG feeds, WebSocket, surface upload
├── camera_thread.py         🟢🟣🟠 DepthCameraThread: RealSense → depth/RGB/skeleton/mask streams
├── depth_extractor.py       🟢🟣🔵 Depth → groove engine: colorize, detect, filter, skeletonize
├── path_extractor.py        🟢🟣 Grooves → pixel chains → smooth → resample → TSP
├── surface.py               🟢🟣 Target mesh: STL/OBJ load, projection, normal TCP orientations
├── registration.py          🟢 Corner→TCP touch-off placement (1-point + Kabsch ≥3-point)
├── path_export.py           🟢🟣⚪ Save toolpath → URScript + JSON (poses+frames) + preview PNG
├── path_executor.py         🟢🟣⚪ Background thread: retract/travel/movep per stroke, progress
├── robot_controller.py      🟢🟣⚪ Thread-safe ur-rtde wrapper (moveL, movep paths, EE pose)
├── workspace.py             🟢 Planar fallback mapping (Test Mode)
├── reach.py                 🟢🟣 Reach-envelope estimate (importable without hardware)
├── stitcher.py              🔵 Dual-Cam Vision: heightmap stitching + auto-align math
├── dual_camera.py           🔵 Dual-Cam Vision: owns two RealSense pipelines
├── stitch_server.py         🔵 Dual-Cam Vision: aiohttp server (port 5006)
├── stitch_main.py           🔵 Dual-Cam Vision entry point (run_stitch.bat)
├── toolpath_loader.py       ⚪ Replay tool: read saved bundles (path.json OR path.script)
├── replay_robot.py          ⚪ Replay tool: robot-brand abstraction (UR now, ABB-ready)
├── replay_server.py         ⚪ Replay tool: aiohttp server (port 5007)
├── replay_main.py           ⚪ Replay tool entry point (run_replay.bat)
├── settings.py              🟢🟣🟠⚪ Persistent JSON settings (last robot IP + projector corners)
├── mcp_server/              🤖 FastMCP tools wrapping the app's HTTP/WS API
├── .mcp.json                🤖 Registers the MCP pipeline server (project scope)
├── CLAUDE.md                🤖 AI-assistant repo guide (pipeline, API, gotchas)
├── environment.yml          🟢🟣🟠🔵⚪🤖 Conda-env recipe ("sandskript": Python 3.11 + all deps)
├── requirements.txt         🟢🟣🟠🔵⚪🤖 pip dependencies (installed by environment.yml)
├── requirements-dev.txt     🟢🟣🟠🔵⚪🤖 dev extras: pytest, mcp
├── run.bat                  🟢🟣🟠 Main-app launcher (double-click)
├── run_stitch.bat           🔵 Dual-Cam launcher
├── run_replay.bat           ⚪ Replay launcher
├── conftest.py              🟢🟣🟠🔵⚪🤖 Pytest shared fixtures
├── pytest.ini               🟢🟣🟠🔵⚪🤖 Test configuration
├── settings.json            🟢🟣🟠⚪ Auto-generated: saved app settings (gitignored)
├── surfaces/                🟢🟣 Uploaded target meshes (gitignored)
├── paths/                   🟢🟣⚪ Saved toolpaths: dated folders of .script/.json/.png (gitignored)
├── presets/                 🟢 Saved Detection-Parameter files, named by date (gitignored)
├── wordlists/               🟣 Profanity wordlists (en/de seed; add LDNOOBW .txt files here)
├── tests/                   🟢🟣🟠🔵⚪🤖 Unit + hardware-gated integration tests
└── viewer/
    ├── index.html           🟢 Single-page app
    ├── viewer.js            🟢 WebSocket client, UI handlers, Three.js 3D path preview
    ├── projection.html      🟠 Projector output / corner-pin calibration window
    ├── depth_view.html      🟣 Participant Mode popup (depth numbers + Auto + trigger)
    ├── depth_overlay.js     🟣 Popup logic: number overlay, Auto toggle, status chip
    ├── stitch.html          🔵 Dual-Cam Vision prototype UI
    ├── stitch.js            🔵 Dual-Cam Vision logic (setup/stitch modes, calibration)
    ├── replay.html          ⚪ Toolpath replay tool UI
    ├── replay.js            ⚪ Replay UI logic (connect, pick bundle, run)
    ├── style.css            🟢 Responsive layout
    └── lib/
        ├── three.min.js     🟢 Three.js (3D rendering)
        └── OrbitControls.js 🟢 Mouse/touch orbit controls
```

Feature tags:

🟢 Developer Mode · 🟣 Participant Mode · 🟠 Projection · 🔵 Dual-Cam · ⚪ Replay · 🤖 MCP

---



## Guide pour les nuls

The Developer-Mode workflow, step by step.

1. **Connect** — enter the robot's IP (e.g. `192.168.1.100`) and click **Connect**.
2. **Load the drawing target** — mesh your Rhino surface, export it as **STL/OBJ in millimetres**, and load it at the prompt. There is no manual robot-calibration step: the surface's position relative to the robot is set with the Surface X/Y/Z + rotation sliders (or corner touch-off) and verified visually in the Path Preview.
3. **Aim the RealSense** straight down so it covers the whole sandbox. The four viewports show **Depth** (near = blue → far = red), **RGB**, **Skeleton** (the 1-px centrelines that become the path) and **Mask** (the thick detected region — shows groove *width*, handy while tuning). The **⧉ Participant Mode** popup (Depth viewport) adds the live depth view with **absolute mm-from-camera** labels per iso-depth region (**Region interval** and **Text size** sliders; display-only, computed only while the popup is open) and holds the **Auto** toggle + **Trigger below** box that automate the pipeline — see *Participant Mode* below.
4. **Tune detection live** — the **Detection Parameters** panel works *before* capturing: pick a **Mode** (Valley / Ridge / Band) and adjust **Groove depth**, **Surface scale**, **Denoise**, **Min blob**; the viewports update in real time. Drag a **crop** rectangle on the Depth view to limit the region. **Save** stores the sliders to a dated file under `presets/`, **Load** restores one, **Reset** returns to defaults.
5. **Capture Image** — freezes a temporally averaged depth (+ aligned colour) still; the crop carries over (drag inside to move, corners to resize, **Reset Crop** for full frame). Detection — and the generated path — cover only the crop.
6. **Generate Path** — the 3D viewer shows the surface, the detected skeleton as a **white** on-surface line, and the movep toolpath: **green** blended segments with waypoint dots (**red** = outside estimated reach), **amber** safety/retract points, **grey** pen-up travels. **Spacing** (10–100 mm) sets waypoint distance and regenerates on release; **Distance Threshold** (0–200 mm, default 0 = off) merges strokes whose ends nearly touch — see below — and also regenerates; **Radius** (0–5 mm, default 0.5) is the movep corner blend — clamped per stroke to 45 % of the shortest segment so the controller never rejects the path; Offset/Safety edits update the preview live. **Path | Order** switches to a numbered stroke-order view; **⧉ Pop out** opens the preview in its own window. Re-tune and regenerate freely, or **Retake**.
7. **Run** — set **Speed** (% of max TCP speed, governs the *entire* motion), **Offset** (mm off the surface along the local normal), **Safety** (retract mm) and **Radius**, then Run. The blue dot tracks the live TCP; a progress bar tracks execution; **Cancel** stops mid-stroke. Execution uses the same blended movep as the saved `path.script`, so live and saved runs trace identically. **💾 Save Path** writes the toolpath — current settings baked in — to a timestamped folder under `paths/` (see *Saving toolpaths*).

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

Detection is *relative* (mm below the local surface), so a hand raking or a person leaning over creates phantom relief. The **Ignore closer than (mm)** box (Mask viewport, always visible) is an *absolute* cutoff from the camera: any groove blob touching a region nearer than this (grown by a safety margin) is dropped from the mask — live views, projection and path generation alike. Set it a little above the sand's distance (read it off the Participant popup's labels); 0 or empty disables.

### Participant Mode (automated pipeline)

The **⧉ Participant Mode popup** replaces the buttons with a **depth trigger**: a participant rakes, pulls their hand out, and the robot retraces — no clicks. Enter a **Trigger below** distance (mm from camera, same unit as the depth labels — sand at 900 mm → e.g. 700), then switch **Auto ON**. The popup shows only the Developer-Mode crop — the labels and the trigger watch that region too; the crop itself can only be changed in Developer Mode. Statuses appear large in the popup's top-right:


| Status               | Meaning                                                           |
| -------------------- | ----------------------------------------------------------------- |
| **Auto Off**         | Toggle off — the popup is just the depth-number viewport.         |
| **Auto On**          | Armed; nothing in frame is closer than the trigger.               |
| **Alerted**          | Something closer than the trigger is in frame (a hand raking).    |
| **Sensing**          | Frame stayed clear for ~1 s → capturing the averaged depth still. |
| **Generating Paths** | Extracting strokes and building the toolpath.                     |
| **Actuating**        | Saving the bundle to `paths/` and running it on the robot.        |
| **Invalid**          | The profanity guard rejected the drawing — nothing saved, nothing run. |


After Actuating it returns to **Auto On**, ready for the next participant. While Auto is **ON**, the manual Capture / Retake / Generate / Run buttons grey out (the server also refuses them) — **Cancel stays active** as the emergency stop. Worth knowing: the automated run reuses the **same pipeline and current settings** as the Developer-Mode buttons (set everything up, then flip Auto ON; the Developer window shows each step live); an empty trigger box can never fire; without a robot the toolpath is still generated and **saved**, only the run is skipped; Auto stays ON server-side even if the popup closes; Sensing deliberately waits ~1 s so the averaged still doesn't contain the hand.

#### Profanity guard

Because Participant Mode runs unattended, it reads the sand before the robot does. Between *Generating Paths* and *Actuating*, the detected **mask** is passed through OCR (English + German); if what was raked spells something on a wordlist, the run stops: the chip turns red and reads **Invalid**, and **nothing is saved and nothing is sent to the robot**.

**Invalid is sticky** — it stays on screen so whoever drew it sees the verdict, while the trigger stays armed. The next participant clears it; so does toggling Auto off and on.

- **Where it looks** — the mask, not the skeleton. Thick strokes are what OCR can read; 1-px centrelines are not.
- **What it reads** — the mask upright *and* upside down (participants write from the far side of the sandbox), in both black-on-white and white-on-black. Four quick passes, once per capture — no effect on the live feeds.
- **What counts** — case, umlauts/ß, accents and leetspeak (`5H1T`) are all folded before matching, and OCR's broken spacing is handled by also testing the words run together. Entries under 4 letters only match as standalone words, so `assist` and `classic` are safe.
- **Wordlists** — every `.txt` in `wordlists/`. A short English and German seed list ships with the repo; for real coverage drop in the [LDNOOBW](https://github.com/LDNOOBW/List-of-Dirty-Naughty-Obscene-and-Otherwise-Bad-Words) `en` and `de` files (rename to anything ending `.txt`). All files are merged — no code change needed.

**It fails open, on purpose.** If Tesseract is missing, the wordlist is empty, or OCR errors out, the drawing goes through and a line is printed to the console. An installation should not stop dead because an OCR dependency is absent. ⚠️ The flip side: this is a coarse filter, not a guarantee — handwriting in sand is genuinely hard to read, so expect misses. It does not detect offensive **symbols** at all, only text. Keep a human able to hit **Cancel**.

Turn it off with `PROFANITY_CHECK_ENABLED = False` in `config.py`. It never runs in Developer Mode, where an operator is present to judge for themselves.

### Test mode (no robot)

**Test Mode (no robot)** sets a synthetic workspace so the depth → groove → path-preview pipeline can be exercised without a robot. Run stays gated on a real connection.

### How the drawing maps onto the surface

**1.** In Rhino, `Mesh` the surface and **export as STL/OBJ in millimetres**. It may be flat, tilted or vertical — projection follows the mesh's dominant (area-weighted) face normal, and the drawing lands on the side the normals point (flip with `Dir` if paths appear on the back). Exception: a **steep surface** (more than ~45° from horizontal) always draws on the **side facing the robot base**, so a positive TCP offset always moves the tool *toward* the robot and never behind a wall. **2.** The full camera frame (4:3) is **fitted centred** onto the surface's footprint, aspect preserved — each stroke lands at the same relative position it has in the camera view; cropping only selects which grooves exist, it doesn't move or zoom the drawing. **3.** Every waypoint gets a tool orientation **perpendicular to the surface** with minimal wrist twist; rays that miss the mesh split the stroke. **4.** Placement is live: the **Surface X/Y/Z + Rot X/Y/Z** sliders position the mesh in the robot base frame; the **TCP offset (mm)** slider bakes a hover distance at Generate time, the execution bar's **Offset** adds more at Run time. **5.** Contact depth comes from the offsets (planar `DRAW_Z` is not applied in surface mode); retracts follow the tool axis, pulling *away* from tilted surfaces. **Clear Surface** returns to the flat Test-Mode mapping. If the robot draws on a *real* surface, the virtual placement must match reality — verify with the preview and a slow, offset-first run.

### Register Corner → TCP (touch-off placement)

Measure the placement with the robot instead of guessing sliders. **Register Corner → TCP…** (Target surface section) opens a non-modal dialog — the Path Preview stays orbitable — with **numbered markers** on the mesh's corners (vertices nearest the bounding-box corners; a sheet shows 4). **1.** Pick a corner: click its marker in the preview or a row in the list (hover = cyan + enlarged, selected = green). **2.** **Start Freedrive** and touch the tool tip to that corner of the physical object. **3.** **Confirm** — the pose updates so the mesh corner sits exactly at the measured TCP point; the sliders jump to the solved values; re-run **Generate Path**. One corner fixes **position only** — rotation keeps the slider values, so orient first. Registration is optional (closing without confirming keeps the pose); freedrive ends on confirm or close. A ≥3-corner version that also solves rotation is planned — the solver already supports it.

### Saving toolpaths

**💾 Save Path** (execution bar) writes a **timestamped subfolder** under `paths/` (e.g. `paths/2026-07-13_14-32-08/`) with three files: `path.script` — a **URScript** program (movel travels + movep draws, Speed/Offset/Safety baked in), directly runnable on the pendant — verify TCP/payload and run slow first; `path.json` — the strokes as 6-DOF poses **plus a full plane/frame per waypoint** (`origin` + orthonormal `xaxis`/`yaxis`/`zaxis`, z = tool approach), for frame-guided workflows (Grasshopper, custom motion); `preview.png` — the 3D preview, to identify the path at a glance. `paths/` is gitignored; each `.script` header records mode, surface, speed, offset, safety and stroke count.

### Projecting the mask onto the sand

A projector pointed at the sandbox lights up the detected grooves in place: **⧉ Project** (Mask viewport) opens `/projection` — drag it onto the projector display and press **F11**. No extra software; a corner-pin homography in the browser does the mapping, and the projector-side stream is only computed while the window is open. **Calibrate once:** rake reference marks into the sand corners, then drag the projected handles **1–4** until the mask lands on the physical marks (arrows nudge 1 px, Shift = 10 px); saved to `settings.json`; **C** re-enters calibration, **B** blanks. The projection uses the **full-frame** mask (stable regardless of crop). **Capture auto-blanks** the projector and waits ~1 s for the depth buffer to refill, so projected light never contaminates the capture. Projector: keystone OFF, no digital zoom, fixed mount — recalibrate after any bump; a dimmer room gives crisper grooves.

### Dual-Cam Vision prototype (standalone)

A **contained** prototype — not part of Developer or Participant Mode — that merges the feeds of **two** D435i cameras into one combined depth image covering a larger sand area, aiming for a **5–10 % frame overlap**.

- Launch with `run_stitch.bat` → [http://localhost:5006](http://localhost:5006). **Close the main app first** — each RealSense can only be owned by one process. With fewer than two cameras connected, the tool runs on a **synthetic** sand scene (banner shows why) so the UI and calibration workflow can still be tried.
- The big **Stitch** button switches between two screens:
  - **Stitch OFF — setup.** Each camera's live depth and RGB shown side by side. Use **⇄ Swap left/right** if the cameras come up on the wrong sides, and **⟲ Rotate 180°** under either side if a camera is mounted upside-down, until the feeds match reality. (First start opens here; once a calibration has been saved, later starts open stitched.)
  - **Stitch ON — combined.** Four live views: **combined depth** (the overlap band is outlined), **combined RGB**, and the detected **mask** and **skeleton**. The RGB view has a dark strip in the middle — expected: the colour lens has a narrower field of view than the depth sensor, and depth is the product here.
- **How it merges:** each camera's depth image is converted to 3D points using its own factory intrinsics, camera 2's points are moved into camera 1's frame by a fixed rig transform, and both are projected onto one top-down heightmap (uniform mm-per-pixel, no perspective seam). Where the frames overlap, the two measurements are **averaged** — the seam region ends up *less* noisy than either camera alone.
- **Aligning the rig (once per mounting):**
  1. Rake a groove **across the seam region** (flat sand has nothing to
    align on), then turn the stitch on — it automatically searches all  plausible camera spacings for the overlap (**Find overlap** retries  this any time).
  2. If needed, trim **tz / yaw** by hand and press **Fine-tune** to
    re-measure the residual XY offset from the overlap band.
  3. **Save calibration** → `stitch_calibration.json` (gitignored), reloaded
    automatically next start.
- **Detection parameters** (same engine and meaning as Developer Mode) tune the mask/skeleton computed on the stitched heightmap, live at ~4 Hz.
- Like the main app, closing the last browser tab stops the program.



### Toolpath replay tool (standalone)

A **contained** tool — not part of Developer or Participant Mode — that re-runs a previously saved toolpath without the camera or the full app.

- Launch with `run_replay.bat` → [http://localhost:5007](http://localhost:5007). **Close the main app first if it is connected to the robot** — one controller per robot. No camera is needed.
- The left panel lists every bundle in `paths/` (newest first). Click one to load it: the saved **preview.png** is shown, along with strokes/waypoints and the metadata it was saved with. Both files in a bundle work — clicking the row loads **path.json**; the small `json` / `script` badges load either file explicitly (the URScript is parsed back into waypoints, so a bundle with only a `.script` still replays).
- Enter the robot IP (prefilled from the last one used in the main app) and **Connect**, set **Speed / Safety / Radius** (prefilled from the file's own saved values), then **Run**. The saved waypoints are executed *literally* — offset and contact depth were already baked in at save time — with the same movel/movep actuation as the main app, so a replay traces exactly what `path.script` would. **Cancel** stops mid-path; a progress bar tracks the run.
- **Future robots:** everything brand-specific sits behind one small interface (`replay_robot.ReplayBackend`). Porting to e.g. an ABB GoFa means writing one backend class (compas_rrc: one MoveL per waypoint; `path.json` even carries a ready-made plane per waypoint for ABB's quaternion frames) and switching `REPLAY_BACKEND` in `config.py` — the loader, server and UI stay unchanged.
- Like the main app, closing the last browser tab stops the program.



### Configuration reference

All parameters live in `config.py`.

#### Server


| Variable    | Default       | Description  |
| ----------- | ------------- | ------------ |
| `HTTP_HOST` | `"localhost"` | Bind address |
| `HTTP_PORT` | `5005`        | Web UI port  |




#### Depth camera (RealSense)


| Variable                                   | Default | Description                           |
| ------------------------------------------ | ------- | ------------------------------------- |
| `DEPTH_WIDTH`                              | `640`   | Depth stream width (px)               |
| `DEPTH_HEIGHT`                             | `480`   | Depth stream height (px)              |
| `DEPTH_FPS`                                | `30`    | Depth stream frame rate               |
| `DEPTH_AVERAGE_FRAMES`                     | `30`    | Frames temporally averaged on Capture |
| `DEPTH_COLOR_NEAR_M` / `DEPTH_COLOR_FAR_M` | `0.0`   | Colormap range in metres (0 = auto)   |




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
| `MAX_TCP_SPEED`    | `1.0`       | m/s   | 100% on the Speed slider (UR10e rated max tool speed)                  |
| `DRAW_ACCEL`       | `0.3`       | m/s²  | Drawing acceleration                                                   |
| `TRAVEL_ACCEL`     | `0.5`       | m/s²  | Travel/retract acceleration                                            |
| `TOOL_ORIENTATION` | `[0, π, 0]` | rad   | Planar-mode TCP orientation (surface mode derives it per waypoint)     |
| `UR_REACH_M`       | `1.30`      | m     | Reach-check envelope radius around the base                            |
| `UR_MIN_REACH_M`   | `0.18`      | m     | Reach-check inner cylinder around the base axis                        |
| `MOVEP_BLEND_M`    | `0.0005`    | m     | Default movep blend radius (UI Radius slider 0–5 mm overrides per run) |




## References

- UR RTDE interface: [ur-rtde documentation](https://sdurobotics.gitlab.io/ur_rtde/)
- Intel RealSense SDK (`pyrealsense2`): [librealsense](https://github.com/IntelRealSense/librealsense)

