# Sandskript_sybil_tracking_projecting_dev

A stripped-down fork of `SANDSKRIPT_depth-cam-to-robot` (`zeromq-bridge`
branch). All UR robot execution removed. Purpose: capture a drawing → detect
the groove → project it onto whichever tile `sandskript_sybil_rrc_dev`
selected → send the result back to it. Private repo, AHA org, built to
eventually fold back into `SANDSKRIPT_depth-cam-to-robot`.

## What's different from the original repo

**Removed entirely:**
- `robot_controller.py`, `path_executor.py`, `registration.py`, `reach.py` — all UR/RTDE-specific
- `workspace.py`'s flat-surface fallback is unused now (kept, since removing it wasn't necessary and it's harmless)

**Robot-related callbacks are now stubs** — `on_robot_connect`, `on_run`, `on_register_freedrive`, `on_register_corner`, etc. still exist (so `Server(...)` doesn't crash on a missing argument) but do nothing except return "not available in this repo."

**Added:**
| File | Purpose |
|---|---|
| `zmq_bridge.py` | `send_path_to_charlotte()` (metres→mm, adds `tile_id`, sends the result to `sandskript_sybil_rrc_dev`) and `TileReceiver` (background thread, receives the next tile from `sandskript_sybil_rrc_dev`) |
| `tile_viewer.py` | `compas_viewer` windows, mirrors `sandskript_sybil_rrc_dev`'s `view_utils.py` exactly (same colors, line weights, camera framing). Runs as a **separate process per window** — needed to avoid an OpenGL context crash between two windows in one process |
| `surfaces/tile_001.obj` – `tile_006.obj` | Exported tile meshes, native mm |

**New settings, all at the top of `main.py`:**
| Setting | Default | Purpose |
|---|---|---|
| `EMULATE_CAPTURE` | `True` | Fakes camera capture + groove detection after every tile switch. `False` = wait for a real capture. |
| `STARTUP_TILE_ID` | `1` | Uses this tile immediately on startup, no message needed. `None` = wait for `sandskript_sybil_rrc_dev`. |
| `SHOW_PROJECTION_VIEWER` | `True` | Opens two `compas_viewer` windows per generated path (raw stroke, then projected result). Blocks until each is closed — turn off for unattended runs. |

**One real bug fixed, not cosmetic:** `TileReceiver` now increments a `next_tile_seq` counter on every message received, not just when the tile number changes. Without this, if the same tile got selected twice in a row, the watcher saw no change and silently stalled.

## What's verified working

- ZeroMQ, both directions — real messages, real receive, tested repeatedly between this repo and `sandskript_sybil_rrc_dev`
- Tile switching — confirmed loading the correct `.obj` per tile ID
- **Projection (raycasting)** — confirmed with the actual point count: 73 of 80 emulated points landed on the real trimmed tile mesh (the other 7 correctly missed the trim edge — expected, not a bug)
- Save (`save_bundle`) and send (`send_path_to_charlotte`) — confirmed, `path.json` arrives correctly in `sandskript_sybil_rrc_dev` and runs through its `301`/`302`/`304`
- The full automated loop — tile selected → switched → captured (emulated) → projected → saved → sent → received → next tile selected → repeat, running unattended across multiple cycles, confirmed with both repos running together

## What's NOT yet verified — read this before assuming more than is proven

- **Real camera capture.** `camera_thread.py` was never touched, but also never tested this session — no RealSense was connected.
- **Real groove extraction** (`extract_from_edges`: smoothing, joining, resampling, ordering) **with real depth-camera input.** The current emulation *bypasses this entirely* — it fabricates already-clean pixel points directly, skipping smoothing/joining/resampling/ordering altogether.
- **A genuine attempt was made to run the real extraction pipeline** (feeding a synthetic groove image into the actual `extract_from_edges()`), and it surfaced a real, understood bug — **in the synthetic test image, not in the extraction code:**
  > Independent random jitter applied to every point, when the jitter is comparable in size to the spacing between points, makes the line double back on itself locally. `_chains_from_edges`' chain-following handles a genuine 1-pixel skeleton fine, but not a locally self-crossing tangle — this produced a runaway point count (over a million), not a real result.
  >
  > **The fix, identified but not yet applied:** replace independent per-point jitter with a second, smaller sine wave as continuous noise — smooth and wiggly, but mathematically guaranteed not to double back on itself. This is a real, ready next step if someone wants to extend the emulation further; not required for anything currently working.
- Participant Mode's auto-trigger and the profanity guard are present and untouched in the file, but dormant — both depend on real camera input to do anything.

## How to run it

```powershell
conda activate sandskript
python main.py
```

With the defaults above, it runs the whole loop automatically on startup — no manual trigger needed. Two `compas_viewer` windows will open per generated path; close each to let it continue.

**To run the full loop together with `sandskript_sybil_rrc_dev`:** start that repo's `main.py` first (autonomous mode, listening), then start this repo's `main.py`. Order matters — the receiving side needs to be listening before anything gets sent.

## Switching to a real camera

Two changes, nothing else:
1. `EMULATE_CAPTURE = False`
2. In the browser UI: turn on **Participant Mode → Auto**, set a trigger distance

Everything downstream (extraction, projection, save, send, tile selection) is already real code and needs no changes.

## Notes for whoever picks up `SANDSKRIPT_depth-cam-to-robot` next

This repo is a *fork*, not a replacement — the original repo and its `zeromq-bridge` branch are untouched. What changed here:
- Robot execution removed (not needed for this piece of the project)
- Tile-switching and emulation added on top of the real capture/projection code, which was not modified
- The groove-extraction bug above is the most concrete open item — a real synthetic-image fix, using the real `extract_from_edges()` unmodified
