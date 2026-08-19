# surfaces/

Target meshes the captured drawing is ray-cast onto. **This folder is
`.gitignore`d apart from this file**, so a fresh clone or a ZIP download
arrives empty — the meshes have to be put back by hand.

## What has to be here

`main.py`'s `switch_to_tile()` builds one filename and only one, then
gives up if it is not there:

```python
path = SURFACE_DIR / f"tile_{tile_id:03d}.obj"
```

So the required set, today, is exactly these four files:

```
surfaces/tile_001.obj
surfaces/tile_002.obj
surfaces/tile_003.obj
surfaces/tile_004.obj
```

### The name, part by part

| Part | Must be | Why |
|---|---|---|
| folder | `surfaces/`, beside `main.py` | `config.SURFACE_DIR` is the **relative** path `Path("surfaces")`, so it resolves against the working directory. `run.bat` does `cd /d "%~dp0"` first, which is what makes that safe — launching `main.py` from somewhere else looks for `surfaces/` there instead. |
| prefix | `tile_` | Literal, with the underscore. |
| number | 3 digits, zero-padded | `001`, not `1` or `01`. From `{tile_id:03d}` — padding only grows past 999, so tile 42 would be `tile_042.obj`. |
| extension | `.obj` | Hardcoded here. `SurfaceModel.load()` reads STL perfectly well, but tile switching never asks for one — see below. |
| case | anything, on Windows | NTFS is case-insensitive, so `Tile_001.OBJ` is found. Match the lowercase form anyway: on Linux or macOS it would not be. |

`tile_id` is whatever `sandskript_sybil_rrc_dev` announces on ZeroMQ port
5558, and its `tile_selector` draws from `range(1, TOTAL_TILES + 1)` with
`tile_status.TOTAL_TILES = 4`. Tiles 1–4 are therefore the only IDs that
can arrive. Raise that constant and this folder needs the matching extra
files on the same day.

### Names that silently do nothing

`tile_1.obj` · `tile_01.obj` · `tile1.obj` · `tile_0001.obj` ·
`tile_001.stl` · `tile_001 (1).obj` · `tile-001.obj`

None of these raise an error. They are simply not the string the code
builds, so the tile is reported missing and the capture is skipped.

### STL

An `.stl` works if you load it by hand through the browser UI's surface
loader, because that path takes whatever filename you give it. It will
**never** be picked up by tile switching, which only ever asks for
`.obj`. If your Rhino export is STL, re-export as OBJ — or change the
extension in `switch_to_tile()`, which is a one-line edit.

**Do not just rename the file.** trimesh picks its parser by extension,
so an STL called `.obj` is handed to the OBJ reader, and the error you
get back is misleading:

```
ModuleNotFoundError: No module named 'charset_normalizer'
```

That is the OBJ reader failing to decode binary STL bytes as text, not a
broken environment. Confirmed on trimesh 4.12.2 — a genuine `.obj` in
the same env loads fine.

## Units

**Millimetres**, as exported from Rhino. `SurfaceModel.load()` converts
mm → m on the way in — that conversion is why an OBJ authored in metres
comes out 1000× too small.

Each tile keeps the position it was authored at in the Rhino document, in
the robot's base frame. Do **not** re-centre them on export: the tile's
placement in the file is what puts the drawing on the right part of the
garden.

## Symptom when they are missing

Nothing crashes — the failure is quiet, which is why it is worth knowing:

```
[zmq_bridge] tile 3 not found: surfaces\tile_003.obj
[emulate] tile 3: no surface loaded, skipping
```

A tile arrives, no mesh loads, `shared_state["surface_model"]` stays
`None`, and the emulated (or real) capture is skipped. Saved bundles then
come out with `"mode": "planar"` and `"surface_name": null` in
`path.json/meta` — that is the tell that this folder was empty when they
were written.

## Also ignored here

Any other `.obj` / `.stl` uploaded through the browser UI's surface
loader lands in this folder too (`main.py`'s `on_surface_upload`). Those
are working files, not project geometry — only the `tile_*.obj` set needs
to be restored.
