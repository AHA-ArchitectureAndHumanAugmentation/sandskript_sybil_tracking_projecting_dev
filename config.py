import math
from pathlib import Path

# ── Server ────────────────────────────────────────────────────────────────────
# The 51xx block is deliberately off the common 8080/8000 range so this depth app
# can run alongside other tools (TouchDesigner, earlier RGB prototypes, etc.)
# without a port clash.
# This is the sybil tracking/projecting DEV-TEST variant, so it sits on 5105-5108
# — a +100 offset from the 5005-5008 block used by the sibling depth_cam-to-robot
# checkout. Both can therefore be up at the same time. Keep that offset when
# adding a new tool port here.
HTTP_HOST    = "localhost"
HTTP_PORT    = 5105
DEPTH_PATH   = "/depth"             # MJPEG: colorized depth (the live view)
RGB_PATH     = "/rgb"               # MJPEG: aligned colour image
GROOVE_PATH  = "/depth/grooves"     # MJPEG: detected groove centrelines (skeleton)
MASK_PATH    = "/depth/mask"        # MJPEG: thick detected-region mask
WS_PATH      = "/ws"
STATIC_PATH  = "/static"

# ── Persistent files ──────────────────────────────────────────────────────────
WORKSPACE_FILE = Path("workspace.json")
SETTINGS_FILE  = Path("settings.json")

# ── Target surface (Rhino mesh export) ────────────────────────────────────────
# STL/OBJ meshes uploaded from the browser land here. Rhino exports are usually
# in millimetres, so files are scaled by SURFACE_UNITS_TO_M on load (set to 1.0
# if you export in metres).
SURFACE_DIR        = Path("surfaces")
SURFACE_UPLOAD_URL = "/surface/upload"
SURFACE_UNITS_TO_M = 0.001
SURFACE_MAX_FACES  = 80000   # warn above this — browser preview gets heavy

# ── Saved toolpaths ───────────────────────────────────────────────────────────
# Save writes one timestamped subfolder per toolpath here (JSON +
# preview image). Gitignored.
PATHS_DIR = Path("paths")

# ── Depth-number overlay (reference popup on the Depth viewport) ─────────────
# The /depths popup shows the live depth feed with the absolute distance (mm
# from the camera) written at the centre of each iso-depth region. Regions are
# depth bands `interval_mm` wide (popup slider); labels are computed at half
# resolution, only while the popup is open, throttled like the groove preview.
DEPTH_LABELS_EVERY       = 4      # compute every Nth combined canvas (~1.2 Hz)
DEPTH_LABELS_INTERVAL_MM = 10.0   # default band width (popup slider, mm)
DEPTH_LABELS_MIN_AREA_PX = 60     # min region area in half-res pixels
DEPTH_LABELS_MAX         = 150    # cap on labels per frame (declutter + cost)

# ── Participant Mode (automated pipeline) ─────────────────────────────────────
# The ⧉ Participant Mode popup (/depths) runs capture → generate → save+run
# automatically while its Auto toggle is ON. The trigger watches the live depth
# frame: when at least TRIGGER_MIN_AREA_PX valid pixels read as "something is
# there", status becomes "Alerted"; once the frame stays clear for
# PARTICIPANT_CLEAR_S, the pipeline starts. The area minimum keeps single-pixel
# sensor noise from firing.
#
# What the threshold MEASURES depends on whether a reference frame is set:
# with one it is a height ABOVE THE SAND (tilt-proof, tens of mm); without one
# it is a raw distance from the camera (hundreds of mm). One range covers both,
# so the floor is low enough for the relative mode and the ceiling high enough
# for the absolute one.
TRIGGER_MIN_AREA_PX = 150    # valid pixels past the threshold that count as "something in frame"
TRIGGER_MIN_MM      = 5.0    # smallest accepted threshold (a low hover above the sand)
TRIGGER_MAX_MM      = 5000.0 # largest (a camera metres away, absolute mode)
PARTICIPANT_TICK_S  = 0.1    # automation poll interval (s)
PARTICIPANT_CLEAR_S = 1.0    # frame must stay clear this long before triggering
# Max Drawing Time: how long ONE participant may keep the sand occupied —
# measured from "Alerted" (hand enters) to the frame going clear (hand leaves).
# Entered in the popup in MINUTES (empty/0 = no limit); the countdown in the
# stage's top-left corner turns red for the last PARTICIPANT_WARN_S seconds, and
# running out is an Invalid verdict: nothing is saved and nothing is run.
PARTICIPANT_MAX_DRAW_MIN     = 0.0    # default limit (minutes; 0 = off)
PARTICIPANT_MAX_DRAW_MIN_MIN = 0.1    # clamp: shortest limit the box accepts (6 s)
PARTICIPANT_MAX_DRAW_MAX_MIN = 120.0  # clamp: longest limit the box accepts
PARTICIPANT_WARN_S           = 10.0   # countdown goes red with this much left
# preview.png comes from a Developer window's 3D canvas, which nobody clicks Save
# on during an automated run — so while Auto is ON the browser pushes the shot up
# by itself after each Generate Path. Cap what we will hold in memory: this is a
# client-supplied blob, and a ~600x400 canvas encodes well under 1 MB.
PREVIEW_MAX_BYTES = 8 * 1024 * 1024

# ── Participant-Mode sound cues (projection window) ───────────────────────────
# Four short synthesized cues mark the stages a participant can hear: the
# machine noticing them, the drawing being accepted, the robot about to move,
# and a refusal. They are played by the PROJECTION OUTPUT window only (that is
# the window plugged into the projector, whose speaker they come out of) — no
# projection window open, no sound. sound_design.py renders the .wav files into
# SOUNDS_DIR; drop your own recordings in under the same names to replace them.
SOUNDS_DIR        = Path("sounds")
SOUNDS_URL_PATH   = "/sounds"    # static route the projection page fetches from
SOUND_SAMPLE_RATE = 44100
# Participant status → cue file stem (<stem>.wav). The cue plays once, when the
# status is ENTERED. Statuses not listed here are silent.
SOUND_CUES = {
    "Alerted":   "engaged",        # hand enters — "I see you, go ahead"
    "Sensing":   "acknowledged",   # hand leaves — "got it, working on it"
    "Actuating": "anticipation",   # saving + running — "here it comes"
    "Invalid":   "alarm",          # refused — profanity, too long, out of time
}

# ── Detection-parameter presets ───────────────────────────────────────────────
# The Save/Load buttons in the Detection Parameters panel write one timestamped
# JSON per preset here (just the slider values + detect mode). Gitignored.
PRESETS_DIR = Path("presets")

# ── Depth camera (Intel RealSense D435i) ──────────────────────────────────────
# The D435i streams metric depth directly, so grooves raked into sand — a few-mm
# physical depression invisible to an RGB camera — are measured, not inferred
# from shading. We keep the raw depth internally and only colorize it for display.
DEPTH_WIDTH          = 640
DEPTH_HEIGHT         = 480
DEPTH_FPS            = 30
DEPTH_AVERAGE_FRAMES = 30     # frames temporally averaged on Capture (cuts noise ~√n)

# Colormap range (metres) for the live depth view. 0 = auto (per-frame percentile).
DEPTH_COLOR_NEAR_M = 0.0
DEPTH_COLOR_FAR_M  = 0.0

# ── Groove detection (depth → groove centrelines) ─────────────────────────────
# See depth_extractor.grooves_from_depth for the algorithm. These are the live-feed
# defaults and the initial values of the browser's Adjust controls.
GROOVE_SMOOTH_SIGMA_PX  = 1.5    # denoise the depth map first
GROOVE_DETREND_SIGMA_PX = 25.0   # blur radius estimating the bare-sand surface
GROOVE_DEPTH_MM         = 1.5    # how much deeper than the surface counts as a groove
GROOVE_MIN_BLOB_PX      = 40     # discard connected specks smaller than this
GROOVE_DETECT           = "valley"  # "valley"=grooves, "ridge"=raised lines, "band"=iso-depth
GROOVE_NEAR_MARGIN_PX   = 12     # dilation around a too-near object when rejecting its mask blobs

# ── Path extraction ───────────────────────────────────────────────────────────
CONTOUR_MIN_PIXELS  = 20    # discard contours shorter than this many pixels
RESAMPLE_SPACING_MM = 10.0  # default spacing between robot waypoints in mm (the
                            # Path Preview "Spacing" slider overrides this per run)
RESAMPLE_SPACING_MIN_MM = 10.0   # slider lower bound
RESAMPLE_SPACING_MAX_MM = 100.0  # slider upper bound

# Stroke joining — the Path Preview "Distance Threshold" box. Two endpoints
# belonging to different strokes are connected (the strokes merge into one
# continuous toolpath) when the gap between them is below this distance.
JOIN_DISTANCE_MM     = 0.0    # default; 0 = off, every stroke stays separate
JOIN_DISTANCE_MIN_MM = 0.0    # box lower bound (0 disables joining)
JOIN_DISTANCE_MAX_MM = 200.0  # box upper bound
# When the straight line closing a gap is crossed by a THIRD stroke, that
# crossing is evidence the two ends are one gesture interrupted by another
# groove — so the threshold is multiplied by this before the pair is judged.
JOIN_CROSSING_FACTOR = 2.0

# Max Total Length — the Path Preview box beside Distance Threshold. A ceiling
# on how far the tool may travel WHILE DRAWING (path_length.blended_length: the
# green line, after surface projection, at the current Spacing / Radius /
# Distance Threshold). Over it, the path is refused: Run and Save Path both
# decline, and Participant Mode calls the drawing Invalid instead of actuating.
# Bounds material use rather than motion, so travels and retracts don't count.
MAX_PATH_LENGTH_MM     = 0.0       # default; 0 = off, any length allowed
MAX_PATH_LENGTH_MIN_MM = 0.0       # box lower bound (0 disables the limit)
MAX_PATH_LENGTH_MAX_MM = 100000.0  # box upper bound (100 m of drawing)

# ── Profanity guard (Participant Mode only) ───────────────────────────────────
# OCR the detected groove mask and refuse the drawing when it spells something
# on a wordlist. Runs once per automated capture, between path generation and
# actuation — never in Developer Mode, where the operator decides.
PROFANITY_CHECK_ENABLED = True
PROFANITY_LANGS = "eng+deu"   # Tesseract language packs (both ship with the env)
PROFANITY_WORDLIST_DIR = "wordlists"
# Entries at least this long also match inside a run-together word; shorter ones
# must stand alone. Lower it and "ass" starts flagging "classic".
PROFANITY_MIN_SUBSTRING_LEN = 4
# Degrees to re-read the mask at — a participant on the far side of the sandbox
# writes upside down relative to the camera.
PROFANITY_OCR_ROTATIONS = (0, 180)

# ── Console module attribution (module_trace.py) ──────────────────────────────
# Startup table mapping every feature to its .py files, and a one-line module
# chain under each task the app performs. Console only — no effect on the UI.
SHOW_MODULE_BANNER = True
SHOW_MODULE_TRACE  = True

# ── Stroke motion parameters ──────────────────────────────────────────────────
# No robot connects from this repo (that's sandskript_sybil_rrc_dev's job —
# see main.py's _NO_ROBOT_MSG), but these still shape the SAVED bundle: the
# speed/offset/safety/blend defaults in the exec-parameter UI, and the numbers
# written into path.json's meta for whatever runs the path downstream.
TRAVEL_Z         =  0.050  # m — pen-up travel height above workspace surface origin
DRAW_SPEED       = 0.05    # m/s during drawing strokes (default; UI Speed slider overrides)
# 100% on the Speed slider, set to the GoFa 10's rated tool speed. NOTE this is
# the number every Speed percentage is relative to, so a bundle saved before the
# ABB port at "50%" was 0.5 m/s and now replays at 1.0 m/s. DRAW_SPEED above is
# absolute and unaffected; only the slider's meaning moved.
MAX_TCP_SPEED    = 2.0     # m/s
DRAW_ACCEL       = 0.3     # m/s² — read by path_export.stroke_blend
TRAVEL_ACCEL     = 0.5     # m/s² — read by path_export.stroke_blend
TOOL_ORIENTATION = [0.0, math.pi, 0.0]  # tool-down [rx, ry, rz] — flat-workspace fallback

# Corner blend at each drawing waypoint — RAPID "zone data", the radius (m)
# within which the controller may round off a corner instead of stopping on it.
# Must stay smaller than half the waypoint spacing or corners get cut; see
# path_export.stroke_blend, which clamps it per stroke.
BLEND_ZONE_M     = 0.0005  # 0.5 mm
BLEND_ZONE_MAX_MM = 50.0   # Radius slider upper bound; stroke_blend still
                           # clamps it per stroke, so asking for more than the
                           # spacing allows simply rounds as much as it can

# Saved path.json carries an identity work object as three COMPAS points
# (origin + a point on each axis), for whatever compas_rrc script consumes it
# downstream to rebuild a plane from them. The axis points are 1 m out along
# X and Y; only their DIRECTION matters, so this is a readability choice, not
# a scale.
WOBJ_AXIS_MM     = 1000.0

# ── Visualization ─────────────────────────────────────────────────────────────
VIS_INTERVAL = 0.05  # seconds between WebSocket state broadcasts

# ── Multi-Cam Vision (stitch_main.py — the placement tool) ────────────────────
# A standalone tool (run_stitch.bat → http://localhost:5106) that merges the
# depth feeds of however many D435i cameras are plugged in (1-4) into one
# top-down heightmap covering a larger sand area. The cameras are fixed, so
# placement is set by hand and saved to STITCH_CALIB_FILE — nothing searches
# for the overlap.
# The MAIN APP READS THAT SAME FILE: every depth/colour view in Developer and
# Participant Mode is this combined canvas, so the tool is where the picture the
# pipeline sees gets shaped. It cannot run at the same time as the main app:
# one process per RealSense.
STITCH_HTTP_PORT      = 5106
STITCH_CALIB_FILE     = Path("stitch_calibration.json")  # per-camera placements, gitignored
STITCH_AVERAGE_FRAMES = 10     # temporal averaging per camera (smaller than main: live-ish)
STITCH_EVERY_S        = 0.25   # seconds between stitched-output recomputes (~4 Hz)
STITCH_MM_PER_PX      = 0.0    # heightmap grid resolution; 0 = auto from median depth
STITCH_MAX_GRID_W     = 1920   # cap the heightmap size (cost bound)
STITCH_MAX_GRID_H     = 1080
STITCH_MAX_CAMERAS    = 4      # extra devices are ignored (MJPEG/USB bandwidth)
STITCH_SYNTHETIC_CAMERAS = 3   # cameras faked when no RealSense is attached
STITCH_HEIGHT_STEP_MM = 2.0    # one press of Raise/Lower on a camera
# Seam check: how far apart two cameras' depths are where they see the same
# sand, and whether that gap is a constant HEIGHT step (the Height ▼▲ buttons
# cancel it) or varies across the seam because the cameras differ in ANGLE (they
# cannot). Slower than the canvas because it re-warps every camera a second
# time, and the answer only moves when the operator moves something.
STITCH_SEAM_EVERY_S   = 1.0    # seconds between seam re-measurements
# Nominal D435 depth intrinsics (87°×58° FOV) — used for the synthetic fallback
# and as a sanity default; real runs read exact intrinsics from each device.
STITCH_NOMINAL_HFOV_DEG = 87.0
STITCH_NOMINAL_VFOV_DEG = 58.0

# How the MAIN APP drives the same stitch. Every camera is still buffered at the
# full DEPTH_FPS (Capture averages DEPTH_AVERAGE_FRAMES of raw frames per camera
# and stitches the averages), but the LIVE canvas — the depth/colour views, the
# groove preview and the Participant trigger — is rebuilt on this slower clock,
# because warping several 640×480 frames onto a big canvas 30 times a second
# would buy nothing on static sand.
# This repo runs it at ~5 Hz rather than upstream's ~10 Hz default — keep this
# value; it is a deliberate choice for this rig, not a stale copy.
STITCH_MAIN_EVERY_S    = 0.2    # seconds between live canvas rebuilds (~5 Hz)
# Groove/mask preview every Nth canvas. 1 = same rate as the depth view, which
# is what keeps the Mask viewport from lagging visibly behind Depth; 2 halves
# its cost at the price of doubling its latency.
LIVE_GROOVE_EVERY      = 1

# ── Live-view profiling (diagnostic; off by default) ──────────────────────────
# Both live views come off ONE thread, so a slow stage delays every stage after
# it. Switch this on to print, every PROFILE_EVERY_S, where the canvas cycle's
# time went and what rate it ACHIEVED versus the 1/STITCH_MAIN_EVERY_S it aims
# for — plus how many MJPEG frames each stream sent versus how many were new
# pictures. Read the achieved Hz first: near target = the cadence is the limit
# and the work has headroom; well under = the work is the limit, and the top
# stage in the list is where the time is. Costs nothing while False.
PROFILE_PIPELINE = False
PROFILE_EVERY_S  = 5.0    # seconds between profile reports
# Wait this long for every camera's first frame before freezing the canvas
# geometry. Freezing early would size the canvas to a partial rig.
STITCH_MAIN_BIND_TIMEOUT_S = 4.0

# ── Scheduler (contained tool) ────────────────────────────────────────────────
# A standalone, READ-ONLY tool (run_scheduler.bat → http://localhost:5108) that
# lists the saved bundles under paths/ as a numbered spreadsheet: which path
# went out, and when. Touches no hardware and writes nothing, so it is the one
# tool that is safe to leave running beside the main app.
SCHEDULER_HTTP_PORT = 5108
SCHEDULER_REFRESH_S = 2.0   # how often paths/ is re-scanned for new bundles
