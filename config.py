import math
from pathlib import Path

# ── Server ────────────────────────────────────────────────────────────────────
# Port 5005 is deliberately off the common 8080/8000 range so this depth app can
# run alongside other tools (TouchDesigner, earlier RGB prototypes, etc.) without
# a port clash.
HTTP_HOST    = "localhost"
HTTP_PORT    = 5005
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
# Save writes one timestamped subfolder per toolpath here (URScript + JSON +
# preview image). Gitignored.
PATHS_DIR = Path("paths")

# ── Depth-number overlay (reference popup on the Depth viewport) ─────────────
# The /depths popup shows the live depth feed with the absolute distance (mm
# from the camera) written at the centre of each iso-depth region. Regions are
# depth bands `interval_mm` wide (popup slider); labels are computed at half
# resolution, only while the popup is open, throttled like the groove preview.
DEPTH_LABELS_EVERY       = 8      # compute every Nth camera frame (~3.75 Hz)
DEPTH_LABELS_INTERVAL_MM = 10.0   # default band width (popup slider, mm)
DEPTH_LABELS_MIN_AREA_PX = 60     # min region area in half-res pixels
DEPTH_LABELS_MAX         = 150    # cap on labels per frame (declutter + cost)

# ── Participant Mode (automated pipeline) ─────────────────────────────────────
# The ⧉ Participant Mode popup (/depths) runs capture → generate → save+run
# automatically while its Auto toggle is ON. The trigger watches the live depth
# frame: when at least TRIGGER_MIN_AREA_PX valid pixels are CLOSER to the camera
# than the user-entered threshold (mm), status becomes "Alerted"; once the frame
# stays clear for PARTICIPANT_CLEAR_S, the pipeline starts. The area minimum
# keeps single-pixel sensor noise from firing.
TRIGGER_MIN_AREA_PX = 150    # valid pixels below threshold that count as "something in frame"
PARTICIPANT_TICK_S  = 0.1    # automation poll interval (s)
PARTICIPANT_CLEAR_S = 1.0    # frame must stay clear this long before triggering

# ── Detection-parameter presets ───────────────────────────────────────────────
# The Save/Load buttons in the Detection Parameters panel write one timestamped
# JSON per preset here (just the slider values + detect mode). Gitignored.
PRESETS_DIR = Path("presets")

# ── Robot start position (joint angles in radians) ───────────────────────────
START_JOINT_ANGLES = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]
START_SPEED = 0.3
START_ACCEL = 0.5

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

# ── Robot drawing ─────────────────────────────────────────────────────────────
DRAW_Z           = -0.010  # m — pen contact (negative = below workspace surface origin)
TRAVEL_Z         =  0.050  # m — pen-up travel height above workspace surface origin
DRAW_SPEED       = 0.05    # m/s during drawing strokes (default; UI Speed slider overrides)
MAX_TCP_SPEED    = 1.0     # m/s — 100% on the Speed slider (UR10e rated max tool speed)

# ── Reach estimate (UR10e) ────────────────────────────────────────────────────
# Rough reachability envelope used to warn about waypoints the arm cannot get
# to: a sphere of REACH_M around the base, minus a thin inner cylinder around
# the base axis where the wrist cannot fold in. An estimate — not full IK.
UR_REACH_M     = 1.30
UR_MIN_REACH_M = 0.18
DRAW_ACCEL       = 0.3     # m/s²
TRAVEL_SPEED     = 0.15    # m/s during pen-up travel moves
TRAVEL_ACCEL     = 0.5     # m/s²
TOOL_ORIENTATION = [0.0, math.pi, 0.0]  # tool-down [rx, ry, rz]

# Blend radius (m) at each movep waypoint. Shared by the live executor and the
# saved URScript export so live drawing and a saved-file run trace identically.
# Must stay smaller than half the waypoint spacing or the controller rejects it.
MOVEP_BLEND_M    = 0.0005  # 0.5 mm

# ── Visualization ─────────────────────────────────────────────────────────────
VIS_INTERVAL = 0.05  # seconds between WebSocket state broadcasts

# ── Multi-Cam Vision prototype (stitch_main.py — CONTAINED) ───────────────────
# A standalone tool (run_stitch.bat → http://localhost:5006) that merges the
# depth feeds of however many D435i cameras are plugged in (1-4) into one
# top-down heightmap covering a larger sand area. The cameras are fixed, so
# placement is set by hand and saved — nothing searches for the overlap.
# Deliberately NOT wired into Developer or Participant Mode yet. It cannot run
# at the same time as the main app: one process per RealSense.
STITCH_HTTP_PORT      = 5006
STITCH_CALIB_FILE     = Path("stitch_calibration.json")  # per-camera placements, gitignored
STITCH_AVERAGE_FRAMES = 10     # temporal averaging per camera (smaller than main: live-ish)
STITCH_EVERY_S        = 0.25   # seconds between stitched-output recomputes (~4 Hz)
STITCH_MM_PER_PX      = 0.0    # heightmap grid resolution; 0 = auto from median depth
STITCH_MAX_GRID_W     = 1920   # cap the heightmap size (cost bound)
STITCH_MAX_GRID_H     = 1080
STITCH_MAX_CAMERAS    = 4      # extra devices are ignored (MJPEG/USB bandwidth)
STITCH_SYNTHETIC_CAMERAS = 3   # cameras faked when no RealSense is attached
STITCH_HEIGHT_STEP_MM = 2.0    # one press of Raise/Lower on a camera
# Nominal D435 depth intrinsics (87°×58° FOV) — used for the synthetic fallback
# and as a sanity default; real runs read exact intrinsics from each device.
STITCH_NOMINAL_HFOV_DEG = 87.0
STITCH_NOMINAL_VFOV_DEG = 58.0

# ── Saved-toolpath replay tool (replay_main.py — CONTAINED) ───────────────────
# A standalone tool (run_replay.bat → http://localhost:5007) that connects to
# the robot, lists the saved bundles under paths/ and re-runs one (path.json or
# path.script both load). NOT wired into Developer/Participant Mode. Cannot run
# while the main app is connected to the robot (one RTDE controller per robot).
# The robot brand is abstracted behind replay_robot.ReplayBackend so a future
# ABB GoFa port only adds a backend class + changes REPLAY_BACKEND.
REPLAY_HTTP_PORT = 5007
REPLAY_BACKEND   = "ur"     # see replay_robot.make_backend()
