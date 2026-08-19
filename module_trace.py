"""
Console module attribution — which .py file does what, printed as it happens.

Two things, both console-only:

  * ``banner()`` — a startup table mapping every feature to the modules that
    implement it, with a live ✓/· marker showing which are actually imported in
    this process (so an optional dependency that silently failed is visible).
  * ``log(action, message)`` — the normal task line with the module chain that
    served it appended, e.g.

        Generated path: 12 strokes, 340 points
          └ depth_extractor.py → path_extractor.py → surface.py → reach.py

Contained by design (same rule as ``stitcher`` / ``text_guard``): pure data and
string formatting, no import of ``main`` and no side effects beyond printing.
The chains are hand-maintained — when a stage changes owner, update STAGES here
in the same commit, exactly as CLAUDE.md's pipeline list is updated.

Turn the per-task trail off with SHOW_MODULE_TRACE = False in config.py; the
startup banner is controlled by SHOW_MODULE_BANNER.
"""
from __future__ import annotations

import sys

from config import SHOW_MODULE_BANNER, SHOW_MODULE_TRACE

# Feature → the modules that implement it, in rough call order. Mirrors the
# program tree in README.md and the pipeline list in CLAUDE.md.
FEATURES: dict[str, tuple[str, ...]] = {
    "Core / server":      ("main", "server", "config", "settings"),
    # Capture is multi-camera: every RealSense is read (realsense_source) and
    # laid onto ONE combined canvas (stitcher) using the layout saved by the
    # Multi-Cam Vision tool — that canvas is what the pipeline sees.
    "Capture":            ("camera_thread", "realsense_source", "stitcher",
                           "depth_extractor", "view_rotation"),
    "Groove detection":   ("depth_extractor",),
    "Stroke extraction":  ("path_extractor",),
    "Surface mapping":    ("surface", "workspace"),
    # No robot connects from this repo — a saved bundle is sent onward over
    # ZeroMQ instead (see main.py's _NO_ROBOT_MSG and zmq_bridge.py).
    "Export / send":      ("path_export", "zmq_bridge"),
    "Participant Mode":   ("automation", "text_guard"),
    # sound_design renders the cue .wav files ahead of time and is not imported
    # by the running app (the projection window just fetches them), so it shows
    # as not-loaded here on purpose.
    "Sound cues":         ("sound_design", "server"),
}

# Pipeline action → the module chain that actually runs for it. Keys are the
# short names passed to log().
STAGES: dict[str, tuple[str, ...]] = {
    # connect/disconnect/register/run/cancel are robot features, stubbed in
    # this repo — no module chain does real work for them, so they're absent
    # here rather than pointing at modules that no longer exist.
    "capture":      ("camera_thread", "realsense_source", "stitcher", "depth_extractor"),
    "preview":      ("depth_extractor",),
    "reference":    ("camera_thread", "depth_extractor"),
    # Turning the canvas re-bases the crop and reference (view_rotation) and the
    # frame aspect the flat workspace is shaped by.
    "rotate":       ("camera_thread", "view_rotation", "workspace"),
    "generate":     ("depth_extractor", "path_extractor"),
    "surface":      ("surface",),
    "save":         ("path_export", "zmq_bridge"),
    "participant":  ("automation",),
    "guard":        ("text_guard",),
    "projector":    ("server", "camera_thread"),
}

_ARROW = " → "


def _loaded(module: str) -> bool:
    """True when the module is imported in this process right now."""
    return module in sys.modules


def chain(action: str, extra: tuple[str, ...] = ()) -> str:
    """
    "depth_extractor.py → path_extractor.py → surface.py" for one action.

    ``extra`` appends modules the caller only knows about at runtime — the
    surface-vs-workspace branch in generate, for instance.
    """
    mods = STAGES.get(action, ())
    seen: list[str] = []
    for m in (*mods, *extra):
        if m and m not in seen:
            seen.append(m)
    return _ARROW.join(f"{m}.py" for m in seen)


def log(action: str, message: str, extra: tuple[str, ...] = ()) -> None:
    """Print a task line, then the module chain that served it underneath."""
    print(message)
    if not SHOW_MODULE_TRACE:
        return
    trail = chain(action, extra)
    if trail:
        print(f"  └ {trail}")


def banner() -> str:
    """The startup feature→modules table. Returns the text (also easy to test)."""
    width = max(len(name) for name in FEATURES)
    lines = ["", "── Python modules by feature "
                 "──────────────────────────────────────────────",
             "   ✓ = imported in this process    · = not loaded", ""]
    for feature, mods in FEATURES.items():
        marks = "  ".join(
            f"{'✓' if _loaded(m) else '·'} {m}.py" for m in mods)
        lines.append(f"   {feature:<{width}}   {marks}")
    lines += ["",
              "   Each task below prints the modules that served it on the "
              "next line.",
              "───────────────────────────────────────────────────────────"
              "──────────────", ""]
    return "\n".join(lines)


def print_banner() -> None:
    """Print the startup table unless SHOW_MODULE_BANNER is off."""
    if SHOW_MODULE_BANNER:
        print(banner())
