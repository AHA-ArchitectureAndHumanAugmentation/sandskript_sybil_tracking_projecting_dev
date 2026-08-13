"""
Profanity guard for the detected groove mask (Participant Mode only).

Reads whatever was raked into the sand with OCR and refuses the drawing when it
spells something on a wordlist. The mask — thick white strokes on black — is
what OCR is built for; the 1-px skeleton and the projected 3D strokes are not,
so the check always runs on the mask, before any robot motion is prepared.

Contained by design (same rule as ``stitcher`` / ``toolpath_loader``): pure
functions over a numpy array, no import of ``main`` or ``camera_thread``, no
hardware. ``find_profanity`` takes plain text, so the matching half is fully
unit-testable without an OCR engine installed.

Engine: Tesseract via ``pytesseract``. Both live INSIDE the sandskript conda env
(``conda install -c conda-forge tesseract libcurl``; libcurl is a real transitive
dependency — tesseract55.dll will not load without it). ``_ensure_engine`` points
pytesseract at the env copy and puts ``Library/bin`` on PATH, because run.bat
launches the env python directly rather than activating the env, so those DLLs
are not otherwise findable.

If the engine is missing the guard reports ``available=False`` and
``profane=False`` — a machine without Tesseract still runs the pipeline
normally rather than crashing or blocking every drawing.

Wordlists: every ``.txt`` in ``wordlists/`` (one entry per line, ``#`` comments).
Ships a small seed for English + German; drop in the LDNOOBW ``en``/``de`` files
under any name to extend coverage without touching code.
"""
from __future__ import annotations

import os
import re
import sys
import unicodedata
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

import numpy as np

from config import (
    PROFANITY_LANGS, PROFANITY_MIN_SUBSTRING_LEN, PROFANITY_OCR_ROTATIONS,
    PROFANITY_WORDLIST_DIR,
)

# Leetspeak / OCR-confusion folding applied before matching, so "sh1t" and a
# mis-read "5hit" still land on the same wordlist entry.
_LEET = str.maketrans({
    "0": "o", "1": "i", "3": "e", "4": "a", "5": "s", "7": "t", "8": "b",
    "@": "a", "$": "s", "!": "i", "|": "i",
})

# German folding so a list entry written either way still matches.
_DE_FOLD = (("ß", "ss"), ("ä", "ae"), ("ö", "oe"), ("ü", "ue"))

_WORD_RE = re.compile(r"[a-z]+")


@dataclass
class GuardVerdict:
    """Outcome of one check. ``profane`` is the only thing the pipeline gates on."""
    profane: bool = False
    word: str | None = None        # the wordlist entry that matched
    text: str = ""                 # what OCR actually read (for the log)
    available: bool = True         # False = no OCR engine, nothing was checked
    reason: str = ""               # human-readable, shown in the popup

    # Every distinct reading OCR produced, kept for logging/debugging.
    readings: list[str] = field(default_factory=list)


# ─────────────────────────────────────────────────────────────────────────────
# Text matching — pure, no OCR needed
# ─────────────────────────────────────────────────────────────────────────────

def normalize(text: str) -> str:
    """Lowercase, fold German umlauts and leetspeak, drop everything else."""
    text = text.lower()
    for src, dst in _DE_FOLD:
        text = text.replace(src, dst)
    # Strip accents (é → e) without touching the folding above.
    text = "".join(c for c in unicodedata.normalize("NFD", text)
                   if unicodedata.category(c) != "Mn")
    return text.translate(_LEET)


def find_profanity(
    text: str,
    words: set[str],
    min_substring_len: int = PROFANITY_MIN_SUBSTRING_LEN,
) -> str | None:
    """
    Return the first wordlist entry found in ``text``, or None.

    Two passes, because OCR of hand-raked sand breaks words apart unpredictably:
      1. whole-token match — any entry that appears as a standalone word;
      2. substring match against the de-spaced text, but only for entries at
         least ``min_substring_len`` long. The length floor is what keeps
         "assist" from tripping on a 3-letter entry (the Scunthorpe problem);
         it cannot eliminate false positives entirely.
    """
    if not text or not words:
        return None

    norm = normalize(text)
    tokens = set(_WORD_RE.findall(norm))
    for w in sorted(words, key=len, reverse=True):
        if w in tokens:
            return w

    squashed = "".join(_WORD_RE.findall(norm))
    if squashed:
        for w in sorted(words, key=len, reverse=True):
            if len(w) >= min_substring_len and w in squashed:
                return w
    return None


@lru_cache(maxsize=1)
def load_wordlists(directory: str | None = None) -> frozenset[str]:
    """
    Read every ``.txt`` under ``wordlists/`` into one normalized set.

    One entry per line; blank lines and ``#`` comments ignored. Any file works,
    so the LDNOOBW ``en``/``de`` lists can be dropped in as-is.
    """
    root = Path(directory) if directory else Path(__file__).parent / PROFANITY_WORDLIST_DIR
    words: set[str] = set()
    if not root.is_dir():
        return frozenset()
    for path in sorted(root.glob("*.txt")):
        try:
            raw = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        for line in raw.splitlines():
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            entry = "".join(_WORD_RE.findall(normalize(line)))
            if entry:
                words.add(entry)
    return frozenset(words)


# ─────────────────────────────────────────────────────────────────────────────
# OCR engine
# ─────────────────────────────────────────────────────────────────────────────

@lru_cache(maxsize=1)
def _ensure_engine() -> bool:
    """
    Point pytesseract at the copy inside this conda env and make its DLLs
    loadable. Cached — the probe runs once per process. False = unusable.
    """
    try:
        import pytesseract
    except ImportError:
        return False

    lib = Path(sys.prefix) / "Library" / "bin"          # Windows conda layout
    exe = lib / "tesseract.exe"
    if exe.is_file():
        # run.bat starts the env python WITHOUT activating the env, so
        # tesseract55.dll's neighbours are not on PATH unless we add them.
        if str(lib) not in os.environ.get("PATH", ""):
            os.environ["PATH"] = f"{lib}{os.pathsep}{os.environ.get('PATH', '')}"
        pytesseract.pytesseract.tesseract_cmd = str(exe)
    tessdata = Path(sys.prefix) / "share" / "tessdata"
    if tessdata.is_dir():
        os.environ.setdefault("TESSDATA_PREFIX", str(tessdata))

    try:
        pytesseract.get_tesseract_version()
        return True
    except Exception:
        return False


def engine_available() -> bool:
    """True when OCR can actually run (tests skip themselves on False)."""
    return _ensure_engine()


def _read_text(mask: np.ndarray, languages: str) -> list[str]:
    """
    OCR one mask, returning every distinct non-empty reading.

    The mask is white-on-black; Tesseract is trained on dark-text-on-light, so
    both polarities are tried. Each is also read upside down, because a
    participant standing on the far side of the sandbox writes rotated 180°.
    Four cheap passes on a 640×480 binary image, once per capture — not a
    per-frame cost.
    """
    import pytesseract

    binary = (mask > 0).astype(np.uint8) * 255
    readings: list[str] = []
    for image in (255 - binary, binary):                    # dark-on-light first
        for rot in PROFANITY_OCR_ROTATIONS:
            view = np.rot90(image, k=rot // 90) if rot else image
            try:
                out = pytesseract.image_to_string(
                    np.ascontiguousarray(view), lang=languages).strip()
            except Exception:
                continue
            if out and out not in readings:
                readings.append(out)
    return readings


def check_mask(
    mask: np.ndarray | None,
    languages: str = PROFANITY_LANGS,
    wordlist_dir: str | None = None,
) -> GuardVerdict:
    """
    Read a groove mask and decide whether it may proceed to the robot.

    Never raises: any failure (no engine, no language data, OCR error) comes
    back as ``available=False, profane=False`` so the pipeline keeps running.
    """
    if mask is None or getattr(mask, "size", 0) == 0:
        return GuardVerdict(available=False, reason="no mask to check")

    if not _ensure_engine():
        return GuardVerdict(available=False,
                            reason="OCR engine unavailable — drawing not checked")

    words = load_wordlists(wordlist_dir)
    if not words:
        return GuardVerdict(available=False,
                            reason="no wordlist found — drawing not checked")

    try:
        readings = _read_text(mask, languages)
    except Exception as exc:                                # pragma: no cover
        return GuardVerdict(available=False, reason=f"OCR failed: {exc}")

    for text in readings:
        hit = find_profanity(text, set(words))
        if hit:
            return GuardVerdict(
                profane=True, word=hit, text=text, readings=readings,
                reason=f"drawing reads as offensive text ({hit!r})",
            )

    joined = " ".join(readings)
    return GuardVerdict(profane=False, text=joined, readings=readings,
                        reason="no offensive text found")
