"""
The Participant-Mode sound cues: four short pieces, synthesized from scratch.

Run this file to (re)write them as .wav into ``sounds/``::

    <ENVPY> sound_design.py            # → sounds/engaged.wav, ... (4 files)

They are played by the projection OUTPUT window (viewer/projection.html), which
is the window on the projector — so the projector's speaker is what a
participant hears, and closing the projection means silence. config.SOUND_CUES
maps a participant status to the cue that plays when it is entered.

Why compose them here rather than ship recordings: the cues have to be
regenerable and adjustable (a nudge in pitch or length is an edit to a line
below, not a trip to an audio editor), and four seconds of mono audio is
cheaper to synthesize than to store well. Real recordings can still replace
them — the browser only asks for ``<stem>.wav``.

## The four cues, and why they sound the way they do

Each one has a job in the participant's head, and the choices below are the
ordinary psychoacoustics of getting that job done:

* **engaged** (hand enters) — an A major triad, ascending, on soft bell tones.
  Rising pitch reads as invitation and opening; a consonant major triad reads
  as friendly; a slow attack keeps it from startling someone who has just
  leaned over the sand. It ends on the fifth rather than the root, which the
  ear hears as unfinished: *go on*.
* **acknowledged** (hand leaves) — the same three notes in reverse, landing on
  the root, with a low octave under it. Falling pitch to the tonic is closure
  ("received"), and the final note is held with a slow amplitude pulse, which
  is heard as a machine still working rather than a machine finished.
* **anticipation** (saving + running) — five notes of A major pentatonic rising
  with the gaps between them shrinking, over a swelling low tone. Accelerating
  rhythm plus rising pitch plus a crescendo is the standard grammar of "about
  to happen"; the pentatonic scale has no semitone clashes, so it lifts without
  sounding anxious. It stops on the sixth — again unresolved, leaning forward.
* **alarm** (drawing refused) — a two-tone klaxon a tritone apart, sawtooth,
  with a sub-octave, a fast tremolo and a downward bend in each honk. Every
  one of those is a warning-sound cliché for a reason: the tritone is the most
  dissonant interval in the scale, the harmonics of a sawtooth are dense enough
  to be *rough* rather than musical, and rapid modulation is what the ear reads
  as urgency. It is also the loudest of the four, so it cannot be mistaken for
  one of the friendly ones.

Contained module: pure numpy + the stdlib ``wave`` writer. It imports nothing
from ``main`` and touches no hardware, so the cues can be rendered and tested
without the app running.
"""
from __future__ import annotations

import wave
from pathlib import Path

import numpy as np

from config import SOUND_SAMPLE_RATE, SOUNDS_DIR

SR = SOUND_SAMPLE_RATE

# One partial = (frequency multiple, amplitude). A struck-bell-ish stack: a
# strong fundamental with quiet, slightly inharmonic upper partials.
BELL = ((1.0, 1.0), (2.0, 0.42), (3.0, 0.18), (4.2, 0.08))
SOFT = ((1.0, 1.0), (2.0, 0.30), (3.0, 0.10))
PURE = ((1.0, 1.0),)
# A sawtooth (all harmonics, 1/n) — dense enough to sound harsh, which is the
# whole point of the alarm.
SAW = tuple((float(k), 1.0 / k) for k in range(1, 13))


# ── Primitives ───────────────────────────────────────────────────────────────
def _times(duration: float) -> np.ndarray:
    return np.arange(max(1, int(round(duration * SR))), dtype=np.float64) / SR


def _canvas(duration: float) -> np.ndarray:
    return np.zeros(max(1, int(round(duration * SR))), dtype=np.float64)


def _hz(semitones: float) -> float:
    """Equal-tempered pitch, in semitones from A4 (440 Hz)."""
    return 440.0 * (2.0 ** (semitones / 12.0))


def _envelope(t: np.ndarray, attack: float, tau: float,
              release: float = 0.05) -> np.ndarray:
    """
    Raised-cosine attack → exponential decay → raised-cosine release.

    The two cosine tapers are not decoration: a waveform that starts or stops
    at a non-zero sample clicks, and a click is itself an alarming noise.
    """
    end = t[-1] if len(t) else 0.0
    rise = np.clip(t / max(attack, 1e-6), 0.0, 1.0)
    fall = np.clip((end - t) / max(release, 1e-6), 0.0, 1.0)
    smooth = lambda x: 0.5 - 0.5 * np.cos(np.pi * x)   # noqa: E731
    return smooth(rise) * np.exp(-t / max(tau, 1e-6)) * smooth(fall)


def _tone(freq: float, duration: float, partials=PURE, *, tau: float = 0.3,
          attack: float = 0.008, tremolo_hz: float = 0.0,
          tremolo_depth: float = 0.0, bend: float = 1.0) -> np.ndarray:
    """
    One note. ``bend`` glides the pitch to ``freq * bend`` across the note
    (the siren character in the alarm); the phase is integrated rather than
    multiplied so the glide stays continuous.
    """
    t = _times(duration)
    ramp = np.linspace(1.0, bend, len(t))
    sig = np.zeros_like(t)
    for mult, amp in partials:
        phase = np.cumsum(2.0 * np.pi * freq * mult * ramp) / SR
        sig += amp * np.sin(phase)
    env = _envelope(t, attack, tau)
    if tremolo_hz > 0.0:
        env = env * (1.0 - tremolo_depth * 0.5
                     * (1.0 - np.cos(2.0 * np.pi * tremolo_hz * t)))
    return sig * env


def _mix(canvas: np.ndarray, signal: np.ndarray, at: float) -> None:
    """Add ``signal`` into ``canvas`` starting at ``at`` seconds (in place)."""
    i = max(0, int(round(at * SR)))
    n = min(len(signal), len(canvas) - i)
    if n > 0:
        canvas[i:i + n] += signal[:n]


def _normalize(sig: np.ndarray, peak: float) -> np.ndarray:
    """Scale to a target peak. Cue peaks differ on purpose — see the docstring."""
    high = float(np.max(np.abs(sig))) if len(sig) else 0.0
    return sig * (peak / high) if high > 0.0 else sig


# ── The cues ─────────────────────────────────────────────────────────────────
def cue_engaged() -> np.ndarray:
    """Hand enters: an ascending A major triad — noticed, and invited to draw."""
    out = _canvas(1.35)
    for i, semis in enumerate((0, 4, 7)):        # A4 → C#5 → E5
        # The notes overlap, so the arpeggio blooms into a held chord instead
        # of reading as three separate beeps.
        _mix(out, _tone(_hz(semis), 1.1, BELL, tau=0.34, attack=0.012)
             * (0.55 + 0.15 * i), 0.13 * i)
    return _normalize(out, 0.50)


def cue_acknowledged() -> np.ndarray:
    """Hand leaves: the same notes descending to the root — received, working."""
    out = _canvas(2.5)
    for i, semis in enumerate((7, 4, 0)):        # E5 → C#5 → A4
        last = i == 2
        _mix(out, _tone(_hz(semis), 1.9 if last else 0.9, SOFT,
                        tau=0.85 if last else 0.30, attack=0.012,
                        # The pulse on the final note is the "still thinking"
                        # part: an unmodulated tone would sound finished.
                        tremolo_hz=5.5 if last else 0.0, tremolo_depth=0.35)
             * 0.60, 0.17 * i)
    _mix(out, _tone(_hz(-12), 2.0, SOFT, tau=1.0, attack=0.05) * 0.30, 0.34)
    return _normalize(out, 0.50)


def cue_anticipation() -> np.ndarray:
    """Saving + running: an accelerating pentatonic rise — something is coming."""
    out = _canvas(2.6)
    notes = (0, 2, 4, 7, 9)                       # A B C# E F# (A pentatonic)
    onsets = (0.0, 0.200, 0.375, 0.525, 0.650)    # gaps shrink: 200/175/150/125 ms
    for i, (semis, at) in enumerate(zip(notes, onsets)):
        last = i == len(notes) - 1
        _mix(out, _tone(_hz(semis), 1.9 if last else 0.7, BELL,
                        tau=0.75 if last else 0.22, attack=0.006)
             * (0.34 + 0.10 * i), at)             # …and each note is louder
    _mix(out, _tone(_hz(21), 1.6, PURE, tau=0.60, attack=0.02) * 0.16, 0.65)
    # A low tone swelling underneath the whole run (attack = crescendo).
    _mix(out, _tone(_hz(-24), 2.3, SOFT, tau=6.0, attack=0.90) * 0.30, 0.0)
    return _normalize(out, 0.62)


def cue_alarm() -> np.ndarray:
    """Refused: a two-tone tritone klaxon. Harsh, urgent, and the loudest cue."""
    honk, gap, count = 0.30, 0.07, 4
    out = _canvas(count * (honk + gap) + 0.35)
    rng = np.random.default_rng(7)                # seeded: cues are reproducible
    for i in range(count):
        semis = -7 if i % 2 == 0 else -13         # D4 ↔ G#3 — a tritone apart
        sig = _tone(_hz(semis), honk, SAW, tau=1.6, attack=0.004,
                    tremolo_hz=7.0, tremolo_depth=0.45, bend=0.97)
        sig += _tone(_hz(semis - 12), honk, PURE, tau=1.6, attack=0.004) * 0.5
        # Multiplicative grit: roughness that follows the note's own envelope,
        # so the gaps between honks stay silent.
        sig = sig * (1.0 + 0.05 * rng.normal(0.0, 1.0, len(sig)))
        _mix(out, sig, i * (honk + gap))
    return _normalize(out, 0.92)


CUES = {
    "engaged": cue_engaged,
    "acknowledged": cue_acknowledged,
    "anticipation": cue_anticipation,
    "alarm": cue_alarm,
}


# ── Files ────────────────────────────────────────────────────────────────────
def write_wav(path: Path, samples: np.ndarray) -> Path:
    """Write mono 16-bit PCM at SR. Anything over full scale is clipped."""
    pcm = (np.clip(samples, -1.0, 1.0) * 32767.0).astype("<i2")
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(pcm.tobytes())
    return path


def render_all(dest: Path | str = SOUNDS_DIR) -> list[Path]:
    """Render every cue into ``dest`` as ``<name>.wav``. Returns the paths."""
    folder = Path(dest)
    return [write_wav(folder / f"{name}.wav", build())
            for name, build in CUES.items()]


if __name__ == "__main__":
    import sys

    target = Path(sys.argv[1]) if len(sys.argv) > 1 else SOUNDS_DIR
    for out in render_all(target):
        seconds = out.stat().st_size / (SR * 2)
        print(f"  {out}  ({seconds:.2f} s, {out.stat().st_size / 1024:.0f} KB)")
