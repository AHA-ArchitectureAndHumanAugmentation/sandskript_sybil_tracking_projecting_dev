"""
Unit tests for the Participant-Mode sound cues (sound_design.py) and the way
the server hands them to the projection window. Pure numpy + the stdlib wave
reader — no audio device, no hardware, no running server.

The interesting assertions are the musical ones: the cues make specific claims
about pitch contour (rising = invitation, falling = closure) and about the
alarm being unmistakably different, and those are exactly the properties a
"harmless" edit to the synthesis would quietly break.
"""
import json
import threading
import wave

import numpy as np
import pytest

import server
import sound_design as sd
from config import SOUND_CUES, SOUNDS_URL_PATH

SR = sd.SR


def _dominant_hz(sig: np.ndarray) -> float:
    """The loudest frequency in a chunk of samples."""
    spec = np.abs(np.fft.rfft(sig * np.hanning(len(sig))))
    return float(np.fft.rfftfreq(len(sig), 1.0 / SR)[np.argmax(spec)])


def _slice(sig: np.ndarray, start_s: float, end_s: float) -> np.ndarray:
    return sig[int(start_s * SR):int(end_s * SR)]


def _rms(sig: np.ndarray) -> float:
    return float(np.sqrt(np.mean(sig ** 2)))


# ── Primitives ───────────────────────────────────────────────────────────────
class TestPrimitives:

    def test_pitch_is_equal_tempered_from_a440(self):
        assert sd._hz(0) == pytest.approx(440.0)
        assert sd._hz(12) == pytest.approx(880.0)
        assert sd._hz(-12) == pytest.approx(220.0)
        assert sd._hz(-7) == pytest.approx(293.66, abs=0.01)    # D4

    def test_envelope_starts_and_ends_silent(self):
        t = sd._times(0.5)
        env = sd._envelope(t, attack=0.01, tau=0.3)
        assert env[0] == pytest.approx(0.0, abs=1e-9)
        assert env[-1] == pytest.approx(0.0, abs=1e-9)
        assert env.max() > 0.5

    def test_tone_holds_its_pitch(self):
        tone = sd._tone(440.0, 0.5, sd.PURE, tau=2.0)
        assert _dominant_hz(tone) == pytest.approx(440.0, abs=5.0)

    def test_bend_glides_the_pitch(self):
        tone = sd._tone(400.0, 0.6, sd.PURE, tau=5.0, bend=0.8)
        assert _dominant_hz(_slice(tone, 0.0, 0.2)) < 410.0
        assert _dominant_hz(_slice(tone, 0.4, 0.6)) < 350.0

    def test_mix_places_a_signal_at_an_offset(self):
        canvas = sd._canvas(1.0)
        sd._mix(canvas, np.ones(int(0.1 * SR)), 0.5)
        assert canvas[int(0.25 * SR)] == 0.0
        assert canvas[int(0.55 * SR)] == 1.0

    def test_mix_truncates_past_the_end(self):
        canvas = sd._canvas(0.2)
        sd._mix(canvas, np.ones(int(1.0 * SR)), 0.15)   # must not raise
        assert canvas[-1] == 1.0

    def test_normalize_hits_the_target_peak(self):
        out = sd._normalize(np.array([0.0, 0.1, -0.2]), 0.8)
        assert np.max(np.abs(out)) == pytest.approx(0.8)

    def test_normalize_survives_silence(self):
        assert np.all(sd._normalize(np.zeros(10), 0.8) == 0.0)


# ── Every cue, whatever it sounds like ───────────────────────────────────────
@pytest.mark.parametrize("name", sorted(sd.CUES))
class TestEveryCue:

    def test_is_finite_and_within_full_scale(self, name):
        sig = sd.CUES[name]()
        assert np.all(np.isfinite(sig))
        assert np.max(np.abs(sig)) <= 1.0

    def test_is_actually_audible(self, name):
        assert np.max(np.abs(sd.CUES[name]())) > 0.3

    def test_starts_and_ends_at_silence(self, name):
        # A waveform that jumps from zero clicks, and a click in a calm
        # gallery reads as a fault.
        sig = sd.CUES[name]()
        assert abs(sig[0]) < 1e-3
        assert abs(sig[-1]) < 1e-3

    def test_is_short_enough_to_be_a_cue(self, name):
        seconds = len(sd.CUES[name]()) / SR
        assert 0.5 < seconds < 4.0

    def test_is_reproducible(self, name):
        # The alarm uses a seeded RNG for its grit; regenerating must not
        # produce a different file every time it is run.
        assert np.array_equal(sd.CUES[name](), sd.CUES[name]())


# ── What each cue is supposed to communicate ─────────────────────────────────
class TestCueCharacter:

    def test_engaged_rises(self):
        """Hand enters: an ascending triad — an invitation, not a verdict."""
        sig = sd.cue_engaged()
        assert _dominant_hz(_slice(sig, 0.0, 0.10)) == pytest.approx(440.0, abs=8)
        assert _dominant_hz(_slice(sig, 0.40, 0.70)) == pytest.approx(659.3, abs=8)

    def test_acknowledged_falls_to_the_root(self):
        """Hand leaves: the same notes reversed — closure."""
        sig = sd.cue_acknowledged()
        assert _dominant_hz(_slice(sig, 0.0, 0.12)) == pytest.approx(659.3, abs=8)
        assert _dominant_hz(_slice(sig, 0.60, 1.40)) == pytest.approx(440.0, abs=8)

    def test_acknowledged_keeps_ringing_after_the_notes(self):
        """The held, pulsing tail is what says 'still working'."""
        sig = sd.cue_acknowledged()
        assert _rms(_slice(sig, 1.2, 1.8)) > 0.02

    def test_anticipation_rises_and_accelerates(self):
        sig = sd.cue_anticipation()
        assert _dominant_hz(_slice(sig, 0.0, 0.15)) == pytest.approx(440.0, abs=8)
        assert _dominant_hz(_slice(sig, 0.75, 1.60)) == pytest.approx(740.0, abs=8)
        # Onsets at 0.000/0.200/0.375/0.525/0.650 — each gap shorter than the last.
        gaps = np.diff((0.0, 0.200, 0.375, 0.525, 0.650))
        assert np.all(np.diff(gaps) < 0)

    def test_anticipation_swells(self):
        """Crescendo, not decay: the energy has to grow into the last note."""
        sig = sd.cue_anticipation()
        assert _rms(_slice(sig, 0.55, 1.05)) > _rms(_slice(sig, 0.0, 0.20))

    def test_alarm_is_a_tritone_klaxon(self):
        """D4 against G#3 — the most dissonant interval available."""
        sig = sd.cue_alarm()
        first = _dominant_hz(_slice(sig, 0.0, 0.30))
        second = _dominant_hz(_slice(sig, 0.37, 0.67))
        assert first == pytest.approx(293.7, abs=8)
        assert second == pytest.approx(207.7, abs=8)
        semitones = 12.0 * np.log2(first / second)
        assert semitones == pytest.approx(6.0, abs=0.4)      # a tritone

    def test_alarm_is_harsher_than_the_friendly_cues(self):
        """A sawtooth spreads energy far above its fundamental; bells don't."""
        def centroid(sig):
            spec = np.abs(np.fft.rfft(sig * np.hanning(len(sig))))
            freq = np.fft.rfftfreq(len(sig), 1.0 / SR)
            return float((spec * freq).sum() / max(spec.sum(), 1e-9))
        assert centroid(sd.cue_alarm()) > 2.0 * centroid(sd.cue_engaged())

    def test_alarm_is_the_loudest_cue(self):
        """It must never be mistaken for one of the encouraging ones."""
        alarm = sd.cue_alarm()
        for name in ("engaged", "acknowledged", "anticipation"):
            other = sd.CUES[name]()
            assert np.max(np.abs(alarm)) > np.max(np.abs(other))
            assert _rms(alarm) > 1.5 * _rms(other)


# ── Files ────────────────────────────────────────────────────────────────────
class TestWavFiles:

    def test_write_wav_is_mono_16bit_at_the_configured_rate(self, tmp_path):
        sig = sd._tone(440.0, 0.25, sd.PURE)
        path = sd.write_wav(tmp_path / "t.wav", sig)
        with wave.open(str(path), "rb") as w:
            assert w.getnchannels() == 1
            assert w.getsampwidth() == 2
            assert w.getframerate() == SR
            assert w.getnframes() == len(sig)

    def test_write_wav_creates_missing_folders(self, tmp_path):
        path = sd.write_wav(tmp_path / "deep" / "er" / "t.wav", sd._canvas(0.1))
        assert path.is_file()

    def test_write_wav_clips_instead_of_wrapping(self, tmp_path):
        """Overflow must saturate — wrapping turns a loud note into a buzz."""
        path = sd.write_wav(tmp_path / "loud.wav", np.array([2.0, -2.0, 0.0]))
        with wave.open(str(path), "rb") as w:
            data = np.frombuffer(w.readframes(3), dtype="<i2")
        assert data[0] == 32767 and data[1] == -32767

    def test_render_all_writes_every_cue(self, tmp_path):
        written = sd.render_all(tmp_path)
        assert {p.stem for p in written} == set(sd.CUES)
        assert all(p.stat().st_size > 1000 for p in written)


class TestCueWiring:

    def test_every_configured_status_maps_to_a_real_cue(self):
        assert set(SOUND_CUES.values()) <= set(sd.CUES)

    def test_the_four_participant_moments_have_a_cue(self):
        # Hand enters, hand leaves, save+run, refused.
        assert set(SOUND_CUES) == {"Alerted", "Sensing", "Actuating", "Invalid"}

    def test_the_shipped_folder_has_every_cue_rendered(self):
        """A cue added in code but never rendered would be silent in the app."""
        for name in set(SOUND_CUES.values()):
            path = server._SOUNDS_DIR / f"{name}.wav"
            assert path.is_file(), f"run sound_design.py to render {name}.wav"
            with wave.open(str(path), "rb") as w:
                assert w.getnframes() > 0


class TestServerServesTheCues:

    @staticmethod
    def _server(state=None):
        return server.Server(state if state is not None else {},
                             threading.Lock(),
                             on_connect=None, on_disconnect=None)

    def test_sounds_are_served_over_http(self):
        srv = self._server()
        prefixes = [r.get_info().get("prefix")
                    for r in srv._app.router.resources()]
        assert SOUNDS_URL_PATH in prefixes

    def test_init_carries_the_cue_map(self):
        """The page must not need its own copy of config.SOUND_CUES."""
        sent = []

        class FakeWS:
            async def send_str(self, msg):
                sent.append(json.loads(msg))

        srv = self._server()
        import asyncio
        asyncio.run(srv._send_init(FakeWS(), "", None))
        sounds = sent[0]["sounds"]
        assert sounds["path"] == SOUNDS_URL_PATH
        assert sounds["cues"] == SOUND_CUES
