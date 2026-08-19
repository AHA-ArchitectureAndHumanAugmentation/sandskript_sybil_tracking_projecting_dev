"""
Unit tests for text_guard.py — the Participant-Mode profanity guard.

The matching half (normalize / find_profanity / load_wordlists) is pure text
work and always runs. The OCR half needs Tesseract from the conda env, so those
tests skip themselves when the engine is unavailable rather than failing on a
machine that has not installed it.
"""
import numpy as np
import pytest

import text_guard
from text_guard import (
    GuardVerdict,
    check_mask,
    engine_available,
    find_profanity,
    load_wordlists,
    normalize,
)

WORDS = {"shit", "fuck", "arsch", "scheisse", "ass"}

needs_ocr = pytest.mark.skipif(
    not engine_available(), reason="Tesseract not available in this environment"
)


# ─────────────────────────────────────────────────────────────────────────────
# normalize
# ─────────────────────────────────────────────────────────────────────────────

class TestNormalize:

    def test_lowercases(self):
        assert normalize("FUCK") == "fuck"

    def test_folds_german_eszett(self):
        assert normalize("Scheiße") == "scheisse"

    def test_folds_umlauts(self):
        assert normalize("MÖSE") == "moese"
        assert normalize("Ärsch") == "aersch"

    def test_strips_accents(self):
        assert normalize("café") == "cafe"

    def test_folds_leetspeak(self):
        assert normalize("5H1T") == "shit"
        assert normalize("@ss") == "ass"


# ─────────────────────────────────────────────────────────────────────────────
# find_profanity
# ─────────────────────────────────────────────────────────────────────────────

class TestFindProfanity:

    def test_clean_text_returns_none(self):
        assert find_profanity("hello world", WORDS) is None

    def test_empty_inputs_return_none(self):
        assert find_profanity("", WORDS) is None
        assert find_profanity("shit", set()) is None

    def test_exact_word_match(self):
        assert find_profanity("oh shit", WORDS) == "shit"

    def test_case_and_leetspeak_still_match(self):
        assert find_profanity("5H1T", WORDS) == "shit"

    def test_german_eszett_match(self):
        assert find_profanity("SCHEISSE", WORDS) == "scheisse"
        assert find_profanity("Scheiße", WORDS) == "scheisse"

    def test_broken_spacing_still_matches(self):
        """OCR of sand writing splits words — the de-spaced pass catches it."""
        assert find_profanity("f u c k", WORDS) == "fuck"

    def test_short_entry_does_not_match_as_substring(self):
        """'ass' is 3 letters, below the substring floor — no Scunthorpe."""
        assert find_profanity("assist the classic", WORDS) is None

    def test_short_entry_still_matches_standalone(self):
        assert find_profanity("you ass", WORDS) == "ass"

    def test_substring_floor_is_configurable(self):
        assert find_profanity("assist", WORDS, min_substring_len=3) == "ass"

    def test_punctuation_and_newlines_ignored(self):
        assert find_profanity("well...\n  shit!\n", WORDS) == "shit"

    def test_longest_match_preferred(self):
        """'scheisse' wins over a shorter entry contained in the same text."""
        assert find_profanity("scheisse", {"scheisse", "eiss"}) == "scheisse"


# ─────────────────────────────────────────────────────────────────────────────
# load_wordlists
# ─────────────────────────────────────────────────────────────────────────────

class TestLoadWordlists:

    def test_ships_a_seed_list(self):
        words = load_wordlists()
        assert len(words) > 20
        assert "fuck" in words          # en seed
        assert "arschloch" in words     # de seed

    def test_entries_are_normalized(self):
        assert all(w.isalpha() and w.islower() for w in load_wordlists())

    def test_reads_any_txt_and_skips_comments(self, tmp_path):
        (tmp_path / "custom.txt").write_text(
            "# a comment\n\nBadWord\ntrailing  # inline\n", encoding="utf-8")
        words = load_wordlists(str(tmp_path))
        assert "badword" in words
        assert "trailing" in words
        assert not any("comment" in w for w in words)

    def test_missing_directory_is_empty_not_an_error(self, tmp_path):
        assert load_wordlists(str(tmp_path / "nope")) == frozenset()


# ─────────────────────────────────────────────────────────────────────────────
# check_mask — degradation paths (no OCR engine needed)
# ─────────────────────────────────────────────────────────────────────────────

class TestCheckMaskDegradation:

    def test_none_mask_is_unavailable_not_profane(self):
        v = check_mask(None)
        assert isinstance(v, GuardVerdict)
        assert v.available is False and v.profane is False

    def test_empty_mask_is_unavailable_not_profane(self):
        v = check_mask(np.zeros((0, 0), np.uint8))
        assert v.available is False and v.profane is False

    def test_missing_engine_lets_the_drawing_through(self, monkeypatch, tmp_path):
        """No Tesseract must never block a drawing — the pipeline keeps running."""
        monkeypatch.setattr(text_guard, "_ensure_engine", lambda: False)
        v = check_mask(np.zeros((100, 100), np.uint8))
        assert v.available is False and v.profane is False
        assert "unavailable" in v.reason

    def test_empty_wordlist_lets_the_drawing_through(self, monkeypatch, tmp_path):
        monkeypatch.setattr(text_guard, "_ensure_engine", lambda: True)
        v = check_mask(np.zeros((100, 100), np.uint8), wordlist_dir=str(tmp_path))
        assert v.available is False and v.profane is False
        assert "wordlist" in v.reason

    def test_ocr_exception_lets_the_drawing_through(self, monkeypatch):
        monkeypatch.setattr(text_guard, "_ensure_engine", lambda: True)
        monkeypatch.setattr(text_guard, "_read_text",
                            lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
        v = check_mask(np.zeros((100, 100), np.uint8))
        assert v.available is False and v.profane is False

    def test_verdict_carries_the_matched_word(self, monkeypatch):
        monkeypatch.setattr(text_guard, "_ensure_engine", lambda: True)
        monkeypatch.setattr(text_guard, "_read_text", lambda *a, **k: ["oh FUCK"])
        v = check_mask(np.zeros((100, 100), np.uint8))
        assert v.profane is True and v.word == "fuck"
        assert "fuck" in v.reason

    def test_clean_reading_passes(self, monkeypatch):
        monkeypatch.setattr(text_guard, "_ensure_engine", lambda: True)
        monkeypatch.setattr(text_guard, "_read_text", lambda *a, **k: ["hello welt"])
        v = check_mask(np.zeros((100, 100), np.uint8))
        assert v.profane is False and v.available is True

    def test_any_rotation_reading_can_trip_it(self, monkeypatch):
        """A hit in the upside-down pass counts just as much as the upright one."""
        monkeypatch.setattr(text_guard, "_ensure_engine", lambda: True)
        monkeypatch.setattr(text_guard, "_read_text",
                            lambda *a, **k: ["gibberish", "SHIT"])
        v = check_mask(np.zeros((100, 100), np.uint8))
        assert v.profane is True and v.word == "shit"


# ─────────────────────────────────────────────────────────────────────────────
# check_mask — real OCR (skipped without Tesseract)
# ─────────────────────────────────────────────────────────────────────────────

def _text_mask(word: str, rotate: bool = False) -> np.ndarray:
    """A mask that looks like the real thing: thick white strokes on black."""
    import cv2
    img = np.zeros((240, 900), np.uint8)
    cv2.putText(img, word, (30, 160), cv2.FONT_HERSHEY_SIMPLEX, 3.0, 255, 14)
    return np.rot90(img, 2).copy() if rotate else img


@needs_ocr
class TestCheckMaskWithOcr:

    def test_blank_mask_is_clean(self):
        v = check_mask(np.zeros((240, 900), np.uint8))
        assert v.profane is False

    def test_innocuous_word_passes(self):
        assert check_mask(_text_mask("HELLO")).profane is False

    def test_profane_english_is_caught(self):
        v = check_mask(_text_mask("FUCK"))
        assert v.profane is True and v.word == "fuck"

    def test_profane_german_is_caught(self):
        v = check_mask(_text_mask("ARSCHLOCH"))
        assert v.profane is True and v.word == "arschloch"

    def test_upside_down_writing_is_caught(self):
        """Someone on the far side of the sandbox writes rotated 180deg."""
        v = check_mask(_text_mask("FUCK", rotate=True))
        assert v.profane is True and v.word == "fuck"
