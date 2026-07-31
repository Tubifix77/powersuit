"""Deterministic mock TTS: an honest, audible tone-sequence placeholder.

Per sentence, each word becomes a short tone burst: duration follows word length
(so utterance length tracks word count) and pitch steps follow the word's vowels.
16 kHz sine with short fades — clearly a placeholder, clearly non-silent, and
bit-identical across runs.
"""

from __future__ import annotations

import re

import numpy as np

SAMPLE_RATE = 16_000
_AMPLITUDE = 11_000
_VOWELS = "aeiou"
# Pitch per vowel (roughly a pentatonic step ladder).
_VOWEL_HZ = {"a": 220.0, "e": 262.0, "i": 330.0, "o": 392.0, "u": 440.0}
_WORD_GAP_S = 0.04
_SENTENCE_GAP_S = 0.18
_FADE_S = 0.005


def _tone(freq: float, dur_s: float) -> np.ndarray:
    n = max(int(round(dur_s * SAMPLE_RATE)), 32)
    t = np.arange(n, dtype=np.float64) / SAMPLE_RATE
    wave = np.sin(2.0 * np.pi * freq * t)
    fade_n = min(int(_FADE_S * SAMPLE_RATE), n // 2)
    if fade_n > 0:
        ramp = np.linspace(0.0, 1.0, fade_n)
        wave[:fade_n] *= ramp
        wave[-fade_n:] *= ramp[::-1]
    return wave


def _silence(dur_s: float) -> np.ndarray:
    return np.zeros(int(round(dur_s * SAMPLE_RATE)), dtype=np.float64)


class MockTts:
    async def synth(self, text: str) -> tuple[bytes, int]:
        sentences = [s.strip() for s in re.split(r"[.!?]+", text) if s.strip()]
        pieces: list[np.ndarray] = []
        for sentence in sentences:
            words = re.findall(r"[A-Za-z0-9']+", sentence)
            for word in words:
                vowels = [c for c in word.lower() if c in _VOWELS] or ["e"]
                word_dur = min(0.06 + 0.018 * len(word), 0.24)
                seg_dur = word_dur / len(vowels)
                for v in vowels:
                    pieces.append(_tone(_VOWEL_HZ[v], seg_dur))
                pieces.append(_silence(_WORD_GAP_S))
            pieces.append(_silence(_SENTENCE_GAP_S))
        if not pieces:
            pieces.append(_silence(0.1))
        pcm = np.concatenate(pieces)
        samples = np.clip(pcm * _AMPLITUDE, -32768, 32767).astype("<i2")
        return samples.tobytes(), SAMPLE_RATE
