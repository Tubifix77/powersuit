"""Shared deterministic signal helper for the audio scenarios."""
from __future__ import annotations

import math


def tone(n: int, freq: float = 440.0, amp: int = 9000, rate: int = 8000) -> list[int]:
    return [int(amp * math.sin(2 * math.pi * freq * i / rate)) for i in range(n)]


def is_audible(pcm: list[int], floor: int = 200) -> bool:
    if not pcm:
        return False
    peak = max(abs(s) for s in pcm)
    return peak > floor
