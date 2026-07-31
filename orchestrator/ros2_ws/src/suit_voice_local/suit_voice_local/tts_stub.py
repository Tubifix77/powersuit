"""Acknowledgment tone synthesizer — PURE python (no rclpy, no numpy).

Real speech arrives from Node 9 TTS (docs/link-protocol.md §7); this stub
guarantees an offline acknowledgment for every locally-handled intent
(ARCHITECTURE.md offline-first). Output: PCM16LE mono @ 8 kHz bytes, ready to be
wrapped in suit_msgs/AudioChunk codec=2.
"""

from __future__ import annotations

import math
import struct

SAMPLE_RATE = 8000
_AMP = 9000  # ~-11 dBFS, comfortable in-helmet level

# Per-intent distinct beep patterns: list of (freq_hz, duration_ms); 0 Hz = rest.
BEEP_PATTERNS: dict[str, list[tuple[int, int]]] = {
    "status_report": [(880, 90), (0, 30), (1175, 90)],
    "power_level": [(660, 90), (0, 30), (660, 90)],
    "deploy_airbrakes": [(523, 70), (659, 70), (784, 110)],       # rising triad
    "retract_airbrakes": [(784, 70), (659, 70), (523, 110)],      # falling triad
    "engage_estop": [(1568, 120), (0, 40), (1568, 120), (0, 40), (1568, 200)],
    "clear_estop": [(392, 120), (523, 180)],
    "hud_brightness": [(988, 60), (0, 20), (988, 60)],
    "cloud_query": [(740, 60), (0, 40), (740, 60), (0, 40), (740, 60)],  # "thinking"
}
_DEFAULT_PATTERN = [(700, 80), (0, 40), (700, 80)]


def tone(freq_hz: float, duration_ms: int, amp: int = _AMP,
         sample_rate: int = SAMPLE_RATE) -> bytes:
    """One sine burst (or silence for freq 0) as PCM16LE bytes."""
    n = max(0, int(sample_rate * duration_ms / 1000))
    if freq_hz <= 0:
        return b"\x00\x00" * n
    out = bytearray()
    w = 2.0 * math.pi * freq_hz / sample_rate
    for i in range(n):
        # 5 ms linear fade in/out to avoid clicks.
        env = min(1.0, i / (0.005 * sample_rate), (n - 1 - i) / (0.005 * sample_rate) + 1e-9)
        out += struct.pack("<h", int(amp * env * math.sin(w * i)))
    return bytes(out)


def synth_ack(intent: str, word_count: int = 1) -> bytes:
    """Acknowledgment tones for an intent; length grows with the spoken word count
    (a longer command earns a longer trailing confirmation tone)."""
    pattern = BEEP_PATTERNS.get(intent, _DEFAULT_PATTERN)
    pcm = bytearray()
    for freq, ms in pattern:
        pcm += tone(freq, ms)
    extra_ms = min(400, 40 * max(0, word_count - 1))
    if extra_ms and pattern:
        last_freq = next((f for f, _ in reversed(pattern) if f > 0), 700)
        pcm += tone(0, 30) + tone(last_freq, extra_ms)
    return bytes(pcm)
