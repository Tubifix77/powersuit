"""PCM16 mono linear resampler (pure numpy; link-protocol §7 wants 8 kHz out)."""

from __future__ import annotations

import numpy as np

TARGET_RATE = 8_000


def resample_pcm16(pcm: bytes, src_rate: int, dst_rate: int = TARGET_RATE) -> bytes:
    if src_rate <= 0 or dst_rate <= 0:
        raise ValueError("rates must be positive")
    samples = np.frombuffer(pcm, dtype="<i2")
    if src_rate == dst_rate or samples.size == 0:
        return bytes(pcm)
    n_dst = max(int(round(samples.size * dst_rate / src_rate)), 1)
    # Sample positions of the destination grid expressed on the source grid.
    x_dst = np.arange(n_dst, dtype=np.float64) * (src_rate / dst_rate)
    x_src = np.arange(samples.size, dtype=np.float64)
    out = np.interp(x_dst, x_src, samples.astype(np.float64))
    return np.clip(np.rint(out), -32768, 32767).astype("<i2").tobytes()
