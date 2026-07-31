"""IMA ADPCM codec — bit-identical mirror of common/c/src/adpcm_ima.c.

4 bits/sample, mono, low nibble first. Continuous-state streaming; the AUDIO SYNC
frame (wire.AudioSync) carries (predictor, step_index) for mid-stream resync.
"""

from __future__ import annotations

from dataclasses import dataclass

STEP_TABLE = (
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767,
)

INDEX_TABLE = (-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8)


def _clamp16(v: int) -> int:
    return max(-32768, min(32767, v))


@dataclass
class AdpcmState:
    predictor: int = 0
    step_index: int = 0


def _encode_sample(st: AdpcmState, sample: int) -> int:
    step = STEP_TABLE[st.step_index]
    diff = sample - st.predictor
    code = 0
    if diff < 0:
        code = 8
        diff = -diff
    vpdiff = step >> 3
    if diff >= step:
        code |= 4
        diff -= step
        vpdiff += step
    step >>= 1
    if diff >= step:
        code |= 2
        diff -= step
        vpdiff += step
    step >>= 1
    if diff >= step:
        code |= 1
        vpdiff += step
    if code & 8:
        st.predictor = _clamp16(st.predictor - vpdiff)
    else:
        st.predictor = _clamp16(st.predictor + vpdiff)
    st.step_index = max(0, min(88, st.step_index + INDEX_TABLE[code]))
    return code


def _decode_sample(st: AdpcmState, code: int) -> int:
    step = STEP_TABLE[st.step_index]
    vpdiff = step >> 3
    if code & 4:
        vpdiff += step
    if code & 2:
        vpdiff += step >> 1
    if code & 1:
        vpdiff += step >> 2
    if code & 8:
        st.predictor = _clamp16(st.predictor - vpdiff)
    else:
        st.predictor = _clamp16(st.predictor + vpdiff)
    st.step_index = max(0, min(88, st.step_index + INDEX_TABLE[code]))
    return st.predictor


def encode(st: AdpcmState, pcm: list[int] | tuple[int, ...]) -> bytes:
    if len(pcm) % 2:
        raise ValueError("sample count must be even")
    out = bytearray(len(pcm) // 2)
    for i in range(len(out)):
        lo = _encode_sample(st, pcm[2 * i])
        hi = _encode_sample(st, pcm[2 * i + 1])
        out[i] = lo | (hi << 4)
    return bytes(out)


def decode(st: AdpcmState, data: bytes | bytearray | memoryview) -> list[int]:
    out: list[int] = []
    for byte in data:
        out.append(_decode_sample(st, byte & 0xF))
        out.append(_decode_sample(st, byte >> 4))
    return out
