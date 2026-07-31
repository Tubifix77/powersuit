import json
import math
import pathlib

from powersuit_proto.adpcm import AdpcmState, decode, encode

VECTORS = pathlib.Path(__file__).parent / "vectors" / "adpcm_ref.json"


def _test_signal(n: int) -> list[int]:
    # Deterministic voice-band-ish signal: two sines + LCG noise (mirrors gen_vectors.py).
    out = []
    lcg = 0x2A5F_11E7
    for i in range(n):
        lcg = (lcg * 1103515245 + 12345) & 0x7FFF_FFFF
        noise = (lcg >> 16) % 1201 - 600
        s = (
            int(9000 * math.sin(2 * math.pi * 233.0 * i / 8000.0))
            + int(4000 * math.sin(2 * math.pi * 941.0 * i / 8000.0))
            + noise
        )
        out.append(max(-32768, min(32767, s)))
    return out


def _snr_db(ref: list[int], test: list[int]) -> float:
    sig = sum(r * r for r in ref)
    err = sum((r - t) ** 2 for r, t in zip(ref, test))
    if err == 0:
        return float("inf")
    return 10.0 * math.log10(sig / err)


def test_roundtrip_snr():
    pcm = _test_signal(2048)
    dec = decode(AdpcmState(), encode(AdpcmState(), pcm))
    assert _snr_db(pcm, dec) > 20.0


def test_chunked_equals_oneshot():
    pcm = _test_signal(1024)
    one = encode(AdpcmState(), pcm)
    st = AdpcmState()
    chunks = b"".join(encode(st, pcm[i:i + 16]) for i in range(0, len(pcm), 16))
    assert chunks == one


def test_decoder_state_resync():
    # A decoder joining mid-stream with state from an AudioSync frame must produce
    # identical output to one that decoded from the start.
    pcm = _test_signal(800)
    st_enc = AdpcmState()
    first = encode(st_enc, pcm[:400])
    sync_pred, sync_idx = st_enc.predictor, st_enc.step_index
    second = encode(st_enc, pcm[400:])

    ref = decode(AdpcmState(), first + second)[400:]
    late = decode(AdpcmState(predictor=sync_pred, step_index=sync_idx), second)
    assert late == ref


def test_vectors_locked():
    data = json.loads(VECTORS.read_text())
    pcm = data["adpcm"]["pcm"]
    st = AdpcmState()
    enc = encode(st, pcm)
    assert enc.hex() == data["adpcm"]["encoded_hex"]
    assert st.predictor == data["adpcm"]["final_predictor"]
    assert st.step_index == data["adpcm"]["final_step_index"]
    dec = decode(AdpcmState(), bytes.fromhex(data["adpcm"]["encoded_hex"]))
    assert dec == data["adpcm"]["decoded"]
