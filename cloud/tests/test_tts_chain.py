"""Mock TTS -> resample to 8 kHz -> IMA ADPCM encode -> decode round trip
(docs/link-protocol.md §3, §7: "synthesize -> resample 8 kHz mono -> IMA ADPCM
encode ... same codec as common/, locked by shared vectors").

Uses the exact helpers `strategy.py` uses in production (`_encode_stream`,
`chunk_adpcm`) so this test tracks the real pipeline, and decodes with
powersuit_proto.adpcm — the shared codec, never reimplemented here.
"""

from __future__ import annotations

import pytest
from powersuit_proto import adpcm

from cloud_ai_core.strategy import TTS_CHUNK_BYTES, _encode_stream, chunk_adpcm
from cloud_ai_core.tts.mock_tts import MockTts
from cloud_ai_core.tts.resample import resample_pcm16


def _padded_sample_count(n_samples: int) -> int:
    return n_samples + (-n_samples) % 16


class TestChainRoundTrip:
    async def test_full_chain_nonsilent_expected_length(self) -> None:
        tts = MockTts()
        pcm16k, rate = await tts.synth("Suit nominal. Battery eighty one percent.")
        assert rate == 16_000
        assert len(pcm16k) > 0

        pcm8k = resample_pcm16(pcm16k, rate, 8_000)
        n_samples = len(pcm8k) // 2
        expected_samples = _padded_sample_count(n_samples)

        encoded = _encode_stream(pcm8k)
        assert len(encoded) == expected_samples // 2  # 2 samples/byte, 4 bits each
        assert len(encoded) % 8 == 0  # link-protocol §3: multiple of 8 bytes

        decoded = adpcm.decode(adpcm.AdpcmState(), encoded)
        assert len(decoded) == expected_samples

        peak = max(abs(s) for s in decoded)
        assert peak > 500, "decoded audio should clearly not be silence"
        # A meaningful fraction of samples should be far from zero (not just a
        # brief transient) — MockTts amplitude is 11000.
        loud = sum(1 for s in decoded if abs(s) > 1000)
        assert loud > len(decoded) // 10

    async def test_chain_is_deterministic(self) -> None:
        tts = MockTts()
        text = "Cell group three running warm."
        pcm_a, rate_a = await tts.synth(text)
        pcm_b, rate_b = await tts.synth(text)
        assert (pcm_a, rate_a) == (pcm_b, rate_b)

        enc_a = _encode_stream(resample_pcm16(pcm_a, rate_a, 8_000))
        enc_b = _encode_stream(resample_pcm16(pcm_b, rate_b, 8_000))
        assert enc_a == enc_b

    async def test_empty_text_still_produces_short_nonempty_stream(self) -> None:
        tts = MockTts()
        pcm, rate = await tts.synth("")
        assert len(pcm) > 0  # MockTts falls back to a short silence, not nothing
        encoded = _encode_stream(resample_pcm16(pcm, rate, 8_000))
        assert len(encoded) % 8 == 0


class TestChunking:
    async def test_chunks_are_tts_chunk_bytes_except_last(self) -> None:
        tts = MockTts()
        pcm, rate = await tts.synth("Status report. All systems nominal. Battery good.")
        encoded = _encode_stream(resample_pcm16(pcm, rate, 8_000))
        chunks = chunk_adpcm(encoded, TTS_CHUNK_BYTES)
        assert len(chunks) >= 1
        for chunk in chunks[:-1]:
            assert len(chunk) == TTS_CHUNK_BYTES
        assert 0 < len(chunks[-1]) <= TTS_CHUNK_BYTES
        assert b"".join(chunks) == encoded

    def test_chunk_size_must_be_multiple_of_8(self) -> None:
        with pytest.raises(ValueError):
            chunk_adpcm(b"\x00" * 32, chunk_bytes=5)
