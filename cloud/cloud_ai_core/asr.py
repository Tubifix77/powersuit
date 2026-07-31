"""Server-side ASR for uplinked audio (link-protocol §2.4).

Production would run a real model; tests and the sim use MockAsr, which returns a
fixed transcript — the roundtrip under test is framing/credits/TTS, not speech.
"""

from __future__ import annotations

from powersuit_proto import adpcm

from .protocol import CODEC_ADPCM_8K

MOCK_TRANSCRIPT = "status report"


class MockAsr:
    """Deterministic ASR stand-in. Decodes ADPCM (exercising the shared codec)
    but always transcribes to MOCK_TRANSCRIPT."""

    def transcribe(self, audio: bytes, codec: int) -> str:
        if codec == CODEC_ADPCM_8K and audio:
            adpcm.decode(adpcm.AdpcmState(), audio)  # validates the byte stream decodes
        return MOCK_TRANSCRIPT
