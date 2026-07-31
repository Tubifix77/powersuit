"""TTS backend protocol."""

from __future__ import annotations

from typing import Protocol, runtime_checkable


class TtsUnavailable(Exception):
    """The TTS backend cannot run (missing binary/model)."""


@runtime_checkable
class TtsBase(Protocol):
    async def synth(self, text: str) -> tuple[bytes, int]:
        """Synthesize `text` -> (PCM16LE mono bytes, sample_rate)."""
        ...
