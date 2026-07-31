"""Engine protocol (link-protocol §7 pipeline: RAG docs + prompt -> answer text)."""

from __future__ import annotations

from typing import Protocol, runtime_checkable


class EngineError(Exception):
    pass


class EngineUnavailable(EngineError):
    """The backing inference endpoint cannot be reached."""


@runtime_checkable
class EngineBase(Protocol):
    async def generate(self, system: str, user: str, context_docs: list[str]) -> str:
        """Produce the answer text for a voice query."""
        ...
