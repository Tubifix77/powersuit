"""LLM engines: deterministic mock and OpenAI-compatible HTTP client."""

from .engine import EngineBase, EngineError, EngineUnavailable

__all__ = ["EngineBase", "EngineError", "EngineUnavailable"]
