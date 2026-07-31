"""Rolling suit state built from telemetry_batch payloads (link-protocol §2.3)."""

from __future__ import annotations

import time
from typing import Any


class RollingState:
    """Latest-value view over the coalesced 5 Hz telemetry batches."""

    def __init__(self) -> None:
        self.sources: dict[str, Any] = {}
        self.window_ms: int = 0
        self.batches: int = 0
        self.dropped_last: int = 0
        self.dropped_total: int = 0
        self._updated_mono: float | None = None

    def update(self, payload: dict[str, Any]) -> None:
        sources = payload.get("sources")
        if isinstance(sources, dict):
            self.sources.update(sources)
        self.window_ms = int(payload.get("window_ms", self.window_ms) or 0)
        dropped = payload.get("dropped_count", 0)
        self.dropped_last = int(dropped) if isinstance(dropped, (int, float)) else 0
        self.dropped_total += self.dropped_last
        self.batches += 1
        self._updated_mono = time.monotonic()

    # --- views ---------------------------------------------------------------

    def staleness_s(self) -> float | None:
        if self._updated_mono is None:
            return None
        return time.monotonic() - self._updated_mono

    def _power(self) -> dict[str, Any]:
        p = self.sources.get("power")
        return p if isinstance(p, dict) else {}

    def _safety(self) -> dict[str, Any]:
        s = self.sources.get("safety")
        return s if isinstance(s, dict) else {}

    @property
    def soc(self) -> float | None:
        v = self._power().get("soc")
        return float(v) if isinstance(v, (int, float)) else None

    @property
    def temp_max(self) -> float | None:
        v = self._power().get("t_max")
        return float(v) if isinstance(v, (int, float)) else None

    @property
    def fault_bits(self) -> int:
        v = self._power().get("faults")
        return int(v) if isinstance(v, (int, float)) else 0

    @property
    def estop(self) -> bool:
        return bool(self._safety().get("estop", False))

    @property
    def safety_state(self) -> str:
        v = self._safety().get("state")
        return v if isinstance(v, str) else "UNKNOWN"

    def facts(self) -> str:
        """One-line human summary used as LLM context (deterministic)."""
        parts: list[str] = [f"state={self.safety_state}"]
        if self.soc is not None:
            parts.append(f"soc={self.soc:.0f}%")
        if self.temp_max is not None:
            parts.append(f"temp_max={self.temp_max:.0f}C")
        if self.estop:
            parts.append("ESTOP LATCHED")
        return " ".join(parts)
