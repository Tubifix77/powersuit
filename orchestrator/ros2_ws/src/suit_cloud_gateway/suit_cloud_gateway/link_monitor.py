"""Bearer selection with hysteresis — PURE python (docs/link-protocol.md §5).

healthy := connected AND rtt_ms < bearer budget AND loss_pct < 5.
Switch away when the active bearer has been unhealthy for >= 3 s; switch back when
a higher-priority bearer has been continuously healthy for >= 10 s. Decisions carry
a make-before-break intent flag: keep the old socket until hello_ack on the new one.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

DEFAULT_RTT_BUDGET_MS = {"5g": 150.0, "wifi": 80.0, "sat": 1200.0}
DEFAULT_PRIORITY = {"5g": 1, "wifi": 2, "sat": 3}
LOSS_LIMIT_PCT = 5.0
UNHEALTHY_SWITCH_S = 3.0
PREEMPT_SWITCH_S = 10.0


@dataclass(frozen=True)
class Bearer:
    name: str
    url: str
    priority: int               # lower wins
    rtt_budget_ms: float


@dataclass
class _State:
    connected: bool = False
    rtt_ms: float | None = None
    loss_pct: float = 0.0
    healthy: bool = False
    healthy_since: float | None = None
    unhealthy_since: float | None = None
    ever_updated: bool = False


@dataclass(frozen=True)
class Decision:
    active: str | None
    switch_to: str | None
    make_before_break: bool
    reason: str


def load_bearers(path: str | Path) -> tuple[list[Bearer], dict]:
    """Parse links.yaml; returns (bearers, extras like ca_bundle)."""
    import yaml
    with open(path, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    bearers = []
    for entry in doc.get("bearers", []):
        name = str(entry["name"])
        bearers.append(Bearer(
            name=name,
            url=str(entry["url"]),
            priority=int(entry.get("priority", DEFAULT_PRIORITY.get(name, 99))),
            rtt_budget_ms=float(entry.get("rtt_budget_ms",
                                          DEFAULT_RTT_BUDGET_MS.get(name, 500.0))),
        ))
    extras = {k: v for k, v in doc.items() if k != "bearers"}
    return bearers, extras


class LinkMonitor:
    def __init__(self, bearers: list[Bearer],
                 unhealthy_switch_s: float = UNHEALTHY_SWITCH_S,
                 preempt_switch_s: float = PREEMPT_SWITCH_S,
                 loss_limit_pct: float = LOSS_LIMIT_PCT,
                 clock: Callable[[], float] = time.monotonic):
        if not bearers:
            raise ValueError("at least one bearer required")
        self._bearers = {b.name: b for b in bearers}
        self._order = sorted(bearers, key=lambda b: b.priority)
        self._state = {b.name: _State() for b in bearers}
        self._unhealthy_s = unhealthy_switch_s
        self._preempt_s = preempt_switch_s
        self._loss_limit = loss_limit_pct
        self._clock = clock
        self._active: str | None = None
        self._switches = 0

    # ------------------------------------------------------------------ inputs
    def update(self, name: str, *, connected: bool,
               rtt_ms: float | None = None, loss_pct: float = 0.0) -> None:
        """Feed a probe/connection observation for one bearer."""
        b = self._bearers[name]
        st = self._state[name]
        now = self._clock()
        st.connected = connected
        st.rtt_ms = rtt_ms
        st.loss_pct = loss_pct
        st.ever_updated = True
        healthy = (connected
                   and rtt_ms is not None
                   and rtt_ms < b.rtt_budget_ms
                   and loss_pct < self._loss_limit)
        if healthy and not st.healthy:
            st.healthy_since = now
            st.unhealthy_since = None
        elif not healthy and st.healthy:
            st.unhealthy_since = now
            st.healthy_since = None
        elif not healthy and st.unhealthy_since is None:
            st.unhealthy_since = now  # first observation and it is unhealthy
        st.healthy = healthy

    # ------------------------------------------------------------------ queries
    def is_healthy(self, name: str) -> bool:
        return self._state[name].healthy

    @property
    def active(self) -> str | None:
        return self._active

    @property
    def switches(self) -> int:
        return self._switches

    def bearer(self, name: str) -> Bearer:
        return self._bearers[name]

    def stats(self, name: str) -> tuple[float | None, float]:
        st = self._state[name]
        return (st.rtt_ms, st.loss_pct)

    def _healthy_for(self, name: str, now: float) -> float:
        st = self._state[name]
        if not st.healthy or st.healthy_since is None:
            return 0.0
        return now - st.healthy_since

    def _unhealthy_for(self, name: str, now: float) -> float:
        st = self._state[name]
        if st.healthy or st.unhealthy_since is None:
            return 0.0
        return now - st.unhealthy_since

    def _best_healthy(self, exclude: str | None = None) -> str | None:
        for b in self._order:
            if b.name != exclude and self._state[b.name].healthy:
                return b.name
        return None

    # ------------------------------------------------------------------ policy
    def evaluate(self) -> Decision:
        now = self._clock()
        if self._active is None:
            cand = self._best_healthy()
            if cand:
                return Decision(None, cand, False, "initial attach")
            return Decision(None, None, False, "no healthy bearer")

        act = self._active
        if not self._state[act].healthy:
            if self._unhealthy_for(act, now) >= self._unhealthy_s:
                cand = self._best_healthy(exclude=act)
                if cand:
                    return Decision(act, cand, True,
                                    f"{act} unhealthy {self._unhealthy_s:.0f}s")
                return Decision(act, None, False, f"{act} unhealthy, no alternative")
            return Decision(act, None, False, f"{act} unhealthy, inside grace")

        act_prio = self._bearers[act].priority
        for b in self._order:
            if b.priority >= act_prio:
                break
            if self._healthy_for(b.name, now) >= self._preempt_s:
                return Decision(act, b.name, True,
                                f"{b.name} healthy {self._preempt_s:.0f}s, higher priority")
        return Decision(act, None, False, "steady")

    def commit(self, name: str) -> None:
        """Caller confirms the new bearer is up (hello_ack received)."""
        if self._active is not None and self._active != name:
            self._switches += 1
        self._active = name

    def drop_active(self) -> None:
        self._active = None
