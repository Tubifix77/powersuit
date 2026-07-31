"""Threshold rules over the rolling state -> unsolicited advisory payloads.

Each rule is fire-once-with-hysteresis: it fires when its trigger predicate goes
true while armed, then stays quiet until its reset predicate re-arms it.
Advisory ids are deterministic: r-<rule>-<nth firing>.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from powersuit_proto import wire

from ..config import RuleThresholds
from .state import RollingState

_FAULT_BIT_NAMES: tuple[tuple[int, str], ...] = (
    (wire.BMSF_SHORT_LATCH, "short_latch"),
    (wire.BMSF_OV, "ov"),
    (wire.BMSF_UV, "uv"),
    (wire.BMSF_OT, "ot"),
    (wire.BMSF_UT, "ut"),
    (wire.BMSF_OC_CHARGE, "oc_charge"),
    (wire.BMSF_OC_DISCHARGE, "oc_discharge"),
    (wire.BMSF_COMP_ARMED, "comparator_armed"),
)


"""BMSF_COMP_ARMED is a health bit, not a fault: it reports that the hardware
short-circuit comparator is armed, which is the state the pack should always be
in (docs/safety.md §4). Treating it as a fault would cry wolf on every healthy
suit — and, worse, would say nothing when the comparator is actually disarmed
and the pack has lost its sub-microsecond protection."""
_ACTUAL_FAULT_MASK = sum(bit for bit, _ in _FAULT_BIT_NAMES) & ~wire.BMSF_COMP_ARMED


def decode_fault_bits(bits: int) -> list[str]:
    return [name for bit, name in _FAULT_BIT_NAMES if bits & bit]


def decode_actual_faults(bits: int) -> list[str]:
    return decode_fault_bits(bits & _ACTUAL_FAULT_MASK)


@dataclass
class _Rule:
    name: str
    severity: str
    title: str
    trigger: Callable[[RollingState], bool]
    reset: Callable[[RollingState], bool]
    body: Callable[[RollingState], str]
    hud: dict[str, Any] | None = None
    armed: bool = True
    fired: int = 0


class RulesEngine:
    def __init__(self, thresholds: RuleThresholds | None = None):
        t = thresholds or RuleThresholds()
        self.thresholds = t
        self._rules: list[_Rule] = [
            _Rule(
                name="soc_low",
                severity="notice",
                title="Battery low",
                trigger=lambda s: s.soc is not None and s.soc < t.soc_notice,
                reset=lambda s: s.soc is not None and s.soc >= t.soc_notice + 5,
                body=lambda s: f"State of charge {s.soc:.0f}% is below {t.soc_notice}%. "
                               "Plan to land/dock and recharge.",
                hud={"icon": "battery", "ttl_s": 10},
            ),
            _Rule(
                name="soc_critical",
                severity="warning",
                title="Battery critically low",
                trigger=lambda s: s.soc is not None and s.soc < t.soc_warning,
                reset=lambda s: s.soc is not None and s.soc >= t.soc_warning + 5,
                body=lambda s: f"State of charge {s.soc:.0f}% is below {t.soc_warning}%. "
                               "Reduce load now; flight operations are not advised.",
                hud={"icon": "battery", "ttl_s": 10},
            ),
            _Rule(
                name="overtemp",
                severity="critical",
                title="Pack overtemperature",
                trigger=lambda s: s.temp_max is not None and s.temp_max >= t.temp_critical,
                reset=lambda s: s.temp_max is not None and s.temp_max < t.temp_critical - 5,
                body=lambda s: f"Maximum cell temperature {s.temp_max:.0f}C has reached the "
                               f"{t.temp_critical:.0f}C limit. Stop and let the pack cool.",
                hud={"icon": "thermal", "ttl_s": 15},
            ),
            _Rule(
                name="estop",
                severity="critical",
                title="E-stop latched",
                trigger=lambda s: s.estop,
                reset=lambda s: not s.estop,
                body=lambda s: "Suit reports E-STOP latched. Follow the clear procedure in the "
                               "operator manual once the cause is resolved.",
                hud={"icon": "estop", "ttl_s": 30},
            ),
            _Rule(
                name="telemetry_gaps",
                severity="notice",
                title="Telemetry gaps",
                trigger=lambda s: s.dropped_last > t.dropped_notice,
                reset=lambda s: s.dropped_last <= max(t.dropped_notice // 10, 5),
                body=lambda s: f"{s.dropped_last} telemetry frames dropped in the last batch "
                               "window; link quality is degraded.",
            ),
            _Rule(
                name="bms_fault",
                severity="warning",
                title="BMS fault bits set",
                trigger=lambda s: (s.fault_bits & _ACTUAL_FAULT_MASK) != 0,
                reset=lambda s: (s.fault_bits & _ACTUAL_FAULT_MASK) == 0,
                body=lambda s: "BMS reports fault bits: "
                               + ", ".join(decode_actual_faults(s.fault_bits))
                               + ". See battery care section of the manual.",
            ),
            _Rule(
                name="comparator_disarmed",
                severity="critical",
                title="Short-circuit protection disarmed",
                # Only meaningful once the pack has actually reported: an empty
                # state must not be mistaken for a disarmed comparator.
                trigger=lambda s: s.soc is not None
                and not (s.fault_bits & wire.BMSF_COMP_ARMED),
                reset=lambda s: bool(s.fault_bits & wire.BMSF_COMP_ARMED),
                body=lambda s: "The battery short-circuit comparator reports disarmed. "
                               "The pack has lost its hardware protection; firmware "
                               "cannot substitute for it. Land and power down.",
                hud={"icon": "battery", "ttl_s": 30},
            ),
        ]

    def evaluate(self, state: RollingState) -> list[dict[str, Any]]:
        """Run all rules against the current state; returns advisory payloads."""
        out: list[dict[str, Any]] = []
        for rule in self._rules:
            if rule.armed:
                if rule.trigger(state):
                    rule.armed = False
                    rule.fired += 1
                    payload: dict[str, Any] = {
                        "advisory_id": f"r-{rule.name}-{rule.fired}",
                        "severity": rule.severity,
                        "title": rule.title,
                        "body": rule.body(state),
                    }
                    if rule.hud:
                        payload["hud"] = dict(rule.hud)
                    out.append(payload)
            elif rule.reset(state):
                rule.armed = True
        return out
