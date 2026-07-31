"""Threshold rules over the rolling suit state (telemetry/rules.py).

Every rule is fire-once-with-hysteresis: it fires once while armed, then stays
quiet until its reset predicate re-arms it, then can fire again. This exercises
every rule in RulesEngine plus that hysteresis behaviour, against the actual
default thresholds in config.RuleThresholds.
"""

from __future__ import annotations

from cloud_ai_core.config import RuleThresholds
from cloud_ai_core.telemetry.rules import RulesEngine, decode_fault_bits
from cloud_ai_core.telemetry.state import RollingState


def _state(**power_safety: dict) -> RollingState:
    rs = RollingState()
    payload = {"sources": {}, "window_ms": 200}
    if "power" in power_safety:
        payload["sources"]["power"] = power_safety["power"]
    if "safety" in power_safety:
        payload["sources"]["safety"] = power_safety["safety"]
    if "dropped_count" in power_safety:
        payload["dropped_count"] = power_safety["dropped_count"]
    rs.update(payload)
    return rs


def _titles(advisories: list[dict]) -> list[str]:
    return [a["title"] for a in advisories]


class TestSocLow:
    def test_fires_below_notice_threshold(self) -> None:
        engine = RulesEngine(RuleThresholds(soc_notice=20))
        out = engine.evaluate(_state(power={"soc": 19}))
        assert _titles(out) == ["Battery low"]
        assert out[0]["severity"] == "notice"
        assert out[0]["advisory_id"] == "r-soc_low-1"
        assert out[0]["hud"] == {"icon": "battery", "ttl_s": 10}

    def test_does_not_fire_at_or_above_threshold(self) -> None:
        engine = RulesEngine(RuleThresholds(soc_notice=20))
        assert engine.evaluate(_state(power={"soc": 20})) == []

    def test_fire_once_hysteresis_then_refire_after_reset(self) -> None:
        engine = RulesEngine(RuleThresholds(soc_notice=20))
        assert _titles(engine.evaluate(_state(power={"soc": 19}))) == ["Battery low"]
        # Still below threshold: must NOT refire while armed=False.
        assert engine.evaluate(_state(power={"soc": 18})) == []
        assert engine.evaluate(_state(power={"soc": 19})) == []
        # Reset requires soc >= notice + 5.
        assert engine.evaluate(_state(power={"soc": 24})) == []  # not yet reset (needs >=25)
        assert engine.evaluate(_state(power={"soc": 25})) == []  # this call rearms
        # Now a fresh dip must refire, with an incremented advisory id.
        out = engine.evaluate(_state(power={"soc": 19}))
        assert _titles(out) == ["Battery low"]
        assert out[0]["advisory_id"] == "r-soc_low-2"


class TestSocCritical:
    def test_fires_below_warning_threshold(self) -> None:
        engine = RulesEngine(RuleThresholds(soc_notice=5, soc_warning=10))
        out = engine.evaluate(_state(power={"soc": 9}))
        assert _titles(out) == ["Battery critically low"]
        assert out[0]["severity"] == "warning"
        assert "10%" in out[0]["body"]

    def test_soc_9_fires_both_notice_and_warning_independently(self) -> None:
        # A single reading below both thresholds fires both rules in one pass.
        engine = RulesEngine(RuleThresholds(soc_notice=20, soc_warning=10))
        out = engine.evaluate(_state(power={"soc": 9}))
        assert set(_titles(out)) == {"Battery low", "Battery critically low"}


class TestOvertemp:
    def test_fires_at_or_above_critical(self) -> None:
        engine = RulesEngine(RuleThresholds(temp_critical=60.0))
        out = engine.evaluate(_state(power={"t_max": 60}))
        assert _titles(out) == ["Pack overtemperature"]
        assert out[0]["severity"] == "critical"
        assert out[0]["hud"] == {"icon": "thermal", "ttl_s": 15}

    def test_hysteresis_requires_five_degree_drop_to_reset(self) -> None:
        engine = RulesEngine(RuleThresholds(temp_critical=60.0))
        engine.evaluate(_state(power={"t_max": 61}))
        assert engine.evaluate(_state(power={"t_max": 56})) == []  # not below 55, still armed=False
        assert engine.evaluate(_state(power={"t_max": 54})) == []  # this call rearms
        out = engine.evaluate(_state(power={"t_max": 60}))
        assert _titles(out) == ["Pack overtemperature"]


class TestEstop:
    def test_fires_when_latched_and_resets_when_cleared(self) -> None:
        engine = RulesEngine()
        assert engine.evaluate(_state(safety={"estop": False})) == []
        out = engine.evaluate(_state(safety={"estop": True}))
        assert _titles(out) == ["E-stop latched"]
        assert out[0]["severity"] == "critical"
        # Still latched: no refire.
        assert engine.evaluate(_state(safety={"estop": True})) == []
        # Cleared: rearms.
        assert engine.evaluate(_state(safety={"estop": False})) == []
        # Latched again: fires again.
        out2 = engine.evaluate(_state(safety={"estop": True}))
        assert _titles(out2) == ["E-stop latched"]
        assert out2[0]["advisory_id"] == "r-estop-2"


class TestTelemetryGaps:
    def test_fires_above_dropped_notice(self) -> None:
        engine = RulesEngine(RuleThresholds(dropped_notice=50))
        out = engine.evaluate(_state(dropped_count=51))
        assert _titles(out) == ["Telemetry gaps"]
        assert out[0]["severity"] == "notice"

    def test_reset_threshold_is_a_tenth_of_notice_floor_5(self) -> None:
        engine = RulesEngine(RuleThresholds(dropped_notice=50))
        engine.evaluate(_state(dropped_count=51))
        assert engine.evaluate(_state(dropped_count=10)) == []  # 5 < 10, still armed=False
        assert engine.evaluate(_state(dropped_count=5)) == []  # <=5 rearms
        out = engine.evaluate(_state(dropped_count=60))
        assert _titles(out) == ["Telemetry gaps"]

    def test_small_dropped_notice_floors_reset_at_5(self) -> None:
        # dropped_notice // 10 would be 0 for dropped_notice=5; the rule floors at 5.
        engine = RulesEngine(RuleThresholds(dropped_notice=5))
        engine.evaluate(_state(dropped_count=6))
        assert engine.evaluate(_state(dropped_count=5)) == []  # rearms (<=5)


class TestBmsFault:
    def test_fires_when_any_fault_bit_set(self) -> None:
        from powersuit_proto import wire

        engine = RulesEngine()
        out = engine.evaluate(_state(power={"faults": wire.BMSF_OV}))
        assert _titles(out) == ["BMS fault bits set"]
        assert out[0]["severity"] == "warning"
        assert "ov" in out[0]["body"]

    def test_decode_fault_bits_names_every_set_bit(self) -> None:
        from powersuit_proto import wire

        bits = wire.BMSF_SHORT_LATCH | wire.BMSF_OT | wire.BMSF_COMP_ARMED
        names = decode_fault_bits(bits)
        assert names == ["short_latch", "ot", "comparator_armed"]

    def test_resets_when_fault_bits_clear(self) -> None:
        from powersuit_proto import wire

        engine = RulesEngine()
        engine.evaluate(_state(power={"faults": wire.BMSF_UV}))
        assert engine.evaluate(_state(power={"faults": wire.BMSF_UV})) == []  # still set, no refire
        assert engine.evaluate(_state(power={"faults": 0})) == []  # clears -> rearms
        out = engine.evaluate(_state(power={"faults": wire.BMSF_OC_CHARGE}))
        assert _titles(out) == ["BMS fault bits set"]


class TestMultipleRulesIndependent:
    def test_rules_are_independent_of_each_other(self) -> None:
        engine = RulesEngine(RuleThresholds(soc_notice=20, soc_warning=10, temp_critical=60.0))
        out = engine.evaluate(_state(power={"soc": 5, "t_max": 65}, safety={"estop": True}))
        assert set(_titles(out)) == {"Battery low", "Battery critically low", "Pack overtemperature", "E-stop latched"}
