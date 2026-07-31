"""Unsolicited advisories from telemetry (docs/link-protocol.md §7: "Telemetry
batches update the rolling suit state; threshold rules ... emit unsolicited
advisories"). A telemetry_batch carrying soc=9 crosses the default soc_warning
threshold (10, config.RuleThresholds) and must produce a warning advisory with
no in_reply_to — it wasn't asked for."""

from __future__ import annotations

import asyncio

import pytest


def _batch(soc: float, **extra_power) -> dict:
    power = {"pack_v": 42.0, "current_a": 5.0, "soc": soc, "t_max": 30, "faults": 0}
    power.update(extra_power)
    return {
        "window_ms": 200,
        "sources": {
            "power": power,
            "safety": {"state": "OPERATIONAL", "estop": False, "hb_ok": True},
        },
        "dropped_count": 0,
    }


class TestLowSocAdvisory:
    async def test_soc_9_produces_unsolicited_warning_advisory(self, client) -> None:
        await client.send("telemetry_batch", _batch(soc=9))

        # soc=9 crosses both soc_notice(20) and soc_warning(10); find the warning.
        warning = None
        for _ in range(5):
            adv = await client.recv_type("advisory", timeout=2.0)
            if adv["payload"]["severity"] == "warning":
                warning = adv
                break
        assert warning is not None, "expected a warning-severity advisory"
        assert warning["payload"]["title"] == "Battery critically low"
        assert "in_reply_to" not in warning["payload"]
        assert "9%" in warning["payload"]["body"]
        assert warning["payload"]["hud"] == {"icon": "battery", "ttl_s": 10}

    async def test_healthy_telemetry_produces_no_advisory(self, client) -> None:
        await client.send("telemetry_batch", _batch(soc=90))
        with pytest.raises(asyncio.TimeoutError):
            await client.recv(timeout=0.3)

    async def test_repeated_low_soc_batches_do_not_spam_the_warning(self, client) -> None:
        await client.send("telemetry_batch", _batch(soc=9))
        # Drain whatever fires on the first low reading (notice + warning).
        got_warning = False
        for _ in range(5):
            try:
                adv = await client.recv(timeout=0.5)
            except asyncio.TimeoutError:
                break
            if isinstance(adv, dict) and adv["payload"].get("severity") == "warning":
                got_warning = True
        assert got_warning

        # Same low soc again: the warning rule is disarmed until soc >= 15; no refire.
        await client.send("telemetry_batch", _batch(soc=9))
        with pytest.raises(asyncio.TimeoutError):
            await client.recv(timeout=0.3)

        # Recover past the soc_critical reset point (>=15) — soc_low needs >=25
        # and stays disarmed — then dip again: soc_critical must refire.
        await client.send("telemetry_batch", _batch(soc=16))
        await client.send("telemetry_batch", _batch(soc=9))
        adv = await client.recv_type("advisory", timeout=2.0)
        assert adv["payload"]["severity"] == "warning"
        assert adv["payload"]["title"] == "Battery critically low"
