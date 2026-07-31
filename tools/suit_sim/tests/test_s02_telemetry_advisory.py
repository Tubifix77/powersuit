"""Scenario 2 — telemetry reaches the cloud and advice comes back.

Proves the whole uplink chain: hub BMS -> SPI frames -> bridge state -> batched
JSON over a real WebSocket -> the actual rules engine -> an unsolicited advisory
that survives the downlink whitelist (docs/link-protocol.md §7).
"""

from __future__ import annotations

import pytest

from suit_sim.harness import SuitHarness, until

pytestmark = pytest.mark.asyncio


async def test_low_battery_produces_an_unsolicited_warning():
    async with SuitHarness(with_cloud=True) as suit:
        cloud = suit.bridge.cloud
        assert cloud is not None and cloud.connected

        suit.hub.soc_pct = 9
        assert await until(lambda: suit.bridge.bms.get("soc") == 9, timeout=2.0), \
            "the low reading never reached the bridge over SPI"

        await cloud.send_telemetry()

        assert await until(
            lambda: any(a.get("severity") == "warning" for a in cloud.advisories),
            timeout=3.0,
        ), cloud.advisories

        warning = next(a for a in cloud.advisories if a["severity"] == "warning")
        assert "9%" in warning["body"]
        # Unsolicited: nobody asked a question, so there is nothing to reply to.
        assert "in_reply_to" not in warning
        assert cloud.rejected == 0, "something arrived that the whitelist should have dropped"


async def test_healthy_battery_produces_no_advisory():
    async with SuitHarness(with_cloud=True) as suit:
        cloud = suit.bridge.cloud
        assert cloud is not None

        suit.hub.soc_pct = 88
        assert await until(lambda: suit.bridge.bms.get("soc") == 88, timeout=2.0)
        await cloud.send_telemetry()

        assert not await until(lambda: bool(cloud.advisories), timeout=0.6), \
            f"unexpected advisory for a healthy pack: {cloud.advisories}"
