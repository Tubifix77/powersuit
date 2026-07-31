"""Scenario 7 — losing the cloud link must be survivable, not fatal.

A suit moving between 5G, Wi-Fi and satellite will drop its session regularly.
The rule is that the suit keeps flying either way: the safety planes never
depended on the cloud, and the session resumes without losing control messages
(docs/link-protocol.md §1, §5).
"""

from __future__ import annotations

import asyncio

import pytest
from powersuit_proto import wire

from suit_sim.harness import SuitHarness, until

pytestmark = pytest.mark.asyncio


async def test_session_resumes_after_the_link_bounces():
    async with SuitHarness(with_cloud=True) as suit:
        cloud = suit.bridge.cloud
        assert cloud is not None and cloud.connected
        first_session = cloud.session_id
        assert first_session

        await suit.restart_cloud()
        assert await until(lambda: not cloud.connected, timeout=3.0), \
            "the client never noticed the link drop"

        await cloud.connect(resume=True)
        assert cloud.connected

        # The link works again end to end.
        suit.hub.soc_pct = 9
        assert await until(lambda: suit.bridge.bms.get("soc") == 9, timeout=2.0)
        await cloud.send_telemetry()
        assert await until(lambda: bool(cloud.advisories), timeout=3.0), \
            "no advisory after resume — the session did not really recover"


async def test_the_suit_keeps_beating_while_the_cloud_is_gone():
    """The whole point of the tiering: Node 9 is not in the control path."""
    async with SuitHarness(with_cloud=True) as suit:
        assert await suit.arm_all()
        cloud = suit.bridge.cloud
        assert cloud is not None

        beats_before = suit.bridge.beats_sent
        await suit.restart_cloud()
        assert await until(lambda: not cloud.connected, timeout=3.0)

        await asyncio.sleep(0.2)
        assert suit.bridge.beats_sent > beats_before, "heartbeat stopped when the cloud did"
        assert all(l.state == wire.State.OPERATIONAL for l in suit.limbs.values()), \
            "limbs dropped out because a cloud service restarted"
