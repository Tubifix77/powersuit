"""Scenario 5 — a corrupted SPI frame must be dropped whole.

CRC16 exists so that a bit flip on the harness never reaches the ROS graph as a
plausible-looking joint angle. The link must also resynchronise on its own
(docs/network-map.md §6).
"""

from __future__ import annotations

import asyncio

import pytest

from suit_sim.harness import SuitHarness, until

pytestmark = pytest.mark.asyncio


async def test_corrupted_frames_are_rejected_and_the_link_recovers():
    async with SuitHarness() as suit:
        assert await suit.arm_all()
        await until(lambda: bool(suit.bridge.bms), timeout=1.0)

        good_before = suit.bridge.frames_ok
        soc_before = suit.bridge.bms.get("soc")

        suit.hub.corrupt_next_frames(3)
        assert await until(lambda: suit.bridge.crc_errors >= 3, timeout=1.0), \
            f"expected 3 CRC rejections, saw {suit.bridge.crc_errors}"

        # Nothing from the corrupted frames leaked into the state.
        assert suit.bridge.bms.get("soc") == soc_before

        # The stream keeps flowing afterwards without a manual resync.
        assert await until(
            lambda: suit.bridge.frames_ok > good_before + 5, timeout=1.0
        ), "link did not recover after corruption"


async def test_corruption_does_not_stall_the_heartbeat():
    async with SuitHarness() as suit:
        assert await suit.arm_all()
        suit.hub.corrupt_next_frames(2)
        before = suit.bridge.beats_sent
        await asyncio.sleep(0.15)
        assert suit.bridge.beats_sent > before, "heartbeat stopped over a recoverable fault"
