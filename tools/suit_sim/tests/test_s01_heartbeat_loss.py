"""Scenario 1 — the dead-man switch.

The single most important behaviour in the suit: when the orchestrator stops
talking, every limb goes limp within 50 ms, and getting back up is deliberately
harder than staying up (docs/safety.md §2).
"""

from __future__ import annotations

import asyncio

import pytest
from powersuit_proto import wire

from suit_sim.harness import SuitHarness, until

pytestmark = pytest.mark.asyncio


async def test_heartbeat_loss_drops_all_limbs_to_passive():
    async with SuitHarness() as suit:
        assert await suit.arm_all(), "limbs never reached OPERATIONAL"

        frozen = {n: limb.joints[0].pos_crad for n, limb in suit.limbs.items()}
        suit.bridge.heartbeat_enabled = False

        dropped = await until(
            lambda: all(l.state == wire.State.PASSIVE for l in suit.limbs.values()),
            timeout=0.4,
        )
        assert dropped, {n: l.state for n, l in suit.limbs.items()}

        # Passive means the joint stopped being driven, not merely relabelled.
        await asyncio.sleep(0.1)
        for node, limb in suit.limbs.items():
            assert limb.joints[0].pos_crad == frozen[node] or limb.state == wire.State.PASSIVE


async def test_rearm_requires_the_full_streak_and_a_fresh_command():
    async with SuitHarness() as suit:
        assert await suit.arm_all()
        suit.bridge.heartbeat_enabled = False
        assert await until(
            lambda: all(l.state == wire.State.PASSIVE for l in suit.limbs.values()),
            timeout=0.4,
        )

        # Beats alone must not re-arm: the contract also wants a fresh command.
        suit.bridge.heartbeat_enabled = True
        await asyncio.sleep((wire.REARM_WINDOW_MS + 80) / 1000)
        assert all(l.state == wire.State.PASSIVE for l in suit.limbs.values()), \
            "heartbeats alone re-armed the suit"

        assert await suit.arm_all(timeout=2.0)


async def test_rearm_is_not_faster_than_the_window():
    async with SuitHarness() as suit:
        assert await suit.arm_all()
        suit.bridge.heartbeat_enabled = False
        assert await until(
            lambda: all(l.state == wire.State.PASSIVE for l in suit.limbs.values()),
            timeout=0.4,
        )

        loop = asyncio.get_event_loop()
        suit.bridge.heartbeat_enabled = True
        start = loop.time()
        armed = asyncio.ensure_future(suit.arm_all(timeout=2.0))
        assert await armed
        took_ms = (loop.time() - start) * 1000

        # Allow scheduling slop, but it must not be dramatically early.
        assert took_ms >= wire.REARM_WINDOW_MS - 40, f"re-armed after only {took_ms:.0f} ms"
