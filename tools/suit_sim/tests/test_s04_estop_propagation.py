"""Scenario 4 — e-stop propagation and replay resistance.

A battery short is the fastest-moving fault in the suit. Every limb must latch
within the watchdog window, the orchestrator must stop beating, and un-latching
must be impossible to trigger by replaying a captured frame (docs/safety.md §3).
"""

from __future__ import annotations

import asyncio

import pytest
from powersuit_proto import wire

from suit_sim.harness import SuitHarness, until

pytestmark = pytest.mark.asyncio


async def test_short_circuit_latches_every_limb_fast():
    async with SuitHarness() as suit:
        assert await suit.arm_all()

        suit.hub.short_circuit()

        latched = await until(
            lambda: all(l.state == wire.State.ESTOP for l in suit.limbs.values()),
            timeout=wire.HEARTBEAT_TIMEOUT_MS / 1000,
        )
        assert latched, {n: l.state for n, l in suit.limbs.items()}

        # The bridge learns of the latch over SPI and stops beating.
        assert await until(lambda: suit.bridge.estop_latched, timeout=0.5)
        before = suit.bridge.beats_sent
        await asyncio.sleep(0.1)
        assert suit.bridge.beats_sent == before, "bridge kept beating through an e-stop"


async def test_clear_estop_recovers_but_a_replayed_counter_does_not():
    async with SuitHarness() as suit:
        assert await suit.arm_all()
        suit.hub.short_circuit()
        assert await until(
            lambda: all(l.state == wire.State.ESTOP for l in suit.limbs.values()),
            timeout=0.3,
        )

        # A replay of counter 0 (never accepted, not greater than the initial
        # value) must be ignored.
        suit.bridge.send_clear_estop(0)
        await asyncio.sleep(0.15)
        assert all(l.state == wire.State.ESTOP for l in suit.limbs.values()), \
            "a non-monotonic CLEAR_ESTOP un-latched the suit"

        # A genuine, higher counter is accepted; the limbs still need the
        # post-clear heartbeat streak before they leave the latch.
        suit.bridge.send_clear_estop(1)
        recovered = await until(
            lambda: all(l.state != wire.State.ESTOP for l in suit.limbs.values()),
            timeout=(wire.ESTOP_REARM_MS / 1000) + 1.0,
        )
        assert recovered, {n: l.state for n, l in suit.limbs.items()}

        # And replaying that same counter afterwards changes nothing.
        suit.hub.short_circuit()
        assert await until(
            lambda: all(l.state == wire.State.ESTOP for l in suit.limbs.values()),
            timeout=0.3,
        )
        suit.bridge.send_clear_estop(1)
        await asyncio.sleep(0.2)
        assert all(l.state == wire.State.ESTOP for l in suit.limbs.values()), \
            "a replayed CLEAR_ESTOP counter was accepted twice"
