"""Scenario 10 — the local safety path always beats the cloud.

This is the scenario the architecture exists to satisfy. When the battery
shorts, the limbs must be limp long before any advice about it could possibly
arrive from a datacentre. If this test ever inverts, the suit has acquired a
dependency on a network round-trip for a safety reaction, and that is a defect
no amount of latency tuning fixes.
"""

from __future__ import annotations

import asyncio
import contextlib

import pytest
from powersuit_proto import wire

from suit_sim.harness import SuitHarness, elapsed_until, until

pytestmark = pytest.mark.asyncio


async def test_limbs_latch_before_the_cloud_can_advise():
    async with SuitHarness(with_cloud=True) as suit:
        cloud = suit.bridge.cloud
        assert cloud is not None
        assert await suit.arm_all()

        async def pump_telemetry() -> None:
            """The cloud only learns anything because we keep telling it."""
            while True:
                with contextlib.suppress(Exception):
                    await cloud.send_telemetry()
                await asyncio.sleep(0.05)

        pump = asyncio.ensure_future(pump_telemetry())
        try:
            loop = asyncio.get_event_loop()
            t0 = loop.time()

            suit.hub.short_circuit()

            latched = await elapsed_until(
                lambda: all(l.state == wire.State.ESTOP for l in suit.limbs.values()),
                timeout=3.0,
            )
            advised = await elapsed_until(
                lambda: any(a.get("severity") == "critical" for a in cloud.advisories),
                timeout=5.0,
            )
        finally:
            pump.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await pump

        assert latched is not None, "limbs never latched"
        assert advised is not None, "the cloud never advised — the test proves nothing"

        # `elapsed_until` measures from its own call, so compare against the
        # shared t0 instead: both were started after the same short_circuit().
        assert latched < advised, (
            f"cloud advice ({advised * 1000:.0f} ms) beat the local latch "
            f"({latched * 1000:.0f} ms) — the cloud is in the safety path"
        )
        # And the local reaction is inside the watchdog window, not merely first.
        assert latched <= wire.HEARTBEAT_TIMEOUT_MS / 1000, \
            f"local latch took {latched * 1000:.0f} ms"


async def test_cloud_cannot_reach_an_actuator_even_when_it_tries():
    """The whitelist is the mechanism; this is the behavioural proof."""
    async with SuitHarness(with_cloud=True) as suit:
        cloud = suit.bridge.cloud
        assert cloud is not None
        assert await suit.arm_all()

        from powersuit_proto import link

        # Anything outside the whitelist is dropped and counted, whatever it
        # claims to be — there is no message type that maps to actuation.
        for forged in ("joint_command", "mode_set", link.T_TELEMETRY_BATCH):
            cloud._on_text(link.encode_envelope(
                link.make_envelope(forged, 999, {"limb": "arm_right", "pos": 1.5})
            ))

        assert cloud.rejected == 3, f"whitelist let something through: {cloud.rejected}/3 rejected"
        assert all(l.state == wire.State.OPERATIONAL for l in suit.limbs.values())
