"""Scenario 9 — speech is credit-gated, advisories are not.

The helmet's audio buffer is small and the CAN AUDIO plane is narrow, so the
cloud may only send as much speech as the suit has granted credit for. Warnings
must never queue behind a long-winded answer (docs/link-protocol.md §2.7, §3).
"""

from __future__ import annotations

import asyncio

import pytest

from suit_sim.harness import SuitHarness, until

pytestmark = pytest.mark.asyncio


async def test_server_stalls_without_credit_then_completes_when_granted():
    async with SuitHarness(with_cloud=True) as suit:
        cloud = suit.bridge.cloud
        assert cloud is not None
        cloud.auto_credit = False   # withhold the automatic top-up

        await cloud.voice_query("tell me about the battery care procedure")
        assert await until(lambda: cloud.tts_chunks > 0, timeout=5.0), \
            "no speech started at all"

        # Whatever arrives must fit inside the initial allowance.
        await asyncio.sleep(0.6)
        stalled_at = cloud.tts_chunks
        initial = suit.cloud_server.settings.initial_credits
        assert stalled_at <= initial, \
            f"server sent {stalled_at} chunks on {initial} credits"

        # It is genuinely stalled, not merely slow.
        await asyncio.sleep(0.4)
        assert cloud.tts_chunks == stalled_at, "server kept sending without credit"

        # Granting credit lets it finish.
        await cloud.grant_credits(1, 256)
        assert await until(lambda: cloud.tts_chunks > stalled_at, timeout=5.0), \
            "granting credit did not unblock the stream"


async def test_advisories_are_not_blocked_by_a_stalled_audio_stream():
    async with SuitHarness(with_cloud=True) as suit:
        cloud = suit.bridge.cloud
        assert cloud is not None
        cloud.auto_credit = False

        await cloud.voice_query("describe the airbrake deployment sequence")
        await until(lambda: cloud.tts_chunks > 0, timeout=5.0)
        await asyncio.sleep(0.4)

        # With speech stalled on credit, a critical battery advisory must still
        # get through — the two are independent flows.
        advisories_before = len(cloud.advisories)
        suit.hub.soc_pct = 9
        assert await until(lambda: suit.bridge.bms.get("soc") == 9, timeout=2.0)
        await cloud.send_telemetry()

        assert await until(
            lambda: len(cloud.advisories) > advisories_before, timeout=3.0
        ), "an advisory was stuck behind a credit-stalled audio stream"
