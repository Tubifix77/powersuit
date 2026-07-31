"""Scenario 3 — the full voice loop, helmet to cloud and back to the ear.

Microphone PCM is ADPCM-encoded onto the CAN AUDIO plane, tunnelled over SPI,
decoded by the bridge, answered by the real cloud service, and the synthesised
reply is re-framed back down to the helmet. Every codec on that path is the
shared implementation in `powersuit_proto`, so this also proves the firmware and
cloud codecs agree.
"""

from __future__ import annotations

import pytest

from suit_sim.harness import SuitHarness, until

from _sig import is_audible, tone

pytestmark = pytest.mark.asyncio


async def test_utterance_reaches_the_bridge_intact():
    async with SuitHarness() as suit:
        sent = suit.helmet.push_utterance(tone(16 * 100))
        assert sent == 100

        assert await until(lambda: len(suit.bridge.uplink_pcm) >= 1500, timeout=3.0), \
            f"only {len(suit.bridge.uplink_pcm)} samples arrived"
        assert suit.bridge.audio_gaps == 0
        assert is_audible(suit.bridge.uplink_pcm)


async def test_voice_query_returns_speech_and_reaches_the_helmet():
    async with SuitHarness(with_cloud=True) as suit:
        cloud = suit.bridge.cloud
        assert cloud is not None

        suit.helmet.push_utterance(tone(16 * 60))
        assert await until(lambda: len(suit.bridge.uplink_pcm) > 500, timeout=3.0)
        suit.helmet.end_utterance()

        await cloud.voice_query("status report")

        assert await until(lambda: bool(cloud.advisories), timeout=5.0), \
            "the cloud never answered the query"
        assert await until(lambda: cloud.tts_chunks > 0, timeout=5.0), \
            "no speech came back"
        assert await until(lambda: len(cloud.tts_pcm) > 400, timeout=5.0)
        assert is_audible(cloud.tts_pcm), "the reply decoded to silence"

        # Re-frame the reply down to the helmet the way the bridge does.
        suit.bridge.send_audio_down(cloud.tts_pcm[:1600])
        assert await until(lambda: len(suit.helmet.downlink_frames) > 50, timeout=3.0)
        assert suit.helmet.downlink_syncs, "no SYNC preceded the downlink audio"
        assert is_audible(suit.helmet.downlink_pcm), "the helmet decoded silence"

        # Ordered delivery: the AUDIO plane's low byte must advance by one.
        seqs = [f.id for f in suit.helmet.downlink_frames[:40]]
        assert seqs == [(seqs[0] + i) & 0xFF for i in range(len(seqs))], "frames arrived out of order"
