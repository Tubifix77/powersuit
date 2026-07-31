"""Scenario 8 — a lost voice frame must not desynchronise the decoder forever.

ADPCM is stateful: a dropped frame makes every subsequent sample wrong until the
decoder is reseeded. The AUDIO plane therefore carries a SYNC every 50 frames,
and the receiver inserts silence and waits for it rather than emitting noise
(docs/network-map.md §3.5).
"""

from __future__ import annotations

import pytest

from suit_sim.harness import SuitHarness, until

from _sig import is_audible, tone

pytestmark = pytest.mark.asyncio


async def test_dropped_frames_produce_gaps_but_the_stream_relocks():
    async with SuitHarness() as suit:
        suit.hub.drop_audio_every = 37

        sent = suit.helmet.push_utterance(tone(16 * 400))
        assert sent >= 300

        assert await until(lambda: suit.bridge.audio_gaps > 0, timeout=3.0), \
            "hub dropped frames but the bridge never noticed a gap"
        assert await until(lambda: len(suit.bridge.uplink_pcm) > 2000, timeout=3.0)

        # It re-locked at a later SYNC rather than staying dead.
        pcm = suit.bridge.uplink_pcm
        assert is_audible(pcm[-800:]), "decoder never recovered after the gaps"


async def test_clean_stream_has_no_gaps():
    async with SuitHarness() as suit:
        suit.helmet.push_utterance(tone(16 * 120))
        assert await until(lambda: len(suit.bridge.uplink_pcm) > 1500, timeout=3.0)
        assert suit.bridge.audio_gaps == 0
        assert is_audible(suit.bridge.uplink_pcm)
