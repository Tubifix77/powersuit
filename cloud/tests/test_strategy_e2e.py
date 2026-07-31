"""Full in-process round trip over the real link (docs/link-protocol.md §7):

hello -> voice_query("status report") -> advisory + tts_meta start -> grant
credits -> TTS_CHUNK binary frames -> tts_meta end. Also credit gating: with
zero credits, no audio chunks arrive until credits are granted (§2.7: "Server
MUST stop sending when credits hit 0").
"""

from __future__ import annotations

import asyncio

import pytest

from cloud_ai_core.protocol import BTYPE_TTS_CHUNK, CODEC_ADPCM_8K, parse_audio_frame


class TestVoiceQueryRoundTrip:
    async def test_hello_voice_query_advisory_and_tts_stream(self, client) -> None:
        await client.send(
            "voice_query",
            {"query_id": "q-1", "text": "status report", "source": "helmet", "context": {}},
        )

        advisory = await client.recv_type("advisory")
        assert advisory["payload"]["in_reply_to"] == "q-1"
        assert advisory["payload"]["title"] == "Voice response"
        assert advisory["payload"]["severity"] == "info"
        assert "status report" in advisory["payload"]["body"]

        meta_start = await client.recv_type("tts_meta")
        assert meta_start["payload"]["state"] == "start"
        assert meta_start["payload"]["query_id"] == "q-1"
        assert meta_start["payload"]["codec"] == "adpcm8k"
        stream_id = meta_start["payload"]["stream_id"]

        # The default hello_ack already granted `audio_credits` initial credits
        # (32); the manual's answer needs more than that to finish, so grant
        # more — this is the "grant credits" step of the round trip.
        await client.send("audio_credit", {"stream_id": stream_id, "credits": 1000})

        chunks: list[bytes] = []
        while True:
            msg = await client.recv(timeout=2.0)
            if isinstance(msg, bytes):
                chunks.append(msg)
                continue
            assert msg["type"] == "tts_meta"
            assert msg["payload"]["stream_id"] == stream_id
            assert msg["payload"]["state"] == "end"
            break

        assert chunks, "expected at least one TTS_CHUNK binary frame"
        seqs = []
        for raw in chunks:
            btype, codec, sid, seq, payload = parse_audio_frame(raw)
            assert btype == BTYPE_TTS_CHUNK
            assert codec == CODEC_ADPCM_8K
            assert sid == stream_id
            assert len(payload) > 0
            seqs.append(seq)
        assert seqs == list(range(len(seqs)))  # per-stream seq from 0, in order

    async def test_asr_delimited_voice_query_via_binary_audio_falls_back_to_mock_transcript(
        self, client
    ) -> None:
        # No voice_query claims this uplink audio stream, so the server's silence
        # timeout (asr_silence_s) fires and it synthesizes the query itself.
        from cloud_ai_core.protocol import BTYPE_AUDIO_UP_CHUNK, pack_audio_frame

        frame = pack_audio_frame(BTYPE_AUDIO_UP_CHUNK, CODEC_ADPCM_8K, 1, 0, b"\x00" * 8)
        await client.send_binary(frame)

        advisory = await client.recv_type("advisory", timeout=3.0)
        assert advisory["payload"]["in_reply_to"] == "asr-1"
        assert "status report" in advisory["payload"]["body"]  # MockAsr.MOCK_TRANSCRIPT


class TestCreditGating:
    async def test_zero_credits_blocks_audio_until_granted(self, make_server, connect) -> None:
        srv = await make_server(initial_credits=0)
        c = await connect(srv)

        await c.send("voice_query", {"query_id": "q-2", "text": "status report", "source": "helmet"})

        advisory = await c.recv_type("advisory")
        assert advisory["payload"]["in_reply_to"] == "q-2"

        meta_start = await c.recv_type("tts_meta")
        assert meta_start["payload"]["state"] == "start"
        stream_id = meta_start["payload"]["stream_id"]

        # Zero credits: no binary chunk shows up even though the server has
        # already started synthesizing/encoding.
        with pytest.raises(asyncio.TimeoutError):
            await c.recv(timeout=0.4)

        # Grant generously so the whole stream can drain in one go.
        await c.send("audio_credit", {"stream_id": stream_id, "credits": 1000})

        got_chunk = False
        while True:
            msg = await c.recv(timeout=2.0)
            if isinstance(msg, bytes):
                got_chunk = True
                continue
            assert msg["type"] == "tts_meta"
            assert msg["payload"]["stream_id"] == stream_id
            assert msg["payload"]["state"] == "end"
            break
        assert got_chunk

    async def test_partial_credits_release_exactly_that_many_chunks(self, make_server, connect) -> None:
        srv = await make_server(initial_credits=0)
        c = await connect(srv)
        await c.send("voice_query", {"query_id": "q-3", "text": "status report", "source": "helmet"})
        await c.recv_type("advisory")
        meta_start = await c.recv_type("tts_meta")
        stream_id = meta_start["payload"]["stream_id"]

        await c.send("audio_credit", {"stream_id": stream_id, "credits": 2})
        chunk1 = await c.recv(timeout=1.0)
        chunk2 = await c.recv(timeout=1.0)
        assert isinstance(chunk1, bytes) and isinstance(chunk2, bytes)

        # Exactly 2 credits granted -> a 3rd chunk must not appear yet.
        with pytest.raises(asyncio.TimeoutError):
            await c.recv(timeout=0.3)

        # Drain the rest so the test doesn't leak a stalled background task.
        await c.send("audio_credit", {"stream_id": stream_id, "credits": 1000})
        while True:
            msg = await c.recv(timeout=2.0)
            if isinstance(msg, dict) and msg.get("type") == "tts_meta":
                assert msg["payload"]["state"] == "end"
                break
