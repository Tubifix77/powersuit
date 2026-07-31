"""Handshake and session lifecycle (docs/link-protocol.md §1, §2.1-§2.2, §4).

Bad token -> close 4001. Bad protocol version -> close 4002. hello_ack field
shape. Resume replays exactly the buffered messages after resume_from. Session
TTL expiry means an old session cannot be resumed once it's purged.
"""

from __future__ import annotations

import asyncio

import pytest
from websockets.exceptions import ConnectionClosed

from cloud_ai_core.protocol import CLOSE_AUTH_FAILED, CLOSE_BAD_VERSION, PROTO_VERSION

from conftest import SUIT_ID, TOKEN


class TestHandshakeFailures:
    async def test_bad_token_closes_4001(self, raw_client) -> None:
        await raw_client.send(
            "hello",
            {"proto": PROTO_VERSION, "suit_id": SUIT_ID, "token": "wrong-token", "agent": "t/0.1"},
        )
        with pytest.raises(ConnectionClosed) as exc_info:
            await raw_client.recv()
        assert exc_info.value.rcvd.code == CLOSE_AUTH_FAILED

    async def test_missing_token_closes_4001(self, raw_client) -> None:
        await raw_client.send("hello", {"proto": PROTO_VERSION, "suit_id": SUIT_ID, "agent": "t/0.1"})
        with pytest.raises(ConnectionClosed) as exc_info:
            await raw_client.recv()
        assert exc_info.value.rcvd.code == CLOSE_AUTH_FAILED

    async def test_bad_protocol_version_in_hello_payload_closes_4002(self, raw_client) -> None:
        await raw_client.send(
            "hello",
            {"proto": PROTO_VERSION + 1, "suit_id": SUIT_ID, "token": TOKEN, "agent": "t/0.1"},
        )
        with pytest.raises(ConnectionClosed) as exc_info:
            await raw_client.recv()
        assert exc_info.value.rcvd.code == CLOSE_BAD_VERSION

    async def test_bad_envelope_version_closes_4002(self, raw_client) -> None:
        await raw_client.send(
            "hello",
            {"proto": PROTO_VERSION, "suit_id": SUIT_ID, "token": TOKEN, "agent": "t/0.1"},
            v=99,
        )
        with pytest.raises(ConnectionClosed) as exc_info:
            await raw_client.recv()
        assert exc_info.value.rcvd.code == CLOSE_BAD_VERSION

    async def test_first_frame_not_hello_closes_4002(self, raw_client) -> None:
        await raw_client.send("link_stats", {"bearer": "wifi", "rtt_ms": 1, "loss_pct": 0.0, "switches": 0})
        with pytest.raises(ConnectionClosed) as exc_info:
            await raw_client.recv()
        assert exc_info.value.rcvd.code == CLOSE_BAD_VERSION

    async def test_binary_first_frame_closes_4002(self, raw_client) -> None:
        await raw_client.send_binary(b"\x02\x01\x00\x00\x00\x00\x00\x00")
        with pytest.raises(ConnectionClosed) as exc_info:
            await raw_client.recv()
        assert exc_info.value.rcvd.code == CLOSE_BAD_VERSION

    async def test_hello_timeout_closes_4002(self, make_server, connect) -> None:
        srv = await make_server(hello_timeout_s=0.2)
        c = await connect(srv, do_hello=False)
        with pytest.raises(ConnectionClosed) as exc_info:
            await c.recv(timeout=2.0)
        assert exc_info.value.rcvd.code == CLOSE_BAD_VERSION


class TestHelloAck:
    async def test_hello_ack_field_shape(self, raw_client) -> None:
        ts_sent = 1_700_000_000.5
        ack = await raw_client.hello(ts=ts_sent)
        assert ack["type"] == "hello_ack"
        payload = ack["payload"]
        assert set(payload.keys()) == {
            "session_id", "resume_from", "ts_echo", "audio_credits", "heartbeat_s", "capabilities",
        }
        assert isinstance(payload["session_id"], str) and payload["session_id"]
        assert payload["resume_from"] == 0
        assert payload["ts_echo"] == ts_sent
        assert payload["audio_credits"] == 32  # Settings() default
        assert payload["heartbeat_s"] == 5  # Settings() default
        assert payload["capabilities"] == ["rag", "tts", "advisory"]

    async def test_hello_ack_respects_custom_settings(self, make_server, connect) -> None:
        srv = await make_server(initial_credits=7, heartbeat_s=2)
        c = await connect(srv)
        payload = c.hello_ack["payload"]
        assert payload["audio_credits"] == 7
        assert payload["heartbeat_s"] == 2


class TestResume:
    async def test_resume_replays_exactly_messages_after_resume_from(self, server, connect) -> None:
        c1 = await connect(server)
        session_id = c1.session_id
        last_rx_seq = c1.hello_ack["seq"]  # the hello_ack itself, seq 1

        # Disconnect without `bye` so the session survives in the store.
        await c1.close()
        await asyncio.sleep(0.05)

        await server.push_advisory("first", "body one", advisory_id="adv-1")
        await server.push_advisory("second", "body two", advisory_id="adv-2")

        c2 = await connect(
            server, do_hello=False,
        )
        ack = await c2.hello(resume={"session_id": session_id, "last_rx_seq": last_rx_seq})
        assert ack["payload"]["session_id"] == session_id
        assert ack["payload"]["resume_from"] == last_rx_seq + 1

        msg1 = await c2.recv()
        msg2 = await c2.recv()
        assert [msg1["type"], msg2["type"]] == ["advisory", "advisory"]
        assert [msg1["payload"]["title"], msg2["payload"]["title"]] == ["first", "second"]
        assert msg1["seq"] < msg2["seq"]
        assert msg1["seq"] > last_rx_seq

        # Nothing else queued — no third message shows up.
        with pytest.raises(asyncio.TimeoutError):
            await c2.recv(timeout=0.3)

    async def test_resume_with_unknown_session_id_starts_fresh(self, server, connect) -> None:
        c = await connect(server, do_hello=False)
        ack = await c.hello(resume={"session_id": "does-not-exist", "last_rx_seq": 5})
        assert ack["payload"]["resume_from"] == 0
        assert ack["payload"]["session_id"] != "does-not-exist"

    async def test_resume_with_mismatched_suit_id_starts_fresh(self, server, connect) -> None:
        c1 = await connect(server, suit_id="suit-a")
        session_id = c1.session_id
        await c1.close()
        await asyncio.sleep(0.05)

        c2 = await connect(server, do_hello=False)
        ack = await c2.hello(suit_id="suit-b", resume={"session_id": session_id, "last_rx_seq": 1})
        assert ack["payload"]["resume_from"] == 0
        assert ack["payload"]["session_id"] != session_id


class TestSessionTtl:
    async def test_expired_session_cannot_be_resumed(self, make_server, connect) -> None:
        srv = await make_server(session_ttl_s=0.15)
        c1 = await connect(srv)
        session_id = c1.session_id
        await c1.close()

        await asyncio.sleep(0.35)  # well past the TTL

        c2 = await connect(srv, do_hello=False)
        ack = await c2.hello(resume={"session_id": session_id, "last_rx_seq": 1})
        assert ack["payload"]["resume_from"] == 0
        assert ack["payload"]["session_id"] != session_id

    async def test_unexpired_session_can_still_be_resumed(self, make_server, connect) -> None:
        srv = await make_server(session_ttl_s=5.0)
        c1 = await connect(srv)
        session_id = c1.session_id
        last_rx_seq = c1.hello_ack["seq"]
        await c1.close()

        await asyncio.sleep(0.1)  # well within the TTL

        c2 = await connect(srv, do_hello=False)
        ack = await c2.hello(resume={"session_id": session_id, "last_rx_seq": last_rx_seq})
        assert ack["payload"]["session_id"] == session_id
        assert ack["payload"]["resume_from"] == last_rx_seq + 1
