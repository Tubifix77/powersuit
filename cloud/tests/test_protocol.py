"""Uplink type gating and error envelopes (docs/link-protocol.md §2).

Unit-level tests against cloud_ai_core.protocol for the parts owned by this
package (the uplink whitelist and error helper); the rest — envelope shape,
codecs — belongs to powersuit_proto and is not duplicated here. Also a couple
of integration checks that a live connection actually behaves this way: an
unknown type or malformed frame gets an `error` envelope and the connection
stays open (docs/link-protocol.md §2: "do not close").
"""

from __future__ import annotations

import pytest

from cloud_ai_core.protocol import (
    LinkProtocolError,
    T_ADVISORY,
    T_AUDIO_CREDIT,
    T_BYE,
    T_HELLO,
    T_LINK_STATS,
    T_TELEMETRY_BATCH,
    T_TTS_META,
    T_VOICE_QUERY,
    UPLINK_TYPES,
    error_payload,
    validate_uplink,
)

from conftest import SUIT_ID, TOKEN


def _env(msg_type: str, seq: int = 1) -> dict:
    return {"v": 1, "type": msg_type, "seq": seq, "ts": 0.0, "payload": {}}


class TestUplinkGating:
    @pytest.mark.parametrize(
        "msg_type",
        [T_HELLO, T_TELEMETRY_BATCH, T_VOICE_QUERY, T_AUDIO_CREDIT, T_LINK_STATS, T_BYE],
    )
    def test_accepted_types_pass(self, msg_type: str) -> None:
        validate_uplink(_env(msg_type))  # must not raise

    @pytest.mark.parametrize(
        "msg_type",
        [T_TTS_META, T_ADVISORY, "hello_ack", "bogus_type", ""],
    )
    def test_rejected_types_raise_unknown_type(self, msg_type: str) -> None:
        with pytest.raises(LinkProtocolError) as exc_info:
            validate_uplink(_env(msg_type))
        assert exc_info.value.code == "unknown_type"

    def test_downlink_only_types_are_not_uplink_types(self) -> None:
        # advisory/tts_meta/hello_ack are server->client only; never accepted uplink.
        assert T_TTS_META not in UPLINK_TYPES
        assert T_ADVISORY not in UPLINK_TYPES
        assert "hello_ack" not in UPLINK_TYPES


class TestErrorPayload:
    def test_error_payload_shape_without_ref_seq(self) -> None:
        payload = error_payload("bad_payload", "not valid JSON")
        assert payload == {"code": "bad_payload", "detail": "not valid JSON"}

    def test_error_payload_shape_with_ref_seq(self) -> None:
        payload = error_payload("unknown_type", "type 'x' not accepted", ref_seq=42)
        assert payload == {"code": "unknown_type", "detail": "type 'x' not accepted", "ref_seq": 42}


# --- integration: the running server actually does this ------------------------


class TestLiveErrorEnvelopes:
    async def test_unknown_type_gets_error_and_connection_stays_open(self, client) -> None:
        await client.send("bogus_type", {"anything": 1})
        err = await client.recv_type("error")
        assert err["payload"]["code"] == "unknown_type"

        # Connection must still be usable afterwards (§2: "do not close").
        await client.send("link_stats", {"bearer": "wifi", "rtt_ms": 10, "loss_pct": 0.0, "switches": 0})
        await client.send("bye", {"reason": "shutdown"})
        # bye is accepted uplink; server closes cleanly with 1000.
        with pytest.raises(Exception):
            await client.recv(timeout=1.0)

    async def test_downlink_only_type_from_client_is_rejected(self, client) -> None:
        await client.send(T_TTS_META, {"stream_id": 1, "query_id": "q", "state": "start",
                                        "codec": "adpcm8k", "text": ""})
        err = await client.recv_type("error")
        assert err["payload"]["code"] == "unknown_type"

    async def test_malformed_json_gets_bad_payload_error(self, client) -> None:
        await client.send_raw("{not json")
        err = await client.recv_type("error")
        assert err["payload"]["code"] == "bad_payload"
        # still usable
        await client.send("link_stats", {"bearer": "wifi", "rtt_ms": 10, "loss_pct": 0.0, "switches": 0})

    async def test_wrong_envelope_version_gets_bad_payload_error(self, client) -> None:
        await client.send("link_stats", {"bearer": "wifi", "rtt_ms": 10, "loss_pct": 0.0, "switches": 0}, v=99)
        err = await client.recv_type("error")
        assert err["payload"]["code"] == "bad_payload"

    async def test_missing_field_gets_bad_payload_error(self, client) -> None:
        await client.send_raw('{"v":1,"type":"link_stats","seq":5,"payload":{}}')  # no ts
        err = await client.recv_type("error")
        assert err["payload"]["code"] == "bad_payload"

    async def test_duplicate_hello_gets_bad_payload_error_with_ref_seq(self, client) -> None:
        seq = await client.send(
            "hello",
            {"proto": 1, "suit_id": SUIT_ID, "token": TOKEN, "agent": "test-client/0.1"},
        )
        err = await client.recv()
        assert err["type"] == "error"
        assert err["payload"]["code"] == "bad_payload"
        assert "duplicate hello" in err["payload"]["detail"]
        assert err["payload"]["ref_seq"] == seq
