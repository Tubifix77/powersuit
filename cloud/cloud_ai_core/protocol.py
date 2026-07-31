"""Server-side glue over powersuit_proto.link (docs/link-protocol.md).

powersuit_proto.link owns the envelope/binary codecs; this module adds the
server's sequencing (SessionCodec), the uplink type gate, and error helpers.
"""

from __future__ import annotations

from typing import Any

from powersuit_proto.link import (  # noqa: F401  (re-exported for package users)
    ADVISORY_SEVERITIES,
    BIN_HDR_SIZE,
    BTYPE_AUDIO_UP_CHUNK,
    BTYPE_TTS_CHUNK,
    CLOSE_AUTH_FAILED,
    CLOSE_BAD_VERSION,
    CODEC_ADPCM_8K,
    CODEC_PCM16_16K,
    DOWNLINK_WHITELIST,
    PROTO_VERSION,
    LinkProtocolError,
    T_ADVISORY,
    T_AUDIO_CREDIT,
    T_BYE,
    T_ERROR,
    T_HELLO,
    T_HELLO_ACK,
    T_LINK_STATS,
    T_TELEMETRY_BATCH,
    T_TTS_META,
    T_VOICE_QUERY,
    decode_envelope,
    encode_envelope,
    make_envelope,
    pack_audio_frame,
    parse_audio_frame,
)

# The ONLY client-originated text types the server accepts. Anything else gets an
# `error` envelope with code `unknown_type` (connection stays open, §2).
UPLINK_TYPES = frozenset(
    {T_HELLO, T_TELEMETRY_BATCH, T_VOICE_QUERY, T_AUDIO_CREDIT, T_LINK_STATS, T_BYE}
)

# TTS binary chunk payload size: 320 ADPCM bytes = 640 samples = 80 ms @ 8 kHz.
TTS_CHUNK_BYTES = 320


class SessionCodec:
    """Per-session downlink sequencer. seq is monotonic from 1 (§2)."""

    def __init__(self) -> None:
        self._seq = 0

    @property
    def seq(self) -> int:
        return self._seq

    def next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def make(self, msg_type: str, payload: dict[str, Any], ts: float | None = None) -> dict[str, Any]:
        return make_envelope(msg_type, self.next_seq(), payload, ts=ts)


def validate_uplink(env: dict[str, Any]) -> None:
    """Gate a decoded client envelope against the uplink whitelist."""
    if env["type"] not in UPLINK_TYPES:
        raise LinkProtocolError("unknown_type", f"type {env['type']!r} not accepted uplink")


def error_payload(code: str, detail: str, ref_seq: int | None = None) -> dict[str, Any]:
    payload: dict[str, Any] = {"code": code, "detail": detail}
    if ref_seq is not None:
        payload["ref_seq"] = ref_seq
    return payload
