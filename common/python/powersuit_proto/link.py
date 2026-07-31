"""Suit <-> cloud link protocol codecs — normative reference: docs/link-protocol.md.

JSON text envelope for control, 8-byte-header binary frames for audio. Used by the
Node 9 server (cloud_ai_core), the Node 8 gateway, and tools/suit_sim.
"""

from __future__ import annotations

import json
import struct
import time
from typing import Any

PROTO_VERSION = 1

# Envelope types.
T_HELLO = "hello"
T_HELLO_ACK = "hello_ack"
T_TELEMETRY_BATCH = "telemetry_batch"
T_VOICE_QUERY = "voice_query"
T_ADVISORY = "advisory"
T_TTS_META = "tts_meta"
T_AUDIO_CREDIT = "audio_credit"
T_LINK_STATS = "link_stats"
T_ERROR = "error"
T_BYE = "bye"

# The ONLY server-originated types the Node 8 gateway accepts (docs/link-protocol.md §6).
DOWNLINK_WHITELIST = frozenset(
    {T_HELLO_ACK, T_ADVISORY, T_TTS_META, T_AUDIO_CREDIT, T_ERROR, T_BYE}
)

ADVISORY_SEVERITIES = ("info", "notice", "warning", "critical")

# WS close codes.
CLOSE_AUTH_FAILED = 4001
CLOSE_BAD_VERSION = 4002

# Binary frame header: btype u8, codec u8, stream_id u16, seq u32 (LE).
_BIN_HDR = struct.Struct("<BBHI")
BIN_HDR_SIZE = _BIN_HDR.size

BTYPE_TTS_CHUNK = 0x01
BTYPE_AUDIO_UP_CHUNK = 0x02

CODEC_ADPCM_8K = 0x01
CODEC_PCM16_16K = 0x02

MAX_BIN_PAYLOAD = 2048


class LinkProtocolError(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def make_envelope(msg_type: str, seq: int, payload: dict[str, Any], ts: float | None = None) -> dict[str, Any]:
    return {
        "v": PROTO_VERSION,
        "type": msg_type,
        "seq": seq,
        "ts": time.time() if ts is None else ts,
        "payload": payload,
    }


def encode_envelope(env: dict[str, Any]) -> str:
    return json.dumps(env, separators=(",", ":"))


def decode_envelope(raw: str | bytes) -> dict[str, Any]:
    """Parse and structurally validate an envelope. Raises LinkProtocolError."""
    try:
        env = json.loads(raw)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise LinkProtocolError("bad_payload", f"not valid JSON: {exc}") from exc
    if not isinstance(env, dict):
        raise LinkProtocolError("bad_payload", "envelope is not an object")
    if env.get("v") != PROTO_VERSION:
        raise LinkProtocolError("bad_version", f"unsupported protocol version {env.get('v')!r}")
    msg_type = env.get("type")
    if not isinstance(msg_type, str) or not msg_type:
        raise LinkProtocolError("bad_payload", "missing type")
    seq = env.get("seq")
    if not isinstance(seq, int) or seq < 0:
        raise LinkProtocolError("bad_payload", "missing/invalid seq")
    if not isinstance(env.get("ts"), (int, float)):
        raise LinkProtocolError("bad_payload", "missing/invalid ts")
    if not isinstance(env.get("payload"), dict):
        raise LinkProtocolError("bad_payload", "missing/invalid payload")
    return env


def pack_audio_frame(btype: int, codec: int, stream_id: int, seq: int, payload: bytes) -> bytes:
    if len(payload) > MAX_BIN_PAYLOAD:
        raise LinkProtocolError("bad_payload", f"binary payload {len(payload)} > {MAX_BIN_PAYLOAD}")
    return _BIN_HDR.pack(btype, codec, stream_id, seq) + payload


def parse_audio_frame(data: bytes | memoryview) -> tuple[int, int, int, int, bytes]:
    """Returns (btype, codec, stream_id, seq, payload). Raises LinkProtocolError."""
    if len(data) < BIN_HDR_SIZE:
        raise LinkProtocolError("bad_payload", f"binary frame shorter than header ({len(data)})")
    btype, codec, stream_id, seq = _BIN_HDR.unpack_from(data)
    payload = bytes(data[BIN_HDR_SIZE:])
    if len(payload) > MAX_BIN_PAYLOAD:
        raise LinkProtocolError("bad_payload", f"binary payload {len(payload)} > {MAX_BIN_PAYLOAD}")
    return btype, codec, stream_id, seq, payload
