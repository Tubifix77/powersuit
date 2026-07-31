"""Suit-side link client — no rclpy (docs/link-protocol.md).

Implements the Node 8 gateway behavior: hello/auth (token from env
POWERSUIT_LINK_TOKEN unless injected), session resume, envelope codec via
powersuit_proto.link, downlink WHITELIST enforcement with a rejected counter,
TTS credit management (grant blocks of 16 while the CAN-descent buffer sits
below 50 %), and WS protocol pings every 5 s (3 misses => reconnect).

`websockets` (>=17, asyncio implementation) is imported lazily so the module
stays importable in ROS-free unit contexts.
"""

from __future__ import annotations

import asyncio
import os
import random
import time
from typing import Any, Callable

from powersuit_proto import link as pslink

PING_INTERVAL_S = 5.0
PING_MISSES = 3
CREDIT_BLOCK = 16
CREDIT_LOW_WATER = 8          # grant when outstanding drops below this
BUFFER_GRANT_THRESHOLD = 0.5  # ...and downlink buffer fill is below this
BACKOFF_MIN_S = 1.0
BACKOFF_MAX_S = 32.0


class LinkClient:
    """One WS connection lifecycle manager for one bearer URL."""

    def __init__(
        self,
        url: str,
        token: str | None = None,
        *,
        suit_id: str = "powersuit-01",
        agent: str = "suit_cloud_gateway/0.1",
        ssl_context=None,
        on_advisory: Callable[[dict], None] | None = None,
        on_tts_meta: Callable[[dict], None] | None = None,
        on_tts_chunk: Callable[[int, int, int, bytes], None] | None = None,
        on_error_msg: Callable[[dict], None] | None = None,
        on_session: Callable[[str, dict], None] | None = None,  # event, info
        downlink_buffer_fill: Callable[[], float] | None = None,
        clock: Callable[[], float] = time.time,
    ):
        self.url = url
        self._token = token if token is not None else os.environ.get("POWERSUIT_LINK_TOKEN", "")
        self._suit_id = suit_id
        self._agent = agent
        self._ssl = ssl_context
        self._on_advisory = on_advisory
        self._on_tts_meta = on_tts_meta
        self._on_tts_chunk = on_tts_chunk
        self._on_error_msg = on_error_msg
        self._on_session = on_session
        self._buffer_fill = downlink_buffer_fill or (lambda: 0.0)
        self._clock = clock

        # Session state (survives reconnects for resume).
        self.session_id: str | None = None
        self.last_rx_seq = 0
        self.rejected = 0            # whitelist drops (link_stats.rejected)
        self.connected = False
        self.rtt_ms: float | None = None
        self._tx_seq = 0
        self._ws = None
        self._send_lock: asyncio.Lock | None = None

        # TTS credit accounting.
        self._credits_outstanding = 0
        self._active_stream: int | None = None

    # ------------------------------------------------------------------ tx
    def _envelope(self, msg_type: str, payload: dict) -> str:
        self._tx_seq += 1
        return pslink.encode_envelope(
            pslink.make_envelope(msg_type, self._tx_seq, payload, ts=self._clock()))

    async def send(self, msg_type: str, payload: dict) -> bool:
        ws = self._ws
        if ws is None or self._send_lock is None:
            return False
        try:
            async with self._send_lock:
                await ws.send(self._envelope(msg_type, payload))
            return True
        except Exception:
            return False

    async def send_telemetry(self, payload: dict) -> bool:
        return await self.send(pslink.T_TELEMETRY_BATCH, payload)

    async def send_voice_query(self, query_id: str, text: str, context: dict) -> bool:
        return await self.send(pslink.T_VOICE_QUERY, {
            "query_id": query_id, "text": text, "source": "helmet", "context": context})

    async def send_link_stats(self, bearer: str, rtt_ms: float, loss_pct: float,
                              switches: int) -> bool:
        return await self.send(pslink.T_LINK_STATS, {
            "bearer": bearer, "rtt_ms": rtt_ms, "loss_pct": loss_pct,
            "switches": switches, "rejected": self.rejected})

    async def send_audio_up(self, codec: int, stream_id: int, seq: int,
                            payload: bytes) -> bool:
        ws = self._ws
        if ws is None or self._send_lock is None:
            return False
        frame = pslink.pack_audio_frame(
            pslink.BTYPE_AUDIO_UP_CHUNK, codec, stream_id, seq, payload)
        try:
            async with self._send_lock:
                await ws.send(frame)
            return True
        except Exception:
            return False

    async def bye(self, reason: str = "shutdown") -> None:
        await self.send(pslink.T_BYE, {"reason": reason})
        ws = self._ws
        if ws is not None:
            try:
                await ws.close(1000)
            except Exception:
                pass

    # ------------------------------------------------------------------ rx
    def _handle_text(self, raw: str | bytes) -> dict | None:
        """Decode + whitelist-check one text frame. Returns env or None (rejected)."""
        try:
            env = pslink.decode_envelope(raw)
        except pslink.LinkProtocolError:
            self.rejected += 1
            return None
        if env["type"] not in pslink.DOWNLINK_WHITELIST:
            # Hard security boundary (§6): count and drop, never dispatch.
            self.rejected += 1
            return None
        seq = env["seq"]
        if seq > self.last_rx_seq:
            self.last_rx_seq = seq
        return env

    async def _dispatch(self, env: dict) -> None:
        t = env["type"]
        payload = env.get("payload", {})
        if t == pslink.T_ADVISORY and self._on_advisory:
            self._on_advisory(payload)
        elif t == pslink.T_TTS_META:
            state = payload.get("state")
            if state == "start":
                self._active_stream = payload.get("stream_id")
            elif state in ("end", "abort"):
                self._active_stream = None
                self._credits_outstanding = 0
            if self._on_tts_meta:
                self._on_tts_meta(payload)
        elif t == pslink.T_AUDIO_CREDIT:
            pass  # echo type, informational
        elif t == pslink.T_ERROR:
            if self._on_error_msg:
                self._on_error_msg(payload)
        elif t == pslink.T_BYE:
            raise _ServerBye(payload.get("reason", ""))

    def _handle_binary(self, data: bytes) -> None:
        try:
            btype, codec, stream_id, seq, payload = pslink.parse_audio_frame(data)
        except pslink.LinkProtocolError:
            self.rejected += 1
            return
        if btype != pslink.BTYPE_TTS_CHUNK:
            self.rejected += 1  # only TTS_CHUNK is whitelisted downlink binary
            return
        if self._credits_outstanding > 0:
            self._credits_outstanding -= 1
        if self._on_tts_chunk:
            self._on_tts_chunk(codec, stream_id, seq, payload)

    async def _maybe_grant_credits(self) -> None:
        if self._active_stream is None:
            return
        if (self._credits_outstanding < CREDIT_LOW_WATER
                and self._buffer_fill() < BUFFER_GRANT_THRESHOLD):
            ok = await self.send(pslink.T_AUDIO_CREDIT, {
                "stream_id": self._active_stream, "credits": CREDIT_BLOCK})
            if ok:
                self._credits_outstanding += CREDIT_BLOCK

    # ------------------------------------------------------------------ lifecycle
    async def run_once(self) -> str:
        """One connect->hello->serve cycle. Returns a close reason string."""
        from websockets.asyncio.client import connect  # lazy: keep module light

        self._send_lock = asyncio.Lock()
        try:
            async with connect(
                self.url,
                ssl=self._ssl,
                ping_interval=PING_INTERVAL_S,
                ping_timeout=PING_INTERVAL_S * PING_MISSES,
                max_size=4 * 1024 * 1024,
            ) as ws:
                self._ws = ws
                hello: dict[str, Any] = {
                    "proto": pslink.PROTO_VERSION,
                    "suit_id": self._suit_id,
                    "token": self._token,
                    "agent": self._agent,
                }
                if self.session_id:
                    hello["resume"] = {"session_id": self.session_id,
                                       "last_rx_seq": self.last_rx_seq}
                async with self._send_lock:
                    await ws.send(self._envelope(pslink.T_HELLO, hello))

                raw = await asyncio.wait_for(ws.recv(), timeout=10.0)
                env = self._handle_text(raw) if not isinstance(raw, (bytes, bytearray)) \
                    else None
                if env is None or env["type"] != pslink.T_HELLO_ACK:
                    return "bad hello_ack"
                ack = env["payload"]
                self.session_id = ack.get("session_id", self.session_id)
                self._credits_outstanding = int(ack.get("audio_credits", 0))
                self.connected = True
                if self._on_session:
                    self._on_session("connected", ack)

                async for raw in ws:
                    if isinstance(raw, (bytes, bytearray)):
                        self._handle_binary(bytes(raw))
                        await self._maybe_grant_credits()
                    else:
                        env = self._handle_text(raw)
                        if env is not None:
                            await self._dispatch(env)
                    self.rtt_ms = (ws.latency or 0.0) * 1000.0 if ws.latency else self.rtt_ms
                return "closed"
        except _ServerBye as exc:
            return f"bye: {exc}"
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # connect refused, TLS, timeout, protocol...
            return f"error: {type(exc).__name__}: {exc}"
        finally:
            was = self.connected
            self.connected = False
            self._ws = None
            if was and self._on_session:
                self._on_session("disconnected", {})

    async def run(self, stop: asyncio.Event) -> None:
        """Reconnect loop with jittered exponential backoff (1 s -> 32 s)."""
        backoff = BACKOFF_MIN_S
        while not stop.is_set():
            reason = await self.run_once()
            if stop.is_set():
                break
            if reason == "closed":
                backoff = BACKOFF_MIN_S  # normal close after a good session
            delay = backoff * (0.5 + random.random())
            backoff = min(backoff * 2.0, BACKOFF_MAX_S)
            try:
                await asyncio.wait_for(stop.wait(), timeout=delay)
            except asyncio.TimeoutError:
                pass


class _ServerBye(Exception):
    pass
