"""Session state for the link gateway: resume buffer, credit ledger, TTL store."""

from __future__ import annotations

import asyncio
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Protocol

from ..protocol import SessionCodec, encode_envelope
from ..telemetry.rules import RulesEngine
from ..telemetry.state import RollingState

RESUME_BUFFER_LEN = 512


class _WsLike(Protocol):
    async def send(self, message: str | bytes) -> None: ...


@dataclass
class Session:
    """One suit's server-side session; survives socket loss for the store TTL."""

    id: str
    suit_id: str
    initial_credits: int
    codec: SessionCodec = field(default_factory=SessionCodec)
    buffer: deque[tuple[int, str]] = field(default_factory=lambda: deque(maxlen=RESUME_BUFFER_LEN))
    telemetry: RollingState = field(default_factory=RollingState)
    rules: RulesEngine | None = None
    last_rx_seq: int = 0
    ws: _WsLike | None = None
    disconnected_at: float | None = None
    tasks: set[asyncio.Task[None]] = field(default_factory=set)
    link_stats: dict[str, Any] = field(default_factory=dict)
    _send_lock: asyncio.Lock = field(default_factory=asyncio.Lock)
    _credits: dict[int, int] = field(default_factory=dict)
    _credit_events: dict[int, asyncio.Event] = field(default_factory=dict)
    _pending_grants: dict[int, int] = field(default_factory=dict)
    _next_stream_id: int = 0

    # --- downlink text ------------------------------------------------------

    async def send(self, msg_type: str, payload: dict[str, Any], buffer: bool = True) -> int:
        """Sequence, buffer for resume, and (best-effort) transmit a text message.

        Buffering is unconditional for normal traffic: messages produced while the
        socket is down are replayed on resume (§1 — bearer failover is lossless for
        control messages). hello_ack passes buffer=False (a stale ack must never be
        replayed into a later epoch). Returns the assigned seq.
        """
        env = self.codec.make(msg_type, payload)
        text = encode_envelope(env)
        if buffer:
            self.buffer.append((env["seq"], text))
        ws = self.ws
        if ws is not None:
            try:
                async with self._send_lock:
                    await ws.send(text)
            except Exception:
                pass  # connection died mid-send; message stays buffered for resume
        return env["seq"]

    async def send_binary(self, data: bytes) -> None:
        """Transmit a binary frame. Audio is never buffered/replayed (§1)."""
        ws = self.ws
        if ws is None:
            raise ConnectionError("session has no live connection")
        async with self._send_lock:
            await ws.send(data)

    async def replay_after(self, last_rx_seq: int) -> int:
        """Resend buffered messages with seq > last_rx_seq, in order."""
        ws = self.ws
        if ws is None:
            return 0
        n = 0
        for seq, text in list(self.buffer):
            if seq > last_rx_seq:
                async with self._send_lock:
                    await ws.send(text)
                n += 1
        return n

    # --- audio credit ledger (§2.7) ------------------------------------------

    def alloc_stream(self) -> int:
        self._next_stream_id += 1
        sid = self._next_stream_id
        self._credits[sid] = self.initial_credits + self._pending_grants.pop(sid, 0)
        return sid

    def credits(self, stream_id: int) -> int:
        return self._credits.get(stream_id, 0)

    def grant(self, stream_id: int, n: int) -> None:
        if n <= 0:
            return
        if stream_id in self._credits:
            self._credits[stream_id] += n
        else:
            # Grant arrived before the stream was allocated; hold it.
            self._pending_grants[stream_id] = self._pending_grants.get(stream_id, 0) + n
        ev = self._credit_events.get(stream_id)
        if ev is not None:
            ev.set()

    async def consume_credit(self, stream_id: int) -> None:
        """Block until one credit is available, then take it (never negative).

        Raises ConnectionError once the socket is gone: audio is never replayed
        (§1), so a stalled TTS sender must abort rather than wait out a TTL.
        """
        while True:
            if self.ws is None:
                raise ConnectionError("session disconnected")
            if self._credits.get(stream_id, 0) > 0:
                self._credits[stream_id] -= 1
                return
            ev = self._credit_events.setdefault(stream_id, asyncio.Event())
            ev.clear()
            await ev.wait()

    def close_stream(self, stream_id: int) -> None:
        self._credits.pop(stream_id, None)
        self._credit_events.pop(stream_id, None)

    # --- lifecycle ------------------------------------------------------------

    def attach(self, ws: _WsLike) -> None:
        self.ws = ws
        self.disconnected_at = None

    def detach(self) -> None:
        self.ws = None
        self.disconnected_at = time.monotonic()
        # Unblock any credit waiters so stream tasks can notice the dead socket.
        for ev in self._credit_events.values():
            ev.set()

    def spawn(self, coro: Any) -> asyncio.Task[None]:
        task = asyncio.ensure_future(coro)
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)
        return task

    def cancel_tasks(self) -> None:
        for task in list(self.tasks):
            task.cancel()


class SessionStore:
    """Sessions by id with TTL purge; expiry is evaluated on access (§1: TTL 120 s)."""

    def __init__(self, ttl_s: float = 120.0):
        self.ttl_s = ttl_s
        self._sessions: dict[str, Session] = {}

    def purge(self) -> None:
        now = time.monotonic()
        for sid, sess in list(self._sessions.items()):
            if sess.ws is None and sess.disconnected_at is not None:
                if now - sess.disconnected_at > self.ttl_s:
                    sess.cancel_tasks()
                    del self._sessions[sid]

    def get(self, session_id: str) -> Session | None:
        self.purge()
        return self._sessions.get(session_id)

    def add(self, session: Session) -> None:
        self.purge()
        self._sessions[session.id] = session

    def remove(self, session_id: str) -> None:
        sess = self._sessions.pop(session_id, None)
        if sess is not None:
            sess.cancel_tasks()

    def all(self) -> list[Session]:
        self.purge()
        return list(self._sessions.values())
