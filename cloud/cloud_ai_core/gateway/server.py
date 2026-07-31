"""WebSocket link server (docs/link-protocol.md §1-§3, §7).

Connection lifecycle: hello within 5 s -> hello_ack (or close 4001 bad token /
4002 bad version), optional resume replay, then the dispatch loop. Text frames
are JSON envelopes gated by the uplink whitelist; binary frames are AUDIO_UP
chunks feeding the server-side ASR path.
"""

from __future__ import annotations

import asyncio
import logging
import ssl
from dataclasses import dataclass, field
from typing import Any

from websockets.asyncio.server import Server, ServerConnection, serve
from websockets.exceptions import ConnectionClosed

from ..asr import MockAsr
from ..config import Settings
from ..inference.engine import EngineBase
from ..inference.mock_engine import MockEngine
from ..inference.openai_compat_engine import OpenAiCompatEngine
from ..protocol import (
    BTYPE_AUDIO_UP_CHUNK,
    CLOSE_AUTH_FAILED,
    CLOSE_BAD_VERSION,
    LinkProtocolError,
    PROTO_VERSION,
    T_ADVISORY,
    T_AUDIO_CREDIT,
    T_BYE,
    T_ERROR,
    T_HELLO,
    T_HELLO_ACK,
    T_LINK_STATS,
    T_TELEMETRY_BATCH,
    T_VOICE_QUERY,
    decode_envelope,
    error_payload,
    parse_audio_frame,
    validate_uplink,
)
from ..rag.ingest import ingest_dir
from ..rag.store import VectorStore
from ..strategy import handle_voice_query
from ..telemetry.rules import RulesEngine
from ..tts.base import TtsBase
from ..tts.mock_tts import MockTts
from .auth import check_token, new_session_id
from .session import Session, SessionStore

log = logging.getLogger(__name__)

CAPABILITIES = ["rag", "tts", "advisory"]


def build_engine(settings: Settings) -> EngineBase:
    if settings.engine == "openai":
        return OpenAiCompatEngine(
            settings.openai_base_url, settings.openai_model, settings.openai_api_key
        )
    return MockEngine()


def build_tts(settings: Settings) -> TtsBase:
    if settings.tts == "piper":
        from ..tts.piper_tts import PiperTts

        return PiperTts(settings.piper_model or "")
    return MockTts()


@dataclass
class _UplinkAudio:
    codec: int = 0
    chunks: list[bytes] = field(default_factory=list)
    timer: asyncio.Task[None] | None = None
    claimed: bool = False

    def data(self) -> bytes:
        return b"".join(self.chunks)


class CloudServer:
    def __init__(
        self,
        settings: Settings,
        engine: EngineBase | None = None,
        store: VectorStore | None = None,
        tts: TtsBase | None = None,
        asr: MockAsr | None = None,
    ):
        self.settings = settings
        self.engine = engine if engine is not None else build_engine(settings)
        self.store = store if store is not None else ingest_dir(settings.rag_docs_dir)
        self.tts = tts if tts is not None else build_tts(settings)
        self.asr = asr if asr is not None else MockAsr()
        self.sessions = SessionStore(settings.session_ttl_s)
        self._server: Server | None = None
        self._bound_port: int | None = None
        self._audio_bufs: dict[tuple[str, int], _UplinkAudio] = {}

    # --- lifecycle -------------------------------------------------------------

    @property
    def port(self) -> int:
        if self._bound_port is None:
            raise RuntimeError("server not started")
        return self._bound_port

    def _ssl_context(self) -> ssl.SSLContext | None:
        s = self.settings
        if not (s.tls_cert and s.tls_key):
            return None
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.minimum_version = ssl.TLSVersion.TLSv1_3
        ctx.load_cert_chain(s.tls_cert, s.tls_key)
        return ctx

    async def start(self) -> None:
        port = self._bound_port if self._bound_port is not None else self.settings.port
        self._server = await serve(
            self._handler, self.settings.host, port, ssl=self._ssl_context()
        )
        self._bound_port = self._server.sockets[0].getsockname()[1]
        log.info("cloud_ai_core listening on %s:%d", self.settings.host, self._bound_port)

    async def stop(self, final: bool = True) -> None:
        """Graceful shutdown. final=False keeps the session store (restart)."""
        if self._server is not None:
            self._server.close(close_connections=True)
            await self._server.wait_closed()
            self._server = None
        if final:
            for sess in self.sessions.all():
                sess.cancel_tasks()
            self.sessions = SessionStore(self.settings.session_ttl_s)
        await asyncio.sleep(0)  # let close callbacks/detach run

    async def restart(self) -> None:
        """Close the listener and every connection but keep sessions (TTL applies)."""
        await self.stop(final=False)
        await self.start()

    async def serve_forever(self) -> None:
        await self.start()
        assert self._server is not None
        try:
            await self._server.wait_closed()
        finally:
            await self.stop()

    async def push_advisory(
        self, title: str, body: str, severity: str = "notice", advisory_id: str = "ops-1"
    ) -> None:
        """Operator/ops broadcast to every live session; buffered for offline ones."""
        for sess in self.sessions.all():
            await sess.send(
                T_ADVISORY,
                {"advisory_id": advisory_id, "severity": severity, "title": title, "body": body},
            )

    # --- connection handling -----------------------------------------------------

    async def _handler(self, ws: ServerConnection) -> None:
        session: Session | None = None
        try:
            session = await self._handshake(ws)
            if session is None:
                return
            await self._dispatch_loop(ws, session)
        except (ConnectionClosed, asyncio.IncompleteReadError):
            pass
        except Exception:
            log.exception("connection handler failed")
        finally:
            if session is not None and session.ws is ws:
                session.detach()

    async def _handshake(self, ws: ServerConnection) -> Session | None:
        try:
            raw = await asyncio.wait_for(ws.recv(), self.settings.hello_timeout_s)
        except (TimeoutError, asyncio.TimeoutError):
            await ws.close(CLOSE_BAD_VERSION, "hello timeout")
            return None
        except ConnectionClosed:
            return None
        if isinstance(raw, (bytes, bytearray)):
            await ws.close(CLOSE_BAD_VERSION, "first frame must be hello text")
            return None
        try:
            env = decode_envelope(raw)
        except LinkProtocolError as exc:
            code = CLOSE_BAD_VERSION
            await ws.close(code, f"bad hello: {exc}")
            return None
        payload = env["payload"]
        if env["type"] != T_HELLO or payload.get("proto") != PROTO_VERSION:
            await ws.close(CLOSE_BAD_VERSION, "unsupported protocol")
            return None
        if not check_token(payload.get("token"), self.settings.tokens):
            await ws.close(CLOSE_AUTH_FAILED, "auth failed")
            return None

        suit_id = str(payload.get("suit_id", ""))
        session: Session | None = None
        replay_from: int | None = None
        resume = payload.get("resume")
        if isinstance(resume, dict):
            old = self.sessions.get(str(resume.get("session_id", "")))
            if old is not None and old.suit_id == suit_id and old.ws is None:
                session = old
                replay_from = int(resume.get("last_rx_seq", 0))
        if session is None:
            session = Session(
                id=new_session_id(),
                suit_id=suit_id,
                initial_credits=self.settings.initial_credits,
                rules=RulesEngine(self.settings.rules),
            )
            self.sessions.add(session)
        session.attach(ws)
        session.last_rx_seq = max(session.last_rx_seq, env["seq"])

        await session.send(
            T_HELLO_ACK,
            {
                "session_id": session.id,
                "resume_from": 0 if replay_from is None else replay_from + 1,
                "ts_echo": env["ts"],
                "audio_credits": self.settings.initial_credits,
                "heartbeat_s": self.settings.heartbeat_s,
                "capabilities": list(CAPABILITIES),
            },
            buffer=False,
        )
        if replay_from is not None:
            n = await session.replay_after(replay_from)
            log.info("session %s resumed; replayed %d messages", session.id, n)
        return session

    async def _dispatch_loop(self, ws: ServerConnection, session: Session) -> None:
        async for raw in ws:
            if isinstance(raw, (bytes, bytearray)):
                await self._on_binary(session, bytes(raw))
                continue
            try:
                env = decode_envelope(raw)
                validate_uplink(env)
            except LinkProtocolError as exc:
                code = exc.code if exc.code in ("unknown_type", "bad_payload") else "bad_payload"
                await session.send(T_ERROR, error_payload(code, str(exc)))
                continue
            session.last_rx_seq = max(session.last_rx_seq, env["seq"])
            msg_type: str = env["type"]
            payload: dict[str, Any] = env["payload"]
            if msg_type == T_HELLO:
                await session.send(
                    T_ERROR, error_payload("bad_payload", "duplicate hello", env["seq"])
                )
            elif msg_type == T_TELEMETRY_BATCH:
                session.telemetry.update(payload)
                assert session.rules is not None
                for adv in session.rules.evaluate(session.telemetry):
                    await session.send(T_ADVISORY, adv)
            elif msg_type == T_VOICE_QUERY:
                self._start_voice(session, dict(payload))
            elif msg_type == T_AUDIO_CREDIT:
                session.grant(int(payload.get("stream_id", 0)), int(payload.get("credits", 0)))
            elif msg_type == T_LINK_STATS:
                session.link_stats = payload
            elif msg_type == T_BYE:
                self.sessions.remove(session.id)
                await ws.close(1000, "bye")
                return

    # --- uplink audio / ASR (link-protocol §2.4) ----------------------------------

    async def _on_binary(self, session: Session, raw: bytes) -> None:
        try:
            btype, codec, stream_id, _seq, payload = parse_audio_frame(raw)
        except LinkProtocolError as exc:
            await session.send(T_ERROR, error_payload("bad_payload", str(exc)))
            return
        if btype != BTYPE_AUDIO_UP_CHUNK:
            await session.send(
                T_ERROR, error_payload("bad_payload", f"unexpected binary type 0x{btype:02X}")
            )
            return
        key = (session.id, stream_id)
        entry = self._audio_bufs.setdefault(key, _UplinkAudio())
        entry.codec = codec
        entry.chunks.append(payload)
        if entry.timer is not None:
            entry.timer.cancel()
        entry.timer = session.spawn(self._asr_silence_timeout(session, stream_id))

    async def _asr_silence_timeout(self, session: Session, stream_id: int) -> None:
        """Uplink audio streams are delimited by the voice_query{stream_id} that
        follows the chunks; if none arrives, 1.5 s of silence closes the stream
        and the server synthesizes the voice_query itself."""
        await asyncio.sleep(self.settings.asr_silence_s)
        entry = self._audio_bufs.pop((session.id, stream_id), None)
        if entry is None or entry.claimed:
            return
        text = self.asr.transcribe(entry.data(), entry.codec)
        self._start_voice(
            session,
            {"query_id": f"asr-{stream_id}", "text": text, "source": "helmet",
             "stream_id": stream_id},
        )

    def _claim_audio(self, session: Session, payload: dict[str, Any]) -> dict[str, Any]:
        sid = payload.get("stream_id")
        if isinstance(sid, int):
            entry = self._audio_bufs.pop((session.id, sid), None)
            if entry is not None:
                entry.claimed = True
                if entry.timer is not None:
                    entry.timer.cancel()
                if not payload.get("text"):
                    payload["text"] = self.asr.transcribe(entry.data(), entry.codec)
        return payload

    def _start_voice(self, session: Session, payload: dict[str, Any]) -> None:
        payload = self._claim_audio(session, payload)
        session.spawn(
            handle_voice_query(session, payload, self.engine, self.store, self.tts)
        )
