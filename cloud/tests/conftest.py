"""Shared fixtures for cloud_ai_core tests.

Starts the *real* gateway server (`cloud_ai_core.gateway.server.CloudServer`) in
process on an ephemeral port, with the mock inference engine and mock TTS
backend (docs/link-protocol.md is the normative reference; nothing here
reimplements the wire format — it drives the real server over a real
`websockets` client).
"""

from __future__ import annotations

import asyncio
import json
import time
from collections.abc import AsyncIterator, Awaitable, Callable
from contextlib import AsyncExitStack
from dataclasses import dataclass, field
from typing import Any

import pytest
import pytest_asyncio
from websockets.asyncio.client import ClientConnection, connect as ws_connect

from cloud_ai_core.config import Settings
from cloud_ai_core.gateway.server import CloudServer
from cloud_ai_core.protocol import PROTO_VERSION

TOKEN = "test-token"
SUIT_ID = "powersuit-test"


def make_settings(**overrides: Any) -> Settings:
    """Settings for a bench server: known token, mock engine/TTS, ephemeral port."""
    kwargs: dict[str, Any] = dict(
        host="127.0.0.1",
        port=0,
        tokens=(TOKEN,),
        engine="mock",
        tts="mock",
    )
    kwargs.update(overrides)
    return Settings(**kwargs)


@dataclass
class LinkTestClient:
    """Thin helper over a websockets connection for driving the JSON envelope
    and binary frame protocol (docs/link-protocol.md §2-§3) from tests."""

    ws: ClientConnection
    _seq: int = 0
    session_id: str | None = None
    hello_ack: dict[str, Any] | None = None

    def next_seq(self) -> int:
        self._seq += 1
        return self._seq

    async def send(
        self,
        msg_type: str,
        payload: dict[str, Any],
        *,
        seq: int | None = None,
        v: int = PROTO_VERSION,
        ts: float | None = None,
    ) -> int:
        s = seq if seq is not None else self.next_seq()
        env = {
            "v": v,
            "type": msg_type,
            "seq": s,
            "ts": time.time() if ts is None else ts,
            "payload": payload,
        }
        await self.ws.send(json.dumps(env))
        return s

    async def send_raw(self, raw: str) -> None:
        await self.ws.send(raw)

    async def send_binary(self, data: bytes) -> None:
        await self.ws.send(data)

    async def recv(self, timeout: float = 2.0) -> Any:
        raw = await asyncio.wait_for(self.ws.recv(), timeout)
        if isinstance(raw, (bytes, bytearray)):
            return bytes(raw)
        return json.loads(raw)

    async def recv_type(self, msg_type: str, timeout: float = 2.0, max_messages: int = 20) -> dict[str, Any]:
        """Receive messages, skipping anything that isn't the wanted text type
        (e.g. an unsolicited rules advisory racing the message under test)."""
        for _ in range(max_messages):
            msg = await self.recv(timeout)
            if isinstance(msg, dict) and msg.get("type") == msg_type:
                return msg
        raise AssertionError(f"no {msg_type!r} message seen within {max_messages} messages")

    async def hello(
        self,
        *,
        token: str = TOKEN,
        suit_id: str = SUIT_ID,
        proto: int = PROTO_VERSION,
        v: int = PROTO_VERSION,
        resume: dict[str, Any] | None = None,
        seq: int | None = None,
        ts: float | None = None,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "proto": proto,
            "suit_id": suit_id,
            "token": token,
            "agent": "test-client/0.1",
        }
        if resume is not None:
            payload["resume"] = resume
        await self.send("hello", payload, seq=seq, v=v, ts=ts)
        ack = await self.recv()
        if isinstance(ack, dict) and ack.get("type") == "hello_ack":
            self.hello_ack = ack
            self.session_id = ack["payload"]["session_id"]
        return ack

    async def close(self) -> None:
        await self.ws.close()


@pytest_asyncio.fixture
async def make_server() -> AsyncIterator[Callable[..., Awaitable[CloudServer]]]:
    """Factory fixture: `await make_server(**settings_overrides)` -> a started
    CloudServer on an ephemeral port. Every server made this way is stopped at
    teardown."""
    started: list[CloudServer] = []

    async def _make(**overrides: Any) -> CloudServer:
        srv = CloudServer(make_settings(**overrides))
        await srv.start()
        started.append(srv)
        return srv

    yield _make
    for srv in started:
        await srv.stop()


@pytest_asyncio.fixture
async def server(make_server: Callable[..., Awaitable[CloudServer]]) -> CloudServer:
    """A default bench server (default settings, mock engine/TTS)."""
    return await make_server()


@pytest_asyncio.fixture
async def connect() -> AsyncIterator[Callable[..., Awaitable[LinkTestClient]]]:
    """`await connect(server, **hello_kwargs)` -> a hello-completed client.
    `await connect(server, do_hello=False)` -> a bare connected client (for
    handshake-failure tests). Every connection opened this way is closed at
    teardown."""
    async with AsyncExitStack() as stack:

        async def _connect(srv: CloudServer, do_hello: bool = True, **hello_kwargs: Any) -> LinkTestClient:
            url = f"ws://{srv.settings.host}:{srv.port}/link"
            ws = await stack.enter_async_context(ws_connect(url, open_timeout=5))
            c = LinkTestClient(ws)
            if do_hello:
                ack = await c.hello(**hello_kwargs)
                assert ack.get("type") == "hello_ack", f"hello failed: {ack}"
            return c

        yield _connect


@pytest_asyncio.fixture
async def raw_client(server: CloudServer, connect: Callable[..., Awaitable[LinkTestClient]]) -> LinkTestClient:
    """A connected-but-not-helloed client against the default `server`."""
    return await connect(server, do_hello=False)


@pytest_asyncio.fixture
async def client(server: CloudServer, connect: Callable[..., Awaitable[LinkTestClient]]) -> LinkTestClient:
    """A fully-helloed client against the default `server`."""
    return await connect(server)
