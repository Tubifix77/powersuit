"""Wires the whole simulated suit together, optionally including the real
Node 9 server running in-process over a real WebSocket.

Nothing here is a mock of the protocol — the frames, the framing, the codec and
the cloud envelope all come from `powersuit_proto`, and the cloud is the actual
`cloud_ai_core` application. Only the silicon and the wires are fake.
"""

from __future__ import annotations

import asyncio
import contextlib
import socket
from typing import Any, Callable

from powersuit_proto.can_id import Node

from .bridge_lite import BridgeLite, CloudLink
from .fake_flight import FakeFlight
from .fake_helmet import FakeHelmet
from .fake_hub import FakeHub
from .fake_limb import FakeLimb
from .virtual_can import VirtualBus

LIMB_NODES = (Node.ARM_R, Node.ARM_L, Node.LEG_R, Node.LEG_L)
CLOUD_TOKEN = "dev-token"


async def until(predicate: Callable[[], bool], timeout: float = 2.0,
                interval: float = 0.005) -> bool:
    """Poll until `predicate` holds. Returns False on timeout so callers can
    assert with a useful message instead of a bare TimeoutError."""
    loop = asyncio.get_event_loop()
    deadline = loop.time() + timeout
    while loop.time() < deadline:
        if predicate():
            return True
        await asyncio.sleep(interval)
    return predicate()


async def elapsed_until(predicate: Callable[[], bool], timeout: float = 2.0,
                        interval: float = 0.005) -> float | None:
    """Like `until`, but returns how long it took — used by the ordering tests."""
    loop = asyncio.get_event_loop()
    start = loop.time()
    if await until(predicate, timeout, interval):
        return loop.time() - start
    return None


class SuitHarness:
    def __init__(self, *, with_cloud: bool = False, spi_rate_hz: float = 200.0) -> None:
        self.with_cloud = with_cloud
        self._spi_rate = spi_rate_hz

        self.bus1 = VirtualBus("bus1")
        self.bus2 = VirtualBus("bus2")
        self.hub = FakeHub(self.bus1, self.bus2)
        self.limbs: dict[int, FakeLimb] = {}
        self.helmet = FakeHelmet(self.bus1)
        self.flight = FakeFlight(self.bus2)
        self.bridge: BridgeLite | None = None
        self.cloud_server: Any = None
        self.cloud_port = 0

        for node in LIMB_NODES:
            bus = self.bus1 if node in (Node.ARM_R, Node.ARM_L) else self.bus2
            self.limbs[node] = FakeLimb(node, bus)

    async def __aenter__(self) -> "SuitHarness":
        if self.with_cloud:
            await self._start_cloud()

        await self.hub.start()
        for limb in self.limbs.values():
            await limb.start()
        await self.helmet.start()
        await self.flight.start()

        self.bridge = BridgeLite("127.0.0.1", self.hub.port, spi_rate_hz=self._spi_rate)
        await self.bridge.start()

        if self.with_cloud:
            self.bridge.cloud = CloudLink(
                f"ws://127.0.0.1:{self.cloud_port}", CLOUD_TOKEN, self.bridge
            )
            await self.bridge.cloud.connect()
        return self

    async def __aexit__(self, *exc: object) -> None:
        if self.bridge is not None:
            await self.bridge.stop()
        await self.helmet.stop()
        await self.flight.stop()
        for limb in self.limbs.values():
            await limb.stop()
        await self.hub.stop()
        await self._stop_cloud()

    # --- cloud ----------------------------------------------------------------

    async def _start_cloud(self) -> None:
        from dataclasses import replace

        from cloud_ai_core.config import Settings
        from cloud_ai_core.gateway.server import CloudServer

        # A concrete port, not 0: restart() re-runs start(), and an ephemeral
        # port would move under the client mid-failover.
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            free_port = probe.getsockname()[1]

        settings = replace(
            Settings(),
            host="127.0.0.1",
            port=free_port,
            tokens=(CLOUD_TOKEN,),
            engine="mock",
            tts="mock",
        )
        self.cloud_server = CloudServer(settings)
        await self.cloud_server.start()
        self.cloud_port = self.cloud_server.port

    async def _stop_cloud(self) -> None:
        if self.cloud_server is not None:
            with contextlib.suppress(Exception):
                await self.cloud_server.stop()
            self.cloud_server = None

    async def restart_cloud(self) -> None:
        """Bounce the listener while keeping the session store alive — which is
        exactly what a bearer failover looks like from the suit's side, and what
        makes `resume` meaningful."""
        assert self.cloud_server is not None
        await self.cloud_server.restart()
        self.cloud_port = self.cloud_server.port

    # --- convenience ----------------------------------------------------------

    async def arm_all(self, timeout: float = 2.0) -> bool:
        """Drive every limb from STANDBY to OPERATIONAL the way the orchestrator
        does: mode intent, then a steady command stream on top of heartbeats."""
        from powersuit_proto import wire

        assert self.bridge is not None
        for node in self.limbs:
            self.bridge.send_mode(node, wire.State.OPERATIONAL)

        async def pump() -> None:
            while True:
                for node in self.limbs:
                    self.bridge.send_joint_cmd(node, 0, 100)
                await asyncio.sleep(0.02)

        pump_task = asyncio.ensure_future(pump())
        try:
            return await until(
                lambda: all(limb.state == wire.State.OPERATIONAL
                            for limb in self.limbs.values()),
                timeout,
            )
        finally:
            pump_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await pump_task
