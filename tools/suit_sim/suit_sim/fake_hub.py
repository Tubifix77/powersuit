"""Node 7 stand-in: the dual-CAN gateway and the SPI slave.

Implements the routing table in docs/network-map.md §5 and the 512-byte framing
in §6. The SPI side is a TCP server speaking the exact contract the real C++
bridge's mock transport speaks:

    accept one connection, then forever:
        receive exactly 512 bytes   (master frame, from Node 8)
        send    exactly 512 bytes   (slave frame,  to Node 8)

The BMS emulation models the honest trip chain from docs/safety.md §4: the
comparator opens the pack in hardware and the firmware's job is only to observe
and shout, so `short_circuit()` puts ESTOP on both buses immediately rather than
waiting for anything to be polled.
"""

from __future__ import annotations

import asyncio

from powersuit_proto import can_id, spi_frame, wire
from powersuit_proto.can_id import Cls, MsgType, Node

from .virtual_can import CanFrame, VirtualBus

XFER = spi_frame.SPI_XFER_SIZE
MAX_RECORDS = spi_frame.SPI_MAX_RECORDS


def _bus_port_of(node: int) -> int | None:
    if node in can_id.BUS1_NODES:
        return spi_frame.BUS_CAN1
    if node in can_id.BUS2_NODES:
        return spi_frame.BUS_CAN2
    return None


class FakeHub:
    def __init__(self, bus1: VirtualBus, bus2: VirtualBus, *, host: str = "127.0.0.1") -> None:
        self.node_id = Node.HUB
        self.bus1 = bus1
        self.bus2 = bus2
        self.host = host
        self.port = 0

        self.soc_pct = 84
        self.estop_latched = False
        self.fault_bits = wire.BMSF_COMP_ARMED

        # Test hooks.
        self.corrupt_frames = 0
        self.drop_audio_every = 0

        self._uplink: list[spi_frame.CanRecord] = []
        self._audio_up_seen = 0
        self._tx_seq = 0
        self._rx_seq: int | None = None
        self.overflow = False
        self.downlink_records = 0
        self.seq_gaps = 0

        self._server: asyncio.AbstractServer | None = None
        self._tasks: list[asyncio.Task[None]] = []
        # The hub sits on both buses, so anything it injects would come straight
        # back through its own receive callback. Bus delivery is synchronous, so
        # a plain re-entrancy flag is enough to break the loop.
        self._forwarding = False

        bus1.attach(self.node_id, lambda f: self._on_bus(f, spi_frame.BUS_CAN1))
        bus2.attach(self.node_id, lambda f: self._on_bus(f, spi_frame.BUS_CAN2))

    async def start(self) -> None:
        self._server = await asyncio.start_server(self._serve, self.host, 0)
        self.port = self._server.sockets[0].getsockname()[1]
        self._tasks = [asyncio.ensure_future(self._bms_loop())]

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        for task in self._tasks:
            try:
                await task
            except asyncio.CancelledError:
                pass
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
        self.bus1.detach(self.node_id)
        self.bus2.detach(self.node_id)

    # --- CAN side -------------------------------------------------------------

    def _emit(self, bus: VirtualBus, frame: CanFrame) -> None:
        self._forwarding = True
        try:
            bus.send(frame)
        finally:
            self._forwarding = False

    def _on_bus(self, frame: CanFrame, origin: int) -> None:
        """Apply the §5 routing policy to a frame arriving from one bus."""
        if self._forwarding:
            return
        fields = can_id.unpack(frame.id)
        other = self.bus2 if origin == spi_frame.BUS_CAN1 else self.bus1

        if fields.cls == Cls.SAFETY:
            # Cut-through: both buses and the orchestrator, unconditionally.
            self._emit(other, frame)
            if fields.type == MsgType.ESTOP:
                self.estop_latched = True
            self._queue(frame, origin)
            return

        if fields.cls == Cls.AUDIO:
            # Uplink audio is bus 1 only, and never crosses to bus 2.
            if origin != spi_frame.BUS_CAN1:
                return
            if fields.type == MsgType.AUDIO_UP:
                self._audio_up_seen += 1
                if self.drop_audio_every and self._audio_up_seen % self.drop_audio_every == 0:
                    return
            self._queue(frame, origin)
            return

        if fields.cls in (Cls.TELEM, Cls.XRCE, Cls.MGMT, Cls.CONTROL):
            # Never cross-bus: telemetry and XRCE go up the SPI link only.
            self._queue(frame, origin)

    def _queue(self, frame: CanFrame, bus: int) -> None:
        if len(self._uplink) >= 512:
            self.overflow = True
            return
        self._uplink.append(
            spi_frame.CanRecord(
                id=frame.id,
                bus=bus,
                dlc=len(frame.data),
                ts_ms=int(asyncio.get_event_loop().time() * 1000) & 0xFFFF,
                data=frame.data,
            )
        )

    def _emit_local(self, mtype: int, payload: bytes, low: int = 0) -> None:
        """Hub-originated telemetry goes straight up the SPI link; it never
        occupies CAN bandwidth (§5)."""
        rec = spi_frame.CanRecord(
            id=can_id.pack(Cls.TELEM, self.node_id, Node.ORCH, mtype, low),
            bus=spi_frame.BUS_HUB_LOCAL,
            dlc=len(payload),
            ts_ms=int(asyncio.get_event_loop().time() * 1000) & 0xFFFF,
            data=payload,
        )
        if len(self._uplink) < 512:
            self._uplink.append(rec)
        else:
            self.overflow = True

    # --- BMS ------------------------------------------------------------------

    async def _bms_loop(self) -> None:
        while True:
            summary = wire.BmsSummary(
                pack_cV=5020,
                current_cA=-1210,
                soc_pct=self.soc_pct,
                temp_max_C=38,
                fault_bits=self.fault_bits,
            )
            self._emit_local(MsgType.BMS_SUMMARY, summary.pack())
            await asyncio.sleep(0.1)

    def short_circuit(self) -> None:
        """Hardware comparator opens the pack; firmware observes and shouts."""
        self.estop_latched = True
        self.fault_bits |= wire.BMSF_SHORT_LATCH
        estop = wire.Estop(
            cause=wire.EstopCause.BMS_SHORT, origin_node=self.node_id, seq=1, uptime_ms=0
        )
        for i in range(wire.ESTOP_REPEAT):
            frame = CanFrame(
                id=can_id.pack(
                    Cls.SAFETY, self.node_id, Node.BROADCAST, MsgType.ESTOP, i
                ),
                data=estop.pack(),
            )
            self._emit(self.bus1, frame)
            self._emit(self.bus2, frame)
        self._queue(
            CanFrame(
                id=can_id.pack(Cls.SAFETY, self.node_id, Node.ORCH, MsgType.ESTOP, 0),
                data=estop.pack(),
            ),
            spi_frame.BUS_HUB_LOCAL,
        )

    # --- SPI side -------------------------------------------------------------

    async def _serve(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        try:
            while True:
                master = await reader.readexactly(XFER)
                self._consume_master(master)
                writer.write(self._build_slave())
                await writer.drain()
        except (asyncio.IncompleteReadError, ConnectionResetError, asyncio.CancelledError):
            pass
        finally:
            writer.close()

    def _consume_master(self, buf: bytes) -> None:
        if not any(buf):
            return   # idle frame, not an error
        try:
            _flags, seq, records = spi_frame.parse_frame(buf)
        except spi_frame.SpiFrameError:
            return
        if self._rx_seq is not None and seq != ((self._rx_seq + 1) & 0xFF):
            self.seq_gaps += 1
        self._rx_seq = seq

        for rec in records:
            self.downlink_records += 1
            fields = can_id.unpack(rec.id)
            frame = CanFrame(id=rec.id, data=rec.data[: rec.dlc])

            if fields.cls == Cls.SAFETY:
                # Downlink safety reaches both buses regardless of bus tag.
                self._emit(self.bus1, frame)
                self._emit(self.bus2, frame)
                if fields.type == MsgType.ESTOP:
                    self.estop_latched = True
                elif fields.type == MsgType.CLEAR_ESTOP:
                    clear = wire.ClearEstop.unpack(rec.data)
                    if clear.magic == wire.CLEAR_ESTOP_MAGIC:
                        self.estop_latched = False
                        self.fault_bits &= ~wire.BMSF_SHORT_LATCH
                continue

            if rec.bus == spi_frame.BUS_HUB_LOCAL or fields.dst == self.node_id:
                continue   # consumed locally (LED patterns, flow control)

            port = _bus_port_of(fields.dst)
            if port == spi_frame.BUS_CAN1:
                self._emit(self.bus1, frame)
            elif port == spi_frame.BUS_CAN2:
                self._emit(self.bus2, frame)

    def _build_slave(self) -> bytes:
        batch = self._uplink[:MAX_RECORDS]
        del self._uplink[: len(batch)]

        flags = 0
        if self.estop_latched:
            flags |= spi_frame.SPIF_ESTOP_LATCHED
        if self.overflow:
            flags |= spi_frame.SPIF_OVERFLOW
            self.overflow = False
        if self._uplink:
            flags |= spi_frame.SPIF_MORE_PENDING

        frame = bytearray(spi_frame.build_frame(flags, self._tx_seq, batch))
        self._tx_seq = (self._tx_seq + 1) & 0xFF

        if self.corrupt_frames > 0:
            # Flip a bit the CRC actually covers. With no records in the frame,
            # everything past the header is uncovered padding, so target the
            # header instead — otherwise an empty frame would sail through and
            # the test would be measuring nothing.
            self.corrupt_frames -= 1
            if batch:
                frame[spi_frame.SPI_HDR_SIZE + 12] ^= 0x40
            else:
                frame[5] ^= 0x40   # seq: always inside the CRC
        return bytes(frame)

    def corrupt_next_frames(self, n: int) -> None:
        self.corrupt_frames = n
