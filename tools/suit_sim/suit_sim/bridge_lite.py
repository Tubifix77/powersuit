"""Node 8 stand-in: SPI master, heartbeat authority, and cloud client.

This mirrors the responsibilities of the real C++ `suit_canspi_bridge` closely
enough to exercise the protocol end to end. Two properties it shares with the
real thing and must never lose:

  * the heartbeat is gated, not scheduled — if the SPI link is stale or an
    e-stop is latched, beats simply stop and the limbs fall limp (docs/safety.md §2);
  * the cloud downlink is filtered through `link.DOWNLINK_WHITELIST`, so nothing
    the cloud says can reach an actuator (docs/link-protocol.md §6).
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import time
from typing import Any

import websockets

from powersuit_proto import adpcm, can_id, link, spi_frame, wire
from powersuit_proto.can_id import Cls, MsgType, Node

XFER = spi_frame.SPI_XFER_SIZE
HEARTBEAT_PERIOD_S = wire.HEARTBEAT_PERIOD_MS / 1000.0
TRANSPORT_STALE_S = 0.1


class BridgeLite:
    def __init__(self, host: str, port: int, *, spi_rate_hz: float = 200.0) -> None:
        self.host = host
        self.port = port
        self._period = 1.0 / spi_rate_hz

        # Suit state assembled from uplink telemetry.
        self.telemetry: dict[str, Any] = {}
        self.bms: dict[str, Any] = {}
        self.estop_latched = False
        self.heartbeat_enabled = True
        self.hb_seq = 0
        self.beats_sent = 0

        # Counters mirroring the real bridge's diagnostics.
        self.crc_errors = 0
        self.seq_gaps = 0
        self.frames_ok = 0

        # Audio uplink reassembly.
        self._audio_dec = adpcm.AdpcmState()
        self._audio_locked = False
        self._audio_expect = 0
        self.audio_gaps = 0
        self.uplink_pcm: list[int] = []

        # XRCE per-source reassembly (the C++ bridge writes these to PTYs).
        self.xrce_streams: dict[int, bytearray] = {}

        # Cloud session.
        self.cloud: CloudLink | None = None

        self._downlink: list[spi_frame.CanRecord] = []
        self._tx_seq = 0
        self._rx_seq: int | None = None
        self._last_good = 0.0
        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._tasks: list[asyncio.Task[None]] = []

    # --- lifecycle ------------------------------------------------------------

    async def start(self) -> None:
        self._reader, self._writer = await asyncio.open_connection(self.host, self.port)
        self._last_good = asyncio.get_event_loop().time()
        self._tasks = [
            asyncio.ensure_future(self._spi_loop()),
            asyncio.ensure_future(self._heartbeat_loop()),
        ]

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        for task in self._tasks:
            with contextlib.suppress(asyncio.CancelledError):
                await task
        if self.cloud is not None:
            await self.cloud.close()
        if self._writer is not None:
            self._writer.close()
            with contextlib.suppress(Exception):
                await self._writer.wait_closed()

    # --- SPI ------------------------------------------------------------------

    @property
    def transport_fresh(self) -> bool:
        return (asyncio.get_event_loop().time() - self._last_good) < TRANSPORT_STALE_S

    async def _spi_loop(self) -> None:
        assert self._reader is not None and self._writer is not None
        while True:
            batch = self._downlink[: spi_frame.SPI_MAX_RECORDS]
            del self._downlink[: len(batch)]
            self._writer.write(spi_frame.build_frame(0, self._tx_seq, batch))
            self._tx_seq = (self._tx_seq + 1) & 0xFF
            await self._writer.drain()

            slave = await self._reader.readexactly(XFER)
            self._consume_slave(slave)
            await asyncio.sleep(self._period)

    def _consume_slave(self, buf: bytes) -> None:
        try:
            flags, seq, records = spi_frame.parse_frame(buf)
        except spi_frame.SpiFrameError:
            # A corrupted frame is dropped whole: no half-parsed record may
            # reach the ROS graph.
            self.crc_errors += 1
            return

        if self._rx_seq is not None and seq != ((self._rx_seq + 1) & 0xFF):
            self.seq_gaps += 1
        self._rx_seq = seq
        self._last_good = asyncio.get_event_loop().time()
        self.frames_ok += 1
        self.estop_latched = bool(flags & spi_frame.SPIF_ESTOP_LATCHED)

        for rec in records:
            self._on_record(rec)

    def _on_record(self, rec: spi_frame.CanRecord) -> None:
        f = can_id.unpack(rec.id)
        payload = rec.data[: rec.dlc]

        if f.cls == Cls.TELEM:
            self._on_telem(f, payload)
        elif f.cls == Cls.AUDIO:
            self._on_audio(f, payload)
        elif f.cls == Cls.XRCE:
            self.xrce_streams.setdefault(f.src, bytearray()).extend(payload)
        elif f.cls == Cls.SAFETY and f.type == MsgType.ESTOP:
            self.estop_latched = True

    def _on_telem(self, f: can_id.CanId, payload: bytes) -> None:
        src = f.src
        if f.type == MsgType.JOINT_STATE:
            js = wire.JointState.unpack(payload)
            self.telemetry.setdefault(src, {}).setdefault("joints", {})[js.joint] = {
                "pos": js.pos_crad / 100.0,
                "vel": js.vel_crad_s / 100.0,
                "eff": js.eff_cNm / 100.0,
            }
        elif f.type == MsgType.BMS_SUMMARY:
            b = wire.BmsSummary.unpack(payload)
            self.bms = {
                "pack_v": b.pack_cV / 100.0,
                "current_a": b.current_cA / 100.0,
                "soc": b.soc_pct,
                "t_max": b.temp_max_C,
                "faults": b.fault_bits,
            }
        elif f.type == MsgType.AERO_STATE:
            a = wire.AeroState.unpack(payload)
            self.telemetry.setdefault(src, {})["aero"] = {
                "ias": a.ias_cms / 100.0,
                "q": a.q_pa,
            }

    def _on_audio(self, f: can_id.CanId, payload: bytes) -> None:
        if f.type == MsgType.AUDIO_SYNC:
            sync = wire.AudioSync.unpack(payload)
            self._audio_dec = adpcm.AdpcmState(
                predictor=sync.predictor, step_index=sync.step_index
            )
            self._audio_expect = sync.frame_seq & 0xFF
            self._audio_locked = True
        elif f.type == MsgType.AUDIO_UP:
            if not self._audio_locked:
                return
            if f.low != self._audio_expect:
                # A dropped frame desynchronises the decoder: insert silence and
                # wait for the next SYNC rather than emitting garbage.
                self.audio_gaps += 1
                self.uplink_pcm.extend([0] * 16)
                self._audio_locked = False
                return
            self.uplink_pcm.extend(adpcm.decode(self._audio_dec, payload))
            self._audio_expect = (self._audio_expect + 1) & 0xFF

    # --- downlink helpers -----------------------------------------------------

    def _queue(self, cls: int, dst: int, mtype: int, low: int, payload: bytes,
               bus: int = spi_frame.BUS_CAN1) -> None:
        self._downlink.append(
            spi_frame.CanRecord(
                id=can_id.pack(cls, Node.ORCH, dst, mtype, low),
                bus=bus,
                dlc=len(payload),
                ts_ms=0,
                data=payload,
            )
        )

    async def _heartbeat_loop(self) -> None:
        while True:
            if self.heartbeat_enabled and self.transport_fresh and not self.estop_latched:
                hb = wire.Heartbeat(
                    seq=self.hb_seq & 0xFFFF,
                    flags=wire.HB_CLOUD_UP if self.cloud and self.cloud.connected else 0,
                    src_state=wire.State.OPERATIONAL,
                    uptime_ms=int(time.monotonic() * 1000) & 0xFFFFFFFF,
                )
                self._queue(Cls.SAFETY, Node.BROADCAST, MsgType.HEARTBEAT,
                            self.hb_seq & 0xFF, hb.pack())
                self.hb_seq += 1
                self.beats_sent += 1
            await asyncio.sleep(HEARTBEAT_PERIOD_S)

    def send_mode(self, node: int, state: int) -> None:
        self._queue(Cls.CONTROL, node, MsgType.MODE_SET, 0, wire.ModeSet(target_state=state).pack())

    def send_joint_cmd(self, node: int, joint: int, pos_crad: int) -> None:
        cmd = wire.JointCmd(joint=joint, mode=wire.JointMode.POSITION, pos_crad=pos_crad)
        self._queue(Cls.CONTROL, node, MsgType.JOINT_CMD, 0, cmd.pack())

    def send_clear_estop(self, counter: int) -> None:
        self._queue(Cls.SAFETY, Node.BROADCAST, MsgType.CLEAR_ESTOP, 0,
                    wire.ClearEstop(counter=counter).pack())
        self.estop_latched = False

    def send_audio_down(self, pcm: list[int]) -> None:
        """Re-frame cloud speech onto the AUDIO plane toward the helmet."""
        enc = adpcm.AdpcmState()
        seq = 0
        sync = wire.AudioSync(dir=0, step_index=enc.step_index,
                              predictor=enc.predictor, frame_seq=0)
        self._queue(Cls.AUDIO, Node.HELMET, MsgType.AUDIO_SYNC, 0, sync.pack())
        for off in range(0, len(pcm) - 15, 16):
            if seq and seq % 50 == 0:
                s = wire.AudioSync(dir=0, step_index=enc.step_index,
                                   predictor=enc.predictor, frame_seq=seq)
                self._queue(Cls.AUDIO, Node.HELMET, MsgType.AUDIO_SYNC, 0, s.pack())
            block = adpcm.encode(enc, pcm[off:off + 16])
            self._queue(Cls.AUDIO, Node.HELMET, MsgType.AUDIO_DOWN, seq & 0xFF, block)
            seq += 1


class CloudLink:
    """Node 8's side of docs/link-protocol.md, compressed to what the scenarios need."""

    def __init__(self, url: str, token: str, bridge: BridgeLite, *, suit_id: str = "sim-01"):
        self.url = url
        self.token = token
        self.bridge = bridge
        self.suit_id = suit_id

        self.connected = False
        self.session_id: str | None = None
        self.advisories: list[dict[str, Any]] = []
        self.tts_pcm: list[int] = []
        self.tts_chunks = 0
        self.rejected = 0
        self.auto_credit = True

        self._ws: Any = None
        self._seq = 0
        self._last_rx_seq = 0
        self._tts_dec = adpcm.AdpcmState()
        self._task: asyncio.Task[None] | None = None

    async def connect(self, *, resume: bool = False) -> None:
        self._ws = await websockets.connect(self.url)
        payload: dict[str, Any] = {
            "proto": link.PROTO_VERSION,
            "suit_id": self.suit_id,
            "token": self.token,
            "agent": "suit_sim/bridge_lite",
        }
        if resume and self.session_id:
            payload["resume"] = {
                "session_id": self.session_id,
                "last_rx_seq": self._last_rx_seq,
            }
        await self._send(link.T_HELLO, payload)

        ack = link.decode_envelope(await self._ws.recv())
        self.session_id = ack["payload"].get("session_id")
        self.connected = True
        self._task = asyncio.ensure_future(self._rx_loop())

    async def close(self) -> None:
        self.connected = False
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
        if self._ws is not None:
            with contextlib.suppress(Exception):
                await self._ws.close()

    async def _send(self, msg_type: str, payload: dict[str, Any]) -> None:
        self._seq += 1
        await self._ws.send(link.encode_envelope(link.make_envelope(msg_type, self._seq, payload)))

    async def send_telemetry(self) -> None:
        await self._send(link.T_TELEMETRY_BATCH, {
            "window_ms": 200,
            "sources": {
                "power": self.bridge.bms,
                "safety": {
                    "state": "ESTOP" if self.bridge.estop_latched else "OPERATIONAL",
                    "estop": self.bridge.estop_latched,
                    "hb_ok": self.bridge.transport_fresh,
                },
            },
            "dropped_count": 0,
        })

    async def voice_query(self, text: str, query_id: str = "q-1") -> None:
        await self._send(link.T_VOICE_QUERY, {"query_id": query_id, "text": text,
                                              "source": "helmet"})

    async def grant_credits(self, stream_id: int, credits: int) -> None:
        await self._send(link.T_AUDIO_CREDIT, {"stream_id": stream_id, "credits": credits})

    async def _rx_loop(self) -> None:
        try:
            async for message in self._ws:
                if isinstance(message, bytes):
                    self._on_binary(message)
                else:
                    self._on_text(message)
        except (asyncio.CancelledError, websockets.exceptions.ConnectionClosed):
            self.connected = False

    def _on_text(self, raw: str) -> None:
        try:
            env = link.decode_envelope(raw)
        except link.LinkProtocolError:
            self.rejected += 1
            return
        # The security boundary: anything outside the whitelist is dropped and
        # counted, never acted on.
        if env["type"] not in link.DOWNLINK_WHITELIST:
            self.rejected += 1
            return
        self._last_rx_seq = max(self._last_rx_seq, int(env.get("seq", 0)))
        if env["type"] == link.T_ADVISORY:
            self.advisories.append(env["payload"])
        elif env["type"] == link.T_TTS_META:
            if env["payload"].get("state") == "start":
                self._tts_dec = adpcm.AdpcmState()
                if self.auto_credit:
                    stream = env["payload"].get("stream_id", 0)
                    asyncio.ensure_future(self.grant_credits(stream, 16))

    def _on_binary(self, data: bytes) -> None:
        try:
            btype, codec, _stream, _seq, payload = link.parse_audio_frame(data)
        except link.LinkProtocolError:
            self.rejected += 1
            return
        if btype != link.BTYPE_TTS_CHUNK:
            self.rejected += 1
            return
        self.tts_chunks += 1
        if codec == link.CODEC_ADPCM_8K:
            self.tts_pcm.extend(adpcm.decode(self._tts_dec, payload))
        else:
            self.tts_pcm.extend(
                int.from_bytes(payload[i:i + 2], "little", signed=True)
                for i in range(0, len(payload) - 1, 2)
            )
