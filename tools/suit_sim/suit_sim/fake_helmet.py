"""Node 6 stand-in: the voice path.

Mirrors the firmware's AUDIO-plane cadence (ps_audio/src/audio_pkt.c): 16-sample
frames, a SYNC before frame 0 and every 50 frames thereafter, sequence carried in
the CAN id's low byte. The codec itself comes from `powersuit_proto.adpcm`, so a
divergence between this and the firmware shows up as a test failure rather than
as sludge in someone's ear.
"""

from __future__ import annotations

import asyncio

from powersuit_proto import adpcm, can_id, wire
from powersuit_proto.can_id import Cls, MsgType, Node

from .virtual_can import CanFrame, VirtualBus

SAMPLES_PER_FRAME = 16
SYNC_INTERVAL = 50


class FakeHelmet:
    """Emits ADPCM voice uplink and records what comes back down."""

    def __init__(self, bus: VirtualBus) -> None:
        self.node_id = Node.HELMET
        self.bus = bus

        self.downlink_frames: list[CanFrame] = []
        self.downlink_syncs: list[CanFrame] = []
        self.downlink_pcm: list[int] = []
        self.state: int = wire.State.STANDBY

        self._enc = adpcm.AdpcmState()
        self._dec = adpcm.AdpcmState()
        self._frame_seq = 0
        self._since_sync = 0
        self._started = False
        self._tasks: list[asyncio.Task[None]] = []

        bus.attach(self.node_id, self._on_frame)

    async def start(self) -> None:
        self._tasks = [asyncio.ensure_future(self._env_loop())]

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        for task in self._tasks:
            try:
                await task
            except asyncio.CancelledError:
                pass
        self.bus.detach(self.node_id)

    # --- uplink ---------------------------------------------------------------

    def push_utterance(self, pcm: list[int]) -> int:
        """Packetise PCM16 @ 8 kHz onto the AUDIO plane. Returns frames sent."""
        if not self._started:
            self._send_ctl(cmd=1)
            self._emit_sync()
            self._started = True

        sent = 0
        for off in range(0, len(pcm) - SAMPLES_PER_FRAME + 1, SAMPLES_PER_FRAME):
            if self._since_sync >= SYNC_INTERVAL:
                self._emit_sync()
            block = adpcm.encode(self._enc, pcm[off:off + SAMPLES_PER_FRAME])
            self._send(MsgType.AUDIO_UP, self._frame_seq & 0xFF, block, dst=Node.ORCH)
            self._frame_seq = (self._frame_seq + 1) & 0xFFFF
            self._since_sync += 1
            sent += 1
        return sent

    def end_utterance(self) -> None:
        if self._started:
            self._send_ctl(cmd=0)
            self._started = False

    def _emit_sync(self) -> None:
        sync = wire.AudioSync(
            dir=1,
            step_index=self._enc.step_index,
            predictor=self._enc.predictor,
            frame_seq=self._frame_seq,
        )
        self._send(MsgType.AUDIO_SYNC, 0, sync.pack(), dst=Node.ORCH)
        self._since_sync = 0

    def _send_ctl(self, cmd: int) -> None:
        ctl = wire.AudioCtl(dir=1, cmd=cmd, sample_rate=8000)
        self._send(MsgType.AUDIO_CTL, 0, ctl.pack(), dst=Node.ORCH)

    # --- downlink -------------------------------------------------------------

    def _on_frame(self, frame: CanFrame) -> None:
        fields = can_id.unpack(frame.id)
        if fields.cls == Cls.SAFETY:
            self._on_safety(fields, frame.data)
        elif fields.cls == Cls.AUDIO:
            self._on_audio(fields, frame.data)

    def _on_safety(self, fields: can_id.CanId, data: bytes) -> None:
        if fields.type == MsgType.ESTOP:
            self.state = wire.State.ESTOP
        elif fields.type == MsgType.HEARTBEAT and self.state != wire.State.ESTOP:
            hb = wire.Heartbeat.unpack(data)
            self.state = (
                wire.State.ESTOP if hb.flags & wire.HB_ESTOP_LATCHED else wire.State.OPERATIONAL
            )

    def _on_audio(self, fields: can_id.CanId, data: bytes) -> None:
        if fields.type == MsgType.AUDIO_SYNC:
            sync = wire.AudioSync.unpack(data)
            self._dec = adpcm.AdpcmState(predictor=sync.predictor, step_index=sync.step_index)
            self.downlink_syncs.append(CanFrame(id=0, data=data))
        elif fields.type == MsgType.AUDIO_DOWN:
            self.downlink_frames.append(CanFrame(id=fields.low, data=data))
            self.downlink_pcm.extend(adpcm.decode(self._dec, data))

    # --- housekeeping ---------------------------------------------------------

    async def _env_loop(self) -> None:
        """1 Hz ENV telemetry, as the real helmet emits."""
        while True:
            env = wire.Env(temp_cC=2410, rh_pm=0xFFFF, press_dhPa=0xFFFF)
            self._send(MsgType.ENV, 0, env.pack(), dst=Node.ORCH, cls=Cls.TELEM)
            await asyncio.sleep(1.0)

    def _send(
        self, mtype: int, low: int, payload: bytes, *, dst: int, cls: int = Cls.AUDIO
    ) -> None:
        self.bus.send(
            CanFrame(id=can_id.pack(cls, self.node_id, dst, mtype, low), data=payload)
        )
