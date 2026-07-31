"""Simulated flight/aero node (Node 5) on a virtual CAN bus.

docs/safety.md §2: "aero engine keeps publishing AERO_STATE (telemetry never
stops)" even in PASSIVE, so unlike FakeLimb this node has no safety state
machine at all — it just always emits. It exists to give the hub/bridge real
CAN-2 traffic to route and the bridge a torso IMU source for the cloud
telemetry batch's `torso_imu` field (docs/link-protocol.md §2.3).
"""

from __future__ import annotations

import asyncio
import math

from powersuit_proto import can_id, wire
from powersuit_proto.can_id import Cls, MsgType, Node

from .virtual_can import CanFrame, VirtualBus

AERO_PERIOD_S = 0.02  # 50 Hz (docs/network-map.md §3.3)
IMU_PERIOD_S = 0.01  # 100 Hz


class FakeFlight:
    def __init__(self, bus: VirtualBus) -> None:
        self.node_id = Node.FLIGHT
        self.bus = bus
        self.telem_count = 0
        self._tasks: list[asyncio.Task[None]] = []

    async def start(self) -> None:
        self._tasks = [
            asyncio.ensure_future(self._aero_loop()),
            asyncio.ensure_future(self._imu_loop()),
        ]

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        for task in self._tasks:
            try:
                await task
            except asyncio.CancelledError:
                pass
        self.bus.detach(self.node_id)

    async def _aero_loop(self) -> None:
        seq = 0
        try:
            while True:
                await asyncio.sleep(AERO_PERIOD_S)
                phase = seq * 0.1
                payload = wire.AeroState(
                    ias_cms=int(500 + 50 * math.sin(phase)),
                    q_pa=100,
                    aoa_cdeg=int(20 * math.sin(phase)),
                    flags=0,
                ).pack()
                self._send(MsgType.AERO_STATE, seq, payload)
                seq += 1
                self.telem_count += 1
        except asyncio.CancelledError:
            raise

    async def _imu_loop(self) -> None:
        seq = 0
        try:
            while True:
                await asyncio.sleep(IMU_PERIOD_S)
                phase = seq * 0.05
                self._send(
                    MsgType.IMU_QUAT,
                    seq,
                    wire.ImuQuat(qw=32767, qx=int(1000 * math.sin(phase))).pack(),
                )
                self._send(MsgType.IMU_ACC, seq, wire.ImuAcc(az=-1000).pack())
                self._send(MsgType.IMU_GYR, seq, wire.ImuGyr().pack())
                seq += 1
                self.telem_count += 1
        except asyncio.CancelledError:
            raise

    def _send(self, mtype: int, low: int, payload: bytes) -> None:
        frame = CanFrame(
            id=can_id.pack(Cls.TELEM, self.node_id, Node.ORCH, mtype, low & 0xFF),
            data=payload,
        )
        self.bus.send(frame)
