"""Simulated actuating edge node (arm/leg) on a virtual CAN bus.

This is a Python re-implementation of the REAL edge safety semantics from
docs/safety.md §2 — the state machine, timing constants (imported from
`powersuit_proto.wire`, never redefined here), and wire formats are the real
contract. The only thing that is simulated rather than real is the "physics":
synthetic sinusoidal joint/IMU/force motion stands in for FOC + kinematics,
so tests can observe "the joint stopped" without a motor model.
"""

from __future__ import annotations

import asyncio
import math

from powersuit_proto import can_id, wire
from powersuit_proto.can_id import Cls, MsgType, Node

from .virtual_can import CanFrame, VirtualBus

WATCHDOG_PERIOD_S = 0.005  # 5 ms — docs/safety.md §2 watchdog cadence.
TELEM_PERIOD_S = 0.01  # 100 Hz (docs/network-map.md §3.3).
JOINTS_PER_LIMB = 2


class EstopLatch:
    """CLEAR_ESTOP validation + the ESTOP->STANDBY re-arm timer, shared shape
    for any node that latches ESTOP (FakeLimb here; FakeHub has its own
    simpler variant since the hub's latch models different hardware — see
    fake_hub.py).

    docs/safety.md §3: CLEAR_ESTOP is accepted only with the magic constant
    and a strictly monotonic counter; even once accepted, the latch does not
    drop until `PS_ESTOP_REARM_MS` of *continuous* heartbeat has elapsed
    starting no earlier than the accept time (a heartbeat gap after acceptance
    restarts the countdown).
    """

    def __init__(self) -> None:
        self.latched = False
        self.last_accepted_counter = 0
        self._clear_accepted_at: float | None = None

    def raise_estop(self) -> None:
        self.latched = True
        self._clear_accepted_at = None

    def try_clear(self, clear: wire.ClearEstop, now: float) -> bool:
        """Validate a CLEAR_ESTOP frame. Returns True if accepted (this does
        NOT drop the latch yet — see `poll`)."""
        if not self.latched:
            return False
        if clear.magic != wire.CLEAR_ESTOP_MAGIC:
            return False
        if clear.counter <= self.last_accepted_counter:
            return False  # replayed/stale counter — rejected, stays latched
        self.last_accepted_counter = clear.counter
        self._clear_accepted_at = now
        return True

    def poll(self, now: float, hb_streak_start: float, hb_fresh: bool) -> bool:
        """Call periodically. Returns True exactly on the tick the latch
        actually drops."""
        if not self.latched or self._clear_accepted_at is None:
            return False
        threshold_start = max(hb_streak_start, self._clear_accepted_at)
        if hb_fresh and (now - threshold_start) >= wire.ESTOP_REARM_MS / 1000:
            self.latched = False
            self._clear_accepted_at = None
            return True
        return False


class FakeLimb:
    """One arm/leg edge node. Emits JOINT_STATE x2 / IMU_QUAT / IMU_ACC /
    IMU_GYR / FORCE at 100 Hz; handles JOINT_CMD, MODE_SET, HEARTBEAT, ESTOP,
    CLEAR_ESTOP exactly as docs/safety.md §2-3 specify."""

    def __init__(
        self,
        node_id: int,
        bus: VirtualBus,
        *,
        motion_hz: float = 0.5,
        motion_amp_crad: int = 40,
    ) -> None:
        self.node_id = node_id
        self.bus = bus
        self._motion_hz = motion_hz
        self._motion_amp = motion_amp_crad

        self.state: int = wire.State.STANDBY
        self.estop = EstopLatch()
        self.telem_count = 0
        self.last_cmd_ms: float | None = None

        self.joints: list[wire.JointState] = [
            wire.JointState(joint=i) for i in range(JOINTS_PER_LIMB)
        ]
        self.imu_quat = wire.ImuQuat()
        self.imu_acc = wire.ImuAcc()
        self.imu_gyr = wire.ImuGyr()
        self.force = wire.Force()

        self._phase = 0.0
        self._last_hb_time: float | None = None
        self._hb_streak_start: float = 0.0
        self._joint_seq = 0
        self._imu_seq = 0
        self._force_seq = 0
        self._tasks: list[asyncio.Task[None]] = []

        bus.attach(node_id, self._on_frame)

    # --- lifecycle ---------------------------------------------------------

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        now = loop.time()
        # Boot grace: treat the node as freshly-heartbeat'd at construction so
        # it doesn't spuriously trip before the bridge ever sends one.
        self._last_hb_time = now
        self._hb_streak_start = now
        self._tasks = [
            asyncio.ensure_future(self._watchdog_loop()),
            asyncio.ensure_future(self._telem_loop()),
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

    # --- public helpers ------------------------------------------------------

    @property
    def hb_fresh(self) -> bool:
        loop = asyncio.get_event_loop()
        return (
            self._last_hb_time is not None
            and (loop.time() - self._last_hb_time) <= wire.HEARTBEAT_TIMEOUT_MS / 1000
        )

    # --- CAN reception -------------------------------------------------------

    def _on_frame(self, frame: CanFrame) -> None:
        cid = can_id.unpack(frame.id)
        loop = asyncio.get_event_loop()
        now = loop.time()
        if cid.cls == Cls.SAFETY:
            if cid.type == MsgType.HEARTBEAT:
                self._on_heartbeat(now)
            elif cid.type == MsgType.ESTOP:
                self._on_estop(now)
            elif cid.type == MsgType.CLEAR_ESTOP:
                self._on_clear_estop(frame.data, now)
        elif cid.cls == Cls.CONTROL:
            if cid.dst not in (self.node_id, Node.BROADCAST):
                return
            if cid.type == MsgType.JOINT_CMD:
                self._on_joint_cmd(now)
            elif cid.type == MsgType.MODE_SET:
                self._on_mode_set(frame.data, now)

    def _on_heartbeat(self, now: float) -> None:
        gap_ok = (
            self._last_hb_time is not None
            and (now - self._last_hb_time) <= wire.HEARTBEAT_TIMEOUT_MS / 1000
        )
        if not gap_ok:
            self._hb_streak_start = now
        self._last_hb_time = now

    def _on_estop(self, now: float) -> None:
        self.estop.raise_estop()
        self.state = wire.State.ESTOP

    def _on_clear_estop(self, data: bytes, now: float) -> None:
        clear = wire.ClearEstop.unpack(data)
        self.estop.try_clear(clear, now)  # actual unlatch resolved in the watchdog

    def _on_joint_cmd(self, now: float) -> None:
        self.last_cmd_ms = now * 1000.0

    def _on_mode_set(self, data: bytes, now: float) -> None:
        mode = wire.ModeSet.unpack(data)
        hb_fresh = (
            self._last_hb_time is not None
            and (now - self._last_hb_time) <= wire.HEARTBEAT_TIMEOUT_MS / 1000
        )
        if mode.target_state == wire.State.OPERATIONAL and self.state == wire.State.STANDBY and hb_fresh:
            self.state = wire.State.OPERATIONAL
            self.last_cmd_ms = now * 1000.0

    # --- watchdog (docs/safety.md §2) ----------------------------------------

    async def _watchdog_loop(self) -> None:
        loop = asyncio.get_running_loop()
        try:
            while True:
                await asyncio.sleep(WATCHDOG_PERIOD_S)
                now = loop.time()
                hb_fresh = (
                    self._last_hb_time is not None
                    and (now - self._last_hb_time) <= wire.HEARTBEAT_TIMEOUT_MS / 1000
                )
                if not hb_fresh and self.state == wire.State.OPERATIONAL:
                    self.state = wire.State.PASSIVE

                if self.state == wire.State.PASSIVE:
                    rearm_ok = (
                        hb_fresh
                        and (now - self._hb_streak_start) >= wire.REARM_WINDOW_MS / 1000
                        and self.last_cmd_ms is not None
                        and self.last_cmd_ms / 1000.0 >= self._hb_streak_start
                    )
                    if rearm_ok:
                        self.state = wire.State.OPERATIONAL

                if self.state == wire.State.ESTOP:
                    if self.estop.poll(now, self._hb_streak_start, hb_fresh):
                        self.state = wire.State.STANDBY
        except asyncio.CancelledError:
            raise

    # --- telemetry (docs/network-map.md §3.3) --------------------------------

    async def _telem_loop(self) -> None:
        loop = asyncio.get_running_loop()
        last = loop.time()
        try:
            while True:
                await asyncio.sleep(TELEM_PERIOD_S)
                now = loop.time()
                dt = now - last
                last = now
                if self.state == wire.State.OPERATIONAL:
                    self._phase += 2 * math.pi * self._motion_hz * dt
                self._update_synthetic()
                self._emit_telemetry()
        except asyncio.CancelledError:
            raise

    def _update_synthetic(self) -> None:
        passive = self.state != wire.State.OPERATIONAL
        for j, js in enumerate(self.joints):
            off = j * math.pi / 3
            js.pos_crad = int(self._motion_amp * math.sin(self._phase + off))
            js.vel_crad_s = int(
                self._motion_amp * self._motion_hz * 2 * math.pi * math.cos(self._phase + off)
            )
            js.eff_cNm = int(5 * math.sin(self._phase + off))
            js.flags = 0x02 if passive else 0x00  # b1 passive (docs/network-map.md §3.3)
        self.imu_quat.qx = int(1000 * math.sin(self._phase))
        self.imu_acc.ax = int(50 * math.sin(self._phase))
        self.imu_acc.az = -1000
        self.imu_gyr.gx = int(20 * math.cos(self._phase))
        self.force.ch0 = int(30 * math.sin(self._phase))

    def _emit_telemetry(self) -> None:
        for js in self.joints:
            self._send(MsgType.JOINT_STATE, self._joint_seq, js.pack())
            self.telem_count += 1
        self._joint_seq = (self._joint_seq + 1) & 0xFF
        self._send(MsgType.IMU_QUAT, self._imu_seq, self.imu_quat.pack())
        self._send(MsgType.IMU_ACC, self._imu_seq, self.imu_acc.pack())
        self._send(MsgType.IMU_GYR, self._imu_seq, self.imu_gyr.pack())
        self.telem_count += 3
        self._imu_seq = (self._imu_seq + 1) & 0xFF
        self._send(MsgType.FORCE, self._force_seq, self.force.pack())
        self.telem_count += 1
        self._force_seq = (self._force_seq + 1) & 0xFF

    def _send(self, mtype: int, low: int, payload: bytes) -> None:
        frame = CanFrame(
            id=can_id.pack(Cls.TELEM, self.node_id, Node.ORCH, mtype, low & 0xFF),
            data=payload,
        )
        self.bus.send(frame)

    # --- XRCE (used directly by scenario tests to manufacture interleaving) --

    def send_xrce_bytes(self, data: bytes) -> None:
        """Emit `data` as a run of XRCE STREAM frames, <=8 bytes each (§3.4)."""
        for i in range(0, len(data), 8):
            chunk = data[i : i + 8]
            frame = CanFrame(
                id=can_id.pack(Cls.XRCE, self.node_id, Node.ORCH, MsgType.XRCE_STREAM, 0),
                data=chunk,
            )
            self.bus.send(frame)
