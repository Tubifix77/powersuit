"""Wire payload structs — mirror of common/c/include/powersuit_proto/wire.h.

Every payload packs to exactly 8 bytes (classic CAN), little-endian.
"""

from __future__ import annotations

import struct
from dataclasses import astuple, dataclass
from enum import IntEnum
from typing import ClassVar, TypeVar

# Contract constants (docs/safety.md §2).
HEARTBEAT_PERIOD_MS = 10
HEARTBEAT_TIMEOUT_MS = 50
REARM_WINDOW_MS = 250
CMD_STALE_MS = 200
ESTOP_REARM_MS = 1000
ESTOP_REPEAT = 3
CLEAR_ESTOP_MAGIC = 0x52A4_C13A


class State(IntEnum):
    BOOT = 0
    STANDBY = 1
    OPERATIONAL = 2
    PASSIVE = 3
    ESTOP = 4
    FAULT = 5


class EstopCause(IntEnum):
    BMS_SHORT = 1
    BMS_OVERVOLT = 2
    BMS_UNDERVOLT = 3
    OPERATOR = 4
    THERMAL = 5
    COMM_LOSS = 6
    SOFTWARE = 7


class JointMode(IntEnum):
    PASSIVE = 0
    POSITION = 1
    VELOCITY = 2
    TORQUE = 3
    IMPEDANCE = 4


# HEARTBEAT flag bits.
HB_ESTOP_LATCHED = 1 << 0
HB_DEGRADED = 1 << 1
HB_CLOUD_UP = 1 << 2

# BMS fault bits.
BMSF_SHORT_LATCH = 1 << 0
BMSF_OV = 1 << 1
BMSF_UV = 1 << 2
BMSF_OT = 1 << 3
BMSF_UT = 1 << 4
BMSF_OC_CHARGE = 1 << 5
BMSF_OC_DISCHARGE = 1 << 6
BMSF_COMP_ARMED = 1 << 7

T = TypeVar("T", bound="Wire")


class Wire:
    """Base for fixed-format payloads. Subclasses set _fmt; field order == dataclass order."""

    _fmt: ClassVar[str]

    def pack(self) -> bytes:
        return struct.pack(self._fmt, *astuple(self))

    @classmethod
    def unpack(cls: type[T], data: bytes | memoryview) -> T:
        return cls(*struct.unpack_from(cls._fmt, data))

    @classmethod
    def size(cls) -> int:
        return struct.calcsize(cls._fmt)


# --- SAFETY ---------------------------------------------------------------------


@dataclass
class Heartbeat(Wire):
    _fmt: ClassVar[str] = "<HBBI"
    seq: int = 0
    flags: int = 0
    src_state: int = 0
    uptime_ms: int = 0


@dataclass
class Estop(Wire):
    _fmt: ClassVar[str] = "<BBHI"
    cause: int = 0
    origin_node: int = 0
    seq: int = 0
    uptime_ms: int = 0


@dataclass
class ClearEstop(Wire):
    _fmt: ClassVar[str] = "<II"
    magic: int = CLEAR_ESTOP_MAGIC
    counter: int = 0


@dataclass
class NodeFault(Wire):
    _fmt: ClassVar[str] = "<BBHI"
    fault_code: int = 0
    severity: int = 0
    detail: int = 0
    uptime_ms: int = 0


# --- CONTROL --------------------------------------------------------------------


@dataclass
class JointCmd(Wire):
    _fmt: ClassVar[str] = "<BBhhh"
    joint: int = 0
    mode: int = 0
    pos_crad: int = 0
    vel_crad_s: int = 0
    eff_cNm: int = 0


@dataclass
class FlapCmd(Wire):
    _fmt: ClassVar[str] = "<BBhHH"
    flap: int = 0
    rate_lim: int = 0
    pos_pm: int = 0
    flags: int = 0
    rsvd: int = 0


@dataclass
class ModeSet(Wire):
    _fmt: ClassVar[str] = "<B7x"
    target_state: int = 0


@dataclass
class LedPattern(Wire):
    _fmt: ClassVar[str] = "<BBBBBB2x"
    pattern: int = 0
    brightness: int = 0
    r: int = 0
    g: int = 0
    b: int = 0
    speed: int = 0


# --- TELEM ----------------------------------------------------------------------


@dataclass
class JointState(Wire):
    _fmt: ClassVar[str] = "<BBhhh"
    joint: int = 0
    flags: int = 0
    pos_crad: int = 0
    vel_crad_s: int = 0
    eff_cNm: int = 0


@dataclass
class ImuQuat(Wire):
    _fmt: ClassVar[str] = "<hhhh"
    qw: int = 32767  # Q15 identity
    qx: int = 0
    qy: int = 0
    qz: int = 0


@dataclass
class ImuAcc(Wire):
    _fmt: ClassVar[str] = "<hhh2x"
    ax: int = 0
    ay: int = 0
    az: int = 0


@dataclass
class ImuGyr(Wire):
    _fmt: ClassVar[str] = "<hhh2x"
    gx: int = 0
    gy: int = 0
    gz: int = 0


@dataclass
class Force(Wire):
    _fmt: ClassVar[str] = "<hhhh"
    ch0: int = 0
    ch1: int = 0
    ch2: int = 0
    ch3: int = 0


@dataclass
class BmsSummary(Wire):
    _fmt: ClassVar[str] = "<HhBbH"
    pack_cV: int = 0
    current_cA: int = 0
    soc_pct: int = 0
    temp_max_C: int = 0
    fault_bits: int = 0


@dataclass
class BmsCells(Wire):
    _fmt: ClassVar[str] = "<BxHHH"
    group: int = 0
    mv0: int = 0
    mv1: int = 0
    mv2: int = 0


@dataclass
class AeroState(Wire):
    _fmt: ClassVar[str] = "<HHhH"
    ias_cms: int = 0
    q_pa: int = 0
    aoa_cdeg: int = 0
    flags: int = 0


@dataclass
class FlapState(Wire):
    _fmt: ClassVar[str] = "<BBhh2x"
    flap: int = 0
    flags: int = 0
    pos_pm: int = 0
    target_pm: int = 0


@dataclass
class Env(Wire):
    _fmt: ClassVar[str] = "<hHH2x"
    temp_cC: int = 0
    rh_pm: int = 0
    press_dhPa: int = 0


@dataclass
class NodeStats(Wire):
    _fmt: ClassVar[str] = "<BBHHH"
    cpu_pct: int = 0
    state: int = 0
    rx_fps: int = 0
    tx_fps: int = 0
    err_cnt: int = 0


# --- AUDIO ----------------------------------------------------------------------


@dataclass
class AudioSync(Wire):
    _fmt: ClassVar[str] = "<BBhH2x"
    dir: int = 0  # 0 down, 1 up
    step_index: int = 0
    predictor: int = 0
    frame_seq: int = 0


@dataclass
class AudioCtl(Wire):
    _fmt: ClassVar[str] = "<BBH4x"
    dir: int = 0
    cmd: int = 0  # 0 stop, 1 start, 2 rate
    sample_rate: int = 8000


# --- MGMT -----------------------------------------------------------------------


@dataclass
class TimeSync(Wire):
    _fmt: ClassVar[str] = "<IH2x"
    epoch_ms_lo: int = 0
    seq: int = 0


@dataclass
class FlowCtl(Wire):
    _fmt: ClassVar[str] = "<BB6x"
    plane: int = 0
    level: int = 0


@dataclass
class Version(Wire):
    _fmt: ClassVar[str] = "<BBBBI"
    major: int = 0
    minor: int = 0
    patch: int = 0
    node_state: int = 0
    git_short: int = 0


ALL_WIRE_TYPES: tuple[type[Wire], ...] = (
    Heartbeat, Estop, ClearEstop, NodeFault,
    JointCmd, FlapCmd, ModeSet, LedPattern,
    JointState, ImuQuat, ImuAcc, ImuGyr, Force,
    BmsSummary, BmsCells, AeroState, FlapState, Env, NodeStats,
    AudioSync, AudioCtl,
    TimeSync, FlowCtl, Version,
)
