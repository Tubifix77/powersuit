"""CAN identifier layout — mirror of common/c/include/powersuit_proto/can_id.h."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

CAN_ID_MASK = 0x1FFF_FFFF


class Cls(IntEnum):
    SAFETY = 0
    CONTROL = 1
    TELEM = 2
    XRCE = 3
    AUDIO = 4
    MGMT = 5


class Node(IntEnum):
    BROADCAST = 0
    ARM_R = 1
    ARM_L = 2
    LEG_R = 3
    LEG_L = 4
    FLIGHT = 5
    HELMET = 6
    HUB = 7
    ORCH = 8


class MsgType(IntEnum):
    # SAFETY
    HEARTBEAT = 0x01
    ESTOP = 0x02
    CLEAR_ESTOP = 0x03
    NODE_FAULT = 0x04
    # CONTROL
    JOINT_CMD = 0x10
    FLAP_CMD = 0x11
    MODE_SET = 0x12
    LED_PATTERN = 0x13
    # TELEM
    JOINT_STATE = 0x20
    IMU_QUAT = 0x21
    IMU_ACC = 0x22
    IMU_GYR = 0x23
    FORCE = 0x24
    BMS_SUMMARY = 0x25
    BMS_CELLS = 0x26
    AERO_STATE = 0x27
    FLAP_STATE = 0x28
    ENV = 0x29
    NODE_STATS = 0x2A
    # XRCE
    XRCE_STREAM = 0x30
    # AUDIO
    AUDIO_DOWN = 0x40
    AUDIO_UP = 0x41
    AUDIO_SYNC = 0x42
    AUDIO_CTL = 0x43
    # MGMT
    TIME_SYNC = 0x50
    FLOW_CTL = 0x51
    STATS_REQ = 0x52
    LOG = 0x53
    VERSION = 0x54


# Limb string IDs used in ROS topic names, keyed by node ID.
LIMB_NAMES = {
    Node.ARM_R: "arm_right",
    Node.ARM_L: "arm_left",
    Node.LEG_R: "leg_right",
    Node.LEG_L: "leg_left",
}

# Bus membership (docs/network-map.md §1): CAN 1 and CAN 2.
BUS1_NODES = frozenset({Node.ARM_R, Node.ARM_L, Node.HELMET})
BUS2_NODES = frozenset({Node.LEG_R, Node.LEG_L, Node.FLIGHT})


@dataclass(frozen=True)
class CanId:
    cls: int
    src: int
    dst: int
    type: int
    low: int


def pack(cls: int, src: int, dst: int, type: int, low: int = 0) -> int:
    return (
        ((cls & 0x7) << 26)
        | ((src & 0x1F) << 21)
        | ((dst & 0x1F) << 16)
        | ((type & 0xFF) << 8)
        | (low & 0xFF)
    )


def unpack(can_id: int) -> CanId:
    return CanId(
        cls=(can_id >> 26) & 0x7,
        src=(can_id >> 21) & 0x1F,
        dst=(can_id >> 16) & 0x1F,
        type=(can_id >> 8) & 0xFF,
        low=can_id & 0xFF,
    )
