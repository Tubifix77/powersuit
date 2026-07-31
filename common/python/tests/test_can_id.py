import random

from powersuit_proto import can_id
from powersuit_proto.can_id import Cls, MsgType, Node


def test_known_example():
    # TELEM JOINT_STATE from arm_right to orchestrator, seq 5
    cid = can_id.pack(Cls.TELEM, Node.ARM_R, Node.ORCH, MsgType.JOINT_STATE, 5)
    assert cid == (2 << 26) | (1 << 21) | (8 << 16) | (0x20 << 8) | 5
    fields = can_id.unpack(cid)
    assert fields.cls == Cls.TELEM
    assert fields.src == Node.ARM_R
    assert fields.dst == Node.ORCH
    assert fields.type == MsgType.JOINT_STATE
    assert fields.low == 5


def test_roundtrip_random():
    rng = random.Random(1234)
    for _ in range(2000):
        cls, src, dst, typ, low = (
            rng.randrange(8),
            rng.randrange(32),
            rng.randrange(32),
            rng.randrange(256),
            rng.randrange(256),
        )
        cid = can_id.pack(cls, src, dst, typ, low)
        assert cid <= can_id.CAN_ID_MASK
        f = can_id.unpack(cid)
        assert (f.cls, f.src, f.dst, f.type, f.low) == (cls, src, dst, typ, low)


def test_safety_wins_arbitration():
    # Lower numeric ID wins CAN arbitration; SAFETY must beat every other class
    # regardless of src/dst/type.
    estop = can_id.pack(Cls.SAFETY, 31, 31, 0xFF, 0xFF)
    for cls in (Cls.CONTROL, Cls.TELEM, Cls.XRCE, Cls.AUDIO, Cls.MGMT):
        assert estop < can_id.pack(cls, 0, 0, 0, 0)


def test_bus_membership_disjoint():
    assert not (can_id.BUS1_NODES & can_id.BUS2_NODES)
    assert Node.HUB not in can_id.BUS1_NODES | can_id.BUS2_NODES
