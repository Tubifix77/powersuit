import sys

import pytest

from powersuit_proto import wire


def test_platform_little_endian():
    # The C twin memcpy's packed LE structs; contract requires LE hosts.
    assert sys.byteorder == "little"


@pytest.mark.parametrize("cls", wire.ALL_WIRE_TYPES, ids=lambda c: c.__name__)
def test_every_payload_is_exactly_8_bytes(cls):
    assert cls.size() == 8
    assert len(cls().pack()) == 8


@pytest.mark.parametrize("cls", wire.ALL_WIRE_TYPES, ids=lambda c: c.__name__)
def test_default_roundtrip(cls):
    obj = cls()
    assert cls.unpack(obj.pack()) == obj


def test_heartbeat_roundtrip_extremes():
    hb = wire.Heartbeat(seq=0xFFFF, flags=wire.HB_ESTOP_LATCHED | wire.HB_CLOUD_UP,
                        src_state=wire.State.OPERATIONAL, uptime_ms=0xFFFF_FFFF)
    assert wire.Heartbeat.unpack(hb.pack()) == hb


def test_joint_cmd_signed_extremes():
    cmd = wire.JointCmd(joint=1, mode=wire.JointMode.TORQUE,
                        pos_crad=-32768, vel_crad_s=32767, eff_cNm=-1)
    back = wire.JointCmd.unpack(cmd.pack())
    assert back == cmd


def test_clear_estop_magic_layout():
    raw = wire.ClearEstop(counter=7).pack()
    # magic occupies the first 4 bytes, little-endian
    assert raw[:4] == (0x52A4C13A).to_bytes(4, "little")
    assert raw[4:] == (7).to_bytes(4, "little")


def test_bms_summary_units():
    s = wire.BmsSummary(pack_cV=5020, current_cA=-1210, soc_pct=81,
                        temp_max_C=-40, fault_bits=wire.BMSF_COMP_ARMED)
    back = wire.BmsSummary.unpack(s.pack())
    assert back.current_cA == -1210
    assert back.temp_max_C == -40


def test_contract_constants():
    assert wire.HEARTBEAT_PERIOD_MS == 10
    assert wire.HEARTBEAT_TIMEOUT_MS == 50
    assert wire.HEARTBEAT_TIMEOUT_MS // wire.HEARTBEAT_PERIOD_MS >= 4  # loss tolerance
    assert wire.REARM_WINDOW_MS == 250
    assert wire.ESTOP_REPEAT == 3
