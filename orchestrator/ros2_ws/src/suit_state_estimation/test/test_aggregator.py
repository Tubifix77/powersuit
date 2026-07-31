"""Native tests for suit_state_estimation.aggregator_core — ROS-free."""

from suit_state_estimation.aggregator_core import LIMB_ORDER, aggregate


def _limb(prefix, j1, j2, base):
    return ([f"{prefix}_{j1}", f"{prefix}_{j2}"],
            [base + 0.1, base + 0.2], [base + 0.3, base + 0.4], [base + 0.5, base + 0.6])


def test_deterministic_order_all_limbs():
    by_limb = {
        "leg_left": _limb("leg_left", "hip", "knee", 40.0),
        "arm_right": _limb("arm_right", "elbow", "wrist", 10.0),
        "leg_right": _limb("leg_right", "hip", "knee", 30.0),
        "arm_left": _limb("arm_left", "elbow", "wrist", 20.0),
    }
    names, pos, vel, eff = aggregate(by_limb)
    assert names == [
        "arm_right_elbow", "arm_right_wrist",
        "arm_left_elbow", "arm_left_wrist",
        "leg_right_hip", "leg_right_knee",
        "leg_left_hip", "leg_left_knee",
    ]
    assert pos[0] == 10.1 and pos[-1] == 40.2
    assert vel[2] == 20.3 and eff[5] == 30.6
    assert len(pos) == len(vel) == len(eff) == 8


def test_missing_limb_absent():
    by_limb = {"arm_right": _limb("arm_right", "elbow", "wrist", 1.0)}
    names, pos, vel, eff = aggregate(by_limb)
    assert names == ["arm_right_elbow", "arm_right_wrist"]
    assert len(pos) == 2


def test_ragged_arrays_padded():
    by_limb = {"arm_left": (["arm_left_elbow", "arm_left_wrist"], [0.5], [], [1.0, 2.0, 3.0])}
    names, pos, vel, eff = aggregate(by_limb)
    assert pos == [0.5, 0.0]
    assert vel == [0.0, 0.0]
    assert eff == [1.0, 2.0]


def test_duplicate_names_last_writer_wins():
    by_limb = {
        "arm_right": (["shared"], [1.0], [1.0], [1.0]),
        "arm_left": (["shared"], [2.0], [2.0], [2.0]),
    }
    names, pos, vel, eff = aggregate(by_limb)
    assert names == ["shared"]
    assert pos == [2.0]


def test_limb_order_constant():
    assert LIMB_ORDER == ("arm_right", "arm_left", "leg_right", "leg_left")
