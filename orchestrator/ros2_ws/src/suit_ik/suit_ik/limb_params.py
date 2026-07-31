"""Limb geometry and joint limits (ROS-free).

Joints per limb (docs/network-map.md §1): arms 0=elbow, 1=wrist; legs 0=hip, 1=knee.
The 2-link planar model: q1 rotates link 1 (length l1) about the limb root, q2 is the
relative angle of link 2 (length l2). `branch` selects the elbow-up (-1) or elbow-down
(+1) IK solution: arms fold "up" (q2 <= 0), legs fold knee-forward (q2 >= 0).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class LimbParams:
    l1: float                       # proximal link length [m]
    l2: float                       # distal link length [m]
    branch: int                     # +1 elbow-down, -1 elbow-up
    joint_names: tuple[str, str]
    limits: tuple[tuple[float, float], tuple[float, float]]  # (min, max) per joint [rad]

    @property
    def max_reach(self) -> float:
        return self.l1 + self.l2

    @property
    def min_reach(self) -> float:
        return abs(self.l1 - self.l2)


_ARM_LIMITS = ((-2.62, 2.62), (-2.80, 0.0))   # elbow sweep, wrist folds up only
_LEG_LIMITS = ((-2.62, 2.62), (0.0, 2.80))    # hip sweep, knee folds down only

LIMBS: dict[str, LimbParams] = {
    "arm_right": LimbParams(0.30, 0.28, -1, ("arm_right_elbow", "arm_right_wrist"), _ARM_LIMITS),
    "arm_left": LimbParams(0.30, 0.28, -1, ("arm_left_elbow", "arm_left_wrist"), _ARM_LIMITS),
    "leg_right": LimbParams(0.45, 0.43, +1, ("leg_right_hip", "leg_right_knee"), _LEG_LIMITS),
    "leg_left": LimbParams(0.45, 0.43, +1, ("leg_left_hip", "leg_left_knee"), _LEG_LIMITS),
}

LIMB_NAMES = tuple(LIMBS.keys())
