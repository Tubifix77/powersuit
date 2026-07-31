"""Planar 2-link inverse kinematics — PURE python, no rclpy (natively tested).

solve() clamps unreachable targets to the nearest point of the annular workspace,
then clamps joint angles to the per-limb limits from limb_params.
"""

from __future__ import annotations

import math

from .limb_params import LIMBS, LimbParams

_EPS = 1e-12


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp_to_workspace(limb: str, x: float, y: float) -> tuple[float, float]:
    """Nearest reachable point of the annulus [min_reach, max_reach]."""
    p = LIMBS[limb]
    r = math.hypot(x, y)
    if r < _EPS:
        # Degenerate: no direction — snap along +x to the inner boundary.
        return (p.min_reach, 0.0)
    if r > p.max_reach:
        s = p.max_reach / r
        return (x * s, y * s)
    if r < p.min_reach:
        s = p.min_reach / r
        return (x * s, y * s)
    return (x, y)


def solve(limb: str, x: float, y: float, branch: int | None = None) -> tuple[float, float]:
    """IK for target (x, y) in the limb root frame. Returns (q1, q2) [rad].

    branch overrides the limb's configured elbow branch (tests only).
    """
    p: LimbParams = LIMBS[limb]
    sign = p.branch if branch is None else branch
    x, y = clamp_to_workspace(limb, x, y)

    d = (x * x + y * y - p.l1 * p.l1 - p.l2 * p.l2) / (2.0 * p.l1 * p.l2)
    d = _clamp(d, -1.0, 1.0)
    q2 = sign * math.acos(d)
    q1 = math.atan2(y, x) - math.atan2(p.l2 * math.sin(q2), p.l1 + p.l2 * math.cos(q2))
    q1 = _wrap_pi(q1)

    q1 = _clamp(q1, p.limits[0][0], p.limits[0][1])
    q2 = _clamp(q2, p.limits[1][0], p.limits[1][1])
    return (q1, q2)


def forward(limb: str, q1: float, q2: float) -> tuple[float, float]:
    """FK: end-effector (x, y) in the limb root frame."""
    p = LIMBS[limb]
    return (
        p.l1 * math.cos(q1) + p.l2 * math.cos(q1 + q2),
        p.l1 * math.sin(q1) + p.l2 * math.sin(q1 + q2),
    )
