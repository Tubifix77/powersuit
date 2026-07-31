"""Native tests for suit_ik.ik_core — ROS-free (no rclpy import anywhere below)."""

import math

import pytest

from suit_ik.ik_core import clamp_to_workspace, forward, solve
from suit_ik.limb_params import LIMBS


def _in_limits(limb, q1, q2, tol=1e-9):
    lim = LIMBS[limb].limits
    return (lim[0][0] - tol <= q1 <= lim[0][1] + tol
            and lim[1][0] - tol <= q2 <= lim[1][1] + tol)


# Targets chosen so the exact IK solution is strictly inside the joint limits.
REACHABLE = {
    "arm_right": [(0.45, 0.10), (0.30, 0.30), (0.50, -0.15), (0.20, 0.35), (0.40, 0.0)],
    "arm_left": [(0.45, 0.10), (0.30, 0.30), (0.50, -0.15), (0.20, 0.35), (0.40, 0.0)],
    "leg_right": [(0.70, -0.20), (0.50, -0.50), (0.80, 0.10), (0.40, -0.60), (0.60, 0.0)],
    "leg_left": [(0.70, -0.20), (0.50, -0.50), (0.80, 0.10), (0.40, -0.60), (0.60, 0.0)],
}


@pytest.mark.parametrize("limb", list(LIMBS.keys()))
def test_reachable_roundtrip(limb):
    for (x, y) in REACHABLE[limb]:
        q1, q2 = solve(limb, x, y)
        assert _in_limits(limb, q1, q2)
        fx, fy = forward(limb, q1, q2)
        assert math.hypot(fx - x, fy - y) < 1e-6, (limb, x, y, q1, q2, fx, fy)


@pytest.mark.parametrize("limb", list(LIMBS.keys()))
def test_unreachable_clamps_to_boundary(limb):
    p = LIMBS[limb]
    for theta in (0.0, 0.4, -0.5, 1.0):
        x = 10.0 * math.cos(theta)
        y = 10.0 * math.sin(theta)
        q1, q2 = solve(limb, x, y)
        fx, fy = forward(limb, q1, q2)
        # FK must land on the outer workspace boundary along the target direction.
        tx = p.max_reach * math.cos(theta)
        ty = p.max_reach * math.sin(theta)
        assert math.hypot(fx - tx, fy - ty) < 1e-6
        assert abs(math.hypot(fx, fy) - p.max_reach) < 1e-9


@pytest.mark.parametrize("limb", list(LIMBS.keys()))
def test_inner_hole_clamps_outward(limb):
    p = LIMBS[limb]
    q1, q2 = solve(limb, 1e-6, 0.0)
    fx, fy = forward(limb, q1, q2)
    # Inside the annular hole: clamp to min_reach circle (joint limits may pull
    # slightly off it for the folded pose, so only check we escaped the hole).
    assert math.hypot(fx, fy) >= p.min_reach - 1e-9
    cx, cy = clamp_to_workspace(limb, 0.0, 0.0)
    assert math.hypot(cx, cy) == pytest.approx(p.min_reach)


@pytest.mark.parametrize("limb", list(LIMBS.keys()))
def test_limits_respected_over_sweep(limb):
    p = LIMBS[limb]
    for i in range(64):
        theta = -math.pi + (2 * math.pi) * i / 63.0
        for r in (0.05, 0.3 * p.max_reach, 0.7 * p.max_reach, 0.99 * p.max_reach, 2.0):
            q1, q2 = solve(limb, r * math.cos(theta), r * math.sin(theta))
            assert _in_limits(limb, q1, q2), (limb, theta, r, q1, q2)


@pytest.mark.parametrize("limb", list(LIMBS.keys()))
def test_both_elbow_branches_sane(limb):
    p = LIMBS[limb]
    x, y = REACHABLE[limb][0]
    for branch in (+1, -1):
        q1, q2 = solve(limb, x, y, branch=branch)
        # Branch sign is honored before limit-clamping; the configured branch is
        # always inside the limits, the opposite one may be clamped to 0.
        if branch == p.branch:
            fx, fy = forward(limb, q1, q2)
            assert math.hypot(fx - x, fy - y) < 1e-6
            assert q2 * branch >= -1e-12
        else:
            assert q2 * branch >= -1e-12 or q2 == 0.0


@pytest.mark.parametrize("limb", list(LIMBS.keys()))
def test_configured_branch_matches_limb(limb):
    p = LIMBS[limb]
    x, y = REACHABLE[limb][1]
    q1, q2 = solve(limb, x, y)
    if p.branch < 0:
        assert q2 <= 1e-12  # arms: elbow-up, wrist angle <= 0
    else:
        assert q2 >= -1e-12  # legs: knee-down, knee angle >= 0
