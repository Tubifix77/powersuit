"""Joint-state merge logic — PURE python (no rclpy; unit-testable).

aggregate() merges per-limb JointState-shaped tuples into one deterministic
(name, position, velocity, effort) set, ordered by LIMB_ORDER then by the
per-limb name order. Stale/missing limbs are simply absent (robot_state_publisher
tolerates partial joint states; the EKF does not consume /joint_states).
"""

from __future__ import annotations

LIMB_ORDER = ("arm_right", "arm_left", "leg_right", "leg_left")

# (names, positions, velocities, efforts)
LimbState = tuple[list[str], list[float], list[float], list[float]]


def aggregate(by_limb: dict[str, LimbState],
              limb_order: tuple[str, ...] = LIMB_ORDER) -> LimbState:
    names: list[str] = []
    pos: list[float] = []
    vel: list[float] = []
    eff: list[float] = []
    for limb in limb_order:
        state = by_limb.get(limb)
        if state is None:
            continue
        jn, jp, jv, je = state
        n = len(jn)
        # Tolerate ragged arrays: missing vel/eff pad with zeros, extras are cut.
        jp = (list(jp) + [0.0] * n)[:n]
        jv = (list(jv) + [0.0] * n)[:n]
        je = (list(je) + [0.0] * n)[:n]
        for i, name in enumerate(jn):
            if name in names:  # last writer wins on duplicates
                k = names.index(name)
                pos[k], vel[k], eff[k] = float(jp[i]), float(jv[i]), float(je[i])
                continue
            names.append(name)
            pos.append(float(jp[i]))
            vel.append(float(jv[i]))
            eff.append(float(je[i]))
    return (names, pos, vel, eff)
