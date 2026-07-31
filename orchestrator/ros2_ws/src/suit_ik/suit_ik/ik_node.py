"""IK node: /suit/target/<limb> (PointStamped, limb root frame) ->
/suit/command/<limb> (JointTrajectoryPoint) at 100 Hz (docs/network-map.md §8, §11).

Rate-limited interpolation: joints slew toward the latest IK solution at
max_joint_vel; the bridge packs the point into JOINT_CMD frames.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectoryPoint

from .ik_core import solve
from .limb_params import LIMB_NAMES, LIMBS


class IkNode(Node):
    def __init__(self) -> None:
        super().__init__("suit_ik")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("max_joint_vel", 2.0)  # rad/s slew limit

        self._max_vel = float(self.get_parameter("max_joint_vel").value)
        rate = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / rate

        self._goal: dict[str, tuple[float, float] | None] = {n: None for n in LIMB_NAMES}
        self._q: dict[str, list[float]] = {n: [0.0, 0.0] for n in LIMB_NAMES}
        self._pubs = {}
        self._subs = []
        for limb in LIMB_NAMES:
            self._pubs[limb] = self.create_publisher(
                JointTrajectoryPoint, f"/suit/command/{limb}", 10)
            self._subs.append(self.create_subscription(
                PointStamped, f"/suit/target/{limb}",
                lambda msg, limb=limb: self._on_target(limb, msg), 10))
        self._timer = self.create_timer(self._dt, self._tick)

    def _on_target(self, limb: str, msg: PointStamped) -> None:
        self._goal[limb] = solve(limb, msg.point.x, msg.point.y)

    def _tick(self) -> None:
        step = self._max_vel * self._dt
        for limb in LIMB_NAMES:
            goal = self._goal[limb]
            if goal is None:
                continue  # no target yet: stay quiet, edge nodes hold their own state
            q = self._q[limb]
            vel = [0.0, 0.0]
            for i in (0, 1):
                err = goal[i] - q[i]
                d = math.copysign(min(abs(err), step), err) if err else 0.0
                q[i] += d
                vel[i] = d / self._dt
            lim = LIMBS[limb].limits
            q[0] = min(max(q[0], lim[0][0]), lim[0][1])
            q[1] = min(max(q[1], lim[1][0]), lim[1][1])
            msg = JointTrajectoryPoint()
            msg.positions = [q[0], q[1]]
            msg.velocities = vel
            msg.effort = [0.0, 0.0]
            self._pubs[limb].publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
