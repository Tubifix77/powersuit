"""Aggregator node: 4 x /suit/telemetry/<limb> (JointState) -> /joint_states
at 100 Hz for robot_state_publisher (docs/network-map.md §8)."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .aggregator_core import LIMB_ORDER, aggregate


class JointStateAggregator(Node):
    def __init__(self) -> None:
        super().__init__("suit_joint_state_aggregator")
        self.declare_parameter("rate_hz", 100.0)
        self._latest: dict[str, tuple] = {}
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        for limb in LIMB_ORDER:
            self.create_subscription(
                JointState, f"/suit/telemetry/{limb}",
                lambda msg, limb=limb: self._on_limb(limb, msg), 10)
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)

    def _on_limb(self, limb: str, msg: JointState) -> None:
        self._latest[limb] = (list(msg.name), list(msg.position),
                              list(msg.velocity), list(msg.effort))

    def _tick(self) -> None:
        if not self._latest:
            return
        names, pos, vel, eff = aggregate(self._latest)
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = names
        out.position = pos
        out.velocity = vel
        out.effort = eff
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
