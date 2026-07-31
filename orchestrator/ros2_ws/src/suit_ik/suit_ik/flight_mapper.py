"""Flight mapper: /suit/aero/target_geometry (PoseArray, N<=12,
pose.position.z in [-1,1] = normalized flap deflection) -> /suit/flaps/cmd
(Float32MultiArray, permille) at 20 Hz.

This is the Node 8 termination mandated by docs/network-map.md §12.5: the
PoseArray never crosses the CAN boundary; the bridge packs FLAP_CMD frames.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32MultiArray

N_FLAPS = 12


class FlightMapper(Node):
    def __init__(self) -> None:
        super().__init__("suit_flight_mapper")
        self.declare_parameter("rate_hz", 20.0)
        self._targets_pm = [0.0] * N_FLAPS
        self._have_target = False
        self._pub = self.create_publisher(Float32MultiArray, "/suit/flaps/cmd", 10)
        self._sub = self.create_subscription(
            PoseArray, "/suit/aero/target_geometry", self._on_geometry, 10)
        rate = float(self.get_parameter("rate_hz").value)
        self._timer = self.create_timer(1.0 / rate, self._tick)

    def _on_geometry(self, msg: PoseArray) -> None:
        for i, pose in enumerate(msg.poses[:N_FLAPS]):
            z = min(max(float(pose.position.z), -1.0), 1.0)
            self._targets_pm[i] = z * 1000.0
        self._have_target = True

    def _tick(self) -> None:
        if not self._have_target:
            return
        out = Float32MultiArray()
        out.data = list(self._targets_pm)
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlightMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
