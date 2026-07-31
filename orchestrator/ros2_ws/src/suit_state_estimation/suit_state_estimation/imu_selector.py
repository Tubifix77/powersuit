"""IMU selector: relay /suit/imu/torso -> /imu/data with enforced covariance;
publish a diagnostic and HOLD (stop republishing) if the torso IMU goes silent
for > 500 ms. Phase-1 EKF fuses the torso IMU only (network-map §12.7); a
kinematics-compensated virtual torso IMU is the documented upgrade path.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import Imu


class ImuSelector(Node):
    def __init__(self) -> None:
        super().__init__("suit_imu_selector")
        self.declare_parameter("silence_timeout_s", 0.5)
        self.declare_parameter("orientation_stddev", 0.02)   # rad
        self.declare_parameter("angular_vel_stddev", 0.01)   # rad/s
        self.declare_parameter("linear_acc_stddev", 0.10)    # m/s^2

        self._timeout = float(self.get_parameter("silence_timeout_s").value)
        self._last_rx: float | None = None
        self._silent_reported = False

        self._pub = self.create_publisher(Imu, "/imu/data", 50)
        self._pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 5)
        self.create_subscription(Imu, "/suit/imu/torso", self._on_imu, 50)
        self.create_timer(0.1, self._watchdog)

    def _cov(self, stddev: float) -> list[float]:
        v = stddev * stddev
        return [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v]

    def _on_imu(self, msg: Imu) -> None:
        self._last_rx = self.get_clock().now().nanoseconds * 1e-9
        if self._silent_reported:
            self._silent_reported = False
            self._diag(DiagnosticStatus.OK, "torso IMU restored")
        out = Imu()
        out.header = msg.header
        out.header.frame_id = "imu_torso"
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        out.orientation_covariance = self._cov(
            float(self.get_parameter("orientation_stddev").value))
        out.angular_velocity_covariance = self._cov(
            float(self.get_parameter("angular_vel_stddev").value))
        out.linear_acceleration_covariance = self._cov(
            float(self.get_parameter("linear_acc_stddev").value))
        self._pub.publish(out)

    def _watchdog(self) -> None:
        if self._last_rx is None or self._silent_reported:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_rx > self._timeout:
            self._silent_reported = True
            self._diag(DiagnosticStatus.ERROR,
                       f"torso IMU silent > {self._timeout:.1f}s — holding /imu/data")

    def _diag(self, level: int, message: str) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.level = level if isinstance(level, bytes) else bytes([level])
        st.name = "suit_imu_selector"
        st.message = message
        st.hardware_id = "node_flight_actuation/imu"
        arr.status.append(st)
        self._pub_diag.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
