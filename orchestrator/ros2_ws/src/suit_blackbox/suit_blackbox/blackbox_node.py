"""Blackbox node: decimated key-topic logging into the mmap ring; snapshot on
e-stop rising edge (docs/network-map.md §8 topics).

Record codes: 1 joint_states, 2 bms, 3 safety, 4 odometry, 5 link.
"""

from __future__ import annotations

import os
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from suit_msgs.msg import BmsStatus, LinkStatus, SafetyState

from .ringlog import DEFAULT_SIZE, RingLog

CODE_JOINTS = 1
CODE_BMS = 2
CODE_SAFETY = 3
CODE_ODOM = 4
CODE_LINK = 5


def _writable_dir(preferred: str, fallback: str) -> Path:
    for cand in (preferred, fallback):
        p = Path(cand)
        try:
            p.mkdir(parents=True, exist_ok=True)
            probe = p / ".ps_probe"
            probe.touch()
            probe.unlink()
            return p
        except OSError:
            continue
    return Path(fallback)


class BlackboxNode(Node):
    def __init__(self) -> None:
        super().__init__("suit_blackbox")
        self.declare_parameter("dir", "/var/lib/powersuit")
        self.declare_parameter("fallback_dir", "/tmp")
        self.declare_parameter("size_bytes", DEFAULT_SIZE)
        self.declare_parameter("rate_hz", 10.0)

        base = _writable_dir(str(self.get_parameter("dir").value),
                             str(self.get_parameter("fallback_dir").value))
        self._dir = base
        self._ring = RingLog(base / "blackbox.ring",
                             int(self.get_parameter("size_bytes").value))
        self.get_logger().info(
            f"ring at {base / 'blackbox.ring'} capacity={self._ring.capacity} records")

        self._latest: dict[int, tuple] = {}
        self._estop_latched = False

        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(BmsStatus, "/suit/power/bms", self._on_bms, 10)
        self.create_subscription(SafetyState, "/suit/safety/state", self._on_safety, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 10)
        self.create_subscription(LinkStatus, "/suit/link/status", self._on_link, 10)

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)

    # Cache latest per topic; the 10 Hz timer decimates into the ring.
    def _on_joints(self, msg: JointState) -> None:
        vals = tuple(msg.position[:6])
        self._latest[CODE_JOINTS] = (vals, b"joints")

    def _on_bms(self, msg: BmsStatus) -> None:
        vals = (msg.pack_v, msg.current_a, float(msg.soc_pct),
                float(msg.temp_max_c), float(msg.fault_bits))
        self._latest[CODE_BMS] = (vals, b"bms")

    def _on_safety(self, msg: SafetyState) -> None:
        vals = (float(msg.state), float(msg.estop_latched),
                float(msg.heartbeat_ok), float(msg.estop_cause))
        self._latest[CODE_SAFETY] = (vals, msg.source.encode()[:22])
        if msg.estop_latched and not self._estop_latched:
            self._snapshot()  # rising edge
        self._estop_latched = bool(msg.estop_latched)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self._latest[CODE_ODOM] = ((p.x, p.y, p.z, v.x, v.y, v.z), b"odom")

    def _on_link(self, msg: LinkStatus) -> None:
        vals = (msg.rtt_ms, msg.loss_pct, float(msg.switches),
                float(msg.rejected), float(msg.connected))
        self._latest[CODE_LINK] = (vals, msg.bearer.encode()[:22])

    def _tick(self) -> None:
        now = time.time()
        for code, (vals, tag) in self._latest.items():
            self._ring.append(now, code, vals, tag)
        self._latest.clear()

    def _snapshot(self) -> None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        path = self._dir / f"blackbox-{stamp}.bin"
        try:
            n = self._ring.snapshot(path)
            self.get_logger().warn(f"estop: snapshot {n} records -> {path}")
        except OSError as exc:
            self.get_logger().error(f"snapshot failed: {exc}")

    def destroy_node(self) -> None:
        self._ring.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlackboxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
