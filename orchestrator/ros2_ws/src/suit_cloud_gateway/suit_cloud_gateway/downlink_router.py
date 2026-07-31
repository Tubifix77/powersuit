"""Downlink -> ROS routing (rclpy side; whitelist already enforced by ws_client).

advisory -> visualization_msgs/Marker on /suit/hud/telemetry_display with a
[CLOUD] provenance tag + std_msgs/String on /suit/advisory. TTS binary chunks ->
suit_msgs/AudioChunk codec=1 (ADPCM) on /suit/audio/downlink; the bridge re-frames
for CAN descent with SYNC insertion. Link state -> suit_msgs/LinkStatus.
"""

from __future__ import annotations

from std_msgs.msg import String
from visualization_msgs.msg import Marker

from suit_msgs.msg import AudioChunk, LinkStatus

_SEVERITY_ORDER = ("info", "notice", "warning", "critical")


class DownlinkRouter:
    def __init__(self, node) -> None:
        self._node = node
        self._pub_marker = node.create_publisher(Marker, "/suit/hud/telemetry_display", 5)
        self._pub_advisory = node.create_publisher(String, "/suit/advisory", 10)
        self._pub_audio = node.create_publisher(AudioChunk, "/suit/audio/downlink", 10)
        self._pub_link = node.create_publisher(LinkStatus, "/suit/link/status", 10)
        self._marker_id = 0

    # ------------------------------------------------------------------ advisory
    def on_advisory(self, payload: dict) -> None:
        severity = str(payload.get("severity", "info"))
        title = str(payload.get("title", ""))
        body = str(payload.get("body", ""))
        text = f"[CLOUD] {severity.upper()}: {title}"

        msg = String()
        msg.data = f"{text} — {body}"
        self._pub_advisory.publish(msg)

        marker = Marker()
        marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.header.frame_id = "helmet"
        marker.ns = "cloud_advisory"
        self._marker_id = (self._marker_id + 1) % 1024
        marker.id = self._marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.text = text
        marker.scale.z = 0.05
        marker.pose.position.z = 0.10
        marker.color.a = 1.0
        idx = _SEVERITY_ORDER.index(severity) if severity in _SEVERITY_ORDER else 0
        marker.color.r = (0.2, 0.4, 1.0, 1.0)[idx]
        marker.color.g = (0.8, 0.8, 0.7, 0.1)[idx]
        marker.color.b = (1.0, 0.2, 0.0, 0.0)[idx]
        ttl = float((payload.get("hud") or {}).get("ttl_s", 10))
        marker.lifetime.sec = int(ttl)
        self._pub_marker.publish(marker)

    # ------------------------------------------------------------------ tts audio
    def on_tts_chunk(self, codec: int, stream_id: int, seq: int, payload: bytes) -> None:
        msg = AudioChunk()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.node_id = 6  # helmet
        msg.codec = AudioChunk.CODEC_ADPCM8K if codec == 0x01 else AudioChunk.CODEC_PCM16
        msg.sample_rate = 8000
        msg.seq = seq
        msg.data = list(payload)
        self._pub_audio.publish(msg)

    # ------------------------------------------------------------------ link state
    def publish_link_status(self, *, bearer: str, rtt_ms: float, loss_pct: float,
                            switches: int, rejected: int, connected: bool) -> None:
        msg = LinkStatus()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.bearer = bearer
        msg.rtt_ms = float(rtt_ms)
        msg.loss_pct = float(loss_pct)
        msg.switches = int(switches)
        msg.rejected = int(rejected)
        msg.connected = bool(connected)
        self._pub_link.publish(msg)
