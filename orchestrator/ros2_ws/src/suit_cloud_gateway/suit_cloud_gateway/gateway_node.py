"""Cloud gateway node: ROS graph <-> LinkClient (asyncio in a worker thread).

Uplink: telemetry_batch at 5 Hz from cached suit state, voice queries from
/suit/cloud/query, VOX audio from /suit/audio/uplink while a query window is
open. Downlink (whitelisted only): advisory/tts/link state via DownlinkRouter.
Bearer selection per docs/link-protocol.md §5 with TCP-connect probing of idle
candidates.
"""

from __future__ import annotations

import asyncio
import os
import ssl
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, String

from suit_msgs.msg import AudioChunk, BmsStatus, SafetyState

from .downlink_router import DownlinkRouter
from .link_monitor import Bearer, LinkMonitor, load_bearers
from .offline_queue import OfflineQueue
from .uplink_batcher import UplinkBatcher
from .ws_client import LinkClient

LIMBS = ("arm_right", "arm_left", "leg_right", "leg_left")
_STATE_NAMES = {0: "BOOT", 1: "STANDBY", 2: "OPERATIONAL",
                3: "PASSIVE", 4: "ESTOP", 5: "FAULT"}
VOICE_WINDOW_S = 10.0


def _default_links_yaml() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("suit_cloud_gateway"),
                            "config", "links.yaml")
    except Exception:
        return os.path.join(os.path.dirname(__file__), "..", "config", "links.yaml")


class GatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("suit_cloud_gateway")
        self.declare_parameter("links_yaml", "")
        self.declare_parameter("enable_tls_verify", True)
        self.declare_parameter("suit_id", "powersuit-01")
        self.declare_parameter("batch_hz", 5.0)
        self.declare_parameter("link_stats_period_s", 5.0)

        links_path = str(self.get_parameter("links_yaml").value) or _default_links_yaml()
        bearers, extras = load_bearers(links_path)
        if not bearers:
            bearers = [Bearer("wifi", "wss://127.0.0.1:8443/link", 2, 80.0)]
        self._monitor = LinkMonitor(bearers)
        self._ssl_ctx = self._make_ssl(extras)

        self._batcher = UplinkBatcher()
        self._queue = OfflineQueue()
        self._router = DownlinkRouter(self)
        self._client: LinkClient | None = None
        self._client_stop: asyncio.Event | None = None
        self._client_task: asyncio.Task | None = None
        self._downlink_buffered = 0
        self._voice_deadline = 0.0
        self._audio_up_seq = 0

        # ---- ROS side ----------------------------------------------------
        self.create_subscription(BmsStatus, "/suit/power/bms", self._on_bms, 10)
        self.create_subscription(SafetyState, "/suit/safety/state", self._on_safety, 10)
        self.create_subscription(Float32MultiArray, "/suit/aero/state", self._on_aero, 10)
        self.create_subscription(Imu, "/suit/imu/torso", self._on_torso_imu, 10)
        self.create_subscription(String, "/suit/cloud/query", self._on_query, 10)
        self.create_subscription(AudioChunk, "/suit/audio/uplink", self._on_audio_up, 10)
        for limb in LIMBS:
            self.create_subscription(
                JointState, f"/suit/telemetry/{limb}",
                lambda msg, limb=limb: self._on_limb(limb, msg), 10)

        batch_hz = float(self.get_parameter("batch_hz").value)
        self.create_timer(1.0 / batch_hz, self._send_batch)
        self.create_timer(float(self.get_parameter("link_stats_period_s").value),
                          self._send_link_stats)
        self.create_timer(1.0, self._publish_link_status)
        self.create_timer(1.0, self._evaluate_bearers)

        # ---- asyncio side --------------------------------------------------
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop_main, name="gw-asyncio",
                                        daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ ssl
    def _make_ssl(self, extras: dict) -> ssl.SSLContext:
        ctx = ssl.create_default_context()
        ca = extras.get("ca_bundle")
        if ca and os.path.isfile(str(ca)):
            ctx.load_verify_locations(str(ca))
        if not bool(self.get_parameter("enable_tls_verify").value):
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
            self.get_logger().warn("TLS verification DISABLED (bench mode)")
        return ctx

    # ------------------------------------------------------------------ asyncio plumbing
    def _loop_main(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.create_task(self._probe_loop())
        self._loop.run_forever()

    def _post(self, coro) -> None:
        if self._loop.is_running():
            asyncio.run_coroutine_threadsafe(coro, self._loop)

    async def _probe_loop(self) -> None:
        """TCP-connect probe of non-active bearers (§5: idle-candidate probing)."""
        while True:
            for name in list(self._monitor._bearers):
                if name == self._monitor.active:
                    continue  # active bearer health comes from the live socket
                b = self._monitor.bearer(name)
                host, port = self._host_port(b.url)
                t0 = time.monotonic()
                try:
                    fut = asyncio.open_connection(host, port)
                    _r, w = await asyncio.wait_for(fut, timeout=2.0)
                    rtt = (time.monotonic() - t0) * 1000.0
                    w.close()
                    self._monitor.update(name, connected=True, rtt_ms=rtt, loss_pct=0.0)
                except Exception:
                    self._monitor.update(name, connected=False)
            await asyncio.sleep(2.0)

    @staticmethod
    def _host_port(url: str) -> tuple[str, int]:
        from urllib.parse import urlparse
        u = urlparse(url)
        return (u.hostname or "127.0.0.1", u.port or (443 if u.scheme == "wss" else 80))

    def _start_client(self, bearer_name: str) -> None:
        b = self._monitor.bearer(bearer_name)
        old_stop, old_task = self._client_stop, self._client_task

        client = LinkClient(
            b.url,
            suit_id=str(self.get_parameter("suit_id").value),
            ssl_context=self._ssl_ctx,
            on_advisory=self._router.on_advisory,
            on_tts_meta=lambda p: None,
            on_tts_chunk=self._on_tts_chunk,
            on_session=lambda ev, info, name=bearer_name: self._on_session(name, ev, info),
        )
        # Session continuity across bearers (resume, rejected counter).
        if self._client is not None:
            client.session_id = self._client.session_id
            client.last_rx_seq = self._client.last_rx_seq
            client.rejected = self._client.rejected
        self._client = client
        stop = asyncio.Event()
        self._client_stop = stop

        async def _run():
            # Make-before-break: the old task is stopped only once we are the
            # active client (committed in _on_session on hello_ack).
            await client.run(stop)

        def _spawn():
            self._client_task = self._loop.create_task(_run())
        self._loop.call_soon_threadsafe(_spawn)

        if old_stop is not None:
            def _retire():
                old_stop.set()
                if old_task is not None:
                    old_task.cancel()
            # Old socket lingers 5 s — long enough for hello_ack on the new one.
            self._loop.call_soon_threadsafe(self._loop.call_later, 5.0, _retire)

    def _on_session(self, bearer_name: str, event: str, _info: dict) -> None:
        if event == "connected":
            self._monitor.commit(bearer_name)
            self._monitor.update(bearer_name, connected=True,
                                 rtt_ms=self._client.rtt_ms or 1.0, loss_pct=0.0)
            self.get_logger().info(f"link up on {bearer_name}")
            self._post(self._drain_queue())
        else:
            self._monitor.update(bearer_name, connected=False)

    async def _drain_queue(self) -> None:
        client = self._client
        if client is None or not client.connected:
            return
        for kind, payload in self._queue.drain():
            if kind == "voice_query":
                await client.send_voice_query(**payload)

    # ------------------------------------------------------------------ bearer policy
    def _evaluate_bearers(self) -> None:
        client = self._client
        if client is not None:
            active = self._monitor.active
            if active is not None:
                self._monitor.update(active, connected=client.connected,
                                     rtt_ms=client.rtt_ms or 1.0, loss_pct=0.0)
        decision = self._monitor.evaluate()
        if decision.switch_to:
            self.get_logger().info(
                f"bearer switch -> {decision.switch_to} ({decision.reason}, "
                f"make_before_break={decision.make_before_break})")
            self._start_client(decision.switch_to)

    # ------------------------------------------------------------------ uplink caching
    def _on_limb(self, limb: str, msg: JointState) -> None:
        joints = [{"j": i, "pos": float(p), "vel": float(v), "eff": float(e)}
                  for i, (p, v, e) in enumerate(zip(msg.position, msg.velocity, msg.effort))]
        self._batcher.update_limb(limb, joints)

    def _on_torso_imu(self, msg: Imu) -> None:
        self._batcher.update_torso_imu(
            (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z),
            (msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z))

    def _on_bms(self, msg: BmsStatus) -> None:
        self._batcher.update_power(msg.pack_v, msg.current_a, msg.soc_pct,
                                   msg.temp_max_c, msg.fault_bits)

    def _on_safety(self, msg: SafetyState) -> None:
        self._batcher.update_safety(_STATE_NAMES.get(msg.state, "UNKNOWN"),
                                    msg.estop_latched, msg.heartbeat_ok)

    def _on_aero(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        if len(data) >= 3:
            self._batcher.update_aero(data[0], data[1], data[3:])

    def _on_query(self, msg: String) -> None:
        qid = f"q-{uuid.uuid4().hex[:8]}"
        self._voice_deadline = time.monotonic() + VOICE_WINDOW_S
        payload = {"query_id": qid, "text": msg.data, "context": self._safety_context()}
        client = self._client
        if client is not None and client.connected:
            self._post(client.send_voice_query(**payload))
        else:
            self._queue.put(("voice_query", payload))

    def _safety_context(self) -> dict:
        src = self._batcher._sources
        ctx = {}
        if "safety" in src:
            ctx["state"] = src["safety"]["state"]
        if "power" in src:
            ctx["soc"] = src["power"]["soc"]
        return ctx

    def _on_audio_up(self, msg: AudioChunk) -> None:
        if time.monotonic() > self._voice_deadline:
            return  # only stream mic audio up while a cloud query window is open
        client = self._client
        if client is None or not client.connected:
            return
        codec = 0x01 if msg.codec == AudioChunk.CODEC_ADPCM8K else 0x02
        seq = self._audio_up_seq
        self._audio_up_seq += 1
        self._post(client.send_audio_up(codec, 1, seq, bytes(msg.data)))

    # ------------------------------------------------------------------ timers
    def _send_batch(self) -> None:
        client = self._client
        if client is None or not client.connected:
            self._batcher.add_dropped(0)  # keep counter semantics; batch not sent
            return
        payload = self._batcher.build(window_ms=int(1000 / 5))
        self._post(client.send_telemetry(payload))

    def _send_link_stats(self) -> None:
        client = self._client
        active = self._monitor.active
        if client is None or not client.connected or active is None:
            return
        rtt, loss = self._monitor.stats(active)
        self._post(client.send_link_stats(active, rtt or 0.0, loss,
                                          self._monitor.switches))

    def _publish_link_status(self) -> None:
        client = self._client
        active = self._monitor.active or ""
        rtt, loss = (self._monitor.stats(active) if active else (0.0, 0.0))
        self._router.publish_link_status(
            bearer=active,
            rtt_ms=rtt or 0.0,
            loss_pct=loss,
            switches=self._monitor.switches,
            rejected=client.rejected if client else 0,
            connected=bool(client and client.connected),
        )

    def _on_tts_chunk(self, codec: int, stream_id: int, seq: int, payload: bytes) -> None:
        self._router.on_tts_chunk(codec, stream_id, seq, payload)

    # ------------------------------------------------------------------ shutdown
    def destroy_node(self) -> None:
        if self._client_stop is not None:
            self._loop.call_soon_threadsafe(self._client_stop.set)
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=3.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
