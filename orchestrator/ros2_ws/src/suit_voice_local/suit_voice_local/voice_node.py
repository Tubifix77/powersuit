"""Local voice command node (offline-first, ARCHITECTURE.md).

/suit/voice/trigger (String: "wake" or transcribed text) -> local intent dispatch.
LOCAL intents only; cloud_query text is republished on /suit/cloud/query for the
gateway. Every handled intent gets an offline acknowledgment tone on
/suit/audio/downlink (codec=2 PCM16 @ 8 kHz; the bridge transcodes to ADPCM).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose, PoseArray

from suit_msgs.msg import AudioChunk, BmsStatus, SafetyState
from suit_msgs.srv import SetEstop

from .intents import ACTIONS
from .matcher import Matcher, default_grammar_path, normalize
from .tts_stub import SAMPLE_RATE, synth_ack

_STATE_NAMES = {0: "booting", 1: "standby", 2: "operational",
                3: "passive", 4: "e-stopped", 5: "faulted"}
N_FLAPS = 12


class VoiceNode(Node):
    def __init__(self) -> None:
        super().__init__("suit_voice_local")
        self.declare_parameter("grammar_path", "")
        path = str(self.get_parameter("grammar_path").value) or str(default_grammar_path())
        self._matcher = Matcher.from_file(path)
        self.get_logger().info(f"grammar loaded from {path}")

        self._bms: BmsStatus | None = None
        self._safety: SafetyState | None = None
        self._audio_seq = 0

        self.create_subscription(String, "/suit/voice/trigger", self._on_trigger, 10)
        self.create_subscription(BmsStatus, "/suit/power/bms", self._on_bms, 10)
        self.create_subscription(SafetyState, "/suit/safety/state", self._on_safety, 10)

        self._pub_audio = self.create_publisher(AudioChunk, "/suit/audio/downlink", 10)
        self._pub_aero = self.create_publisher(PoseArray, "/suit/aero/target_geometry", 10)
        self._pub_hud = self.create_publisher(Float32, "/suit/hud/brightness", 10)
        self._pub_cloud = self.create_publisher(String, "/suit/cloud/query", 10)
        self._estop_client = self.create_client(SetEstop, "/suit/estop")

    def _on_bms(self, msg: BmsStatus) -> None:
        self._bms = msg

    def _on_safety(self, msg: SafetyState) -> None:
        self._safety = msg

    # ------------------------------------------------------------------ dispatch
    def _on_trigger(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        if text.lower() == "wake":
            self._ack("status_report", 0)  # audible "listening" chirp
            return
        intent, slots, confidence = self._matcher.match(text)
        if intent is None:
            return
        self.get_logger().info(f"intent={intent} conf={confidence:.2f} slots={slots}")
        action = ACTIONS.get(intent)
        if action is None:
            return
        words = len(normalize(text))
        if action.kind == "speak":
            self._do_speak(action, words)
        elif action.kind == "aero":
            self._do_aero(action.aero_deflection or 0.0, words)
        elif action.kind == "estop":
            self._do_estop(bool(action.extra.get("engage")), text, words)
        elif action.kind == "hud":
            self._do_hud(slots, words)
        elif action.kind == "cloud":
            out = String()
            out.data = text
            self._pub_cloud.publish(out)
            self._ack("cloud_query", words)

    # ------------------------------------------------------------------ actions
    def _state_context(self) -> dict:
        state = self._safety.state if self._safety else 0
        return {
            "state": _STATE_NAMES.get(state, "unknown"),
            "soc": self._bms.soc_pct if self._bms else 0,
            "pack_v": self._bms.pack_v if self._bms else 0.0,
            "hb": ("nominal" if self._safety and self._safety.heartbeat_ok else "stopped"),
        }

    def _do_speak(self, action, words: int) -> None:
        text = action.speak_template.format(**self._state_context())
        self.get_logger().info(f"speak: {text}")
        self._ack(action.name, max(words, len(text.split()) // 3))

    def _do_aero(self, deflection: float, words: int) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "torso"
        for _ in range(N_FLAPS):
            pose = Pose()
            pose.position.z = float(deflection)
            msg.poses.append(pose)
        self._pub_aero.publish(msg)
        self._ack("deploy_airbrakes" if deflection > 0.0 else "retract_airbrakes", words)

    def _do_estop(self, engage: bool, reason: str, words: int) -> None:
        req = SetEstop.Request()
        req.engage = engage
        req.reason = f"voice: {reason}"
        if not self._estop_client.service_is_ready():
            self.get_logger().error("/suit/estop service not available")
            return
        self._estop_client.call_async(req)
        self._ack("engage_estop" if engage else "clear_estop", words)

    def _do_hud(self, slots: dict, words: int) -> None:
        raw = float(slots.get("value", 100.0))
        value = raw / 100.0 if raw > 1.0 else raw
        out = Float32()
        out.data = min(max(value, 0.0), 1.0)
        self._pub_hud.publish(out)
        self._ack("hud_brightness", words)

    def _ack(self, intent: str, words: int) -> None:
        pcm = synth_ack(intent, max(1, words))
        msg = AudioChunk()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_id = 6
        msg.codec = AudioChunk.CODEC_PCM16
        msg.sample_rate = SAMPLE_RATE
        msg.seq = self._audio_seq
        self._audio_seq += 1
        msg.data = list(pcm)
        self._pub_audio.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
