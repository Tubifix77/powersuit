"""Intent -> action descriptors — PURE python (no rclpy).

voice_node dispatches on these; keeping the table separate from the matcher makes
the local action surface auditable (nothing here can reach the SAFETY plane except
via the /suit/estop service, which the bridge owns).
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class IntentAction:
    name: str
    kind: str                 # "speak" | "aero" | "hud" | "estop" | "cloud"
    speak_template: str = ""  # str.format-able with cached state keys
    aero_deflection: float | None = None   # normalized flap target for "aero"
    extra: dict = field(default_factory=dict)


ACTIONS: dict[str, IntentAction] = {
    "status_report": IntentAction(
        "status_report", "speak",
        speak_template="Suit {state}. Battery {soc} percent. Heartbeat {hb}.",
    ),
    "power_level": IntentAction(
        "power_level", "speak",
        speak_template="Battery {soc} percent, {pack_v:.1f} volts.",
    ),
    "deploy_airbrakes": IntentAction("deploy_airbrakes", "aero", aero_deflection=1.0),
    "retract_airbrakes": IntentAction("retract_airbrakes", "aero", aero_deflection=0.0),
    "engage_estop": IntentAction("engage_estop", "estop", extra={"engage": True}),
    "clear_estop": IntentAction("clear_estop", "estop", extra={"engage": False}),
    "hud_brightness": IntentAction("hud_brightness", "hud"),
    "cloud_query": IntentAction("cloud_query", "cloud"),
}
