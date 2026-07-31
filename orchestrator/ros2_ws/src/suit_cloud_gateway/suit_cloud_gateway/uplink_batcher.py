"""Rolling suit state -> telemetry_batch payload — PURE python
(docs/link-protocol.md §2.3). All values SI floats; the cloud never sees
packed centi-units. build() is called at 5 Hz by the gateway node.
"""

from __future__ import annotations

from typing import Any


class UplinkBatcher:
    def __init__(self) -> None:
        self._sources: dict[str, Any] = {}
        self._dropped = 0

    # ------------------------------------------------------------------ inputs
    def update_limb(self, limb: str, joints: list[dict], imu_ok: bool = True) -> None:
        """joints: [{"j": 0, "pos": rad, "vel": rad/s, "eff": Nm}, ...]"""
        self._sources[limb] = {"joints": joints, "imu_ok": bool(imu_ok)}

    def update_torso_imu(self, quat: tuple, acc: tuple) -> None:
        self._sources["torso_imu"] = {"quat": [float(v) for v in quat],
                                      "acc": [float(v) for v in acc]}

    def update_power(self, pack_v: float, current_a: float, soc: int,
                     t_max: float, faults: int) -> None:
        self._sources["power"] = {"pack_v": float(pack_v), "current_a": float(current_a),
                                  "soc": int(soc), "t_max": float(t_max),
                                  "faults": int(faults)}

    def update_aero(self, ias: float, q: float, flaps: list[float]) -> None:
        self._sources["aero"] = {"ias": float(ias), "q": float(q),
                                 "flaps": [float(f) for f in flaps]}

    def update_safety(self, state: str, estop: bool, hb_ok: bool) -> None:
        self._sources["safety"] = {"state": state, "estop": bool(estop),
                                   "hb_ok": bool(hb_ok)}

    def update_env(self, temp_c: float) -> None:
        self._sources["env"] = {"temp_c": float(temp_c)}

    def add_dropped(self, n: int = 1) -> None:
        self._dropped += int(n)

    # ------------------------------------------------------------------ output
    def build(self, window_ms: int = 200) -> dict:
        payload = {
            "window_ms": int(window_ms),
            "sources": dict(self._sources),
            "dropped_count": self._dropped,
        }
        self._dropped = 0
        return payload
