// 100 Hz safety heartbeat (docs/safety.md §1-2, docs/network-map.md §3.1,
// §4). Node 8's bridge is the ONLY emitter of HEARTBEAT records; the gate
// below is the single decision point for whether a beat goes out at all.
//
// Gate (all must hold): transport healthy (last good frame < 100 ms) AND no
// e-stop latched AND rclcpp is ok. When shut, emit nothing — never fake
// liveness. It is exposed as a pure function so the truth table is directly
// unit-testable.
#ifndef SUIT_CANSPI_BRIDGE_HEARTBEAT_HPP_
#define SUIT_CANSPI_BRIDGE_HEARTBEAT_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

extern "C" {
#include "powersuit_proto/spi_frame.h"
}

namespace suit_canspi_bridge {

struct HeartbeatGateInputs {
  bool transport_healthy;  // last good frame received < 100 ms ago
  bool estop_latched;
  bool rclcpp_ok;
};

// Pure gate function — the truth table under test.
inline bool heartbeat_gate_open(const HeartbeatGateInputs& in) {
  return in.transport_healthy && !in.estop_latched && in.rclcpp_ok;
}

class HeartbeatEmitter {
public:
  HeartbeatEmitter() = default;

  // Call at (at least) 100 Hz. Emits a HEARTBEAT record when the gate is
  // open AND the 10 ms period (PS_HEARTBEAT_PERIOD_MS) has elapsed since the
  // last emission; returns std::nullopt otherwise (gate shut or too soon).
  std::optional<ps_can_record_t> tick(std::chrono::steady_clock::time_point now,
                                       const HeartbeatGateInputs& gate,
                                       bool cloud_up, bool degraded,
                                       uint8_t src_state, uint32_t uptime_ms);

  uint16_t seq() const { return seq_; }

private:
  uint16_t seq_ = 0;
  bool have_last_emit_ = false;
  std::chrono::steady_clock::time_point last_emit_{};
};

}  // namespace suit_canspi_bridge

#endif  // SUIT_CANSPI_BRIDGE_HEARTBEAT_HPP_
