#include "suit_canspi_bridge/heartbeat.hpp"

extern "C" {
#include "powersuit_proto/can_id.h"
#include "powersuit_proto/wire.h"
}

#include <cstring>

namespace suit_canspi_bridge {

std::optional<ps_can_record_t> HeartbeatEmitter::tick(
    std::chrono::steady_clock::time_point now, const HeartbeatGateInputs& gate,
    bool cloud_up, bool degraded, uint8_t src_state, uint32_t uptime_ms) {
  if (!heartbeat_gate_open(gate)) {
    // Gate shut: never fake liveness. Do not touch seq/last_emit_ so a
    // freshly-reopened gate starts clean rather than bursting.
    return std::nullopt;
  }

  if (have_last_emit_) {
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_emit_);
    if (elapsed.count() < static_cast<int64_t>(PS_HEARTBEAT_PERIOD_MS)) {
      return std::nullopt;
    }
  }

  last_emit_ = now;
  have_last_emit_ = true;

  ps_heartbeat_t hb{};
  hb.seq = seq_++;
  hb.flags = static_cast<uint8_t>(
      (gate.estop_latched ? PS_HB_ESTOP_LATCHED : 0) |
      (degraded ? PS_HB_DEGRADED : 0) |
      (cloud_up ? PS_HB_CLOUD_UP : 0));
  hb.src_state = src_state;
  hb.uptime_ms = uptime_ms;

  ps_can_record_t rec{};
  rec.id = ps_can_id_pack(PS_CLS_SAFETY, PS_NODE_ORCH, PS_NODE_BROADCAST,
                           PS_T_HEARTBEAT, static_cast<uint8_t>(hb.seq & 0xFFu));
  rec.bus = PS_SPI_BUS_CAN1;  // ignored by the hub: downlink SAFETY always forwards to both buses (network-map §6)
  rec.dlc = sizeof(hb);
  rec.ts_ms = 0;  // filled in by the hub on the wire; not meaningful uplink-side
  std::memset(rec.data, 0, sizeof(rec.data));
  PS_WIRE_WRITE(rec.data, hb);

  return rec;
}

}  // namespace suit_canspi_bridge
