#include "suit_canspi_bridge/estop_service.hpp"

#include <cstring>
#include <filesystem>
#include <fstream>

extern "C" {
#include "powersuit_proto/can_id.h"
#include "powersuit_proto/wire.h"
}

namespace suit_canspi_bridge {

EstopService::EstopService(rclcpp::Node* node, std::string counter_path,
                            std::function<void(const std::vector<ps_can_record_t>&)> emit,
                            uint8_t origin_node)
    : node_(node),
      counter_path_(std::move(counter_path)),
      emit_(std::move(emit)),
      origin_node_(origin_node) {
  start_time_ = node_->get_clock()->now();
  service_ = node_->create_service<suit_msgs::srv::SetEstop>(
      "/suit/estop",
      [this](const std::shared_ptr<suit_msgs::srv::SetEstop::Request> req,
             std::shared_ptr<suit_msgs::srv::SetEstop::Response> resp) { handle(req, resp); });
}

uint32_t EstopService::uptime_ms() const {
  const auto dt = node_->get_clock()->now() - start_time_;
  return static_cast<uint32_t>(dt.nanoseconds() / 1000000);
}

uint32_t EstopService::load_and_bump_counter() {
  uint32_t last = 0;
  {
    std::ifstream in(counter_path_);
    if (in.good()) {
      in >> last;
    }
  }
  const uint32_t next = last + 1;

  std::error_code ec;
  std::filesystem::create_directories(std::filesystem::path(counter_path_).parent_path(), ec);

  std::ofstream out(counter_path_, std::ios::trunc);
  if (!out.good()) {
    // Fall back to /tmp and make it sticky: subsequent clears must keep
    // reading/writing the same file to preserve monotonicity.
    const std::string fallback =
        "/tmp/" + std::filesystem::path(counter_path_).filename().string();
    counter_path_ = fallback;
    std::ofstream fallback_out(counter_path_, std::ios::trunc);
    if (fallback_out.good()) {
      fallback_out << next;
    } else {
      RCLCPP_ERROR(node_->get_logger(),
                   "estop counter: failed to persist to %s (and fallback) — counter will not "
                   "survive a restart",
                   counter_path_.c_str());
    }
  } else {
    out << next;
  }
  return next;
}

void EstopService::handle(const std::shared_ptr<suit_msgs::srv::SetEstop::Request> req,
                           std::shared_ptr<suit_msgs::srv::SetEstop::Response> resp) {
  std::lock_guard<std::mutex> lock(mu_);

  if (req->engage) {
    std::vector<ps_can_record_t> records;
    records.reserve(PS_ESTOP_REPEAT);
    for (unsigned i = 0; i < PS_ESTOP_REPEAT; ++i) {
      ps_estop_t payload{};
      payload.cause = PS_ESTOP_OPERATOR;
      payload.origin_node = origin_node_;
      payload.seq = seq_++;
      payload.uptime_ms = uptime_ms();

      ps_can_record_t rec{};
      rec.id = ps_can_id_pack(PS_CLS_SAFETY, origin_node_, PS_NODE_BROADCAST, PS_T_ESTOP,
                               static_cast<uint8_t>(payload.seq & 0xFFu));
      rec.bus = PS_SPI_BUS_CAN1;  // ignored: downlink SAFETY forwards to both buses
      rec.dlc = sizeof(payload);
      rec.ts_ms = 0;
      std::memset(rec.data, 0, sizeof(rec.data));
      PS_WIRE_WRITE(rec.data, payload);
      records.push_back(rec);
    }
    latched_.store(true);
    latched_cause_.store(PS_ESTOP_OPERATOR);
    emit_(records);

    resp->ok = true;
    resp->detail = "estop engaged (cause=OPERATOR, reason=\"" + req->reason + "\")";
    RCLCPP_WARN(node_->get_logger(), "E-STOP engaged: %s", req->reason.c_str());
    return;
  }

  // Disengage: CLEAR_ESTOP with a strictly-monotonic persisted counter,
  // then clear the local latch (docs/safety.md §3).
  const uint32_t counter = load_and_bump_counter();

  ps_clear_estop_t payload{};
  payload.magic = PS_CLEAR_ESTOP_MAGIC;
  payload.counter = counter;

  ps_can_record_t rec{};
  rec.id = ps_can_id_pack(PS_CLS_SAFETY, origin_node_, PS_NODE_BROADCAST, PS_T_CLEAR_ESTOP, 0);
  rec.bus = PS_SPI_BUS_CAN1;
  rec.dlc = sizeof(payload);
  rec.ts_ms = 0;
  std::memset(rec.data, 0, sizeof(rec.data));
  PS_WIRE_WRITE(rec.data, payload);
  emit_({rec});

  latched_.store(false);
  latched_cause_.store(0);

  resp->ok = true;
  resp->detail = "estop cleared (counter=" + std::to_string(counter) + ")";
  RCLCPP_WARN(node_->get_logger(), "E-STOP cleared, counter=%u", counter);
}

}  // namespace suit_canspi_bridge
