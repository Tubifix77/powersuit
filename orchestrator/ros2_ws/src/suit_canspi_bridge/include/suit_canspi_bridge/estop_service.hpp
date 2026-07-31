// /suit/estop service owner (docs/safety.md §3, docs/network-map.md §3.1).
// engage(): 3x ESTOP(cause=OPERATOR) + latch + heartbeat stops (the gate in
// heartbeat.hpp reads is_latched()). disengage(): CLEAR_ESTOP with a
// strictly monotonic counter persisted to disk, then clears the latch.
#ifndef SUIT_CANSPI_BRIDGE_ESTOP_SERVICE_HPP_
#define SUIT_CANSPI_BRIDGE_ESTOP_SERVICE_HPP_

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <suit_msgs/srv/set_estop.hpp>

extern "C" {
#include "powersuit_proto/spi_frame.h"
}

namespace suit_canspi_bridge {

class EstopService {
public:
  // counter_path: param-configured persistence file; falls back to /tmp on
  // any failure to open/create the configured path (e.g. no write access to
  // /var/lib/powersuit outside a provisioned Pi).
  EstopService(rclcpp::Node* node, std::string counter_path,
               std::function<void(const std::vector<ps_can_record_t>&)> emit,
               uint8_t origin_node);

  bool is_latched() const { return latched_.load(); }
  uint8_t latched_cause() const { return latched_cause_.load(); }

private:
  void handle(const std::shared_ptr<suit_msgs::srv::SetEstop::Request> req,
              std::shared_ptr<suit_msgs::srv::SetEstop::Response> resp);

  uint32_t load_and_bump_counter();
  uint32_t uptime_ms() const;

  rclcpp::Node* node_;
  std::string counter_path_;
  std::function<void(const std::vector<ps_can_record_t>&)> emit_;
  uint8_t origin_node_;
  rclcpp::Service<suit_msgs::srv::SetEstop>::SharedPtr service_;
  rclcpp::Time start_time_;

  std::mutex mu_;
  uint16_t seq_ = 0;
  std::atomic<bool> latched_{false};
  std::atomic<uint8_t> latched_cause_{0};
};

}  // namespace suit_canspi_bridge

#endif  // SUIT_CANSPI_BRIDGE_ESTOP_SERVICE_HPP_
