// suit_canspi_bridge: the Node 8 SPI link owner (docs/network-map.md §6-§8).
//
// Architecture: a 1 kHz transport thread owns the SPI/TCP transaction loop,
// builds each outbound 512-byte frame from whatever is queued (heartbeat,
// e-stop, joint/flap commands, XRCE-down, audio-down), and dispatches each
// inbound record to the relevant subsystem. The rclcpp executor runs on the
// main thread for subscriptions/services/timers. Publishers are called
// directly from the transport thread for TELEM/AUDIO synthesis — rclcpp
// publishers are safe to call from any thread.
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <pthread.h>
#include <sched.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <suit_msgs/msg/link_status.hpp>

#include "suit_canspi_bridge/audio_relay.hpp"
#include "suit_canspi_bridge/estop_service.hpp"
#include "suit_canspi_bridge/frame_parser.hpp"
#include "suit_canspi_bridge/heartbeat.hpp"
#include "suit_canspi_bridge/mock_transport.hpp"
#include "suit_canspi_bridge/spi_transport.hpp"
#include "suit_canspi_bridge/spidev_transport.hpp"
#include "suit_canspi_bridge/telem_synth.hpp"
#include "suit_canspi_bridge/xrce_demux.hpp"

extern "C" {
#include "powersuit_proto/can_id.h"
#include "powersuit_proto/wire.h"
}

using namespace std::chrono_literals;

namespace {

struct LimbCmdTarget {
  const char* name;
  uint8_t node_id;
};
constexpr LimbCmdTarget kLimbCmdTargets[4] = {
    {"arm_right", PS_NODE_ARM_R},
    {"arm_left", PS_NODE_ARM_L},
    {"leg_right", PS_NODE_LEG_R},
    {"leg_left", PS_NODE_LEG_L},
};

// Thread-safe FIFO of records queued by ROS callbacks (commands, estop,
// flaps) for the transport thread to drain into the next outbound frame.
class OutboundQueue {
public:
  void push(const std::vector<ps_can_record_t>& recs) {
    std::lock_guard<std::mutex> lock(mu_);
    for (const auto& r : recs) {
      q_.push_back(r);
    }
  }

  std::vector<ps_can_record_t> drain(size_t max_n) {
    std::lock_guard<std::mutex> lock(mu_);
    std::vector<ps_can_record_t> out;
    while (!q_.empty() && out.size() < max_n) {
      out.push_back(q_.front());
      q_.pop_front();
    }
    return out;
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(mu_);
    return q_.empty();
  }

private:
  mutable std::mutex mu_;
  std::deque<ps_can_record_t> q_;
};

void try_set_realtime(const rclcpp::Logger& logger) {
  sched_param sp{};
  sp.sched_priority = 80;
  const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  if (rc != 0) {
    RCLCPP_WARN(logger,
                "Could not set SCHED_FIFO on the transport thread (errno=%d: %s) — running "
                "best-effort scheduling. Grant CAP_SYS_NICE (or run under a PREEMPT_RT kernel "
                "with the right rtprio limits) to fix.",
                rc, std::strerror(rc));
  }
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("suit_canspi_bridge");
  auto logger = node->get_logger();

  node->declare_parameter("transport", "spidev");  // "spidev" | "mock"
  node->declare_parameter("device", "/dev/spidev0.0");
  node->declare_parameter("spi_speed_hz", 20000000);
  node->declare_parameter("gpiochip", "");
  node->declare_parameter("gpio_line", 0);
  node->declare_parameter("tcp_host", "127.0.0.1");
  node->declare_parameter("tcp_port", 5555);
  node->declare_parameter("transport_rate_hz", 1000.0);
  node->declare_parameter("estop_counter_path", "/var/lib/powersuit/estop_counter");
  node->declare_parameter("xrce_symlink_dir", "/tmp/powersuit/xrce");

  const std::string transport_kind = node->get_parameter("transport").as_string();
  std::unique_ptr<suit_canspi_bridge::SpiTransport> transport;
  if (transport_kind == "mock") {
    const std::string host = node->get_parameter("tcp_host").as_string();
    const int port = node->get_parameter("tcp_port").as_int();
    auto mock = std::make_unique<suit_canspi_bridge::MockTransport>(
        host, static_cast<uint16_t>(port));
    if (!mock->open()) {
      RCLCPP_FATAL(logger, "mock transport: failed to connect to %s:%d", host.c_str(), port);
      rclcpp::shutdown();
      return 1;
    }
    transport = std::move(mock);
  } else {
    suit_canspi_bridge::SpidevConfig cfg;
    cfg.device = node->get_parameter("device").as_string();
    cfg.speed_hz = static_cast<uint32_t>(node->get_parameter("spi_speed_hz").as_int());
    cfg.gpiochip = node->get_parameter("gpiochip").as_string();
    cfg.gpio_line = static_cast<unsigned int>(node->get_parameter("gpio_line").as_int());
    auto spidev = std::make_unique<suit_canspi_bridge::SpidevTransport>(cfg);
    if (!spidev->open()) {
      RCLCPP_FATAL(logger, "spidev transport: failed to open %s", cfg.device.c_str());
      rclcpp::shutdown();
      return 1;
    }
    if (!spidev->using_gpio_data_ready()) {
      RCLCPP_WARN(logger, "spidev transport: DATA_READY GPIO not active — falling back to "
                          "1 kHz polling (set gpiochip/gpio_line params, and build with "
                          "libgpiod-dev present, to enable it)");
    }
    transport = std::move(spidev);
  }

  suit_canspi_bridge::XrceDemux xrce(node->get_parameter("xrce_symlink_dir").as_string());
  if (!xrce.open_all()) {
    RCLCPP_ERROR(logger, "xrce_demux: failed to open one or more PTYs — micro-ROS agent "
                         "connectivity to at least one edge node will be unavailable");
  }

  suit_canspi_bridge::FrameParser frame_parser;
  suit_canspi_bridge::TelemSynth telem_synth(node.get());
  suit_canspi_bridge::AudioRelay audio_relay(node.get());
  suit_canspi_bridge::HeartbeatEmitter heartbeat;

  OutboundQueue outbound;
  std::atomic<bool> cloud_up{false};
  std::atomic<bool> external_estop_latched{false};  // ESTOP seen inbound (e.g. from hub/BMS)

  auto emit_records = [&outbound](const std::vector<ps_can_record_t>& recs) {
    outbound.push(recs);
  };
  suit_canspi_bridge::EstopService estop_service(
      node.get(), node->get_parameter("estop_counter_path").as_string(), emit_records,
      PS_NODE_ORCH);

  // --- Downlink command subscriptions (docs/network-map.md §8) -------------
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  for (const auto& limb : kLimbCmdTargets) {
    subs.push_back(node->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        std::string("/suit/command/") + limb.name, 10,
        [&outbound, node_id = limb.node_id](
            const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg) {
          std::vector<ps_can_record_t> recs;
          const size_t n = std::min<size_t>(2, msg->positions.size());
          for (size_t j = 0; j < n; ++j) {
            ps_joint_cmd_t cmd{};
            cmd.joint = static_cast<uint8_t>(j);
            cmd.mode = PS_JMODE_POSITION;
            cmd.pos_crad = static_cast<int16_t>(msg->positions[j] * 100.0);
            cmd.vel_crad_s = static_cast<int16_t>(
                (j < msg->velocities.size() ? msg->velocities[j] : 0.0) * 100.0);
            cmd.eff_cNm = static_cast<int16_t>(
                (j < msg->effort.size() ? msg->effort[j] : 0.0) * 100.0);

            ps_can_record_t rec{};
            rec.id = ps_can_id_pack(PS_CLS_CONTROL, PS_NODE_ORCH, node_id, PS_T_JOINT_CMD, 0);
            rec.bus = (node_id == PS_NODE_LEG_R || node_id == PS_NODE_LEG_L)
                          ? PS_SPI_BUS_CAN2
                          : PS_SPI_BUS_CAN1;
            rec.dlc = sizeof(cmd);
            rec.ts_ms = 0;
            std::memset(rec.data, 0, sizeof(rec.data));
            PS_WIRE_WRITE(rec.data, cmd);
            recs.push_back(rec);
          }
          outbound.push(recs);
        }));
  }

  auto flaps_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/suit/flaps/cmd", 10,
      [&outbound](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        std::vector<ps_can_record_t> recs;
        const size_t n = std::min<size_t>(12, msg->data.size());
        for (size_t i = 0; i < n; ++i) {
          ps_flap_cmd_t cmd{};
          cmd.flap = static_cast<uint8_t>(i);
          cmd.rate_lim = 0;  // board default
          cmd.pos_pm = static_cast<int16_t>(msg->data[i]);
          cmd.flags = 0;
          cmd.rsvd = 0;

          ps_can_record_t rec{};
          rec.id = ps_can_id_pack(PS_CLS_CONTROL, PS_NODE_ORCH, PS_NODE_FLIGHT, PS_T_FLAP_CMD, 0);
          rec.bus = PS_SPI_BUS_CAN2;  // flight actuation is on CAN 2
          rec.dlc = sizeof(cmd);
          rec.ts_ms = 0;
          std::memset(rec.data, 0, sizeof(rec.data));
          PS_WIRE_WRITE(rec.data, cmd);
          recs.push_back(rec);
        }
        outbound.push(recs);
      });

  auto link_status_sub = node->create_subscription<suit_msgs::msg::LinkStatus>(
      "/suit/link/status", 10, [&cloud_up](const suit_msgs::msg::LinkStatus::SharedPtr msg) {
        cloud_up.store(msg->connected);
      });

  // --- Transport thread (1 kHz nominal) -------------------------------------
  std::atomic<bool> keep_running{true};
  std::thread transport_thread([&]() {
    try_set_realtime(logger);

    const double rate_hz = node->get_parameter("transport_rate_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);

    auto last_good_frame = std::chrono::steady_clock::now() - 1s;  // start "stale"
    auto next_tick = std::chrono::steady_clock::now();
    const rclcpp::Time node_start = node->get_clock()->now();

    std::vector<uint8_t> tx(PS_SPI_XFER_SIZE, 0);
    std::vector<uint8_t> rx(PS_SPI_XFER_SIZE, 0);

    while (keep_running.load() && rclcpp::ok()) {
      const auto now = std::chrono::steady_clock::now();

      // 1) Gather everything queued for this cycle.
      std::vector<ps_can_record_t> batch =
          xrce.poll_outbound(std::chrono::milliseconds(0));
      for (auto& r : audio_relay.drain_outbound()) {
        batch.push_back(r);
      }
      for (auto& r : outbound.drain(PS_SPI_MAX_RECORDS)) {
        batch.push_back(r);
      }

      const bool transport_healthy =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - last_good_frame).count() <
          static_cast<int64_t>(PS_HEARTBEAT_TIMEOUT_MS) * 2;  // 100ms per the task contract
      const bool estop_latched = estop_service.is_latched() || external_estop_latched.load();

      const uint8_t src_state = estop_latched ? PS_STATE_ESTOP
                                 : transport_healthy ? PS_STATE_OPERATIONAL
                                                      : PS_STATE_PASSIVE;
      const auto uptime_ms = static_cast<uint32_t>(
          (node->get_clock()->now() - node_start).nanoseconds() / 1000000);

      suit_canspi_bridge::HeartbeatGateInputs gate{transport_healthy, estop_latched,
                                                    rclcpp::ok()};
      auto hb = heartbeat.tick(now, gate, cloud_up.load(), !transport_healthy, src_state,
                                uptime_ms);
      if (hb.has_value()) {
        batch.push_back(*hb);
      }

      telem_synth.set_safety_state(src_state, estop_latched, hb.has_value(),
                                    estop_latched ? estop_service.latched_cause() : 0,
                                    estop_service.is_latched() ? "operator" : "bridge");

      // 2) Cap to PS_SPI_MAX_RECORDS; requeue overflow for next cycle.
      uint8_t flags = 0;
      if (batch.size() > PS_SPI_MAX_RECORDS) {
        std::vector<ps_can_record_t> overflow(batch.begin() + PS_SPI_MAX_RECORDS, batch.end());
        batch.resize(PS_SPI_MAX_RECORDS);
        outbound.push(overflow);
        flags |= PS_SPIF_MORE_PENDING;
      }
      if (estop_latched) {
        flags |= PS_SPIF_ESTOP_LATCHED;
      }

      std::fill(tx.begin(), tx.end(), 0);
      frame_parser.build(tx.data(), flags, batch);

      // 3) Exchange with the hub.
      const bool ok = transport->xfer(tx.data(), rx.data(), rx.size());
      if (ok) {
        std::vector<ps_can_record_t> in_records;
        uint8_t in_flags = 0;
        uint8_t in_seq = 0;
        const auto result =
            frame_parser.parse(rx.data(), rx.size(), in_records, in_flags, in_seq);
        if (result != suit_canspi_bridge::ParseResult::kError) {
          last_good_frame = now;  // idle frames still prove the link is alive
        }
        if (result == suit_canspi_bridge::ParseResult::kOk) {
          for (const auto& rec : in_records) {
            ps_can_id_t parts{};
            ps_can_id_unpack(rec.id, &parts);
            switch (parts.cls) {
              case PS_CLS_TELEM:
                telem_synth.handle(rec);
                break;
              case PS_CLS_XRCE:
                xrce.write_inbound(parts.src, rec.data, rec.dlc);
                break;
              case PS_CLS_AUDIO:
                audio_relay.handle_uplink_record(rec);
                break;
              case PS_CLS_SAFETY:
                if (parts.type == PS_T_ESTOP) {
                  external_estop_latched.store(true);
                } else if (parts.type == PS_T_CLEAR_ESTOP) {
                  external_estop_latched.store(false);
                } else if (parts.type == PS_T_NODE_FAULT) {
                  telem_synth.handle(rec);
                }
                break;
              case PS_CLS_MGMT:
                telem_synth.handle(rec);
                break;
              default:
                break;
            }
          }
        }
      }

      // 4) Pace to the configured rate (DATA_READY-aware on real hardware).
      next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      const auto sleep_for = next_tick - std::chrono::steady_clock::now();
      if (sleep_for > std::chrono::steady_clock::duration::zero()) {
        transport->wait_data_ready(
            std::chrono::duration_cast<std::chrono::milliseconds>(sleep_for));
      } else {
        next_tick = std::chrono::steady_clock::now();
      }
    }
  });

  rclcpp::spin(node);

  keep_running.store(false);
  transport_thread.join();
  xrce.close_all();
  rclcpp::shutdown();
  return 0;
}
