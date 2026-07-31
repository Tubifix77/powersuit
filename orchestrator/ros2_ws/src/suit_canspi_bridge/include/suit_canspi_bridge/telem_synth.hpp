// TELEM (class 2) record -> ROS topic synthesis (docs/network-map.md §3.3,
// §8). One TelemSynth instance owns every bridge-side publisher that isn't
// audio or safety-service related; feed it every parsed record and it
// dispatches on (src, type) and republishes the affected topic.
#ifndef SUIT_CANSPI_BRIDGE_TELEM_SYNTH_HPP_
#define SUIT_CANSPI_BRIDGE_TELEM_SYNTH_HPP_

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <suit_msgs/msg/bms_status.hpp>
#include <suit_msgs/msg/safety_state.hpp>

extern "C" {
#include "powersuit_proto/spi_frame.h"
}

namespace suit_canspi_bridge {

class TelemSynth {
public:
  explicit TelemSynth(rclcpp::Node* node);

  // Dispatch one parsed TELEM-class CanRecord. No-op for other classes.
  void handle(const ps_can_record_t& rec);

  // Updated by the bridge/estop owner; published on /suit/safety/state at
  // 20 Hz by an internal wall timer.
  void set_safety_state(uint8_t state, bool estop_latched, bool heartbeat_ok,
                         uint8_t estop_cause, const std::string& source);

private:
  struct ImuAccum {
    sensor_msgs::msg::Imu msg;
    bool any = false;
  };
  struct JointAccum {
    std::array<double, 2> pos{0.0, 0.0};
    std::array<double, 2> vel{0.0, 0.0};
    std::array<double, 2> eff{0.0, 0.0};
    bool any = false;
  };

  static int limb_index(uint8_t node_id);       // 0..3, or -1
  static const char* limb_name(int limb_idx);
  static const char* joint_name(int limb_idx, int joint_idx);

  void on_joint_state(uint8_t src, const uint8_t* data);
  void on_imu_quat(uint8_t src, const uint8_t* data);
  void on_imu_acc(uint8_t src, const uint8_t* data);
  void on_imu_gyr(uint8_t src, const uint8_t* data);
  void on_force(uint8_t src, const uint8_t* data);
  void on_bms_summary(const uint8_t* data);
  void on_bms_cells(const uint8_t* data);
  void on_aero_state(const uint8_t* data);
  void on_flap_state(const uint8_t* data);
  void on_node_stats(uint8_t src, const uint8_t* data);
  void on_node_fault(uint8_t src, const uint8_t* data);
  void on_version(uint8_t src, const uint8_t* data);

  void publish_imu(int limb_idx_or_torso, ImuAccum& accum);
  void publish_joint(int limb_idx);
  void publish_aero_locked();
  void publish_bms_locked();
  void publish_diag(const std::string& name, uint8_t level,
                     const std::string& message,
                     const std::vector<std::pair<std::string, std::string>>& kv);

  rclcpp::Node* node_;

  std::array<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr, 4> joint_pubs_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr, 4> limb_imu_pubs_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr torso_imu_pub_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr, 4> force_pubs_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr aero_pub_;
  rclcpp::Publisher<suit_msgs::msg::BmsStatus>::SharedPtr bms_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<suit_msgs::msg::SafetyState>::SharedPtr safety_pub_;
  rclcpp::TimerBase::SharedPtr safety_timer_;

  std::mutex mu_;
  std::array<JointAccum, 4> joints_;
  std::array<ImuAccum, 4> limb_imu_;
  ImuAccum torso_imu_;
  std::array<float, 16> aero_state_{};
  bool aero_any_ = false;
  suit_msgs::msg::BmsStatus bms_msg_;
  bool bms_any_ = false;

  suit_msgs::msg::SafetyState safety_msg_;
};

}  // namespace suit_canspi_bridge

#endif  // SUIT_CANSPI_BRIDGE_TELEM_SYNTH_HPP_
