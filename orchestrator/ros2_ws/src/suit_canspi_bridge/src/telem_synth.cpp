#include "suit_canspi_bridge/telem_synth.hpp"

#include <cmath>
#include <cstring>

extern "C" {
#include "powersuit_proto/can_id.h"
#include "powersuit_proto/wire.h"
}

namespace suit_canspi_bridge {

namespace {
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kGravity = 9.80665;

std::string node_name(uint8_t id) {
  switch (id) {
    case PS_NODE_ARM_R: return "node_arm_right";
    case PS_NODE_ARM_L: return "node_arm_left";
    case PS_NODE_LEG_R: return "node_leg_right";
    case PS_NODE_LEG_L: return "node_leg_left";
    case PS_NODE_FLIGHT: return "node_flight_actuation";
    case PS_NODE_HELMET: return "node_helmet_interface";
    case PS_NODE_HUB: return "node_chest_power_hub";
    case PS_NODE_ORCH: return "node_central_orchestrator";
    default: return "node_" + std::to_string(static_cast<int>(id));
  }
}
}  // namespace

int TelemSynth::limb_index(uint8_t node_id) {
  switch (node_id) {
    case PS_NODE_ARM_R: return 0;
    case PS_NODE_ARM_L: return 1;
    case PS_NODE_LEG_R: return 2;
    case PS_NODE_LEG_L: return 3;
    default: return -1;
  }
}

const char* TelemSynth::limb_name(int limb_idx) {
  static const char* names[4] = {"arm_right", "arm_left", "leg_right", "leg_left"};
  return (limb_idx >= 0 && limb_idx < 4) ? names[limb_idx] : "unknown";
}

const char* TelemSynth::joint_name(int limb_idx, int joint_idx) {
  // arms: 0=elbow,1=wrist ; legs: 0=hip,1=knee (docs/network-map.md §1)
  static const char* arm_joints[2] = {"elbow", "wrist"};
  static const char* leg_joints[2] = {"hip", "knee"};
  if (joint_idx < 0 || joint_idx > 1) {
    return "unknown";
  }
  const bool is_leg = (limb_idx == 2 || limb_idx == 3);
  static thread_local std::string buf;
  buf = std::string(limb_name(limb_idx)) + "_" + (is_leg ? leg_joints[joint_idx] : arm_joints[joint_idx]);
  return buf.c_str();
}

TelemSynth::TelemSynth(rclcpp::Node* node) : node_(node) {
  node_->declare_parameter("imu_orientation_stddev", 0.02);
  node_->declare_parameter("imu_angular_velocity_stddev", 0.01);
  node_->declare_parameter("imu_linear_acceleration_stddev", 0.10);

  for (int i = 0; i < 4; ++i) {
    const std::string limb = limb_name(i);
    joint_pubs_[i] = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/suit/telemetry/" + limb, 20);
    limb_imu_pubs_[i] = node_->create_publisher<sensor_msgs::msg::Imu>(
        "/suit/telemetry/" + limb + "/imu", 20);
    force_pubs_[i] = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/suit/force/" + limb, 20);
  }
  torso_imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/suit/imu/torso", 20);
  aero_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/suit/aero/state", 10);
  bms_pub_ = node_->create_publisher<suit_msgs::msg::BmsStatus>("/suit/power/bms", 10);
  diag_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  safety_pub_ = node_->create_publisher<suit_msgs::msg::SafetyState>("/suit/safety/state", 10);

  safety_msg_.state = suit_msgs::msg::SafetyState::STATE_BOOT;
  safety_msg_.source = "bridge";

  // 20 Hz per the task contract, independent of TELEM arrival cadence.
  safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this]() {
    std::lock_guard<std::mutex> lock(mu_);
    safety_msg_.header.stamp = node_->get_clock()->now();
    safety_pub_->publish(safety_msg_);
  });
}

void TelemSynth::set_safety_state(uint8_t state, bool estop_latched, bool heartbeat_ok,
                                   uint8_t estop_cause, const std::string& source) {
  std::lock_guard<std::mutex> lock(mu_);
  safety_msg_.state = state;
  safety_msg_.estop_latched = estop_latched;
  safety_msg_.heartbeat_ok = heartbeat_ok;
  safety_msg_.estop_cause = estop_cause;
  safety_msg_.source = source;
}

void TelemSynth::handle(const ps_can_record_t& rec) {
  ps_can_id_t parts{};
  ps_can_id_unpack(rec.id, &parts);

  // Most of the diagnostics-worthy record types are TELEM, but NODE_FAULT
  // is a SAFETY-class record and VERSION is MGMT-class (docs/network-map.md
  // §3.1/§3.6) — so the class check has to be per-branch, not a blanket
  // TELEM filter up front.
  if (parts.cls == PS_CLS_TELEM) {
    switch (parts.type) {
      case PS_T_JOINT_STATE: on_joint_state(parts.src, rec.data); return;
      case PS_T_IMU_QUAT: on_imu_quat(parts.src, rec.data); return;
      case PS_T_IMU_ACC: on_imu_acc(parts.src, rec.data); return;
      case PS_T_IMU_GYR: on_imu_gyr(parts.src, rec.data); return;
      case PS_T_FORCE: on_force(parts.src, rec.data); return;
      case PS_T_BMS_SUMMARY: on_bms_summary(rec.data); return;
      case PS_T_BMS_CELLS: on_bms_cells(rec.data); return;
      case PS_T_AERO_STATE: on_aero_state(rec.data); return;
      case PS_T_FLAP_STATE: on_flap_state(rec.data); return;
      case PS_T_NODE_STATS: on_node_stats(parts.src, rec.data); return;
      default: return;
    }
  }
  if (parts.cls == PS_CLS_SAFETY && parts.type == PS_T_NODE_FAULT) {
    on_node_fault(parts.src, rec.data);
    return;
  }
  if (parts.cls == PS_CLS_MGMT && parts.type == PS_T_VERSION) {
    on_version(parts.src, rec.data);
    return;
  }
}

void TelemSynth::on_joint_state(uint8_t src, const uint8_t* data) {
  const int limb = limb_index(src);
  if (limb < 0) {
    return;
  }
  ps_joint_state_t js{};
  PS_WIRE_READ(js, data);
  if (js.joint > 1) {
    return;
  }
  std::lock_guard<std::mutex> lock(mu_);
  JointAccum& acc = joints_[limb];
  acc.pos[js.joint] = js.pos_crad * 0.01;
  acc.vel[js.joint] = js.vel_crad_s * 0.01;
  acc.eff[js.joint] = js.eff_cNm * 0.01;
  acc.any = true;
  publish_joint(limb);
}

void TelemSynth::publish_joint(int limb_idx) {
  const JointAccum& acc = joints_[limb_idx];
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.header.frame_id = limb_name(limb_idx);
  msg.name = {joint_name(limb_idx, 0), joint_name(limb_idx, 1)};
  msg.position = {acc.pos[0], acc.pos[1]};
  msg.velocity = {acc.vel[0], acc.vel[1]};
  msg.effort = {acc.eff[0], acc.eff[1]};
  joint_pubs_[limb_idx]->publish(msg);
}

void TelemSynth::publish_imu(int limb_idx_or_torso, ImuAccum& accum) {
  const double o_sd = node_->get_parameter("imu_orientation_stddev").as_double();
  const double w_sd = node_->get_parameter("imu_angular_velocity_stddev").as_double();
  const double a_sd = node_->get_parameter("imu_linear_acceleration_stddev").as_double();
  accum.msg.orientation_covariance = {o_sd * o_sd, 0, 0, 0, o_sd * o_sd, 0, 0, 0, o_sd * o_sd};
  accum.msg.angular_velocity_covariance = {w_sd * w_sd, 0, 0, 0, w_sd * w_sd, 0, 0, 0, w_sd * w_sd};
  accum.msg.linear_acceleration_covariance = {a_sd * a_sd, 0, 0, 0, a_sd * a_sd, 0, 0, 0, a_sd * a_sd};
  accum.msg.header.stamp = node_->get_clock()->now();

  if (limb_idx_or_torso == -1) {
    accum.msg.header.frame_id = "imu_torso";
    torso_imu_pub_->publish(accum.msg);
  } else {
    accum.msg.header.frame_id = std::string(limb_name(limb_idx_or_torso)) + "_imu";
    limb_imu_pubs_[limb_idx_or_torso]->publish(accum.msg);
  }
}

void TelemSynth::on_imu_quat(uint8_t src, const uint8_t* data) {
  ps_imu_quat_t q{};
  PS_WIRE_READ(q, data);
  constexpr double kQ15 = 1.0 / 32767.0;

  std::lock_guard<std::mutex> lock(mu_);
  const int limb = limb_index(src);
  ImuAccum* target = nullptr;
  int pub_idx = -2;
  if (limb >= 0) {
    target = &limb_imu_[limb];
    pub_idx = limb;
  } else if (src == PS_NODE_FLIGHT) {
    target = &torso_imu_;
    pub_idx = -1;
  } else {
    return;
  }
  target->msg.orientation.w = q.qw * kQ15;
  target->msg.orientation.x = q.qx * kQ15;
  target->msg.orientation.y = q.qy * kQ15;
  target->msg.orientation.z = q.qz * kQ15;
  target->any = true;
  publish_imu(pub_idx, *target);
}

void TelemSynth::on_imu_acc(uint8_t src, const uint8_t* data) {
  ps_imu_acc_t a{};
  PS_WIRE_READ(a, data);
  constexpr double kMgToMs2 = kGravity / 1000.0;

  std::lock_guard<std::mutex> lock(mu_);
  const int limb = limb_index(src);
  ImuAccum* target = nullptr;
  int pub_idx = -2;
  if (limb >= 0) {
    target = &limb_imu_[limb];
    pub_idx = limb;
  } else if (src == PS_NODE_FLIGHT) {
    target = &torso_imu_;
    pub_idx = -1;
  } else {
    return;
  }
  target->msg.linear_acceleration.x = a.ax * kMgToMs2;
  target->msg.linear_acceleration.y = a.ay * kMgToMs2;
  target->msg.linear_acceleration.z = a.az * kMgToMs2;
  target->any = true;
  publish_imu(pub_idx, *target);
}

void TelemSynth::on_imu_gyr(uint8_t src, const uint8_t* data) {
  ps_imu_gyr_t g{};
  PS_WIRE_READ(g, data);
  constexpr double kDdpsToRadS = 0.1 * kDegToRad;

  std::lock_guard<std::mutex> lock(mu_);
  const int limb = limb_index(src);
  ImuAccum* target = nullptr;
  int pub_idx = -2;
  if (limb >= 0) {
    target = &limb_imu_[limb];
    pub_idx = limb;
  } else if (src == PS_NODE_FLIGHT) {
    target = &torso_imu_;
    pub_idx = -1;
  } else {
    return;
  }
  target->msg.angular_velocity.x = g.gx * kDdpsToRadS;
  target->msg.angular_velocity.y = g.gy * kDdpsToRadS;
  target->msg.angular_velocity.z = g.gz * kDdpsToRadS;
  target->any = true;
  publish_imu(pub_idx, *target);
}

void TelemSynth::on_force(uint8_t src, const uint8_t* data) {
  const int limb = limb_index(src);
  if (limb < 0) {
    return;
  }
  ps_force_t f{};
  PS_WIRE_READ(f, data);
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {f.ch[0] * 0.01f, f.ch[1] * 0.01f, f.ch[2] * 0.01f, f.ch[3] * 0.01f};
  force_pubs_[limb]->publish(msg);
}

void TelemSynth::on_bms_summary(const uint8_t* data) {
  ps_bms_summary_t s{};
  PS_WIRE_READ(s, data);
  std::lock_guard<std::mutex> lock(mu_);
  bms_msg_.pack_v = s.pack_cV * 0.01f;
  bms_msg_.current_a = s.current_cA * 0.01f;
  bms_msg_.soc_pct = s.soc_pct;
  bms_msg_.temp_max_c = s.temp_max_C;
  bms_msg_.fault_bits = s.fault_bits;
  bms_any_ = true;
  publish_bms_locked();
}

void TelemSynth::on_bms_cells(const uint8_t* data) {
  ps_bms_cells_t c{};
  PS_WIRE_READ(c, data);
  std::lock_guard<std::mutex> lock(mu_);
  const size_t base = static_cast<size_t>(c.group) * 3;
  if (bms_msg_.cell_v.size() < base + 3) {
    bms_msg_.cell_v.resize(base + 3, 0.0f);
  }
  for (int i = 0; i < 3; ++i) {
    bms_msg_.cell_v[base + static_cast<size_t>(i)] = c.mv[i] / 1000.0f;
  }
  bms_any_ = true;
  publish_bms_locked();
}

void TelemSynth::publish_bms_locked() {
  if (!bms_any_) {
    return;
  }
  bms_msg_.header.stamp = node_->get_clock()->now();
  bms_pub_->publish(bms_msg_);
}

void TelemSynth::on_aero_state(const uint8_t* data) {
  ps_aero_state_t a{};
  PS_WIRE_READ(a, data);
  std::lock_guard<std::mutex> lock(mu_);
  aero_state_[0] = a.ias_cms * 0.01f;   // cm/s -> m/s
  aero_state_[1] = static_cast<float>(a.q_pa);  // Pa, already SI
  aero_state_[2] = a.aoa_cdeg * 0.01f;  // 0.01deg -> deg
  aero_any_ = true;
  publish_aero_locked();
}

void TelemSynth::on_flap_state(const uint8_t* data) {
  ps_flap_state_t f{};
  PS_WIRE_READ(f, data);
  if (f.flap > 11) {
    return;
  }
  std::lock_guard<std::mutex> lock(mu_);
  aero_state_[3 + f.flap] = static_cast<float>(f.pos_pm);  // permille
  aero_any_ = true;
  publish_aero_locked();
}

void TelemSynth::publish_aero_locked() {
  if (!aero_any_) {
    return;
  }
  std_msgs::msg::Float32MultiArray msg;
  msg.data.assign(aero_state_.begin(), aero_state_.end());
  aero_pub_->publish(msg);
}

void TelemSynth::on_node_stats(uint8_t src, const uint8_t* data) {
  ps_node_stats_t s{};
  PS_WIRE_READ(s, data);
  publish_diag(node_name(src), diagnostic_msgs::msg::DiagnosticStatus::OK, "node_stats",
               {{"cpu_pct", std::to_string(s.cpu_pct)},
                {"state", std::to_string(s.state)},
                {"rx_fps", std::to_string(s.rx_fps)},
                {"tx_fps", std::to_string(s.tx_fps)},
                {"err_cnt", std::to_string(s.err_cnt)}});
}

void TelemSynth::on_node_fault(uint8_t src, const uint8_t* data) {
  ps_node_fault_t f{};
  PS_WIRE_READ(f, data);
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  if (f.severity == 1) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  } else if (f.severity >= 2) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  publish_diag(node_name(src), level, "node_fault",
               {{"fault_code", std::to_string(f.fault_code)},
                {"detail", std::to_string(f.detail)},
                {"uptime_ms", std::to_string(f.uptime_ms)}});
}

void TelemSynth::on_version(uint8_t src, const uint8_t* data) {
  ps_version_t v{};
  PS_WIRE_READ(v, data);
  const std::string version = std::to_string(v.major) + "." + std::to_string(v.minor) + "." +
                               std::to_string(v.patch);
  publish_diag(node_name(src), diagnostic_msgs::msg::DiagnosticStatus::OK, "version",
               {{"version", version},
                {"node_state", std::to_string(v.node_state)},
                {"git_short", std::to_string(v.git_short)}});
}

void TelemSynth::publish_diag(const std::string& name, uint8_t level, const std::string& message,
                               const std::vector<std::pair<std::string, std::string>>& kv) {
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = node_->get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = name;
  st.hardware_id = name;
  st.level = level;
  st.message = message;
  for (const auto& [k, v] : kv) {
    diagnostic_msgs::msg::KeyValue e;
    e.key = k;
    e.value = v;
    st.values.push_back(e);
  }
  arr.status.push_back(st);
  diag_pub_->publish(arr);
}

}  // namespace suit_canspi_bridge
