// The heartbeat gate is the suit's dead-man switch (docs/safety.md §2). Its
// truth table is worth testing exhaustively: every false negative here is a
// limb that stays powered when it should have gone limp, and every false
// positive is the bridge lying about liveness.
#include <gtest/gtest.h>

#include <chrono>

#include "suit_canspi_bridge/heartbeat.hpp"

extern "C" {
#include "powersuit_proto/can_id.h"
#include "powersuit_proto/wire.h"
}

using suit_canspi_bridge::HeartbeatEmitter;
using suit_canspi_bridge::HeartbeatGateInputs;
using suit_canspi_bridge::heartbeat_gate_open;
using std::chrono::milliseconds;
using std::chrono::steady_clock;

TEST(HeartbeatGate, OpensOnlyWhenEverythingIsHealthy) {
  // All eight combinations, spelled out: the gate opens in exactly one.
  for (int mask = 0; mask < 8; ++mask) {
    HeartbeatGateInputs in{
        .transport_healthy = (mask & 1) != 0,
        .estop_latched = (mask & 2) != 0,
        .rclcpp_ok = (mask & 4) != 0,
    };
    const bool expected = in.transport_healthy && !in.estop_latched && in.rclcpp_ok;
    EXPECT_EQ(heartbeat_gate_open(in), expected)
        << "transport=" << in.transport_healthy << " estop=" << in.estop_latched
        << " rclcpp=" << in.rclcpp_ok;
  }
}

TEST(HeartbeatGate, EstopAloneShutsTheGate) {
  HeartbeatGateInputs in{true, true, true};
  EXPECT_FALSE(heartbeat_gate_open(in));
}

TEST(HeartbeatGate, StaleTransportShutsTheGate) {
  HeartbeatGateInputs in{false, false, true};
  EXPECT_FALSE(heartbeat_gate_open(in));
}

TEST(HeartbeatEmitterTest, EmitsNothingWhileTheGateIsShut) {
  HeartbeatEmitter emitter;
  auto t = steady_clock::now();
  const HeartbeatGateInputs shut{true, true, true};  // e-stop latched

  for (int i = 0; i < 50; ++i) {
    t += milliseconds(10);
    EXPECT_FALSE(emitter.tick(t, shut, false, false, PS_STATE_ESTOP, 0).has_value())
        << "the bridge must never fake liveness during an e-stop";
  }
}

TEST(HeartbeatEmitterTest, EmitsAtTheContractPeriod) {
  HeartbeatEmitter emitter;
  auto t = steady_clock::now();
  const HeartbeatGateInputs open{true, false, true};

  ASSERT_TRUE(emitter.tick(t, open, false, false, PS_STATE_OPERATIONAL, 0).has_value());

  // Too soon: 100 Hz means one beat per 10 ms, not one per call.
  t += milliseconds(4);
  EXPECT_FALSE(emitter.tick(t, open, false, false, PS_STATE_OPERATIONAL, 0).has_value());

  t += milliseconds(7);  // 11 ms since the last emission
  EXPECT_TRUE(emitter.tick(t, open, false, false, PS_STATE_OPERATIONAL, 0).has_value());
}

TEST(HeartbeatEmitterTest, ProducesAWellFormedBroadcastRecord) {
  HeartbeatEmitter emitter;
  auto t = steady_clock::now();
  const HeartbeatGateInputs open{true, false, true};

  auto rec = emitter.tick(t, open, /*cloud_up=*/true, /*degraded=*/false,
                          PS_STATE_OPERATIONAL, 1234);
  ASSERT_TRUE(rec.has_value());

  ps_can_id_t id{};
  ps_can_id_unpack(rec->id, &id);
  EXPECT_EQ(id.cls, PS_CLS_SAFETY);
  EXPECT_EQ(id.src, PS_NODE_ORCH);
  EXPECT_EQ(id.dst, PS_NODE_BROADCAST) << "heartbeats must reach every node";
  EXPECT_EQ(id.type, PS_T_HEARTBEAT);
  EXPECT_EQ(rec->dlc, sizeof(ps_heartbeat_t));

  ps_heartbeat_t hb{};
  PS_WIRE_READ(hb, rec->data);
  EXPECT_EQ(hb.src_state, PS_STATE_OPERATIONAL);
  EXPECT_TRUE(hb.flags & PS_HB_CLOUD_UP);
  EXPECT_FALSE(hb.flags & PS_HB_ESTOP_LATCHED);
  EXPECT_EQ(hb.uptime_ms, 1234u);
}

TEST(HeartbeatEmitterTest, SequenceAdvancesMonotonically) {
  HeartbeatEmitter emitter;
  auto t = steady_clock::now();
  const HeartbeatGateInputs open{true, false, true};

  uint16_t previous = 0;
  for (int i = 0; i < 5; ++i) {
    auto rec = emitter.tick(t, open, false, false, PS_STATE_OPERATIONAL, 0);
    ASSERT_TRUE(rec.has_value());
    ps_heartbeat_t hb{};
    PS_WIRE_READ(hb, rec->data);
    if (i > 0) {
      EXPECT_EQ(hb.seq, static_cast<uint16_t>(previous + 1));
    }
    previous = hb.seq;
    t += milliseconds(PS_HEARTBEAT_PERIOD_MS);
  }
}

TEST(HeartbeatEmitterTest, GateReopeningResumesEmission) {
  HeartbeatEmitter emitter;
  auto t = steady_clock::now();
  const HeartbeatGateInputs open{true, false, true};
  const HeartbeatGateInputs shut{false, false, true};

  ASSERT_TRUE(emitter.tick(t, open, false, false, PS_STATE_OPERATIONAL, 0).has_value());
  for (int i = 0; i < 10; ++i) {
    t += milliseconds(10);
    ASSERT_FALSE(emitter.tick(t, shut, false, false, PS_STATE_PASSIVE, 0).has_value());
  }
  t += milliseconds(10);
  EXPECT_TRUE(emitter.tick(t, open, false, false, PS_STATE_OPERATIONAL, 0).has_value());
}
