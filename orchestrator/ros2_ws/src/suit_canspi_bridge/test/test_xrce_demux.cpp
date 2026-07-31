// Six micro-ROS clients share one CAN plane and one SPI link. The demux is
// what keeps their byte streams apart; if it ever interleaves two clients,
// both XRCE sessions corrupt silently and the failure looks like flaky ROS
// rather than a bridge bug. Hence the cross-talk tests.
//
// Uses socketpair() through the bind_fd_for_test seam so nothing here depends
// on real ptys or on /tmp symlinks.
#include <gtest/gtest.h>

#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <map>
#include <vector>

#include "suit_canspi_bridge/xrce_demux.hpp"

extern "C" {
#include "powersuit_proto/can_id.h"
}

using suit_canspi_bridge::kEdgeNodes;
using suit_canspi_bridge::XrceDemux;
using std::chrono::milliseconds;

namespace {

// Binds a socketpair to `node_id`; returns the far end the test reads/writes.
int attach_pair(XrceDemux& demux, uint8_t node_id) {
  int fds[2];
  EXPECT_EQ(0, socketpair(AF_UNIX, SOCK_STREAM, 0, fds));
  demux.bind_fd_for_test(node_id, fds[0]);
  return fds[1];
}

std::vector<uint8_t> pattern(uint8_t seed, size_t n) {
  std::vector<uint8_t> out(n);
  for (size_t i = 0; i < n; ++i) {
    out[i] = static_cast<uint8_t>(seed + i);
  }
  return out;
}

}  // namespace

TEST(XrceDemux, InboundBytesReachOnlyTheAddressedNode) {
  XrceDemux demux;
  const int arm = attach_pair(demux, PS_NODE_ARM_R);
  const int leg = attach_pair(demux, PS_NODE_LEG_L);

  const auto payload = pattern(0x10, 32);
  ASSERT_TRUE(demux.write_inbound(PS_NODE_ARM_R, payload.data(), payload.size()));

  uint8_t buf[64] = {};
  const ssize_t got = ::read(arm, buf, sizeof(buf));
  ASSERT_EQ(got, static_cast<ssize_t>(payload.size()));
  EXPECT_EQ(0, std::memcmp(buf, payload.data(), payload.size()));

  // Nothing should have landed on the other node's stream.
  int flags_probe = 0;
  ASSERT_EQ(0, ::shutdown(leg, SHUT_WR));
  (void)flags_probe;

  ::close(arm);
  ::close(leg);
}

TEST(XrceDemux, RejectsUnknownNodes) {
  XrceDemux demux;
  const auto payload = pattern(0x01, 8);
  EXPECT_FALSE(demux.write_inbound(PS_NODE_HUB, payload.data(), payload.size()))
      << "the hub runs no micro-ROS client (network-map §12.4)";
  EXPECT_FALSE(demux.write_inbound(99, payload.data(), payload.size()));
}

// "Outbound" is agent -> node: the agent writes to the pty, the bridge reads
// the master fd and slices the bytes into records addressed DOWN to that node
// (docs/network-map.md §3.4: "Direction down: dst = <node>").
TEST(XrceDemux, OutboundIsSlicedIntoEightByteRecords) {
  XrceDemux demux;
  const int arm = attach_pair(demux, PS_NODE_ARM_R);

  const auto blob = pattern(0xA0, 30);
  ASSERT_EQ(static_cast<ssize_t>(blob.size()), ::write(arm, blob.data(), blob.size()));

  std::vector<uint8_t> reassembled;
  for (int attempt = 0; attempt < 20 && reassembled.size() < blob.size(); ++attempt) {
    for (const auto& rec : demux.poll_outbound(milliseconds(20))) {
      ps_can_id_t id{};
      ps_can_id_unpack(rec.id, &id);
      EXPECT_EQ(id.cls, PS_CLS_XRCE);
      EXPECT_EQ(id.type, PS_T_XRCE_STREAM);
      EXPECT_EQ(id.src, PS_NODE_ORCH);
      EXPECT_EQ(id.dst, PS_NODE_ARM_R);
      EXPECT_LE(rec.dlc, 8) << "classic CAN carries at most 8 bytes";
      reassembled.insert(reassembled.end(), rec.data, rec.data + rec.dlc);
    }
  }
  ASSERT_EQ(reassembled.size(), blob.size());
  EXPECT_EQ(reassembled, blob);

  ::close(arm);
}

TEST(XrceDemux, InterleavedStreamsDoNotCrossTalk) {
  XrceDemux demux;
  std::map<uint8_t, int> ends{
      {PS_NODE_ARM_R, attach_pair(demux, PS_NODE_ARM_R)},
      {PS_NODE_LEG_L, attach_pair(demux, PS_NODE_LEG_L)},
      {PS_NODE_HELMET, attach_pair(demux, PS_NODE_HELMET)},
  };
  std::map<uint8_t, std::vector<uint8_t>> sent{
      {PS_NODE_ARM_R, pattern(0x00, 40)},
      {PS_NODE_LEG_L, pattern(0x50, 40)},
      {PS_NODE_HELMET, pattern(0xA0, 40)},
  };

  // Interleave at 8-byte granularity — the worst case for a demux that
  // concatenates by arrival order rather than by source.
  for (size_t off = 0; off < 40; off += 8) {
    for (auto& [node, blob] : sent) {
      ASSERT_EQ(8, ::write(ends[node], blob.data() + off, 8));
    }
  }

  std::map<uint8_t, std::vector<uint8_t>> got;
  for (int attempt = 0; attempt < 40; ++attempt) {
    for (const auto& rec : demux.poll_outbound(milliseconds(20))) {
      ps_can_id_t id{};
      ps_can_id_unpack(rec.id, &id);
      auto& dst = got[id.dst];
      dst.insert(dst.end(), rec.data, rec.data + rec.dlc);
    }
    const bool done = std::all_of(sent.begin(), sent.end(), [&](const auto& kv) {
      return got[kv.first].size() >= kv.second.size();
    });
    if (done) {
      break;
    }
  }

  for (const auto& [node, blob] : sent) {
    ASSERT_EQ(got[node].size(), blob.size()) << "node " << static_cast<int>(node);
    EXPECT_EQ(got[node], blob) << "stream for node " << static_cast<int>(node)
                               << " was corrupted by another node's traffic";
  }

  for (auto& [node, fd] : ends) {
    (void)node;
    ::close(fd);
  }
}

TEST(XrceDemux, NodeRegistryMatchesTheContract) {
  // Bus membership per docs/network-map.md §1: arms and helmet on bus 1,
  // legs and flight on bus 2.
  std::map<uint8_t, uint8_t> expected{
      {PS_NODE_ARM_R, PS_SPI_BUS_CAN1}, {PS_NODE_ARM_L, PS_SPI_BUS_CAN1},
      {PS_NODE_HELMET, PS_SPI_BUS_CAN1}, {PS_NODE_LEG_R, PS_SPI_BUS_CAN2},
      {PS_NODE_LEG_L, PS_SPI_BUS_CAN2}, {PS_NODE_FLIGHT, PS_SPI_BUS_CAN2},
  };
  ASSERT_EQ(kEdgeNodes.size(), expected.size());
  for (const auto& node : kEdgeNodes) {
    ASSERT_TRUE(expected.count(node.id)) << "unexpected node " << static_cast<int>(node.id);
    EXPECT_EQ(node.bus, expected[node.id]) << "node " << static_cast<int>(node.id)
                                           << " is on the wrong bus";
  }
}

TEST(XrceDemux, EmptyPollReturnsNothing) {
  XrceDemux demux;
  const int arm = attach_pair(demux, PS_NODE_ARM_R);
  EXPECT_TRUE(demux.poll_outbound(milliseconds(5)).empty());
  ::close(arm);
}
