// XRCE plane demux (docs/network-map.md §7, §3.4).
//
// For each of the 6 edge nodes we open a pty pair, put the slave side in raw
// mode, and symlink /tmp/powersuit/xrce/<node_name> -> the slave path so a
// single `micro-ros-agent multiserial` process can attach to all six.
//
// Inbound: XRCE STREAM records received from a source node over SPI/CAN are
// written to that node's pty master (the agent reads them off the slave as
// if the node were talking over a serial line).
// Outbound: bytes the agent writes to a slave become readable on the
// matching master; poll_outbound() drains them and slices them into
// CanRecords (class XRCE, type 0x30, payload <= 8 bytes, bus nibble per the
// node's bus) addressed back down to that node.
#ifndef SUIT_CANSPI_BRIDGE_XRCE_DEMUX_HPP_
#define SUIT_CANSPI_BRIDGE_XRCE_DEMUX_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

extern "C" {
#include "powersuit_proto/spi_frame.h"
}

namespace suit_canspi_bridge {

struct EdgeNode {
  uint8_t id;
  const char* name;
  uint8_t bus;  // PS_SPI_BUS_CAN1 / PS_SPI_BUS_CAN2
};

// Node registry per docs/network-map.md §1 (nodes 1..6, excludes the hub and
// orchestrator itself).
extern const std::array<EdgeNode, 6> kEdgeNodes;

class XrceDemux {
public:
  explicit XrceDemux(std::string symlink_dir = "/tmp/powersuit/xrce");
  ~XrceDemux();

  XrceDemux(const XrceDemux&) = delete;
  XrceDemux& operator=(const XrceDemux&) = delete;

  // Opens a pty pair per edge node, sets the slave to raw mode, and
  // (re)creates the symlinks (mkdir -p the directory, replacing stale
  // links). Returns false if any node failed to open.
  bool open_all();
  void close_all();

  // Write inbound bytes (from an uplink XRCE record) to node_id's master fd.
  // Returns false if the node is unknown or the write failed.
  bool write_inbound(uint8_t node_id, const uint8_t* data, size_t len);

  // Poll every open master fd for up to `timeout`; drain whatever is
  // available and slice it into <=8-byte CanRecords addressed to the
  // originating node. May return an empty vector.
  std::vector<ps_can_record_t> poll_outbound(std::chrono::milliseconds timeout);

  int master_fd(uint8_t node_id) const;
  std::string slave_path(uint8_t node_id) const;

  // Test-only seam: bind an already-open fd (e.g. one end of a socketpair
  // or openpty pair) as the master fd for `node_id`, bypassing real pty
  // creation so demux logic can be exercised without /tmp symlinks.
  void bind_fd_for_test(uint8_t node_id, int fd);

private:
  static int index_of(uint8_t node_id);

  std::string symlink_dir_;
  std::array<int, 6> master_fd_{{-1, -1, -1, -1, -1, -1}};
  std::array<int, 6> slave_fd_{{-1, -1, -1, -1, -1, -1}};
};

}  // namespace suit_canspi_bridge

#endif  // SUIT_CANSPI_BRIDGE_XRCE_DEMUX_HPP_
