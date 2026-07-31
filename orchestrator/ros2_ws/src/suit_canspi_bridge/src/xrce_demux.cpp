#include "suit_canspi_bridge/xrce_demux.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <system_error>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#if defined(__has_include)
#if __has_include(<pty.h>)
#include <pty.h>
#define SUIT_CANSPI_BRIDGE_HAVE_PTY_H 1
#elif __has_include(<util.h>)
#include <util.h>
#define SUIT_CANSPI_BRIDGE_HAVE_PTY_H 1
#endif
#endif

extern "C" {
#include "powersuit_proto/can_id.h"
}

namespace suit_canspi_bridge {

const std::array<EdgeNode, 6> kEdgeNodes = {{
    {1, "node_arm_right", PS_SPI_BUS_CAN1},
    {2, "node_arm_left", PS_SPI_BUS_CAN1},
    {3, "node_leg_right", PS_SPI_BUS_CAN2},
    {4, "node_leg_left", PS_SPI_BUS_CAN2},
    {5, "node_flight_actuation", PS_SPI_BUS_CAN2},
    {6, "node_helmet_interface", PS_SPI_BUS_CAN1},
}};

int XrceDemux::index_of(uint8_t node_id) {
  for (size_t i = 0; i < kEdgeNodes.size(); ++i) {
    if (kEdgeNodes[i].id == node_id) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

XrceDemux::XrceDemux(std::string symlink_dir) : symlink_dir_(std::move(symlink_dir)) {}

XrceDemux::~XrceDemux() { close_all(); }

bool XrceDemux::open_all() {
  std::error_code ec;
  std::filesystem::create_directories(symlink_dir_, ec);
  if (ec) {
    return false;
  }

  bool all_ok = true;
#ifdef SUIT_CANSPI_BRIDGE_HAVE_PTY_H
  for (size_t i = 0; i < kEdgeNodes.size(); ++i) {
    int master = -1;
    int slave = -1;
    char slave_name[256] = {0};
    if (openpty(&master, &slave, slave_name, nullptr, nullptr) != 0) {
      all_ok = false;
      continue;
    }

    termios tio{};
    if (tcgetattr(slave, &tio) == 0) {
      cfmakeraw(&tio);
      tcsetattr(slave, TCSANOW, &tio);
    }

    master_fd_[i] = master;
    slave_fd_[i] = slave;

    const std::string link_path = symlink_dir_ + "/" + kEdgeNodes[i].name;
    ::unlink(link_path.c_str());  // replace a stale link from a previous run
    if (::symlink(slave_name, link_path.c_str()) != 0) {
      all_ok = false;
    }
  }
#else
  all_ok = false;  // no pty.h available at build time: nothing to open
#endif
  return all_ok;
}

void XrceDemux::close_all() {
  for (size_t i = 0; i < kEdgeNodes.size(); ++i) {
    if (master_fd_[i] >= 0) {
      ::close(master_fd_[i]);
      master_fd_[i] = -1;
    }
    if (slave_fd_[i] >= 0) {
      ::close(slave_fd_[i]);
      slave_fd_[i] = -1;
    }
  }
}

bool XrceDemux::write_inbound(uint8_t node_id, const uint8_t* data, size_t len) {
  const int idx = index_of(node_id);
  if (idx < 0 || master_fd_[idx] < 0) {
    return false;
  }
  size_t off = 0;
  while (off < len) {
    const ssize_t n = ::write(master_fd_[idx], data + off, len - off);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    off += static_cast<size_t>(n);
  }
  return true;
}

std::vector<ps_can_record_t> XrceDemux::poll_outbound(std::chrono::milliseconds timeout) {
  std::vector<ps_can_record_t> out;

  std::array<pollfd, 6> pfds{};
  std::array<int, 6> idx_for_pfd{};
  int nfds = 0;
  for (size_t i = 0; i < kEdgeNodes.size(); ++i) {
    if (master_fd_[i] < 0) {
      continue;
    }
    pfds[static_cast<size_t>(nfds)].fd = master_fd_[i];
    pfds[static_cast<size_t>(nfds)].events = POLLIN;
    idx_for_pfd[static_cast<size_t>(nfds)] = static_cast<int>(i);
    ++nfds;
  }
  if (nfds == 0) {
    return out;
  }

  const int rc = ::poll(pfds.data(), static_cast<nfds_t>(nfds), static_cast<int>(timeout.count()));
  if (rc <= 0) {
    return out;
  }

  uint8_t buf[256];
  for (int p = 0; p < nfds; ++p) {
    if (!(pfds[static_cast<size_t>(p)].revents & POLLIN)) {
      continue;
    }
    const int idx = idx_for_pfd[static_cast<size_t>(p)];
    const ssize_t n = ::read(master_fd_[idx], buf, sizeof(buf));
    if (n <= 0) {
      continue;
    }
    const EdgeNode& node = kEdgeNodes[static_cast<size_t>(idx)];
    size_t off = 0;
    while (off < static_cast<size_t>(n)) {
      const size_t chunk = std::min<size_t>(8, static_cast<size_t>(n) - off);
      ps_can_record_t rec{};
      rec.id = ps_can_id_pack(PS_CLS_XRCE, PS_NODE_ORCH, node.id, PS_T_XRCE_STREAM, 0);
      rec.bus = node.bus;
      rec.dlc = static_cast<uint8_t>(chunk);
      rec.ts_ms = 0;
      std::memset(rec.data, 0, sizeof(rec.data));
      std::memcpy(rec.data, buf + off, chunk);
      out.push_back(rec);
      off += chunk;
    }
  }
  return out;
}

int XrceDemux::master_fd(uint8_t node_id) const {
  const int idx = index_of(node_id);
  return idx < 0 ? -1 : master_fd_[static_cast<size_t>(idx)];
}

std::string XrceDemux::slave_path(uint8_t node_id) const {
  const int idx = index_of(node_id);
  if (idx < 0) {
    return {};
  }
  return symlink_dir_ + "/" + kEdgeNodes[static_cast<size_t>(idx)].name;
}

void XrceDemux::bind_fd_for_test(uint8_t node_id, int fd) {
  const int idx = index_of(node_id);
  if (idx >= 0) {
    master_fd_[static_cast<size_t>(idx)] = fd;
  }
}

}  // namespace suit_canspi_bridge
