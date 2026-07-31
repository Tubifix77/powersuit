#include "suit_canspi_bridge/mock_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <thread>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

namespace suit_canspi_bridge {

MockTransport::MockTransport(std::string host, uint16_t port)
    : host_(std::move(host)), port_(port) {}

MockTransport::~MockTransport() {
  if (sock_ >= 0) {
    ::close(sock_);
  }
}

bool MockTransport::open() {
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  addrinfo* res = nullptr;
  const std::string port_str = std::to_string(port_);
  if (::getaddrinfo(host_.c_str(), port_str.c_str(), &hints, &res) != 0 || res == nullptr) {
    return false;
  }

  int sock = -1;
  for (addrinfo* rp = res; rp != nullptr; rp = rp->ai_next) {
    sock = ::socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (sock < 0) {
      continue;
    }
    if (::connect(sock, rp->ai_addr, rp->ai_addrlen) == 0) {
      break;
    }
    ::close(sock);
    sock = -1;
  }
  ::freeaddrinfo(res);

  if (sock < 0) {
    return false;
  }

  // The bridge is always the SPI master: it sends the 512-byte master frame
  // first, then blocks for the 512-byte slave frame, every transaction.
  // Disabling Nagle keeps that request/response pair latency-bounded.
  int one = 1;
  ::setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

  sock_ = sock;
  return true;
}

namespace {
bool send_all(int fd, const uint8_t* buf, size_t len) {
  size_t off = 0;
  while (off < len) {
    const ssize_t n = ::send(fd, buf + off, len - off, 0);
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

bool recv_all(int fd, uint8_t* buf, size_t len) {
  size_t off = 0;
  while (off < len) {
    const ssize_t n = ::recv(fd, buf + off, len - off, 0);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (n == 0) {
      return false;  // peer closed
    }
    off += static_cast<size_t>(n);
  }
  return true;
}
}  // namespace

bool MockTransport::xfer(const uint8_t* tx, uint8_t* rx, size_t len) {
  if (sock_ < 0) {
    return false;
  }
  // Exact contract: send exactly `len` bytes, then receive exactly `len`
  // bytes. Never interleave, never short-cut — the mock server on the other
  // end depends on this framing.
  if (!send_all(sock_, tx, len)) {
    return false;
  }
  return recv_all(sock_, rx, len);
}

bool MockTransport::wait_data_ready(std::chrono::milliseconds timeout) {
  // No hardware DATA_READY line over TCP: the xfer() call itself blocks on
  // the round trip, so this just paces callers that check readiness first.
  std::this_thread::sleep_for(std::min(timeout, std::chrono::milliseconds(1)));
  return true;
}

}  // namespace suit_canspi_bridge
