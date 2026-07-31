// TCP-loop mock transport for dev/CI (docs/network-map.md §6).
//
// Exact contract (implemented by a separate workstream on the server side —
// do not deviate): connect to host:port, then loop { send exactly 512 bytes
// (the master frame), then receive exactly 512 bytes (the slave frame) }.
// The bridge is always the SPI master and the TCP client.
#ifndef SUIT_CANSPI_BRIDGE_MOCK_TRANSPORT_HPP_
#define SUIT_CANSPI_BRIDGE_MOCK_TRANSPORT_HPP_

#include <chrono>
#include <cstdint>
#include <string>

#include "suit_canspi_bridge/spi_transport.hpp"

namespace suit_canspi_bridge {

class MockTransport : public SpiTransport {
public:
  MockTransport(std::string host, uint16_t port);
  ~MockTransport() override;

  bool open();
  bool xfer(const uint8_t* tx, uint8_t* rx, size_t len) override;
  // No hardware DATA_READY line exists over TCP: emulate with a short sleep
  // so the caller still gets ~1 kHz cadence.
  bool wait_data_ready(std::chrono::milliseconds timeout) override;

private:
  std::string host_;
  uint16_t port_;
  int sock_ = -1;
};

}  // namespace suit_canspi_bridge

#endif  // SUIT_CANSPI_BRIDGE_MOCK_TRANSPORT_HPP_
