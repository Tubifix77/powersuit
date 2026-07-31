// Abstract SPI-transaction transport for the Node 8 <-> Node 7 link
// (docs/network-map.md §6). Concrete implementations: spidev_transport.cpp
// (real hardware) and mock_transport.cpp (TCP loop for dev/CI).
#ifndef SUIT_CANSPI_BRIDGE_SPI_TRANSPORT_HPP_
#define SUIT_CANSPI_BRIDGE_SPI_TRANSPORT_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>

namespace suit_canspi_bridge {

class SpiTransport {
public:
  virtual ~SpiTransport() = default;

  // Full-duplex exchange of exactly `len` bytes (512 in practice, see
  // PS_SPI_XFER_SIZE). Returns true on success.
  virtual bool xfer(const uint8_t* tx, uint8_t* rx, size_t len) = 0;

  // Block up to `timeout` waiting for the hub's DATA_READY signal (or, for
  // transports without one, a polling equivalent). Returns true if data is
  // (or should be treated as) ready, false on timeout.
  virtual bool wait_data_ready(std::chrono::milliseconds timeout) = 0;
};

}  // namespace suit_canspi_bridge

#endif  // SUIT_CANSPI_BRIDGE_SPI_TRANSPORT_HPP_
