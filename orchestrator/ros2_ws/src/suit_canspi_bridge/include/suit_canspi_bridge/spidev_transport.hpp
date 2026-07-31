// /dev/spidevX.Y transport for the Node 7 <-> Node 8 SPI link
// (docs/network-map.md §6, §12.1: 20 MHz honest qualification point).
// DATA_READY is read via libgpiod v2 when gpiochip/line params are set; if
// unset (or libgpiod headers are unavailable at build time) this falls back
// to plain 1 kHz polling, so the package still builds in a bare container.
#ifndef SUIT_CANSPI_BRIDGE_SPIDEV_TRANSPORT_HPP_
#define SUIT_CANSPI_BRIDGE_SPIDEV_TRANSPORT_HPP_

#include <chrono>
#include <cstdint>
#include <string>

#include "suit_canspi_bridge/spi_transport.hpp"

namespace suit_canspi_bridge {

struct SpidevConfig {
  std::string device = "/dev/spidev0.0";
  uint32_t speed_hz = 20000000;   // docs/network-map.md §12.1
  uint8_t mode = 0;
  // DATA_READY GPIO (libgpiod v2). Leave gpiochip empty to force polling.
  std::string gpiochip;           // e.g. "/dev/gpiochip0"
  unsigned int gpio_line = 0;
};

class SpidevTransport : public SpiTransport {
public:
  explicit SpidevTransport(const SpidevConfig& cfg);
  ~SpidevTransport() override;

  bool open();
  bool xfer(const uint8_t* tx, uint8_t* rx, size_t len) override;
  bool wait_data_ready(std::chrono::milliseconds timeout) override;

  // True when built against libgpiod and a gpiochip/line were configured.
  bool using_gpio_data_ready() const;

private:
  struct Impl;
  Impl* impl_;
};

}  // namespace suit_canspi_bridge

#endif  // SUIT_CANSPI_BRIDGE_SPIDEV_TRANSPORT_HPP_
