#include "suit_canspi_bridge/spidev_transport.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <thread>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>

// libgpiod v2 (apt: libgpiod-dev on Ubuntu 24.04). Guarded so this package
// builds in a bare container without it — polling is then the only path.
#if defined(__has_include)
#if __has_include(<gpiod.h>)
#include <gpiod.h>
#define SUIT_CANSPI_BRIDGE_HAVE_GPIOD 1
#endif
#endif

namespace suit_canspi_bridge {

struct SpidevTransport::Impl {
  SpidevConfig cfg;
  int fd = -1;
  bool gpio_ready = false;
#ifdef SUIT_CANSPI_BRIDGE_HAVE_GPIOD
  gpiod_chip* chip = nullptr;
  gpiod_line_request* request = nullptr;
#endif

  ~Impl() {
    if (fd >= 0) {
      ::close(fd);
    }
#ifdef SUIT_CANSPI_BRIDGE_HAVE_GPIOD
    if (request != nullptr) {
      gpiod_line_request_release(request);
    }
    if (chip != nullptr) {
      gpiod_chip_close(chip);
    }
#endif
  }

  // Best-effort: returns false (never fatal) if params are unset or the
  // request fails, in which case the caller falls back to polling.
  bool open_gpio() {
#ifdef SUIT_CANSPI_BRIDGE_HAVE_GPIOD
    if (cfg.gpiochip.empty()) {
      return false;
    }
    chip = gpiod_chip_open(cfg.gpiochip.c_str());
    if (chip == nullptr) {
      return false;
    }

    gpiod_line_settings* settings = gpiod_line_settings_new();
    if (settings == nullptr) {
      return false;
    }
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);

    gpiod_line_config* line_cfg = gpiod_line_config_new();
    if (line_cfg == nullptr) {
      gpiod_line_settings_free(settings);
      return false;
    }
    unsigned int offset = cfg.gpio_line;
    const bool added =
        gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings) == 0;

    gpiod_request_config* req_cfg = gpiod_request_config_new();
    if (req_cfg != nullptr) {
      gpiod_request_config_set_consumer(req_cfg, "suit_canspi_bridge");
    }

    if (added) {
      request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    }

    if (req_cfg != nullptr) {
      gpiod_request_config_free(req_cfg);
    }
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);

    gpio_ready = (request != nullptr);
    return gpio_ready;
#else
    return false;
#endif
  }
};

SpidevTransport::SpidevTransport(const SpidevConfig& cfg) : impl_(new Impl{}) {
  impl_->cfg = cfg;
}

SpidevTransport::~SpidevTransport() { delete impl_; }

bool SpidevTransport::open() {
  impl_->fd = ::open(impl_->cfg.device.c_str(), O_RDWR);
  if (impl_->fd < 0) {
    return false;
  }

  uint8_t mode = impl_->cfg.mode;
  uint8_t bits = 8;
  uint32_t speed = impl_->cfg.speed_hz;
  if (::ioctl(impl_->fd, SPI_IOC_WR_MODE, &mode) < 0 ||
      ::ioctl(impl_->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
      ::ioctl(impl_->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
    return false;
  }

  impl_->open_gpio();  // best effort; polling fallback if this fails
  return true;
}

bool SpidevTransport::xfer(const uint8_t* tx, uint8_t* rx, size_t len) {
  if (impl_->fd < 0) {
    return false;
  }
  struct spi_ioc_transfer tr;
  std::memset(&tr, 0, sizeof(tr));
  tr.tx_buf = reinterpret_cast<uint64_t>(tx);
  tr.rx_buf = reinterpret_cast<uint64_t>(rx);
  tr.len = static_cast<uint32_t>(len);
  tr.speed_hz = impl_->cfg.speed_hz;
  tr.bits_per_word = 8;

  const int ret = ::ioctl(impl_->fd, SPI_IOC_MESSAGE(1), &tr);
  return ret >= static_cast<int>(len);
}

bool SpidevTransport::wait_data_ready(std::chrono::milliseconds timeout) {
#ifdef SUIT_CANSPI_BRIDGE_HAVE_GPIOD
  if (impl_->gpio_ready && impl_->request != nullptr) {
    const int64_t timeout_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count();
    const int rc = gpiod_line_request_wait_edge_events(impl_->request, timeout_ns);
    if (rc > 0) {
      gpiod_edge_event_buffer* buf = gpiod_edge_event_buffer_new(1);
      if (buf != nullptr) {
        gpiod_line_request_read_edge_events(impl_->request, buf, 1);
        gpiod_edge_event_buffer_free(buf);
      }
      return true;
    }
    return false;
  }
#endif
  // Polling fallback: pace the caller at ~1 kHz regardless of a real
  // DATA_READY signal, per the task contract for bare-container builds.
  std::this_thread::sleep_for(std::min(timeout, std::chrono::milliseconds(1)));
  return true;
}

bool SpidevTransport::using_gpio_data_ready() const {
#ifdef SUIT_CANSPI_BRIDGE_HAVE_GPIOD
  return impl_->gpio_ready;
#else
  return false;
#endif
}

}  // namespace suit_canspi_bridge
