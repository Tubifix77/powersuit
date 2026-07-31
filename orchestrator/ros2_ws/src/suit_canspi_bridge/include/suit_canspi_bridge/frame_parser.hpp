// Thin wrapper over ps_spi_frame_build / ps_spi_frame_parse
// (common/c/include/powersuit_proto/spi_frame.h). Never reimplement framing,
// CRC or ADPCM here — this class only tracks per-direction sequence and
// counters on top of the C reference implementation.
#ifndef SUIT_CANSPI_BRIDGE_FRAME_PARSER_HPP_
#define SUIT_CANSPI_BRIDGE_FRAME_PARSER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

extern "C" {
#include "powersuit_proto/spi_frame.h"
}

namespace suit_canspi_bridge {

struct FrameStats {
  uint64_t frames_ok = 0;
  uint64_t idle_frames = 0;
  uint64_t crc_errors = 0;
  uint64_t seq_gaps = 0;
  uint64_t resyncs = 0;
};

enum class ParseResult {
  kOk,     // valid, non-idle frame; records/flags/seq populated
  kIdle,   // all-zero buffer: not an error, nothing to dispatch
  kError,  // bad magic/ver/count/crc; counters updated
};

class FrameParser {
public:
  FrameParser() = default;

  // Serialize `recs` (n <= PS_SPI_MAX_RECORDS) into `buf` (must be
  // PS_SPI_XFER_SIZE bytes). Assigns and advances the per-direction tx seq.
  // Returns bytes used, or 0 if n exceeds PS_SPI_MAX_RECORDS.
  size_t build(uint8_t* buf, uint8_t flags,
               const std::vector<ps_can_record_t>& recs);

  // Parse a received PS_SPI_XFER_SIZE-byte transaction buffer. On kOk, fills
  // `out_records` and `out_flags`/`out_seq`.
  ParseResult parse(const uint8_t* buf, size_t len,
                     std::vector<ps_can_record_t>& out_records,
                     uint8_t& out_flags, uint8_t& out_seq);

  const FrameStats& stats() const { return stats_; }
  uint8_t tx_seq() const { return tx_seq_; }

private:
  uint8_t tx_seq_ = 0;
  bool have_rx_seq_ = false;
  uint8_t last_rx_seq_ = 0;
  FrameStats stats_;
};

// True if the entire buffer is zero — treated as an idle frame, not an
// error (SPI transactions run at a fixed cadence even with nothing to say).
bool is_all_zero(const uint8_t* buf, size_t len);

}  // namespace suit_canspi_bridge

#endif  // SUIT_CANSPI_BRIDGE_FRAME_PARSER_HPP_
