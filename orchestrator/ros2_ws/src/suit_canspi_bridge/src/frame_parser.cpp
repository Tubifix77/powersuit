#include "suit_canspi_bridge/frame_parser.hpp"

namespace suit_canspi_bridge {

bool is_all_zero(const uint8_t* buf, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    if (buf[i] != 0) {
      return false;
    }
  }
  return true;
}

size_t FrameParser::build(uint8_t* buf, uint8_t flags,
                           const std::vector<ps_can_record_t>& recs) {
  const ps_can_record_t* recs_ptr = recs.empty() ? nullptr : recs.data();
  size_t used = ps_spi_frame_build(buf, flags, tx_seq_, recs_ptr, recs.size());
  if (used > 0) {
    ++tx_seq_;  // uint8_t wraps at 256, matching the wire field.
  }
  return used;
}

ParseResult FrameParser::parse(const uint8_t* buf, size_t len,
                                std::vector<ps_can_record_t>& out_records,
                                uint8_t& out_flags, uint8_t& out_seq) {
  out_records.clear();

  // A fixed-cadence SPI link legitimately has nothing to say on a given
  // transaction; an all-zero buffer is that idle case, not a corrupt frame.
  if (is_all_zero(buf, len)) {
    ++stats_.idle_frames;
    return ParseResult::kIdle;
  }

  ps_spi_view_t view{};
  int rc = ps_spi_frame_parse(buf, len, &view);
  if (rc != PS_SPI_OK) {
    if (rc == PS_SPI_EBADMAGIC) {
      // Bad magic is the "stream desynchronized" case; ps_spi_frame_scan is
      // the recovery helper firmware/tools/suit_sim also use to locate the
      // next plausible frame start for diagnostics.
      ps_spi_frame_scan(buf, len, 0);
      ++stats_.resyncs;
    } else {
      // EBADVER / EBADCOUNT / EBADCRC / ESHORT: all bucketed as CRC/format
      // errors — none of them represent a frame we can trust the payload of.
      ++stats_.crc_errors;
    }
    return ParseResult::kError;
  }

  out_flags = view.flags;
  out_seq = view.seq;

  if (have_rx_seq_) {
    const uint8_t expected = static_cast<uint8_t>(last_rx_seq_ + 1);
    if (view.seq != expected) {
      ++stats_.seq_gaps;
    }
  }
  have_rx_seq_ = true;
  last_rx_seq_ = view.seq;

  out_records.reserve(view.count);
  for (size_t i = 0; i < view.count; ++i) {
    ps_can_record_t rec{};
    if (ps_spi_view_record(&view, i, &rec) == 0) {
      out_records.push_back(rec);
    }
  }

  ++stats_.frames_ok;
  return ParseResult::kOk;
}

}  // namespace suit_canspi_bridge
