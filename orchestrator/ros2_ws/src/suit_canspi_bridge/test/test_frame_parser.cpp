// SPI framing round-trip, corruption handling, and the cross-language lock:
// a frame produced by the Python implementation must parse identically here.
#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "suit_canspi_bridge/frame_parser.hpp"

extern "C" {
#include "powersuit_proto/can_id.h"
#include "vectors/proto_vectors.h"
}

using suit_canspi_bridge::FrameParser;
using suit_canspi_bridge::ParseResult;

namespace {

ps_can_record_t make_record(int i) {
  ps_can_record_t r{};
  r.id = ps_can_id_pack(PS_CLS_TELEM, static_cast<uint8_t>(1 + (i % 6)), PS_NODE_ORCH,
                        PS_T_JOINT_STATE, static_cast<uint8_t>(i));
  r.bus = static_cast<uint8_t>(i % 2);
  r.dlc = 8;
  r.ts_ms = static_cast<uint16_t>(1000 + i);
  for (int j = 0; j < 8; ++j) {
    r.data[j] = static_cast<uint8_t>(i * 16 + j);
  }
  return r;
}

}  // namespace

TEST(FrameParser, RoundTripsAFullFrame) {
  FrameParser tx, rx;
  std::vector<ps_can_record_t> recs;
  for (int i = 0; i < PS_SPI_MAX_RECORDS; ++i) {
    recs.push_back(make_record(i));
  }

  uint8_t buf[PS_SPI_XFER_SIZE];
  ASSERT_GT(tx.build(buf, PS_SPIF_MORE_PENDING, recs), 0u);

  std::vector<ps_can_record_t> out;
  uint8_t flags = 0, seq = 0;
  ASSERT_EQ(rx.parse(buf, sizeof(buf), out, flags, seq), ParseResult::kOk);
  EXPECT_EQ(flags, PS_SPIF_MORE_PENDING);
  ASSERT_EQ(out.size(), recs.size());
  for (size_t i = 0; i < out.size(); ++i) {
    EXPECT_EQ(out[i].id, recs[i].id);
    EXPECT_EQ(out[i].dlc, recs[i].dlc);
    EXPECT_EQ(0, std::memcmp(out[i].data, recs[i].data, 8));
  }
  EXPECT_EQ(rx.stats().frames_ok, 1u);
  EXPECT_EQ(rx.stats().crc_errors, 0u);
}

TEST(FrameParser, AllZeroBufferIsIdleNotAnError) {
  FrameParser rx;
  uint8_t buf[PS_SPI_XFER_SIZE];
  std::memset(buf, 0, sizeof(buf));

  std::vector<ps_can_record_t> out;
  uint8_t flags = 0, seq = 0;
  EXPECT_EQ(rx.parse(buf, sizeof(buf), out, flags, seq), ParseResult::kIdle);
  EXPECT_EQ(rx.stats().crc_errors, 0u);
  EXPECT_EQ(rx.stats().idle_frames, 1u);
}

TEST(FrameParser, CrcCatchesAFlippedPayloadBit) {
  FrameParser tx, rx;
  std::vector<ps_can_record_t> recs{make_record(0), make_record(1)};
  uint8_t buf[PS_SPI_XFER_SIZE];
  tx.build(buf, 0, recs);

  buf[PS_SPI_HDR_SIZE + 12] ^= 0x40;

  std::vector<ps_can_record_t> out;
  uint8_t flags = 0, seq = 0;
  EXPECT_EQ(rx.parse(buf, sizeof(buf), out, flags, seq), ParseResult::kError);
  EXPECT_EQ(rx.stats().crc_errors, 1u);
  EXPECT_TRUE(out.empty()) << "a rejected frame must yield no records at all";
}

TEST(FrameParser, CountsSequenceGaps) {
  FrameParser tx, rx;
  std::vector<ps_can_record_t> recs{make_record(0)};
  uint8_t buf[PS_SPI_XFER_SIZE];
  std::vector<ps_can_record_t> out;
  uint8_t flags = 0, seq = 0;

  tx.build(buf, 0, recs);
  rx.parse(buf, sizeof(buf), out, flags, seq);
  tx.build(buf, 0, recs);  // seq 1, deliberately dropped
  tx.build(buf, 0, recs);  // seq 2 arrives instead
  rx.parse(buf, sizeof(buf), out, flags, seq);

  EXPECT_EQ(rx.stats().seq_gaps, 1u);
}

TEST(FrameParser, ParsesAFramePythonProduced) {
  // firmware/tests/host/vectors/proto_vectors.h is generated from the Python
  // implementation. If this fails, the two protocol codecs have drifted.
  FrameParser rx;
  std::vector<ps_can_record_t> out;
  uint8_t flags = 0, seq = 0;
  ASSERT_EQ(rx.parse(PS_VEC_SPI_FRAME, PS_VEC_SPI_FRAME_LEN, out, flags, seq),
            ParseResult::kOk);
  EXPECT_EQ(flags, PS_VEC_SPI_FLAGS);
  EXPECT_EQ(seq, PS_VEC_SPI_SEQ);
  ASSERT_EQ(out.size(), static_cast<size_t>(PS_VEC_SPI_COUNT));
  for (size_t i = 0; i < out.size(); ++i) {
    const ps_can_record_t want = make_record(static_cast<int>(i));
    EXPECT_EQ(out[i].id, want.id);
    EXPECT_EQ(0, std::memcmp(out[i].data, want.data, 8));
  }
}

TEST(FrameParser, RefusesMoreRecordsThanAFrameHolds) {
  FrameParser tx;
  std::vector<ps_can_record_t> recs;
  for (int i = 0; i < PS_SPI_MAX_RECORDS + 1; ++i) {
    recs.push_back(make_record(i));
  }
  uint8_t buf[PS_SPI_XFER_SIZE];
  EXPECT_EQ(tx.build(buf, 0, recs), 0u);
}
