#include "powersuit_proto/can_id.h"
#include "powersuit_proto/spi_frame.h"
#include "vectors/proto_vectors.h"
#include "ps_test.h"

static ps_can_record_t make_record(int i)
{
    ps_can_record_t r;
    r.id = ps_can_id_pack(PS_CLS_TELEM, (uint8_t)(1 + (i % 6)), PS_NODE_ORCH,
                          PS_T_JOINT_STATE, (uint8_t)i);
    r.bus = (uint8_t)(i % 2);
    r.dlc = 8;
    r.ts_ms = (uint16_t)(1000 + i);
    for (int j = 0; j < 8; j++) {
        r.data[j] = (uint8_t)(i * 16 + j);
    }
    return r;
}

PS_TEST(roundtrip_full)
{
    static uint8_t buf[PS_SPI_XFER_SIZE];
    ps_can_record_t recs[PS_SPI_MAX_RECORDS];
    for (int i = 0; i < (int)PS_SPI_MAX_RECORDS; i++) {
        recs[i] = make_record(i);
    }
    size_t used = ps_spi_frame_build(buf, PS_SPIF_MORE_PENDING, 42, recs, PS_SPI_MAX_RECORDS);
    PS_ASSERT_EQ_INT(used, PS_SPI_HDR_SIZE + PS_SPI_MAX_RECORDS * PS_SPI_REC_SIZE);

    ps_spi_view_t view;
    PS_ASSERT_EQ_INT(ps_spi_frame_parse(buf, sizeof(buf), &view), PS_SPI_OK);
    PS_ASSERT_EQ_INT(view.flags, PS_SPIF_MORE_PENDING);
    PS_ASSERT_EQ_INT(view.seq, 42);
    PS_ASSERT_EQ_INT(view.count, PS_SPI_MAX_RECORDS);
    for (size_t i = 0; i < view.count; i++) {
        ps_can_record_t rec, want = make_record((int)i);
        PS_ASSERT_EQ_INT(ps_spi_view_record(&view, i, &rec), PS_SPI_OK);
        PS_ASSERT_EQ_INT(rec.id, want.id);
        PS_ASSERT_EQ_INT(rec.bus, want.bus);
        PS_ASSERT_EQ_INT(rec.dlc, want.dlc);
        PS_ASSERT_EQ_INT(rec.ts_ms, want.ts_ms);
        PS_ASSERT_EQ_MEM(rec.data, want.data, 8);
    }
}

PS_TEST(crc_detects_corruption)
{
    static uint8_t buf[PS_SPI_XFER_SIZE];
    ps_can_record_t recs[5];
    for (int i = 0; i < 5; i++) {
        recs[i] = make_record(i);
    }
    ps_spi_frame_build(buf, 0, 7, recs, 5);
    buf[20] ^= 0x40;
    ps_spi_view_t view;
    PS_ASSERT_EQ_INT(ps_spi_frame_parse(buf, sizeof(buf), &view), PS_SPI_EBADCRC);
}

PS_TEST(bad_magic_and_scan)
{
    static uint8_t buf[PS_SPI_XFER_SIZE];
    ps_can_record_t rec = make_record(0);
    ps_spi_frame_build(buf, 0, 9, &rec, 1);

    static uint8_t stream[16 + PS_SPI_XFER_SIZE];
    memset(stream, 0xDE, 16);
    memcpy(stream + 16, buf, PS_SPI_XFER_SIZE);

    ps_spi_view_t view;
    PS_ASSERT_EQ_INT(ps_spi_frame_parse(stream, sizeof(stream), &view), PS_SPI_EBADMAGIC);
    long off = ps_spi_frame_scan(stream, sizeof(stream), 0);
    PS_ASSERT_EQ_INT(off, 16);
    PS_ASSERT_EQ_INT(ps_spi_frame_parse(stream + off, sizeof(stream) - (size_t)off, &view), PS_SPI_OK);
    PS_ASSERT_EQ_INT(view.seq, 9);
}

PS_TEST(python_generated_frame_parses)
{
    /* Cross-language lock: frame built by powersuit_proto (Python) must parse here
     * and contain the deterministic records both sides derive from the same formula. */
    ps_spi_view_t view;
    PS_ASSERT_EQ_INT(ps_spi_frame_parse(PS_VEC_SPI_FRAME, PS_VEC_SPI_FRAME_LEN, &view), PS_SPI_OK);
    PS_ASSERT_EQ_INT(view.flags, PS_VEC_SPI_FLAGS);
    PS_ASSERT_EQ_INT(view.seq, PS_VEC_SPI_SEQ);
    PS_ASSERT_EQ_INT(view.count, PS_VEC_SPI_COUNT);
    for (size_t i = 0; i < view.count; i++) {
        ps_can_record_t rec, want = make_record((int)i);
        PS_ASSERT_EQ_INT(ps_spi_view_record(&view, i, &rec), PS_SPI_OK);
        PS_ASSERT_EQ_INT(rec.id, want.id);
        PS_ASSERT_EQ_MEM(rec.data, want.data, 8);
    }
}

PS_TEST(count_overflow_rejected)
{
    static uint8_t buf[PS_SPI_XFER_SIZE];
    ps_can_record_t recs[1] = { make_record(0) };
    PS_ASSERT_EQ_INT(ps_spi_frame_build(buf, 0, 0, recs, PS_SPI_MAX_RECORDS + 1), 0);
}

PS_TEST_MAIN()
