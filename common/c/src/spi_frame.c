#include "powersuit_proto/spi_frame.h"
#include "powersuit_proto/crc16.h"

#include <string.h>

static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void wr_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

void ps_spi_record_write(uint8_t out[16], const ps_can_record_t *rec)
{
    wr_u32(out, rec->id & 0x1FFFFFFFu);
    out[4] = (uint8_t)(((rec->bus & 0xFu) << 4) | (rec->dlc & 0xFu));
    out[5] = 0;
    wr_u16(out + 6, rec->ts_ms);
    memcpy(out + 8, rec->data, 8);
}

int ps_spi_record_read(const uint8_t in[16], ps_can_record_t *rec)
{
    rec->id = rd_u32(in) & 0x1FFFFFFFu;
    rec->bus = (uint8_t)(in[4] >> 4);
    rec->dlc = (uint8_t)(in[4] & 0xFu);
    rec->ts_ms = rd_u16(in + 6);
    memcpy(rec->data, in + 8, 8);
    return (rec->dlc <= 8) ? PS_SPI_OK : PS_SPI_EBADCOUNT;
}

size_t ps_spi_frame_build(uint8_t *buf, uint8_t flags, uint8_t seq,
                          const ps_can_record_t *recs, size_t n)
{
    if (n > PS_SPI_MAX_RECORDS) {
        return 0;
    }
    memset(buf, 0, PS_SPI_XFER_SIZE);
    wr_u16(buf, PS_SPI_MAGIC);
    buf[2] = PS_SPI_VER;
    buf[3] = flags;
    buf[4] = (uint8_t)n;
    buf[5] = seq;
    for (size_t i = 0; i < n; i++) {
        ps_spi_record_write(buf + PS_SPI_HDR_SIZE + i * PS_SPI_REC_SIZE, &recs[i]);
    }
    uint16_t crc = ps_crc16_update(0xFFFFu, buf + 2, 4);
    crc = ps_crc16_update(crc, buf + PS_SPI_HDR_SIZE, n * PS_SPI_REC_SIZE);
    wr_u16(buf + 6, crc);
    return PS_SPI_HDR_SIZE + n * PS_SPI_REC_SIZE;
}

int ps_spi_frame_parse(const uint8_t *buf, size_t len, ps_spi_view_t *view)
{
    if (len < PS_SPI_HDR_SIZE) {
        return PS_SPI_ESHORT;
    }
    if (rd_u16(buf) != PS_SPI_MAGIC) {
        return PS_SPI_EBADMAGIC;
    }
    if (buf[2] != PS_SPI_VER) {
        return PS_SPI_EBADVER;
    }
    uint8_t count = buf[4];
    if (count > PS_SPI_MAX_RECORDS) {
        return PS_SPI_EBADCOUNT;
    }
    size_t need = PS_SPI_HDR_SIZE + (size_t)count * PS_SPI_REC_SIZE;
    if (len < need) {
        return PS_SPI_ESHORT;
    }
    uint16_t crc = ps_crc16_update(0xFFFFu, buf + 2, 4);
    crc = ps_crc16_update(crc, buf + PS_SPI_HDR_SIZE, (size_t)count * PS_SPI_REC_SIZE);
    if (crc != rd_u16(buf + 6)) {
        return PS_SPI_EBADCRC;
    }
    view->flags = buf[3];
    view->count = count;
    view->seq = buf[5];
    view->records = buf + PS_SPI_HDR_SIZE;
    return PS_SPI_OK;
}

int ps_spi_view_record(const ps_spi_view_t *view, size_t i, ps_can_record_t *rec)
{
    if (i >= view->count) {
        return PS_SPI_EBADCOUNT;
    }
    return ps_spi_record_read(view->records + i * PS_SPI_REC_SIZE, rec);
}

long ps_spi_frame_scan(const uint8_t *buf, size_t len, size_t from)
{
    if (len < 2) {
        return -1;
    }
    for (size_t i = from; i + 1 < len; i++) {
        if (buf[i] == (uint8_t)(PS_SPI_MAGIC & 0xFFu) &&
            buf[i + 1] == (uint8_t)(PS_SPI_MAGIC >> 8)) {
            return (long)i;
        }
    }
    return -1;
}
