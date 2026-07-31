/* SPI bridge framing (Node 7 <-> Node 8) — normative reference: docs/network-map.md §6.
 * Fixed 512-byte full-duplex transactions; 8-byte header + up to 31 x 16-byte CAN records.
 * CRC16-CCITT-FALSE over bytes [2..5] (ver..seq) plus all record bytes. */
#ifndef POWERSUIT_PROTO_SPI_FRAME_H
#define POWERSUIT_PROTO_SPI_FRAME_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PS_SPI_MAGIC        0xA55Au
#define PS_SPI_VER          1u
#define PS_SPI_XFER_SIZE    512u
#define PS_SPI_HDR_SIZE     8u
#define PS_SPI_REC_SIZE     16u
#define PS_SPI_MAX_RECORDS  31u

/* Header flag bits. */
#define PS_SPIF_ESTOP_LATCHED  (1u << 0)
#define PS_SPIF_OVERFLOW       (1u << 1)
#define PS_SPIF_MORE_PENDING   (1u << 2)
#define PS_SPIF_HUB_FAULT      (1u << 3)

/* Record bus nibble values. */
#define PS_SPI_BUS_CAN1       0u
#define PS_SPI_BUS_CAN2       1u
#define PS_SPI_BUS_HUB_LOCAL  3u

/* Parse error codes. */
#define PS_SPI_OK          0
#define PS_SPI_EBADMAGIC  -1
#define PS_SPI_EBADVER    -2
#define PS_SPI_EBADCOUNT  -3
#define PS_SPI_EBADCRC    -4
#define PS_SPI_ESHORT     -5

typedef struct {
    uint32_t id;      /* 29-bit CAN identifier value */
    uint8_t  bus;     /* PS_SPI_BUS_* */
    uint8_t  dlc;     /* 0..8 */
    uint16_t ts_ms;   /* hub-local ms, unwrapped via MGMT TIME_SYNC */
    uint8_t  data[8];
} ps_can_record_t;

typedef struct {
    uint8_t  flags;
    uint8_t  count;
    uint8_t  seq;
    const uint8_t *records;  /* count * 16 bytes, still serialized */
} ps_spi_view_t;

/* Serialize one record into a 16-byte slot. */
void ps_spi_record_write(uint8_t out[16], const ps_can_record_t *rec);
/* Deserialize one 16-byte slot. Returns 0, or PS_SPI_EBADCOUNT on dlc > 8. */
int ps_spi_record_read(const uint8_t in[16], ps_can_record_t *rec);

/* Build a full transaction buffer (buf must hold PS_SPI_XFER_SIZE bytes; the tail
 * beyond header+records is zero-filled). Returns bytes used (hdr + n*16), or 0 when
 * n > PS_SPI_MAX_RECORDS. */
size_t ps_spi_frame_build(uint8_t *buf, uint8_t flags, uint8_t seq,
                          const ps_can_record_t *recs, size_t n);

/* Validate a transaction buffer in place. On success fills view (records pointer
 * aliases buf). Returns PS_SPI_OK or a PS_SPI_E* code. */
int ps_spi_frame_parse(const uint8_t *buf, size_t len, ps_spi_view_t *view);

/* Fetch record i from a parsed view. Returns 0 or PS_SPI_EBADCOUNT. */
int ps_spi_view_record(const ps_spi_view_t *view, size_t i, ps_can_record_t *rec);

/* Locate the next plausible frame start (magic byte pair) in a byte stream at or
 * after `from`. Returns offset, or -1 when not found. Recovery helper for
 * misaligned/corrupted streams. */
long ps_spi_frame_scan(const uint8_t *buf, size_t len, size_t from);

#ifdef __cplusplus
}
#endif

#endif /* POWERSUIT_PROTO_SPI_FRAME_H */
