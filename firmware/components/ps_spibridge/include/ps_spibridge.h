/* ps_spibridge — ESP32-P4 SPI slave bridge to Node 8 (docs/network-map.md §6).
 * FROZEN API. Fixed 512-byte full-duplex DMA transactions on SPI2 (the only P4
 * slave-capable controller), ping-pong buffers, DATA_READY GPIO asserted whenever
 * the uplink queue is non-empty. Framing/CRC via powersuit_proto/spi_frame. */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "powersuit_proto/spi_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int mosi_gpio, miso_gpio, sclk_gpio, cs_gpio;
    int data_ready_gpio;         /* asserted high while uplink pending */
    size_t uplink_queue_len;     /* 0 = default 256 records */
    size_t downlink_queue_len;   /* 0 = default 128 records */
} ps_spib_config_t;

esp_err_t ps_spib_start(const ps_spib_config_t *cfg);

/* Producer side (gateway/BMS/audio): queue a record for the Pi. Returns false when
 * the queue is full — caller drops and ps_spibridge sets SPIF_OVERFLOW on the next
 * uplink frame header. Safe from any task, NOT from ISRs. */
bool ps_spib_uplink_push(const ps_can_record_t *rec);

/* Consumer side: records the Pi sent down. Blocks up to `timeout`. */
bool ps_spib_downlink_pop(ps_can_record_t *rec, TickType_t timeout);

/* Set/clear sticky header flag bits (PS_SPIF_ESTOP_LATCHED, PS_SPIF_HUB_FAULT). */
void ps_spib_set_flag(uint8_t flag, bool on);

typedef struct {
    uint32_t transactions, crc_errors, uplink_overflows, downlink_records, seq_gaps;
} ps_spib_stats_t;
void ps_spib_get_stats(ps_spib_stats_t *out);

#ifdef __cplusplus
}
#endif
