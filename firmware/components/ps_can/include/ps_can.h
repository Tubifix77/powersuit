/* ps_can — TWAI (CAN) driver wrapper. FROZEN API: implemented once on the new
 * esp_driver_twai node API (IDF v5.5), identical on ESP32-S3 (1 controller) and
 * ESP32-P4 (uses controllers 0 and 1). Dispatch is class-based per docs/network-map.md §2.
 *
 * Threading: RX callbacks run in the ps_can RX task context (NOT ISR). Callbacks must
 * be short; hand off to queues for real work. ps_can_send is safe from any task. */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "powersuit_proto/can_id.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t id;       /* 29-bit identifier value (always extended) */
    uint8_t  dlc;      /* 0..8 */
    uint8_t  data[8];
} ps_can_frame_t;

/* Link states reported by the state callback. */
enum {
    PS_CAN_STATE_RUNNING     = 0,
    PS_CAN_STATE_ERR_PASSIVE = 1,
    PS_CAN_STATE_BUS_OFF     = 2,
    PS_CAN_STATE_RECOVERING  = 3,
};

typedef void (*ps_can_rx_cb_t)(const ps_can_frame_t *frame, void *arg);
typedef void (*ps_can_state_cb_t)(int state, void *arg);

typedef struct {
    int      controller;    /* TWAI controller index: S3 apps use 0; hub uses 0 (bus 1) and 1 (bus 2) */
    int      tx_gpio;
    int      rx_gpio;
    uint32_t bitrate;       /* contract: 1000000 */
    uint8_t  node_id;       /* accept dst == node_id or broadcast; 0 = promiscuous (gateway) */
    size_t   tx_queue_len;  /* 0 = default (32) */
    size_t   rx_queue_len;  /* 0 = default (64) */
} ps_can_config_t;

typedef struct ps_can_ctx *ps_can_handle_t;

esp_err_t ps_can_open(const ps_can_config_t *cfg, ps_can_handle_t *out);
esp_err_t ps_can_close(ps_can_handle_t h);

/* Queue a frame for transmission. Returns ESP_ERR_TIMEOUT when the TX queue stays
 * full for `timeout` ticks (callers on the SAFETY plane use portMAX_DELAY-free
 * small timeouts and escalate). */
esp_err_t ps_can_send(ps_can_handle_t h, const ps_can_frame_t *frame, TickType_t timeout);

/* At most one callback per class; passing NULL unregisters. Frames whose class has
 * no callback are counted (rx_dropped) and discarded. */
esp_err_t ps_can_register_class_cb(ps_can_handle_t h, uint8_t cls, ps_can_rx_cb_t cb, void *arg);
esp_err_t ps_can_register_state_cb(ps_can_handle_t h, ps_can_state_cb_t cb, void *arg);

/* Bus-off recovery is automatic (with backoff); this reports the current state. */
int ps_can_state(ps_can_handle_t h);

typedef struct {
    uint32_t rx_frames, tx_frames, rx_dropped, tx_timeouts, bus_errors, recoveries;
} ps_can_stats_t;
void ps_can_get_stats(ps_can_handle_t h, ps_can_stats_t *out);

#ifdef __cplusplus
}
#endif
