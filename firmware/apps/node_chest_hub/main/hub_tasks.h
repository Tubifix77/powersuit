/* hub_tasks.h — internal wiring between the node_chest_hub tasks.
 * App-private; the frozen component APIs live in ps_router/ps_spibridge. */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#include "ps_can.h"
#include "powersuit_proto/wire.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t fwd[4];         /* frames forwarded out of each PS_PORT_* */
    uint32_t local_consumed; /* frames dispatched to the hub itself */
    uint32_t anomalies;      /* nonzero-class frames the policy routed nowhere */
    uint32_t queue_drops;    /* routing queue full (rx cb side) */
    uint32_t tx_timeouts;    /* ps_can_send timeouts while forwarding */
} hub_gw_counters_t;

/* gateway_task.c — dual-CAN pump, SPI downlink pump, suit-state mirror. */
esp_err_t hub_gateway_start(ps_can_handle_t can1, ps_can_handle_t can2);
void hub_gateway_safety_tick(void);          /* 5 ms esp_timer callback */
void hub_gateway_broadcast_time_sync(void);  /* 1 Hz esp_timer callback */
uint8_t hub_gateway_safety_state(void);      /* PS_STATE_* mirror of suit state */
void hub_gateway_local_estop(uint8_t cause); /* BMS trip -> latch mirror */
void hub_gateway_counters(hub_gw_counters_t *out);

/* bms_task.c — 1 kHz sampling, trip escalation, short-circuit observer. */
esp_err_t hub_bms_start(ps_can_handle_t can1, ps_can_handle_t can2);
void hub_bms_notify_clear_accepted(void);    /* gateway: CLEAR_ESTOP accepted */

/* led_task.c — 24-px status ring. */
esp_err_t hub_led_start(void);
void hub_led_override(const ps_led_pattern_t *pat); /* CONTROL LED_PATTERN dst=7 */

#ifdef __cplusplus
}
#endif
