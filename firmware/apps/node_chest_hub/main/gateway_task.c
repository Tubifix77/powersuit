/* gateway_task.c — the Node 7 dual-CAN / SPI pump.
 *
 * Every frame from either CAN bus and every SPI downlink record goes through
 * ps_router_route(); this file only forwards to the returned port set and
 * dispatches PS_PORT_LOCAL — policy lives in ps_router, never here (§5).
 *
 * Suit-state mirror: the ps_safety ESP glue is a singleton wired to ONE
 * ps_can handle and exists to gate actuation. The hub observes BOTH buses and
 * never actuates (safety.md §2, hub row: "gateway and BMS unaffected — routing
 * IS the safety path"), so it instantiates the pure ps_safety_core_t directly,
 * fed by gateway LOCAL dispatch plus a 5 ms esp_timer tick. */
#include "hub_tasks.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "powersuit_proto/can_id.h"
#include "powersuit_proto/spi_frame.h"
#include "ps_router.h"
#include "ps_safety.h"
#include "ps_spibridge.h"

#include "board_hub.h"

static const char *TAG = "node_hub";

#define GW_ROUTE_QUEUE_LEN   128
#define GW_PUMP_PRIO         19    /* SAFETY cut-through path */
#define GW_DOWNLINK_PRIO     18
#define GW_CORE              0     /* comms core; BMS/LED live on core 1 */
#define GW_TX_TIMEOUT_TICKS  pdMS_TO_TICKS(2)

typedef struct {
    uint8_t port;              /* PS_PORT_CAN1 / PS_PORT_CAN2 */
    ps_can_frame_t frame;
} gw_item_t;

static ps_can_handle_t   s_can[2];
static QueueHandle_t     s_route_q;

static portMUX_TYPE      s_gw_lock = portMUX_INITIALIZER_UNLOCKED;
static hub_gw_counters_t s_ctr;

/* Suit-state mirror (pure core; see file header). */
static portMUX_TYPE      s_mirror_lock = portMUX_INITIALIZER_UNLOCKED;
static ps_safety_core_t  s_mirror;
static uint8_t           s_last_pub_state = 0xFF;

/* TIME_SYNC: epoch adopted from the Pi's downlink records (src == 8). */
static portMUX_TYPE      s_time_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t          s_epoch_ms_lo;
static uint32_t          s_epoch_ref_ms;
static bool              s_epoch_valid;
static uint16_t          s_time_seq;

/* FLOW_CTL mirror — informational only (audio downshift is Pi-driven). */
static volatile uint8_t  s_flow_level[8];

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

uint8_t hub_gateway_safety_state(void)
{
    portENTER_CRITICAL(&s_mirror_lock);
    uint8_t st = s_mirror.state;
    portEXIT_CRITICAL(&s_mirror_lock);
    return st;
}

/* Publish mirror transitions outside the spinlock: sticky SPI header flag +
 * one log line. LED reads the state by polling; nothing else to push. */
static void mirror_publish(uint8_t state)
{
    if (state == s_last_pub_state) {
        return;
    }
    ps_spib_set_flag(PS_SPIF_ESTOP_LATCHED, state == PS_STATE_ESTOP);
    ESP_LOGI(TAG, "suit state mirror: %u -> %u", s_last_pub_state, state);
    s_last_pub_state = state;
}

/* Transition work happens in mirror_publish (polled); the core requires a
 * callback pointer that must not do real work under the caller's spinlock. */
static void mirror_noop_cb(uint8_t old_state, uint8_t new_state, uint8_t cause,
                           void *arg)
{
    (void)old_state; (void)new_state; (void)cause; (void)arg;
}

void hub_gateway_safety_tick(void)
{
    portENTER_CRITICAL(&s_mirror_lock);
    uint8_t st = ps_safety_core_tick(&s_mirror, now_ms());
    portEXIT_CRITICAL(&s_mirror_lock);
    mirror_publish(st);
}

void hub_gateway_local_estop(uint8_t cause)
{
    portENTER_CRITICAL(&s_mirror_lock);
    ps_safety_core_on_local_fault(&s_mirror, now_ms(), cause);
    uint8_t st = s_mirror.state;
    portEXIT_CRITICAL(&s_mirror_lock);
    mirror_publish(st);
}

void hub_gateway_counters(hub_gw_counters_t *out)
{
    portENTER_CRITICAL(&s_gw_lock);
    *out = s_ctr;
    portEXIT_CRITICAL(&s_gw_lock);
}

static void ctr_bump(uint32_t *field)
{
    portENTER_CRITICAL(&s_gw_lock);
    (*field)++;
    portEXIT_CRITICAL(&s_gw_lock);
}

/* ---------------- local dispatch (PS_PORT_LOCAL) ---------------- */

static void gw_local_safety(const ps_can_frame_t *f, uint8_t type)
{
    uint32_t now = now_ms();
    uint8_t st;
    switch (type) {
    case PS_T_HEARTBEAT: {
        ps_heartbeat_t hb;
        if (f->dlc < sizeof(hb)) {
            return;
        }
        PS_WIRE_READ(hb, f->data);
        portENTER_CRITICAL(&s_mirror_lock);
        ps_safety_core_on_heartbeat(&s_mirror, now, &hb);
        st = s_mirror.state;
        portEXIT_CRITICAL(&s_mirror_lock);
        mirror_publish(st);
        break;
    }
    case PS_T_ESTOP: {
        ps_estop_t es;
        if (f->dlc < sizeof(es)) {
            return;
        }
        PS_WIRE_READ(es, f->data);
        portENTER_CRITICAL(&s_mirror_lock);
        ps_safety_core_on_estop(&s_mirror, now, es.cause);
        st = s_mirror.state;
        portEXIT_CRITICAL(&s_mirror_lock);
        mirror_publish(st);
        break;
    }
    case PS_T_CLEAR_ESTOP: {
        ps_clear_estop_t cl;
        if (f->dlc < sizeof(cl)) {
            return;
        }
        PS_WIRE_READ(cl, f->data);
        bool accepted;
        portENTER_CRITICAL(&s_mirror_lock);
        accepted = ps_safety_core_on_clear(&s_mirror, now, cl.magic, cl.counter);
        st = s_mirror.state;
        portEXIT_CRITICAL(&s_mirror_lock);
        mirror_publish(st);
        if (accepted) {
            hub_bms_notify_clear_accepted(); /* re-arm interlock precondition */
        }
        break;
    }
    default: /* NODE_FAULT: observed via SPI by the Pi; nothing local to do */
        break;
    }
}

static void gw_local_control(const ps_can_frame_t *f, uint8_t type)
{
    switch (type) {
    case PS_T_LED_PATTERN: {
        ps_led_pattern_t pat;
        if (f->dlc < sizeof(pat)) {
            return;
        }
        PS_WIRE_READ(pat, f->data);
        hub_led_override(&pat);
        break;
    }
    case PS_T_MODE_SET: {
        ps_mode_set_t ms;
        if (f->dlc < 1) {
            return;
        }
        memset(&ms, 0, sizeof(ms));
        memcpy(&ms, f->data, f->dlc < sizeof(ms) ? f->dlc : sizeof(ms));
        portENTER_CRITICAL(&s_mirror_lock);
        ps_safety_core_on_mode_set(&s_mirror, now_ms(), ms.target_state);
        uint8_t st = s_mirror.state;
        portEXIT_CRITICAL(&s_mirror_lock);
        mirror_publish(st);
        break;
    }
    default: /* JOINT_CMD/FLAP_CMD to the hub make no sense: drop */
        break;
    }
}

static void gw_local_mgmt(const ps_can_frame_t *f, uint8_t src, uint8_t type)
{
    switch (type) {
    case PS_T_TIME_SYNC: {
        ps_time_sync_t sync;
        if (f->dlc < sizeof(sync) || src != PS_NODE_ORCH) {
            return; /* only the Pi is a time authority (§3.6) */
        }
        PS_WIRE_READ(sync, f->data);
        portENTER_CRITICAL(&s_time_lock);
        s_epoch_ms_lo = sync.epoch_ms_lo;
        s_epoch_ref_ms = now_ms();
        s_epoch_valid = true;
        portEXIT_CRITICAL(&s_time_lock);
        break;
    }
    case PS_T_FLOW_CTL: {
        ps_flow_ctl_t fc;
        if (f->dlc < 2) {
            return;
        }
        memset(&fc, 0, sizeof(fc));
        memcpy(&fc, f->data, f->dlc < sizeof(fc) ? f->dlc : sizeof(fc));
        if (fc.plane < 8) {
            s_flow_level[fc.plane] = fc.level; /* informational */
        }
        break;
    }
    default: /* STATS_REQ: 1 Hz stats are always on; LOG/VERSION: not for us */
        break;
    }
}

static void gw_local_dispatch(const ps_can_frame_t *f)
{
    ps_can_id_t id;
    ps_can_id_unpack(f->id, &id);
    ctr_bump(&s_ctr.local_consumed);
    switch (id.cls) {
    case PS_CLS_SAFETY:
        gw_local_safety(f, id.type);
        break;
    case PS_CLS_CONTROL:
        gw_local_control(f, id.type);
        break;
    case PS_CLS_MGMT:
        gw_local_mgmt(f, id.src, id.type);
        break;
    default: /* TELEM/XRCE/AUDIO have no hub consumer (§12.4) */
        break;
    }
}

/* ---------------- forwarding ---------------- */

static uint8_t origin_bus_tag(uint8_t origin_port)
{
    switch (origin_port) {
    case PS_PORT_CAN1: return PS_SPI_BUS_CAN1;
    case PS_PORT_CAN2: return PS_SPI_BUS_CAN2;
    default:           return PS_SPI_BUS_HUB_LOCAL;
    }
}

/* Cut-through order is deliberate: buses and SPI first, LOCAL dispatch last,
 * so an ESTOP is already on the wire before the hub processes it (safety.md §3). */
static void hub_forward(uint8_t origin_port, const ps_can_frame_t *f, uint8_t mask)
{
    if (mask & PS_PORT_BIT(PS_PORT_CAN1)) {
        if (ps_can_send(s_can[0], f, GW_TX_TIMEOUT_TICKS) == ESP_OK) {
            ctr_bump(&s_ctr.fwd[PS_PORT_CAN1]);
        } else {
            ctr_bump(&s_ctr.tx_timeouts);
        }
    }
    if (mask & PS_PORT_BIT(PS_PORT_CAN2)) {
        if (ps_can_send(s_can[1], f, GW_TX_TIMEOUT_TICKS) == ESP_OK) {
            ctr_bump(&s_ctr.fwd[PS_PORT_CAN2]);
        } else {
            ctr_bump(&s_ctr.tx_timeouts);
        }
    }
    if (mask & PS_PORT_BIT(PS_PORT_SPI)) {
        ps_can_record_t rec = {
            .id = f->id,
            .bus = origin_bus_tag(origin_port), /* §6: bus nibble = origin */
            .dlc = f->dlc,
            .ts_ms = (uint16_t)(esp_timer_get_time() / 1000),
        };
        memcpy(rec.data, f->data, sizeof(rec.data));
        if (ps_spib_uplink_push(&rec)) {
            ctr_bump(&s_ctr.fwd[PS_PORT_SPI]);
        } /* else: bridge sets SPIF_OVERFLOW and counts the drop */
    }
    if (mask & PS_PORT_BIT(PS_PORT_LOCAL)) {
        gw_local_dispatch(f);
    }
}

/* ---------------- CAN side ---------------- */

/* Runs in the ps_can RX task: queue and get out (contract: callbacks short). */
static void gw_rx_cb(const ps_can_frame_t *frame, void *arg)
{
    gw_item_t item = { .port = (uint8_t)(uintptr_t)arg, .frame = *frame };
    if (xQueueSend(s_route_q, &item, 0) != pdTRUE) {
        ctr_bump(&s_ctr.queue_drops);
    }
}

static void gw_pump_task(void *arg)
{
    (void)arg;
    gw_item_t item;
    for (;;) {
        if (xQueueReceive(s_route_q, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        uint8_t mask = ps_router_route(item.port, item.frame.id);
        if (mask == 0) {
            ctr_bump(&s_ctr.anomalies); /* e.g. CONTROL from CAN, audio on Bus 2 */
            continue;
        }
        hub_forward(item.port, &item.frame, mask);
    }
}

/* ---------------- SPI downlink side ---------------- */

static void gw_downlink_task(void *arg)
{
    (void)arg;
    ps_can_record_t rec;
    for (;;) {
        if (!ps_spib_downlink_pop(&rec, portMAX_DELAY)) {
            continue;
        }
        ps_can_frame_t f = { .id = rec.id & PS_CAN_ID_MASK,
                             .dlc = rec.dlc > 8 ? 8 : rec.dlc };
        memcpy(f.data, rec.data, sizeof(f.data));

        if (rec.bus == PS_SPI_BUS_HUB_LOCAL) {
            /* §6: downlink HUB_LOCAL = command consumed by the hub itself. */
            gw_local_dispatch(&f);
            continue;
        }
        uint8_t mask = ps_router_route(PS_PORT_SPI, f.id);
        if (mask == 0) {
            ctr_bump(&s_ctr.anomalies);
            continue;
        }
        /* SAFETY from SPI resolves to both buses + LOCAL (§5/§6). */
        hub_forward(PS_PORT_SPI, &f, mask);
    }
}

/* ---------------- TIME_SYNC broadcast (1 Hz, esp_timer context) ---------------- */

void hub_gateway_broadcast_time_sync(void)
{
    ps_time_sync_t sync;
    portENTER_CRITICAL(&s_time_lock);
    /* Until the Pi has synced us once, broadcast epoch 0: receivers treat a
     * zero epoch as "unsynced, keep local time" (§3.6). */
    sync.epoch_ms_lo = s_epoch_valid
        ? s_epoch_ms_lo + (now_ms() - s_epoch_ref_ms)
        : 0u;
    sync.seq = s_time_seq++;
    sync.rsvd = 0;
    portEXIT_CRITICAL(&s_time_lock);

    ps_can_frame_t f = {
        .id = ps_can_id_pack(PS_CLS_MGMT, PS_NODE_HUB, PS_NODE_BROADCAST,
                             PS_T_TIME_SYNC, 0),
        .dlc = sizeof(sync),
    };
    memset(f.data, 0, sizeof(f.data));
    PS_WIRE_WRITE(f.data, sync);
    hub_forward(PS_PORT_LOCAL, &f,
                PS_PORT_BIT(PS_PORT_CAN1) | PS_PORT_BIT(PS_PORT_CAN2));
}

/* ---------------- init ---------------- */

esp_err_t hub_gateway_start(ps_can_handle_t can1, ps_can_handle_t can2)
{
    if (can1 == NULL || can2 == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_can[0] = can1;
    s_can[1] = can2;

    s_route_q = xQueueCreate(GW_ROUTE_QUEUE_LEN, sizeof(gw_item_t));
    if (s_route_q == NULL) {
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&s_mirror_lock);
    ps_safety_core_init(&s_mirror, now_ms(), mirror_noop_cb, NULL);
    portEXIT_CRITICAL(&s_mirror_lock);

    /* Promiscuous gateway: every class from both buses lands in the pump. */
    for (uint8_t cls = PS_CLS_SAFETY; cls <= PS_CLS_MGMT; cls++) {
        esp_err_t err = ps_can_register_class_cb(s_can[0], cls, gw_rx_cb,
                                                 (void *)(uintptr_t)PS_PORT_CAN1);
        if (err != ESP_OK) {
            return err;
        }
        err = ps_can_register_class_cb(s_can[1], cls, gw_rx_cb,
                                       (void *)(uintptr_t)PS_PORT_CAN2);
        if (err != ESP_OK) {
            return err;
        }
    }

    BaseType_t ok = xTaskCreatePinnedToCore(gw_pump_task, "hub_gw", 4096, NULL,
                                            GW_PUMP_PRIO, NULL, GW_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ok = xTaskCreatePinnedToCore(gw_downlink_task, "hub_dnl", 4096, NULL,
                                 GW_DOWNLINK_PRIO, NULL, GW_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "gateway up: CAN1(tx%u/rx%u) CAN2(tx%u/rx%u) promiscuous",
             HUB_CAN1_TX_GPIO, HUB_CAN1_RX_GPIO, HUB_CAN2_TX_GPIO, HUB_CAN2_RX_GPIO);
    return ESP_OK;
}
