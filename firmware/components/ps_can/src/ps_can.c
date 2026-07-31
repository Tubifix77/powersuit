/* ps_can — TWAI transport on the IDF 5.5 node-based driver (esp_driver_twai).
 * One implementation serves ESP32-S3 (one controller) and ESP32-P4 (the hub opens
 * two); the driver allocates controllers in call order, so `controller` in the
 * config is board bookkeeping and is only validated/logged here.
 *
 * Two details drive the design:
 *   - twai_node_transmit() does NOT copy the payload, so every in-flight frame
 *     needs storage that outlives the call: hence the TX slot pool, recycled by
 *     the tx-done ISR.
 *   - All driver callbacks run in ISR context, but ps_can.h promises callers a
 *     task context, so everything is funnelled through a queue into rx_task.
 */
#include "ps_can.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "ps_can";

#define PS_CAN_TX_SLOTS        16
#define PS_CAN_RX_TASK_STACK   3072
#define PS_CAN_RX_TASK_PRIO    10
#define PS_CAN_RX_TASK_CORE    0        /* comms core, per CLAUDE.md */
#define PS_CAN_RECOVER_MIN_MS  100u
#define PS_CAN_RECOVER_MAX_MS  1600u
#define PS_CAN_CLASS_COUNT     8

typedef struct {
    twai_frame_t frame;
    uint8_t data[8];
} ps_can_tx_slot_t;

struct ps_can_ctx {
    twai_node_handle_t node;
    uint8_t  node_id;
    int      controller;

    QueueHandle_t rx_q;          /* ps_can_frame_t */
    QueueHandle_t free_slots;    /* uint8_t slot indices */
    ps_can_tx_slot_t slots[PS_CAN_TX_SLOTS];

    ps_can_rx_cb_t cls_cb[PS_CAN_CLASS_COUNT];
    void          *cls_arg[PS_CAN_CLASS_COUNT];
    ps_can_state_cb_t state_cb;
    void             *state_arg;

    volatile int  link_state;
    volatile int  pending_state;   /* -1 = nothing to report */
    uint32_t      recover_backoff_ms;
    int64_t       recover_at_ms;

    ps_can_stats_t stats;
    TaskHandle_t   rx_task;
    volatile bool  running;
};

static int map_error_state(twai_error_state_t s)
{
    switch (s) {
    case TWAI_ERROR_BUS_OFF:
        return PS_CAN_STATE_BUS_OFF;
    case TWAI_ERROR_PASSIVE:
        return PS_CAN_STATE_ERR_PASSIVE;
    default:
        return PS_CAN_STATE_RUNNING;   /* ACTIVE and WARNING are both usable */
    }
}

/* ---------------- ISR callbacks ---------------- */

static bool on_rx_done(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata,
                       void *user_ctx)
{
    (void)handle;
    (void)edata;
    struct ps_can_ctx *ctx = (struct ps_can_ctx *)user_ctx;
    /* Zeroed deliberately: the driver fills only the bytes it received, and a
     * frame whose DLC overstates that would otherwise put uninitialised stack
     * contents onto the CAN plane. Eight bytes in an ISR is free. */
    uint8_t buf[8] = { 0 };
    twai_frame_t rx = { 0 };
    rx.buffer = buf;
    rx.buffer_len = sizeof(buf);

    BaseType_t hp = pdFALSE;
    if (twai_node_receive_from_isr(ctx->node, &rx) == ESP_OK) {
        ps_can_frame_t f;
        uint16_t len = twaifd_dlc2len(rx.header.dlc);
        if (len > 8) {
            len = 8;
        }
        f.id = rx.header.id;
        f.dlc = (uint8_t)len;
        memcpy(f.data, buf, len);
        if (len < 8) {
            memset(f.data + len, 0, 8u - len);
        }
        ctx->stats.rx_frames++;
        if (xQueueSendFromISR(ctx->rx_q, &f, &hp) != pdTRUE) {
            ctx->stats.rx_dropped++;
        }
    }
    return hp == pdTRUE;
}

static bool on_tx_done(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata,
                       void *user_ctx)
{
    (void)handle;
    struct ps_can_ctx *ctx = (struct ps_can_ctx *)user_ctx;
    BaseType_t hp = pdFALSE;

    if (edata != NULL && edata->done_tx_frame != NULL) {
        for (int i = 0; i < PS_CAN_TX_SLOTS; i++) {
            if (&ctx->slots[i].frame == edata->done_tx_frame) {
                uint8_t slot = (uint8_t)i;
                xQueueSendFromISR(ctx->free_slots, &slot, &hp);
                break;
            }
        }
        if (edata->is_tx_success) {
            ctx->stats.tx_frames++;
        } else {
            ctx->stats.bus_errors++;
        }
    }
    return hp == pdTRUE;
}

static bool on_state_change(twai_node_handle_t handle,
                            const twai_state_change_event_data_t *edata, void *user_ctx)
{
    (void)handle;
    struct ps_can_ctx *ctx = (struct ps_can_ctx *)user_ctx;
    if (edata != NULL) {
        int mapped = map_error_state(edata->new_sta);
        ctx->link_state = mapped;
        ctx->pending_state = mapped;   /* reported from task context */
    }
    return false;
}

static bool on_error(twai_node_handle_t handle, const twai_error_event_data_t *edata,
                     void *user_ctx)
{
    (void)handle;
    (void)edata;
    struct ps_can_ctx *ctx = (struct ps_can_ctx *)user_ctx;
    ctx->stats.bus_errors++;
    return false;
}

/* ---------------- RX / housekeeping task ---------------- */

static void service_link(struct ps_can_ctx *ctx)
{
    int pending = ctx->pending_state;
    if (pending >= 0) {
        ctx->pending_state = -1;
        if (pending == PS_CAN_STATE_RUNNING) {
            ctx->recover_backoff_ms = PS_CAN_RECOVER_MIN_MS;
        }
        if (ctx->state_cb) {
            ctx->state_cb(pending, ctx->state_arg);
        }
    }

    /* Bus-off recovery with backoff. The safety layer does not wait for this:
     * it has already treated bus-off as heartbeat loss (docs/safety.md §2). */
    if (ctx->link_state == PS_CAN_STATE_BUS_OFF) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms >= ctx->recover_at_ms) {
            ESP_LOGW(TAG, "bus-off on controller %d, recovering (backoff %ums)",
                     ctx->controller, (unsigned)ctx->recover_backoff_ms);
            if (twai_node_recover(ctx->node) == ESP_OK) {
                ctx->link_state = PS_CAN_STATE_RECOVERING;
                ctx->stats.recoveries++;
            }
            ctx->recover_at_ms = now_ms + ctx->recover_backoff_ms;
            ctx->recover_backoff_ms *= 2;
            if (ctx->recover_backoff_ms > PS_CAN_RECOVER_MAX_MS) {
                ctx->recover_backoff_ms = PS_CAN_RECOVER_MAX_MS;
            }
        }
    }
}

static void rx_task(void *arg)
{
    struct ps_can_ctx *ctx = (struct ps_can_ctx *)arg;
    ps_can_frame_t f;

    while (ctx->running) {
        if (xQueueReceive(ctx->rx_q, &f, pdMS_TO_TICKS(20)) == pdTRUE) {
            uint8_t dst = ps_can_id_dst(f.id);
            /* node_id 0 = promiscuous (the hub gateway sees everything). */
            if (ctx->node_id == 0 || dst == ctx->node_id || dst == PS_NODE_BROADCAST) {
                uint8_t cls = ps_can_id_cls(f.id);
                ps_can_rx_cb_t cb = ctx->cls_cb[cls];
                if (cb != NULL) {
                    cb(&f, ctx->cls_arg[cls]);
                } else {
                    ctx->stats.rx_dropped++;
                }
            }
        }
        service_link(ctx);
    }
    vTaskDelete(NULL);
}

/* ---------------- public API ---------------- */

esp_err_t ps_can_open(const ps_can_config_t *cfg, ps_can_handle_t *out)
{
    if (cfg == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    struct ps_can_ctx *ctx = calloc(1, sizeof(*ctx));
    if (ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ctx->node_id = cfg->node_id;
    ctx->controller = cfg->controller;
    ctx->link_state = PS_CAN_STATE_RUNNING;
    ctx->pending_state = -1;
    ctx->recover_backoff_ms = PS_CAN_RECOVER_MIN_MS;

    size_t rx_len = cfg->rx_queue_len ? cfg->rx_queue_len : 64;
    ctx->rx_q = xQueueCreate(rx_len, sizeof(ps_can_frame_t));
    ctx->free_slots = xQueueCreate(PS_CAN_TX_SLOTS, sizeof(uint8_t));
    if (ctx->rx_q == NULL || ctx->free_slots == NULL) {
        goto fail;
    }
    for (uint8_t i = 0; i < PS_CAN_TX_SLOTS; i++) {
        xQueueSend(ctx->free_slots, &i, 0);
    }

    twai_onchip_node_config_t nc = { 0 };
    nc.io_cfg.tx = cfg->tx_gpio;
    nc.io_cfg.rx = cfg->rx_gpio;
    nc.io_cfg.quanta_clk_out = -1;
    nc.io_cfg.bus_off_indicator = -1;
    nc.bit_timing.bitrate = cfg->bitrate ? cfg->bitrate : 1000000;
    nc.tx_queue_depth = cfg->tx_queue_len ? cfg->tx_queue_len : PS_CAN_TX_SLOTS;
    nc.intr_priority = 0;
    /* Bounded retries: a node that cannot get on the bus must not monopolise it
     * forever — the safety watchdog is the backstop, not infinite retransmission. */
    nc.fail_retry_cnt = 3;

    esp_err_t err = twai_new_node_onchip(&nc, &ctx->node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_new_node_onchip failed: %s", esp_err_to_name(err));
        goto fail;
    }

    twai_event_callbacks_t cbs = {
        .on_tx_done = on_tx_done,
        .on_rx_done = on_rx_done,
        .on_state_change = on_state_change,
        .on_error = on_error,
    };
    err = twai_node_register_event_callbacks(ctx->node, &cbs, ctx);
    if (err != ESP_OK) {
        goto fail_node;
    }

    /* No hardware acceptance filter is installed: the software filter in rx_task
     * is authoritative, and the hub needs promiscuous reception anyway. */

    err = twai_node_enable(ctx->node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_enable failed: %s", esp_err_to_name(err));
        goto fail_node;
    }

    ctx->running = true;
    if (xTaskCreatePinnedToCore(rx_task, "ps_can_rx", PS_CAN_RX_TASK_STACK, ctx,
                                PS_CAN_RX_TASK_PRIO, &ctx->rx_task,
                                PS_CAN_RX_TASK_CORE) != pdPASS) {
        ctx->running = false;
        err = ESP_ERR_NO_MEM;
        goto fail_enabled;
    }

    ESP_LOGI(TAG, "controller %d up: %" PRIu32 " bps, tx=%d rx=%d, node_id=%u%s",
             cfg->controller, nc.bit_timing.bitrate, cfg->tx_gpio, cfg->rx_gpio,
             (unsigned)cfg->node_id, cfg->node_id == 0 ? " (promiscuous)" : "");
    *out = ctx;
    return ESP_OK;

fail_enabled:
    twai_node_disable(ctx->node);
fail_node:
    twai_node_delete(ctx->node);
fail:
    if (ctx->rx_q) {
        vQueueDelete(ctx->rx_q);
    }
    if (ctx->free_slots) {
        vQueueDelete(ctx->free_slots);
    }
    free(ctx);
    return ESP_FAIL;
}

esp_err_t ps_can_close(ps_can_handle_t h)
{
    if (h == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    h->running = false;
    vTaskDelay(pdMS_TO_TICKS(40));   /* let rx_task observe the flag and exit */
    twai_node_disable(h->node);
    twai_node_delete(h->node);
    vQueueDelete(h->rx_q);
    vQueueDelete(h->free_slots);
    free(h);
    return ESP_OK;
}

esp_err_t ps_can_send(ps_can_handle_t h, const ps_can_frame_t *frame, TickType_t timeout)
{
    if (h == NULL || frame == NULL || frame->dlc > 8 || (frame->id & ~PS_CAN_ID_MASK)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t slot;
    if (xQueueReceive(h->free_slots, &slot, timeout) != pdTRUE) {
        h->stats.tx_timeouts++;
        return ESP_ERR_TIMEOUT;
    }

    ps_can_tx_slot_t *s = &h->slots[slot];
    memcpy(s->data, frame->data, frame->dlc);
    memset(&s->frame.header, 0, sizeof(s->frame.header));
    s->frame.header.id = frame->id;
    s->frame.header.dlc = twaifd_len2dlc(frame->dlc);
    s->frame.header.ide = 1;   /* the whole contract is 29-bit extended */
    s->frame.buffer = s->data;
    s->frame.buffer_len = frame->dlc;

    esp_err_t err = twai_node_transmit(h->node, &s->frame, 0);
    if (err != ESP_OK) {
        xQueueSend(h->free_slots, &slot, 0);
        if (err == ESP_ERR_TIMEOUT) {
            h->stats.tx_timeouts++;
        }
    }
    return err;
}

esp_err_t ps_can_register_class_cb(ps_can_handle_t h, uint8_t cls, ps_can_rx_cb_t cb, void *arg)
{
    if (h == NULL || cls >= PS_CAN_CLASS_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    h->cls_cb[cls] = cb;
    h->cls_arg[cls] = arg;
    return ESP_OK;
}

esp_err_t ps_can_register_state_cb(ps_can_handle_t h, ps_can_state_cb_t cb, void *arg)
{
    if (h == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    h->state_cb = cb;
    h->state_arg = arg;
    return ESP_OK;
}

int ps_can_state(ps_can_handle_t h)
{
    return h ? h->link_state : PS_CAN_STATE_BUS_OFF;
}

void ps_can_get_stats(ps_can_handle_t h, ps_can_stats_t *out)
{
    if (h != NULL && out != NULL) {
        *out = h->stats;
    }
}
