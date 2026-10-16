/* ps_can — TWAI transport on the IDF 5.5 node-based driver (esp_driver_twai).
 * One implementation serves ESP32-S3 (one controller) and ESP32-P4 (the hub opens
 * two); the driver allocates controllers in call order, so `controller` in the
 * config is board bookkeeping and is only validated/logged here.
 *
 * Three details drive the design:
 *   - twai_node_transmit() does NOT copy the payload, so every in-flight frame
 *     needs storage that outlives the call: hence the TX slot pool, recycled by
 *     the tx-done ISR.
 *   - The driver's own transmit queue is FIFO (esp_twai_onchip.c, IDF 5.5.5), so
 *     a SAFETY frame queued behind the node's own telemetry would not reach
 *     arbitration until all of it had gone. Pending frames therefore wait in
 *     ps_can_txq, ordered by class, and the driver holds at most one at a time,
 *     fed by tx_task. One submitter means no two contexts ever race on
 *     twai_node_transmit().
 *   - All driver callbacks run in ISR context, but ps_can.h promises callers a
 *     task context, so everything is funnelled through a queue into rx_task.
 */
#include "ps_can.h"
#include "ps_can_txq.h"

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

#define PS_CAN_TX_SLOTS        16       /* == PS_CAN_TXQ_DEPTH: one class can hold them all */
#define PS_CAN_DRV_TX_DEPTH    2        /* driver queue; see tx_task for why not 1 */
#define PS_CAN_RX_TASK_STACK   3072
#define PS_CAN_RX_TASK_PRIO    10
#define PS_CAN_TX_TASK_STACK   3072     /* the driver logs on a rejected frame */
#define PS_CAN_TX_TASK_PRIO    12       /* above rx: a SAFETY frame must not wait on RX work */
#define PS_CAN_TASK_CORE       0        /* comms core, per CLAUDE.md */
#define PS_CAN_RECOVER_MIN_MS  100u
#define PS_CAN_RECOVER_MAX_MS  1600u
#define PS_CAN_CLASS_COUNT     8

_Static_assert(PS_CAN_TX_SLOTS <= PS_CAN_TXQ_DEPTH, "every slot must fit one class's ring");

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

    /* Guarded by `lock` (taken from tasks and from the tx-done ISR). */
    portMUX_TYPE  lock;
    ps_can_txq_t  txq;           /* slot indices waiting, by class */
    bool          tx_busy;       /* one slot is with the driver */
    uint8_t       tx_slot;       /* ... and this is it */

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
    TaskHandle_t   tx_task;
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

static void release_slot(struct ps_can_ctx *ctx, uint8_t slot)
{
    xQueueSend(ctx->free_slots, &slot, 0);
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

    if (edata == NULL || edata->done_tx_frame == NULL) {
        return false;
    }
    /* Only the frame we handed over counts. A report for anything else (a frame
     * already reclaimed at bus-off) is ignored, so a slot is never freed twice. */
    bool mine = false;
    uint8_t slot = 0;
    portENTER_CRITICAL_ISR(&ctx->lock);
    if (ctx->tx_busy && edata->done_tx_frame == &ctx->slots[ctx->tx_slot].frame) {
        ctx->tx_busy = false;
        slot = ctx->tx_slot;
        mine = true;
    }
    portEXIT_CRITICAL_ISR(&ctx->lock);
    if (!mine) {
        return false;
    }

    xQueueSendFromISR(ctx->free_slots, &slot, &hp);
    if (edata->is_tx_success) {
        ctx->stats.tx_frames++;
    } else {
        ctx->stats.bus_errors++;
    }
    /* The driver runs this callback before it looks at its own queue, in the same
     * interrupt; tx_task's next frame starts as soon as it is submitted. */
    vTaskNotifyGiveFromISR(ctx->tx_task, &hp);
    return hp == pdTRUE;
}

static bool on_state_change(twai_node_handle_t handle,
                            const twai_state_change_event_data_t *edata, void *user_ctx)
{
    (void)handle;
    struct ps_can_ctx *ctx = (struct ps_can_ctx *)user_ctx;
    BaseType_t hp = pdFALSE;
    if (edata != NULL) {
        int mapped = map_error_state(edata->new_sta);
        ctx->link_state = mapped;
        ctx->pending_state = mapped;   /* reported from task context */
        if (mapped == PS_CAN_STATE_BUS_OFF && ctx->tx_task != NULL) {
            vTaskNotifyGiveFromISR(ctx->tx_task, &hp);   /* flush promptly */
        }
    }
    return hp == pdTRUE;
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

/* ---------------- TX task ---------------- */

/* Nothing queued survives a bus-off. The controller aborts the frame in flight,
 * and the driver's recovery path restarts only from its own queue
 * (esp_twai_onchip.c, IDF 5.5.5), so that frame is never reported done: reclaim it
 * here. Waiting frames are dropped rather than sent after recovery — a heartbeat
 * leaving 100+ ms after it was made is a replayed beat (CLAUDE.md invariant).
 * Counted as tx_timeouts: accepted, never sent.
 *
 * One residual window, not closed: a frame parked in the driver's 2-deep queue
 * (tx_task's cross-core case, microseconds) at the instant of bus-off would be
 * re-sent by the driver after recovery from a slot already reclaimed here. It needs
 * both coincidences at once; if bench evidence ever shows it, close it by draining
 * the driver with twai_node_disable()/enable() here. */
static void tx_flush(struct ps_can_ctx *ctx)
{
    for (;;) {
        int slot;
        portENTER_CRITICAL(&ctx->lock);
        slot = ps_can_txq_pop(&ctx->txq);
        if (slot < 0 && ctx->tx_busy) {
            slot = ctx->tx_slot;
            ctx->tx_busy = false;
        }
        portEXIT_CRITICAL(&ctx->lock);
        if (slot < 0) {
            return;
        }
        release_slot(ctx, (uint8_t)slot);
        ctx->stats.tx_timeouts++;
    }
}

/* The only caller of twai_node_transmit(). It hands over one frame and waits for
 * on_tx_done before the next, so the driver's FIFO never holds more than the frame
 * on the wire. The driver queue is 2 deep, not 1, for the window in which this
 * task (core 0) submits before the TX ISR, possibly on the other core, has marked
 * the hardware idle: the driver then parks the frame and that same ISR starts it. */
static void tx_task(void *arg)
{
    struct ps_can_ctx *ctx = (struct ps_can_ctx *)arg;

    while (ctx->running) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        if (!ctx->running) {
            break;
        }
        if (ctx->link_state == PS_CAN_STATE_BUS_OFF) {
            tx_flush(ctx);
            continue;
        }
        for (;;) {
            int slot = -1;
            portENTER_CRITICAL(&ctx->lock);
            if (!ctx->tx_busy) {
                slot = ps_can_txq_pop(&ctx->txq);
                if (slot >= 0) {
                    ctx->tx_busy = true;
                    ctx->tx_slot = (uint8_t)slot;
                }
            }
            portEXIT_CRITICAL(&ctx->lock);
            if (slot < 0) {
                break;               /* idle, or a frame is already on the wire */
            }
            if (twai_node_transmit(ctx->node, &ctx->slots[slot].frame, 0) == ESP_OK) {
                break;               /* on_tx_done wakes us for the next */
            }
            /* Refused (bus-off between the check and the call, or recovery still
             * under way): this frame will never be reported, so drop it and try the
             * next, which meets the same fate until the link is back. */
            portENTER_CRITICAL(&ctx->lock);
            ctx->tx_busy = false;
            portEXIT_CRITICAL(&ctx->lock);
            release_slot(ctx, (uint8_t)slot);
            ctx->stats.tx_timeouts++;
        }
    }
    vTaskDelete(NULL);
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
    portMUX_INITIALIZE(&ctx->lock);
    ps_can_txq_init(&ctx->txq);

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
    /* Not cfg->tx_queue_len: pending frames wait in ps_can_txq, in class order,
     * and the driver only ever holds the one being sent (see tx_task). */
    nc.tx_queue_depth = PS_CAN_DRV_TX_DEPTH;
    nc.intr_priority = 0;
    /* Bounded retries: a node that cannot get on the bus must not monopolise it
     * forever — the safety watchdog is the backstop, not infinite retransmission. */
    nc.fail_retry_cnt = 3;
    nc.flags.enable_self_test = cfg->loopback;
    nc.flags.enable_loopback = cfg->loopback;

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

    /* tx_task exists before the node is enabled: the callbacks notify it. */
    ctx->running = true;
    if (xTaskCreatePinnedToCore(tx_task, "ps_can_tx", PS_CAN_TX_TASK_STACK, ctx,
                                PS_CAN_TX_TASK_PRIO, &ctx->tx_task,
                                PS_CAN_TASK_CORE) != pdPASS) {
        ctx->running = false;
        err = ESP_ERR_NO_MEM;
        goto fail_node;
    }

    err = twai_node_enable(ctx->node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_enable failed: %s", esp_err_to_name(err));
        goto fail_tx;
    }

    if (xTaskCreatePinnedToCore(rx_task, "ps_can_rx", PS_CAN_RX_TASK_STACK, ctx,
                                PS_CAN_RX_TASK_PRIO, &ctx->rx_task,
                                PS_CAN_TASK_CORE) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto fail_enabled;
    }

    ESP_LOGI(TAG, "controller %d up: %" PRIu32 " bps, tx=%d rx=%d, node_id=%u%s%s",
             cfg->controller, nc.bit_timing.bitrate, cfg->tx_gpio, cfg->rx_gpio,
             (unsigned)cfg->node_id, cfg->node_id == 0 ? " (promiscuous)" : "",
             cfg->loopback ? " LOOPBACK SELF-TEST" : "");
    *out = ctx;
    return ESP_OK;

fail_enabled:
    twai_node_disable(ctx->node);
fail_tx:
    ctx->running = false;
    xTaskNotifyGive(ctx->tx_task);
    vTaskDelay(pdMS_TO_TICKS(40));   /* let tx_task observe the flag and exit */
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
    xTaskNotifyGive(h->tx_task);
    vTaskDelay(pdMS_TO_TICKS(40));   /* let both tasks observe the flag and exit */
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
    /* What the driver would answer, said up front: nothing is accepted for later. */
    if (h->link_state == PS_CAN_STATE_BUS_OFF) {
        return ESP_ERR_INVALID_STATE;
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

    /* Cannot fail while every class ring holds the whole slot pool (asserted
     * above); checked anyway so a future resize fails loudly, not silently. */
    portENTER_CRITICAL(&h->lock);
    bool queued = ps_can_txq_push(&h->txq, ps_can_id_cls(frame->id), slot);
    portEXIT_CRITICAL(&h->lock);
    if (!queued) {
        release_slot(h, slot);
        h->stats.tx_timeouts++;
        return ESP_ERR_NO_MEM;
    }
    xTaskNotifyGive(h->tx_task);
    return ESP_OK;
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
