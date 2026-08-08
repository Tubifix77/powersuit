/* ps_safety_esp — wiring between the pure safety core and the suit's CAN plane.
 * This file adds no rules: it decodes SAFETY frames, ticks the core, and exposes
 * the actuation gate. Every transition decision lives in ps_safety_core.c. */
#include "ps_safety.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "ps_can.h"
#include "powersuit_proto/can_id.h"

#include <inttypes.h>
#include <string.h>

static const char *TAG = "ps_safety";

#define PS_SAFETY_TICK_US  5000   /* 5 ms: 10x finer than the 50 ms watchdog */

static struct {
    ps_safety_core_t core;
    ps_can_handle_t  can;
    uint8_t          node_id;
    SemaphoreHandle_t lock;
    esp_timer_handle_t tick_timer;
    ps_safety_transition_cb_t user_cb;
    void *user_arg;
    uint8_t safety_low;      /* rolling counter for outgoing SAFETY frame IDs */
    uint16_t estop_seq;
    bool started;
} s;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* Runs with the lock held; the user callback is invoked outside it by callers
 * that can afford to, but transitions originate deep inside the core, so it is
 * kept simple: user code must not call back into ps_safety_* from here. */
static void on_core_transition(uint8_t old_state, uint8_t new_state, uint8_t cause, void *arg)
{
    (void)arg;
    ESP_LOGW(TAG, "state %u -> %u (cause %u)", old_state, new_state, cause);
    if (s.user_cb) {
        s.user_cb(old_state, new_state, cause, s.user_arg);
    }
}

static esp_err_t send_safety(uint8_t type, const void *payload, uint8_t len)
{
    ps_can_frame_t f;
    f.id = ps_can_id_pack(PS_CLS_SAFETY, s.node_id, PS_NODE_BROADCAST, type, s.safety_low++);
    f.dlc = len;
    memset(f.data, 0, sizeof(f.data));
    if (payload && len) {
        memcpy(f.data, payload, len);
    }
    return ps_can_send(s.can, &f, pdMS_TO_TICKS(5));
}

static void safety_rx(const ps_can_frame_t *frame, void *arg)
{
    (void)arg;
    uint32_t t = now_ms();
    uint8_t type = ps_can_id_type(frame->id);

    xSemaphoreTake(s.lock, portMAX_DELAY);
    switch (type) {
    case PS_T_HEARTBEAT: {
        ps_heartbeat_t hb;
        PS_WIRE_READ(hb, frame->data);
        ps_safety_core_on_heartbeat(&s.core, t, &hb);
        break;
    }
    case PS_T_ESTOP: {
        ps_estop_t e;
        PS_WIRE_READ(e, frame->data);
        ps_safety_core_on_estop(&s.core, t, e.cause);
        break;
    }
    case PS_T_CLEAR_ESTOP: {
        ps_clear_estop_t c;
        PS_WIRE_READ(c, frame->data);
        if (!ps_safety_core_on_clear(&s.core, t, c.magic, c.counter)) {
            ESP_LOGW(TAG, "rejected CLEAR_ESTOP (magic %08" PRIx32 ", counter %" PRIu32 ")",
                     c.magic, c.counter);
        }
        break;
    }
    default:
        break;   /* NODE_FAULT from peers is informational for a limb node */
    }
    xSemaphoreGive(s.lock);
}

static void tick_cb(void *arg)
{
    (void)arg;
    xSemaphoreTake(s.lock, portMAX_DELAY);
    ps_safety_core_tick(&s.core, now_ms());
    xSemaphoreGive(s.lock);
}

static void can_state_cb(int state, void *arg)
{
    (void)arg;
    if (state == PS_CAN_STATE_BUS_OFF || state == PS_CAN_STATE_ERR_PASSIVE) {
        xSemaphoreTake(s.lock, portMAX_DELAY);
        ps_safety_core_on_bus_down(&s.core, now_ms());
        xSemaphoreGive(s.lock);
    }
}

esp_err_t ps_safety_start(const ps_safety_config_t *cfg)
{
    if (cfg == NULL || cfg->can == NULL || s.started) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(&s, 0, sizeof(s));
    s.can = (ps_can_handle_t)cfg->can;
    s.node_id = cfg->node_id;
    s.user_cb = cfg->on_transition;
    s.user_arg = cfg->arg;

    s.lock = xSemaphoreCreateMutex();
    if (s.lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ps_safety_core_init(&s.core, now_ms(), on_core_transition, NULL);

    esp_err_t err = ps_can_register_class_cb(s.can, PS_CLS_SAFETY, safety_rx, NULL);
    if (err != ESP_OK) {
        return err;
    }
    err = ps_can_register_state_cb(s.can, can_state_cb, NULL);
    if (err != ESP_OK) {
        return err;
    }

    const esp_timer_create_args_t targs = {
        .callback = tick_cb,
        .name = "ps_safety_tick",
        .dispatch_method = ESP_TIMER_TASK,
    };
    err = esp_timer_create(&targs, &s.tick_timer);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_timer_start_periodic(s.tick_timer, PS_SAFETY_TICK_US);
    if (err != ESP_OK) {
        return err;
    }

    s.started = true;
    ESP_LOGI(TAG, "armed for node %u (watchdog %ums, re-arm %ums)", (unsigned)s.node_id,
             (unsigned)PS_HEARTBEAT_TIMEOUT_MS, (unsigned)PS_REARM_WINDOW_MS);
    return ESP_OK;
}

uint8_t ps_safety_state(void)
{
    return s.started ? s.core.state : PS_STATE_BOOT;
}

bool ps_safety_can_actuate(void)
{
    /* The single gate every actuation write goes through. */
    return s.started && s.core.state == PS_STATE_OPERATIONAL;
}

bool ps_safety_cmd_fresh(void)
{
    return s.started && ps_safety_core_cmd_fresh(&s.core, now_ms());
}

void ps_safety_note_cmd(void)
{
    if (!s.started) {
        return;
    }
    xSemaphoreTake(s.lock, portMAX_DELAY);
    ps_safety_core_on_cmd(&s.core, now_ms());
    xSemaphoreGive(s.lock);
}

void ps_safety_note_mode_set(uint8_t target_state)
{
    if (!s.started) {
        return;
    }
    xSemaphoreTake(s.lock, portMAX_DELAY);
    ps_safety_core_on_mode_set(&s.core, now_ms(), target_state);
    xSemaphoreGive(s.lock);
}

esp_err_t ps_safety_raise_estop(uint8_t cause)
{
    if (!s.started) {
        return ESP_ERR_INVALID_STATE;
    }
    ps_estop_t e;
    memset(&e, 0, sizeof(e));
    e.cause = cause;
    e.origin_node = s.node_id;
    e.seq = ++s.estop_seq;
    e.uptime_ms = now_ms();

    /* Repeated, unacknowledged: three arbitration-priority frames make loss
     * negligible on a bus that never exceeds ~42% load (docs/safety.md §3). */
    esp_err_t first = ESP_OK;
    for (unsigned i = 0; i < PS_ESTOP_REPEAT; i++) {
        esp_err_t err = send_safety(PS_T_ESTOP, &e, (uint8_t)sizeof(e));
        if (i == 0) {
            first = err;
        }
    }

    xSemaphoreTake(s.lock, portMAX_DELAY);
    ps_safety_core_on_estop(&s.core, now_ms(), cause);
    xSemaphoreGive(s.lock);
    return first;
}

esp_err_t ps_safety_send_fault(uint8_t fault_code, uint8_t severity, uint16_t detail)
{
    if (!s.started) {
        return ESP_ERR_INVALID_STATE;
    }
    ps_node_fault_t f;
    memset(&f, 0, sizeof(f));
    f.fault_code = fault_code;
    f.severity = severity;
    f.detail = detail;
    f.uptime_ms = now_ms();
    return send_safety(PS_T_NODE_FAULT, &f, (uint8_t)sizeof(f));
}
