/* comms_task — Node 5 comms, core 0 (ARCHITECTURE.md §2: comms off the control
 * core). Owns the CAN handle, the CONTROL-class RX path, runtime parameters
 * (rho / rate_scale), and the TELEM/MGMT emit helper used by aero_task.c and
 * this file. The 100 Hz FOC-class loop budget does not apply here — Node 5 has
 * no FOC; this core carries CONTROL ingest + 1 Hz housekeeping + (optionally)
 * the XRCE parameter/alive plane.
 *
 * FLAP_CMD -> flight_targets_set(): the "already clamped to +-1000 at ingest"
 * contract in node_flight.h is enforced HERE, before the target buffer is
 * touched; servo_task.c's per-flap envelope/slew clamp (docs/safety.md §5.1)
 * is the second, authoritative clamp immediately before the PWM write. Two
 * clamps, two purposes: this one rejects a malformed wire value, that one
 * enforces the mechanical envelope. */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "powersuit_proto/can_id.h"
#include "powersuit_proto/wire.h"
#include "ps_can.h"
#include "ps_safety.h"
#include "node_flight.h"

static const char *TAG = "node_flight";

#define COMMS_HOUSEKEEP_PERIOD_MS 1000u   /* 1 Hz NODE_STATS + VERSION */

/* ---- CAN handle (set once at boot by app_main) ---- */
static ps_can_handle_t s_can;

ps_can_handle_t flight_can(void)
{
    return s_can;
}

void flight_set_can(ps_can_handle_t can)
{
    s_can = can;
}

/* ---- TELEM/MGMT emit: per-type sequence in the id low byte (network-map §2/§3) ---- */
static uint8_t s_telem_seq[256];

void flight_telem_send(uint8_t cls, uint8_t type, const void *payload, uint8_t len)
{
    ps_can_handle_t can = s_can;
    if (can == NULL) {
        return;
    }
    ps_can_frame_t f;
    f.id = ps_can_id_pack(cls, PS_NODE_FLIGHT, PS_NODE_ORCH, type, s_telem_seq[type]++);
    f.dlc = len;
    memset(f.data, 0, sizeof(f.data));
    if (payload != NULL && len != 0) {
        memcpy(f.data, payload, len > sizeof(f.data) ? sizeof(f.data) : len);
    }
    /* TELEM is best-effort at this rate; a short, non-blocking timeout keeps a
     * momentarily full TX queue from stalling the caller (aero_task at 100 Hz). */
    (void)ps_can_send(can, &f, pdMS_TO_TICKS(2));
}

/* ---- runtime parameters (XRCE set_parameters service; §7) ---- */
static portMUX_TYPE s_param_lock = portMUX_INITIALIZER_UNLOCKED;
static float s_rho = PS_RHO_DEFAULT;
static float s_rate_scale = 1.0f;

float flight_param_rho(void)
{
    taskENTER_CRITICAL(&s_param_lock);
    float v = s_rho;
    taskEXIT_CRITICAL(&s_param_lock);
    return v;
}

float flight_param_rate_scale(void)
{
    taskENTER_CRITICAL(&s_param_lock);
    float v = s_rate_scale;
    taskEXIT_CRITICAL(&s_param_lock);
    return v;
}

bool flight_param_set(const char *name, double value)
{
    if (name == NULL) {
        return false;
    }
    if (strcmp(name, "rho") == 0) {
        if (value < 0.5 || value > 2.5) { /* sane air-density bracket, sea level to thin */
            return false;
        }
        taskENTER_CRITICAL(&s_param_lock);
        s_rho = (float)value;
        taskEXIT_CRITICAL(&s_param_lock);
        return true;
    }
    if (strcmp(name, "rate_scale") == 0) {
        if (value < 0.05 || value > 1.0) { /* node_flight.h contract range */
            return false;
        }
        taskENTER_CRITICAL(&s_param_lock);
        s_rate_scale = (float)value;
        taskEXIT_CRITICAL(&s_param_lock);
        return true;
    }
    return false;
}

/* ---- CONTROL class RX (network-map §3.2) ---- */
static void control_rx(const ps_can_frame_t *frame, void *arg)
{
    (void)arg;
    uint8_t type = ps_can_id_type(frame->id);

    switch (type) {
    case PS_T_FLAP_CMD: {
        ps_flap_cmd_t cmd;
        if (frame->dlc < sizeof(cmd)) {
            return;
        }
        PS_WIRE_READ(cmd, frame->data);
        if (cmd.flap >= PS_FLAP_COUNT) {
            return; /* malformed index: drop, never index out of the target buffer */
        }
        int16_t pos_pm = cmd.pos_pm;
        if (pos_pm < -1000) {
            pos_pm = -1000;
        } else if (pos_pm > 1000) {
            pos_pm = 1000;
        }
        bool brake = (cmd.flags & 0x0001u) != 0;
        flight_targets_set(cmd.flap, pos_pm, cmd.rate_lim, brake);
        ps_safety_note_cmd();
        break;
    }
    case PS_T_MODE_SET: {
        ps_mode_set_t ms;
        memset(&ms, 0, sizeof(ms));
        memcpy(&ms, frame->data, frame->dlc < sizeof(ms) ? frame->dlc : sizeof(ms));
        /* ps_safety_esp.c (firmware/components/ps_safety) registers only the
         * PS_CLS_SAFETY class on its singleton and exposes no setter that
         * reaches ps_safety_core_t.want_operational; there is no frozen-API
         * hook for an app to drive a MODE_SET into the FSM (unlike node 7,
         * which sidesteps this by owning its own ps_safety_core_t directly).
         * This is a genuine gap in the shared component, out of scope for
         * this app directory -- see report. ps_safety_note_cmd() is still
         * correct: docs/safety.md §2 lists MODE_SET among the "fresh valid
         * command" inputs the PASSIVE -> OPERATIONAL re-arm check requires. */
        ps_safety_note_cmd();
        ESP_LOGW(TAG, "MODE_SET target=%u observed (freshness noted only; "
                      "ps_safety exposes no mode-set hook)", ms.target_state);
        break;
    }
    default:
        break; /* JOINT_CMD/LED_PATTERN make no sense on a flap node: drop */
    }
}

/* ---- 1 Hz housekeeping: NODE_STATS + VERSION ---- */
static void comms_task(void *arg)
{
    (void)arg;
    ps_can_stats_t prev = { 0 };
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(COMMS_HOUSEKEEP_PERIOD_MS));

        ps_can_stats_t cur;
        ps_can_get_stats(s_can, &cur);
        uint32_t rx = cur.rx_frames - prev.rx_frames;
        uint32_t tx = cur.tx_frames - prev.tx_frames;
        uint32_t err = (cur.bus_errors - prev.bus_errors) + (cur.rx_dropped - prev.rx_dropped) +
                       (cur.tx_timeouts - prev.tx_timeouts);
        prev = cur;

        ps_node_stats_t stats = {
            .cpu_pct = 0, /* not instrumented on this node yet */
            .state = ps_safety_state(),
            .rx_fps = (uint16_t)(rx > 0xFFFFu ? 0xFFFFu : rx),
            .tx_fps = (uint16_t)(tx > 0xFFFFu ? 0xFFFFu : tx),
            .err_cnt = (uint16_t)(err > 0xFFFFu ? 0xFFFFu : err),
        };
        flight_telem_send(PS_CLS_TELEM, PS_T_NODE_STATS, &stats, sizeof(stats));

        ps_version_t ver = {
            .major = 0, .minor = 1, .patch = 0,
            .node_state = ps_safety_state(),
            .git_short = 0, /* stamped by the build system once it exists */
        };
        flight_telem_send(PS_CLS_MGMT, PS_T_VERSION, &ver, sizeof(ver));
    }
}

void comms_task_start(ps_can_handle_t can)
{
    s_can = can;
    esp_err_t err = ps_can_register_class_cb(can, PS_CLS_CONTROL, control_rx, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CONTROL class callback register failed: %d", (int)err);
    }
    BaseType_t ok = xTaskCreatePinnedToCore(comms_task, "comms", 4096, NULL, 12, NULL, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "comms task create failed");
    }
}

/* ---- micro-ROS entities (optional; PS_UROS_ENABLED gates the client build) ----
 * §2 rate policy: only low-rate parameters and a liveness heartbeat ride XRCE
 * on this node. FLAP_CMD/FLAP_STATE/AERO_STATE/IMU_* stay on the CONTROL/TELEM
 * planes at their native rates (control_rx above, aero_task.c) -- a full
 * sensor_msgs-class topic at 50-100 Hz over XRCE would blow the CAN2 budget
 * (network-map §10). Unverified in this build: PS_UROS_ENABLED is 0 here
 * (micro-ROS client not vendored), so this block has never been compiled. */
#if PS_UROS_ENABLED

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <std_msgs/msg/bool.h>

static rclc_parameter_server_t s_param_server;
static rcl_publisher_t s_alive_pub;
static rcl_timer_t s_alive_timer;
static std_msgs__msg__Bool s_alive_msg;
static rcl_node_t *s_uros_node;

/* Returning false REJECTS the change (rclc_parameter_callback_t). That is the
 * mechanism by which the network cannot widen a limit the board defines, so
 * this must propagate flight_param_set's verdict rather than swallow it. */
static bool on_param_changed(const Parameter *old_param, const Parameter *new_param,
                             void *context)
{
    (void)old_param;
    (void)context;
    if (new_param == NULL || new_param->value.type != RCLC_PARAMETER_DOUBLE) {
        return false;
    }
    if (!flight_param_set(new_param->name.data, new_param->value.double_value)) {
        ESP_LOGW(TAG, "rejected parameter set: %s", new_param->name.data);
        return false;
    }
    return true;
}

static void alive_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;
    s_alive_msg.data = true;
    (void)rcl_publish(&s_alive_pub, &s_alive_msg, NULL);
}

void flight_uros_create_entities(void *support, void *node, void *executor, void *arg)
{
    (void)arg;
    rclc_support_t *sup = (rclc_support_t *)support;
    rcl_node_t *nd = (rcl_node_t *)node;
    rclc_executor_t *exec = (rclc_executor_t *)executor;
    s_uros_node = nd;

    const rclc_parameter_options_t popts = {
        .notify_changed_over_dds = true,
        .max_params = 2,          /* rho, rate_scale */
        .allow_undeclared_parameters = false,
        .low_mem_mode = false,
    };
    rclc_parameter_server_init_with_option(&s_param_server, nd, &popts);
    rclc_executor_add_parameter_server(exec, &s_param_server, on_param_changed);
    rclc_add_parameter(&s_param_server, "rho", RCLC_PARAMETER_DOUBLE);
    rclc_parameter_set_double(&s_param_server, "rho", (double)flight_param_rho());
    rclc_add_parameter(&s_param_server, "rate_scale", RCLC_PARAMETER_DOUBLE);
    rclc_parameter_set_double(&s_param_server, "rate_scale", (double)flight_param_rate_scale());

    rclc_publisher_init_default(&s_alive_pub, nd,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "~/alive");
    rclc_timer_init_default(&s_alive_timer, sup, RCL_MS_TO_NS(1000), alive_timer_cb);
    rclc_executor_add_timer(exec, &s_alive_timer);
}

void flight_uros_destroy_entities(void *arg)
{
    (void)arg;
    (void)rcl_timer_fini(&s_alive_timer);
    (void)rcl_publisher_fini(&s_alive_pub, s_uros_node);
    (void)rclc_parameter_server_fini(&s_param_server, s_uros_node);
}

#else /* !PS_UROS_ENABLED */

void flight_uros_create_entities(void *support, void *node, void *executor, void *arg)
{
    (void)support;
    (void)node;
    (void)executor;
    (void)arg;
}

void flight_uros_destroy_entities(void *arg)
{
    (void)arg;
}

#endif /* PS_UROS_ENABLED */
