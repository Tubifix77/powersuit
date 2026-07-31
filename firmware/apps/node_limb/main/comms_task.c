/* comms_task — Core 0: 100 Hz TELEM emission, CONTROL-class RX, and (when
 * vendored) the micro-ROS parameter server + alive beacon.
 *
 * Commands and telemetry ride the raw SAFETY/CONTROL/TELEM CAN planes, never
 * XRCE: network-map.md section 2's rate policy caps XRCE to low-rate topics,
 * parameters and services, and JOINT_STATE/IMU/FORCE alone are five 100 Hz
 * streams per limb. This node therefore subscribes to nothing over XRCE; the
 * only entities it ever creates are the parameter server and a 1 Hz uptime
 * publisher (both explicitly low-rate).
 *
 * KNOWN GAP in the shared ps_safety component (reported, not worked around
 * here — see the app's build report): ps_safety.h's ESP glue exposes
 * ps_safety_note_cmd() for command freshness but no public hook to latch
 * ps_safety_core_t.want_operational from a received MODE_SET. The pure core
 * (ps_safety_core_on_mode_set) supports it and the hub's own mirror uses it
 * directly, but the limb app only has the singleton ps_safety_esp.c instance,
 * which never calls it. A limb node built against the current ps_safety
 * therefore cannot programmatically leave STANDBY/PASSIVE for OPERATIONAL;
 * MODE_SET is still applied to command freshness below, which is the one
 * part of the contract reachable through the exposed API. */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "powersuit_proto/can_id.h"
#include "powersuit_proto/wire.h"
#include "ps_can.h"
#include "ps_safety.h"

#include "limb_tasks.h"

static const char *TAG = "node_limb";

#define COMMS_TASK_PERIOD_MS 10u   /* 100 Hz */

static ps_can_handle_t s_can;
static uint8_t s_node_id;
static uint8_t s_seq[256];   /* per-type rolling low byte (network-map.md section 2) */

static portMUX_TYPE s_ctr_lock = portMUX_INITIALIZER_UNLOCKED;
static limb_counters_t s_ctr;

void limb_counters_get(limb_counters_t *out)
{
    portENTER_CRITICAL(&s_ctr_lock);
    *out = s_ctr;
    portEXIT_CRITICAL(&s_ctr_lock);
}

void limb_counters_bump_control_tick(void)
{
    portENTER_CRITICAL(&s_ctr_lock);
    s_ctr.control_ticks++;
    portEXIT_CRITICAL(&s_ctr_lock);
}

void limb_counters_bump_overcurrent(void)
{
    portENTER_CRITICAL(&s_ctr_lock);
    s_ctr.overcurrent_trips++;
    portEXIT_CRITICAL(&s_ctr_lock);
}

static void ctr_bump(uint32_t *field)
{
    portENTER_CRITICAL(&s_ctr_lock);
    (*field)++;
    portEXIT_CRITICAL(&s_ctr_lock);
}

void limb_telem_send(uint8_t cls, uint8_t type, const void *payload, uint8_t len)
{
    ps_can_frame_t f = {
        .id = ps_can_id_pack(cls, s_node_id, PS_NODE_ORCH, type, s_seq[type]++),
        .dlc = len,
    };
    memset(f.data, 0, sizeof(f.data));
    if (payload != NULL && len > 0) {
        memcpy(f.data, payload, len);
    }
    (void)ps_can_send(s_can, &f, pdMS_TO_TICKS(2));
}

/* ---------------- CONTROL-class RX ---------------- */

static void handle_joint_cmd(const ps_can_frame_t *f)
{
    ps_joint_cmd_t cmd;
    if (f->dlc < sizeof(cmd)) {
        ctr_bump(&s_ctr.cmd_rejected);
        return;
    }
    PS_WIRE_READ(cmd, f->data);
    if (cmd.joint >= PS_LIMB_JOINT_COUNT || cmd.mode > PS_JMODE_IMPEDANCE) {
        ctr_bump(&s_ctr.cmd_rejected);
        return;
    }
    const ps_limb_joint_cfg_t *board = &PS_LIMB_JOINTS[cmd.joint];
    int16_t pos = cmd.pos_crad;
    if (pos < board->pos_min_crad) {
        pos = board->pos_min_crad;
    } else if (pos > board->pos_max_crad) {
        pos = board->pos_max_crad;
    }
    limb_setpoint_t sp = {
        .mode = cmd.mode,
        .pos_crad = pos,
        .vel_crad_s = cmd.vel_crad_s,
        .eff_cNm = cmd.eff_cNm,
    };
    limb_setpoint_set(cmd.joint, &sp);
    ps_safety_note_cmd();
    ctr_bump(&s_ctr.joint_cmd_rx);
}

static void handle_mode_set(const ps_can_frame_t *f)
{
    ps_mode_set_t ms;
    memset(&ms, 0, sizeof(ms));
    memcpy(&ms, f->data, f->dlc < sizeof(ms) ? f->dlc : sizeof(ms));
    /* MODE_SET counts toward command freshness (network-map.md section 4
     * re-arm list); latching want_operational itself is the documented gap
     * noted at the top of this file. */
    ps_safety_note_cmd();
    ctr_bump(&s_ctr.mode_set_rx);
    ESP_LOGD(TAG, "MODE_SET target=%u observed", (unsigned)ms.target_state);
}

static void control_rx_cb(const ps_can_frame_t *frame, void *arg)
{
    (void)arg;
    switch (ps_can_id_type(frame->id)) {
    case PS_T_JOINT_CMD:
        handle_joint_cmd(frame);
        break;
    case PS_T_MODE_SET:
        handle_mode_set(frame);
        break;
    case PS_T_LED_PATTERN:
        break;   /* no status LEDs on the limb board */
    default:
        break;
    }
}

/* ---------------- micro-ROS entities (parameters + alive beacon) ---------------- */

#if PS_UROS_ENABLED

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <std_msgs/msg/u_int32.h>

#define UROS_PARAMS_PER_JOINT 5
#define UROS_MAX_PARAMS       (PS_LIMB_JOINT_COUNT * UROS_PARAMS_PER_JOINT)

static rclc_parameter_server_t s_param_server;
static rcl_publisher_t s_alive_pub;
static rcl_timer_t s_alive_timer;
static std_msgs__msg__UInt32 s_alive_msg;
static char s_param_names[UROS_MAX_PARAMS][40];

static bool uros_on_param_changed(const Parameter *old_param, const Parameter *new_param, void *context)
{
    (void)old_param; (void)context;
    if (new_param == NULL || new_param->value.type != RCLC_PARAMETER_DOUBLE) {
        return false;   /* deletions and non-numeric types are rejected outright */
    }
    /* limb_param_set re-clamps against board_limb.h before touching a live
     * PID/torque-limit value: the network may never exceed the board table. */
    return limb_param_set(new_param->name.data, new_param->value.double_value);
}

static void uros_alive_timer_cb(rcl_timer_t *timer, int64_t last_call_time_ns)
{
    (void)timer; (void)last_call_time_ns;
    s_alive_msg.data++;
    (void)rcl_publish(&s_alive_pub, &s_alive_msg, NULL);
}

void limb_uros_create_entities(void *support, void *node, void *executor, void *arg)
{
    rclc_support_t *sup = (rclc_support_t *)support;
    rcl_node_t *nd = (rcl_node_t *)node;
    rclc_executor_t *exec = (rclc_executor_t *)executor;
    const char *alive_topic = (const char *)arg;

    rclc_parameter_options_t popts = rclc_parameter_get_default_options();
    popts.notify_changed_over_dds = true;
    popts.max_params = UROS_MAX_PARAMS;
    popts.allow_undeclared_parameters = false;
    rclc_parameter_server_init_with_option(&s_param_server, nd, &popts);
    rclc_executor_add_parameter_server(exec, &s_param_server, uros_on_param_changed);

    static const char *const suffixes[UROS_PARAMS_PER_JOINT] = {
        "pos_kp", "pos_ki", "vel_kp", "vel_ki", "torque_limit_cnm",
    };
    int n = 0;
    for (int j = 0; j < PS_LIMB_JOINT_COUNT; j++) {
        for (int k = 0; k < UROS_PARAMS_PER_JOINT; k++, n++) {
            snprintf(s_param_names[n], sizeof(s_param_names[n]), "%s_%s",
                     PS_LIMB_JOINTS[j].name, suffixes[k]);
            double v = 0.0;
            (void)limb_param_get(s_param_names[n], &v);
            rclc_add_parameter(&s_param_server, s_param_names[n], RCLC_PARAMETER_DOUBLE);
            rclc_parameter_set_double(&s_param_server, s_param_names[n], v);
        }
    }

    rclc_publisher_init_default(&s_alive_pub, nd,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), alive_topic);
    s_alive_msg.data = 0;
    rclc_timer_init_default(&s_alive_timer, sup, RCL_MS_TO_NS(1000), uros_alive_timer_cb);
    rclc_executor_add_timer(exec, &s_alive_timer);
}

#else /* !PS_UROS_ENABLED */

void limb_uros_create_entities(void *support, void *node, void *executor, void *arg)
{
    (void)support; (void)node; (void)executor; (void)arg;
    /* Unreachable: ps_uros_start() returns ESP_ERR_NOT_SUPPORTED without ever
     * invoking create_entities when the client is not vendored. Defined here
     * either way so app_main.c can hand this pointer to ps_uros_start()
     * unconditionally, per ps_uros.h's contract. */
}

#endif /* PS_UROS_ENABLED */

/* ---------------- 100 Hz task ---------------- */

static void comms_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    uint32_t tick = 0;
    ps_can_stats_t prev_can;
    memset(&prev_can, 0, sizeof(prev_can));

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(COMMS_TASK_PERIOD_MS));
        tick++;

        limb_joint_state_t st[PS_LIMB_JOINT_COUNT];
        limb_joint_state_snapshot(st);
        for (int j = 0; j < PS_LIMB_JOINT_COUNT; j++) {
            ps_joint_state_t js = {
                .joint = (uint8_t)j,
                .flags = (uint8_t)((st[j].saturated ? 0x01u : 0u) |
                                   (st[j].passive ? 0x02u : 0u)),
                .pos_crad = st[j].pos_crad,
                .vel_crad_s = st[j].vel_crad_s,
                .eff_cNm = st[j].eff_cNm,
            };
            limb_telem_send(PS_CLS_TELEM, PS_T_JOINT_STATE, &js, sizeof(js));
        }

        limb_sensor_snapshot_t snap;
        limb_sensor_snapshot(&snap);
        limb_telem_send(PS_CLS_TELEM, PS_T_IMU_QUAT, &snap.quat, sizeof(snap.quat));
        limb_telem_send(PS_CLS_TELEM, PS_T_IMU_ACC, &snap.acc, sizeof(snap.acc));
        limb_telem_send(PS_CLS_TELEM, PS_T_IMU_GYR, &snap.gyr, sizeof(snap.gyr));
        limb_telem_send(PS_CLS_TELEM, PS_T_FORCE, &snap.force, sizeof(snap.force));

        if ((tick % 100u) == 0u) {   /* 1 Hz NODE_STATS + VERSION */
            ps_can_stats_t cur;
            ps_can_get_stats(s_can, &cur);
            uint32_t rx = cur.rx_frames - prev_can.rx_frames;
            uint32_t tx = cur.tx_frames - prev_can.tx_frames;
            uint32_t err = (cur.bus_errors - prev_can.bus_errors) +
                           (cur.rx_dropped - prev_can.rx_dropped) +
                           (cur.tx_timeouts - prev_can.tx_timeouts);
            prev_can = cur;

            ps_node_stats_t nstat = {
                .cpu_pct = 0,   /* not instrumented on this node yet */
                .state = ps_safety_state(),
                .rx_fps = (uint16_t)(rx > 0xFFFFu ? 0xFFFFu : rx),
                .tx_fps = (uint16_t)(tx > 0xFFFFu ? 0xFFFFu : tx),
                .err_cnt = (uint16_t)(err > 0xFFFFu ? 0xFFFFu : err),
            };
            limb_telem_send(PS_CLS_TELEM, PS_T_NODE_STATS, &nstat, sizeof(nstat));

            ps_version_t ver = {
                .major = 0, .minor = 1, .patch = 0,
                .node_state = ps_safety_state(),
                .git_short = 0,   /* stamped by the build system once it exists */
            };
            limb_telem_send(PS_CLS_MGMT, PS_T_VERSION, &ver, sizeof(ver));
        }
    }
}

esp_err_t limb_comms_start(ps_can_handle_t can, uint8_t node_id)
{
    if (can == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_can = can;
    s_node_id = node_id;

    esp_err_t err = ps_can_register_class_cb(s_can, PS_CLS_CONTROL, control_rx_cb, NULL);
    if (err != ESP_OK) {
        return err;
    }
    BaseType_t ok = xTaskCreatePinnedToCore(comms_task, "limb_comms", 4096, NULL, 12, NULL, 0);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "comms up: node %u, %s + %s", (unsigned)node_id,
             PS_LIMB_JOINTS[0].name, PS_LIMB_JOINTS[1].name);
    return ESP_OK;
}
