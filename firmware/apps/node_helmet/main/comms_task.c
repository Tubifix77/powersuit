/* Helmet comms: environment telemetry, wake events, and the HUD feed.
 *
 * Note the asymmetry with the other nodes. A limb's data is high-rate, so it
 * rides the raw TELEM plane. The helmet's is not: ambient temperature at 1 Hz
 * and an occasional wake word fit comfortably inside micro-ROS, and the HUD
 * content it consumes is composed on Node 8 and arrives as a Marker. That is
 * why this is the one node whose interesting traffic is mostly XRCE, and why
 * the HUD does not simply read cross-bus TELEM — §5 forbids that routing. */
#include "helmet_tasks.h"
#include "board_helmet.h"

#include "driver/temperature_sensor.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ps_can.h"
#include "ps_safety.h"
#include "powersuit_proto/can_id.h"
#include "powersuit_proto/wire.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "node_helmet";

#define COMMS_STACK 4096
#define COMMS_PRIO  10
#define COMMS_CORE  0

static ps_can_handle_t s_can;
static temperature_sensor_handle_t s_temp;
static uint8_t s_env_seq;
static uint8_t s_stats_seq;

static void telem_send(uint8_t type, const void *payload, uint8_t len, uint8_t seq)
{
    ps_can_frame_t f;
    f.id = ps_can_id_pack(PS_CLS_TELEM, PS_NODE_HELMET, PS_NODE_ORCH, type, seq);
    f.dlc = len;
    memset(f.data, 0, sizeof(f.data));
    memcpy(f.data, payload, len);
    (void)ps_can_send(s_can, &f, pdMS_TO_TICKS(5));
}

static void comms_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));

        float tsens = 0.0f;
        ps_env_t env;
        memset(&env, 0, sizeof(env));
        if (temperature_sensor_get_celsius(s_temp, &tsens) == ESP_OK) {
            env.temp_cC = (int16_t)(tsens * 100.0f);
        }
        /* Humidity and pressure sensors are not fitted on this build; 0xFFFF is
         * the agreed "not present" sentinel rather than a plausible zero. */
        env.rh_pm = 0xFFFF;
        env.press_dhPa = 0xFFFF;
        telem_send(PS_T_ENV, &env, sizeof(env), s_env_seq++);

        ps_node_stats_t st;
        memset(&st, 0, sizeof(st));
        ps_can_stats_t cs;
        ps_can_get_stats(s_can, &cs);
        st.state = ps_safety_state();
        st.rx_fps = (uint16_t)(cs.rx_frames & 0xFFFF);
        st.tx_fps = (uint16_t)(cs.tx_frames & 0xFFFF);
        st.err_cnt = (uint16_t)(cs.bus_errors & 0xFFFF);
        telem_send(PS_T_NODE_STATS, &st, sizeof(st), s_stats_seq++);
    }
}

esp_err_t helmet_comms_start(ps_can_handle_t can)
{
    s_can = can;

    temperature_sensor_config_t tcfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    ESP_RETURN_ON_ERROR(temperature_sensor_install(&tcfg, &s_temp), TAG, "tsens install");
    ESP_RETURN_ON_ERROR(temperature_sensor_enable(s_temp), TAG, "tsens enable");

    if (xTaskCreatePinnedToCore(comms_task, "helmet_comms", COMMS_STACK, NULL, COMMS_PRIO,
                                NULL, COMMS_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ---- micro-ROS entities (absent when the client is not vendored) ---- */
#if PS_UROS_ENABLED

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/temperature.h>
#include <std_msgs/msg/string.h>
#include <visualization_msgs/msg/marker.h>

static rcl_publisher_t s_voice_pub;
static rcl_publisher_t s_ambient_pub;
static rcl_subscription_t s_hud_sub;
static visualization_msgs__msg__Marker s_marker;
static std_msgs__msg__String s_voice_msg;
static sensor_msgs__msg__Temperature s_ambient_msg;
static char s_voice_buf[32];
static rcl_timer_t s_pub_timer;

/* Node 8 composes HUD content and sends it as a Marker. The mapping is:
 *   ns == "battery"  -> scale.x carries the pack percentage
 *   ns == "warning"  -> text is appended to the warning list
 *   ns == "clear"    -> drop all warnings
 * A cloud-sourced advisory reaches this path only after Node 8 has vetted it
 * against the downlink whitelist, and it can never set the safety banner. */
static void hud_marker_cb(const void *msgin)
{
    const visualization_msgs__msg__Marker *m = (const visualization_msgs__msg__Marker *)msgin;
    if (m->ns.data == NULL) {
        return;
    }
    if (strcmp(m->ns.data, "battery") == 0) {
        double pct = m->scale.x;
        if (pct >= 0.0 && pct <= 100.0) {
            helmet_hud_set_battery((uint8_t)pct);
        }
    } else if (strcmp(m->ns.data, "warning") == 0 && m->text.data != NULL) {
        helmet_hud_push_warning(m->text.data);
    } else if (strcmp(m->ns.data, "clear") == 0) {
        helmet_hud_clear_warnings();
    }
}

static void pub_timer_cb(rcl_timer_t *timer, int64_t last_call)
{
    (void)timer;
    (void)last_call;

    if (helmet_take_wake_event()) {
        snprintf(s_voice_buf, sizeof(s_voice_buf), "wake");
        s_voice_msg.data.data = s_voice_buf;
        s_voice_msg.data.size = strlen(s_voice_buf);
        s_voice_msg.data.capacity = sizeof(s_voice_buf);
        (void)rcl_publish(&s_voice_pub, &s_voice_msg, NULL);
    }

    float tsens = 0.0f;
    if (temperature_sensor_get_celsius(s_temp, &tsens) == ESP_OK) {
        s_ambient_msg.temperature = tsens;
        s_ambient_msg.variance = 0.0;
        (void)rcl_publish(&s_ambient_pub, &s_ambient_msg, NULL);
    }
}

void helmet_uros_create_entities(void *support, void *node, void *executor, void *arg)
{
    (void)arg;
    rclc_support_t *sup = (rclc_support_t *)support;
    rcl_node_t *nd = (rcl_node_t *)node;
    rclc_executor_t *exec = (rclc_executor_t *)executor;

    rclc_publisher_init_default(
        &s_voice_pub, nd, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/suit/voice/trigger");
    rclc_publisher_init_default(
        &s_ambient_pub, nd, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
        "/suit/environment/ambient");

    visualization_msgs__msg__Marker__init(&s_marker);
    rclc_subscription_init_best_effort(
        &s_hud_sub, nd, ROSIDL_GET_MSG_TYPE_SUPPORT(visualization_msgs, msg, Marker),
        "/suit/hud/telemetry_display");
    rclc_executor_add_subscription(exec, &s_hud_sub, &s_marker, hud_marker_cb, ON_NEW_DATA);

    /* 4 Hz: fast enough that a wake word is not perceptibly late, slow enough
     * that the XRCE plane stays a rounding error in the §10 budget. */
    rclc_timer_init_default2(&s_pub_timer, sup, RCL_MS_TO_NS(250), pub_timer_cb, true);
    rclc_executor_add_timer(exec, &s_pub_timer);

    ESP_LOGI(TAG, "micro-ROS entities up");
}

#else

void helmet_uros_create_entities(void *support, void *node, void *executor, void *arg)
{
    (void)support;
    (void)node;
    (void)executor;
    (void)arg;
}

#endif /* PS_UROS_ENABLED */
