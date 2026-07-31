/* Node 5 — node_flight_actuation (ESP32-S3): flap servos + aero engine.
 *
 * Boot order: CAN must be open before ps_safety_start (it registers the
 * SAFETY-class RX callback), and ps_safety must be armed before servo_task
 * starts walking the horns -- ps_safety_can_actuate() is the gate every
 * write checks (docs/safety.md §7). comms_task registers the CONTROL-class
 * callback and owns the CAN handle for TELEM/MGMT emission, so it starts
 * after the actuation/telemetry tasks that call into it are already able to
 * queue frames. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ps_can.h"
#include "ps_safety.h"
#include "ps_uros.h"

#include "node_flight.h"
#include "board_flight.h"

static const char *TAG = "node_flight";

/* Defined in safety_glue.c; exact-signature match to ps_safety_transition_cb_t.
 * Not carried in node_flight.h so that header stays free of the ps_safety.h
 * dependency (see safety_glue.c comment). */
extern void flight_safety_on_transition(uint8_t old_state, uint8_t new_state,
                                        uint8_t cause, void *arg);

void app_main(void)
{
    ESP_LOGI(TAG, "flight actuation node booting");

    ps_can_config_t can_cfg = {
        .controller = 0,
        .tx_gpio = PS_BOARD_CAN_TX_GPIO,
        .rx_gpio = PS_BOARD_CAN_RX_GPIO,
        .bitrate = 1000000,
        .node_id = PS_NODE_FLIGHT,
    };
    ps_can_handle_t can;
    ESP_ERROR_CHECK(ps_can_open(&can_cfg, &can));
    flight_set_can(can);

    flight_safety_glue_init();
    ps_safety_config_t safety_cfg = {
        .can = (struct ps_can_ctx *)can,
        .node_id = PS_NODE_FLIGHT,
        .on_transition = flight_safety_on_transition,
        .arg = NULL,
    };
    ESP_ERROR_CHECK(ps_safety_start(&safety_cfg));

    servo_task_start();
    aero_task_start();
    comms_task_start(can);

    ps_uros_config_t uros_cfg = {
        .can = can,
        .node_id = PS_NODE_FLIGHT,
        .node_name = "node_flight_actuation",
        .ros_namespace = "",
        .create_entities = flight_uros_create_entities,
        .destroy_entities = flight_uros_destroy_entities,
        .on_agent_state = NULL,
        .arg = NULL,
        .spin_period_ms = 0,   /* default 10 ms */
        .task_priority = 5,
        .core = 0,
    };
    esp_err_t uros_err = ps_uros_start(&uros_cfg);
    if (uros_err == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "micro-ROS client not vendored; XRCE plane disabled");
    } else {
        ESP_ERROR_CHECK(uros_err);
    }

    ESP_LOGI(TAG, "flight node up: can tx%d/rx%d, %u flaps",
             PS_BOARD_CAN_TX_GPIO, PS_BOARD_CAN_RX_GPIO, (unsigned)PS_FLAP_COUNT);

    /* Slow health-log loop; every real duty lives in the tasks above. */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ps_can_stats_t stats;
        ps_can_get_stats(can, &stats);
        ESP_LOGI(TAG,
                 "state=%u uros=%d rx=%lu tx=%lu drop=%lu err=%lu",
                 ps_safety_state(), (int)ps_uros_connected(),
                 (unsigned long)stats.rx_frames, (unsigned long)stats.tx_frames,
                 (unsigned long)stats.rx_dropped, (unsigned long)stats.bus_errors);
    }
}
