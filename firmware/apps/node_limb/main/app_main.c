/* Nodes 1-4 — node_limb (arms/legs). One image, four boards: CONFIG_PS_LIMB_NODE_ID
 * (Kconfig.projbuild) selects the identity and board_limb.h's joint table.
 *
 * Boot order: board/peripherals come up inside limb_control_start() and
 * limb_comms_start() themselves; what matters here is the ORDER those two
 * calls happen in relative to ps_can_open/ps_safety_start, and that nothing
 * can actuate before ps_safety exists. ps_focdrv_init() (inside
 * limb_control_start) always leaves the driver disabled on boot, so even the
 * brief window between control_task starting and ps_safety_start returning
 * is inert. */
#include "limb_tasks.h"
#include "board_limb.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ps_can.h"
#include "ps_safety.h"
#include "ps_uros.h"

#include "powersuit_proto/can_id.h"

#include <stdio.h>

static const char *TAG = "node_limb";

static const char *node_name_for(uint8_t node_id)
{
    switch (node_id) {
    case PS_NODE_ARM_R: return "node_arm_right";
    case PS_NODE_ARM_L: return "node_arm_left";
    case PS_NODE_LEG_R: return "node_leg_right";
    case PS_NODE_LEG_L: return "node_leg_left";
    default:            return "node_limb_unknown";
    }
}

void app_main(void)
{
    const uint8_t node_id = (uint8_t)CONFIG_PS_LIMB_NODE_ID;
    const char *node_name = node_name_for(node_id);

    ESP_LOGI(TAG, "%s (node %u) booting: %s / %s", node_name, (unsigned)node_id,
             PS_LIMB_JOINTS[0].name, PS_LIMB_JOINTS[1].name);

    limb_safety_glue_init();

    ps_can_handle_t can;
    ps_can_config_t can_cfg = {
        .controller = 0,   /* ESP32-S3: one TWAI controller (ps_can.h contract) */
        .tx_gpio = PS_BOARD_CAN_TX_GPIO,
        .rx_gpio = PS_BOARD_CAN_RX_GPIO,
        .bitrate = 1000000u,
        .node_id = node_id,
    };
    ESP_ERROR_CHECK(ps_can_open(&can_cfg, &can));

    ps_safety_config_t safety_cfg = {
        .can = can,
        .node_id = node_id,
        .on_transition = limb_safety_on_transition,
        .arg = NULL,
    };
    ESP_ERROR_CHECK(ps_safety_start(&safety_cfg));

    ESP_ERROR_CHECK(limb_control_start());   /* core 1, prio 20 */
    ESP_ERROR_CHECK(limb_comms_start(can, node_id));  /* core 0, prio 12 */

    /* Static storage: ps_uros_config_t.arg outlives this function and is
     * handed back to limb_uros_create_entities on every (re)connect. */
    static char alive_topic[64];
    snprintf(alive_topic, sizeof(alive_topic), "/suit/nodes/%s/alive", node_name);

    ps_uros_config_t uros_cfg = {
        .can = can,
        .node_id = node_id,
        .node_name = node_name,
        .ros_namespace = "",
        .create_entities = limb_uros_create_entities,
        .destroy_entities = NULL,
        .on_agent_state = NULL,
        .arg = alive_topic,
        .spin_period_ms = 0,      /* default 10 ms */
        .task_priority = 5,
        .core = 0,                /* comms core (ps_uros.h contract) */
    };
    /* Unconditional: returns ESP_ERR_NOT_SUPPORTED harmlessly when the
     * micro-ROS client is not vendored (this verify build). */
    esp_err_t uros_err = ps_uros_start(&uros_cfg);
    if (uros_err != ESP_OK && uros_err != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGE(TAG, "ps_uros_start failed: %s", esp_err_to_name(uros_err));
    }

    /* The node stays in STANDBY (ps_safety_core_t boots there after the first
     * tick) until the orchestrator commands OPERATIONAL; every actuation
     * write already gates on ps_safety_can_actuate() regardless. */
    ESP_LOGI(TAG, "%s up: CAN tx=%d rx=%d, waiting for OPERATIONAL", node_name,
             PS_BOARD_CAN_TX_GPIO, PS_BOARD_CAN_RX_GPIO);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        limb_counters_t ctr;
        limb_counters_get(&ctr);
        ps_can_stats_t can_stats;
        ps_can_get_stats(can, &can_stats);
        ESP_LOGI(TAG,
                 "state=%u ticks=%lu oc_trips=%lu cmd[ok=%lu rej=%lu mode=%lu] "
                 "can[rx=%lu tx=%lu drop=%lu err=%lu]",
                 (unsigned)ps_safety_state(),
                 (unsigned long)ctr.control_ticks, (unsigned long)ctr.overcurrent_trips,
                 (unsigned long)ctr.joint_cmd_rx, (unsigned long)ctr.cmd_rejected,
                 (unsigned long)ctr.mode_set_rx,
                 (unsigned long)can_stats.rx_frames, (unsigned long)can_stats.tx_frames,
                 (unsigned long)can_stats.rx_dropped, (unsigned long)can_stats.bus_errors);
    }
}
