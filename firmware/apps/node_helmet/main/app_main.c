/* Node 6 — node_helmet_interface (ESP32-S3).
 *
 * The wearer's only window into the suit: microphone, bone-conduction audio,
 * and the heads-up display. It actuates nothing, so its boot order is relaxed
 * compared with a limb — but the HUD comes up early on purpose, so that a fault
 * during the rest of initialisation is something the wearer can actually see. */
#include "helmet_tasks.h"
#include "board_helmet.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "ps_can.h"
#include "ps_safety.h"
#include "ps_uros.h"
#include "powersuit_proto/can_id.h"

static const char *TAG = "node_helmet";

void app_main(void)
{
    ESP_LOGI(TAG, "helmet interface booting");

    /* Wi-Fi (for ESP-NOW) needs NVS initialised before esp_wifi_init. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(helmet_hud_start());

    ps_can_handle_t can;
    ps_can_config_t can_cfg = {
        .controller = 0,
        .tx_gpio = HELMET_CAN_TX_GPIO,
        .rx_gpio = HELMET_CAN_RX_GPIO,
        .bitrate = HELMET_CAN_BITRATE,
        .node_id = PS_NODE_HELMET,
    };
    ESP_ERROR_CHECK(ps_can_open(&can_cfg, &can));

    ps_safety_config_t safety_cfg = {
        .can = can,
        .node_id = PS_NODE_HELMET,
        .on_transition = helmet_safety_on_transition,
        .arg = NULL,
    };
    ESP_ERROR_CHECK(ps_safety_start(&safety_cfg));

    ESP_ERROR_CHECK(helmet_audio_out_start(can));
    ESP_ERROR_CHECK(helmet_audio_in_start(can));
    ESP_ERROR_CHECK(helmet_comms_start(can));
    ESP_ERROR_CHECK(helmet_espnow_start());

    ps_uros_config_t uros_cfg = {
        .can = can,
        .node_id = PS_NODE_HELMET,
        .node_name = "node_helmet_interface",
        .ros_namespace = "",
        .create_entities = helmet_uros_create_entities,
        .destroy_entities = NULL,
        .on_agent_state = NULL,
        .arg = NULL,
        .spin_period_ms = 0,
        .task_priority = 5,
        .core = 0,
    };
    esp_err_t uros_err = ps_uros_start(&uros_cfg);
    if (uros_err != ESP_OK && uros_err != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGE(TAG, "ps_uros_start failed: %s", esp_err_to_name(uros_err));
    }
    helmet_hud_set_cloud(false);

    ESP_LOGI(TAG, "helmet up: CAN tx=%d rx=%d", HELMET_CAN_TX_GPIO, HELMET_CAN_RX_GPIO);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        helmet_hud_set_cloud(ps_uros_connected());
    }
}
