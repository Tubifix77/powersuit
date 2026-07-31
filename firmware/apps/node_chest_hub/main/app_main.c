/* Node 7 — node_chest_power_hub (ESP32-P4).
 *
 * Boot order matters here more than on any other node: the hub is the only path
 * between the two CAN buses and the orchestrator, so the SPI bridge and the
 * gateway must be live before the BMS is allowed to start raising trips, and the
 * BMS short-circuit observer must be armed before anything draws current.
 *
 * The hub deliberately does not run ps_safety_esp: that singleton binds to one
 * CAN handle and drives actuation gating, neither of which fits a node with two
 * buses and no actuators. The gateway keeps its own mirror of suit safety state
 * (hub_gateway_safety_state) purely to annunciate and to tag SPI frame headers. */
#include "hub_tasks.h"
#include "board_hub.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ps_can.h"
#include "ps_spibridge.h"

#include <inttypes.h>

static const char *TAG = "node_hub";

static ps_can_handle_t s_can1;
static ps_can_handle_t s_can2;

static void safety_tick_cb(void *arg)
{
    (void)arg;
    hub_gateway_safety_tick();
}

static void time_sync_cb(void *arg)
{
    (void)arg;
    hub_gateway_broadcast_time_sync();
}

static esp_err_t start_periodic(esp_timer_cb_t cb, const char *name, uint64_t period_us)
{
    esp_timer_handle_t h;
    const esp_timer_create_args_t args = {
        .callback = cb,
        .name = name,
        .dispatch_method = ESP_TIMER_TASK,
    };
    esp_err_t err = esp_timer_create(&args, &h);
    if (err != ESP_OK) {
        return err;
    }
    return esp_timer_start_periodic(h, period_us);
}

void app_main(void)
{
    ESP_LOGI(TAG, "chest power hub booting");

    /* No NVS on this node: nothing the hub owns is persisted across boots. The
     * e-stop clear counter lives in the orchestrator, and the BMS re-arm
     * interlock is hardware. */

    /* Both controllers open promiscuous (node_id 0): the gateway must see every
     * frame on both buses, not just those addressed to node 7. */
    ps_can_config_t can1 = {
        .controller = 0,
        .tx_gpio = HUB_CAN1_TX_GPIO,
        .rx_gpio = HUB_CAN1_RX_GPIO,
        .bitrate = HUB_CAN_BITRATE,
        .node_id = 0,
        .rx_queue_len = 128,
    };
    ps_can_config_t can2 = can1;
    can2.controller = 1;
    can2.tx_gpio = HUB_CAN2_TX_GPIO;
    can2.rx_gpio = HUB_CAN2_RX_GPIO;

    ESP_ERROR_CHECK(ps_can_open(&can1, &s_can1));
    ESP_ERROR_CHECK(ps_can_open(&can2, &s_can2));

    /* SPI first: the gateway pushes uplink records the moment it starts. */
    ps_spib_config_t spi = {
        .mosi_gpio = HUB_SPI_MOSI_GPIO,
        .miso_gpio = HUB_SPI_MISO_GPIO,
        .sclk_gpio = HUB_SPI_SCLK_GPIO,
        .cs_gpio = HUB_SPI_CS_GPIO,
        .data_ready_gpio = HUB_SPI_DATA_READY_GPIO,
    };
    ESP_ERROR_CHECK(ps_spib_start(&spi));

    ESP_ERROR_CHECK(hub_gateway_start(s_can1, s_can2));
    ESP_ERROR_CHECK(hub_led_start());
    ESP_ERROR_CHECK(hub_bms_start(s_can1, s_can2));

    /* 5 ms safety mirror tick (10x finer than the 50 ms watchdog) and the 1 Hz
     * MGMT TIME_SYNC broadcast that lets edge nodes unwrap 16-bit timestamps. */
    ESP_ERROR_CHECK(start_periodic(safety_tick_cb, "hub_safety", 5000));
    ESP_ERROR_CHECK(start_periodic(time_sync_cb, "hub_tsync", 1000000));

    ESP_LOGI(TAG, "hub up: bus1 tx=%d rx=%d, bus2 tx=%d rx=%d, spi %d MHz",
             HUB_CAN1_TX_GPIO, HUB_CAN1_RX_GPIO, HUB_CAN2_TX_GPIO, HUB_CAN2_RX_GPIO, 20);

    /* Periodic health line; every real duty lives in the tasks. */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        hub_gw_counters_t gw;
        ps_spib_stats_t spi_stats;
        hub_gateway_counters(&gw);
        ps_spib_get_stats(&spi_stats);
        ESP_LOGI(TAG,
                 "state=%u fwd[can1=%" PRIu32 " can2=%" PRIu32 " spi=%" PRIu32
                 " local=%" PRIu32 "] drops=%" PRIu32 " spi[xfer=%" PRIu32 " crc=%" PRIu32
                 " ovf=%" PRIu32 "]",
                 hub_gateway_safety_state(), gw.fwd[0], gw.fwd[1], gw.fwd[2],
                 gw.local_consumed, gw.queue_drops, spi_stats.transactions,
                 spi_stats.crc_errors, spi_stats.uplink_overflows);
    }
}
