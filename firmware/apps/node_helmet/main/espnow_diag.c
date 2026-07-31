/* ESP-NOW diagnostics beacon.
 *
 * Broadcast-only by design. The handler below reads inbound frames purely to
 * count them and never acts on their contents: ESP-NOW is unauthenticated and
 * trivially spoofed from outside the suit, so giving it any influence over
 * state would be a control path with no access control in front of it. The suit
 * is commanded over CAN and micro-ROS, both behind Node 8. */
#include "helmet_tasks.h"
#include "board_helmet.h"

#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"

#include "ps_safety.h"
#include "powersuit_proto/wire.h"

#include <string.h>

static const char *TAG = "node_helmet";

static const uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

static esp_timer_handle_t s_beacon;
static uint32_t s_rx_count;

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)data;
    (void)len;
    s_rx_count++;
    if (info != NULL && (s_rx_count % 50u) == 1u) {
        ESP_LOGD(TAG, "espnow peer " MACSTR " (%u frames, ignored)",
                 MAC2STR(info->src_addr), (unsigned)s_rx_count);
    }
    /* Deliberately no dispatch: diagnostics only. */
}

static void beacon_cb(void *arg)
{
    (void)arg;
    ps_node_stats_t st;
    memset(&st, 0, sizeof(st));
    st.state = ps_safety_state();
    st.err_cnt = (uint16_t)(s_rx_count & 0xFFFF);
    (void)esp_now_send(BROADCAST_MAC, (const uint8_t *)&st, sizeof(st));
}

esp_err_t helmet_espnow_start(void)
{
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wcfg), TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");
    /* STA mode without associating: ESP-NOW needs the radio, not a network. */
    ESP_RETURN_ON_ERROR(
        esp_wifi_set_channel(HELMET_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE), TAG, "channel");

    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "espnow init");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(on_recv), TAG, "espnow recv cb");

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, BROADCAST_MAC, ESP_NOW_ETH_ALEN);
    peer.channel = HELMET_ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_RETURN_ON_ERROR(esp_now_add_peer(&peer), TAG, "espnow peer");

    const esp_timer_create_args_t args = {
        .callback = beacon_cb,
        .name = "helmet_beacon",
        .dispatch_method = ESP_TIMER_TASK,
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&args, &s_beacon), TAG, "beacon timer");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_beacon, 1000000), TAG, "beacon start");

    ESP_LOGI(TAG, "espnow diagnostics beacon on channel %d", HELMET_ESPNOW_CHANNEL);
    return ESP_OK;
}
