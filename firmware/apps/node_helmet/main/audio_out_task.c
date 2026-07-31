/* AUDIO plane -> bone-conduction speaker.
 *
 * The jitter buffer lives in ps_audio; this file is the I2S plumbing around it.
 * When the buffer runs dry the task writes silence rather than blocking, which
 * keeps the DAC clocked and avoids the click you get from starving it. */
#include "helmet_tasks.h"
#include "board_helmet.h"

#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ps_audio.h"
#include "powersuit_proto/can_id.h"

#include <string.h>

static const char *TAG = "node_helmet";

#define AUDIO_OUT_STACK 4096
#define AUDIO_OUT_PRIO  11
#define AUDIO_OUT_CORE  0

static i2s_chan_handle_t s_tx_chan;
static ps_audio_rx_t s_rx;

static void audio_rx_cb(const ps_can_frame_t *frame, void *arg)
{
    (void)arg;
    /* Only speech addressed to this helmet; uplink echoes are ignored. */
    if (ps_can_id_dst(frame->id) != PS_NODE_HELMET) {
        return;
    }
    ps_audio_rx_on_frame(&s_rx, frame->id, frame->data, frame->dlc);
}

static esp_err_t spk_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_tx_chan, NULL), TAG, "i2s_new_channel");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(HELMET_SPK_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = HELMET_SPK_BCLK_GPIO,
            .ws = HELMET_SPK_WS_GPIO,
            .dout = HELMET_SPK_DOUT_GPIO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx_chan, &std_cfg), TAG, "i2s_init_std");
    return i2s_channel_enable(s_tx_chan);
}

static void audio_out_task(void *arg)
{
    (void)arg;
    static int16_t block[HELMET_SPK_BLOCK_SAMPLES];

    while (true) {
        size_t got = ps_audio_rx_pull(&s_rx, block, HELMET_SPK_BLOCK_SAMPLES);
        if (got < HELMET_SPK_BLOCK_SAMPLES) {
            memset(block + got, 0, (HELMET_SPK_BLOCK_SAMPLES - got) * sizeof(int16_t));
        }
        size_t written = 0;
        (void)i2s_channel_write(s_tx_chan, block, sizeof(block), &written, portMAX_DELAY);
    }
}

esp_err_t helmet_audio_out_start(ps_can_handle_t can)
{
    ps_audio_rx_init(&s_rx);
    ESP_RETURN_ON_ERROR(spk_init(), TAG, "speaker init");
    ESP_RETURN_ON_ERROR(ps_can_register_class_cb(can, PS_CLS_AUDIO, audio_rx_cb, NULL),
                        TAG, "audio cb");

    if (xTaskCreatePinnedToCore(audio_out_task, "helmet_spk", AUDIO_OUT_STACK, NULL,
                                AUDIO_OUT_PRIO, NULL, AUDIO_OUT_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "speaker up: %d Hz", HELMET_SPK_RATE_HZ);
    return ESP_OK;
}
