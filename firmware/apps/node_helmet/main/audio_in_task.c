/* Microphone -> AUDIO plane (docs/network-map.md §3.5, docs/safety.md §6).
 *
 * Capture is 16 kHz because that is what the MEMS part offers; the plane is
 * 8 kHz, so blocks are decimated by two before anything else touches them. The
 * VOX gate is what keeps a hot mic from spending 500 frames/s of bus 1 on room
 * tone — without it the §10 budget does not close. */
#include "helmet_tasks.h"
#include "board_helmet.h"

#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ps_audio.h"
#include "powersuit_proto/can_id.h"

#include <stdatomic.h>
#include <string.h>

static const char *TAG = "node_helmet";

#define AUDIO_IN_STACK 4096
#define AUDIO_IN_PRIO  12
#define AUDIO_IN_CORE  0

static i2s_chan_handle_t s_rx;
static ps_can_handle_t s_can;
static ps_audio_tx_t s_tx;
static ps_egate_t s_gate;
static atomic_bool s_wake_pending;

bool helmet_take_wake_event(void)
{
    return atomic_exchange(&s_wake_pending, false);
}

/* Hands one finished AUDIO frame to the bus. A full TX queue means bus 1 is
 * congested; dropping voice is the right sacrifice — it is the lowest-priority
 * class on the wire for exactly this reason. */
static void emit_frame(uint32_t id, const uint8_t *data, uint8_t dlc, void *arg)
{
    (void)arg;
    ps_can_frame_t f;
    f.id = id;
    f.dlc = dlc;
    memcpy(f.data, data, dlc);
    if (dlc < 8) {
        memset(f.data + dlc, 0, 8u - dlc);
    }
    (void)ps_can_send(s_can, &f, 0);
}

static void send_ctl(uint8_t cmd)
{
    ps_audio_ctl_t ctl = { .dir = PS_AUDIO_DIR_UP, .cmd = cmd, .sample_rate = PS_AUDIO_SAMPLE_RATE };
    ps_can_frame_t f;
    f.id = ps_can_id_pack(PS_CLS_AUDIO, PS_NODE_HELMET, PS_NODE_ORCH, PS_T_AUDIO_CTL, 0);
    f.dlc = sizeof(ctl);
    memset(f.data, 0, sizeof(f.data));
    memcpy(f.data, &ctl, sizeof(ctl));
    (void)ps_can_send(s_can, &f, pdMS_TO_TICKS(5));
}

static esp_err_t mic_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &s_rx), TAG, "i2s_new_channel");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(HELMET_MIC_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = HELMET_MIC_BCLK_GPIO,
            .ws = HELMET_MIC_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din = HELMET_MIC_DIN_GPIO,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx, &std_cfg), TAG, "i2s_init_std");
    return i2s_channel_enable(s_rx);
}

static void audio_in_task(void *arg)
{
    (void)arg;
    static int32_t raw[HELMET_MIC_BLOCK_SAMPLES];
    static int16_t pcm8k[HELMET_MIC_BLOCK_SAMPLES / HELMET_MIC_DECIMATE];
    bool streaming = false;

    while (true) {
        size_t got = 0;
        if (i2s_channel_read(s_rx, raw, sizeof(raw), &got, portMAX_DELAY) != ESP_OK) {
            continue;
        }
        size_t n32 = got / sizeof(int32_t);

        /* 32-bit slot carries the sample in the high bits; decimate by
         * averaging pairs, which is both the anti-alias filter and the
         * rate conversion at this ratio. */
        size_t n8 = 0;
        for (size_t i = 0; i + 1 < n32; i += 2) {
            int32_t a = raw[i] >> 16;
            int32_t b = raw[i + 1] >> 16;
            pcm8k[n8++] = (int16_t)((a + b) / 2);
        }

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        bool open = ps_egate_process(&s_gate, pcm8k, n8, now_ms);

        if (open && !streaming) {
            send_ctl(1);
            ps_audio_tx_sync(&s_tx, emit_frame, NULL);
            atomic_store(&s_wake_pending, true);
            streaming = true;
            ESP_LOGI(TAG, "voice uplink open");
        }

        if (streaming) {
            ps_audio_tx_push(&s_tx, pcm8k, n8, emit_frame, NULL);
        }

        if (!open && streaming) {
            send_ctl(0);
            streaming = false;
            ESP_LOGI(TAG, "voice uplink closed");
        }
    }
}

esp_err_t helmet_audio_in_start(ps_can_handle_t can)
{
    s_can = can;
    ps_audio_tx_init(&s_tx, PS_AUDIO_DIR_UP, PS_NODE_HELMET, PS_NODE_ORCH);
    ps_egate_init(&s_gate, HELMET_VOX_THRESHOLD_DBFS, HELMET_VOX_HANG_MS,
                  HELMET_VOX_MAX_UTTER_MS);
    atomic_init(&s_wake_pending, false);

    ESP_RETURN_ON_ERROR(mic_init(), TAG, "mic init");
    if (xTaskCreatePinnedToCore(audio_in_task, "helmet_mic", AUDIO_IN_STACK, NULL,
                                AUDIO_IN_PRIO, NULL, AUDIO_IN_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "mic up: %d Hz -> %d Hz, VOX %d dBFS",
             HELMET_MIC_RATE_HZ, PS_AUDIO_SAMPLE_RATE, HELMET_VOX_THRESHOLD_DBFS);
    return ESP_OK;
}
