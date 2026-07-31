/* Arc-reactor status ring: 24 WS2812 pixels driven over RMT.
 *
 * The ring is the suit's most visible safety annunciator, so its pattern is
 * derived from the mirrored suit state rather than from anything a caller
 * passes in. A CONTROL LED_PATTERN addressed to the hub can take the ring over
 * temporarily, but the override always expires back to the state pattern —
 * nothing can leave the ring lying about an e-stop. */
#include "hub_tasks.h"
#include "board_hub.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#include <math.h>
#include <string.h>

static const char *TAG = "node_hub";

#define LED_TASK_STACK   3072
#define LED_TASK_PRIO    4
#define LED_TASK_CORE    1
#define LED_FRAME_MS     33      /* ~30 Hz */

static led_strip_handle_t s_strip;
static ps_led_pattern_t s_override;
static volatile bool s_override_active;
static int64_t s_override_until_us;

static uint8_t scale(uint8_t v, uint8_t bright)
{
    return (uint8_t)(((uint32_t)v * bright) / 255u);
}

static void fill(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < HUB_LED_COUNT; i++) {
        led_strip_set_pixel(s_strip, i, r, g, b);
    }
}

/* 0..1 triangle wave used for breathing and pulsing patterns. */
static float wave(uint32_t phase_ms, uint32_t period_ms)
{
    float x = (float)(phase_ms % period_ms) / (float)period_ms;
    return x < 0.5f ? (x * 2.0f) : (2.0f - x * 2.0f);
}

static void render_state(uint8_t state, uint32_t t_ms)
{
    switch (state) {
    case PS_STATE_BOOT: {
        /* Blue spinner while the node comes up. */
        fill(0, 0, 0);
        int head = (int)((t_ms / 60u) % HUB_LED_COUNT);
        for (int i = 0; i < 4; i++) {
            int px = (head - i + HUB_LED_COUNT) % HUB_LED_COUNT;
            uint8_t v = (uint8_t)(HUB_LED_MAX_BRIGHTNESS >> i);
            led_strip_set_pixel(s_strip, px, 0, 0, v);
        }
        break;
    }
    case PS_STATE_STANDBY: {
        uint8_t v = (uint8_t)(wave(t_ms, 2000) * (float)HUB_LED_MAX_BRIGHTNESS);
        fill(v, scale(v, 140), 0);          /* amber breathe */
        break;
    }
    case PS_STATE_OPERATIONAL: {
        uint8_t v = (uint8_t)((0.55f + 0.45f * wave(t_ms, 2000)) *
                              (float)HUB_LED_MAX_BRIGHTNESS);
        fill(scale(v, 190), scale(v, 220), v);   /* white-blue core pulse */
        break;
    }
    case PS_STATE_PASSIVE:
        fill(HUB_LED_MAX_BRIGHTNESS, scale(HUB_LED_MAX_BRIGHTNESS, 120), 0);
        break;
    case PS_STATE_ESTOP:
        /* 5 Hz red strobe: unmistakable, and never dimmed. */
        fill(((t_ms / 100u) % 2u) ? 255 : 0, 0, 0);
        break;
    case PS_STATE_FAULT:
    default:
        if ((t_ms / 300u) % 2u) {
            fill(255, 0, 0);
        } else {
            fill(180, 90, 0);
        }
        break;
    }
}

static void render_override(uint32_t t_ms)
{
    const ps_led_pattern_t *p = &s_override;
    uint8_t bright = p->brightness ? p->brightness : HUB_LED_MAX_BRIGHTNESS;
    uint32_t period = p->speed ? (2000u / (uint32_t)p->speed) : 1000u;
    if (period < 50u) {
        period = 50u;
    }

    switch (p->pattern) {
    case 1: {   /* breathe */
        float w = wave(t_ms, period);
        uint8_t v = (uint8_t)(w * (float)bright);
        fill(scale(p->r, v), scale(p->g, v), scale(p->b, v));
        break;
    }
    case 2: {   /* spinner */
        fill(0, 0, 0);
        int head = (int)((t_ms / (period / HUB_LED_COUNT + 1u)) % HUB_LED_COUNT);
        for (int i = 0; i < 4; i++) {
            int px = (head - i + HUB_LED_COUNT) % HUB_LED_COUNT;
            uint8_t v = (uint8_t)(bright >> i);
            led_strip_set_pixel(s_strip, px, scale(p->r, v), scale(p->g, v), scale(p->b, v));
        }
        break;
    }
    default:    /* solid */
        fill(scale(p->r, bright), scale(p->g, bright), scale(p->b, bright));
        break;
    }
}

static void led_task(void *arg)
{
    (void)arg;
    const int64_t t0 = esp_timer_get_time();

    while (true) {
        uint32_t t_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
        uint8_t state = hub_gateway_safety_state();

        /* An e-stop always wins the ring, override or not. */
        bool override_ok = s_override_active && state != PS_STATE_ESTOP &&
                           esp_timer_get_time() < s_override_until_us;
        if (s_override_active && !override_ok) {
            s_override_active = false;
        }

        if (override_ok) {
            render_override(t_ms);
        } else {
            render_state(state, t_ms);
        }
        led_strip_refresh(s_strip);
        vTaskDelay(pdMS_TO_TICKS(LED_FRAME_MS));
    }
}

esp_err_t hub_led_start(void)
{
    led_strip_config_t scfg = {
        .strip_gpio_num = HUB_LED_GPIO,
        .max_leds = HUB_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = { .invert_out = false },
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = { .with_dma = false },
    };
    esp_err_t err = led_strip_new_rmt_device(&scfg, &rmt_cfg, &s_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip init failed: %s", esp_err_to_name(err));
        return err;
    }
    led_strip_clear(s_strip);

    if (xTaskCreatePinnedToCore(led_task, "hub_led", LED_TASK_STACK, NULL, LED_TASK_PRIO, NULL,
                                LED_TASK_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "arc ring up: %d px on GPIO %d", HUB_LED_COUNT, HUB_LED_GPIO);
    return ESP_OK;
}

void hub_led_override(const ps_led_pattern_t *pat)
{
    if (pat == NULL) {
        return;
    }
    s_override = *pat;
    s_override_until_us = esp_timer_get_time() + (int64_t)HUB_LED_OVERRIDE_TIMEOUT_MS * 1000;
    s_override_active = true;
}
