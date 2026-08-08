/* Two-board bench bring-up: the dead-man switch on real wire.
 *
 * Flash one DevKitC-1 as ORCHESTRATOR and one as LIMB (menuconfig ->
 * "Powersuit bench node"). The orchestrator emits the contract's 100 Hz
 * heartbeat over TWAI. The limb runs the real ps_safety_core state machine and
 * paints its state on the onboard RGB LED.
 *
 * Then pull the orchestrator's USB lead. The limb's LED must go amber within
 * 50 ms (docs/safety.md §2). That is the single most important behaviour in
 * the suit, and this is the cheapest way to see it actually happen.
 *
 * No motors, no soldering, no ADC. Wiring is in docs/bringup.md §A.
 *
 * Pin safety (ESP32-S3, worst case N16R8 with Octal PSRAM):
 *   forbidden: 0, 3, 45, 46 (strapping), 19/20 (USB), 43/44 (UART0 console),
 *              26-32 (flash/PSRAM), 35-37 (Octal PSRAM only)
 *   TWAI defaults 4 and 5 sit clear of every one of those.
 * The RGB LED pin is NOT hardcoded: it is GPIO48 on DevKitC-1 v1.0 and GPIO38
 * on v1.1, and a wrong guess looks exactly like a broken driver.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#include "ps_can.h"
#include "ps_safety.h"
#include "powersuit_proto/can_id.h"
#include "powersuit_proto/wire.h"

static const char *TAG = "bench";

#if CONFIG_PS_BENCH_LED_V11
#define BENCH_LED_GPIO 38
#define BENCH_LED_REV  "v1.1"
#elif CONFIG_PS_BENCH_LED_V10
#define BENCH_LED_GPIO 48
#define BENCH_LED_REV  "v1.0"
#elif CONFIG_PS_BENCH_LED_CUSTOM
#define BENCH_LED_GPIO CONFIG_PS_BENCH_LED_GPIO_CUSTOM
#define BENCH_LED_REV  "custom"
#else
#define BENCH_LED_GPIO (-1)
#define BENCH_LED_REV  "none"
#endif

/* An unselected Kconfig bool is undefined, not 0 — fine in #if, an undeclared
 * identifier anywhere else. Resolve both forms here, once. */
#if CONFIG_PS_BENCH_IS_ORCH
#define BENCH_NODE_ID   PS_NODE_ORCH
#define BENCH_ROLE_NAME "ORCHESTRATOR"
#else
#define BENCH_NODE_ID   CONFIG_PS_BENCH_NODE_ID
#define BENCH_ROLE_NAME "LIMB"
#endif

static led_strip_handle_t s_led;
static ps_can_handle_t s_can;

/* ---------------- LED ---------------- */

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_led == NULL) {
        return;
    }
    led_strip_set_pixel(s_led, 0, r, g, b);
    led_strip_refresh(s_led);
}

static esp_err_t led_open(int gpio, led_strip_handle_t *out)
{
    led_strip_config_t scfg = {
        .strip_gpio_num = gpio,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
    };
    return led_strip_new_rmt_device(&scfg, &rmt, out);
}

#if CONFIG_PS_BENCH_LED_PROBE
/* Which board is this? Drive each candidate pin in turn and let the user look.
 * Cheaper than reading a silkscreen with a magnifying glass. */
static void led_probe(void)
{
    const int candidates[] = { 38, 48 };
    for (unsigned i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        led_strip_handle_t h = NULL;
        ESP_LOGW(TAG, "PROBE: driving GPIO%d (DevKitC-1 %s) for 2 s — is the LED lit?",
                 candidates[i], candidates[i] == 38 ? "v1.1" : "v1.0");
        if (led_open(candidates[i], &h) == ESP_OK) {
            for (int k = 0; k < 4; k++) {
                led_strip_set_pixel(h, 0, 60, 0, 60);
                led_strip_refresh(h);
                vTaskDelay(pdMS_TO_TICKS(250));
                led_strip_clear(h);
                vTaskDelay(pdMS_TO_TICKS(250));
            }
            led_strip_del(h);
        } else {
            ESP_LOGE(TAG, "PROBE: could not open GPIO%d", candidates[i]);
        }
    }
    ESP_LOGW(TAG, "PROBE done. Set the revision in menuconfig and disable "
                  "PS_BENCH_LED_PROBE.");
}
#endif

/* Colour per safety state — same vocabulary as the hub's arc reactor. */
static void paint_state(uint8_t state, uint32_t t_ms)
{
    switch (state) {
    case PS_STATE_OPERATIONAL: {
        uint8_t v = (uint8_t)(40 + 40 * ((t_ms / 500) % 2));
        led_set(v, v, 120);            /* white-blue pulse */
        break;
    }
    case PS_STATE_STANDBY:
        led_set(90, 50, 0);            /* amber */
        break;
    case PS_STATE_PASSIVE:
        led_set(140, 45, 0);           /* brighter amber: link lost, limp */
        break;
    case PS_STATE_ESTOP:
        led_set(((t_ms / 100) % 2) ? 255 : 0, 0, 0);   /* 5 Hz red strobe */
        break;
    case PS_STATE_FAULT:
        led_set(180, 0, 60);
        break;
    default:
        led_set(0, 0, 60);             /* boot */
        break;
    }
}

/* ---------------- orchestrator role ---------------- */

#if CONFIG_PS_BENCH_IS_ORCH
static void heartbeat_task(void *arg)
{
    (void)arg;
    uint16_t seq = 0;
    TickType_t wake = xTaskGetTickCount();
    const int64_t t0 = esp_timer_get_time();

    ESP_LOGI(TAG, "beating at %u Hz — pull this board's USB to trip the limb",
             (unsigned)(1000 / PS_HEARTBEAT_PERIOD_MS));

    while (true) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PS_HEARTBEAT_PERIOD_MS));

        ps_heartbeat_t hb;
        memset(&hb, 0, sizeof(hb));
        hb.seq = seq;
        hb.src_state = PS_STATE_OPERATIONAL;
        hb.uptime_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);

        ps_can_frame_t f;
        f.id = ps_can_id_pack(PS_CLS_SAFETY, PS_NODE_ORCH, PS_NODE_BROADCAST,
                              PS_T_HEARTBEAT, (uint8_t)(seq & 0xFF));
        f.dlc = sizeof(hb);
        memcpy(f.data, &hb, sizeof(hb));
        (void)ps_can_send(s_can, &f, 0);

        /* Heartbeats prove liveness but do not grant authority: the limb needs
         * MODE_SET(OPERATIONAL) to latch intent (docs/safety.md §2). 1 Hz is
         * ample and mirrors what the real orchestrator does. */
        if ((seq % 100u) == 0u) {
            ps_mode_set_t ms;
            memset(&ms, 0, sizeof(ms));
            ms.target_state = PS_STATE_OPERATIONAL;
            ps_can_frame_t mf;
            mf.id = ps_can_id_pack(PS_CLS_CONTROL, PS_NODE_ORCH, PS_NODE_BROADCAST,
                                   PS_T_MODE_SET, 0);
            mf.dlc = sizeof(ms);
            memcpy(mf.data, &ms, sizeof(ms));
            (void)ps_can_send(s_can, &mf, 0);
        }

        /* Green blink so you can see it is alive without a serial console. */
        paint_state(PS_STATE_OPERATIONAL, hb.uptime_ms);
        seq++;
    }
}
#endif

/* ---------------- limb role ---------------- */

#if !CONFIG_PS_BENCH_IS_ORCH
static void on_transition(uint8_t old_state, uint8_t new_state, uint8_t cause, void *arg)
{
    (void)arg;
    ESP_LOGW(TAG, ">>> %u -> %u (cause %u) at %" PRId64 " ms", old_state, new_state,
             cause, esp_timer_get_time() / 1000);
}

/* ps_safety owns the SAFETY plane; CONTROL is the app's to route. */
static void control_rx_cb(const ps_can_frame_t *frame, void *arg)
{
    (void)arg;
    if (ps_can_id_type(frame->id) != PS_T_MODE_SET) {
        return;
    }
    ps_mode_set_t ms;
    memset(&ms, 0, sizeof(ms));
    memcpy(&ms, frame->data, frame->dlc < sizeof(ms) ? frame->dlc : sizeof(ms));
    ps_safety_note_cmd();
    ps_safety_note_mode_set(ms.target_state);
}

static void limb_task(void *arg)
{
    (void)arg;
    /* Ask to be operational; ps_safety only grants it once heartbeats arrive. */
    ESP_LOGI(TAG, "limb node %d waiting for heartbeats", BENCH_NODE_ID);

    uint32_t last_log = 0;
    while (true) {
        uint32_t t_ms = (uint32_t)(esp_timer_get_time() / 1000);
        uint8_t state = ps_safety_state();
        paint_state(state, t_ms);

        /* A command every 100 ms: re-arming after a trip needs one inside the
         * healthy heartbeat streak (docs/safety.md §2), and without it the LED
         * would sit amber forever and look like a bug. */
        ps_safety_note_cmd();

        if (t_ms - last_log >= 2000) {
            last_log = t_ms;
            ps_can_stats_t cs;
            ps_can_get_stats(s_can, &cs);
            ESP_LOGI(TAG, "state=%u rx=%" PRIu32 " tx=%" PRIu32 " err=%" PRIu32,
                     state, cs.rx_frames, cs.tx_frames, cs.bus_errors);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif

/* ---------------- boot ---------------- */

void app_main(void)
{
    printf("\n");
    ESP_LOGI(TAG, "==== powersuit bench node ====");
    ESP_LOGI(TAG, "role      : %s", BENCH_ROLE_NAME);
    ESP_LOGI(TAG, "node id   : %d", BENCH_NODE_ID);
    ESP_LOGI(TAG, "TWAI      : TX=GPIO%d RX=GPIO%d @ 1 Mbps",
             CONFIG_PS_BENCH_CAN_TX_GPIO, CONFIG_PS_BENCH_CAN_RX_GPIO);
    ESP_LOGI(TAG, "RGB LED   : GPIO%d (DevKitC-1 %s)", BENCH_LED_GPIO, BENCH_LED_REV);
    /* Said loudly and unconditionally: a wrong revision is a dark LED, which
     * is indistinguishable from a broken driver if nothing tells you. */
    ESP_LOGW(TAG, "if the LED stays dark, you have the OTHER board revision — "
                  "run 'idf.py menuconfig' -> Powersuit bench node -> DevKitC-1 "
                  "revision, or enable LED PROBE to let the board tell you");

#if CONFIG_PS_BENCH_LED_PROBE
    led_probe();
#endif

#if BENCH_LED_GPIO >= 0
    if (led_open(BENCH_LED_GPIO, &s_led) != ESP_OK) {
        ESP_LOGE(TAG, "RGB LED on GPIO%d failed to initialise — continuing "
                      "without it; the serial log still shows every transition",
                 BENCH_LED_GPIO);
        s_led = NULL;
    } else {
        led_strip_clear(s_led);
    }
#endif
    paint_state(PS_STATE_BOOT, 0);

    ps_can_config_t can_cfg = {
        .controller = 0,
        .tx_gpio = CONFIG_PS_BENCH_CAN_TX_GPIO,
        .rx_gpio = CONFIG_PS_BENCH_CAN_RX_GPIO,
        .bitrate = 1000000,
        .node_id = BENCH_NODE_ID,
    };
    esp_err_t err = ps_can_open(&can_cfg, &s_can);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI init failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "check the transceiver has 3V3 and GND, and that TX/RX "
                      "are not swapped");
        paint_state(PS_STATE_FAULT, 0);
        return;
    }

#if CONFIG_PS_BENCH_IS_ORCH
    xTaskCreatePinnedToCore(heartbeat_task, "beat", 4096, NULL, 12, NULL, 0);
#else
    ps_safety_config_t safety_cfg = {
        .can = s_can,
        .node_id = BENCH_NODE_ID,
        .on_transition = on_transition,
        .arg = NULL,
    };
    ESP_ERROR_CHECK(ps_safety_start(&safety_cfg));
    ESP_ERROR_CHECK(ps_can_register_class_cb(s_can, PS_CLS_CONTROL, control_rx_cb, NULL));
    ps_safety_note_cmd();
    xTaskCreatePinnedToCore(limb_task, "limb", 4096, NULL, 10, NULL, 1);
#endif
}
