/* servo_task — 100 Hz flap output, core 1 (the "hardware task" of
 * ARCHITECTURE.md §2 / docs/safety.md §5). Chain per flap and per tick:
 *   target -> envelope clamp -> pairwise exclusion -> slew limit -> µs -> PWM.
 * flap_write() is the ONLY path that touches a comparator, and it re-clamps
 * to the mechanical pulse envelope immediately before the write, so no code
 * path can bypass the limit table. PASSIVE/ESTOP (ps_safety_can_actuate()
 * false) forces neutral targets, still slew-limited (§5.4).
 *
 * PWM: MCPWM, 1 µs timebase, 20 ms period. 12 outputs = 2 groups x 3
 * operators x 2 generators — ESP32-S3 LEDC tops out at 8 channels, see
 * board_flight.h for the recorded deviation. */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#include "ps_safety.h"
#include "node_flight.h"

static const char *TAG = "node_flight";

#define SERVO_TASK_PERIOD_MS 10u   /* 100 Hz */

/* ---- shared target buffer (writer: comms core 0; reader: here, core 1) ---- */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static flight_target_t s_targets[PS_FLAP_COUNT];
static flight_flap_state_t s_states[PS_FLAP_COUNT];

void flight_targets_set(uint8_t flap, int16_t pos_pm, uint8_t rate_lim, bool brake)
{
    if (flap >= PS_FLAP_COUNT) {
        return;
    }
    taskENTER_CRITICAL(&s_lock);
    s_targets[flap].pos_pm = pos_pm;
    s_targets[flap].rate_lim = rate_lim;
    s_targets[flap].brake = brake;
    taskEXIT_CRITICAL(&s_lock);
}

void flight_targets_neutral_all(void)
{
    taskENTER_CRITICAL(&s_lock);
    for (int i = 0; i < PS_FLAP_COUNT; i++) {
        s_targets[i].pos_pm = 0;
        s_targets[i].rate_lim = 0;
        s_targets[i].brake = false;
    }
    taskEXIT_CRITICAL(&s_lock);
}

void flight_targets_snapshot(flight_target_t out[PS_FLAP_COUNT])
{
    taskENTER_CRITICAL(&s_lock);
    memcpy(out, s_targets, sizeof(s_targets));
    taskEXIT_CRITICAL(&s_lock);
}

void flight_flap_state_publish(const flight_flap_state_t *st, uint8_t flap)
{
    taskENTER_CRITICAL(&s_lock);
    s_states[flap] = *st;
    taskEXIT_CRITICAL(&s_lock);
}

void flight_flap_state_snapshot(flight_flap_state_t out[PS_FLAP_COUNT])
{
    taskENTER_CRITICAL(&s_lock);
    memcpy(out, s_states, sizeof(s_states));
    taskEXIT_CRITICAL(&s_lock);
}

/* ---- MCPWM bank ---- */
static mcpwm_cmpr_handle_t s_cmp[PS_FLAP_COUNT];

static void servo_bank_init(void)
{
    mcpwm_timer_handle_t timers[2];
    mcpwm_oper_handle_t opers[2][3];

    for (int grp = 0; grp < 2; grp++) {
        mcpwm_timer_config_t tcfg = {
            .group_id = grp,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 1000000,               /* 1 tick = 1 µs */
            .period_ticks = PS_SERVO_PERIOD_TICKS,  /* 20 ms -> 50 Hz */
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, &timers[grp]));
        for (int op = 0; op < 3; op++) {
            mcpwm_operator_config_t ocfg = { .group_id = grp };
            ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &opers[grp][op]));
            ESP_ERROR_CHECK(mcpwm_operator_connect_timer(opers[grp][op], timers[grp]));
        }
    }
    for (int i = 0; i < PS_FLAP_COUNT; i++) {
        int grp = i / 6, op = (i % 6) / 2;
        mcpwm_gen_handle_t gen;
        mcpwm_comparator_config_t ccfg = { .flags.update_cmp_on_tez = true };
        mcpwm_generator_config_t gcfg = { .gen_gpio_num = PS_FLAP_TABLE[i].gpio };

        ESP_ERROR_CHECK(mcpwm_new_comparator(opers[grp][op], &ccfg, &s_cmp[i]));
        ESP_ERROR_CHECK(mcpwm_new_generator(opers[grp][op], &gcfg, &gen));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
            s_cmp[i], PS_FLAP_TABLE[i].pulse_neutral_us));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                           s_cmp[i], MCPWM_GEN_ACTION_LOW)));
    }
    for (int grp = 0; grp < 2; grp++) {
        ESP_ERROR_CHECK(mcpwm_timer_enable(timers[grp]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timers[grp], MCPWM_TIMER_START_NO_STOP));
    }
}

static int16_t clamp_pm(int16_t v, int16_t lo, int16_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/* THE single write path (docs/safety.md §5.1): envelope clamp -> permille to
 * µs -> mechanical pulse clamp -> comparator. Nothing else writes PWM. */
static void flap_write(int flap, int16_t pos_pm)
{
    const ps_flap_cfg_t *c = &PS_FLAP_TABLE[flap];
    int32_t pulse;

    pos_pm = clamp_pm(pos_pm, c->env_min_pm, c->env_max_pm);
    if (pos_pm >= 0) {
        pulse = c->pulse_neutral_us +
                ((int32_t)pos_pm * (c->pulse_max_us - c->pulse_neutral_us)) / 1000;
    } else {
        pulse = c->pulse_neutral_us +
                ((int32_t)pos_pm * (c->pulse_neutral_us - c->pulse_min_us)) / 1000;
    }
    if (pulse < c->pulse_min_us) {
        pulse = c->pulse_min_us;
    }
    if (pulse > c->pulse_max_us) {
        pulse = c->pulse_max_us;
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_cmp[flap], (uint32_t)pulse));
}

static void servo_task(void *arg)
{
    static int16_t cur_pm[PS_FLAP_COUNT];       /* value on the horn */
    flight_target_t tgt[PS_FLAP_COUNT];
    int16_t want[PS_FLAP_COUNT];
    bool limited[PS_FLAP_COUNT];
    TickType_t wake = xTaskGetTickCount();

    (void)arg;
    servo_bank_init();
    memset(cur_pm, 0, sizeof(cur_pm));

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SERVO_TASK_PERIOD_MS));

        flight_targets_snapshot(tgt);
        const bool can_act = ps_safety_can_actuate();
        const float rate_scale = flight_param_rate_scale();

        for (int i = 0; i < PS_FLAP_COUNT; i++) {
            const ps_flap_cfg_t *c = &PS_FLAP_TABLE[i];
            int16_t w;

            if (!can_act) {
                w = 0; /* PASSIVE/ESTOP: neutral column is authoritative (§5.4) */
            } else if (tgt[i].brake) {
                w = c->brake_pm;
            } else {
                w = tgt[i].pos_pm;
            }
            /* 1. envelope clamp */
            int16_t clamped = clamp_pm(w, c->env_min_pm, c->env_max_pm);
            limited[i] = (clamped != w) ||
                         (clamped == c->env_min_pm) || (clamped == c->env_max_pm);
            want[i] = clamped;
        }

        /* 2. pairwise exclusion: pull both members of an overlapping pair
         * toward the nearest legal point — magnitudes reduced equally until
         * |a| + |b| <= joint_max_sum_pm (docs/safety.md §5.2). */
        for (int p = 0; p < PS_FLAP_PAIR_COUNT; p++) {
            const ps_flap_pair_t *pr = &PS_FLAP_PAIRS[p];
            int32_t sum = (int32_t)(want[pr->a] < 0 ? -want[pr->a] : want[pr->a]) +
                          (int32_t)(want[pr->b] < 0 ? -want[pr->b] : want[pr->b]);
            if (sum > pr->joint_max_sum_pm) {
                int32_t cut = (sum - pr->joint_max_sum_pm + 1) / 2;
                for (int k = 0; k < 2; k++) {
                    uint8_t f = (k == 0) ? pr->a : pr->b;
                    int32_t mag = want[f] < 0 ? -want[f] : want[f];
                    mag -= cut;
                    if (mag < 0) {
                        mag = 0;
                    }
                    want[f] = (int16_t)(want[f] < 0 ? -mag : mag);
                    limited[f] = true;
                }
            }
        }

        for (int i = 0; i < PS_FLAP_COUNT; i++) {
            const ps_flap_cfg_t *c = &PS_FLAP_TABLE[i];

            /* 3. slew limit: FLAP_CMD.rate_lim may only lower the board cap;
             * the global rate_scale parameter derates everything. 1 % = 10 pm. */
            uint32_t pct_s = c->slew_default_pct_s;
            if (can_act && tgt[i].rate_lim != 0 && tgt[i].rate_lim < pct_s) {
                pct_s = tgt[i].rate_lim;
            }
            if (pct_s > c->slew_max_pct_s) {
                pct_s = c->slew_max_pct_s;
            }
            int32_t step = (int32_t)((float)pct_s * 10.0f * rate_scale) *
                           (int32_t)SERVO_TASK_PERIOD_MS / 1000;
            if (step < 1) {
                step = 1;
            }
            int32_t delta = (int32_t)want[i] - cur_pm[i];
            if (delta > step) {
                delta = step;
            } else if (delta < -step) {
                delta = -step;
            }
            cur_pm[i] = (int16_t)(cur_pm[i] + delta);

            /* 4. the write (envelope re-clamped inside) */
            flap_write(i, cur_pm[i]);

            flight_flap_state_t st = {
                .pos_pm = cur_pm[i],
                .target_pm = want[i],
                .at_limit = limited[i],
            };
            flight_flap_state_publish(&st, (uint8_t)i);
        }
    }
}

void servo_task_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(servo_task, "servo", 4096, NULL, 18, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "servo task create failed");
    }
}
