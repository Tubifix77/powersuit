/* Three-phase center-aligned MCPWM output stage (ps_focdrv.h contract).
 *
 * One MCPWM timer per joint drives three operators (one per phase). Each phase
 * uses a single comparator and a high-side generator; the low-side generator is
 * produced by the dead-time module as an inverted, delayed copy. That is what
 * makes shoot-through a hardware impossibility rather than a software promise. */
#include "ps_focdrv.h"

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "ps_focdrv";

#define FOCDRV_RESOLUTION_HZ  10000000u   /* 10 MHz timer tick */
#define FOCDRV_DEFAULT_HZ     20000u
#define FOCDRV_DEFAULT_DT_NS  200u

struct ps_focdrv_ctx {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper[3];
    mcpwm_cmpr_handle_t cmpr[3];
    mcpwm_gen_handle_t gen_high[3];
    mcpwm_gen_handle_t gen_low[3];
    uint32_t period_ticks;   /* half-period: up-down counting doubles it */
};

esp_err_t ps_focdrv_init(const ps_focdrv_config_t *cfg, ps_focdrv_handle_t *out)
{
    if (cfg == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    struct ps_focdrv_ctx *ctx = calloc(1, sizeof(*ctx));
    if (ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    uint32_t pwm_hz = cfg->pwm_hz ? cfg->pwm_hz : FOCDRV_DEFAULT_HZ;
    uint32_t dt_ns = cfg->deadtime_ns ? cfg->deadtime_ns : FOCDRV_DEFAULT_DT_NS;
    /* Up-down (center-aligned) counting traverses the period twice per cycle. */
    ctx->period_ticks = FOCDRV_RESOLUTION_HZ / (2u * pwm_hz);
    uint32_t dt_ticks = (FOCDRV_RESOLUTION_HZ / 1000000u) * dt_ns / 1000u;
    if (dt_ticks == 0) {
        dt_ticks = 1;
    }

    esp_err_t err;
    mcpwm_timer_config_t tcfg = {
        .group_id = cfg->group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = FOCDRV_RESOLUTION_HZ,
        .period_ticks = ctx->period_ticks,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
    };
    err = mcpwm_new_timer(&tcfg, &ctx->timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_timer(group %d): %s", cfg->group_id, esp_err_to_name(err));
        goto fail;
    }

    const int gpio_high[3] = { cfg->gpio_uh, cfg->gpio_vh, cfg->gpio_wh };
    const int gpio_low[3] = { cfg->gpio_ul, cfg->gpio_vl, cfg->gpio_wl };

    for (int i = 0; i < 3; i++) {
        mcpwm_operator_config_t ocfg = { .group_id = cfg->group_id };
        err = mcpwm_new_operator(&ocfg, &ctx->oper[i]);
        if (err != ESP_OK) {
            goto fail;
        }
        err = mcpwm_operator_connect_timer(ctx->oper[i], ctx->timer);
        if (err != ESP_OK) {
            goto fail;
        }

        mcpwm_comparator_config_t ccfg = { .flags = { .update_cmp_on_tez = true } };
        err = mcpwm_new_comparator(ctx->oper[i], &ccfg, &ctx->cmpr[i]);
        if (err != ESP_OK) {
            goto fail;
        }
        err = mcpwm_comparator_set_compare_value(ctx->cmpr[i], ctx->period_ticks / 2);
        if (err != ESP_OK) {
            goto fail;
        }

        mcpwm_generator_config_t gh = { .gen_gpio_num = gpio_high[i] };
        err = mcpwm_new_generator(ctx->oper[i], &gh, &ctx->gen_high[i]);
        if (err != ESP_OK) {
            goto fail;
        }
        mcpwm_generator_config_t gl = { .gen_gpio_num = gpio_low[i] };
        err = mcpwm_new_generator(ctx->oper[i], &gl, &ctx->gen_low[i]);
        if (err != ESP_OK) {
            goto fail;
        }

        /* Center-aligned: go low on the up-count compare, high on the way back. */
        err = mcpwm_generator_set_actions_on_compare_event(
            ctx->gen_high[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, ctx->cmpr[i],
                                           MCPWM_GEN_ACTION_LOW),
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, ctx->cmpr[i],
                                           MCPWM_GEN_ACTION_HIGH),
            MCPWM_GEN_COMPARE_EVENT_ACTION_END());
        if (err != ESP_OK) {
            goto fail;
        }

        /* Dead time: high side delayed on its rising edge, low side inverted and
         * delayed on the falling edge. Both derive from gen_high. */
        mcpwm_dead_time_config_t dt_high = { .posedge_delay_ticks = dt_ticks };
        err = mcpwm_generator_set_dead_time(ctx->gen_high[i], ctx->gen_high[i], &dt_high);
        if (err != ESP_OK) {
            goto fail;
        }
        mcpwm_dead_time_config_t dt_low = {
            .negedge_delay_ticks = dt_ticks,
            .flags = { .invert_output = true },
        };
        err = mcpwm_generator_set_dead_time(ctx->gen_high[i], ctx->gen_low[i], &dt_low);
        if (err != ESP_OK) {
            goto fail;
        }
    }

    err = mcpwm_timer_enable(ctx->timer);
    if (err != ESP_OK) {
        goto fail;
    }
    err = mcpwm_timer_start_stop(ctx->timer, MCPWM_TIMER_START_NO_STOP);
    if (err != ESP_OK) {
        goto fail;
    }

    *out = ctx;
    ps_focdrv_disable(ctx);   /* never energise a joint on boot */
    ESP_LOGI(TAG, "group %d: %" PRIu32 " Hz, dead time %" PRIu32 " ns",
             cfg->group_id, pwm_hz, dt_ns);
    return ESP_OK;

fail:
    free(ctx);
    return err == ESP_OK ? ESP_FAIL : err;
}

esp_err_t ps_focdrv_set_duty(ps_focdrv_handle_t h, const float duty[3])
{
    if (h == NULL || duty == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < 3; i++) {
        float d = duty[i];
        if (d < 0.0f) {
            d = 0.0f;
        } else if (d > 1.0f) {
            d = 1.0f;
        }
        uint32_t ticks = (uint32_t)(d * (float)h->period_ticks);
        if (ticks > h->period_ticks) {
            ticks = h->period_ticks;
        }
        esp_err_t err = mcpwm_comparator_set_compare_value(h->cmpr[i], ticks);
        if (err != ESP_OK) {
            return err;
        }
        /* Release any force level left over from disable/brake. */
        mcpwm_generator_set_force_level(h->gen_high[i], -1, true);
        mcpwm_generator_set_force_level(h->gen_low[i], -1, true);
    }
    return ESP_OK;
}

esp_err_t ps_focdrv_disable(ps_focdrv_handle_t h)
{
    if (h == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < 3; i++) {
        mcpwm_generator_set_force_level(h->gen_high[i], 0, true);
        mcpwm_generator_set_force_level(h->gen_low[i], 0, true);
    }
    return ESP_OK;
}

esp_err_t ps_focdrv_brake(ps_focdrv_handle_t h)
{
    if (h == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < 3; i++) {
        mcpwm_generator_set_force_level(h->gen_high[i], 0, true);
        mcpwm_generator_set_force_level(h->gen_low[i], 1, true);
    }
    return ESP_OK;
}
