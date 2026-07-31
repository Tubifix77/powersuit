/* ps_focdrv — three-phase MCPWM output stage for one BLDC joint.
 * Center-aligned complementary PWM with hardware dead time; the low side is
 * derived from the high side by the dead-time module, so software can never
 * command a shoot-through pair.
 *
 * Safety contract: ps_focdrv_set_duty() is only reached through the limb app's
 * actuation gate (ps_safety_can_actuate). Disable/brake are always permitted —
 * they are the safe directions. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int group_id;          /* ESP32-S3 has two MCPWM groups: one joint each */
    int gpio_uh, gpio_ul;  /* phase U high/low side */
    int gpio_vh, gpio_vl;
    int gpio_wh, gpio_wl;
    uint32_t pwm_hz;       /* 0 = default 20 kHz */
    uint32_t deadtime_ns;  /* 0 = default 200 ns */
} ps_focdrv_config_t;

typedef struct ps_focdrv_ctx *ps_focdrv_handle_t;

esp_err_t ps_focdrv_init(const ps_focdrv_config_t *cfg, ps_focdrv_handle_t *out);

/* Duties in [0,1] per phase; values outside are clamped. */
esp_err_t ps_focdrv_set_duty(ps_focdrv_handle_t h, const float duty[3]);

/* Free-wheeling: all six switches off. The joint back-drives with no resistance. */
esp_err_t ps_focdrv_disable(ps_focdrv_handle_t h);

/* Active damping: all low-side switches on, shorting the windings so motion is
 * resisted by the motor's own back-EMF (Passive Compliance, docs/safety.md §2). */
esp_err_t ps_focdrv_brake(ps_focdrv_handle_t h);

#ifdef __cplusplus
}
#endif
