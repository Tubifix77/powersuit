/* ps_foc — field-oriented-control math. FROZEN API. Pure C99 (foc_math.c), host-tested;
 * the MCPWM output stage and encoder readout are separate ESP-only sources in ps_ctl. */
#pragma once

#include "ps_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float alpha, beta; } ps_ab_t;
typedef struct { float d, q; } ps_dq_t;

/* Clarke: 2 measured phase currents (a, b; c = -a-b) -> alpha/beta. */
void ps_foc_clarke(float ia, float ib, ps_ab_t *out);
/* Park / inverse Park; theta = electrical angle in radians. */
void ps_foc_park(const ps_ab_t *ab, float theta, ps_dq_t *out);
void ps_foc_inv_park(const ps_dq_t *dq, float theta, ps_ab_t *out);
/* Space-vector modulation: v_alpha/beta (volts), bus voltage -> 3 duties in [0,1].
 * Min/max (midpoint) injection; magnitudes beyond the linear region saturate. */
void ps_foc_svpwm(const ps_ab_t *v_ab, float v_bus, float duty[3]);

/* dq current controller (one PI per axis) producing voltage targets. */
typedef struct {
    ps_pid_t pid_d, pid_q;
    float    v_limit;   /* per-axis output clamp, volts */
} ps_foc_ctl_t;

void ps_foc_ctl_init(ps_foc_ctl_t *c, float kp, float ki, float v_limit);
/* One 1 kHz step: measured currents + electrical angle + targets -> phase duties. */
void ps_foc_ctl_step(ps_foc_ctl_t *c, float ia, float ib, float theta,
                     float id_target, float iq_target, float v_bus,
                     float dt, float duty[3]);
void ps_foc_ctl_reset(ps_foc_ctl_t *c);

#ifdef __cplusplus
}
#endif
