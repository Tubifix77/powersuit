/* ps_pid — PI(D) controller with clamping anti-windup. FROZEN API. Pure C99, host-tested. */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp, ki, kd;
    float out_min, out_max;
    float integ;
    float prev_err;
    bool  primed;   /* first sample: skip derivative kick */
} ps_pid_t;

void  ps_pid_init(ps_pid_t *p, float kp, float ki, float kd, float out_min, float out_max);
float ps_pid_step(ps_pid_t *p, float err, float dt);
void  ps_pid_reset(ps_pid_t *p);

#ifdef __cplusplus
}
#endif
