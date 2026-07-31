/* ps_pid — PI(D) controller with clamping anti-windup (ps_pid.h contract).
 * Pure C99: no RTOS, no IDF includes; host-tested by tests/host/test_pid.c. */
#include "ps_pid.h"

void ps_pid_init(ps_pid_t *p, float kp, float ki, float kd, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    ps_pid_reset(p);
}

void ps_pid_reset(ps_pid_t *p)
{
    p->integ = 0.0f;
    p->prev_err = 0.0f;
    p->primed = false;
}

float ps_pid_step(ps_pid_t *p, float err, float dt)
{
    /* Derivative on error; the primed flag suppresses the first-sample kick
     * (prev_err starts undefined relative to the running signal). */
    float d = 0.0f;
    if (p->primed && dt > 0.0f) {
        d = p->kd * (err - p->prev_err) / dt;
    }
    p->prev_err = err;
    p->primed = true;

    float di = (dt > 0.0f) ? p->ki * err * dt : 0.0f;
    float unsat = p->kp * err + p->integ + di + d;

    float out = unsat;
    if (out > p->out_max) {
        out = p->out_max;
    } else if (out < p->out_min) {
        out = p->out_min;
    }

    /* Clamping anti-windup: the integral is frozen while the unsaturated
     * output is pushing further into the active limit; it keeps integrating
     * as soon as the error reverses, so recovery is immediate. */
    bool windup = (unsat > p->out_max && err > 0.0f) ||
                  (unsat < p->out_min && err < 0.0f);
    if (!windup) {
        p->integ += di;
    }
    return out;
}
