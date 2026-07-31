/* ps_foc — field-oriented-control math (ps_foc.h contract).
 * Pure C99: no RTOS, no IDF includes; host-tested by tests/host/test_foc_math.c.
 * Conventions: phase currents in amps, voltages in volts, theta = electrical
 * angle in radians, duties normalized to [0,1]. */
#include "ps_foc.h"

#include <math.h>

#define PS_SQRT3      1.7320508075688772f
#define PS_INV_SQRT3  0.5773502691896258f
#define PS_SQRT3_2    0.8660254037844386f

void ps_foc_clarke(float ia, float ib, ps_ab_t *out)
{
    /* Balanced three-phase (ic = -ia - ib) reduces the amplitude-invariant
     * Clarke transform to these two terms. */
    out->alpha = ia;
    out->beta = (ia + 2.0f * ib) * PS_INV_SQRT3;
}

void ps_foc_park(const ps_ab_t *ab, float theta, ps_dq_t *out)
{
    float c = cosf(theta);
    float s = sinf(theta);
    out->d = ab->alpha * c + ab->beta * s;
    out->q = -ab->alpha * s + ab->beta * c;
}

void ps_foc_inv_park(const ps_dq_t *dq, float theta, ps_ab_t *out)
{
    float c = cosf(theta);
    float s = sinf(theta);
    out->alpha = dq->d * c - dq->q * s;
    out->beta = dq->d * s + dq->q * c;
}

void ps_foc_svpwm(const ps_ab_t *v_ab, float v_bus, float duty[3])
{
    if (v_bus <= 0.0f) {
        duty[0] = duty[1] = duty[2] = 0.5f;
        return;
    }

    /* Inverse Clarke to per-phase voltages. */
    float va = v_ab->alpha;
    float vb = -0.5f * v_ab->alpha + PS_SQRT3_2 * v_ab->beta;
    float vc = -0.5f * v_ab->alpha - PS_SQRT3_2 * v_ab->beta;

    /* Min-max (midpoint) injection: shifting all three phases by the common
     * mode leaves line-to-line voltages untouched but extends the linear range
     * to |v_phase| = v_bus / sqrt(3), which is what makes this SVPWM rather
     * than sine PWM. */
    float vmax = va > vb ? va : vb;
    if (vc > vmax) {
        vmax = vc;
    }
    float vmin = va < vb ? va : vb;
    if (vc < vmin) {
        vmin = vc;
    }
    float vcom = 0.5f * (vmax + vmin);

    float inv_bus = 1.0f / v_bus;
    duty[0] = 0.5f + (va - vcom) * inv_bus;
    duty[1] = 0.5f + (vb - vcom) * inv_bus;
    duty[2] = 0.5f + (vc - vcom) * inv_bus;

    for (int i = 0; i < 3; i++) {
        if (duty[i] > 1.0f) {
            duty[i] = 1.0f;
        } else if (duty[i] < 0.0f) {
            duty[i] = 0.0f;
        }
    }
}

void ps_foc_ctl_init(ps_foc_ctl_t *c, float kp, float ki, float v_limit)
{
    c->v_limit = v_limit;
    ps_pid_init(&c->pid_d, kp, ki, 0.0f, -v_limit, v_limit);
    ps_pid_init(&c->pid_q, kp, ki, 0.0f, -v_limit, v_limit);
}

void ps_foc_ctl_reset(ps_foc_ctl_t *c)
{
    ps_pid_reset(&c->pid_d);
    ps_pid_reset(&c->pid_q);
}

void ps_foc_ctl_step(ps_foc_ctl_t *c, float ia, float ib, float theta,
                     float id_target, float iq_target, float v_bus,
                     float dt, float duty[3])
{
    ps_ab_t i_ab;
    ps_dq_t i_dq;
    ps_foc_clarke(ia, ib, &i_ab);
    ps_foc_park(&i_ab, theta, &i_dq);

    ps_dq_t v_dq;
    v_dq.d = ps_pid_step(&c->pid_d, id_target - i_dq.d, dt);
    v_dq.q = ps_pid_step(&c->pid_q, iq_target - i_dq.q, dt);

    ps_ab_t v_ab;
    ps_foc_inv_park(&v_dq, theta, &v_ab);
    ps_foc_svpwm(&v_ab, v_bus, duty);
}
