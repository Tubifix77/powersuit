#include "ps_foc.h"
#include "ps_test.h"

#include <math.h>

#define SQRT3 1.7320508075688772f

PS_TEST(clarke_balanced_set)
{
    /* A balanced three-phase set of unit amplitude must map to a unit vector
     * rotating in the alpha-beta plane. */
    for (int deg = 0; deg < 360; deg += 15) {
        float th = (float)deg * (float)M_PI / 180.0f;
        float ia = cosf(th);
        float ib = cosf(th - 2.0f * (float)M_PI / 3.0f);
        ps_ab_t ab;
        ps_foc_clarke(ia, ib, &ab);
        PS_ASSERT_NEAR(ab.alpha, cosf(th), 1e-5);
        PS_ASSERT_NEAR(ab.beta, sinf(th), 1e-5);
    }
}

PS_TEST(park_inverse_roundtrip)
{
    for (int deg = 0; deg < 360; deg += 17) {
        float th = (float)deg * (float)M_PI / 180.0f;
        ps_ab_t ab = { 0.37f, -0.82f };
        ps_dq_t dq;
        ps_ab_t back;
        ps_foc_park(&ab, th, &dq);
        ps_foc_inv_park(&dq, th, &back);
        PS_ASSERT_NEAR(back.alpha, ab.alpha, 1e-5);
        PS_ASSERT_NEAR(back.beta, ab.beta, 1e-5);
    }
}

PS_TEST(park_dc_alignment)
{
    /* A vector aligned with the rotor angle is pure d-axis, no q component. */
    float th = 0.7f;
    ps_ab_t ab = { cosf(th) * 2.0f, sinf(th) * 2.0f };
    ps_dq_t dq;
    ps_foc_park(&ab, th, &dq);
    PS_ASSERT_NEAR(dq.d, 2.0f, 1e-5);
    PS_ASSERT_NEAR(dq.q, 0.0f, 1e-5);
}

PS_TEST(svpwm_zero_vector_is_midpoint)
{
    ps_ab_t v = { 0.0f, 0.0f };
    float duty[3];
    ps_foc_svpwm(&v, 24.0f, duty);
    for (int i = 0; i < 3; i++) {
        PS_ASSERT_NEAR(duty[i], 0.5f, 1e-6);
    }
}

PS_TEST(svpwm_linear_region_reaches_rails_without_clipping)
{
    /* The SVPWM linear limit is |v_phase| = v_bus / sqrt(3); at exactly that
     * amplitude the duties should touch 0 and 1 over a full revolution but
     * never need clamping. */
    const float v_bus = 24.0f;
    const float amp = v_bus / SQRT3;
    float seen_max = 0.0f, seen_min = 1.0f;
    for (int deg = 0; deg < 360; deg++) {
        float th = (float)deg * (float)M_PI / 180.0f;
        ps_ab_t v = { amp * cosf(th), amp * sinf(th) };
        float duty[3];
        ps_foc_svpwm(&v, v_bus, duty);
        for (int i = 0; i < 3; i++) {
            PS_ASSERT_TRUE(duty[i] >= -1e-6f && duty[i] <= 1.0f + 1e-6f);
            if (duty[i] > seen_max) {
                seen_max = duty[i];
            }
            if (duty[i] < seen_min) {
                seen_min = duty[i];
            }
        }
    }
    PS_ASSERT_NEAR(seen_max, 1.0f, 1e-3);
    PS_ASSERT_NEAR(seen_min, 0.0f, 1e-3);
}

PS_TEST(svpwm_overmodulation_clamps)
{
    ps_ab_t v = { 100.0f, 0.0f };
    float duty[3];
    ps_foc_svpwm(&v, 24.0f, duty);
    for (int i = 0; i < 3; i++) {
        PS_ASSERT_TRUE(duty[i] >= 0.0f && duty[i] <= 1.0f);
    }
}

PS_TEST(current_loop_tracks_iq_target)
{
    /* Closed loop against a discrete RL winding model (R = 1 ohm, L = 5 mH) at
     * the contract's 1 kHz FOC rate. theta is held at zero so the d/q axes line
     * up with alpha/beta and the plant stays readable. */
    const float R = 1.0f, L = 5e-3f, dt = 1e-3f, v_bus = 24.0f;
    const float iq_target = 2.0f;
    ps_foc_ctl_t ctl;
    ps_foc_ctl_init(&ctl, 2.0f, 500.0f, 12.0f);

    float id = 0.0f, iq = 0.0f;
    for (int k = 0; k < 300; k++) {
        float ia = id;
        float ib = (iq * SQRT3 - id) * 0.5f;
        float duty[3];
        ps_foc_ctl_step(&ctl, ia, ib, 0.0f, 0.0f, iq_target, v_bus, dt, duty);

        /* Recover the applied vector from the duties (common mode cancels in a
         * floating-neutral winding). */
        float va = (duty[0] - 0.5f) * v_bus;
        float vb = (duty[1] - 0.5f) * v_bus;
        float vc = (duty[2] - 0.5f) * v_bus;
        float v_alpha = (2.0f * va - vb - vc) / 3.0f;
        float v_beta = (vb - vc) / SQRT3;

        id += (v_alpha - R * id) * (dt / L);
        iq += (v_beta - R * iq) * (dt / L);
    }
    PS_ASSERT_NEAR(iq, iq_target, 0.05);
    PS_ASSERT_NEAR(id, 0.0f, 0.05);
}

PS_TEST(ctl_reset_clears_both_axes)
{
    ps_foc_ctl_t ctl;
    ps_foc_ctl_init(&ctl, 1.0f, 100.0f, 12.0f);
    float duty[3];
    ps_foc_ctl_step(&ctl, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 24.0f, 1e-3f, duty);
    ps_foc_ctl_reset(&ctl);
    PS_ASSERT_NEAR(ctl.pid_d.integ, 0.0f, 1e-9);
    PS_ASSERT_NEAR(ctl.pid_q.integ, 0.0f, 1e-9);
}

PS_TEST_MAIN()
