#include "ps_pid.h"
#include "ps_test.h"

PS_TEST(output_respects_limits)
{
    ps_pid_t p;
    ps_pid_init(&p, 10.0f, 0.0f, 0.0f, -2.0f, 3.0f);
    PS_ASSERT_NEAR(ps_pid_step(&p, 100.0f, 0.01f), 3.0f, 1e-6);
    ps_pid_reset(&p);
    PS_ASSERT_NEAR(ps_pid_step(&p, -100.0f, 0.01f), -2.0f, 1e-6);
}

PS_TEST(no_derivative_kick_on_first_sample)
{
    ps_pid_t p;
    ps_pid_init(&p, 1.0f, 0.0f, 1.0f, -1e6f, 1e6f);
    /* A step into a cold controller must not produce a derivative spike. */
    PS_ASSERT_NEAR(ps_pid_step(&p, 10.0f, 0.01f), 10.0f, 1e-5);
    /* Steady error keeps the derivative at zero. */
    PS_ASSERT_NEAR(ps_pid_step(&p, 10.0f, 0.01f), 10.0f, 1e-5);
    /* A genuine change does produce one: (11-10)/0.01 = 100, plus kp*11. */
    PS_ASSERT_NEAR(ps_pid_step(&p, 11.0f, 0.01f), 111.0f, 1e-3);
}

PS_TEST(integral_frozen_while_saturated)
{
    ps_pid_t p;
    ps_pid_init(&p, 1.0f, 100.0f, 0.0f, -1.0f, 1.0f);
    for (int i = 0; i < 100; i++) {
        PS_ASSERT_NEAR(ps_pid_step(&p, 10.0f, 0.01f), 1.0f, 1e-6);
    }
    PS_ASSERT_NEAR(p.integ, 0.0f, 1e-6);

    /* Because nothing wound up, a small error responds proportionally at once
     * instead of staying pinned at the limit. */
    float out = ps_pid_step(&p, 0.1f, 0.01f);
    PS_ASSERT_TRUE(out < 0.5f);
}

PS_TEST(integral_recovers_immediately_on_reversal)
{
    ps_pid_t p;
    ps_pid_init(&p, 1.0f, 100.0f, 0.0f, -1.0f, 1.0f);
    for (int i = 0; i < 50; i++) {
        ps_pid_step(&p, 10.0f, 0.01f);
    }
    PS_ASSERT_NEAR(ps_pid_step(&p, -10.0f, 0.01f), -1.0f, 1e-6);
}

PS_TEST(integral_accumulates_when_unsaturated)
{
    ps_pid_t p;
    ps_pid_init(&p, 0.0f, 10.0f, 0.0f, -100.0f, 100.0f);
    for (int i = 0; i < 10; i++) {
        ps_pid_step(&p, 1.0f, 0.01f);
    }
    /* 10 steps x ki(10) x err(1) x dt(0.01) = 1.0, minus the newest step which
     * is added after the output is formed. */
    PS_ASSERT_NEAR(p.integ, 1.0f, 1e-5);
}

PS_TEST(converges_on_first_order_plant)
{
    ps_pid_t p;
    ps_pid_init(&p, 0.5f, 5.0f, 0.0f, -5.0f, 5.0f);
    float y = 0.0f;
    const float target = 1.0f;
    for (int i = 0; i < 400; i++) {
        float u = ps_pid_step(&p, target - y, 0.01f);
        y += (u - y) * 0.2f;
    }
    PS_ASSERT_NEAR(y, target, 0.01);
}

PS_TEST(reset_clears_state)
{
    ps_pid_t p;
    ps_pid_init(&p, 1.0f, 10.0f, 1.0f, -100.0f, 100.0f);
    ps_pid_step(&p, 5.0f, 0.01f);
    ps_pid_step(&p, 5.0f, 0.01f);
    ps_pid_reset(&p);
    PS_ASSERT_NEAR(p.integ, 0.0f, 1e-9);
    PS_ASSERT_NEAR(p.prev_err, 0.0f, 1e-9);
    PS_ASSERT_FALSE(p.primed);
}

PS_TEST_MAIN()
