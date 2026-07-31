#include "ps_imu_filter.h"
#include "ps_test.h"

#include <math.h>

/* Gravity direction the filter currently predicts, in the sensor frame. */
static void predicted_up(const ps_mahony_t *m, float v[3])
{
    v[0] = 2.0f * (m->q1 * m->q3 - m->q0 * m->q2);
    v[1] = 2.0f * (m->q0 * m->q1 + m->q2 * m->q3);
    v[2] = m->q0 * m->q0 - m->q1 * m->q1 - m->q2 * m->q2 + m->q3 * m->q3;
}

PS_TEST(converges_to_measured_gravity)
{
    /* Start level, feed a constant 30-degree tilt: the filter must rotate its
     * estimate onto the measured up-vector. */
    ps_mahony_t m;
    ps_mahony_init(&m, 2.0f, 0.0f);
    const float ax = 0.5f, ay = 0.0f, az = 0.8660254f;
    const float dt = 1.0f / 250.0f;
    for (int i = 0; i < 1250; i++) {   /* 5 s at 250 Hz */
        ps_mahony_update(&m, 0.0f, 0.0f, 0.0f, ax, ay, az, dt);
    }
    float v[3];
    predicted_up(&m, v);
    float dot = v[0] * ax + v[1] * ay + v[2] * az;
    PS_ASSERT_TRUE(dot > cosf(2.0f * (float)M_PI / 180.0f));
}

PS_TEST(integrates_pure_gyro_rotation)
{
    /* No accelerometer signal: the update must fall back to dead reckoning
     * instead of injecting a correction from a zero vector. */
    ps_mahony_t m;
    ps_mahony_init(&m, 2.0f, 0.0f);
    const float rate = (float)M_PI / 2.0f;   /* 90 deg/s about z */
    const float dt = 1.0f / 250.0f;
    for (int i = 0; i < 250; i++) {
        ps_mahony_update(&m, 0.0f, 0.0f, rate, 0.0f, 0.0f, 0.0f, dt);
    }
    float q[4];
    ps_mahony_get_quat(&m, q);
    float yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                       1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
    PS_ASSERT_NEAR(yaw * 180.0f / (float)M_PI, 90.0f, 3.0);
}

PS_TEST(quaternion_stays_normalized)
{
    ps_mahony_t m;
    ps_mahony_init(&m, 1.0f, 0.05f);
    const float dt = 1.0f / 250.0f;
    for (int i = 0; i < 2000; i++) {
        float t = (float)i * dt;
        ps_mahony_update(&m, 0.4f * sinf(t), 0.3f * cosf(t), 0.2f,
                         0.1f * sinf(t), 0.2f, 0.97f, dt);
        float q[4];
        ps_mahony_get_quat(&m, q);
        float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        PS_ASSERT_NEAR(n, 1.0f, 1e-3);
    }
}

PS_TEST(freefall_does_not_corrupt_estimate)
{
    /* Zero-g must leave the attitude untouched rather than slew it somewhere. */
    ps_mahony_t m;
    ps_mahony_init(&m, 2.0f, 0.0f);
    for (int i = 0; i < 500; i++) {
        ps_mahony_update(&m, 0.0f, 0.0f, 0.0f, 0.3f, 0.0f, 0.95f, 1.0f / 250.0f);
    }
    float before[4], after[4];
    ps_mahony_get_quat(&m, before);
    for (int i = 0; i < 250; i++) {
        ps_mahony_update(&m, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f / 250.0f);
    }
    ps_mahony_get_quat(&m, after);
    for (int i = 0; i < 4; i++) {
        PS_ASSERT_NEAR(after[i], before[i], 1e-5);
    }
}

PS_TEST(q15_export_identity_and_clamp)
{
    ps_mahony_t m;
    ps_mahony_init(&m, 1.0f, 0.0f);
    int16_t q[4];
    ps_mahony_get_quat_q15(&m, q);
    PS_ASSERT_EQ_INT(q[0], 32767);
    PS_ASSERT_EQ_INT(q[1], 0);
    PS_ASSERT_EQ_INT(q[2], 0);
    PS_ASSERT_EQ_INT(q[3], 0);

    /* Round-trip a half-turn about x: (0, 1, 0, 0). */
    m.q0 = 0.0f;
    m.q1 = -1.0f;
    m.q2 = 0.0f;
    m.q3 = 0.0f;
    ps_mahony_get_quat_q15(&m, q);
    PS_ASSERT_EQ_INT(q[0], 0);
    PS_ASSERT_EQ_INT(q[1], -32767);
}

PS_TEST_MAIN()
