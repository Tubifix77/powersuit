/* ps_imu_filter — Mahony AHRS (gyro + accel), ps_imu_filter.h contract.
 * Pure C99: no RTOS, no IDF includes; host-tested by tests/host/test_imu_filter.c.
 * The accelerometer measures specific force, so a device at rest reads +1 g along
 * its local "up"; the filter drives the quaternion until the gravity direction it
 * predicts matches that measurement. */
#include "ps_imu_filter.h"

#include <math.h>

void ps_mahony_init(ps_mahony_t *m, float kp, float ki)
{
    m->q0 = 1.0f;
    m->q1 = 0.0f;
    m->q2 = 0.0f;
    m->q3 = 0.0f;
    m->two_kp = 2.0f * kp;
    m->two_ki = 2.0f * ki;
    m->integ_x = 0.0f;
    m->integ_y = 0.0f;
    m->integ_z = 0.0f;
}

void ps_mahony_update(ps_mahony_t *m, float gx, float gy, float gz,
                      float ax, float ay, float az, float dt)
{
    float norm_sq = ax * ax + ay * ay + az * az;

    /* Free-fall or a dead sensor carries no attitude information: integrate the
     * gyro alone rather than feeding a garbage correction into the loop. */
    if (norm_sq > 1e-12f) {
        float recip = 1.0f / sqrtf(norm_sq);
        ax *= recip;
        ay *= recip;
        az *= recip;

        /* Gravity direction predicted by the current quaternion (half-scale). */
        float halfvx = m->q1 * m->q3 - m->q0 * m->q2;
        float halfvy = m->q0 * m->q1 + m->q2 * m->q3;
        float halfvz = m->q0 * m->q0 - 0.5f + m->q3 * m->q3;

        /* Error = measured x predicted, expressed in the sensor frame. */
        float halfex = ay * halfvz - az * halfvy;
        float halfey = az * halfvx - ax * halfvz;
        float halfez = ax * halfvy - ay * halfvx;

        if (m->two_ki > 0.0f) {
            m->integ_x += m->two_ki * halfex * dt;
            m->integ_y += m->two_ki * halfey * dt;
            m->integ_z += m->two_ki * halfez * dt;
            gx += m->integ_x;
            gy += m->integ_y;
            gz += m->integ_z;
        } else {
            m->integ_x = 0.0f;
            m->integ_y = 0.0f;
            m->integ_z = 0.0f;
        }

        gx += m->two_kp * halfex;
        gy += m->two_kp * halfey;
        gz += m->two_kp * halfez;
    }

    /* Quaternion derivative, integrated with the half-angle factor folded in. */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = m->q0;
    float qb = m->q1;
    float qc = m->q2;
    float qd = m->q3;
    m->q0 += -qb * gx - qc * gy - qd * gz;
    m->q1 += qa * gx + qc * gz - qd * gy;
    m->q2 += qa * gy - qb * gz + qd * gx;
    m->q3 += qa * gz + qb * gy - qc * gx;

    float n = sqrtf(m->q0 * m->q0 + m->q1 * m->q1 + m->q2 * m->q2 + m->q3 * m->q3);
    if (n > 1e-12f) {
        float recip = 1.0f / n;
        m->q0 *= recip;
        m->q1 *= recip;
        m->q2 *= recip;
        m->q3 *= recip;
    } else {
        m->q0 = 1.0f;
        m->q1 = m->q2 = m->q3 = 0.0f;
    }
}

void ps_mahony_get_quat(const ps_mahony_t *m, float q[4])
{
    q[0] = m->q0;
    q[1] = m->q1;
    q[2] = m->q2;
    q[3] = m->q3;
}

void ps_mahony_get_quat_q15(const ps_mahony_t *m, int16_t q[4])
{
    const float src[4] = { m->q0, m->q1, m->q2, m->q3 };
    for (int i = 0; i < 4; i++) {
        float v = src[i] * 32767.0f;
        v = (v >= 0.0f) ? (v + 0.5f) : (v - 0.5f);   /* round half away from zero */
        if (v > 32767.0f) {
            v = 32767.0f;
        } else if (v < -32768.0f) {
            v = -32768.0f;
        }
        q[i] = (int16_t)v;
    }
}
