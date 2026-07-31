/* ps_imu_filter — Mahony AHRS (gyro + accel) quaternion filter. FROZEN API.
 * Pure C99 (imu_mahony.c), host-tested. Output quaternion is the rotation from the
 * sensor frame to the world frame (gravity-aligned), Q15-packable via wire.ImuQuat. */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q0, q1, q2, q3;      /* unit quaternion, w-first */
    float two_kp, two_ki;
    float integ_x, integ_y, integ_z;
} ps_mahony_t;

/* Typical gains: kp = 0.5, ki = 0.0..0.1. */
void ps_mahony_init(ps_mahony_t *m, float kp, float ki);
/* gx/gy/gz in rad/s; ax/ay/az in any consistent unit (normalized internally;
 * accel correction is skipped when the norm is ~0). dt in seconds. */
void ps_mahony_update(ps_mahony_t *m, float gx, float gy, float gz,
                      float ax, float ay, float az, float dt);
void ps_mahony_get_quat(const ps_mahony_t *m, float q[4]);
/* Convenience: Q15 packing for wire.ImuQuat. */
void ps_mahony_get_quat_q15(const ps_mahony_t *m, int16_t q[4]);

#ifdef __cplusplus
}
#endif
