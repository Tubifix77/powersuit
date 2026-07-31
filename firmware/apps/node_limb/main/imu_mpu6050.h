/* imu_mpu6050 — minimal MPU6050-class 6-DOF driver on the IDF v5.5 i2c_master
 * API. Limb-mounted IMU feeding the 250 Hz Mahony filter in control_task.c.
 * The register map here is a stand-in for bench bring-up; a production IMU
 * swap (different part, same wiring) only ever touches this file. */
#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_dev_handle_t dev;
    float acc_g_per_lsb;    /* set by init from the configured full-scale */
    float gyr_rads_per_lsb;
} imu_mpu6050_t;

/* Wakes the part, configures +-16 g / +-2000 dps / ~44 Hz DLPF. */
esp_err_t imu_mpu6050_init(imu_mpu6050_t *imu, i2c_master_bus_handle_t bus, uint8_t addr);

/* One burst read: acc in g, gyro in rad/s (sensor frame). */
esp_err_t imu_mpu6050_read(imu_mpu6050_t *imu, float acc_g[3], float gyr_rads[3]);

#ifdef __cplusplus
}
#endif
