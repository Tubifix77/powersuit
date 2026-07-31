/* imu_mpu6050 — minimal MPU6050-class 6-DOF driver on the IDF v5.5 i2c_master
 * API. Torso-rigid base IMU for the EKF (network-map §12.6). */
#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_dev_handle_t dev;
    float acc_g_per_lsb;   /* set by init from the configured full-scale */
    float gyr_dps_per_lsb;
} imu_mpu6050_t;

/* Wakes the part, configures +-8 g / +-1000 dps / 44 Hz DLPF / 100 Hz ODR. */
esp_err_t imu_mpu6050_init(imu_mpu6050_t *imu, i2c_master_bus_handle_t bus, uint8_t addr);

/* One burst read: acc in g, gyro in deg/s (sensor frame). */
esp_err_t imu_mpu6050_read(imu_mpu6050_t *imu, float acc_g[3], float gyr_dps[3]);

#ifdef __cplusplus
}
#endif
