#include "imu_mpu6050.h"

#include "esp_log.h"

static const char *TAG = "node_limb";

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define I2C_TIMEOUT_MS   20
#define DEG_TO_RAD       0.017453293f

static esp_err_t wr_reg(imu_mpu6050_t *imu, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(imu->dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t rd_regs(imu_mpu6050_t *imu, uint8_t reg, uint8_t *dst, size_t n)
{
    return i2c_master_transmit_receive(imu->dev, &reg, 1, dst, n, I2C_TIMEOUT_MS);
}

esp_err_t imu_mpu6050_init(imu_mpu6050_t *imu, i2c_master_bus_handle_t bus, uint8_t addr)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    uint8_t who = 0;
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &imu->dev);
    if (err != ESP_OK) {
        return err;
    }

    err = wr_reg(imu, REG_PWR_MGMT_1, 0x01);          /* wake, PLL gyro-X clock */
    if (err != ESP_OK) {
        return err;
    }
    err = wr_reg(imu, REG_CONFIG, 0x03);              /* DLPF ~44 Hz */
    if (err == ESP_OK) {
        err = wr_reg(imu, REG_SMPLRT_DIV, 0x09);      /* 1 kHz / (1+9) = 100 Hz */
    }
    if (err == ESP_OK) {
        err = wr_reg(imu, REG_GYRO_CONFIG, 0x18);     /* FS_SEL=3: +-2000 dps */
    }
    if (err == ESP_OK) {
        err = wr_reg(imu, REG_ACCEL_CONFIG, 0x18);    /* AFS_SEL=3: +-16 g */
    }
    if (err != ESP_OK) {
        return err;
    }
    imu->gyr_rads_per_lsb = DEG_TO_RAD / 16.4f;       /* +-2000 dps scale */
    imu->acc_g_per_lsb = 1.0f / 2048.0f;              /* +-16 g scale */

    if (rd_regs(imu, REG_WHO_AM_I, &who, 1) == ESP_OK) {
        ESP_LOGI(TAG, "limb imu who_am_i=0x%02x", (unsigned)who);
    }
    return ESP_OK;
}

esp_err_t imu_mpu6050_read(imu_mpu6050_t *imu, float acc_g[3], float gyr_rads[3])
{
    uint8_t raw[14]; /* AX AY AZ TEMP GX GY GZ, big-endian pairs */
    esp_err_t err = rd_regs(imu, REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < 3; i++) {
        int16_t a = (int16_t)(((uint16_t)raw[2 * i] << 8) | raw[2 * i + 1]);
        int16_t g = (int16_t)(((uint16_t)raw[8 + 2 * i] << 8) | raw[8 + 2 * i + 1]);
        acc_g[i] = (float)a * imu->acc_g_per_lsb;
        gyr_rads[i] = (float)g * imu->gyr_rads_per_lsb;
    }
    return ESP_OK;
}
