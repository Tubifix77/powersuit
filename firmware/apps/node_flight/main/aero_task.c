/* aero_task — core 1: torso IMU fusion + pitot aero engine (Node 5 is the
 * suit's base IMU, network-map §12.6).
 * 100 Hz: IMU read -> Mahony -> TELEM IMU_QUAT/IMU_ACC/IMU_GYR.
 * 50 Hz:  pitot ADC -> q [Pa] -> IAS = sqrt(2q/rho) -> TELEM AERO_STATE.
 * 20 Hz:  FLAP_STATE — but only flaps that changed; unchanged flaps are
 *         re-sent at 2 Hz keepalive. Rationale: 12 flaps at a flat 20 Hz is
 *         240 fps and the whole node budget is ~400 fps (network-map §10);
 *         change-driven emission keeps the steady-state well under it. */
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

#include "powersuit_proto/wire.h"
#include "ps_imu_filter.h"
#include "node_flight.h"
#include "imu_mpu6050.h"

static const char *TAG = "node_flight";

#define AERO_TICK_MS      10u          /* 100 Hz base rate */
#define AERO_F_NO_VANE    (1u << 0)    /* AERO_STATE.flags: aoa_cdeg invalid */
#define DEG_TO_RAD        0.017453293f

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;       /* NULL when curve fitting unavailable */

static void pitot_init(void)
{
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = PS_PITOT_ADC_UNIT };
    adc_oneshot_chan_cfg_t ccfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_cali_curve_fitting_config_t cal = {
        .unit_id = PS_PITOT_ADC_UNIT,
        .chan = PS_PITOT_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &s_adc));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, PS_PITOT_ADC_CHANNEL, &ccfg));
    if (adc_cali_create_scheme_curve_fitting(&cal, &s_cali) != ESP_OK) {
        s_cali = NULL; /* fall back to a nominal LSB->mV slope */
        ESP_LOGW(TAG, "adc calibration unavailable, using nominal slope");
    }
}

static float pitot_read_mv(void)
{
    int raw = 0;
    if (adc_oneshot_read(s_adc, PS_PITOT_ADC_CHANNEL, &raw) != ESP_OK) {
        return PS_PITOT_ZERO_MV; /* read fault reads as zero airspeed */
    }
    if (s_cali != NULL) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) {
            return (float)mv;
        }
    }
    return (float)raw * (3300.0f / 4095.0f); /* nominal 12-bit full scale */
}

static uint16_t sat_u16(float v)
{
    if (v <= 0.0f) {
        return 0;
    }
    if (v >= 65535.0f) {
        return 65535;
    }
    return (uint16_t)v;
}

static int16_t sat_i16(float v)
{
    if (v <= -32768.0f) {
        return -32768;
    }
    if (v >= 32767.0f) {
        return 32767;
    }
    return (int16_t)v;
}

static void aero_task(void *arg)
{
    static imu_mpu6050_t imu;
    static ps_mahony_t mahony;
    static flight_flap_state_t last_sent[PS_FLAP_COUNT];
    static flight_flap_state_t snap[PS_FLAP_COUNT];
    i2c_master_bus_handle_t bus = NULL;
    bool imu_ok;
    float q_filt = 0.0f;
    uint32_t tick = 0;
    TickType_t wake;

    (void)arg;

    i2c_master_bus_config_t bcfg = {
        .i2c_port = -1,
        .sda_io_num = PS_BOARD_I2C_SDA_GPIO,
        .scl_io_num = PS_BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bcfg, &bus));
    imu_ok = (imu_mpu6050_init(&imu, bus, PS_BOARD_IMU_ADDR) == ESP_OK);
    if (!imu_ok) {
        ESP_LOGE(TAG, "torso imu init failed; imu telemetry suppressed");
    }
    pitot_init();
    ps_mahony_init(&mahony, 0.5f, 0.01f);
    memset(last_sent, 0xFF, sizeof(last_sent)); /* force first FLAP_STATE burst */

    wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(AERO_TICK_MS));
        tick++;

        /* --- 100 Hz IMU + Mahony + TELEM --- */
        if (imu_ok) {
            float a[3], g[3];
            if (imu_mpu6050_read(&imu, a, g) == ESP_OK) {
                ps_mahony_update(&mahony,
                                 g[0] * DEG_TO_RAD, g[1] * DEG_TO_RAD, g[2] * DEG_TO_RAD,
                                 a[0], a[1], a[2], (float)AERO_TICK_MS / 1000.0f);
                int16_t q15[4];
                ps_mahony_get_quat_q15(&mahony, q15);
                ps_imu_quat_t quat = { q15[0], q15[1], q15[2], q15[3] };
                ps_imu_acc_t acc = {
                    sat_i16(a[0] * 1000.0f), sat_i16(a[1] * 1000.0f),
                    sat_i16(a[2] * 1000.0f), 0
                };
                ps_imu_gyr_t gyr = {
                    sat_i16(g[0] * 10.0f), sat_i16(g[1] * 10.0f),
                    sat_i16(g[2] * 10.0f), 0
                };
                flight_telem_send(PS_CLS_TELEM, PS_T_IMU_QUAT, &quat, sizeof(quat));
                flight_telem_send(PS_CLS_TELEM, PS_T_IMU_ACC, &acc, sizeof(acc));
                flight_telem_send(PS_CLS_TELEM, PS_T_IMU_GYR, &gyr, sizeof(gyr));
            }
        }

        /* --- 50 Hz pitot -> AERO_STATE --- */
        if ((tick % 2u) == 0u) {
            float mv = pitot_read_mv();
            float q_pa = (mv - PS_PITOT_ZERO_MV) * PS_PITOT_PA_PER_MV;
            if (q_pa < 0.0f) {
                q_pa = 0.0f; /* single-port probe: no reverse flow */
            }
            q_filt += 0.2f * (q_pa - q_filt); /* 1-pole smoothing at 50 Hz */
            float rho = flight_param_rho();
            float ias_ms = sqrtf(fmaxf(0.0f, 2.0f * q_filt / rho));
            ps_aero_state_t aero = {
                .ias_cms = sat_u16(ias_ms * 100.0f),
                .q_pa = sat_u16(q_filt),
                .aoa_cdeg = 0,                /* no AoA vane fitted */
                .flags = AERO_F_NO_VANE,
            };
            flight_telem_send(PS_CLS_TELEM, PS_T_AERO_STATE, &aero, sizeof(aero));
        }

        /* --- 20 Hz FLAP_STATE, change-driven (+2 Hz keepalive) --- */
        if ((tick % 5u) == 0u) {
            const bool keepalive = ((tick % 500u) == 0u); /* every 5 s: full set
                * at the 20 Hz slot; per-flap staggering below yields 2 Hz. */
            flight_flap_state_snapshot(snap);
            for (int i = 0; i < PS_FLAP_COUNT; i++) {
                bool changed = (snap[i].pos_pm != last_sent[i].pos_pm) ||
                               (snap[i].target_pm != last_sent[i].target_pm) ||
                               (snap[i].at_limit != last_sent[i].at_limit);
                /* stagger: flap i keepalives on 20 Hz slot (tick/5) mod 10 == i%10
                 * -> each flap re-sent every 500 ms = 2 Hz */
                bool slot = (((tick / 5u) % 10u) == (uint32_t)(i % 10));
                if (changed || slot || keepalive) {
                    ps_flap_state_t fs = {
                        .flap = (uint8_t)i,
                        .flags = (uint8_t)(snap[i].at_limit ? 0x01 : 0x00),
                        .pos_pm = snap[i].pos_pm,
                        .target_pm = snap[i].target_pm,
                        .rsvd = 0,
                    };
                    flight_telem_send(PS_CLS_TELEM, PS_T_FLAP_STATE, &fs, sizeof(fs));
                    last_sent[i] = snap[i];
                }
            }
        }
    }
}

void aero_task_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(aero_task, "aero", 6144, NULL, 15, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "aero task create failed");
    }
}
