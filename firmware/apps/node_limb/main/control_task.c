/* control_task — Core 1: 1 kHz FOC current loop, 250 Hz IMU/force sampling.
 *
 * Per joint per 1 kHz tick: encoder -> phase currents -> mode-specific
 * cascade -> ps_foc_ctl_step -> joint_write(). joint_write() is the ONLY
 * function that touches ps_focdrv_*: it checks ps_safety_can_actuate() itself
 * (no caller may assume it has already been checked) and free-wheels or
 * actively damps whenever actuation is not permitted or the joint's own mode
 * is PASSIVE (docs/safety.md section 2, ps_focdrv.h contract).
 *
 * Every 4th tick (250 Hz): Mahony update from the limb IMU and strain-gauge
 * force sampling, published into the shared sensor snapshot that comms_task
 * re-emits at its own 100 Hz TELEM cadence.
 *
 * The gptimer ISR only notifies this task (vTaskNotifyGiveFromISR); all real
 * work happens in task context so it can call ps_safety_can_actuate()/
 * ps_safety_raise_estop(), which are not ISR-safe. */
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

#include "powersuit_proto/wire.h"
#include "ps_pid.h"
#include "ps_foc.h"
#include "ps_focdrv.h"
#include "ps_imu_filter.h"
#include "ps_safety.h"

#include "limb_tasks.h"
#include "imu_mpu6050.h"

static const char *TAG = "node_limb";

#define CTRL_TICK_HZ          1000u
#define CTRL_TICK_S           (1.0f / (float)CTRL_TICK_HZ)
#define CTRL_GPTIMER_RES_HZ   1000000u
#define SENSOR_TICK_DIV        4u      /* 1 kHz / 4 = 250 Hz */
#define OVERCURRENT_TRIP_TICKS 5u      /* 5 ms at 1 kHz, per contract */
#define DAMP_THRESHOLD_RAD_S   5.0f    /* motor-shaft speed above which PASSIVE
                                         * actively damps instead of free-wheeling */
#define TWO_PI                 6.283185307f
#define RAD_S_TO_DDPS          572.9578f   /* rad/s -> 0.1 deg/s (wire.h ddps) */
#define STRAIN_TARE_SAMPLES    32u

/* ---------------- AS5047-style absolute magnetic encoder ----------------
 * 14-bit angle, single 16-bit SPI transfer (mode 1, "read angle, no CRC"
 * convenience frame 0xFFFF). Board-specific, so it lives here rather than in
 * ps_ctl (ps_foc.h contract header comment). ~60 lines including init. */
typedef struct {
    spi_device_handle_t dev;
    uint16_t last_raw;      /* most recent 14-bit sample, for electrical angle */
    bool     have_last;
    float    angle_rad;     /* multi-turn unwrapped motor-shaft angle */
    float    vel_rad_s;     /* low-pass filtered motor-shaft velocity */
} limb_encoder_t;

static esp_err_t encoder_init(limb_encoder_t *e, spi_host_device_t host, int cs_gpio)
{
    memset(e, 0, sizeof(*e));
    spi_device_interface_config_t dcfg = {
        .mode = 1,                     /* AS5047 SPI mode: CPOL=0, CPHA=1 */
        .clock_speed_hz = 10 * 1000 * 1000,
        .spics_io_num = cs_gpio,
        .queue_size = 1,
    };
    return spi_bus_add_device(host, &dcfg, &e->dev);
}

static uint16_t encoder_read_raw14(limb_encoder_t *e)
{
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 16,
        .tx_data = { 0xFF, 0xFF, 0, 0 },
    };
    if (spi_device_polling_transmit(e->dev, &t) != ESP_OK) {
        return e->last_raw;             /* hold last good sample on a bus fault */
    }
    uint16_t word = ((uint16_t)t.rx_data[0] << 8) | t.rx_data[1];
    return word & 0x3FFFu;              /* bits 15:14 are error/parity, ignored */
}

/* Advances multi-turn angle + a 1-pole velocity estimate. Call once per 1 kHz
 * tick; dt is CTRL_TICK_S. */
static void encoder_update(limb_encoder_t *e, float dt)
{
    uint16_t raw = encoder_read_raw14(e);
    if (e->have_last) {
        int32_t delta = (int32_t)raw - (int32_t)e->last_raw;
        if (delta > 8192) {
            delta -= 16384;             /* wrapped backward through zero */
        } else if (delta < -8192) {
            delta += 16384;             /* wrapped forward through zero */
        }
        float dtheta = (float)delta * (TWO_PI / 16384.0f);
        e->angle_rad += dtheta;
        float inst_vel = dtheta / dt;
        e->vel_rad_s += 0.25f * (inst_vel - e->vel_rad_s);   /* ~60 Hz corner */
    }
    e->last_raw = raw;
    e->have_last = true;
}

/* Electrical angle for FOC commutation: mechanical single-turn angle scaled
 * by pole pairs, wrapped to [0, 2*pi). Uses the raw sample, not the unwrapped
 * multi-turn angle -- commutation only ever needs the current mechanical
 * position within one revolution. */
static float encoder_electrical_theta(uint16_t raw, uint8_t pole_pairs)
{
    float mech = (float)raw * (TWO_PI / 16384.0f);
    float e_theta = mech * (float)pole_pairs;
    return fmodf(e_theta, TWO_PI);
}

/* ---------------- shared state ---------------- */

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static limb_setpoint_t s_setpoints[PS_LIMB_JOINT_COUNT];
static limb_joint_state_t s_states[PS_LIMB_JOINT_COUNT];
static limb_sensor_snapshot_t s_sensors;

void limb_setpoint_set(uint8_t joint, const limb_setpoint_t *sp)
{
    if (joint >= PS_LIMB_JOINT_COUNT || sp == NULL) {
        return;
    }
    taskENTER_CRITICAL(&s_lock);
    s_setpoints[joint] = *sp;
    taskEXIT_CRITICAL(&s_lock);
}

void limb_setpoint_zero_all(void)
{
    taskENTER_CRITICAL(&s_lock);
    for (int i = 0; i < PS_LIMB_JOINT_COUNT; i++) {
        s_setpoints[i].mode = PS_JMODE_PASSIVE;
        s_setpoints[i].pos_crad = 0;
        s_setpoints[i].vel_crad_s = 0;
        s_setpoints[i].eff_cNm = 0;
    }
    taskEXIT_CRITICAL(&s_lock);
}

void limb_setpoint_snapshot(limb_setpoint_t out[PS_LIMB_JOINT_COUNT])
{
    taskENTER_CRITICAL(&s_lock);
    memcpy(out, s_setpoints, sizeof(s_setpoints));
    taskEXIT_CRITICAL(&s_lock);
}

static void joint_state_publish(uint8_t joint, const limb_joint_state_t *st)
{
    taskENTER_CRITICAL(&s_lock);
    s_states[joint] = *st;
    taskEXIT_CRITICAL(&s_lock);
}

void limb_joint_state_snapshot(limb_joint_state_t out[PS_LIMB_JOINT_COUNT])
{
    taskENTER_CRITICAL(&s_lock);
    memcpy(out, s_states, sizeof(s_states));
    taskEXIT_CRITICAL(&s_lock);
}

static void sensor_publish(const limb_sensor_snapshot_t *snap)
{
    taskENTER_CRITICAL(&s_lock);
    s_sensors = *snap;
    taskEXIT_CRITICAL(&s_lock);
}

void limb_sensor_snapshot(limb_sensor_snapshot_t *out)
{
    taskENTER_CRITICAL(&s_lock);
    *out = s_sensors;
    taskEXIT_CRITICAL(&s_lock);
}

/* ---------------- per-joint runtime state (PIDs, FOC, torque ceiling) ---------------- */

static portMUX_TYPE s_param_lock = portMUX_INITIALIZER_UNLOCKED;
static ps_pid_t s_pos_pid[PS_LIMB_JOINT_COUNT];
static ps_pid_t s_vel_pid[PS_LIMB_JOINT_COUNT];
static ps_foc_ctl_t s_foc[PS_LIMB_JOINT_COUNT];
static int16_t s_torque_limit_cnm[PS_LIMB_JOINT_COUNT];
static uint32_t s_oc_ticks[PS_LIMB_JOINT_COUNT];

static ps_focdrv_handle_t s_focdrv[PS_LIMB_JOINT_COUNT];
static limb_encoder_t s_enc[PS_LIMB_JOINT_COUNT];
static adc_oneshot_unit_handle_t s_adc;
static imu_mpu6050_t s_imu;
static bool s_imu_ok;
static ps_mahony_t s_mahony;
static float s_strain_offset[4];

static const adc_channel_t s_strain_chan[4] = {
    PS_BOARD_ADC_STRAIN0, PS_BOARD_ADC_STRAIN1, PS_BOARD_ADC_STRAIN2, PS_BOARD_ADC_STRAIN3,
};

void limb_control_reset_pids(void)
{
    /* Entering OPERATIONAL: reset every PID/FOC integrator so no stale term
     * left over from PASSIVE kicks the joint on re-arm (docs/safety.md section 2). */
    portENTER_CRITICAL(&s_param_lock);
    for (int i = 0; i < PS_LIMB_JOINT_COUNT; i++) {
        ps_pid_reset(&s_pos_pid[i]);
        ps_pid_reset(&s_vel_pid[i]);
        ps_foc_ctl_reset(&s_foc[i]);
    }
    portEXIT_CRITICAL(&s_param_lock);
}

/* Live-tunable names: "<joint_name>_pos_kp" / "_pos_ki" / "_vel_kp" / "_vel_ki"
 * / "_torque_limit_cnm", where <joint_name> is board_limb.h's PS_LIMB_JOINTS[].name
 * (e.g. "elbow_pos_kp"). Board mechanical/electrical limits are never exposed
 * here -- only loop gains and the torque ceiling, and the ceiling can only be
 * lowered relative to the board default, never raised past it. */
static int param_match_joint(const char *name, const char **suffix_out)
{
    for (int j = 0; j < PS_LIMB_JOINT_COUNT; j++) {
        size_t nlen = strlen(PS_LIMB_JOINTS[j].name);
        if (strncmp(name, PS_LIMB_JOINTS[j].name, nlen) == 0 && name[nlen] == '_') {
            *suffix_out = name + nlen + 1;
            return j;
        }
    }
    return -1;
}

bool limb_param_set(const char *name, double value)
{
    if (name == NULL) {
        return false;
    }
    const char *suffix = NULL;
    int j = param_match_joint(name, &suffix);
    if (j < 0) {
        return false;
    }
    bool ok = true;
    portENTER_CRITICAL(&s_param_lock);
    if (strcmp(suffix, "pos_kp") == 0) {
        s_pos_pid[j].kp = (float)value;
    } else if (strcmp(suffix, "pos_ki") == 0) {
        s_pos_pid[j].ki = (float)value;
    } else if (strcmp(suffix, "vel_kp") == 0) {
        s_vel_pid[j].kp = (float)value;
    } else if (strcmp(suffix, "vel_ki") == 0) {
        s_vel_pid[j].ki = (float)value;
    } else if (strcmp(suffix, "torque_limit_cnm") == 0) {
        int32_t v = (int32_t)value;
        int16_t board_max = PS_LIMB_JOINTS[j].torque_limit_cnm;
        if (v < 0) {
            v = 0;
        }
        if (v > board_max) {
            v = board_max;   /* the network may only derate, never override up */
        }
        s_torque_limit_cnm[j] = (int16_t)v;
    } else {
        ok = false;
    }
    portEXIT_CRITICAL(&s_param_lock);
    return ok;
}

bool limb_param_get(const char *name, double *out_value)
{
    if (name == NULL || out_value == NULL) {
        return false;
    }
    const char *suffix = NULL;
    int j = param_match_joint(name, &suffix);
    if (j < 0) {
        return false;
    }
    bool ok = true;
    portENTER_CRITICAL(&s_param_lock);
    if (strcmp(suffix, "pos_kp") == 0) {
        *out_value = s_pos_pid[j].kp;
    } else if (strcmp(suffix, "pos_ki") == 0) {
        *out_value = s_pos_pid[j].ki;
    } else if (strcmp(suffix, "vel_kp") == 0) {
        *out_value = s_vel_pid[j].kp;
    } else if (strcmp(suffix, "vel_ki") == 0) {
        *out_value = s_vel_pid[j].ki;
    } else if (strcmp(suffix, "torque_limit_cnm") == 0) {
        *out_value = s_torque_limit_cnm[j];
    } else {
        ok = false;
    }
    portEXIT_CRITICAL(&s_param_lock);
    return ok;
}

/* ---------------- helpers ---------------- */

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

static float adc_read_amps(adc_channel_t chan, float offset_mv, float mv_per_a)
{
    int raw = 0;
    if (adc_oneshot_read(s_adc, chan, &raw) != ESP_OK) {
        return 0.0f;   /* a read fault reads as zero current, never phantom torque */
    }
    float mv = (float)raw * (3300.0f / 4095.0f);   /* nominal 12-bit full scale */
    return (mv - offset_mv) / mv_per_a;
}

/* THE single actuation write path (ps_focdrv.h contract): checks the safety
 * gate itself, never trusts a caller to have already checked it. PASSIVE mode
 * (energize == false) takes the same free-wheel/damp branch as a safety trip. */
static void joint_write(int j, bool energize, const float duty[3], float motor_vel_rad_s)
{
    if (energize && ps_safety_can_actuate()) {
        ps_focdrv_set_duty(s_focdrv[j], duty);
        return;
    }
    if (fabsf(motor_vel_rad_s) > DAMP_THRESHOLD_RAD_S) {
        ps_focdrv_brake(s_focdrv[j]);
    } else {
        ps_focdrv_disable(s_focdrv[j]);
    }
}

/* ---------------- init ---------------- */

static void control_init_hw(void)
{
    /* Shared I2C bus for the limb IMU. */
    i2c_master_bus_config_t bcfg = {
        .i2c_port = -1,
        .sda_io_num = PS_BOARD_I2C_SDA_GPIO,
        .scl_io_num = PS_BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bcfg, &i2c_bus));
    s_imu_ok = (imu_mpu6050_init(&s_imu, i2c_bus, PS_BOARD_IMU_ADDR) == ESP_OK);
    if (!s_imu_ok) {
        ESP_LOGE(TAG, "limb imu init failed; IMU telemetry suppressed");
    }
    ps_mahony_init(&s_mahony, 0.5f, 0.01f);

    /* Shared SPI bus for both joint encoders. */
    spi_bus_config_t spi_bus_cfg = {
        .sclk_io_num = PS_BOARD_ENC_SCLK_GPIO,
        .mosi_io_num = PS_BOARD_ENC_MOSI_GPIO,
        .miso_io_num = PS_BOARD_ENC_MISO_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(PS_BOARD_ENC_SPI_HOST, &spi_bus_cfg, SPI_DMA_DISABLED));

    /* Phase-current + strain ADC1 channels, one unit shared by both joints. */
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &s_adc));
    adc_oneshot_chan_cfg_t ccfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    for (int j = 0; j < PS_LIMB_JOINT_COUNT; j++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, PS_LIMB_JOINTS[j].adc_ia, &ccfg));
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, PS_LIMB_JOINTS[j].adc_ib, &ccfg));
    }
    for (int c = 0; c < 4; c++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, s_strain_chan[c], &ccfg));
    }

    /* Boot-time auto-tare: strain readings are zero-load offsets, not
     * absolute; only the slope (PS_STRAIN_CN_PER_COUNT) is a fixed constant. */
    for (int c = 0; c < 4; c++) {
        int64_t sum = 0;
        for (uint32_t n = 0; n < STRAIN_TARE_SAMPLES; n++) {
            int raw = 0;
            if (adc_oneshot_read(s_adc, s_strain_chan[c], &raw) == ESP_OK) {
                sum += raw;
            }
        }
        s_strain_offset[c] = (float)sum / (float)STRAIN_TARE_SAMPLES;
    }

    for (int j = 0; j < PS_LIMB_JOINT_COUNT; j++) {
        const ps_limb_joint_cfg_t *cfg = &PS_LIMB_JOINTS[j];

        ESP_ERROR_CHECK(encoder_init(&s_enc[j], PS_BOARD_ENC_SPI_HOST, cfg->encoder_cs_gpio));

        ps_focdrv_config_t fcfg = {
            .group_id = cfg->mcpwm_group,
            .gpio_uh = cfg->gate_uh, .gpio_ul = cfg->gate_ul,
            .gpio_vh = cfg->gate_vh, .gpio_vl = cfg->gate_vl,
            .gpio_wh = cfg->gate_wh, .gpio_wl = cfg->gate_wl,
        };
        ESP_ERROR_CHECK(ps_focdrv_init(&fcfg, &s_focdrv[j]));

        ps_pid_init(&s_pos_pid[j], cfg->pos_kp, cfg->pos_ki, cfg->pos_kd,
                    -cfg->pos_vel_limit_rad_s, cfg->pos_vel_limit_rad_s);
        float i_limit = (float)cfg->torque_limit_cnm / 100.0f / (cfg->gear_ratio * cfg->kt_nm_per_a);
        ps_pid_init(&s_vel_pid[j], cfg->vel_kp, cfg->vel_ki, cfg->vel_kd, -i_limit, i_limit);
        ps_foc_ctl_init(&s_foc[j], cfg->cur_kp, cfg->cur_ki, PS_LIMB_V_BUS_VOLTS * 0.5f);
        s_torque_limit_cnm[j] = cfg->torque_limit_cnm;
    }

    ESP_LOGI(TAG, "control hw up: %s + %s, imu %s", PS_LIMB_JOINTS[0].name,
             PS_LIMB_JOINTS[1].name, s_imu_ok ? "ok" : "MISSING");
}

/* ---------------- 1 kHz control loop ---------------- */

static void control_task(void *arg)
{
    (void)arg;
    control_init_hw();

    uint32_t tick = 0;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   /* woken by the gptimer ISR */
        tick++;
        limb_counters_bump_control_tick();

        limb_setpoint_t sp[PS_LIMB_JOINT_COUNT];
        limb_setpoint_snapshot(sp);
        bool can_act_global = ps_safety_can_actuate();

        for (int j = 0; j < PS_LIMB_JOINT_COUNT; j++) {
            const ps_limb_joint_cfg_t *cfg = &PS_LIMB_JOINTS[j];

            encoder_update(&s_enc[j], CTRL_TICK_S);
            float ia = adc_read_amps(cfg->adc_ia, cfg->cur_offset_mv, cfg->cur_mv_per_a);
            float ib = adc_read_amps(cfg->adc_ib, cfg->cur_offset_mv, cfg->cur_mv_per_a);
            float theta_e = encoder_electrical_theta(s_enc[j].last_raw, cfg->pole_pairs);

            float out_pos = (float)cfg->dir_sign * s_enc[j].angle_rad / cfg->gear_ratio;
            float out_vel = (float)cfg->dir_sign * s_enc[j].vel_rad_s / cfg->gear_ratio;

            portENTER_CRITICAL(&s_param_lock);
            float i_limit = (float)s_torque_limit_cnm[j] / 100.0f /
                            (cfg->gear_ratio * cfg->kt_nm_per_a);
            bool energize = (sp[j].mode != PS_JMODE_PASSIVE);
            float iq_target = 0.0f;
            const float id_target = 0.0f;   /* surface-PM assumption: no field weakening */

            switch (sp[j].mode) {
            case PS_JMODE_POSITION: {
                float pos_target = (float)sp[j].pos_crad / 100.0f;
                float vel_target = ps_pid_step(&s_pos_pid[j], pos_target - out_pos, CTRL_TICK_S);
                float i_cmd = ps_pid_step(&s_vel_pid[j], vel_target - out_vel, CTRL_TICK_S);
                iq_target = (float)cfg->dir_sign * i_cmd;
                break;
            }
            case PS_JMODE_VELOCITY: {
                float vel_target = (float)sp[j].vel_crad_s / 100.0f;
                float i_cmd = ps_pid_step(&s_vel_pid[j], vel_target - out_vel, CTRL_TICK_S);
                iq_target = (float)cfg->dir_sign * i_cmd;
                break;
            }
            case PS_JMODE_TORQUE: {
                float torque_out = (float)sp[j].eff_cNm / 100.0f;
                iq_target = (float)cfg->dir_sign * (torque_out / cfg->gear_ratio) / cfg->kt_nm_per_a;
                break;
            }
            case PS_JMODE_IMPEDANCE: {
                float pos_target = (float)sp[j].pos_crad / 100.0f;
                float vel_target = (float)sp[j].vel_crad_s / 100.0f;
                float torque_out = cfg->k_imp_nm_per_rad * (pos_target - out_pos) +
                                   cfg->d_imp_nms_per_rad * (vel_target - out_vel);
                iq_target = (float)cfg->dir_sign * (torque_out / cfg->gear_ratio) / cfg->kt_nm_per_a;
                break;
            }
            case PS_JMODE_PASSIVE:
            default:
                ps_pid_reset(&s_pos_pid[j]);
                ps_pid_reset(&s_vel_pid[j]);
                break;
            }
            portEXIT_CRITICAL(&s_param_lock);

            bool saturated = false;
            if (iq_target > i_limit) {
                iq_target = i_limit;
                saturated = true;
            } else if (iq_target < -i_limit) {
                iq_target = -i_limit;
                saturated = true;
            }

            float duty[3];
            ps_foc_ctl_step(&s_foc[j], ia, ib, theta_e, id_target, iq_target,
                            PS_LIMB_V_BUS_VOLTS, CTRL_TICK_S, duty);
            joint_write(j, energize, duty, s_enc[j].vel_rad_s);

            /* Overcurrent: measured (not commanded) dq magnitude, sustained
             * past the contract window trips a software estop. */
            ps_ab_t ab;
            ps_dq_t dq;
            ps_foc_clarke(ia, ib, &ab);
            ps_foc_park(&ab, theta_e, &dq);
            float i_mag = sqrtf(dq.d * dq.d + dq.q * dq.q);
            if (i_mag > i_limit) {
                if (++s_oc_ticks[j] > OVERCURRENT_TRIP_TICKS) {
                    ps_safety_raise_estop(PS_ESTOP_SOFTWARE);
                    limb_counters_bump_overcurrent();
                    s_oc_ticks[j] = 0;
                }
            } else {
                s_oc_ticks[j] = 0;
            }

            limb_joint_state_t st = {
                .pos_crad = sat_i16(out_pos * 100.0f),
                .vel_crad_s = sat_i16(out_vel * 100.0f),
                .eff_cNm = sat_i16(dq.q * cfg->kt_nm_per_a * cfg->gear_ratio * 100.0f),
                .saturated = saturated,
                .passive = !energize || !can_act_global,
            };
            joint_state_publish((uint8_t)j, &st);
        }

        if ((tick % SENSOR_TICK_DIV) == 0u) {
            limb_sensor_snapshot_t snap;
            memset(&snap, 0, sizeof(snap));
            if (s_imu_ok) {
                float acc_g[3], gyr_rads[3];
                if (imu_mpu6050_read(&s_imu, acc_g, gyr_rads) == ESP_OK) {
                    ps_mahony_update(&s_mahony, gyr_rads[0], gyr_rads[1], gyr_rads[2],
                                     acc_g[0], acc_g[1], acc_g[2],
                                     (float)SENSOR_TICK_DIV * CTRL_TICK_S);
                    int16_t q15[4];
                    ps_mahony_get_quat_q15(&s_mahony, q15);
                    snap.quat.qw = q15[0]; snap.quat.qx = q15[1];
                    snap.quat.qy = q15[2]; snap.quat.qz = q15[3];
                    snap.acc.ax = sat_i16(acc_g[0] * 1000.0f);
                    snap.acc.ay = sat_i16(acc_g[1] * 1000.0f);
                    snap.acc.az = sat_i16(acc_g[2] * 1000.0f);
                    snap.gyr.gx = sat_i16(gyr_rads[0] * RAD_S_TO_DDPS);
                    snap.gyr.gy = sat_i16(gyr_rads[1] * RAD_S_TO_DDPS);
                    snap.gyr.gz = sat_i16(gyr_rads[2] * RAD_S_TO_DDPS);
                }
            }
            for (int c = 0; c < 4; c++) {
                int raw = 0;
                if (adc_oneshot_read(s_adc, s_strain_chan[c], &raw) == ESP_OK) {
                    snap.force.ch[c] = sat_i16(((float)raw - s_strain_offset[c]) *
                                               PS_STRAIN_CN_PER_COUNT);
                }
            }
            sensor_publish(&snap);
        }
    }
}

/* ---------------- 1 kHz gptimer tick ---------------- */

static TaskHandle_t s_ctrl_task_handle;

static bool IRAM_ATTR ctrl_tick_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
                                    void *user_ctx)
{
    (void)timer; (void)edata; (void)user_ctx;
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(s_ctrl_task_handle, &hpw);
    return hpw == pdTRUE;
}

static esp_err_t ctrl_tick_timer_start(void)
{
    gptimer_handle_t timer;
    gptimer_config_t gcfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = CTRL_GPTIMER_RES_HZ,
    };
    esp_err_t err = gptimer_new_timer(&gcfg, &timer);
    if (err != ESP_OK) {
        return err;
    }
    gptimer_event_callbacks_t cbs = { .on_alarm = ctrl_tick_isr };
    err = gptimer_register_event_callbacks(timer, &cbs, NULL);
    if (err != ESP_OK) {
        return err;
    }
    err = gptimer_enable(timer);
    if (err != ESP_OK) {
        return err;
    }
    gptimer_alarm_config_t acfg = {
        .alarm_count = CTRL_GPTIMER_RES_HZ / CTRL_TICK_HZ,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    err = gptimer_set_alarm_action(timer, &acfg);
    if (err != ESP_OK) {
        return err;
    }
    return gptimer_start(timer);
}

esp_err_t limb_control_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(control_task, "limb_ctrl", 6144, NULL, 20,
                                            &s_ctrl_task_handle, 1);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ctrl_tick_timer_start();
}
