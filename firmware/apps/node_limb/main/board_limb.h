/* board_limb.h — Nodes 1-4 (arms/legs), bench bring-up pinout + per-joint
 * mechanical/electrical/PID table. One image serves all four nodes; the
 * table is selected at compile time by CONFIG_PS_LIMB_NODE_ID (Kconfig.projbuild).
 *
 * ESP32-S3 (45 usable GPIOs). This map deliberately avoids:
 *   0, 3, 45, 46   strapping pins (boot mode / voltage select)
 *   19, 20         native USB / USB-JTAG
 *   26..32         SPI flash (and octal PSRAM on modules that carry it)
 *   43, 44         default UART0 TX/RX (console log)
 * TWAI and MCPWM route through the GPIO matrix (no IO_MUX constraint); the
 * AS5047-style encoder SPI runs well under the GPIO-matrix frequency ceiling
 * so it does not need IO_MUX pins either.
 *
 * Joints per limb (local index, network-map.md section 1): arms
 * 0=elbow/1=wrist, legs 0=hip/1=knee. Each joint gets its own MCPWM group
 * (ESP32-S3 has exactly SOC_MCPWM_GROUPS = 2, ps_focdrv.h) so group_id == joint
 * index throughout this file. */
#pragma once

#include <stdint.h>

#include "sdkconfig.h"
#include "hal/adc_types.h"   /* ADC_CHANNEL_* */

#define PS_LIMB_JOINT_COUNT 2

/* ---- CAN (TWAI controller 0, contract 1 Mbps) ---- */
#define PS_BOARD_CAN_TX_GPIO 17
#define PS_BOARD_CAN_RX_GPIO 18

/* ---- Phase-current sense, ADC1 (GPIO1..8), joint0 then joint1 ---- */
#define PS_BOARD_ADC_J0_IA   ADC_CHANNEL_0   /* GPIO1 */
#define PS_BOARD_ADC_J0_IB   ADC_CHANNEL_1   /* GPIO2 */
#define PS_BOARD_ADC_J1_IA   ADC_CHANNEL_2   /* GPIO3 */
#define PS_BOARD_ADC_J1_IB   ADC_CHANNEL_3   /* GPIO4 */

/* ---- Strain-gauge bridges, ADC1 CH4..7 (GPIO5..8): elbow/wrist or hip/knee
 * load cells, cN per count is a bench placeholder; boot-time auto-tare
 * (control_task.c) removes the zero-load offset so only the slope matters. */
#define PS_BOARD_ADC_STRAIN0 ADC_CHANNEL_4
#define PS_BOARD_ADC_STRAIN1 ADC_CHANNEL_5
#define PS_BOARD_ADC_STRAIN2 ADC_CHANNEL_6
#define PS_BOARD_ADC_STRAIN3 ADC_CHANNEL_7
#define PS_STRAIN_CN_PER_COUNT 12.5f   /* ~500 N full scale over a 12-bit ADC */

/* ---- Torso-side IMU (elbow/hip-mounted, sensor-frame == limb base link) ---- */
#define PS_BOARD_I2C_SDA_GPIO 9
#define PS_BOARD_I2C_SCL_GPIO 10
#define PS_BOARD_IMU_ADDR     0x68

/* ---- Shared SPI bus for both joint encoders (AS5047-style absolute) ---- */
#define PS_BOARD_ENC_SPI_HOST  SPI2_HOST
#define PS_BOARD_ENC_SCLK_GPIO 11
#define PS_BOARD_ENC_MOSI_GPIO 12
#define PS_BOARD_ENC_MISO_GPIO 13
#define PS_BOARD_ENC_J0_CS_GPIO 14
#define PS_BOARD_ENC_J1_CS_GPIO 15

/* ---- MCPWM gate banks, joint0 = group 0, joint1 = group 1 ---- */
#define PS_BOARD_J0_UH_GPIO 21
#define PS_BOARD_J0_UL_GPIO 22
#define PS_BOARD_J0_VH_GPIO 23
#define PS_BOARD_J0_VL_GPIO 24
#define PS_BOARD_J0_WH_GPIO 25
#define PS_BOARD_J0_WL_GPIO 33
#define PS_BOARD_J1_UH_GPIO 34
#define PS_BOARD_J1_UL_GPIO 35
#define PS_BOARD_J1_VH_GPIO 36
#define PS_BOARD_J1_VL_GPIO 37
#define PS_BOARD_J1_WH_GPIO 38
#define PS_BOARD_J1_WL_GPIO 39

/* Nominal battery bus feeding the joint inverters; PI current-loop clamps
 * (ps_foc_ctl_init v_limit) are derived from this, not hand-tuned per joint. */
#define PS_LIMB_V_BUS_VOLTS 24.0f

typedef struct {
    const char *name;                 /* "elbow", "wrist", "hip", "knee" */

    /* Electrical/mechanical output stage. */
    int      mcpwm_group;
    int      gate_uh, gate_ul, gate_vh, gate_vl, gate_wh, gate_wl;
    int      encoder_cs_gpio;
    adc_channel_t adc_ia, adc_ib;
    uint8_t  pole_pairs;

    /* Kinematics/dynamics: motor shaft <-> output (post-gearbox) shaft. */
    float    gear_ratio;              /* motor turns per output-shaft turn, > 1 */
    int8_t   dir_sign;                /* +1/-1: mirrors left vs right assembly */
    int16_t  pos_min_crad, pos_max_crad; /* output-shaft mechanical range, 0.01 rad */
    int16_t  torque_limit_cnm;         /* output-shaft ceiling, 0.01 N*m (default) */
    float    kt_nm_per_a;              /* motor torque constant */

    /* Phase-current sense amp transfer (bidirectional, midrail-referenced). */
    float    cur_mv_per_a;
    float    cur_offset_mv;

    /* IMPEDANCE mode gains, referred to the output shaft. */
    float    k_imp_nm_per_rad;
    float    d_imp_nms_per_rad;

    /* Default PID gains (comms_task.c parameter server can override kp/ki live). */
    float    pos_kp, pos_ki, pos_kd, pos_vel_limit_rad_s; /* position -> velocity target */
    float    vel_kp, vel_ki, vel_kd;                      /* velocity -> iq target (A) */
    float    cur_kp, cur_ki;                              /* dq current loop (ps_foc_ctl) */
} ps_limb_joint_cfg_t;

#define PS_LIMB_JOINT_TABLE_COMMON(NAME0, NAME1, GEAR0, GEAR1, DIR, PMIN0, PMAX0,   \
                                   PMIN1, PMAX1, TLIM0, TLIM1, KT0, KT1, KIMP0,      \
                                   DIMP0, KIMP1, DIMP1)                             \
    {                                                                               \
        {                                                                          \
            .name = NAME0, .mcpwm_group = 0,                                       \
            .gate_uh = PS_BOARD_J0_UH_GPIO, .gate_ul = PS_BOARD_J0_UL_GPIO,         \
            .gate_vh = PS_BOARD_J0_VH_GPIO, .gate_vl = PS_BOARD_J0_VL_GPIO,         \
            .gate_wh = PS_BOARD_J0_WH_GPIO, .gate_wl = PS_BOARD_J0_WL_GPIO,         \
            .encoder_cs_gpio = PS_BOARD_ENC_J0_CS_GPIO,                             \
            .adc_ia = PS_BOARD_ADC_J0_IA, .adc_ib = PS_BOARD_ADC_J0_IB,             \
            .pole_pairs = 7,                                                       \
            .gear_ratio = (GEAR0), .dir_sign = (DIR),                              \
            .pos_min_crad = (PMIN0), .pos_max_crad = (PMAX0),                      \
            .torque_limit_cnm = (TLIM0), .kt_nm_per_a = (KT0),                     \
            .cur_mv_per_a = 55.0f, .cur_offset_mv = 1650.0f,                       \
            .k_imp_nm_per_rad = (KIMP0), .d_imp_nms_per_rad = (DIMP0),             \
            .pos_kp = 6.0f, .pos_ki = 0.5f, .pos_kd = 0.05f,                       \
            .pos_vel_limit_rad_s = 4.0f,                                          \
            .vel_kp = 2.5f, .vel_ki = 4.0f, .vel_kd = 0.0f,                        \
            .cur_kp = 4.0f, .cur_ki = 800.0f,                                      \
        },                                                                         \
        {                                                                          \
            .name = NAME1, .mcpwm_group = 1,                                       \
            .gate_uh = PS_BOARD_J1_UH_GPIO, .gate_ul = PS_BOARD_J1_UL_GPIO,         \
            .gate_vh = PS_BOARD_J1_VH_GPIO, .gate_vl = PS_BOARD_J1_VL_GPIO,         \
            .gate_wh = PS_BOARD_J1_WH_GPIO, .gate_wl = PS_BOARD_J1_WL_GPIO,         \
            .encoder_cs_gpio = PS_BOARD_ENC_J1_CS_GPIO,                             \
            .adc_ia = PS_BOARD_ADC_J1_IA, .adc_ib = PS_BOARD_ADC_J1_IB,             \
            .pole_pairs = 7,                                                       \
            .gear_ratio = (GEAR1), .dir_sign = (DIR),                              \
            .pos_min_crad = (PMIN1), .pos_max_crad = (PMAX1),                      \
            .torque_limit_cnm = (TLIM1), .kt_nm_per_a = (KT1),                     \
            .cur_mv_per_a = 55.0f, .cur_offset_mv = 1650.0f,                       \
            .k_imp_nm_per_rad = (KIMP1), .d_imp_nms_per_rad = (DIMP1),             \
            .pos_kp = 5.0f, .pos_ki = 0.4f, .pos_kd = 0.04f,                       \
            .pos_vel_limit_rad_s = 5.0f,                                          \
            .vel_kp = 2.0f, .vel_ki = 3.0f, .vel_kd = 0.0f,                        \
            .cur_kp = 4.0f, .cur_ki = 800.0f,                                      \
        },                                                                          \
    }

/* dir_sign mirrors the actuator mounting between the right- and left-side
 * assemblies of the same limb type; magnitudes (gear ratio, torque ceiling,
 * range, Kt) are shared because both sides use the same actuator hardware. */
#if CONFIG_PS_LIMB_NODE_ID == 1   /* arm_right: elbow=0, wrist=1 */
static const ps_limb_joint_cfg_t PS_LIMB_JOINTS[PS_LIMB_JOINT_COUNT] =
    PS_LIMB_JOINT_TABLE_COMMON("elbow", "wrist", 6.0f, 4.0f, +1,
                                /*elbow*/ -10, 260, /*wrist*/ -157, 157,
                                /*Tlim*/ 4000, 1500, /*Kt*/ 0.18f, 0.12f,
                                /*Kimp/Dimp elbow*/ 15.0f, 0.5f,
                                /*Kimp/Dimp wrist*/ 6.0f, 0.2f);
#elif CONFIG_PS_LIMB_NODE_ID == 2 /* arm_left: elbow=0, wrist=1 (mirrored) */
static const ps_limb_joint_cfg_t PS_LIMB_JOINTS[PS_LIMB_JOINT_COUNT] =
    PS_LIMB_JOINT_TABLE_COMMON("elbow", "wrist", 6.0f, 4.0f, -1,
                                -10, 260, -157, 157,
                                4000, 1500, 0.18f, 0.12f,
                                15.0f, 0.5f, 6.0f, 0.2f);
#elif CONFIG_PS_LIMB_NODE_ID == 3 /* leg_right: hip=0, knee=1 */
static const ps_limb_joint_cfg_t PS_LIMB_JOINTS[PS_LIMB_JOINT_COUNT] =
    PS_LIMB_JOINT_TABLE_COMMON("hip", "knee", 9.0f, 8.0f, +1,
                                /*hip*/ -52, 209, /*knee*/ 0, 244,
                                /*Tlim*/ 8000, 6000, /*Kt*/ 0.25f, 0.22f,
                                /*Kimp/Dimp hip*/ 25.0f, 0.8f,
                                /*Kimp/Dimp knee*/ 20.0f, 0.6f);
#elif CONFIG_PS_LIMB_NODE_ID == 4 /* leg_left: hip=0, knee=1 (mirrored) */
static const ps_limb_joint_cfg_t PS_LIMB_JOINTS[PS_LIMB_JOINT_COUNT] =
    PS_LIMB_JOINT_TABLE_COMMON("hip", "knee", 9.0f, 8.0f, -1,
                                -52, 209, 0, 244,
                                8000, 6000, 0.25f, 0.22f,
                                25.0f, 0.8f, 20.0f, 0.6f);
#else
#error "CONFIG_PS_LIMB_NODE_ID must be 1..4 (Kconfig.projbuild PS_NODE_ID choice)"
#endif
