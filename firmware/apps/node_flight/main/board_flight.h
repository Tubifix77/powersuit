/* board_flight — Node 5 hardware map + flap envelope table (docs/safety.md §5).
 * Pin assignments are the bring-up harness pinout; final numbers land with the
 * carrier board layout. Envelope/exclusion values are the authoritative safety
 * table: servo_task clamps against THIS table immediately before the PWM write. */
#pragma once

#include <stdint.h>

#include "driver/gpio.h"

/* --- CAN (TWAI controller 0, contract 1 Mbps) --- */
#define PS_BOARD_CAN_TX_GPIO 17
#define PS_BOARD_CAN_RX_GPIO 18

/* --- Flap servo bank ---
 * 12 independent 50 Hz PWM outputs. ESP32-S3 LEDC has only 8 channels
 * (SOC_LEDC_CHANNEL_NUM = 8, low-speed only), so 12 flaps CANNOT ride LEDC;
 * MCPWM provides exactly 12: 2 groups x 3 operators x 2 generators, one
 * comparator per generator (SOC_MCPWM_* caps, IDF v5.5.5). Mapping is fixed:
 *   group = flap / 6, operator = (flap % 6) / 2, generator = flap % 2,
 * one 50 Hz timer per group. Recorded as a deviation from the "LEDC" intent. */
#define PS_FLAP_COUNT           12
#define PS_SERVO_PWM_FREQ_HZ    50
#define PS_SERVO_TICK_US        1        /* 1 MHz MCPWM timebase: ticks == µs */
#define PS_SERVO_PERIOD_TICKS   20000    /* 20 ms frame */

typedef struct {
    uint8_t  gpio;             /* PWM output pin */
    uint16_t pulse_min_us;     /* mechanical envelope, never exceeded */
    uint16_t pulse_neutral_us; /* PASSIVE/ESTOP column (docs/safety.md §5.4) */
    uint16_t pulse_max_us;
    int16_t  env_min_pm;       /* commanded envelope, permille of half-throw */
    int16_t  env_max_pm;
    int16_t  brake_pm;         /* FLAP_CMD flags.b0 brake_mode target */
    uint8_t  slew_max_pct_s;   /* hard slew cap, % of full scale per second */
    uint8_t  slew_default_pct_s; /* used when FLAP_CMD.rate_lim == 0 */
} ps_flap_cfg_t;

/* Surface layout: 0..5 = left wing root->tip, 6..11 = right wing root->tip.
 * Adjacent overlapping segments carry a joint |a|+|b| limit (pairwise
 * exclusion, docs/safety.md §5.2). */
static const ps_flap_cfg_t PS_FLAP_TABLE[PS_FLAP_COUNT] = {
    /*  gpio  min   neut  max   env-   env+  brake  slew  dflt */
    {   10,   900,  1500, 2100, -900,  900,  800,   250,  120 },
    {   11,   900,  1500, 2100, -900,  900,  800,   250,  120 },
    {   12,   900,  1500, 2100, -1000, 1000, 900,   250,  120 },
    {   13,   900,  1500, 2100, -1000, 1000, 900,   250,  120 },
    {   14,   900,  1500, 2100, -800,  800,  700,   200,  100 },
    {   15,   900,  1500, 2100, -800,  800,  700,   200,  100 },
    {   16,   900,  1500, 2100, -900,  900,  800,   250,  120 },
    {   21,   900,  1500, 2100, -900,  900,  800,   250,  120 },
    {   38,   900,  1500, 2100, -1000, 1000, 900,   250,  120 },
    {   39,   900,  1500, 2100, -1000, 1000, 900,   250,  120 },
    {   40,   900,  1500, 2100, -800,  800,  700,   200,  100 },
    {   41,   900,  1500, 2100, -800,  800,  700,   200,  100 },
};

typedef struct {
    uint8_t  a, b;             /* flap indices of an overlapping pair */
    uint16_t joint_max_sum_pm; /* |a| + |b| must stay at/below this */
} ps_flap_pair_t;

#define PS_FLAP_PAIR_COUNT 4
static const ps_flap_pair_t PS_FLAP_PAIRS[PS_FLAP_PAIR_COUNT] = {
    { 0,  1,  1400 },
    { 2,  3,  1600 },
    { 6,  7,  1400 },
    { 8,  9,  1600 },
};

/* --- Pitot (differential pressure -> dynamic pressure q) ---
 * ADC1 channel 5 = GPIO6 on ESP32-S3. Transfer after the on-board divider:
 * q[Pa] = (mV - PS_PITOT_ZERO_MV) * PS_PITOT_PA_PER_MV, negative clamped to 0
 * (single-port probe reads no reverse flow). Calibrate both at installation. */
#define PS_PITOT_ADC_UNIT     ADC_UNIT_1
#define PS_PITOT_ADC_CHANNEL  ADC_CHANNEL_5
#define PS_PITOT_ZERO_MV      1650.0f
#define PS_PITOT_PA_PER_MV    3.0f

/* Air density default; runtime-tunable via the set_parameters service. */
#define PS_RHO_DEFAULT        1.225f

/* --- Torso IMU (network-map §12.6): MPU6050-class on I2C --- */
#define PS_BOARD_I2C_SDA_GPIO 8
#define PS_BOARD_I2C_SCL_GPIO 9
#define PS_BOARD_IMU_ADDR     0x68
