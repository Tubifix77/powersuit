/* board_hub.h — Node 7 chest power/network hub, bench prototype pinout.
 *
 * ESP32-P4 (55 GPIOs, 0..54). This map deliberately avoids:
 *   7/8/9/10   GPSPI2 IO_MUX pad set (CS0/MOSI/SCLK/MISO) — the Node 8 link
 *   16..23     ADC1 channels 0..7
 *   24/25      USB-JTAG
 *   34..38     strapping / ROM console pins
 *   49..54     ADC2 channels 0..5
 * TWAI has no IO_MUX constraint (GPIO matrix); CAN pins are free picks. */
#pragma once

#include <stdint.h>

#include "sdkconfig.h"
#include "hal/adc_types.h"   /* ADC_CHANNEL_* */

/* ---- CAN buses (TWAI controllers 0 and 1, transceivers on the harness) ---- */
#define HUB_CAN1_TX_GPIO        26      /* Bus 1: arms + helmet */
#define HUB_CAN1_RX_GPIO        27
#define HUB_CAN2_TX_GPIO        28      /* Bus 2: legs + flight */
#define HUB_CAN2_RX_GPIO        29
#define HUB_CAN_BITRATE         1000000u /* contract §2 */

/* ---- SPI2 slave link to Node 8 (IO_MUX pads, 20 MHz per §12.1) ---- */
#define HUB_SPI_MOSI_GPIO       CONFIG_PS_SPIB_MOSI_GPIO        /* 8  */
#define HUB_SPI_MISO_GPIO       CONFIG_PS_SPIB_MISO_GPIO        /* 10 */
#define HUB_SPI_SCLK_GPIO       CONFIG_PS_SPIB_SCLK_GPIO        /* 9  */
#define HUB_SPI_CS_GPIO         CONFIG_PS_SPIB_CS_GPIO          /* 7  */
#define HUB_SPI_DATA_READY_GPIO CONFIG_PS_SPIB_DATA_READY_GPIO  /* 4  */

/* ---- BMS short-circuit chain (docs/safety.md §4: hardware first) ---- */
#define HUB_BMS_SC_LATCH_GPIO     5   /* SR-latch output, ACTIVE LOW on trip (ISR input) */
#define HUB_BMS_COMP_STATUS_GPIO  30  /* live comparator out: HIGH = current above threshold */
#define HUB_BMS_REARM_STROBE_GPIO 31  /* 10 ms strobe; hardware ANDs it with comparator-clear */
#define HUB_BMS_KILL_FB_GPIO      32  /* gate-driver feedback: HIGH = discharge FET closed */

/* ---- BMS analog front-end ---- */
/* ADC1 = GPIO16..23 (CH0..7), ADC2 = GPIO49..54 (CH0..5) on the P4. */
#define HUB_ADC1_CH_PACK_V      ADC_CHANNEL_0   /* GPIO16: pack divider */
#define HUB_ADC1_CH_CURRENT     ADC_CHANNEL_1   /* GPIO17: bidirectional shunt amp */
#define HUB_ADC1_CH_CELL_BASE   ADC_CHANNEL_2   /* GPIO18..21: quad 4:1 mux sections A..D */
#define HUB_ADC1_CH_NTC0        ADC_CHANNEL_6   /* GPIO22 */
#define HUB_ADC1_CH_NTC1        ADC_CHANNEL_7   /* GPIO23 */
#define HUB_ADC2_CH_NTC2        ADC_CHANNEL_0   /* GPIO49 */
#define HUB_ADC2_CH_NTC3        ADC_CHANNEL_1   /* GPIO50 */
#define HUB_BMS_MUX_SEL0_GPIO   2   /* quad-mux select bits, shared across sections */
#define HUB_BMS_MUX_SEL1_GPIO   3

/* Pack: 12S3P li-ion. 12 cell taps = 4 mux sections x 3 select states (state 3
 * unused). One tap is read per 1 kHz tick after a full-tick mux settle; with
 * the 4 NTC slots this makes a 16-slot rotation = full pack scan every 16 ms. */
#define HUB_BMS_CELL_COUNT      12
#define HUB_BMS_CELL_GROUPS     4    /* BMS_CELLS carries 3 cells per group */
#define HUB_BMS_NTC_COUNT       4
#define HUB_BMS_SCAN_SLOTS      16

/* Conversion factors (bench values; final calibration lives with the AFE). */
#define HUB_ADC_FS_MV           3300.0f  /* approx full scale at 12 dB atten */
#define HUB_ADC_MAX_RAW         4095.0f  /* 12-bit */
#define HUB_PACK_DIV            21.0f    /* 200k:10k -> 50.4 V max = 2.40 V at pad */
#define HUB_CURR_OFFSET_MV      1650.0f  /* shunt amp midrail (bidirectional) */
#define HUB_CURR_MV_PER_A       5.0f     /* 0.25 mOhm x 20 V/V; positive = discharge */
#define HUB_CELL_DIV            2.0f     /* AFE scales each tap into ADC range */
#define HUB_NTC_PULLUP_OHM      10000.0f /* 3V3 -- Rpull -- pad -- NTC -- GND */
#define HUB_NTC_R25_OHM         10000.0f
#define HUB_NTC_BETA            3435.0f

/* Trip thresholds (units per contract §3: mV, cC = 0.01 C, cA = 0.01 A). */
typedef struct {
    uint16_t cell_ov_mv,      cell_ov_warn_mv;
    uint16_t cell_uv_mv,      cell_uv_warn_mv;
    int16_t  ot_cC,           ot_warn_cC;
    int16_t  ut_cC,           ut_warn_cC;
    int16_t  oc_dis_cA,       oc_dis_warn_cA;
    int16_t  oc_chg_cA,       oc_chg_warn_cA;
    uint16_t debounce_ms;     /* consecutive out-of-range ms before action */
} hub_bms_thresholds_t;

#define HUB_BMS_THRESHOLDS_DEFAULT                                  \
    {                                                               \
        .cell_ov_mv = 4250, .cell_ov_warn_mv = 4150,                \
        .cell_uv_mv = 2800, .cell_uv_warn_mv = 2950,                \
        .ot_cC = 6000,      .ot_warn_cC = 5500,                     \
        .ut_cC = -1000,     .ut_warn_cC = -500,                     \
        .oc_dis_cA = 12000, .oc_dis_warn_cA = 10000, /* 120 A trip */\
        .oc_chg_cA = 2000,  .oc_chg_warn_cA = 1800,  /* 20 A trip  */\
        .debounce_ms = 50,                                          \
    }

/* ---- Status LEDs: 24-pixel WS2812 "arc reactor" ring ---- */
#define HUB_LED_GPIO            33
#define HUB_LED_COUNT           24
#define HUB_LED_MAX_BRIGHTNESS  160   /* state patterns; overrides carry their own */
#define HUB_LED_OVERRIDE_TIMEOUT_MS 5000 /* LED_PATTERN has no duration field */
