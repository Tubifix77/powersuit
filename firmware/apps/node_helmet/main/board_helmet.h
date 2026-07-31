/* Bench pinout for Node 6 (node_helmet_interface).
 *
 * The microphone runs at 16 kHz because that is what an I2S MEMS part gives
 * you; the AUDIO plane is 8 kHz (docs/network-map.md §3.5), so audio_in_task
 * decimates by two. Everything downstream of that decimation — gate, codec,
 * framing — is at the contract rate. */
#pragma once

#include "driver/gpio.h"

/* ---- CAN (bus 1: arms + helmet) ---- */
#define HELMET_CAN_TX_GPIO      4
#define HELMET_CAN_RX_GPIO      5
#define HELMET_CAN_BITRATE      1000000u

/* ---- I2S microphone (INMP441-class, 32-bit slot, mono left) ---- */
#define HELMET_MIC_BCLK_GPIO    15
#define HELMET_MIC_WS_GPIO      16
#define HELMET_MIC_DIN_GPIO     17
#define HELMET_MIC_RATE_HZ      16000
#define HELMET_MIC_DECIMATE     2        /* 16 kHz capture -> 8 kHz plane */

/* ---- I2S speaker (MAX98357-class, 16-bit mono) ---- */
#define HELMET_SPK_BCLK_GPIO    18
#define HELMET_SPK_WS_GPIO      8
#define HELMET_SPK_DOUT_GPIO    9
#define HELMET_SPK_RATE_HZ      8000

/* ---- HUD: twin micro-OLED stand-in, ST7789 240x240 over SPI ---- */
#define HELMET_LCD_SPI_HOST     SPI2_HOST
#define HELMET_LCD_SCLK_GPIO    12
#define HELMET_LCD_MOSI_GPIO    11
#define HELMET_LCD_DC_GPIO      13
#define HELMET_LCD_CS_GPIO      10
#define HELMET_LCD_RST_GPIO     14
#define HELMET_LCD_BL_GPIO      21
#define HELMET_LCD_H_RES        240
#define HELMET_LCD_V_RES        240
#define HELMET_LCD_CLK_HZ       (40 * 1000 * 1000)

/* ---- ESP-NOW diagnostics beacon ---- */
#define HELMET_ESPNOW_CHANNEL   6

/* ---- VOX gate (docs/safety.md §6) ---- */
#define HELMET_VOX_THRESHOLD_DBFS  (-38)
#define HELMET_VOX_HANG_MS         300u
#define HELMET_VOX_MAX_UTTER_MS    8000u

/* Audio block sizes: 16 ms of capture keeps the gate responsive without
 * waking the task more often than the CAN plane can drain. */
#define HELMET_MIC_BLOCK_SAMPLES   256   /* at 16 kHz = 16 ms */
#define HELMET_SPK_BLOCK_SAMPLES   128   /* at 8 kHz = 16 ms */
