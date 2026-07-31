/* bms_task.c — Node 7 battery management: 1 kHz sampling, debounced trip
 * escalation, and the SHORT-CIRCUIT OBSERVER of docs/safety.md §4.
 *
 * The <5 us short-circuit reaction is HARDWARE: analog comparator -> SR latch
 * -> gate driver disable, no software involved. This file is the honest
 * firmware observer: it timestamps the latch edge from an IRAM ISR, broadcasts
 * ESTOP(BMS_SHORT), reports fault bits, and manages the interlocked re-arm.
 * Firmware alone cannot re-close a shorted pack — the re-arm strobe reaches
 * the latch only through a hardware AND with the comparator's clear output. */
#include "hub_tasks.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "powersuit_proto/can_id.h"
#include "powersuit_proto/spi_frame.h"
#include "ps_spibridge.h"

#include "board_hub.h"

static const char *TAG = "node_hub";

#define BMS_TASK_PRIO   18
#define BMS_TASK_CORE   1        /* control core; gateway/SPI own core 0 */
#define BMS_TASK_STACK  6144

#ifndef CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ
#define CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ 360
#endif

/* Escalation families (debounce + latched-trip bookkeeping). */
enum { FAM_OV, FAM_UV, FAM_OT, FAM_UT, FAM_OCD, FAM_OCC, FAM_COUNT };

/* NODE_FAULT fault_code namespace for hub BMS warnings: 0xB0 + family. */
#define BMS_FAULT_CODE(fam) ((uint8_t)(0xB0 + (fam)))

static ps_can_handle_t  s_can[2];
static TaskHandle_t     s_task;
static adc_oneshot_unit_handle_t s_adc1, s_adc2;
static const hub_bms_thresholds_t s_th = HUB_BMS_THRESHOLDS_DEFAULT;

/* Short-circuit observer state (ISR <-> task). */
static volatile uint32_t s_trip_cycles;
static volatile bool     s_trip_flag;
static volatile bool     s_rearm_requested;

/* Measurements. */
static uint16_t s_cell_mv[HUB_BMS_CELL_COUNT];
static int16_t  s_ntc_cC[HUB_BMS_NTC_COUNT];
static uint16_t s_pack_cV;
static int16_t  s_curr_cA;          /* positive = discharge */
static uint16_t s_fault_bits;       /* PS_BMSF_* */
static bool     s_short_latched;
static bool     s_full_scan_done;   /* all 16 slots visited at least twice */
static uint32_t s_tick;

static uint16_t s_deb[FAM_COUNT];
static bool     s_tripped[FAM_COUNT];
static uint32_t s_warn_last_ms[FAM_COUNT];

static uint8_t  s_safety_low;       /* rolling low byte, all hub SAFETY frames */
static uint16_t s_estop_seq;
static uint8_t  s_summary_seq, s_cells_seq, s_stats_seq;
static uint8_t  s_cells_group;
static ps_can_stats_t s_prev_can[2];
static ps_spib_stats_t s_prev_spib;

static uint32_t now_ms32(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ------------- short-circuit latch ISR (IRAM, installed with IRAM flag) ------------- */

static void IRAM_ATTR bms_sc_latch_isr(void *arg)
{
    (void)arg;
    /* Observer only. The comparator + SR latch already opened the discharge
     * gate (<1 us, docs/safety.md §4); anything beyond timestamp + flag +
     * task wake here would be theater. */
    s_trip_cycles = esp_cpu_get_cycle_count();
    s_trip_flag = true;
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &hpw);
    portYIELD_FROM_ISR(hpw);
}

/* ------------- emit helpers ------------- */

/* Hub-origin TELEM/MGMT rides the SPI uplink only, bus nibble = HUB_LOCAL(3)
 * (§5 note + §6): it never spends CAN bandwidth. */
static void bms_push_record(uint32_t id, const void *payload, uint8_t dlc)
{
    ps_can_record_t rec = {
        .id = id,
        .bus = PS_SPI_BUS_HUB_LOCAL,
        .dlc = dlc,
        .ts_ms = (uint16_t)(esp_timer_get_time() / 1000),
    };
    memset(rec.data, 0, sizeof(rec.data));
    memcpy(rec.data, payload, dlc);
    (void)ps_spib_uplink_push(&rec); /* overflow is accounted by the bridge */
}

static void bms_send_safety_both(uint8_t type, const void *payload, uint8_t dlc,
                                 unsigned repeat)
{
    ps_can_frame_t f = {
        .id = ps_can_id_pack(PS_CLS_SAFETY, PS_NODE_HUB, PS_NODE_BROADCAST,
                             type, s_safety_low++),
        .dlc = dlc,
    };
    memset(f.data, 0, sizeof(f.data));
    memcpy(f.data, payload, dlc);
    for (unsigned r = 0; r < repeat; r++) {
        (void)ps_can_send(s_can[0], &f, pdMS_TO_TICKS(2));
        (void)ps_can_send(s_can[1], &f, pdMS_TO_TICKS(2));
    }
    bms_push_record(f.id, payload, dlc); /* SPI link is CRC-framed: once */
}

static void bms_raise_estop(uint8_t cause)
{
    ps_estop_t es = {
        .cause = cause,
        .origin_node = PS_NODE_HUB,
        .seq = s_estop_seq++,
        .uptime_ms = now_ms32(),
    };
    bms_send_safety_both(PS_T_ESTOP, &es, sizeof(es), PS_ESTOP_REPEAT);
    hub_gateway_local_estop(cause); /* mirror latches; LED follows state */
}

static void bms_warn(int fam, uint16_t detail)
{
    uint32_t now = now_ms32();
    if (now - s_warn_last_ms[fam] < 1000u) {
        return; /* 1/s per family */
    }
    s_warn_last_ms[fam] = now;
    ps_node_fault_t nf = {
        .fault_code = BMS_FAULT_CODE(fam),
        .severity = 1, /* warn */
        .detail = detail,
        .uptime_ms = now,
    };
    /* Warn severity stays off the buses; the Pi sees it via the uplink. */
    bms_push_record(ps_can_id_pack(PS_CLS_SAFETY, PS_NODE_HUB, PS_NODE_ORCH,
                                   PS_T_NODE_FAULT, s_safety_low++),
                    &nf, sizeof(nf));
}

/* ------------- sampling ------------- */

static float adc_mv(adc_oneshot_unit_handle_t unit, adc_channel_t chan)
{
    int raw = 0;
    if (adc_oneshot_read(unit, chan, &raw) != ESP_OK) {
        return -1.0f;
    }
    return (float)raw * (HUB_ADC_FS_MV / HUB_ADC_MAX_RAW);
}

static void bms_sample_fast(void)
{
    float mv = adc_mv(s_adc1, HUB_ADC1_CH_PACK_V);
    if (mv >= 0.0f) {
        s_pack_cV = (uint16_t)(mv * HUB_PACK_DIV / 10.0f); /* mV -> cV */
    }
    mv = adc_mv(s_adc1, HUB_ADC1_CH_CURRENT);
    if (mv >= 0.0f) {
        s_curr_cA = (int16_t)((mv - HUB_CURR_OFFSET_MV) / HUB_CURR_MV_PER_A * 100.0f);
    }
}

static void bms_slow_slot(void)
{
    uint32_t slot = s_tick % HUB_BMS_SCAN_SLOTS;
    if (slot < HUB_BMS_CELL_COUNT) {
        /* Mux select for this tap was set one full tick ago: 1 ms settling. */
        adc_channel_t chan = (adc_channel_t)(HUB_ADC1_CH_CELL_BASE + (slot & 3u));
        float mv = adc_mv(s_adc1, chan);
        if (mv >= 0.0f) {
            s_cell_mv[slot] = (uint16_t)(mv * HUB_CELL_DIV);
        }
    } else {
        uint32_t n = slot - HUB_BMS_CELL_COUNT;
        static const struct { int unit; adc_channel_t ch; } ntc_map[HUB_BMS_NTC_COUNT] = {
            { 1, HUB_ADC1_CH_NTC0 }, { 1, HUB_ADC1_CH_NTC1 },
            { 2, HUB_ADC2_CH_NTC2 }, { 2, HUB_ADC2_CH_NTC3 },
        };
        float mv = adc_mv(ntc_map[n].unit == 1 ? s_adc1 : s_adc2, ntc_map[n].ch);
        if (mv > 1.0f && mv < HUB_ADC_FS_MV - 1.0f) {
            float r = mv * HUB_NTC_PULLUP_OHM / (HUB_ADC_FS_MV - mv);
            float inv_t = 1.0f / 298.15f + logf(r / HUB_NTC_R25_OHM) / HUB_NTC_BETA;
            s_ntc_cC[n] = (int16_t)((1.0f / inv_t - 273.15f) * 100.0f);
        }
    }
    /* Pre-select the mux for the NEXT cell tap (settle across the tick). */
    uint32_t next = (s_tick + 1u) % HUB_BMS_SCAN_SLOTS;
    if (next < HUB_BMS_CELL_COUNT) {
        uint32_t sel = next >> 2; /* sections A..D share the 2 select lines */
        gpio_set_level(HUB_BMS_MUX_SEL0_GPIO, sel & 1u);
        gpio_set_level(HUB_BMS_MUX_SEL1_GPIO, (sel >> 1) & 1u);
    }
    if (s_tick >= 2u * HUB_BMS_SCAN_SLOTS) {
        s_full_scan_done = true; /* thresholds armed after two full scans */
    }
}

/* ------------- escalation ------------- */

static void bms_check(int fam, bool crit, bool warn, uint8_t cause, uint16_t bit,
                      uint16_t detail)
{
    if (crit) {
        if (s_deb[fam] < 0xFFFFu) {
            s_deb[fam]++;
        }
    } else {
        s_deb[fam] = 0;
    }
    if (s_tripped[fam]) {
        return; /* latched until CLEAR_ESTOP + STANDBY re-entry */
    }
    if (s_deb[fam] >= s_th.debounce_ms) {
        s_tripped[fam] = true;
        s_fault_bits |= bit;
        ESP_LOGE(TAG, "BMS trip fam=%d cause=%u detail=%u", fam, cause, detail);
        bms_raise_estop(cause);
    } else if (warn) {
        bms_warn(fam, detail);
    }
}

static void bms_check_thresholds(void)
{
    if (!s_full_scan_done) {
        return; /* 32 ms boot blind window; the hardware comparator covers it */
    }
    uint16_t vmin = 0xFFFF, vmax = 0;
    for (int i = 0; i < HUB_BMS_CELL_COUNT; i++) {
        if (s_cell_mv[i] < vmin) vmin = s_cell_mv[i];
        if (s_cell_mv[i] > vmax) vmax = s_cell_mv[i];
    }
    int16_t tmin = s_ntc_cC[0], tmax = s_ntc_cC[0];
    for (int i = 1; i < HUB_BMS_NTC_COUNT; i++) {
        if (s_ntc_cC[i] < tmin) tmin = s_ntc_cC[i];
        if (s_ntc_cC[i] > tmax) tmax = s_ntc_cC[i];
    }

    bms_check(FAM_OV, vmax > s_th.cell_ov_mv, vmax > s_th.cell_ov_warn_mv,
              PS_ESTOP_BMS_OVERVOLT, PS_BMSF_OV, vmax);
    bms_check(FAM_UV, vmin < s_th.cell_uv_mv, vmin < s_th.cell_uv_warn_mv,
              PS_ESTOP_BMS_UNDERVOLT, PS_BMSF_UV, vmin);
    bms_check(FAM_OT, tmax > s_th.ot_cC, tmax > s_th.ot_warn_cC,
              PS_ESTOP_THERMAL, PS_BMSF_OT, (uint16_t)tmax);
    bms_check(FAM_UT, tmin < s_th.ut_cC, tmin < s_th.ut_warn_cC,
              PS_ESTOP_THERMAL, PS_BMSF_UT, (uint16_t)tmin);
    /* §4.4 defines no dedicated OC estop cause; sustained overcurrent's
     * hazard is heating, so it trips as THERMAL with the OC fault bit set. */
    bms_check(FAM_OCD, s_curr_cA > s_th.oc_dis_cA, s_curr_cA > s_th.oc_dis_warn_cA,
              PS_ESTOP_THERMAL, PS_BMSF_OC_DISCHARGE, (uint16_t)s_curr_cA);
    bms_check(FAM_OCC, -s_curr_cA > s_th.oc_chg_cA, -s_curr_cA > s_th.oc_chg_warn_cA,
              PS_ESTOP_THERMAL, PS_BMSF_OC_CHARGE, (uint16_t)(-s_curr_cA));
}

/* ------------- short-circuit observer + interlocked re-arm ------------- */

static void bms_handle_short_trip(void)
{
    if (!s_trip_flag) {
        return;
    }
    s_trip_flag = false;
    uint32_t lat_cycles = esp_cpu_get_cycle_count() - s_trip_cycles;
    s_short_latched = true;
    s_fault_bits |= PS_BMSF_SHORT_LATCH;
    s_fault_bits &= (uint16_t)~PS_BMSF_COMP_ARMED;
    bms_raise_estop(PS_ESTOP_BMS_SHORT);
    /* ~10 us-class observer latency; informational only — the gate opened in
     * hardware before this task ever ran (docs/safety.md §4, network-map §12.2). */
    ESP_LOGE(TAG, "BMS short latch: observer latency %u cycles (~%u us); gate already open",
             (unsigned)lat_cycles,
             (unsigned)(lat_cycles / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ));
}

void hub_bms_notify_clear_accepted(void)
{
    s_rearm_requested = true; /* consumed by the task once mirror is STANDBY */
}

static void bms_try_rearm(void)
{
    if (!s_rearm_requested) {
        return;
    }
    /* Only from STANDBY: CLEAR_ESTOP accepted AND 1 s of heartbeat elapsed. */
    if (hub_gateway_safety_state() != PS_STATE_STANDBY) {
        return;
    }
    s_rearm_requested = false;

    if (!s_short_latched) {
        /* Non-short trips (OV/UV/thermal/OC) need no hardware re-arm: clear
         * the latched families so monitoring can trip fresh. */
        memset(s_tripped, 0, sizeof(s_tripped));
        memset(s_deb, 0, sizeof(s_deb));
        s_fault_bits &= (uint16_t)~(PS_BMSF_OV | PS_BMSF_UV | PS_BMSF_OT |
                                    PS_BMSF_UT | PS_BMSF_OC_CHARGE |
                                    PS_BMSF_OC_DISCHARGE);
        return;
    }
    if (gpio_get_level(HUB_BMS_COMP_STATUS_GPIO)) {
        ESP_LOGE(TAG, "re-arm refused: comparator still above threshold");
        bms_warn(FAM_OCD, 0xFFFF);
        return;
    }
    /* 10 ms strobe into the hardware AND-gate. Firmware alone cannot re-close
     * a shorted pack: the latch clears only if the comparator agrees. Blocking
     * this loop for 11 ms is fine — we are in STANDBY by definition here. */
    gpio_set_level(HUB_BMS_REARM_STROBE_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(HUB_BMS_REARM_STROBE_GPIO, 0);
    vTaskDelay(1);
    if (gpio_get_level(HUB_BMS_KILL_FB_GPIO)) {
        s_short_latched = false;
        s_fault_bits &= (uint16_t)~PS_BMSF_SHORT_LATCH;
        s_fault_bits |= PS_BMSF_COMP_ARMED;
        memset(s_tripped, 0, sizeof(s_tripped));
        memset(s_deb, 0, sizeof(s_deb));
        ESP_LOGI(TAG, "discharge gate re-armed (comparator clear + latch reset)");
    } else {
        ESP_LOGE(TAG, "re-arm refused by hardware: kill gate did not re-close");
    }
}

/* ------------- periodic reports (SPI uplink only) ------------- */

static uint8_t bms_soc_estimate(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < HUB_BMS_CELL_COUNT; i++) {
        sum += s_cell_mv[i];
    }
    int32_t avg = (int32_t)(sum / HUB_BMS_CELL_COUNT);
    /* Naive rest-voltage map 3.00 V -> 0 %, 4.20 V -> 100 %; coulomb counting
     * is a bring-up upgrade, not a contract item. */
    int32_t soc = (avg - 3000) * 100 / 1200;
    if (soc < 0) soc = 0;
    if (soc > 100) soc = 100;
    return (uint8_t)soc;
}

static void bms_reports(void)
{
    if (s_tick % 100u == 0u) { /* 10 Hz BMS_SUMMARY */
        int16_t tmax = s_ntc_cC[0];
        for (int i = 1; i < HUB_BMS_NTC_COUNT; i++) {
            if (s_ntc_cC[i] > tmax) tmax = s_ntc_cC[i];
        }
        int32_t tC = tmax / 100;
        ps_bms_summary_t sum = {
            .pack_cV = s_pack_cV,
            .current_cA = s_curr_cA,
            .soc_pct = bms_soc_estimate(),
            .temp_max_C = (int8_t)(tC > 127 ? 127 : (tC < -128 ? -128 : tC)),
            .fault_bits = s_fault_bits,
        };
        bms_push_record(ps_can_id_pack(PS_CLS_TELEM, PS_NODE_HUB, PS_NODE_ORCH,
                                       PS_T_BMS_SUMMARY, s_summary_seq++),
                        &sum, sizeof(sum));
    }
    if (s_tick % 1000u == 250u) { /* 1 Hz BMS_CELLS, rotating group */
        uint8_t g = s_cells_group;
        s_cells_group = (uint8_t)((g + 1u) % HUB_BMS_CELL_GROUPS);
        ps_bms_cells_t cells = {
            .group = g,
            .rsvd = 0,
            .mv = { s_cell_mv[3 * g], s_cell_mv[3 * g + 1], s_cell_mv[3 * g + 2] },
        };
        bms_push_record(ps_can_id_pack(PS_CLS_TELEM, PS_NODE_HUB, PS_NODE_ORCH,
                                       PS_T_BMS_CELLS, s_cells_seq++),
                        &cells, sizeof(cells));
    }
    if (s_tick % 1000u == 500u) { /* 1 Hz NODE_STATS */
        ps_can_stats_t c1, c2;
        ps_can_get_stats(s_can[0], &c1);
        ps_can_get_stats(s_can[1], &c2);
        ps_spib_stats_t sb;
        ps_spib_get_stats(&sb);
        uint32_t rx = (c1.rx_frames - s_prev_can[0].rx_frames) +
                      (c2.rx_frames - s_prev_can[1].rx_frames);
        uint32_t tx = (c1.tx_frames - s_prev_can[0].tx_frames) +
                      (c2.tx_frames - s_prev_can[1].tx_frames);
        uint32_t err = c1.bus_errors + c2.bus_errors + c1.rx_dropped +
                       c2.rx_dropped + sb.crc_errors + sb.seq_gaps +
                       sb.uplink_overflows;
        s_prev_can[0] = c1;
        s_prev_can[1] = c2;
        s_prev_spib = sb;
        ps_node_stats_t st = {
            .cpu_pct = 0, /* not instrumented on the hub yet */
            .state = hub_gateway_safety_state(),
            .rx_fps = (uint16_t)(rx > 0xFFFFu ? 0xFFFFu : rx),
            .tx_fps = (uint16_t)(tx > 0xFFFFu ? 0xFFFFu : tx),
            .err_cnt = (uint16_t)(err > 0xFFFFu ? 0xFFFFu : err),
        };
        bms_push_record(ps_can_id_pack(PS_CLS_TELEM, PS_NODE_HUB, PS_NODE_ORCH,
                                       PS_T_NODE_STATS, s_stats_seq++),
                        &st, sizeof(st));
    }
    if (s_tick % 1000u == 750u) { /* 1 Hz VERSION */
        ps_version_t ver = {
            .major = 0, .minor = 1, .patch = 0,
            .node_state = hub_gateway_safety_state(),
            .git_short = 0, /* stamped by the build system once it exists */
        };
        bms_push_record(ps_can_id_pack(PS_CLS_MGMT, PS_NODE_HUB, PS_NODE_ORCH,
                                       PS_T_VERSION, 0),
                        &ver, sizeof(ver));
    }
}

/* ------------- task + init ------------- */

static void bms_task(void *arg)
{
    (void)arg;
    for (;;) {
        /* 1-tick blocking take = ~1 kHz pacer (CONFIG_FREERTOS_HZ=1000) that
         * wakes immediately when the latch ISR notifies. */
        uint32_t notified = ulTaskNotifyTake(pdTRUE, 1);
        if (notified) {
            bms_handle_short_trip();
        }
        s_tick++;
        bms_sample_fast();
        bms_slow_slot();
        bms_check_thresholds();
        bms_reports();
        bms_try_rearm();
    }
}

static esp_err_t bms_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t u1 = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_unit_init_cfg_t u2 = { .unit_id = ADC_UNIT_2 };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&u1, &s_adc1), TAG, "adc1");
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&u2, &s_adc2), TAG, "adc2");
    adc_oneshot_chan_cfg_t cc = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    const adc_channel_t adc1_ch[] = {
        HUB_ADC1_CH_PACK_V, HUB_ADC1_CH_CURRENT,
        (adc_channel_t)(HUB_ADC1_CH_CELL_BASE + 0), (adc_channel_t)(HUB_ADC1_CH_CELL_BASE + 1),
        (adc_channel_t)(HUB_ADC1_CH_CELL_BASE + 2), (adc_channel_t)(HUB_ADC1_CH_CELL_BASE + 3),
        HUB_ADC1_CH_NTC0, HUB_ADC1_CH_NTC1,
    };
    for (size_t i = 0; i < sizeof(adc1_ch) / sizeof(adc1_ch[0]); i++) {
        ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc1, adc1_ch[i], &cc),
                            TAG, "adc1 chan");
    }
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc2, HUB_ADC2_CH_NTC2, &cc),
                        TAG, "adc2 chan");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc2, HUB_ADC2_CH_NTC3, &cc),
                        TAG, "adc2 chan");
    return ESP_OK;
}

esp_err_t hub_bms_start(ps_can_handle_t can1, ps_can_handle_t can2)
{
    if (can1 == NULL || can2 == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_can[0] = can1;
    s_can[1] = can2;

    gpio_config_t in = {
        .pin_bit_mask = (1ULL << HUB_BMS_COMP_STATUS_GPIO) |
                        (1ULL << HUB_BMS_KILL_FB_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&in), TAG, "bms inputs");

    gpio_config_t out = {
        .pin_bit_mask = (1ULL << HUB_BMS_REARM_STROBE_GPIO) |
                        (1ULL << HUB_BMS_MUX_SEL0_GPIO) |
                        (1ULL << HUB_BMS_MUX_SEL1_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&out), TAG, "bms outputs");
    gpio_set_level(HUB_BMS_REARM_STROBE_GPIO, 0);
    gpio_set_level(HUB_BMS_MUX_SEL0_GPIO, 0);
    gpio_set_level(HUB_BMS_MUX_SEL1_GPIO, 0);

    /* Latch input: active LOW on trip, falling-edge interrupt. */
    gpio_config_t latch = {
        .pin_bit_mask = 1ULL << HUB_BMS_SC_LATCH_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&latch), TAG, "latch input");

    ESP_RETURN_ON_ERROR(bms_adc_init(), TAG, "adc");

    for (int i = 0; i < HUB_BMS_NTC_COUNT; i++) {
        s_ntc_cC[i] = 2500; /* assume 25 C until first real scan */
    }
    if (gpio_get_level(HUB_BMS_SC_LATCH_GPIO)) {
        s_fault_bits |= PS_BMSF_COMP_ARMED; /* latch idle-high = armed */
    } else {
        s_short_latched = true;
        s_fault_bits |= PS_BMSF_SHORT_LATCH; /* powered up into a tripped latch */
    }

    BaseType_t ok = xTaskCreatePinnedToCore(bms_task, "hub_bms", BMS_TASK_STACK,
                                            NULL, BMS_TASK_PRIO, &s_task,
                                            BMS_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    /* ISR service was installed with ESP_INTR_FLAG_IRAM in app_main; the task
     * handle must exist before the first edge can fire. */
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(HUB_BMS_SC_LATCH_GPIO,
                                             bms_sc_latch_isr, NULL),
                        TAG, "latch isr");
    ESP_LOGI(TAG, "BMS up: 12S scan/16ms, comparator %s",
             (s_fault_bits & PS_BMSF_COMP_ARMED) ? "armed" : "TRIPPED");
    return ESP_OK;
}
