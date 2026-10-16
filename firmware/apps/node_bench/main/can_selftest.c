/* can_selftest — one board, no wiring.
 *
 * The TWAI controller runs in self-test + loopback mode with RX on the TX pin: it
 * hears every frame it sends and needs no ACK, so no transceiver and no second
 * board are involved. That makes it the first place ps_can's transmit path runs on
 * silicon — above all the class ordering of ps_can_txq (network-map §2), which the
 * host tests can only check in isolation, without the driver, its ISR or the
 * transmit task.
 *
 * In loopback the receive order is the transmit order, so the order frames come
 * back in is the order ps_can put them on the wire. The decisive check queues a
 * dozen TELEM frames and then one SAFETY frame: SAFETY must come back near the
 * front. In arrival order — what the driver's own FIFO does — it would come last.
 *
 * Output is line-oriented for a script to read: "SELFTEST PASS|FAIL <name>: ...",
 * then "SELFTEST_DONE OK|FAIL <failures>". */
#include "can_selftest.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "ps_can.h"
#include "powersuit_proto/can_id.h"

static const char *TAG = "selftest";

#define LOG_MAX     64
#define N_BASIC     200
#define N_QUEUED    12      /* TELEM frames queued ahead of the SAFETY frame (< 16 slots) */
#define N_LAT_IDLE  50
#define N_LAT_LOAD  20

typedef struct {
    uint8_t cls;
    uint8_t low;
    int64_t t_us;
} rx_rec_t;

/* Written only by ps_can's rx task, read by this task after waiting on s_n. */
static rx_rec_t          s_log[LOG_MAX];
static volatile uint32_t s_n;
static ps_can_handle_t   s_can;
static int               s_fail;

static void on_rx(const ps_can_frame_t *f, void *arg)
{
    (void)arg;
    int64_t t = esp_timer_get_time();
    uint32_t i = s_n;
    if (i < LOG_MAX) {
        s_log[i].cls = ps_can_id_cls(f->id);
        s_log[i].low = ps_can_id_low(f->id);
        s_log[i].t_us = t;
    }
    s_n = i + 1;
}

static bool wait_rx(uint32_t n, int timeout_ms)
{
    int64_t end = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (s_n < n && esp_timer_get_time() < end) {
        vTaskDelay(1);
    }
    return s_n >= n;
}

static esp_err_t send(uint8_t cls, uint8_t type, uint8_t low, TickType_t timeout)
{
    ps_can_frame_t f;
    f.id = ps_can_id_pack(cls, 1, PS_NODE_BROADCAST, type, low);
    f.dlc = 8;
    memset(f.data, low, sizeof(f.data));
    return ps_can_send(s_can, &f, timeout);
}

static void report(bool ok, const char *name, const char *fmt, ...)
{
    va_list ap;
    printf("SELFTEST %s %s: ", ok ? "PASS" : "FAIL", name);
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    if (!ok) {
        s_fail++;
    }
}

/* Index of the first SAFETY frame among the first n received, or -1. */
static int safety_pos(uint32_t n)
{
    for (uint32_t i = 0; i < n && i < LOG_MAX; i++) {
        if (s_log[i].cls == PS_CLS_SAFETY) {
            return (int)i;
        }
    }
    return -1;
}

static void check_basic(void)
{
    ps_can_stats_t before, after;
    ps_can_get_stats(s_can, &before);
    s_n = 0;
    int refused = 0;
    for (int i = 0; i < N_BASIC; i++) {
        if (send(PS_CLS_TELEM, PS_T_JOINT_STATE, (uint8_t)i, pdMS_TO_TICKS(50)) != ESP_OK) {
            refused++;
        }
    }
    bool all = wait_rx(N_BASIC, 2000);
    vTaskDelay(pdMS_TO_TICKS(20));
    ps_can_get_stats(s_can, &after);
    uint32_t tx = after.tx_frames - before.tx_frames;
    uint32_t err = after.bus_errors - before.bus_errors;
    uint32_t drop = after.tx_timeouts - before.tx_timeouts;
    report(all && refused == 0 && tx == N_BASIC && err == 0 && drop == 0, "basic",
           "%" PRIu32 "/%d frames back, tx_frames %" PRIu32 ", bus_errors %" PRIu32
           ", tx_timeouts %" PRIu32 ", refused %d",
           s_n, N_BASIC, tx, err, drop, refused);
}

/* Queue N_QUEUED TELEM frames, then one SAFETY frame; return the SAFETY frame's
 * 0-based position among what came back, and how many frames had already come back
 * when it was queued. -1 on a refusal or timeout. */
static int queue_and_find_safety(uint32_t *back_before)
{
    vTaskDelay(pdMS_TO_TICKS(20));          /* all slots free, nothing in flight */
    s_n = 0;
    for (int i = 0; i < N_QUEUED; i++) {
        if (send(PS_CLS_TELEM, PS_T_JOINT_STATE, (uint8_t)i, 0) != ESP_OK) {
            return -1;
        }
    }
    *back_before = s_n;
    if (send(PS_CLS_SAFETY, PS_T_HEARTBEAT, 0xAA, 0) != ESP_OK) {
        return -1;
    }
    return wait_rx(N_QUEUED + 1, 500) ? safety_pos(N_QUEUED + 1) : -1;
}

/* A SAFETY frame queued behind N_QUEUED TELEM frames must overtake them. */
static void check_order(void)
{
    /* Decisive case: run above ps_can's transmit task, so nothing can leave until
     * all thirteen frames are queued. The transmit task shares core 0 with this
     * one; on another core it could start early and the case would not be clean. */
    uint32_t back = 0;
    int core = xPortGetCoreID();
    UBaseType_t prio = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    int pos = queue_and_find_safety(&back);
    vTaskPrioritySet(NULL, prio);
    report(pos == 0 && back == 0 && core == 0, "order",
           "all %d queued before any left: SAFETY came back at position %d "
           "(in arrival order it would be %d; core %d)",
           N_QUEUED + 1, pos + 1, N_QUEUED + 1, core);

    /* The TELEM frames themselves must keep their order: FIFO within a class. */
    int expect = 0;
    bool fifo = pos >= 0;
    for (uint32_t i = 0; i < (uint32_t)(N_QUEUED + 1) && i < LOG_MAX; i++) {
        if (s_log[i].cls != PS_CLS_TELEM) {
            continue;
        }
        if (s_log[i].low != expect) {
            fifo = false;
            break;
        }
        expect++;
    }
    report(fifo && expect == N_QUEUED, "fifo",
           "%d of %d TELEM frames back in the order sent", expect, N_QUEUED);

    /* Realistic case: the transmit task runs while frames are being queued, so some
     * leave before SAFETY is queued. SAFETY may follow only those, the one on the
     * wire when it arrived, and one more that can start in the microseconds between
     * counting and queuing — it must overtake everything else still waiting. */
    pos = queue_and_find_safety(&back);
    report(pos >= 0 && (uint32_t)pos <= back + 2, "order_concurrent",
           "%" PRIu32 " back before SAFETY was queued; SAFETY came back at position %d "
           "(may be at most %" PRIu32 "; in arrival order %d)",
           back, pos + 1, back + 3, N_QUEUED + 1);
}

/* Send-to-receive time for one SAFETY frame. Loopback receives a frame when its
 * transmission completes, so this is queueing + wire time + both ISR-to-task hops. */
static void check_latency(void)
{
    /* Generous bounds: about six 150-bit frame times at the configured rate. At
     * 1 Mbit/s that is ~1 ms, where the FIFO inversion this guards against would
     * cost N_QUEUED frames, ~2 ms, under load. */
    const int64_t bound_us = (int64_t)900 * 1000000 / CONFIG_PS_CAN_BITRATE + 100;
    int64_t lo = INT64_MAX, hi = 0;
    bool ok = true;

    for (int i = 0; i < N_LAT_IDLE; i++) {
        vTaskDelay(pdMS_TO_TICKS(2));
        s_n = 0;
        int64_t t0 = esp_timer_get_time();
        if (send(PS_CLS_SAFETY, PS_T_HEARTBEAT, (uint8_t)i, 0) != ESP_OK || !wait_rx(1, 100)) {
            ok = false;
            break;
        }
        int64_t d = s_log[0].t_us - t0;
        lo = d < lo ? d : lo;
        hi = d > hi ? d : hi;
    }
    report(ok && hi <= bound_us, "latency_idle",
           "SAFETY send to receive %" PRId64 "-%" PRId64 " us over %d frames (bound %" PRId64 " us)",
           lo, hi, N_LAT_IDLE, bound_us);

    lo = INT64_MAX;
    hi = 0;
    ok = true;
    for (int i = 0; i < N_LAT_LOAD; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        s_n = 0;
        for (int k = 0; k < N_QUEUED; k++) {
            (void)send(PS_CLS_TELEM, PS_T_JOINT_STATE, (uint8_t)k, 0);
        }
        int64_t t0 = esp_timer_get_time();
        if (send(PS_CLS_SAFETY, PS_T_HEARTBEAT, (uint8_t)i, 0) != ESP_OK ||
            !wait_rx(N_QUEUED + 1, 500)) {
            ok = false;
            break;
        }
        int pos = safety_pos(N_QUEUED + 1);
        if (pos < 0) {
            ok = false;
            break;
        }
        int64_t d = s_log[pos].t_us - t0;
        lo = d < lo ? d : lo;
        hi = d > hi ? d : hi;
    }
    report(ok && hi <= bound_us, "latency_loaded",
           "SAFETY behind %d queued TELEM: %" PRId64 "-%" PRId64 " us over %d runs (bound %" PRId64 " us)",
           N_QUEUED, lo, hi, N_LAT_LOAD, bound_us);
}

bool bench_can_selftest(void)
{
    ESP_LOGW(TAG, "CAN self-test: loopback on GPIO%d, %d bit/s. No transceiver needed; "
                  "disconnect one if attached.",
             CONFIG_PS_BENCH_CAN_TX_GPIO, CONFIG_PS_CAN_BITRATE);

    ps_can_config_t cfg = {
        .controller = 0,
        .tx_gpio = CONFIG_PS_BENCH_CAN_TX_GPIO,
        .rx_gpio = CONFIG_PS_BENCH_CAN_TX_GPIO,   /* RX on the TX pin, per Espressif's own test */
        .bitrate = CONFIG_PS_CAN_BITRATE,
        .node_id = 0,                              /* promiscuous: hear our own broadcasts */
        .loopback = true,
    };
    if (ps_can_open(&cfg, &s_can) != ESP_OK) {
        report(false, "open", "ps_can_open failed");
        printf("SELFTEST_DONE FAIL %d\n", s_fail);
        return false;
    }
    ps_can_register_class_cb(s_can, PS_CLS_SAFETY, on_rx, NULL);
    ps_can_register_class_cb(s_can, PS_CLS_TELEM, on_rx, NULL);

    check_basic();
    check_order();
    check_latency();

    ps_can_stats_t st;
    ps_can_get_stats(s_can, &st);
    printf("SELFTEST stats: rx %" PRIu32 " tx %" PRIu32 " rx_dropped %" PRIu32
           " tx_timeouts %" PRIu32 " bus_errors %" PRIu32 "\n",
           st.rx_frames, st.tx_frames, st.rx_dropped, st.tx_timeouts, st.bus_errors);
    printf("SELFTEST_DONE %s %d\n", s_fail ? "FAIL" : "OK", s_fail);
    return s_fail == 0;
}
