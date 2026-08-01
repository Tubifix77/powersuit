/* On-target self-test, run under QEMU (esp32s3).
 *
 * This is NOT a duplicate of firmware/tests/host. Those suites prove the logic
 * is correct; this one proves it is still correct when compiled for Xtensa LX7
 * and run under FreeRTOS — which is a different claim. Specifically it reaches
 * four things a host test structurally cannot:
 *
 *   1. Packed-struct access on Xtensa, where unaligned loads are not free.
 *   2. The single-precision FPU, where the host quietly used x86 doubles.
 *   3. Real task scheduling on two cores, and the stack each task actually
 *      needs — the sizes in the apps were picked by guesswork.
 *   4. The safety state machine driven by a genuine 5 ms esp_timer against
 *      real elapsed time, rather than a clock the test hands it.
 *
 * QEMU does not emulate TWAI, MCPWM, I2S, the SPI slave or the ADC, so nothing
 * here touches them. That boundary is the whole reason this app exists instead
 * of just booting node_limb.
 *
 * Output is parsed by firmware/tools/qemu_test.sh: every check prints PASS or
 * FAIL, and the run ends with QEMU_SELFTEST_DONE.
 */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "powersuit_proto/adpcm.h"
#include "powersuit_proto/can_id.h"
#include "powersuit_proto/crc16.h"
#include "powersuit_proto/spi_frame.h"
#include "powersuit_proto/wire.h"
#include "ps_foc.h"
#include "ps_imu_filter.h"
#include "ps_router.h"
#include "ps_safety.h"

#include "vectors/proto_vectors.h"

static int g_pass, g_fail;

static void check(bool ok, const char *name, const char *detail)
{
    if (ok) {
        g_pass++;
        printf("PASS %s\n", name);
    } else {
        g_fail++;
        printf("FAIL %s : %s\n", name, detail ? detail : "");
    }
    fflush(stdout);
}

#define CHECK(cond, name) check((cond), (name), #cond)

/* ---------------- 1. wire layout on Xtensa ---------------- */

static void test_wire_layout(void)
{
    /* If the compiler padded any of these for alignment, the CAN payload would
     * silently change shape between host and target. */
    CHECK(sizeof(ps_heartbeat_t) == 8, "wire/heartbeat_is_8_bytes");
    CHECK(sizeof(ps_joint_state_t) == 8, "wire/joint_state_is_8_bytes");
    CHECK(sizeof(ps_bms_summary_t) == 8, "wire/bms_summary_is_8_bytes");
    CHECK(sizeof(ps_estop_t) == 8, "wire/estop_is_8_bytes");

    /* Round-trip through a deliberately odd address: packed structs get
     * memcpy'd out of CAN buffers that carry no alignment guarantee, and
     * Xtensa traps or silently misreads unaligned words where x86 shrugs. */
    static uint8_t backing[16];
    uint8_t *unaligned = backing + 1;
    ps_joint_state_t js = {
        .joint = 1, .flags = 0x03, .pos_crad = -12345,
        .vel_crad_s = 32000, .eff_cNm = -1,
    };
    PS_WIRE_WRITE(unaligned, js);
    ps_joint_state_t back;
    memset(&back, 0, sizeof(back));
    PS_WIRE_READ(back, unaligned);
    CHECK(back.pos_crad == -12345 && back.vel_crad_s == 32000 && back.eff_cNm == -1 &&
              back.joint == 1 && back.flags == 0x03,
          "wire/unaligned_roundtrip");

    /* Little-endian on the wire, whatever the compiler thinks. */
    ps_clear_estop_t ce = { .magic = PS_CLEAR_ESTOP_MAGIC, .counter = 7 };
    uint8_t raw[8];
    PS_WIRE_WRITE(raw, ce);
    CHECK(raw[0] == 0x3A && raw[1] == 0xC1 && raw[2] == 0xA4 && raw[3] == 0x52,
          "wire/clear_estop_is_little_endian");
}

/* ---------------- 2. protocol codecs ---------------- */

static void test_protocol(void)
{
    CHECK(ps_crc16((const uint8_t *)"123456789", 9) == 0x29B1, "crc16/known_answer");

    uint32_t id = ps_can_id_pack(PS_CLS_TELEM, PS_NODE_ARM_R, PS_NODE_ORCH,
                                 PS_T_JOINT_STATE, 5);
    ps_can_id_t f;
    ps_can_id_unpack(id, &f);
    CHECK(f.cls == PS_CLS_TELEM && f.src == PS_NODE_ARM_R && f.dst == PS_NODE_ORCH &&
              f.type == PS_T_JOINT_STATE && f.low == 5,
          "can_id/roundtrip");

    /* Parse the frame the Python implementation generated. Same bytes, same
     * verdict, on a different architecture. */
    ps_spi_view_t view;
    CHECK(ps_spi_frame_parse(PS_VEC_SPI_FRAME, PS_VEC_SPI_FRAME_LEN, &view) == PS_SPI_OK &&
              view.count == PS_VEC_SPI_COUNT && view.seq == PS_VEC_SPI_SEQ,
          "spi_frame/parses_python_vector");

    /* SAFETY must win arbitration from any source. */
    CHECK(ps_can_id_pack(PS_CLS_SAFETY, 31, 31, 0xFF, 0xFF) <
              ps_can_id_pack(PS_CLS_CONTROL, 0, 0, 0, 0),
          "can_id/safety_wins_arbitration");

    /* Routing: an e-stop from bus 1 must reach bus 2, the orchestrator, and
     * the hub itself. */
    uint8_t mask = ps_router_route(PS_PORT_CAN1,
                                   ps_can_id_pack(PS_CLS_SAFETY, PS_NODE_ARM_R,
                                                  PS_NODE_BROADCAST, PS_T_ESTOP, 0));
    CHECK((mask & PS_PORT_BIT(PS_PORT_CAN2)) && (mask & PS_PORT_BIT(PS_PORT_SPI)),
          "router/estop_cuts_through");
}

/* ---------------- 3. ADPCM against the shared vectors ---------------- */

static void test_adpcm(void)
{
    static uint8_t enc[PS_VEC_ADPCM_N / 2];
    ps_adpcm_state_t st;
    ps_adpcm_init(&st);
    ps_adpcm_encode(&st, PS_VEC_ADPCM_PCM, PS_VEC_ADPCM_N, enc);

    CHECK(memcmp(enc, PS_VEC_ADPCM_ENC, sizeof(enc)) == 0, "adpcm/encode_matches_vector");
    CHECK(st.predictor == PS_VEC_ADPCM_FINAL_PREDICTOR &&
              st.step_index == PS_VEC_ADPCM_FINAL_STEP_INDEX,
          "adpcm/final_state_matches_vector");

    static int16_t dec[PS_VEC_ADPCM_N];
    ps_adpcm_state_t dst;
    ps_adpcm_init(&dst);
    ps_adpcm_decode(&dst, enc, sizeof(enc), dec);
    CHECK(memcmp(dec, PS_VEC_ADPCM_DEC, sizeof(dec)) == 0, "adpcm/decode_matches_vector");
}

/* ---------------- 4. control maths on the target FPU ---------------- */

static void test_control_math(void)
{
    /* The host ran this through x86; the S3 has a single-precision FPU and
     * -ffast-math-ish codegen differences are exactly where drift appears. */
    for (int deg = 0; deg < 360; deg += 30) {
        float th = (float)deg * (float)M_PI / 180.0f;
        ps_ab_t ab = { 0.37f, -0.82f }, back;
        ps_dq_t dq;
        ps_foc_park(&ab, th, &dq);
        ps_foc_inv_park(&dq, th, &back);
        if (fabsf(back.alpha - ab.alpha) > 1e-4f || fabsf(back.beta - ab.beta) > 1e-4f) {
            check(false, "foc/park_roundtrip_on_fpu", "drift beyond 1e-4");
            return;
        }
    }
    check(true, "foc/park_roundtrip_on_fpu", NULL);

    ps_ab_t zero = { 0.0f, 0.0f };
    float duty[3];
    ps_foc_svpwm(&zero, 24.0f, duty);
    CHECK(fabsf(duty[0] - 0.5f) < 1e-5f && fabsf(duty[1] - 0.5f) < 1e-5f &&
              fabsf(duty[2] - 0.5f) < 1e-5f,
          "foc/zero_vector_is_midpoint");

    ps_mahony_t m;
    ps_mahony_init(&m, 2.0f, 0.0f);
    for (int i = 0; i < 1250; i++) {
        ps_mahony_update(&m, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.8660254f, 1.0f / 250.0f);
    }
    float vx = 2.0f * (m.q1 * m.q3 - m.q0 * m.q2);
    float vz = m.q0 * m.q0 - m.q1 * m.q1 - m.q2 * m.q2 + m.q3 * m.q3;
    CHECK(vx * 0.5f + vz * 0.8660254f > cosf(2.0f * (float)M_PI / 180.0f),
          "mahony/converges_on_target_fpu");

    float q[4];
    ps_mahony_get_quat(&m, q);
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    CHECK(fabsf(n - 1.0f) < 1e-3f, "mahony/quaternion_normalised");
}

/* ---------------- 5. safety state machine on a real timer ---------------- */

static ps_safety_core_t s_core;
static volatile uint32_t s_tick_count;
static SemaphoreHandle_t s_core_lock;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void safety_tick_cb(void *arg)
{
    (void)arg;
    xSemaphoreTake(s_core_lock, portMAX_DELAY);
    ps_safety_core_tick(&s_core, now_ms());
    s_tick_count++;
    xSemaphoreGive(s_core_lock);
}

static void beat(uint16_t seq)
{
    ps_heartbeat_t hb;
    memset(&hb, 0, sizeof(hb));
    hb.seq = seq;
    xSemaphoreTake(s_core_lock, portMAX_DELAY);
    ps_safety_core_on_heartbeat(&s_core, now_ms(), &hb);
    xSemaphoreGive(s_core_lock);
}

static void test_safety_realtime(void)
{
    s_core_lock = xSemaphoreCreateMutex();
    ps_safety_core_init(&s_core, now_ms(), NULL, NULL);

    esp_timer_handle_t t;
    const esp_timer_create_args_t args = {
        .callback = safety_tick_cb, .name = "safety", .dispatch_method = ESP_TIMER_TASK,
    };
    if (esp_timer_create(&args, &t) != ESP_OK || esp_timer_start_periodic(t, 5000) != ESP_OK) {
        check(false, "safety/timer_started", "esp_timer setup failed");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    CHECK(s_tick_count >= 5, "safety/timer_runs_at_5ms");

    /* Arm it the way the orchestrator does, using wall-clock time. */
    uint16_t seq = 0;
    ps_safety_core_on_mode_set(&s_core, now_ms(), PS_STATE_OPERATIONAL);
    for (int i = 0; i < 30; i++) {
        beat(++seq);
        ps_safety_core_on_cmd(&s_core, now_ms());
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    CHECK(s_core.state == PS_STATE_OPERATIONAL, "safety/arms_under_real_time");

    /* Stop beating and let genuine elapsed time trip the watchdog. t0 is the
     * moment of the LAST beat, not the moment we stopped looping — otherwise
     * the trailing vTaskDelay is silently excluded and the measured latency
     * reads ~10 ms early. */
    beat(++seq);
    int64_t t0 = esp_timer_get_time();
    while (s_core.state == PS_STATE_OPERATIONAL &&
           (esp_timer_get_time() - t0) < 500000) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    int64_t elapsed_us = esp_timer_get_time() - t0;
    CHECK(s_core.state == PS_STATE_PASSIVE, "safety/trips_on_real_silence");

    /* The contract is 50 ms. Allow scheduling slop but catch an order-of-
     * magnitude error, which is what a bad time base would produce. */
    printf("INFO watchdog tripped after %" PRId64 " us\n", elapsed_us);
    CHECK(elapsed_us > 40000 && elapsed_us < 150000, "safety/trip_latency_is_sane");

    esp_timer_stop(t);
    esp_timer_delete(t);
}

/* ---------------- 6. FreeRTOS: cores and stack headroom ---------------- */

static volatile int s_core_seen[2];
static volatile UBaseType_t s_headroom[2];

static void probe_task(void *arg)
{
    int idx = (int)(intptr_t)arg;
    /* Burn a representative amount of stack: the real tasks build wire structs
     * and format log lines on theirs. */
    char scratch[512];
    snprintf(scratch, sizeof(scratch), "probe %d on core %d", idx, xPortGetCoreID());
    s_core_seen[idx] = xPortGetCoreID();
    s_headroom[idx] = uxTaskGetStackHighWaterMark(NULL);
    (void)scratch;
    vTaskDelete(NULL);
}

static void test_freertos(void)
{
    /* The apps pin comms to core 0 and control to core 1 (CLAUDE.md). Prove
     * both cores actually schedule. */
    s_core_seen[0] = s_core_seen[1] = -1;
    xTaskCreatePinnedToCore(probe_task, "probe0", 3072, (void *)0, 10, NULL, 0);
    xTaskCreatePinnedToCore(probe_task, "probe1", 3072, (void *)1, 10, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    CHECK(s_core_seen[0] == 0, "freertos/task_pinned_to_core0");
    CHECK(s_core_seen[1] == 1, "freertos/task_pinned_to_core1");

    /* Smallest free stack ever seen. ESP-IDF reports this in BYTES, unlike
     * vanilla FreeRTOS which uses words — getting that wrong overstates the
     * headroom fourfold. A 3072-byte task reporting ~1150 here has used about
     * two thirds of its stack, which is the real number the app task sizes
     * should be judged against. */
    printf("INFO 3072B task stack free: core0=%u bytes, core1=%u bytes\n",
           (unsigned)s_headroom[0], (unsigned)s_headroom[1]);
    CHECK(s_headroom[0] > 512 && s_headroom[1] > 512, "freertos/3072B_task_has_headroom");

    CHECK(configNUM_CORES == 2, "freertos/dual_core_config");
    CHECK(configTICK_RATE_HZ == 1000, "freertos/tick_is_1khz");
}

void app_main(void)
{
    printf("\nQEMU_SELFTEST_BEGIN\n");
    printf("INFO target=%s cores=%d tick=%dHz\n", CONFIG_IDF_TARGET,
           (int)configNUM_CORES, (int)configTICK_RATE_HZ);

    test_wire_layout();
    test_protocol();
    test_adpcm();
    test_control_math();
    test_safety_realtime();
    test_freertos();

    printf("\nQEMU_SELFTEST_SUMMARY pass=%d fail=%d\n", g_pass, g_fail);
    printf("QEMU_SELFTEST_DONE %s\n", g_fail == 0 ? "OK" : "FAILED");
    fflush(stdout);

    /* Idle rather than return: the runner kills QEMU once it sees DONE. */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
