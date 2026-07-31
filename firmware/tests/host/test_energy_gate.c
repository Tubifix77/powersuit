#include "ps_audio.h"
#include "ps_test.h"

/* Contract numbers under test (docs/safety.md §6): -38 dBFS threshold over
 * 32 ms blocks, 300 ms hang, 8 s utterance cap. Blocks are 256 samples at
 * 8 kHz = 32 ms; now_ms advances 32 per block. */
#define BLK        256u
#define BLK_MS     32u
#define THRESH_DB  (-38)
#define HANG_MS    300u
#define MAX_UTT_MS 8000u

#define T_PI 3.14159265358979323846

static int16_t sine_blk[BLK];
static int16_t zero_blk[BLK];

static void make_blocks(void)
{
    /* -30 dBFS sine: mean square = A^2/2 = 10^(-3) * 32768^2  =>  A ~= 1466. */
    size_t i;
    for (i = 0; i < BLK; i++) {
        sine_blk[i] = (int16_t)(1466.0 * sin(2.0 * T_PI * 440.0 * (double)i / 8000.0));
        zero_blk[i] = 0;
    }
}

PS_TEST(silence_keeps_gate_closed)
{
    ps_egate_t g;
    uint32_t now = 0;
    int i;

    make_blocks();
    ps_egate_init(&g, THRESH_DB, HANG_MS, MAX_UTT_MS);
    for (i = 0; i < 100; i++) {
        PS_ASSERT_FALSE(ps_egate_process(&g, zero_blk, BLK, now));
        now += BLK_MS;
    }
}

PS_TEST(minus30_sine_opens_at_minus38_threshold)
{
    ps_egate_t g;

    make_blocks();
    ps_egate_init(&g, THRESH_DB, HANG_MS, MAX_UTT_MS);
    /* Sanity: -38 dBFS threshold ~= 170e3 mean-square units. */
    PS_ASSERT_TRUE(g.threshold_q > 150000 && g.threshold_q < 190000);
    PS_ASSERT_TRUE(ps_egate_process(&g, sine_blk, BLK, 0));
}

PS_TEST(hang_covers_200ms_silence_then_closes)
{
    ps_egate_t g;
    uint32_t now = 0;
    int i;

    make_blocks();
    ps_egate_init(&g, THRESH_DB, HANG_MS, MAX_UTT_MS);
    /* 4 voiced blocks: last voiced at now = 96 ms. */
    for (i = 0; i < 4; i++) {
        PS_ASSERT_TRUE(ps_egate_process(&g, sine_blk, BLK, now));
        now += BLK_MS;
    }
    /* Silence: gate must survive 200 ms (< 300 ms hang)... */
    while (now - 96u < 200u) {
        PS_ASSERT_TRUE(ps_egate_process(&g, zero_blk, BLK, now));
        now += BLK_MS;
    }
    /* ...stay open until the hang expires... */
    while (now - 96u < HANG_MS) {
        PS_ASSERT_TRUE(ps_egate_process(&g, zero_blk, BLK, now));
        now += BLK_MS;
    }
    /* ...and close on the first block at/after last_voiced + hang. */
    PS_ASSERT_FALSE(ps_egate_process(&g, zero_blk, BLK, now));
}

PS_TEST(max_utterance_force_closes_after_8s)
{
    ps_egate_t g;
    uint32_t now = 0;

    make_blocks();
    ps_egate_init(&g, THRESH_DB, HANG_MS, MAX_UTT_MS);
    /* Continuous speech: blocks at 0, 32, ..., 7968 ms all keep it open. */
    while (now < MAX_UTT_MS) {
        PS_ASSERT_TRUE(ps_egate_process(&g, sine_blk, BLK, now));
        now += BLK_MS;
    }
    /* Block at 8000 ms: cap reached — closes even though still voiced. */
    PS_ASSERT_FALSE(ps_egate_process(&g, sine_blk, BLK, now));
    now += BLK_MS;
    /* Re-arm hysteresis: voiced blocks may NOT reopen until a quiet block. */
    PS_ASSERT_FALSE(ps_egate_process(&g, sine_blk, BLK, now));
    now += BLK_MS;
    PS_ASSERT_FALSE(ps_egate_process(&g, zero_blk, BLK, now)); /* quiet: re-armed */
    now += BLK_MS;
    PS_ASSERT_TRUE(ps_egate_process(&g, sine_blk, BLK, now));  /* fresh utterance */
}

PS_TEST_MAIN()
