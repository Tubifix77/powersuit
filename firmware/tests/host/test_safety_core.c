/* Exhaustive coverage of the suit safety state machine (docs/safety.md §2).
 * Every rule that keeps force off the wearer is asserted here; this suite is the
 * reason the core is pure C. */
#include "ps_safety.h"
#include "ps_test.h"

typedef struct {
    uint8_t old_s, new_s, cause;
} trans_t;

static trans_t g_trans[64];
static int g_ntrans;
static uint16_t g_seq;

static void record(uint8_t old_s, uint8_t new_s, uint8_t cause, void *arg)
{
    (void)arg;
    if (g_ntrans < (int)(sizeof(g_trans) / sizeof(g_trans[0]))) {
        g_trans[g_ntrans].old_s = old_s;
        g_trans[g_ntrans].new_s = new_s;
        g_trans[g_ntrans].cause = cause;
        g_ntrans++;
    }
}

static void beat_at(ps_safety_core_t *c, uint32_t t, uint8_t flags)
{
    ps_heartbeat_t hb;
    memset(&hb, 0, sizeof(hb));
    hb.seq = ++g_seq;
    hb.flags = flags;
    ps_safety_core_on_heartbeat(c, t, &hb);
}

/* Bring a freshly initialised core to OPERATIONAL; returns the current time. */
static uint32_t bring_up(ps_safety_core_t *c)
{
    g_ntrans = 0;
    g_seq = 0;
    ps_safety_core_init(c, 0, record, NULL);
    ps_safety_core_tick(c, 0);
    ps_safety_core_on_mode_set(c, 0, PS_STATE_OPERATIONAL);
    beat_at(c, 10, 0);
    ps_safety_core_tick(c, 10);
    return 10;
}

PS_TEST(boot_then_standby)
{
    ps_safety_core_t c;
    g_ntrans = 0;
    ps_safety_core_init(&c, 0, record, NULL);
    PS_ASSERT_EQ_INT(c.state, PS_STATE_BOOT);
    PS_ASSERT_EQ_INT(ps_safety_core_tick(&c, 0), PS_STATE_STANDBY);
    PS_ASSERT_EQ_INT(g_ntrans, 1);
    PS_ASSERT_EQ_INT(g_trans[0].old_s, PS_STATE_BOOT);
    PS_ASSERT_EQ_INT(g_trans[0].new_s, PS_STATE_STANDBY);
}

PS_TEST(standby_needs_both_mode_and_heartbeat)
{
    ps_safety_core_t c;
    ps_safety_core_init(&c, 0, record, NULL);
    ps_safety_core_tick(&c, 0);

    /* Heartbeats alone must not arm the node. */
    beat_at(&c, 10, 0);
    PS_ASSERT_EQ_INT(ps_safety_core_tick(&c, 10), PS_STATE_STANDBY);

    /* Mode intent alone (heartbeat gone stale) must not arm it either. */
    ps_safety_core_on_mode_set(&c, 200, PS_STATE_OPERATIONAL);
    PS_ASSERT_EQ_INT(ps_safety_core_tick(&c, 200), PS_STATE_STANDBY);

    beat_at(&c, 210, 0);
    PS_ASSERT_EQ_INT(ps_safety_core_tick(&c, 210), PS_STATE_OPERATIONAL);
}

PS_TEST(watchdog_boundary_is_50ms)
{
    ps_safety_core_t c;
    uint32_t t = bring_up(&c);
    PS_ASSERT_EQ_INT(c.state, PS_STATE_OPERATIONAL);

    /* 49 ms of silence is tolerated (>= 4 lost beats of margin by design). */
    PS_ASSERT_EQ_INT(ps_safety_core_tick(&c, t + 49), PS_STATE_OPERATIONAL);
    /* 51 ms is not. */
    PS_ASSERT_EQ_INT(ps_safety_core_tick(&c, t + 51), PS_STATE_PASSIVE);
    PS_ASSERT_EQ_INT(g_trans[g_ntrans - 1].cause, PS_ESTOP_COMM_LOSS);
}

PS_TEST(rearm_requires_streak_and_fresh_command)
{
    ps_safety_core_t c;
    uint32_t t = bring_up(&c);
    ps_safety_core_tick(&c, t + 51);
    PS_ASSERT_EQ_INT(c.state, PS_STATE_PASSIVE);

    /* Heartbeats resume at t = 100; the streak starts there. */
    for (uint32_t k = 100; k <= 340; k += 10) {
        beat_at(&c, k, 0);
        ps_safety_core_tick(&c, k);
    }
    /* 240 ms of streak with no command yet: still passive. */
    PS_ASSERT_EQ_INT(c.state, PS_STATE_PASSIVE);

    /* A command inside the healthy streak, but the window is still short. */
    ps_safety_core_on_cmd(&c, 340);
    beat_at(&c, 345, 0);
    PS_ASSERT_EQ_INT(ps_safety_core_tick(&c, 345), PS_STATE_PASSIVE);

    /* 250 ms of continuous heartbeat AND a fresh command: armed. */
    beat_at(&c, 350, 0);
    PS_ASSERT_EQ_INT(ps_safety_core_tick(&c, 350), PS_STATE_OPERATIONAL);
}

PS_TEST(rearm_rejects_command_older_than_the_streak)
{
    ps_safety_core_t c;
    uint32_t t = bring_up(&c);
    /* Command arrives while the link is already dead. */
    ps_safety_core_on_cmd(&c, t + 20);
    ps_safety_core_tick(&c, t + 51);
    PS_ASSERT_EQ_INT(c.state, PS_STATE_PASSIVE);

    /* Beats resume; the stale pre-loss command must not count toward re-arm. */
    for (uint32_t k = 100; k <= 400; k += 10) {
        beat_at(&c, k, 0);
        ps_safety_core_tick(&c, k);
    }
    PS_ASSERT_EQ_INT(c.state, PS_STATE_PASSIVE);
}

PS_TEST(heartbeat_gap_restarts_the_streak)
{
    ps_safety_core_t c;
    uint32_t t = bring_up(&c);
    ps_safety_core_tick(&c, t + 51);

    for (uint32_t k = 100; k <= 300; k += 10) {
        beat_at(&c, k, 0);
        ps_safety_core_tick(&c, k);
    }
    ps_safety_core_on_cmd(&c, 300);

    /* A 60 ms hole resets the clock even though beats resume afterwards. */
    for (uint32_t k = 370; k <= 500; k += 10) {
        beat_at(&c, k, 0);
        ps_safety_core_on_cmd(&c, k);
        ps_safety_core_tick(&c, k);
    }
    /* Only 130 ms since the restart: not yet. */
    PS_ASSERT_EQ_INT(c.state, PS_STATE_PASSIVE);

    for (uint32_t k = 510; k <= 630; k += 10) {
        beat_at(&c, k, 0);
        ps_safety_core_on_cmd(&c, k);
        ps_safety_core_tick(&c, k);
    }
    PS_ASSERT_EQ_INT(c.state, PS_STATE_OPERATIONAL);
}

PS_TEST(duplicate_sequence_is_ignored)
{
    ps_safety_core_t c;
    ps_safety_core_init(&c, 0, record, NULL);
    ps_safety_core_tick(&c, 0);
    ps_safety_core_on_mode_set(&c, 0, PS_STATE_OPERATIONAL);

    ps_heartbeat_t hb;
    memset(&hb, 0, sizeof(hb));
    hb.seq = 7;
    ps_safety_core_on_heartbeat(&c, 10, &hb);
    PS_ASSERT_EQ_INT(ps_safety_core_tick(&c, 10), PS_STATE_OPERATIONAL);

    /* Replaying seq 7 must not refresh liveness. */
    ps_safety_core_on_heartbeat(&c, 100, &hb);
    PS_ASSERT_EQ_INT(ps_safety_core_tick(&c, 100), PS_STATE_PASSIVE);
}

PS_TEST(estop_latches_from_every_state)
{
    const uint8_t from[] = { PS_STATE_STANDBY, PS_STATE_OPERATIONAL, PS_STATE_PASSIVE };
    for (unsigned i = 0; i < sizeof(from) / sizeof(from[0]); i++) {
        ps_safety_core_t c;
        uint32_t t = bring_up(&c);
        if (from[i] == PS_STATE_STANDBY) {
            ps_safety_core_on_mode_set(&c, t, PS_STATE_STANDBY);
            ps_safety_core_tick(&c, t);
        } else if (from[i] == PS_STATE_PASSIVE) {
            ps_safety_core_tick(&c, t + 51);
            t += 51;
        }
        PS_ASSERT_EQ_INT(c.state, from[i]);

        ps_safety_core_on_estop(&c, t, PS_ESTOP_BMS_SHORT);
        PS_ASSERT_EQ_INT(c.state, PS_STATE_ESTOP);
        PS_ASSERT_EQ_INT(c.estop_cause, PS_ESTOP_BMS_SHORT);
        /* Heartbeats must not walk a latched node back out. */
        for (uint32_t k = t + 10; k <= t + 2000; k += 10) {
            beat_at(&c, k, 0);
            ps_safety_core_tick(&c, k);
        }
        PS_ASSERT_EQ_INT(c.state, PS_STATE_ESTOP);
    }
}

PS_TEST(estop_keeps_originating_cause)
{
    ps_safety_core_t c;
    uint32_t t = bring_up(&c);
    ps_safety_core_on_estop(&c, t, PS_ESTOP_BMS_SHORT);
    ps_safety_core_on_estop(&c, t + 1, PS_ESTOP_OPERATOR);
    PS_ASSERT_EQ_INT(c.estop_cause, PS_ESTOP_BMS_SHORT);
}

PS_TEST(heartbeat_flag_latches_estop)
{
    ps_safety_core_t c;
    uint32_t t = bring_up(&c);
    beat_at(&c, t + 10, PS_HB_ESTOP_LATCHED);
    PS_ASSERT_EQ_INT(c.state, PS_STATE_ESTOP);
}

PS_TEST(clear_rejects_bad_magic_and_replays)
{
    ps_safety_core_t c;
    uint32_t t = bring_up(&c);
    ps_safety_core_on_estop(&c, t, PS_ESTOP_OPERATOR);

    PS_ASSERT_FALSE(ps_safety_core_on_clear(&c, t, 0xDEADBEEFu, 1));
    PS_ASSERT_EQ_INT(c.estop_cause, PS_ESTOP_OPERATOR);

    /* Counter must be strictly greater than every previously accepted one. */
    PS_ASSERT_FALSE(ps_safety_core_on_clear(&c, t, PS_CLEAR_ESTOP_MAGIC, 0));
    PS_ASSERT_TRUE(ps_safety_core_on_clear(&c, t, PS_CLEAR_ESTOP_MAGIC, 5));
    PS_ASSERT_FALSE(ps_safety_core_on_clear(&c, t, PS_CLEAR_ESTOP_MAGIC, 5));
    PS_ASSERT_FALSE(ps_safety_core_on_clear(&c, t, PS_CLEAR_ESTOP_MAGIC, 4));
}

PS_TEST(clear_requires_full_rearm_window)
{
    ps_safety_core_t c;
    uint32_t t = bring_up(&c);
    ps_safety_core_on_estop(&c, t, PS_ESTOP_OPERATOR);
    PS_ASSERT_TRUE(ps_safety_core_on_clear(&c, t, PS_CLEAR_ESTOP_MAGIC, 1));

    /* Still latched: the clear only takes effect after a second of clean beats. */
    PS_ASSERT_EQ_INT(c.state, PS_STATE_ESTOP);
    uint32_t k = t + 10;
    for (; k <= t + 900; k += 10) {
        beat_at(&c, k, 0);
        ps_safety_core_tick(&c, k);
    }
    PS_ASSERT_EQ_INT(c.state, PS_STATE_ESTOP);

    for (; k <= t + 1100; k += 10) {
        beat_at(&c, k, 0);
        ps_safety_core_tick(&c, k);
    }
    PS_ASSERT_EQ_INT(c.state, PS_STATE_STANDBY);

    /* And it comes back disarmed: an explicit MODE_SET is required. */
    beat_at(&c, k, 0);
    PS_ASSERT_EQ_INT(ps_safety_core_tick(&c, k), PS_STATE_STANDBY);
}

PS_TEST(bus_down_trips_immediately)
{
    ps_safety_core_t c;
    uint32_t t = bring_up(&c);
    ps_safety_core_on_bus_down(&c, t);
    PS_ASSERT_EQ_INT(c.state, PS_STATE_PASSIVE);
    PS_ASSERT_EQ_INT(g_trans[g_ntrans - 1].cause, PS_ESTOP_COMM_LOSS);
}

PS_TEST(command_freshness_boundary)
{
    ps_safety_core_t c;
    ps_safety_core_init(&c, 0, record, NULL);
    /* A node that has never seen a command is never "fresh". */
    PS_ASSERT_FALSE(ps_safety_core_cmd_fresh(&c, 0));

    ps_safety_core_on_cmd(&c, 1000);
    PS_ASSERT_TRUE(ps_safety_core_cmd_fresh(&c, 1000 + PS_CMD_STALE_MS - 1));
    PS_ASSERT_FALSE(ps_safety_core_cmd_fresh(&c, 1000 + PS_CMD_STALE_MS));
}

PS_TEST(mode_set_standby_disarms)
{
    ps_safety_core_t c;
    uint32_t t = bring_up(&c);
    ps_safety_core_on_mode_set(&c, t, PS_STATE_STANDBY);
    PS_ASSERT_EQ_INT(c.state, PS_STATE_STANDBY);
    /* want_operational was cleared, so beats alone must not re-arm. */
    beat_at(&c, t + 10, 0);
    PS_ASSERT_EQ_INT(ps_safety_core_tick(&c, t + 10), PS_STATE_STANDBY);
}

PS_TEST(mode_set_estop_is_operator_latch)
{
    ps_safety_core_t c;
    uint32_t t = bring_up(&c);
    ps_safety_core_on_mode_set(&c, t, PS_STATE_ESTOP);
    PS_ASSERT_EQ_INT(c.state, PS_STATE_ESTOP);
    PS_ASSERT_EQ_INT(c.estop_cause, PS_ESTOP_OPERATOR);
}

PS_TEST(transition_sequence_is_exact)
{
    ps_safety_core_t c;
    uint32_t t = bring_up(&c);          /* BOOT->STANDBY, STANDBY->OPERATIONAL */
    ps_safety_core_tick(&c, t + 51);    /* OPERATIONAL->PASSIVE */
    ps_safety_core_on_estop(&c, t + 60, PS_ESTOP_THERMAL);  /* PASSIVE->ESTOP */

    PS_ASSERT_EQ_INT(g_ntrans, 4);
    PS_ASSERT_EQ_INT(g_trans[0].new_s, PS_STATE_STANDBY);
    PS_ASSERT_EQ_INT(g_trans[1].new_s, PS_STATE_OPERATIONAL);
    PS_ASSERT_EQ_INT(g_trans[2].new_s, PS_STATE_PASSIVE);
    PS_ASSERT_EQ_INT(g_trans[2].cause, PS_ESTOP_COMM_LOSS);
    PS_ASSERT_EQ_INT(g_trans[3].new_s, PS_STATE_ESTOP);
    PS_ASSERT_EQ_INT(g_trans[3].cause, PS_ESTOP_THERMAL);
}

PS_TEST(no_callback_on_same_state)
{
    ps_safety_core_t c;
    bring_up(&c);
    int before = g_ntrans;
    ps_safety_core_tick(&c, 20);
    ps_safety_core_tick(&c, 30);
    PS_ASSERT_EQ_INT(g_ntrans, before);
}

PS_TEST_MAIN()
