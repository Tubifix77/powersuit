/* ps_safety_core — the node safety state machine (docs/safety.md §2).
 * Pure C99: no RTOS, no IDF includes; host-tested by tests/host/test_safety_core.c.
 * This file is the single source of truth for suit safety transitions — the ESP
 * glue only feeds it events and reacts to its callbacks, it never adds rules.
 *
 * Two encodings worth knowing:
 *   - "clear accepted, awaiting re-arm" is state == ESTOP with estop_cause == 0.
 *   - last_cmd_ms / hb_streak_start_ms are seeded stale at init so a node that
 *     has never heard a command cannot be mistaken for a fresh one at t = 0.
 */
#include "ps_safety.h"

/* Wrap-safe elapsed time: valid while gaps stay under ~24 days. */
static int32_t elapsed(uint32_t now_ms, uint32_t then_ms)
{
    return (int32_t)(now_ms - then_ms);
}

static void go(ps_safety_core_t *c, uint8_t next, uint8_t cause)
{
    if (c->state == next) {
        return;
    }
    uint8_t old = c->state;
    c->state = next;
    if (c->on_transition) {
        c->on_transition(old, next, cause, c->cb_arg);
    }
}

static bool heartbeat_ok(const ps_safety_core_t *c, uint32_t now_ms)
{
    return c->have_hb && elapsed(now_ms, c->last_hb_ms) <= (int32_t)PS_HEARTBEAT_TIMEOUT_MS;
}

static int32_t streak_ms(const ps_safety_core_t *c, uint32_t now_ms)
{
    return c->have_hb ? elapsed(now_ms, c->hb_streak_start_ms) : 0;
}

void ps_safety_core_init(ps_safety_core_t *c, uint32_t now_ms,
                         ps_safety_transition_cb_t cb, void *arg)
{
    c->state = PS_STATE_BOOT;
    c->estop_cause = 0;
    c->last_hb_seq = 0;
    c->last_hb_ms = now_ms - PS_HEARTBEAT_TIMEOUT_MS - 1u;
    c->hb_streak_start_ms = c->last_hb_ms;
    c->last_cmd_ms = now_ms - PS_CMD_STALE_MS - 1u;
    c->clear_counter = 0;
    c->estop_since_ms = 0;
    c->have_hb = false;
    c->want_operational = false;
    c->on_transition = cb;
    c->cb_arg = arg;
}

void ps_safety_core_on_heartbeat(ps_safety_core_t *c, uint32_t now_ms,
                                 const ps_heartbeat_t *hb)
{
    /* A repeated sequence number is a duplicate or a replay, never proof of
     * liveness: ignore it entirely so it cannot extend a streak. */
    if (c->have_hb && hb->seq == c->last_hb_seq) {
        return;
    }

    /* Any gap longer than the watchdog window breaks the streak: re-arm timing
     * must measure uninterrupted liveness, not total beats received. */
    if (!c->have_hb || elapsed(now_ms, c->last_hb_ms) > (int32_t)PS_HEARTBEAT_TIMEOUT_MS) {
        c->hb_streak_start_ms = now_ms;
    }
    c->have_hb = true;
    c->last_hb_ms = now_ms;
    c->last_hb_seq = hb->seq;

    /* Belt and braces: the orchestrator normally stops beating while an e-stop
     * is latched, but a beat that still advertises the latch must latch us too.
     * The upstream cause is not carried in the flag, so it is attributed to the
     * operator/originator rather than guessed. */
    if (hb->flags & PS_HB_ESTOP_LATCHED) {
        ps_safety_core_on_estop(c, now_ms, PS_ESTOP_OPERATOR);
    }
}

void ps_safety_core_on_estop(ps_safety_core_t *c, uint32_t now_ms, uint8_t cause)
{
    if (c->state == PS_STATE_ESTOP && c->estop_cause != 0) {
        return;  /* keep the originating cause; later reports are echoes */
    }
    c->estop_cause = cause ? cause : PS_ESTOP_SOFTWARE;
    c->estop_since_ms = now_ms;
    c->want_operational = false;
    go(c, PS_STATE_ESTOP, c->estop_cause);
}

bool ps_safety_core_on_clear(ps_safety_core_t *c, uint32_t now_ms, uint32_t magic,
                             uint32_t counter)
{
    if (magic != PS_CLEAR_ESTOP_MAGIC) {
        return false;
    }
    /* Strictly monotonic: a replayed or corrupted frame must never un-latch. */
    if (counter <= c->clear_counter) {
        return false;
    }
    c->clear_counter = counter;

    if (c->state == PS_STATE_ESTOP) {
        /* Accepted, but the node stays latched until it has seen continuous
         * heartbeats for PS_ESTOP_REARM_MS (checked in tick). */
        c->estop_cause = 0;
        c->estop_since_ms = now_ms;
    }
    return true;
}

void ps_safety_core_on_cmd(ps_safety_core_t *c, uint32_t now_ms)
{
    c->last_cmd_ms = now_ms;
}

void ps_safety_core_on_mode_set(ps_safety_core_t *c, uint32_t now_ms, uint8_t target_state)
{
    switch (target_state) {
    case PS_STATE_OPERATIONAL:
        c->want_operational = true;
        break;
    case PS_STATE_STANDBY:
        c->want_operational = false;
        if (c->state == PS_STATE_OPERATIONAL || c->state == PS_STATE_PASSIVE) {
            go(c, PS_STATE_STANDBY, 0);
        }
        break;
    case PS_STATE_PASSIVE:
        c->want_operational = false;
        if (c->state == PS_STATE_OPERATIONAL || c->state == PS_STATE_STANDBY) {
            go(c, PS_STATE_PASSIVE, 0);
        }
        break;
    case PS_STATE_ESTOP:
        ps_safety_core_on_estop(c, now_ms, PS_ESTOP_OPERATOR);
        break;
    default:
        break;  /* BOOT/FAULT are not commandable */
    }
}

void ps_safety_core_on_local_fault(ps_safety_core_t *c, uint32_t now_ms, uint8_t estop_cause)
{
    ps_safety_core_on_estop(c, now_ms, estop_cause);
}

void ps_safety_core_on_bus_down(ps_safety_core_t *c, uint32_t now_ms)
{
    /* Bus-off is heartbeat loss that we already know about: drop immediately
     * instead of waiting out the watchdog window. The timestamp is unused —
     * invalidating have_hb is what makes every later check fail. */
    (void)now_ms;
    c->have_hb = false;
    if (c->state == PS_STATE_OPERATIONAL) {
        go(c, PS_STATE_PASSIVE, PS_ESTOP_COMM_LOSS);
    }
}

bool ps_safety_core_cmd_fresh(const ps_safety_core_t *c, uint32_t now_ms)
{
    return elapsed(now_ms, c->last_cmd_ms) < (int32_t)PS_CMD_STALE_MS;
}

uint8_t ps_safety_core_tick(ps_safety_core_t *c, uint32_t now_ms)
{
    switch (c->state) {
    case PS_STATE_BOOT:
        go(c, PS_STATE_STANDBY, 0);
        break;

    case PS_STATE_ESTOP:
        /* estop_cause == 0 means a valid CLEAR_ESTOP has been accepted. */
        if (c->estop_cause == 0 && heartbeat_ok(c, now_ms) &&
            streak_ms(c, now_ms) >= (int32_t)PS_ESTOP_REARM_MS) {
            go(c, PS_STATE_STANDBY, 0);
        }
        break;

    case PS_STATE_OPERATIONAL:
        if (!heartbeat_ok(c, now_ms)) {
            go(c, PS_STATE_PASSIVE, PS_ESTOP_COMM_LOSS);
        }
        break;

    case PS_STATE_STANDBY:
        if (c->want_operational && heartbeat_ok(c, now_ms)) {
            go(c, PS_STATE_OPERATIONAL, 0);
        }
        break;

    case PS_STATE_PASSIVE:
        /* Recovering from a comms loss is deliberately harder than starting up:
         * the link must have been continuously healthy for the re-arm window AND
         * a command must have arrived within that same healthy streak. */
        if (c->want_operational && heartbeat_ok(c, now_ms) &&
            streak_ms(c, now_ms) >= (int32_t)PS_REARM_WINDOW_MS &&
            elapsed(c->last_cmd_ms, c->hb_streak_start_ms) >= 0 &&
            ps_safety_core_cmd_fresh(c, now_ms)) {
            go(c, PS_STATE_OPERATIONAL, 0);
        }
        break;

    case PS_STATE_FAULT:
    default:
        break;  /* FAULT is terminal until reboot */
    }
    return c->state;
}
