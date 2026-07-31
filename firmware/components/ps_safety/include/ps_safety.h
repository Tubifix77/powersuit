/* ps_safety — the node safety state machine (docs/safety.md §2).
 * FROZEN API. Two layers:
 *   1. ps_safety_core_*  — pure C99 state machine, no IDF deps, host-tested.
 *   2. ps_safety_*       — ESP glue: wires the core to ps_can + esp_timer.
 * The core is the single source of truth; the glue never adds transitions. */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "powersuit_proto/wire.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ps_safety_transition_cb_t)(uint8_t old_state, uint8_t new_state,
                                          uint8_t cause, void *arg);

/* ---------------- pure core (host-testable) ---------------- */

typedef struct {
    uint8_t  state;             /* PS_STATE_* (wire.h) */
    uint8_t  estop_cause;
    uint16_t last_hb_seq;
    uint32_t last_hb_ms;        /* time of last valid heartbeat */
    uint32_t hb_streak_start_ms;/* start of current uninterrupted heartbeat streak */
    uint32_t last_cmd_ms;       /* time of last valid actuation command */
    uint32_t clear_counter;     /* highest CLEAR_ESTOP counter accepted */
    uint32_t estop_since_ms;
    bool     have_hb;
    bool     want_operational;  /* latched by MODE_SET(OPERATIONAL) */
    ps_safety_transition_cb_t on_transition;
    void    *cb_arg;
} ps_safety_core_t;

void ps_safety_core_init(ps_safety_core_t *c, uint32_t now_ms,
                         ps_safety_transition_cb_t cb, void *arg);

/* Feed events. All take the current monotonic time in ms. */
void ps_safety_core_on_heartbeat(ps_safety_core_t *c, uint32_t now_ms, const ps_heartbeat_t *hb);
void ps_safety_core_on_estop(ps_safety_core_t *c, uint32_t now_ms, uint8_t cause);
/* Returns true when the clear was accepted (magic + monotonic counter valid). */
bool ps_safety_core_on_clear(ps_safety_core_t *c, uint32_t now_ms, uint32_t magic, uint32_t counter);
void ps_safety_core_on_cmd(ps_safety_core_t *c, uint32_t now_ms);
void ps_safety_core_on_mode_set(ps_safety_core_t *c, uint32_t now_ms, uint8_t target_state);
void ps_safety_core_on_local_fault(ps_safety_core_t *c, uint32_t now_ms, uint8_t estop_cause);
/* Bus-off/err-passive from ps_can is fed here; treated as heartbeat loss. */
void ps_safety_core_on_bus_down(ps_safety_core_t *c, uint32_t now_ms);

/* Drive timeouts; call at >= 100 Hz. Returns the (possibly new) state. */
uint8_t ps_safety_core_tick(ps_safety_core_t *c, uint32_t now_ms);

bool ps_safety_core_cmd_fresh(const ps_safety_core_t *c, uint32_t now_ms); /* < PS_CMD_STALE_MS */

/* ---------------- ESP glue ---------------- */

struct ps_can_ctx;

typedef struct {
    struct ps_can_ctx *can;         /* ps_can_handle_t */
    uint8_t node_id;
    ps_safety_transition_cb_t on_transition;  /* called from safety task context */
    void *arg;
} ps_safety_config_t;

esp_err_t ps_safety_start(const ps_safety_config_t *cfg);

uint8_t ps_safety_state(void);
/* THE actuation gate: every torque/servo write checks this, nothing else does. */
bool ps_safety_can_actuate(void);
bool ps_safety_cmd_fresh(void);

/* Broadcast ESTOP (repeated PS_ESTOP_REPEAT times) and latch locally. */
esp_err_t ps_safety_raise_estop(uint8_t cause);
esp_err_t ps_safety_send_fault(uint8_t fault_code, uint8_t severity, uint16_t detail);
/* Feed a valid received actuation command (JOINT_CMD/FLAP_CMD) into freshness tracking. */
void ps_safety_note_cmd(void);

#ifdef __cplusplus
}
#endif
