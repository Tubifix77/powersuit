/* safety_glue — the ps_safety transition callback for node_limb.
 *
 * Runs from the ps_safety task context, inside the component's internal lock
 * (ps_safety_esp.c: "user code must not call back into ps_safety_* from
 * here"), so this file only ever touches limb_tasks state, never ps_safety
 * itself. Adds no rules of its own: docs/safety.md section 2 defines the
 * transitions, this just reacts to them. */
#include "limb_tasks.h"

#include "esp_log.h"

#include "powersuit_proto/wire.h"

static const char *TAG = "node_limb";

void limb_safety_on_transition(uint8_t old_state, uint8_t new_state, uint8_t cause, void *arg)
{
    (void)arg;
    switch (new_state) {
    case PS_STATE_PASSIVE:
    case PS_STATE_ESTOP:
    case PS_STATE_FAULT:
        /* Fail toward limpness (docs/safety.md section 1): every joint's
         * setpoint drops to PASSIVE + zero so control_task's actuation gate
         * free-wheels/damps it, regardless of what the orchestrator last
         * asked for. */
        limb_setpoint_zero_all();
        ESP_LOGW(TAG, "safety %u -> %u (cause %u): setpoints zeroed, joints forced PASSIVE",
                 old_state, new_state, cause);
        break;
    case PS_STATE_OPERATIONAL:
        /* Reset every PID/FOC integrator before this state is live so a
         * stale term accumulated during PASSIVE cannot kick the joint the
         * instant actuation is re-armed. */
        limb_control_reset_pids();
        ESP_LOGI(TAG, "safety %u -> OPERATIONAL: PIDs reset", old_state);
        break;
    default:
        break;
    }
}

void limb_safety_glue_init(void)
{
    /* Boot-time defaults: every joint starts PASSIVE with a zeroed setpoint,
     * matching what a PASSIVE/ESTOP/FAULT transition would force later. This
     * must run before ps_safety_start() so there is never a window where a
     * stale (zero-initialized but unvalidated) setpoint could be read as
     * live before the state machine exists. */
    limb_setpoint_zero_all();
    ESP_LOGI(TAG, "safety glue armed");
}
