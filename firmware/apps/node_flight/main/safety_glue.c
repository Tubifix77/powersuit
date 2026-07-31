/* safety_glue — Node 5 transition callback (docs/safety.md §2/§5.4).
 *
 * servo_task.c already forces every flap to neutral whenever
 * ps_safety_can_actuate() is false, and that write is itself slew-limited
 * (the transition callback fires once per state change; the 100 Hz servo
 * loop is what actually walks the horn there). This callback exists only to
 * make the "go neutral" intent explicit at the instant of transition and to
 * log it -- it must NOT write PWM directly (servo_task.c's flap_write() is
 * the only path, per §5.1) and must NOT duplicate the clamp math. */
#include "esp_log.h"

#include "powersuit_proto/wire.h"
#include "node_flight.h"

static const char *TAG = "node_flight";

/* Non-static and exact-signature-matched to ps_safety_transition_cb_t
 * (ps_safety.h) so app_main.c can hand it to ps_safety_config_t.on_transition
 * via a local extern prototype, without node_flight.h needing to carry the
 * ps_safety.h type (kept out of the frozen header on purpose). */
void flight_safety_on_transition(uint8_t old_state, uint8_t new_state, uint8_t cause, void *arg)
{
    (void)arg;
    switch (new_state) {
    case PS_STATE_PASSIVE:
    case PS_STATE_ESTOP:
    case PS_STATE_FAULT:
        /* Target buffer to neutral (pos_pm = 0); servo_task's can_act gate
         * additionally forces neutral every tick regardless, so this is
         * belt-and-braces for the transition instant itself. */
        flight_targets_neutral_all();
        break;
    default:
        break;
    }
    ESP_LOGI(TAG, "safety state %u -> %u (cause %u)", old_state, new_state, cause);
}

void flight_safety_glue_init(void)
{
    /* No state to allocate: flight_safety_on_transition is stateless and is
     * wired into ps_safety_config_t.on_transition by app_main.c before
     * ps_safety_start() runs. This function is the named hook node_flight.h
     * exposes for that wiring step, kept separate from the callback itself
     * so app_main's boot sequence reads as one call per subsystem. */
}
