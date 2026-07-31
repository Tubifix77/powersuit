/* Safety transitions on the helmet.
 *
 * Nothing here actuates — the helmet has no motors. Its job during a fault is
 * to tell the wearer, immediately and unambiguously, which is why the banner is
 * driven straight off the state machine and not from anything the cloud says. */
#include "helmet_tasks.h"

#include "esp_log.h"

#include "powersuit_proto/wire.h"

static const char *TAG = "node_helmet";

void helmet_safety_on_transition(uint8_t old_state, uint8_t new_state, uint8_t cause, void *arg)
{
    (void)arg;
    (void)old_state;

    helmet_hud_set_state(new_state);

    switch (new_state) {
    case PS_STATE_ESTOP:
        helmet_hud_push_warning("E-STOP LATCHED");
        break;
    case PS_STATE_PASSIVE:
        helmet_hud_push_warning(cause == PS_ESTOP_COMM_LOSS ? "LINK LOST - LIMP"
                                                            : "PASSIVE MODE");
        break;
    case PS_STATE_FAULT:
        helmet_hud_push_warning("NODE FAULT");
        break;
    case PS_STATE_OPERATIONAL:
        /* Recovery clears the transient warnings so the list reflects now. */
        helmet_hud_clear_warnings();
        break;
    default:
        break;
    }
    ESP_LOGW(TAG, "safety state -> %u (cause %u)", new_state, cause);
}
