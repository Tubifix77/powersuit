/* Internal wiring between the node_helmet tasks. App-private.
 *
 * The HUD model is the one piece of shared mutable state: comms and safety_glue
 * write it, hud_task renders it. Everything else moves through ps_can. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "ps_can.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HELMET_MAX_WARNINGS   6
#define HELMET_WARNING_LEN    40

typedef struct {
    uint8_t safety_state;      /* PS_STATE_* */
    uint8_t battery_pct;
    bool    battery_valid;
    bool    cloud_up;
    float   ias_ms;
    char    warnings[HELMET_MAX_WARNINGS][HELMET_WARNING_LEN];
    uint8_t warning_count;
} hud_model_t;

/* Snapshot under the model mutex; render from the copy, never the live struct. */
void helmet_hud_snapshot(hud_model_t *out);
void helmet_hud_set_state(uint8_t safety_state);
void helmet_hud_set_battery(uint8_t pct);
void helmet_hud_set_cloud(bool up);
void helmet_hud_push_warning(const char *text);
void helmet_hud_clear_warnings(void);

/* ---- audio_in_task.c ---- */
esp_err_t helmet_audio_in_start(ps_can_handle_t can);
/* Non-zero once the VOX gate has opened since the last drain: comms turns this
 * into a /suit/voice/trigger publication. */
bool helmet_take_wake_event(void);

/* ---- audio_out_task.c ---- */
esp_err_t helmet_audio_out_start(ps_can_handle_t can);

/* ---- hud_task.c ---- */
esp_err_t helmet_hud_start(void);

/* ---- espnow_diag.c ---- */
esp_err_t helmet_espnow_start(void);

/* ---- comms_task.c ---- */
esp_err_t helmet_comms_start(ps_can_handle_t can);
void helmet_uros_create_entities(void *support, void *node, void *executor, void *arg);

/* ---- safety_glue.c ---- */
void helmet_safety_on_transition(uint8_t old_state, uint8_t new_state, uint8_t cause, void *arg);

#ifdef __cplusplus
}
#endif
