/* node_flight — cross-task wiring for the Node 5 app. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ps_can.h"
#include "board_flight.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Commanded target for one flap (written by comms, consumed by servo_task). */
typedef struct {
    int16_t pos_pm;       /* permille, already clamped to +-1000 at ingest */
    uint8_t rate_lim;     /* %/s from FLAP_CMD, 0 = board default */
    bool    brake;        /* FLAP_CMD flags.b0 */
} flight_target_t;

/* Actual per-flap output state (written by servo_task, read for FLAP_STATE). */
typedef struct {
    int16_t pos_pm;       /* slew-limited value currently on the horn */
    int16_t target_pm;    /* post-clamp/exclusion target being tracked */
    bool    at_limit;     /* envelope or pairwise clamp active */
} flight_flap_state_t;

void flight_targets_set(uint8_t flap, int16_t pos_pm, uint8_t rate_lim, bool brake);
void flight_targets_neutral_all(void);
void flight_targets_snapshot(flight_target_t out[PS_FLAP_COUNT]);

void flight_flap_state_publish(const flight_flap_state_t *st, uint8_t flap);
void flight_flap_state_snapshot(flight_flap_state_t out[PS_FLAP_COUNT]);

/* Runtime parameters (XRCE set_parameters service; defaults from board). */
float flight_param_rho(void);
float flight_param_rate_scale(void);   /* 0.05..1.0 multiplier on slew limits */
bool  flight_param_set(const char *name, double value); /* false = rejected */

/* TELEM/MGMT emit helper: per-type sequence in the id low byte (network-map §2). */
void flight_telem_send(uint8_t cls, uint8_t type, const void *payload, uint8_t len);

void servo_task_start(void);                    /* core 1, prio 18, 100 Hz */
void aero_task_start(void);                     /* core 1, prio 15, 100 Hz */
void comms_task_start(ps_can_handle_t can);     /* core 0, prio 12 */
void flight_safety_glue_init(void);

ps_can_handle_t flight_can(void);
void flight_set_can(ps_can_handle_t can);

/* micro-ROS entity callbacks handed to ps_uros_start (defined in comms_task.c). */
void flight_uros_create_entities(void *support, void *node, void *executor, void *arg);
void flight_uros_destroy_entities(void *arg);

#ifdef __cplusplus
}
#endif
