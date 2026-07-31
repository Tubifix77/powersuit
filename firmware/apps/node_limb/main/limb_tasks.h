/* limb_tasks.h — app-private wiring between the node_limb tasks. Mirrors
 * hub_tasks.h: the frozen component APIs live in ps_can/ps_safety/ps_ctl/ps_uros,
 * this header only carries the shared state and start functions specific to
 * this app. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "ps_can.h"
#include "powersuit_proto/wire.h"
#include "board_limb.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Commanded target for one joint (writer: comms core 0; reader: control
 * core 1). Units match ps_joint_cmd_t (network-map.md section 3.2). */
typedef struct {
    uint8_t mode;        /* PS_JMODE_* */
    int16_t pos_crad;
    int16_t vel_crad_s;
    int16_t eff_cNm;
} limb_setpoint_t;

/* Actual per-joint output state (writer: control core 1; reader: comms core 0
 * for JOINT_STATE TELEM). Units match ps_joint_state_t. */
typedef struct {
    int16_t pos_crad;
    int16_t vel_crad_s;
    int16_t eff_cNm;
    bool    saturated;   /* torque-limit clamp active this tick */
    bool    passive;     /* joint free-wheeling/damping this tick */
} limb_joint_state_t;

/* 250 Hz sensor snapshot (control core 1), consumed by comms at 100 Hz.
 * Already wire-packed so comms_task.c does no unit math, only framing. */
typedef struct {
    ps_imu_quat_t quat;
    ps_imu_acc_t  acc;
    ps_imu_gyr_t  gyr;
    ps_force_t    force;
} limb_sensor_snapshot_t;

typedef struct {
    uint32_t control_ticks;      /* 1 kHz control loop iterations */
    uint32_t overcurrent_trips;  /* estop-worthy overcurrent events raised */
    uint32_t joint_cmd_rx;       /* valid JOINT_CMD frames applied */
    uint32_t mode_set_rx;        /* MODE_SET frames observed */
    uint32_t cmd_rejected;       /* JOINT_CMD dropped: bad joint/mode index */
} limb_counters_t;

/* ---- control_task.c: setpoints in, joint state + sensor snapshot out ---- */
void limb_setpoint_set(uint8_t joint, const limb_setpoint_t *sp);
void limb_setpoint_zero_all(void);   /* safety_glue: force PASSIVE + zero on trip */
void limb_setpoint_snapshot(limb_setpoint_t out[PS_LIMB_JOINT_COUNT]);

void limb_joint_state_snapshot(limb_joint_state_t out[PS_LIMB_JOINT_COUNT]);
void limb_sensor_snapshot(limb_sensor_snapshot_t *out);

void limb_control_reset_pids(void);  /* safety_glue: before OPERATIONAL re-entry */

/* Live parameter server target (kp/ki per joint's position+velocity loop,
 * and the runtime torque ceiling); board_limb.h values are the boot defaults.
 * Board mechanical limits are always re-applied on top, never bypassed. */
bool limb_param_set(const char *name, double value);
bool limb_param_get(const char *name, double *out_value);

esp_err_t limb_control_start(void);   /* core 1, prio 20, 1 kHz gptimer-notified */

/* ---- comms_task.c: CAN plane in/out, counters, micro-ROS entities ---- */
esp_err_t limb_comms_start(ps_can_handle_t can, uint8_t node_id);  /* core 0, prio 12 */

void limb_telem_send(uint8_t cls, uint8_t type, const void *payload, uint8_t len);
void limb_counters_get(limb_counters_t *out);
void limb_counters_bump_control_tick(void);
void limb_counters_bump_overcurrent(void);

/* micro-ROS entity callback handed to ps_uros_start (defined in comms_task.c).
 * arg is the "/suit/nodes/<node_name>/alive" topic name string. */
void limb_uros_create_entities(void *support, void *node, void *executor, void *arg);

/* ---- safety_glue.c: the ps_safety transition callback ---- */
void limb_safety_glue_init(void);
void limb_safety_on_transition(uint8_t old_state, uint8_t new_state, uint8_t cause, void *arg);

#ifdef __cplusplus
}
#endif
