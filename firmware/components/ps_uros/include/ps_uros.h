/* ps_uros — micro-ROS session over the XRCE CAN plane. FROZEN API.
 * Transport: rmw_uros_set_custom_transport(framing = true) writing/reading
 * PS_T_XRCE_STREAM frames (docs/network-map.md §3.4, §7). The HDLC stream the
 * client emits is chunked into 1..8-byte CAN payloads toward dst=PS_NODE_ORCH;
 * inbound frames are drained into the transport read buffer.
 *
 * Task model: one task (core 0) owns session lifecycle: create -> spin executor
 * with spin_period_ms -> ping agent on silence -> destroy/recreate on loss.
 * Entities are (re)created via the callback after every (re)connect. */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "ps_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward decls so app code includes rclc headers, not this header. */
typedef struct ps_uros_ctx ps_uros_ctx_t;

/* Called in the ps_uros task after a session is established. The app registers its
 * publishers/subscriptions/timers on the provided rclc objects (passed as void* to
 * keep rclc out of this header; cast in the app):
 *   support:  rclc_support_t*, node: rcl_node_t*, executor: rclc_executor_t*  */
typedef void (*ps_uros_entities_cb_t)(void *support, void *node, void *executor, void *arg);
typedef void (*ps_uros_agent_state_cb_t)(bool connected, void *arg);

typedef struct {
    ps_can_handle_t can;
    uint8_t     node_id;          /* CAN source node */
    const char *node_name;        /* ROS node name, e.g. "node_arm_right" */
    const char *ros_namespace;    /* "" for root */
    ps_uros_entities_cb_t create_entities;   /* required */
    void (*destroy_entities)(void *arg);     /* optional, before teardown */
    ps_uros_agent_state_cb_t on_agent_state; /* optional */
    void *arg;
    uint32_t spin_period_ms;      /* 0 = default 10 */
    UBaseType_t task_priority;    /* 0 = default (tskIDLE_PRIORITY + 5) */
    BaseType_t  core;             /* contract: 0 (comms core) */
} ps_uros_config_t;

esp_err_t ps_uros_start(const ps_uros_config_t *cfg);
bool ps_uros_connected(void);

#ifdef __cplusplus
}
#endif
