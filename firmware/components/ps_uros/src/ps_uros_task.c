/* micro-ROS session lifecycle (ps_uros.h contract).
 *
 * Standard micro-ROS reconnection ladder: wait for the agent, build entities,
 * spin, and tear everything down the moment the agent stops answering pings.
 * Rebuilding from scratch is deliberate — a half-alive session is worse than no
 * session, and the suit's real-time planes (SAFETY/TELEM/CONTROL) never depend
 * on this one. */
#include "ps_uros_internal.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ps_uros";

#if PS_UROS_ENABLED

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#ifndef CONFIG_PS_UROS_TASK_STACK
#define CONFIG_PS_UROS_TASK_STACK 16384
#endif
#ifndef CONFIG_PS_UROS_RX_RING
#define CONFIG_PS_UROS_RX_RING 2048
#endif
#ifndef CONFIG_PS_UROS_PING_PERIOD_MS
#define CONFIG_PS_UROS_PING_PERIOD_MS 2000
#endif

/* Executor slots. Only subscriptions, timers, services, clients and guard
 * conditions consume one — publishers do not. An rclc parameter server alone
 * takes RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES (6), so the limb node's server
 * plus its timer already sits at 7. Overflowing this fails when entities are
 * added at runtime, not at build time, so leave generous headroom. */
#define PS_UROS_EXECUTOR_HANDLES 16

typedef enum {
    LINK_WAITING_AGENT,
    LINK_CONNECTED,
} link_state_t;

static ps_uros_config_t s_cfg;
static volatile bool s_connected;

static void set_connected(bool up)
{
    if (s_connected != up) {
        s_connected = up;
        ESP_LOGW(TAG, "agent %s", up ? "connected" : "lost");
        if (s_cfg.on_agent_state) {
            s_cfg.on_agent_state(up, s_cfg.arg);
        }
    }
}

static void session_task(void *arg)
{
    (void)arg;
    const uint32_t spin_ms = s_cfg.spin_period_ms ? s_cfg.spin_period_ms : 10;

    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    link_state_t state = LINK_WAITING_AGENT;
    uint32_t since_ping_ms = 0;

    while (true) {
        switch (state) {
        case LINK_WAITING_AGENT:
            if (rmw_uros_ping_agent(200, 1) == RMW_RET_OK) {
                if (rclc_support_init(&support, 0, NULL, &allocator) == RCL_RET_OK &&
                    rclc_node_init_default(&node, s_cfg.node_name,
                                           s_cfg.ros_namespace ? s_cfg.ros_namespace : "",
                                           &support) == RCL_RET_OK &&
                    rclc_executor_init(&executor, &support.context,
                                       PS_UROS_EXECUTOR_HANDLES,
                                       &allocator) == RCL_RET_OK) {
                    s_cfg.create_entities(&support, &node, &executor, s_cfg.arg);
                    set_connected(true);
                    since_ping_ms = 0;
                    state = LINK_CONNECTED;
                } else {
                    ESP_LOGE(TAG, "session init failed; retrying");
                    rclc_support_fini(&support);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            break;

        case LINK_CONNECTED:
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(spin_ms));
            vTaskDelay(pdMS_TO_TICKS(spin_ms));

            since_ping_ms += spin_ms;
            if (since_ping_ms >= CONFIG_PS_UROS_PING_PERIOD_MS) {
                since_ping_ms = 0;
                if (rmw_uros_ping_agent(100, 2) != RMW_RET_OK) {
                    set_connected(false);
                    if (s_cfg.destroy_entities) {
                        s_cfg.destroy_entities(s_cfg.arg);
                    }
                    rclc_executor_fini(&executor);
                    rcl_node_fini(&node);
                    rclc_support_fini(&support);
                    ps_uros_transport_log_stats();
                    state = LINK_WAITING_AGENT;
                }
            }
            break;
        }
    }
}

esp_err_t ps_uros_start(const ps_uros_config_t *cfg)
{
    if (cfg == NULL || cfg->can == NULL || cfg->create_entities == NULL ||
        cfg->node_name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;

    esp_err_t err = ps_uros_transport_init(cfg->can, cfg->node_id, CONFIG_PS_UROS_RX_RING);
    if (err != ESP_OK) {
        return err;
    }

    /* framing = true: the client emits an HDLC stream, which is exactly what the
     * agent's serial/multiserial transport expects on the far side of the PTY. */
    if (rmw_uros_set_custom_transport(true, NULL, ps_uros_open, ps_uros_close,
                                      ps_uros_write, ps_uros_read) != RMW_RET_OK) {
        ESP_LOGE(TAG, "rmw_uros_set_custom_transport failed");
        return ESP_FAIL;
    }

    UBaseType_t prio = cfg->task_priority ? cfg->task_priority : (tskIDLE_PRIORITY + 5);
    if (xTaskCreatePinnedToCore(session_task, "ps_uros", CONFIG_PS_UROS_TASK_STACK, NULL, prio,
                                NULL, cfg->core) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "session task started for %s (core %d)", cfg->node_name, (int)cfg->core);
    return ESP_OK;
}

bool ps_uros_connected(void)
{
    return s_connected;
}

#else /* !PS_UROS_ENABLED */

/* The micro-ROS client is not vendored in this build. The XRCE plane is simply
 * absent: every safety-, control-, telemetry- and audio-carrying plane is
 * independent of it by design (docs/network-map.md §2 rate policy), so the node
 * is fully functional minus low-rate ROS topics, parameters and services. */
esp_err_t ps_uros_start(const ps_uros_config_t *cfg)
{
    (void)cfg;
    ESP_LOGW(TAG, "micro-ROS client not vendored: XRCE plane disabled "
                  "(run firmware/tools/fetch_deps.sh and rebuild to enable)");
    return ESP_ERR_NOT_SUPPORTED;
}

bool ps_uros_connected(void)
{
    return false;
}

#endif /* PS_UROS_ENABLED */
