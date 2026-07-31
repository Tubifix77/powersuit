/* ps_router — the hub's routing decision table (docs/network-map.md §5).
 * FROZEN API. Pure C99, no IDF deps, host-tested. The gateway task feeds every
 * frame through ps_router_route and forwards to the returned port set; policy
 * lives HERE, never in the pump code. */
#pragma once

#include <stdint.h>

#include "powersuit_proto/can_id.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    PS_PORT_CAN1  = 0,
    PS_PORT_CAN2  = 1,
    PS_PORT_SPI   = 2,
    PS_PORT_LOCAL = 3,   /* consumed by the hub itself */
};

#define PS_PORT_BIT(p) (1u << (p))

/* Which CAN bus a node lives on. Returns PS_PORT_CAN1, PS_PORT_CAN2, or -1
 * (node 7/8/unknown are not on a bus). */
int ps_router_bus_of_node(uint8_t node);

/* Route one frame: origin_port = where it came from (PS_PORT_*), id = 29-bit CAN
 * identifier. Returns a bitmask of destination ports, excluding the origin.
 * Encodes: SAFETY cut-through everywhere; TELEM/XRCE/AUDIO-up only toward SPI;
 * AUDIO-down only toward node 6's bus; CONTROL/XRCE from SPI by dst bus;
 * dst==7 or HUB_LOCAL semantics -> PS_PORT_LOCAL; MGMT broadcast both + SPI. */
uint8_t ps_router_route(uint8_t origin_port, uint32_t id);

#ifdef __cplusplus
}
#endif
