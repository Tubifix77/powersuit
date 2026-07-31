/* ps_router — hub routing decision table, docs/network-map.md §5.
 * Pure C99, no IDF dependencies: this file is compiled and exhaustively table-
 * tested on the host (firmware/tests/host/test_router.c). Policy lives here;
 * the gateway pump only forwards to whatever port set this function returns. */
#include "ps_router.h"

#define PS_ROUTER_ALL_PORTS                                            \
    (PS_PORT_BIT(PS_PORT_CAN1) | PS_PORT_BIT(PS_PORT_CAN2) |           \
     PS_PORT_BIT(PS_PORT_SPI) | PS_PORT_BIT(PS_PORT_LOCAL))

int ps_router_bus_of_node(uint8_t node)
{
    switch (node) {
    case PS_NODE_ARM_R:
    case PS_NODE_ARM_L:
    case PS_NODE_HELMET:
        return PS_PORT_CAN1;
    case PS_NODE_LEG_R:
    case PS_NODE_LEG_L:
    case PS_NODE_FLIGHT:
        return PS_PORT_CAN2;
    default:
        return -1; /* hub/orchestrator/broadcast/unknown: not on a bus */
    }
}

/* Port bit of dst's bus, or 0 when dst is not a bus node. */
static uint8_t bus_bit_of(uint8_t dst)
{
    int bus = ps_router_bus_of_node(dst);
    return (bus < 0) ? 0u : (uint8_t)PS_PORT_BIT((unsigned)bus);
}

uint8_t ps_router_route(uint8_t origin_port, uint32_t id)
{
    if (origin_port > PS_PORT_LOCAL) {
        return 0;
    }

    ps_can_id_t f;
    ps_can_id_unpack(id, &f);

    const int from_spi = (origin_port == PS_PORT_SPI);
    uint8_t mask = 0;

    switch (f.cls) {
    case PS_CLS_SAFETY:
        /* §5: cut-through everywhere; the hub always observes (estop latch,
         * heartbeat mirror). Origin bit is stripped at the end. */
        mask = PS_ROUTER_ALL_PORTS;
        break;

    case PS_CLS_CONTROL:
        if (from_spi) {
            if (f.dst == PS_NODE_HUB) {
                mask = PS_PORT_BIT(PS_PORT_LOCAL); /* §5: dst=7 consume */
            } else {
                mask = bus_bit_of(f.dst); /* 0 for broadcast/orch/unknown */
            }
        } else {
            /* §5: CONTROL originates from SPI only. From a CAN bus the sole
             * legitimate flow is a request up to the orchestrator; anything
             * else (incl. dst==7) is an anomaly the caller counts and drops. */
            mask = (f.dst == PS_NODE_ORCH) ? PS_PORT_BIT(PS_PORT_SPI) : 0u;
        }
        break;

    case PS_CLS_TELEM:
        if (from_spi) {
            mask = bus_bit_of(f.dst); /* bus node: deliver; else drop (mask 0) */
            if (f.dst == PS_NODE_HUB) {
                mask = PS_PORT_BIT(PS_PORT_LOCAL);
            }
        } else {
            /* §5: never cross-bus; high-rate planes go up only. */
            mask = PS_PORT_BIT(PS_PORT_SPI);
            if (f.dst == PS_NODE_HUB) {
                mask |= PS_PORT_BIT(PS_PORT_LOCAL);
            }
        }
        break;

    case PS_CLS_XRCE:
        if (from_spi) {
            mask = bus_bit_of(f.dst); /* down: dst's bus */
            if (f.dst == PS_NODE_HUB) {
                mask = PS_PORT_BIT(PS_PORT_LOCAL); /* §12.4: hub has no client */
            }
        } else {
            mask = PS_PORT_BIT(PS_PORT_SPI); /* up: never cross-bus */
            if (f.dst == PS_NODE_HUB) {
                mask |= PS_PORT_BIT(PS_PORT_LOCAL);
            }
        }
        break;

    case PS_CLS_AUDIO:
        /* §5: audio lives on Bus 1 (helmet) and SPI only, never CAN 2. The id
         * dst field decides direction: up dst==8 (FRAME_UP/SYNC/CTL), down
         * dst==6. Audio arriving from CAN 2 or with any other dst is dropped. */
        if (origin_port == PS_PORT_CAN1 && f.dst == PS_NODE_ORCH) {
            mask = PS_PORT_BIT(PS_PORT_SPI);
        } else if (from_spi && f.dst == PS_NODE_HELMET) {
            mask = PS_PORT_BIT(PS_PORT_CAN1);
        } else {
            mask = 0;
        }
        break;

    case PS_CLS_MGMT:
        if (from_spi) {
            /* §5: broadcast both + hub consume/emit. */
            mask = PS_PORT_BIT(PS_PORT_CAN1) | PS_PORT_BIT(PS_PORT_CAN2) |
                   PS_PORT_BIT(PS_PORT_LOCAL);
        } else {
            mask = PS_PORT_BIT(PS_PORT_SPI) | PS_PORT_BIT(PS_PORT_LOCAL);
            if (f.dst == PS_NODE_BROADCAST) {
                /* Other bus only for broadcast; origin strip removes own bus. */
                mask |= PS_PORT_BIT(PS_PORT_CAN1) | PS_PORT_BIT(PS_PORT_CAN2);
            }
        }
        break;

    default:
        mask = 0; /* classes 6/7 are unassigned: drop */
        break;
    }

    /* Never echo a frame out of the port it came in on. */
    mask &= (uint8_t)~PS_PORT_BIT(origin_port);
    return mask;
}
