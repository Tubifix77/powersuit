/* Exhaustive table test of the hub routing policy (docs/network-map.md §5)
 * against ps_router. Every row of the §5 policy table has explicit cases here;
 * two property sweeps lock the global invariants (origin bit never echoed,
 * SAFETY cut-through reaches every other port). */
#include "powersuit_proto/can_id.h"
#include "ps_router.h"
#include "ps_test.h"

#define B_CAN1  PS_PORT_BIT(PS_PORT_CAN1)
#define B_CAN2  PS_PORT_BIT(PS_PORT_CAN2)
#define B_SPI   PS_PORT_BIT(PS_PORT_SPI)
#define B_LOCAL PS_PORT_BIT(PS_PORT_LOCAL)
#define B_ALL   (B_CAN1 | B_CAN2 | B_SPI | B_LOCAL)

typedef struct {
    uint8_t origin;
    uint8_t cls, src, dst, type;
    uint8_t expect;
} route_case_t;

static void run_cases(const route_case_t *cases, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        const route_case_t *c = &cases[i];
        uint32_t id = ps_can_id_pack(c->cls, c->src, c->dst, c->type, 0x5A);
        uint8_t got = ps_router_route(c->origin, id);
        if (got != c->expect) {
            PS_FAIL("case %zu: origin=%u cls=%u src=%u dst=%u type=0x%02X -> "
                    "got 0x%02X, expected 0x%02X",
                    i, c->origin, c->cls, c->src, c->dst, c->type, got, c->expect);
        }
    }
}

PS_TEST(bus_of_node_registry)
{
    for (int n = 0; n < 32; n++) {
        int want;
        switch (n) {
        case PS_NODE_ARM_R:
        case PS_NODE_ARM_L:
        case PS_NODE_HELMET:
            want = PS_PORT_CAN1;
            break;
        case PS_NODE_LEG_R:
        case PS_NODE_LEG_L:
        case PS_NODE_FLIGHT:
            want = PS_PORT_CAN2;
            break;
        default:
            want = -1; /* broadcast, hub, orchestrator, unassigned */
            break;
        }
        PS_ASSERT_EQ_INT(ps_router_bus_of_node((uint8_t)n), want);
    }
}

PS_TEST(safety_cut_through_every_origin)
{
    /* §5 rows 1-2: SAFETY forwards to every other port from every origin,
     * regardless of type, src, or dst (including dst==HUB, dst==broadcast). */
    static const uint8_t types[] = { PS_T_HEARTBEAT, PS_T_ESTOP,
                                     PS_T_CLEAR_ESTOP, PS_T_NODE_FAULT };
    static const uint8_t dsts[]  = { 0, 3, PS_NODE_HUB, PS_NODE_ORCH };
    static const uint8_t srcs[]  = { 1, 5, 7, 8 };
    for (uint8_t origin = PS_PORT_CAN1; origin <= PS_PORT_LOCAL; origin++) {
        uint8_t want = (uint8_t)(B_ALL & ~PS_PORT_BIT(origin));
        for (size_t t = 0; t < sizeof(types); t++) {
            for (size_t d = 0; d < sizeof(dsts); d++) {
                for (size_t s = 0; s < sizeof(srcs); s++) {
                    uint32_t id = ps_can_id_pack(PS_CLS_SAFETY, srcs[s], dsts[d],
                                                 types[t], 0x11);
                    PS_ASSERT_EQ_INT(ps_router_route(origin, id), want);
                }
            }
        }
    }
    /* Spelled out: the three transport origins each reach the other three ports. */
    uint32_t hb = ps_can_id_pack(PS_CLS_SAFETY, PS_NODE_ORCH, 0, PS_T_HEARTBEAT, 1);
    PS_ASSERT_EQ_INT(ps_router_route(PS_PORT_CAN1, hb), B_CAN2 | B_SPI | B_LOCAL);
    PS_ASSERT_EQ_INT(ps_router_route(PS_PORT_CAN2, hb), B_CAN1 | B_SPI | B_LOCAL);
    PS_ASSERT_EQ_INT(ps_router_route(PS_PORT_SPI, hb),  B_CAN1 | B_CAN2 | B_LOCAL);
}

PS_TEST(telem_from_can_goes_up_only)
{
    static const route_case_t cases[] = {
        /* normal uplink telemetry */
        { PS_PORT_CAN1, PS_CLS_TELEM, 1, PS_NODE_ORCH, PS_T_JOINT_STATE, B_SPI },
        { PS_PORT_CAN1, PS_CLS_TELEM, 6, PS_NODE_ORCH, PS_T_ENV,         B_SPI },
        { PS_PORT_CAN2, PS_CLS_TELEM, 5, PS_NODE_ORCH, PS_T_AERO_STATE,  B_SPI },
        { PS_PORT_CAN2, PS_CLS_TELEM, 3, PS_NODE_ORCH, PS_T_IMU_QUAT,    B_SPI },
        /* cross-bus attempts: never forwarded to the other bus (§5 row 4) */
        { PS_PORT_CAN1, PS_CLS_TELEM, 1, 3, PS_T_JOINT_STATE, B_SPI },
        { PS_PORT_CAN2, PS_CLS_TELEM, 5, 6, PS_T_AERO_STATE,  B_SPI },
        { PS_PORT_CAN1, PS_CLS_TELEM, 2, 6, PS_T_FORCE,       B_SPI }, /* same bus too */
        /* broadcast telem still goes up only */
        { PS_PORT_CAN1, PS_CLS_TELEM, 1, 0, PS_T_NODE_STATS, B_SPI },
        /* addressed to the hub: up + local observe */
        { PS_PORT_CAN1, PS_CLS_TELEM, 6, PS_NODE_HUB, PS_T_ENV, B_SPI | B_LOCAL },
        { PS_PORT_CAN2, PS_CLS_TELEM, 5, PS_NODE_HUB, PS_T_ENV, B_SPI | B_LOCAL },
    };
    run_cases(cases, sizeof(cases) / sizeof(cases[0]));
}

PS_TEST(telem_from_spi_by_dst)
{
    static const route_case_t cases[] = {
        { PS_PORT_SPI, PS_CLS_TELEM, 8, 1, PS_T_JOINT_STATE, B_CAN1 },
        { PS_PORT_SPI, PS_CLS_TELEM, 8, 2, PS_T_JOINT_STATE, B_CAN1 },
        { PS_PORT_SPI, PS_CLS_TELEM, 8, 6, PS_T_ENV,         B_CAN1 },
        { PS_PORT_SPI, PS_CLS_TELEM, 8, 3, PS_T_JOINT_STATE, B_CAN2 },
        { PS_PORT_SPI, PS_CLS_TELEM, 8, 4, PS_T_JOINT_STATE, B_CAN2 },
        { PS_PORT_SPI, PS_CLS_TELEM, 8, 5, PS_T_AERO_STATE,  B_CAN2 },
        /* dst==HUB: local only */
        { PS_PORT_SPI, PS_CLS_TELEM, 8, PS_NODE_HUB, PS_T_BMS_SUMMARY, B_LOCAL },
        /* not a bus node and not the hub: dropped */
        { PS_PORT_SPI, PS_CLS_TELEM, 8, PS_NODE_ORCH, PS_T_JOINT_STATE, 0 },
        { PS_PORT_SPI, PS_CLS_TELEM, 8, 0,  PS_T_JOINT_STATE, 0 },
        { PS_PORT_SPI, PS_CLS_TELEM, 8, 30, PS_T_JOINT_STATE, 0 },
    };
    run_cases(cases, sizeof(cases) / sizeof(cases[0]));
}

PS_TEST(xrce_up_and_down)
{
    static const route_case_t cases[] = {
        /* up: SPI only, never cross-bus (§5 row 4) */
        { PS_PORT_CAN1, PS_CLS_XRCE, 1, PS_NODE_ORCH, PS_T_XRCE_STREAM, B_SPI },
        { PS_PORT_CAN1, PS_CLS_XRCE, 6, PS_NODE_ORCH, PS_T_XRCE_STREAM, B_SPI },
        { PS_PORT_CAN2, PS_CLS_XRCE, 4, PS_NODE_ORCH, PS_T_XRCE_STREAM, B_SPI },
        { PS_PORT_CAN2, PS_CLS_XRCE, 5, 1,            PS_T_XRCE_STREAM, B_SPI },
        /* down: dst's bus (§5 row 5) */
        { PS_PORT_SPI, PS_CLS_XRCE, 8, 1, PS_T_XRCE_STREAM, B_CAN1 },
        { PS_PORT_SPI, PS_CLS_XRCE, 8, 2, PS_T_XRCE_STREAM, B_CAN1 },
        { PS_PORT_SPI, PS_CLS_XRCE, 8, 6, PS_T_XRCE_STREAM, B_CAN1 },
        { PS_PORT_SPI, PS_CLS_XRCE, 8, 3, PS_T_XRCE_STREAM, B_CAN2 },
        { PS_PORT_SPI, PS_CLS_XRCE, 8, 5, PS_T_XRCE_STREAM, B_CAN2 },
        /* hub runs no micro-ROS client (§12.4): local delivery, gateway drops */
        { PS_PORT_SPI, PS_CLS_XRCE, 8, PS_NODE_HUB, PS_T_XRCE_STREAM, B_LOCAL },
        /* nonsense destinations from SPI: dropped */
        { PS_PORT_SPI, PS_CLS_XRCE, 8, PS_NODE_ORCH, PS_T_XRCE_STREAM, 0 },
        { PS_PORT_SPI, PS_CLS_XRCE, 8, 0,            PS_T_XRCE_STREAM, 0 },
    };
    run_cases(cases, sizeof(cases) / sizeof(cases[0]));
}

PS_TEST(control_from_spi_by_dst)
{
    static const route_case_t cases[] = {
        { PS_PORT_SPI, PS_CLS_CONTROL, 8, 1, PS_T_JOINT_CMD, B_CAN1 },
        { PS_PORT_SPI, PS_CLS_CONTROL, 8, 2, PS_T_JOINT_CMD, B_CAN1 },
        { PS_PORT_SPI, PS_CLS_CONTROL, 8, 6, PS_T_LED_PATTERN, B_CAN1 },
        { PS_PORT_SPI, PS_CLS_CONTROL, 8, 3, PS_T_JOINT_CMD, B_CAN2 },
        { PS_PORT_SPI, PS_CLS_CONTROL, 8, 4, PS_T_MODE_SET,  B_CAN2 },
        { PS_PORT_SPI, PS_CLS_CONTROL, 8, 5, PS_T_FLAP_CMD,  B_CAN2 },
        /* §5: dst=7 -> consume */
        { PS_PORT_SPI, PS_CLS_CONTROL, 8, PS_NODE_HUB, PS_T_LED_PATTERN, B_LOCAL },
        { PS_PORT_SPI, PS_CLS_CONTROL, 8, PS_NODE_HUB, PS_T_MODE_SET,    B_LOCAL },
        /* no bus to carry it: dropped (orchestrator unicasts per node) */
        { PS_PORT_SPI, PS_CLS_CONTROL, 8, 0,            PS_T_MODE_SET,  0 },
        { PS_PORT_SPI, PS_CLS_CONTROL, 8, PS_NODE_ORCH, PS_T_JOINT_CMD, 0 },
        { PS_PORT_SPI, PS_CLS_CONTROL, 8, 30,           PS_T_JOINT_CMD, 0 },
    };
    run_cases(cases, sizeof(cases) / sizeof(cases[0]));
}

PS_TEST(control_from_can_is_anomalous_except_to_orch)
{
    /* §5: CONTROL originates from SPI only. From CAN, only dst==ORCH is
     * forwarded (up); everything else — including dst==HUB and node-to-node
     * attempts — returns 0 and is counted as an anomaly by the caller. */
    static const route_case_t cases[] = {
        { PS_PORT_CAN1, PS_CLS_CONTROL, 6, PS_NODE_ORCH, PS_T_LED_PATTERN, B_SPI },
        { PS_PORT_CAN2, PS_CLS_CONTROL, 5, PS_NODE_ORCH, PS_T_MODE_SET,    B_SPI },
        { PS_PORT_CAN1, PS_CLS_CONTROL, 1, 3,           PS_T_JOINT_CMD, 0 },
        { PS_PORT_CAN1, PS_CLS_CONTROL, 1, 2,           PS_T_JOINT_CMD, 0 },
        { PS_PORT_CAN2, PS_CLS_CONTROL, 3, 4,           PS_T_JOINT_CMD, 0 },
        { PS_PORT_CAN1, PS_CLS_CONTROL, 6, PS_NODE_HUB, PS_T_LED_PATTERN, 0 },
        { PS_PORT_CAN2, PS_CLS_CONTROL, 5, PS_NODE_HUB, PS_T_MODE_SET,    0 },
        { PS_PORT_CAN1, PS_CLS_CONTROL, 1, 0,           PS_T_MODE_SET,   0 },
        { PS_PORT_CAN2, PS_CLS_CONTROL, 4, 0,           PS_T_MODE_SET,   0 },
    };
    run_cases(cases, sizeof(cases) / sizeof(cases[0]));
}

PS_TEST(audio_up_from_can1_only)
{
    static const route_case_t cases[] = {
        /* helmet uplink (dst==ORCH): SPI only */
        { PS_PORT_CAN1, PS_CLS_AUDIO, 6, PS_NODE_ORCH, PS_T_AUDIO_UP,   B_SPI },
        { PS_PORT_CAN1, PS_CLS_AUDIO, 6, PS_NODE_ORCH, PS_T_AUDIO_SYNC, B_SPI },
        { PS_PORT_CAN1, PS_CLS_AUDIO, 6, PS_NODE_ORCH, PS_T_AUDIO_CTL,  B_SPI },
        /* audio has no business on CAN 2 in either direction (§5 "never") */
        { PS_PORT_CAN2, PS_CLS_AUDIO, 5, PS_NODE_ORCH,   PS_T_AUDIO_UP,   0 },
        { PS_PORT_CAN2, PS_CLS_AUDIO, 3, PS_NODE_HELMET, PS_T_AUDIO_DOWN, 0 },
        { PS_PORT_CAN2, PS_CLS_AUDIO, 5, PS_NODE_ORCH,   PS_T_AUDIO_SYNC, 0 },
        /* down-direction ids arriving from CAN are anomalies */
        { PS_PORT_CAN1, PS_CLS_AUDIO, 6, PS_NODE_HELMET, PS_T_AUDIO_DOWN, 0 },
        { PS_PORT_CAN1, PS_CLS_AUDIO, 6, 1,              PS_T_AUDIO_UP,   0 },
    };
    run_cases(cases, sizeof(cases) / sizeof(cases[0]));
}

PS_TEST(audio_down_from_spi_to_helmet_bus_only)
{
    static const route_case_t cases[] = {
        { PS_PORT_SPI, PS_CLS_AUDIO, 8, PS_NODE_HELMET, PS_T_AUDIO_DOWN, B_CAN1 },
        { PS_PORT_SPI, PS_CLS_AUDIO, 8, PS_NODE_HELMET, PS_T_AUDIO_SYNC, B_CAN1 },
        { PS_PORT_SPI, PS_CLS_AUDIO, 8, PS_NODE_HELMET, PS_T_AUDIO_CTL,  B_CAN1 },
        /* wrong destinations from SPI: dropped, never CAN 2 */
        { PS_PORT_SPI, PS_CLS_AUDIO, 8, 5,            PS_T_AUDIO_DOWN, 0 },
        { PS_PORT_SPI, PS_CLS_AUDIO, 8, 3,            PS_T_AUDIO_DOWN, 0 },
        { PS_PORT_SPI, PS_CLS_AUDIO, 8, PS_NODE_ORCH, PS_T_AUDIO_UP,   0 },
        { PS_PORT_SPI, PS_CLS_AUDIO, 8, 0,            PS_T_AUDIO_DOWN, 0 },
        { PS_PORT_SPI, PS_CLS_AUDIO, 8, PS_NODE_HUB,  PS_T_AUDIO_DOWN, 0 },
    };
    run_cases(cases, sizeof(cases) / sizeof(cases[0]));

    /* Property: no AUDIO id, from any origin, ever routes to CAN 2. */
    static const uint8_t types[] = { PS_T_AUDIO_DOWN, PS_T_AUDIO_UP,
                                     PS_T_AUDIO_SYNC, PS_T_AUDIO_CTL };
    for (uint8_t origin = PS_PORT_CAN1; origin <= PS_PORT_LOCAL; origin++) {
        for (size_t t = 0; t < sizeof(types); t++) {
            for (uint8_t dst = 0; dst < 10; dst++) {
                uint32_t id = ps_can_id_pack(PS_CLS_AUDIO, 6, dst, types[t], 3);
                PS_ASSERT_EQ_INT(ps_router_route(origin, id) & B_CAN2, 0);
            }
        }
    }
}

PS_TEST(mgmt_broadcast_both_plus_local)
{
    static const route_case_t cases[] = {
        /* from SPI: both buses + hub consume, unicast or broadcast alike */
        { PS_PORT_SPI, PS_CLS_MGMT, 8, 0, PS_T_TIME_SYNC, B_CAN1 | B_CAN2 | B_LOCAL },
        { PS_PORT_SPI, PS_CLS_MGMT, 8, 3, PS_T_STATS_REQ, B_CAN1 | B_CAN2 | B_LOCAL },
        { PS_PORT_SPI, PS_CLS_MGMT, 8, PS_NODE_HUB, PS_T_FLOW_CTL, B_CAN1 | B_CAN2 | B_LOCAL },
        /* from CAN unicast: up + hub observe, never the other bus */
        { PS_PORT_CAN1, PS_CLS_MGMT, 6, PS_NODE_ORCH, PS_T_LOG,     B_SPI | B_LOCAL },
        { PS_PORT_CAN2, PS_CLS_MGMT, 3, PS_NODE_ORCH, PS_T_VERSION, B_SPI | B_LOCAL },
        { PS_PORT_CAN1, PS_CLS_MGMT, 1, PS_NODE_HUB,  PS_T_STATS_REQ, B_SPI | B_LOCAL },
        /* from CAN broadcast: other bus joins in */
        { PS_PORT_CAN1, PS_CLS_MGMT, 6, 0, PS_T_FLOW_CTL, B_CAN2 | B_SPI | B_LOCAL },
        { PS_PORT_CAN2, PS_CLS_MGMT, 5, 0, PS_T_VERSION,  B_CAN1 | B_SPI | B_LOCAL },
    };
    run_cases(cases, sizeof(cases) / sizeof(cases[0]));
}

PS_TEST(property_origin_bit_never_set)
{
    /* Full sweep: every class x representative type x src x dst x origin.
     * The returned mask must never include the origin port. */
    static const uint8_t types_by_cls[6][4] = {
        { PS_T_HEARTBEAT, PS_T_ESTOP, PS_T_CLEAR_ESTOP, PS_T_NODE_FAULT },
        { PS_T_JOINT_CMD, PS_T_FLAP_CMD, PS_T_MODE_SET, PS_T_LED_PATTERN },
        { PS_T_JOINT_STATE, PS_T_BMS_SUMMARY, PS_T_AERO_STATE, PS_T_NODE_STATS },
        { PS_T_XRCE_STREAM, PS_T_XRCE_STREAM, PS_T_XRCE_STREAM, PS_T_XRCE_STREAM },
        { PS_T_AUDIO_DOWN, PS_T_AUDIO_UP, PS_T_AUDIO_SYNC, PS_T_AUDIO_CTL },
        { PS_T_TIME_SYNC, PS_T_FLOW_CTL, PS_T_LOG, PS_T_VERSION },
    };
    static const uint8_t srcs[] = { 1, 3, 5, 6, 7, 8 };
    for (uint8_t origin = PS_PORT_CAN1; origin <= PS_PORT_LOCAL; origin++) {
        for (uint8_t cls = 0; cls < 6; cls++) {
            for (size_t t = 0; t < 4; t++) {
                for (size_t s = 0; s < sizeof(srcs); s++) {
                    for (uint8_t dst = 0; dst < 10; dst++) {
                        uint32_t id = ps_can_id_pack(cls, srcs[s], dst,
                                                     types_by_cls[cls][t], 0x7F);
                        uint8_t mask = ps_router_route(origin, id);
                        if (mask & PS_PORT_BIT(origin)) {
                            PS_FAIL("origin echo: origin=%u cls=%u dst=%u mask=0x%02X",
                                    origin, cls, dst, mask);
                        }
                    }
                }
            }
        }
    }
}

PS_TEST(unassigned_class_and_bad_origin_drop)
{
    uint32_t id6 = ps_can_id_pack(6, 1, 8, 0x60, 0);
    uint32_t id7 = ps_can_id_pack(7, 1, 8, 0x70, 0);
    for (uint8_t origin = PS_PORT_CAN1; origin <= PS_PORT_LOCAL; origin++) {
        PS_ASSERT_EQ_INT(ps_router_route(origin, id6), 0);
        PS_ASSERT_EQ_INT(ps_router_route(origin, id7), 0);
    }
    uint32_t hb = ps_can_id_pack(PS_CLS_SAFETY, 8, 0, PS_T_HEARTBEAT, 1);
    PS_ASSERT_EQ_INT(ps_router_route(4, hb), 0);
    PS_ASSERT_EQ_INT(ps_router_route(255, hb), 0);
}

PS_TEST_MAIN()
