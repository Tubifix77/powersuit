#include "powersuit_proto/can_id.h"
#include "ps_test.h"

PS_TEST(known_example)
{
    uint32_t id = ps_can_id_pack(PS_CLS_TELEM, PS_NODE_ARM_R, PS_NODE_ORCH,
                                 PS_T_JOINT_STATE, 5);
    PS_ASSERT_EQ_INT(id, (2u << 26) | (1u << 21) | (8u << 16) | (0x20u << 8) | 5u);
    ps_can_id_t f;
    ps_can_id_unpack(id, &f);
    PS_ASSERT_EQ_INT(f.cls, PS_CLS_TELEM);
    PS_ASSERT_EQ_INT(f.src, PS_NODE_ARM_R);
    PS_ASSERT_EQ_INT(f.dst, PS_NODE_ORCH);
    PS_ASSERT_EQ_INT(f.type, PS_T_JOINT_STATE);
    PS_ASSERT_EQ_INT(f.low, 5);
}

PS_TEST(roundtrip_sweep)
{
    /* LCG sweep mirrors the Python property test. */
    uint32_t lcg = 0x1234;
    for (int i = 0; i < 2000; i++) {
        lcg = lcg * 1103515245u + 12345u;
        uint8_t cls = (lcg >> 8) & 0x7, src = (lcg >> 11) & 0x1F, dst = (lcg >> 16) & 0x1F;
        uint8_t type = (lcg >> 21) & 0xFF, low = (lcg >> 3) & 0xFF;
        uint32_t id = ps_can_id_pack(cls, src, dst, type, low);
        PS_ASSERT_TRUE(id <= PS_CAN_ID_MASK);
        ps_can_id_t f;
        ps_can_id_unpack(id, &f);
        PS_ASSERT_EQ_INT(f.cls, cls);
        PS_ASSERT_EQ_INT(f.src, src);
        PS_ASSERT_EQ_INT(f.dst, dst);
        PS_ASSERT_EQ_INT(f.type, type);
        PS_ASSERT_EQ_INT(f.low, low);
    }
}

PS_TEST(safety_wins_arbitration)
{
    uint32_t worst_safety = ps_can_id_pack(PS_CLS_SAFETY, 31, 31, 0xFF, 0xFF);
    PS_ASSERT_TRUE(worst_safety < ps_can_id_pack(PS_CLS_CONTROL, 0, 0, 0, 0));
    PS_ASSERT_TRUE(worst_safety < ps_can_id_pack(PS_CLS_TELEM, 0, 0, 0, 0));
    PS_ASSERT_TRUE(worst_safety < ps_can_id_pack(PS_CLS_XRCE, 0, 0, 0, 0));
}

PS_TEST_MAIN()
