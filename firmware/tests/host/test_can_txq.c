/* ps_can_txq: a node's own transmit order. The overtaking test is the one that
 * matters — it is exactly the case the driver's FIFO gets wrong (a SAFETY frame
 * queued behind the node's own telemetry), and it fails if pop() is reduced to
 * arrival order. The FIFO-within-class tests guard the other direction: XRCE is a
 * byte stream and TELEM carries sequence numbers, so reordering inside a class
 * would corrupt them. */
#include "powersuit_proto/can_id.h"
#include "ps_can_txq.h"
#include "ps_test.h"

PS_TEST(empty_pops_minus_one)
{
    ps_can_txq_t q;
    ps_can_txq_init(&q);
    PS_ASSERT_EQ_INT(ps_can_txq_pop(&q), -1);
    PS_ASSERT_EQ_INT(ps_can_txq_len(&q), 0);
}

PS_TEST(safety_overtakes_queued_telemetry)
{
    ps_can_txq_t q;
    ps_can_txq_init(&q);
    for (uint8_t i = 0; i < 10; i++) {
        PS_ASSERT_TRUE(ps_can_txq_push(&q, PS_CLS_TELEM, i));
    }
    PS_ASSERT_TRUE(ps_can_txq_push(&q, PS_CLS_SAFETY, 42));
    PS_ASSERT_EQ_INT(ps_can_txq_pop(&q), 42);      /* not 0: it did not wait its turn */
    PS_ASSERT_EQ_INT(ps_can_txq_pop(&q), 0);
    PS_ASSERT_EQ_INT(ps_can_txq_len(&q), 9);
}

PS_TEST(classes_leave_in_priority_order)
{
    ps_can_txq_t q;
    ps_can_txq_init(&q);
    /* Pushed lowest priority first; must come out SAFETY..MGMT. */
    const uint8_t order[] = { PS_CLS_MGMT, PS_CLS_AUDIO, PS_CLS_XRCE,
                              PS_CLS_TELEM, PS_CLS_CONTROL, PS_CLS_SAFETY };
    for (size_t i = 0; i < sizeof(order); i++) {
        PS_ASSERT_TRUE(ps_can_txq_push(&q, order[i], order[i]));
    }
    for (int want = PS_CLS_SAFETY; want <= PS_CLS_MGMT; want++) {
        PS_ASSERT_EQ_INT(ps_can_txq_pop(&q), want);
    }
    PS_ASSERT_EQ_INT(ps_can_txq_pop(&q), -1);
}

PS_TEST(fifo_within_a_class_across_wraparound)
{
    ps_can_txq_t q;
    ps_can_txq_init(&q);
    /* Interleave pushes and pops so head wraps the ring several times, popping
     * early whenever the class is full so the push never meets a full ring. */
    int next_in = 0, next_out = 0;
    for (int round = 0; round < 5 * (int)PS_CAN_TXQ_DEPTH; round++) {
        if (ps_can_txq_len(&q) == PS_CAN_TXQ_DEPTH) {
            PS_ASSERT_EQ_INT(ps_can_txq_pop(&q), next_out++ & 0xFF);
        }
        PS_ASSERT_TRUE(ps_can_txq_push(&q, PS_CLS_XRCE, (uint8_t)(next_in++ & 0xFF)));
        if (round % 3 != 0) {
            PS_ASSERT_EQ_INT(ps_can_txq_pop(&q), next_out++ & 0xFF);
        }
    }
    PS_ASSERT_TRUE(next_out > (int)(3 * PS_CAN_TXQ_DEPTH));   /* head really wrapped */
    while (ps_can_txq_len(&q) > 0) {
        PS_ASSERT_EQ_INT(ps_can_txq_pop(&q), next_out++ & 0xFF);
    }
    PS_ASSERT_EQ_INT(next_in, next_out);
}

PS_TEST(full_class_refuses_without_side_effects)
{
    ps_can_txq_t q;
    ps_can_txq_init(&q);
    for (uint8_t i = 0; i < PS_CAN_TXQ_DEPTH; i++) {
        PS_ASSERT_TRUE(ps_can_txq_push(&q, PS_CLS_TELEM, i));
    }
    PS_ASSERT_FALSE(ps_can_txq_push(&q, PS_CLS_TELEM, 99));
    /* A full class does not block another class. */
    PS_ASSERT_TRUE(ps_can_txq_push(&q, PS_CLS_SAFETY, 77));
    PS_ASSERT_EQ_INT(ps_can_txq_len(&q), PS_CAN_TXQ_DEPTH + 1);
    PS_ASSERT_EQ_INT(ps_can_txq_pop(&q), 77);
    for (uint8_t i = 0; i < PS_CAN_TXQ_DEPTH; i++) {
        PS_ASSERT_EQ_INT(ps_can_txq_pop(&q), i);   /* 99 never got in */
    }
}

PS_TEST(out_of_range_class_refused)
{
    ps_can_txq_t q;
    ps_can_txq_init(&q);
    PS_ASSERT_FALSE(ps_can_txq_push(&q, PS_CAN_TXQ_CLASSES, 1));
    PS_ASSERT_FALSE(ps_can_txq_push(&q, 255, 1));
    PS_ASSERT_EQ_INT(ps_can_txq_len(&q), 0);
}

PS_TEST(class_matches_the_can_id_field)
{
    /* The scheduler ranks by the class field ps_can_id_cls() extracts, so its
     * order is the bus's arbitration order by construction. */
    uint32_t hb = ps_can_id_pack(PS_CLS_SAFETY, 8, 0, PS_T_HEARTBEAT, 1);
    uint32_t tl = ps_can_id_pack(PS_CLS_TELEM, 1, 8, 0x20, 1);
    PS_ASSERT_TRUE(hb < tl);
    PS_ASSERT_TRUE(ps_can_id_cls(hb) < ps_can_id_cls(tl));
}

PS_TEST_MAIN()
