/* ps_can_txq — see the header for why a node orders its own transmissions. */
#include "ps_can_txq.h"

#include <string.h>

void ps_can_txq_init(ps_can_txq_t *q)
{
    memset(q, 0, sizeof(*q));
}

bool ps_can_txq_push(ps_can_txq_t *q, uint8_t cls, uint8_t item)
{
    if (cls >= PS_CAN_TXQ_CLASSES || q->count[cls] >= PS_CAN_TXQ_DEPTH) {
        return false;
    }
    uint8_t tail = (uint8_t)((q->head[cls] + q->count[cls]) % PS_CAN_TXQ_DEPTH);
    q->ring[cls][tail] = item;
    q->count[cls]++;
    return true;
}

int ps_can_txq_pop(ps_can_txq_t *q)
{
    /* Class 0 is SAFETY: the lowest value is the highest priority, exactly as the
     * class field ranks a frame in arbitration (it is the top of the 29-bit ID). */
    for (uint8_t cls = 0; cls < PS_CAN_TXQ_CLASSES; cls++) {
        if (q->count[cls] > 0) {
            uint8_t item = q->ring[cls][q->head[cls]];
            q->head[cls] = (uint8_t)((q->head[cls] + 1u) % PS_CAN_TXQ_DEPTH);
            q->count[cls]--;
            return item;
        }
    }
    return -1;
}

size_t ps_can_txq_len(const ps_can_txq_t *q)
{
    size_t n = 0;
    for (uint8_t cls = 0; cls < PS_CAN_TXQ_CLASSES; cls++) {
        n += q->count[cls];
    }
    return n;
}
