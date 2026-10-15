/* ps_can_txq — the order a node puts its own frames on the wire.
 *
 * CAN arbitration only ranks frames that are already competing for the bus. The
 * TWAI driver feeds the controller from a FIFO (esp_twai_onchip.c in IDF 5.5.5:
 * tx_mount_queue is a FreeRTOS queue), so a SAFETY frame handed to it waits behind
 * every frame the same node queued earlier, and never reaches arbitration until
 * they have all gone. Measured on this bench's hardware by the sibling project:
 * 2.5-5.5 ms for a frame whose wire time is ~0.3 ms (docs/bench-log.md).
 *
 * So ps_can keeps a node's pending frames here and hands the driver one at a time:
 * highest class first (network-map §2: lower class value = higher priority, the
 * same order arbitration uses), FIFO within a class, so XRCE byte streams and
 * TELEM/AUDIO sequence numbers keep their order.
 *
 * Pure C99, no FreeRTOS or ESP headers: compiled and tested on the host. Not
 * thread-safe; the caller serialises access. */
#ifndef PS_CAN_TXQ_H
#define PS_CAN_TXQ_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PS_CAN_TXQ_CLASSES 8u    /* the 3-bit class field */
#define PS_CAN_TXQ_DEPTH   16u   /* per class: ps_can's whole slot pool fits in one */

typedef struct {
    uint8_t ring[PS_CAN_TXQ_CLASSES][PS_CAN_TXQ_DEPTH];
    uint8_t head[PS_CAN_TXQ_CLASSES];
    uint8_t count[PS_CAN_TXQ_CLASSES];
} ps_can_txq_t;

void ps_can_txq_init(ps_can_txq_t *q);

/* Append `item` (ps_can's slot index) to class `cls`. False if the class is full
 * or `cls` is out of range; the queue is unchanged then. */
bool ps_can_txq_push(ps_can_txq_t *q, uint8_t cls, uint8_t item);

/* Remove and return the oldest item of the highest-priority non-empty class, or
 * -1 when every class is empty. */
int ps_can_txq_pop(ps_can_txq_t *q);

/* Items queued across all classes. */
size_t ps_can_txq_len(const ps_can_txq_t *q);

#ifdef __cplusplus
}
#endif

#endif /* PS_CAN_TXQ_H */
