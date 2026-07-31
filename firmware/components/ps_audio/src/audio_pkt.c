/* AUDIO-plane packetizer/depacketizer (docs/network-map.md §3.5).
 * Pure C99 — no IDF/FreeRTOS includes; host-tested by tests/host/test_audio_pkt.c.
 *
 * TX: 16-sample (2 ms) frames -> 8 ADPCM bytes; SYNC precedes frame 0 and every
 * PS_AUDIO_SYNC_INTERVAL-th frame, carrying the encoder state valid BEFORE that
 * frame so a receiver seeded from it decodes the following frames bit-exactly.
 * RX: strict low-byte sequence check; on a discontinuity it inserts one frame of
 * silence, unlocks, and waits for the next SYNC (§3.5 resync rule). */
#include "ps_audio.h"

#include "powersuit_proto/can_id.h"

#include <string.h>

/* ---------------------------------------------------------------- TX ------ */

void ps_audio_tx_init(ps_audio_tx_t *tx, uint8_t dir, uint8_t src_node, uint8_t dst_node)
{
    ps_adpcm_init(&tx->enc);
    tx->frame_seq = 0;
    tx->dir = dir;
    tx->src_node = src_node;
    tx->dst_node = dst_node;
    tx->frames_since_sync = 0;
    tx->started = false;
    tx->pending_n = 0;
}

static void tx_emit_sync(ps_audio_tx_t *tx, ps_audio_emit_cb_t emit, void *arg)
{
    ps_audio_sync_t s;
    uint8_t payload[PS_AUDIO_FRAME_BYTES];

    s.dir = tx->dir;
    s.step_index = (uint8_t)tx->enc.step_index;     /* 0..88 by codec invariant */
    s.predictor = (int16_t)tx->enc.predictor;       /* clamped to int16 by codec */
    s.frame_seq = tx->frame_seq;                    /* seq of the NEXT frame */
    s.rsvd = 0;
    PS_WIRE_WRITE(payload, s);
    emit(ps_can_id_pack(PS_CLS_AUDIO, tx->src_node, tx->dst_node, PS_T_AUDIO_SYNC,
                        (uint8_t)(tx->frame_seq & 0xFFu)),
         payload, (uint8_t)sizeof(s), arg);
    tx->frames_since_sync = 0;
    tx->started = true;
}

void ps_audio_tx_sync(ps_audio_tx_t *tx, ps_audio_emit_cb_t emit, void *arg)
{
    tx_emit_sync(tx, emit, arg);
}

size_t ps_audio_tx_push(ps_audio_tx_t *tx, const int16_t *pcm, size_t n,
                        ps_audio_emit_cb_t emit, void *arg)
{
    size_t emitted = 0;

    while (n > 0) {
        size_t take = PS_AUDIO_FRAME_SAMPLES - tx->pending_n;
        if (take > n) {
            take = n;
        }
        memcpy(&tx->pending[tx->pending_n], pcm, take * sizeof(int16_t));
        tx->pending_n += take;
        pcm += take;
        n -= take;

        if (tx->pending_n == PS_AUDIO_FRAME_SAMPLES) {
            uint8_t adpcm[PS_AUDIO_FRAME_BYTES];
            uint8_t type;

            if (!tx->started || tx->frames_since_sync >= PS_AUDIO_SYNC_INTERVAL) {
                tx_emit_sync(tx, emit, arg); /* state snapshot BEFORE this frame */
            }
            ps_adpcm_encode(&tx->enc, tx->pending, PS_AUDIO_FRAME_SAMPLES, adpcm);
            type = (tx->dir == PS_AUDIO_DIR_UP) ? PS_T_AUDIO_UP : PS_T_AUDIO_DOWN;
            emit(ps_can_id_pack(PS_CLS_AUDIO, tx->src_node, tx->dst_node, type,
                                (uint8_t)(tx->frame_seq & 0xFFu)),
                 adpcm, PS_AUDIO_FRAME_BYTES, arg);
            tx->frame_seq = (uint16_t)(tx->frame_seq + 1u); /* wraps at 65536 */
            tx->frames_since_sync++;
            tx->pending_n = 0;
            emitted++;
        }
    }
    return emitted;
}

/* ---------------------------------------------------------------- RX ------ */

void ps_audio_rx_init(ps_audio_rx_t *rx)
{
    ps_adpcm_init(&rx->dec);
    rx->expect_seq = 0;
    rx->locked = false;
    rx->gaps = 0;
    rx->head = 0;
    rx->tail = 0;
}

/* SPSC ring, drop-oldest on overflow: one slot is sacrificed to distinguish
 * full from empty, so capacity is PS_AUDIO_RX_RING_SAMPLES - 1. */
static void rx_ring_push(ps_audio_rx_t *rx, const int16_t *s, size_t n)
{
    size_t i;

    for (i = 0; i < n; i++) {
        rx->ring[rx->head] = s[i];
        rx->head = (rx->head + 1u) % PS_AUDIO_RX_RING_SAMPLES;
        if (rx->head == rx->tail) {
            rx->tail = (rx->tail + 1u) % PS_AUDIO_RX_RING_SAMPLES;
        }
    }
}

void ps_audio_rx_on_frame(ps_audio_rx_t *rx, uint32_t id, const uint8_t *data, uint8_t dlc)
{
    uint8_t type;

    if (ps_can_id_cls(id) != PS_CLS_AUDIO) {
        return;
    }
    type = ps_can_id_type(id);

    if (type == PS_T_AUDIO_SYNC) {
        ps_audio_sync_t s;

        if (dlc < (uint8_t)sizeof(s)) {
            return;
        }
        PS_WIRE_READ(s, data);
        rx->dec.predictor = s.predictor;
        rx->dec.step_index = s.step_index;
        if (rx->dec.step_index > 88) { /* defend the codec invariant on the wire */
            rx->dec.step_index = 88;
        }
        rx->expect_seq = (uint16_t)(s.frame_seq & 0xFFu);
        rx->locked = true;
    } else if (type == PS_T_AUDIO_UP || type == PS_T_AUDIO_DOWN) {
        if (dlc != PS_AUDIO_FRAME_BYTES) {
            return;
        }
        if (!rx->locked) {
            /* Waiting for SYNC; the discontinuity was counted when we unlocked. */
            return;
        }
        if (ps_can_id_low(id) == (uint8_t)rx->expect_seq) {
            int16_t pcm[PS_AUDIO_FRAME_SAMPLES];

            ps_adpcm_decode(&rx->dec, data, PS_AUDIO_FRAME_BYTES, pcm);
            rx_ring_push(rx, pcm, PS_AUDIO_FRAME_SAMPLES);
            rx->expect_seq = (uint16_t)((rx->expect_seq + 1u) & 0xFFu);
        } else {
            /* Discontinuity: one frame (2 ms) of comfort silence, unlock, and
             * hold until the next SYNC reseeds predictor/step_index (§3.5). */
            int16_t silence[PS_AUDIO_FRAME_SAMPLES];

            memset(silence, 0, sizeof(silence));
            rx_ring_push(rx, silence, PS_AUDIO_FRAME_SAMPLES);
            rx->gaps++;
            rx->locked = false;
        }
    }
    /* PS_T_AUDIO_CTL is session signalling — the app layer owns it, not the codec. */
}

size_t ps_audio_rx_available(const ps_audio_rx_t *rx)
{
    return (rx->head + PS_AUDIO_RX_RING_SAMPLES - rx->tail) % PS_AUDIO_RX_RING_SAMPLES;
}

size_t ps_audio_rx_pull(ps_audio_rx_t *rx, int16_t *pcm, size_t max_samples)
{
    size_t avail = ps_audio_rx_available(rx);
    size_t n = (max_samples < avail) ? max_samples : avail;
    size_t i;

    for (i = 0; i < n; i++) {
        pcm[i] = rx->ring[rx->tail];
        rx->tail = (rx->tail + 1u) % PS_AUDIO_RX_RING_SAMPLES;
    }
    return n;
}
