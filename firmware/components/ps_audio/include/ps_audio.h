/* ps_audio — AUDIO-plane packetizer/depacketizer + energy gate (docs/network-map.md §3.5).
 * FROZEN API. Everything in this header is pure C99 (no IDF deps) and host-tested;
 * the I2S plumbing lives in the helmet app, not here.
 *
 * TX: PCM16 @ 8 kHz in -> ADPCM -> 8-byte AUDIO frames + SYNC every
 * PS_AUDIO_SYNC_INTERVAL frames (and one SYNC at stream start).
 * RX: AUDIO/SYNC frames in -> jitter-tolerant decode -> PCM16 pull. On a seq gap the
 * decoder holds silence and re-locks at the next SYNC. */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "powersuit_proto/adpcm.h"
#include "powersuit_proto/wire.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PS_AUDIO_SAMPLE_RATE      8000u
#define PS_AUDIO_FRAME_BYTES      8u                 /* ADPCM bytes per CAN frame */
#define PS_AUDIO_FRAME_SAMPLES    16u                /* 2 ms per frame */
#define PS_AUDIO_SYNC_INTERVAL    50u                /* frames between SYNCs (100 ms) */
#define PS_AUDIO_DIR_DOWN         0u
#define PS_AUDIO_DIR_UP           1u
#define PS_AUDIO_RX_RING_SAMPLES  4096u              /* ~512 ms of PCM */

/* Emit callback: hands a ready CAN frame (AUDIO FRAME or SYNC, fully addressed)
 * to the caller, who queues it on ps_can / SPI. */
struct ps_can_frame;  /* matches ps_can.h layout: {u32 id; u8 dlc; u8 data[8];} */
typedef void (*ps_audio_emit_cb_t)(uint32_t id, const uint8_t *data, uint8_t dlc, void *arg);

typedef struct {
    ps_adpcm_state_t enc;
    uint16_t frame_seq;
    uint8_t  dir;
    uint8_t  src_node, dst_node;
    uint32_t frames_since_sync;
    bool     started;
    int16_t  pending[PS_AUDIO_FRAME_SAMPLES];
    size_t   pending_n;
} ps_audio_tx_t;

void ps_audio_tx_init(ps_audio_tx_t *tx, uint8_t dir, uint8_t src_node, uint8_t dst_node);
/* Push any number of samples; emits complete frames via cb. Returns frames emitted. */
size_t ps_audio_tx_push(ps_audio_tx_t *tx, const int16_t *pcm, size_t n,
                        ps_audio_emit_cb_t emit, void *arg);
/* Emit a SYNC immediately (stream start / after CTL start). */
void ps_audio_tx_sync(ps_audio_tx_t *tx, ps_audio_emit_cb_t emit, void *arg);

typedef struct {
    ps_adpcm_state_t dec;
    uint16_t expect_seq;      /* next FRAME low-byte expected */
    bool     locked;          /* false until first SYNC (or first frame from seq 0) */
    uint32_t gaps;            /* seq discontinuities observed */
    int16_t  ring[PS_AUDIO_RX_RING_SAMPLES];
    size_t   head, tail;      /* SPSC ring: producer on_frame, consumer pull */
} ps_audio_rx_t;

void ps_audio_rx_init(ps_audio_rx_t *rx);
/* Feed an AUDIO-class CAN frame (FRAME_UP/FRAME_DOWN/SYNC — id decides). */
void ps_audio_rx_on_frame(ps_audio_rx_t *rx, uint32_t id, const uint8_t *data, uint8_t dlc);
/* Pull decoded PCM; returns samples copied (0 when dry — caller feeds silence). */
size_t ps_audio_rx_pull(ps_audio_rx_t *rx, int16_t *pcm, size_t max_samples);
size_t ps_audio_rx_available(const ps_audio_rx_t *rx);

/* --- VOX energy gate (docs/safety.md §6) --- */
typedef struct {
    int32_t threshold_q;      /* mean-square threshold (internal units) */
    uint32_t hang_ms;
    uint32_t max_utterance_ms;
    /* state */
    bool     open;
    uint32_t opened_ms;
    uint32_t last_voiced_ms;
} ps_egate_t;

/* threshold_dbfs e.g. -38 (contract default), hang 300 ms, max utterance 8000 ms. */
void ps_egate_init(ps_egate_t *g, int threshold_dbfs, uint32_t hang_ms, uint32_t max_utterance_ms);
/* Process one block; now_ms monotonic. Returns current gate state (open = stream). */
bool ps_egate_process(ps_egate_t *g, const int16_t *pcm, size_t n, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
