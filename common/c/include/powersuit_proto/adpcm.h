/* IMA ADPCM codec (4 bits/sample, mono) — the suit's voice codec at 8 kHz.
 * Low nibble carries the first sample of each byte. Continuous-state streaming with
 * periodic AUDIO SYNC resync frames (docs/network-map.md §3.5). Bit-identical to
 * common/python/powersuit_proto/adpcm.py; locked by generated shared vectors. */
#ifndef POWERSUIT_PROTO_ADPCM_H
#define POWERSUIT_PROTO_ADPCM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t predictor;   /* clamped to int16 range */
    int32_t step_index;  /* 0..88 */
} ps_adpcm_state_t;

void ps_adpcm_init(ps_adpcm_state_t *st);

/* Encode n PCM16 samples (n must be even). Writes n/2 bytes to out. Returns n/2. */
size_t ps_adpcm_encode(ps_adpcm_state_t *st, const int16_t *pcm, size_t n, uint8_t *out);

/* Decode nbytes ADPCM bytes into nbytes*2 PCM16 samples. Returns sample count. */
size_t ps_adpcm_decode(ps_adpcm_state_t *st, const uint8_t *in, size_t nbytes, int16_t *pcm);

#ifdef __cplusplus
}
#endif

#endif /* POWERSUIT_PROTO_ADPCM_H */
