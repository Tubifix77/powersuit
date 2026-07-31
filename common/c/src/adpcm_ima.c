#include "powersuit_proto/adpcm.h"

static const int16_t STEP_TABLE[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

static const int8_t INDEX_TABLE[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

void ps_adpcm_init(ps_adpcm_state_t *st)
{
    st->predictor = 0;
    st->step_index = 0;
}

static int32_t clamp16(int32_t v)
{
    if (v > 32767) {
        return 32767;
    }
    if (v < -32768) {
        return -32768;
    }
    return v;
}

static uint8_t encode_sample(ps_adpcm_state_t *st, int16_t sample)
{
    int32_t step = STEP_TABLE[st->step_index];
    int32_t diff = (int32_t)sample - st->predictor;
    uint8_t code = 0;

    if (diff < 0) {
        code = 8;
        diff = -diff;
    }
    int32_t vpdiff = step >> 3;
    if (diff >= step) {
        code |= 4;
        diff -= step;
        vpdiff += step;
    }
    step >>= 1;
    if (diff >= step) {
        code |= 2;
        diff -= step;
        vpdiff += step;
    }
    step >>= 1;
    if (diff >= step) {
        code |= 1;
        vpdiff += step;
    }

    if (code & 8) {
        st->predictor = clamp16(st->predictor - vpdiff);
    } else {
        st->predictor = clamp16(st->predictor + vpdiff);
    }
    st->step_index += INDEX_TABLE[code];
    if (st->step_index < 0) {
        st->step_index = 0;
    } else if (st->step_index > 88) {
        st->step_index = 88;
    }
    return code;
}

static int16_t decode_sample(ps_adpcm_state_t *st, uint8_t code)
{
    int32_t step = STEP_TABLE[st->step_index];
    int32_t vpdiff = step >> 3;

    if (code & 4) {
        vpdiff += step;
    }
    if (code & 2) {
        vpdiff += step >> 1;
    }
    if (code & 1) {
        vpdiff += step >> 2;
    }
    if (code & 8) {
        st->predictor = clamp16(st->predictor - vpdiff);
    } else {
        st->predictor = clamp16(st->predictor + vpdiff);
    }
    st->step_index += INDEX_TABLE[code];
    if (st->step_index < 0) {
        st->step_index = 0;
    } else if (st->step_index > 88) {
        st->step_index = 88;
    }
    return (int16_t)st->predictor;
}

size_t ps_adpcm_encode(ps_adpcm_state_t *st, const int16_t *pcm, size_t n, uint8_t *out)
{
    size_t nbytes = n / 2;
    for (size_t i = 0; i < nbytes; i++) {
        uint8_t lo = encode_sample(st, pcm[2 * i]);
        uint8_t hi = encode_sample(st, pcm[2 * i + 1]);
        out[i] = (uint8_t)(lo | (hi << 4));
    }
    return nbytes;
}

size_t ps_adpcm_decode(ps_adpcm_state_t *st, const uint8_t *in, size_t nbytes, int16_t *pcm)
{
    for (size_t i = 0; i < nbytes; i++) {
        pcm[2 * i] = decode_sample(st, (uint8_t)(in[i] & 0xF));
        pcm[2 * i + 1] = decode_sample(st, (uint8_t)(in[i] >> 4));
    }
    return nbytes * 2;
}
