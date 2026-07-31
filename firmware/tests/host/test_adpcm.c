#include "powersuit_proto/adpcm.h"
#include "vectors/proto_vectors.h"
#include "ps_test.h"

PS_TEST(matches_python_vectors)
{
    /* Cross-language lock: encoding the shared PCM must yield byte-identical ADPCM
     * and the exact final codec state the Python twin produced. */
    ps_adpcm_state_t st;
    ps_adpcm_init(&st);
    static uint8_t enc[PS_VEC_ADPCM_N / 2];
    size_t n = ps_adpcm_encode(&st, PS_VEC_ADPCM_PCM, PS_VEC_ADPCM_N, enc);
    PS_ASSERT_EQ_INT(n, PS_VEC_ADPCM_N / 2);
    PS_ASSERT_EQ_MEM(enc, PS_VEC_ADPCM_ENC, sizeof(enc));
    PS_ASSERT_EQ_INT(st.predictor, PS_VEC_ADPCM_FINAL_PREDICTOR);
    PS_ASSERT_EQ_INT(st.step_index, PS_VEC_ADPCM_FINAL_STEP_INDEX);

    ps_adpcm_state_t dst;
    ps_adpcm_init(&dst);
    static int16_t dec[PS_VEC_ADPCM_N];
    size_t ns = ps_adpcm_decode(&dst, enc, sizeof(enc), dec);
    PS_ASSERT_EQ_INT(ns, PS_VEC_ADPCM_N);
    PS_ASSERT_EQ_MEM(dec, PS_VEC_ADPCM_DEC, sizeof(dec));
}

PS_TEST(chunked_equals_oneshot)
{
    ps_adpcm_state_t a, b;
    ps_adpcm_init(&a);
    ps_adpcm_init(&b);
    static uint8_t one[PS_VEC_ADPCM_N / 2], chunked[PS_VEC_ADPCM_N / 2];
    ps_adpcm_encode(&a, PS_VEC_ADPCM_PCM, PS_VEC_ADPCM_N, one);
    size_t off = 0;
    for (size_t i = 0; i < PS_VEC_ADPCM_N; i += 16) {
        off += ps_adpcm_encode(&b, PS_VEC_ADPCM_PCM + i, 16, chunked + off);
    }
    PS_ASSERT_EQ_INT(off, sizeof(chunked));
    PS_ASSERT_EQ_MEM(chunked, one, sizeof(one));
}

PS_TEST(decoder_state_resync)
{
    /* A decoder seeded with mid-stream state (as carried by AUDIO SYNC) must
     * reproduce the tail exactly. */
    ps_adpcm_state_t enc_st;
    ps_adpcm_init(&enc_st);
    static uint8_t part1[PS_VEC_ADPCM_N / 4], part2[PS_VEC_ADPCM_N / 4];
    ps_adpcm_encode(&enc_st, PS_VEC_ADPCM_PCM, PS_VEC_ADPCM_N / 2, part1);
    ps_adpcm_state_t sync = enc_st;
    ps_adpcm_encode(&enc_st, PS_VEC_ADPCM_PCM + PS_VEC_ADPCM_N / 2, PS_VEC_ADPCM_N / 2, part2);

    ps_adpcm_state_t full;
    ps_adpcm_init(&full);
    static int16_t ref[PS_VEC_ADPCM_N];
    ps_adpcm_decode(&full, part1, sizeof(part1), ref);
    ps_adpcm_decode(&full, part2, sizeof(part2), ref + PS_VEC_ADPCM_N / 2);

    ps_adpcm_state_t late = sync;
    static int16_t tail[PS_VEC_ADPCM_N / 2];
    ps_adpcm_decode(&late, part2, sizeof(part2), tail);
    PS_ASSERT_EQ_MEM(tail, ref + PS_VEC_ADPCM_N / 2, sizeof(tail));
}

PS_TEST_MAIN()
