#include "ps_audio.h"
#include "ps_test.h"

/* Deterministic pcm exercise: 300 frames = 4800 samples crosses the 256-wide
 * low-byte sequence wrap (frame 256) and forces SYNC cadence over 6 intervals. */
#define T_FRAMES 300u
#define T_NSAMP  (T_FRAMES * PS_AUDIO_FRAME_SAMPLES)

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
} emission_t;

#define MAX_EMIS 512u
static emission_t emis[MAX_EMIS];
static size_t emis_n;

static void emit_cb(uint32_t id, const uint8_t *data, uint8_t dlc, void *arg)
{
    (void)arg;
    if (emis_n < MAX_EMIS) {
        emis[emis_n].id = id;
        memcpy(emis[emis_n].data, data, dlc);
        emis[emis_n].dlc = dlc;
        emis_n++;
    }
}

static int16_t pcm_in[T_NSAMP];
static int16_t ref_pcm[T_NSAMP];

static void make_input(void)
{
    /* LCG noise shaped to +-8k: rough for ADPCM, fully deterministic. */
    uint32_t x = 0x12345u;
    size_t i;
    for (i = 0; i < T_NSAMP; i++) {
        x = x * 1103515245u + 12345u;
        pcm_in[i] = (int16_t)(((x >> 16) & 0x3FFFu)) - 8192;
    }
}

/* Reference chain: one continuous encode + one continuous decode from fresh
 * state — exactly what an in-sync packetized receiver must reproduce. */
static void make_reference(void)
{
    ps_adpcm_state_t e, d;
    static uint8_t bytes[T_NSAMP / 2];
    ps_adpcm_init(&e);
    ps_adpcm_init(&d);
    ps_adpcm_encode(&e, pcm_in, T_NSAMP, bytes);
    ps_adpcm_decode(&d, bytes, sizeof(bytes), ref_pcm);
}

static ps_audio_rx_t rx_st; /* ~8 KB: keep off the stack */

static void run_tx(void)
{
    ps_audio_tx_t tx;
    size_t off = 0;
    size_t frames = 0;
    static const size_t chunks[] = { 1, 7, 16, 33, 160 }; /* exercise pending buffer */
    size_t ci = 0;

    make_input();
    make_reference();
    emis_n = 0;
    ps_audio_tx_init(&tx, PS_AUDIO_DIR_UP, PS_NODE_HELMET, PS_NODE_ORCH);
    while (off < T_NSAMP) {
        size_t n = chunks[ci % (sizeof(chunks) / sizeof(chunks[0]))];
        ci++;
        if (n > T_NSAMP - off) {
            n = T_NSAMP - off;
        }
        frames += ps_audio_tx_push(&tx, pcm_in + off, n, emit_cb, NULL);
        off += n;
    }
    if (frames != T_FRAMES) {
        emis_n = 0; /* poison so dependent asserts fail loudly */
    }
}

PS_TEST(roundtrip_matches_reference_chain)
{
    static int16_t out[T_NSAMP];
    size_t out_n = 0;
    size_t i;

    run_tx();
    PS_ASSERT_EQ_INT(emis_n, T_FRAMES + T_FRAMES / PS_AUDIO_SYNC_INTERVAL);

    ps_audio_rx_init(&rx_st);
    for (i = 0; i < emis_n; i++) {
        ps_audio_rx_on_frame(&rx_st, emis[i].id, emis[i].data, emis[i].dlc);
        out_n += ps_audio_rx_pull(&rx_st, out + out_n, T_NSAMP - out_n);
    }
    PS_ASSERT_EQ_INT(out_n, T_NSAMP);
    PS_ASSERT_EQ_MEM(out, ref_pcm, sizeof(ref_pcm));
    PS_ASSERT_EQ_INT(rx_st.gaps, 0);
    PS_ASSERT_TRUE(rx_st.locked);
}

PS_TEST(sync_cadence_every_50_frames_plus_start)
{
    size_t syncs = 0;
    size_t i;

    run_tx();
    /* First emission is the stream-start SYNC. */
    PS_ASSERT_EQ_INT(ps_can_id_type(emis[0].id), PS_T_AUDIO_SYNC);
    for (i = 0; i < emis_n; i++) {
        if (ps_can_id_type(emis[i].id) == PS_T_AUDIO_SYNC) {
            /* SYNC + following 50 frames = 51 emissions per interval. */
            PS_ASSERT_EQ_INT(i, syncs * (PS_AUDIO_SYNC_INTERVAL + 1u));
            syncs++;
            /* SYNC announces the next frame: payload frame_seq low byte must
             * match the immediately following FRAME id low byte. */
            PS_ASSERT_TRUE(i + 1 < emis_n);
            {
                ps_audio_sync_t s;
                PS_WIRE_READ(s, emis[i].data);
                PS_ASSERT_EQ_INT(ps_can_id_type(emis[i + 1].id), PS_T_AUDIO_UP);
                PS_ASSERT_EQ_INT(ps_can_id_low(emis[i + 1].id), s.frame_seq & 0xFFu);
                PS_ASSERT_EQ_INT(s.dir, PS_AUDIO_DIR_UP);
            }
        }
    }
    PS_ASSERT_EQ_INT(syncs, T_FRAMES / PS_AUDIO_SYNC_INTERVAL); /* 6 = 1 start + 5 */
}

PS_TEST(frame_low_byte_wraps_at_256)
{
    size_t frame_idx = 0;
    size_t i;

    run_tx();
    for (i = 0; i < emis_n; i++) {
        if (ps_can_id_type(emis[i].id) == PS_T_AUDIO_UP) {
            PS_ASSERT_EQ_INT(ps_can_id_low(emis[i].id), frame_idx & 0xFFu);
            frame_idx++;
        }
    }
    PS_ASSERT_EQ_INT(frame_idx, T_FRAMES); /* 300 > 256: wrap exercised */
}

PS_TEST(single_frame_loss_silence_then_bit_exact_resync)
{
    /* Drop the FRAME carrying seq 60. Expected receiver output:
     * frames 0..59 decoded (960), one 16-sample silence gap at the mismatch,
     * frames 61..99 held (unlocked), then from the SYNC before frame 100 the
     * tail decodes bit-exactly against the reference chain (state seeding). */
    static int16_t out[T_NSAMP];
    static const int16_t zeros[PS_AUDIO_FRAME_SAMPLES] = { 0 };
    size_t out_n = 0;
    size_t frame_idx = 0;
    size_t i;

    run_tx();
    ps_audio_rx_init(&rx_st);
    for (i = 0; i < emis_n; i++) {
        int is_frame = (ps_can_id_type(emis[i].id) == PS_T_AUDIO_UP);
        size_t this_frame = frame_idx;
        if (is_frame) {
            frame_idx++;
        }
        if (is_frame && this_frame == 60u) {
            continue; /* the induced loss */
        }
        ps_audio_rx_on_frame(&rx_st, emis[i].id, emis[i].data, emis[i].dlc);
        out_n += ps_audio_rx_pull(&rx_st, out + out_n, T_NSAMP - out_n);
    }

    PS_ASSERT_EQ_INT(rx_st.gaps, 1);
    /* 60 frames + 1 silence frame + frames 100..299. */
    PS_ASSERT_EQ_INT(out_n, 60u * 16u + 16u + (T_FRAMES - 100u) * 16u);
    PS_ASSERT_EQ_MEM(out, ref_pcm, 60u * 16u * sizeof(int16_t));
    PS_ASSERT_EQ_MEM(out + 60u * 16u, zeros, sizeof(zeros));
    PS_ASSERT_EQ_MEM(out + 61u * 16u, ref_pcm + 100u * 16u,
                     (T_FRAMES - 100u) * 16u * sizeof(int16_t));
}

PS_TEST(ring_overflow_drops_oldest)
{
    /* Feed all 300 frames without pulling: 4800 decoded samples into a ring
     * holding PS_AUDIO_RX_RING_SAMPLES-1; the oldest 705 must be gone. */
    static int16_t out[T_NSAMP];
    const size_t cap = PS_AUDIO_RX_RING_SAMPLES - 1u;
    size_t got;
    size_t i;

    run_tx();
    ps_audio_rx_init(&rx_st);
    for (i = 0; i < emis_n; i++) {
        ps_audio_rx_on_frame(&rx_st, emis[i].id, emis[i].data, emis[i].dlc);
    }
    PS_ASSERT_EQ_INT(ps_audio_rx_available(&rx_st), cap);
    got = ps_audio_rx_pull(&rx_st, out, T_NSAMP);
    PS_ASSERT_EQ_INT(got, cap);
    PS_ASSERT_EQ_MEM(out, ref_pcm + (T_NSAMP - cap), cap * sizeof(int16_t));
    PS_ASSERT_EQ_INT(ps_audio_rx_available(&rx_st), 0);
}

PS_TEST(frames_before_first_sync_are_held)
{
    /* A receiver that missed the stream-start SYNC must stay silent until the
     * next SYNC, then decode bit-exactly. Skip emission 0 (the start SYNC). */
    static int16_t out[T_NSAMP];
    size_t out_n = 0;
    size_t i;

    run_tx();
    ps_audio_rx_init(&rx_st);
    for (i = 1; i < emis_n; i++) {
        ps_audio_rx_on_frame(&rx_st, emis[i].id, emis[i].data, emis[i].dlc);
        out_n += ps_audio_rx_pull(&rx_st, out + out_n, T_NSAMP - out_n);
    }
    /* Frames 0..49 held; SYNC before frame 50 locks; 250 frames decoded. */
    PS_ASSERT_EQ_INT(out_n, (T_FRAMES - 50u) * 16u);
    PS_ASSERT_EQ_MEM(out, ref_pcm + 50u * 16u, out_n * sizeof(int16_t));
}

PS_TEST_MAIN()
