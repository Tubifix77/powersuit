/* VOX energy gate (docs/safety.md §6): mean-square threshold with hang time and
 * a hard per-utterance cap. Pure C99 — host-tested by tests/host/test_energy_gate.c.
 *
 * threshold_q is the block mean square (sample^2 units) equivalent of the
 * configured dBFS level: ms = 10^(dBFS/10) * 32768^2. The gate opens on the
 * first block at/above threshold, stays open for hang_ms past the last voiced
 * block, and force-closes after max_utterance_ms; after a force-close one
 * sub-threshold block is required before it may open again (re-arm hysteresis).
 *
 * State encoding: the declared struct has no dedicated re-arm field, so while
 * CLOSED, opened_ms doubles as the needs-quiet flag (UINT32_MAX sentinel).
 * While OPEN, opened_ms is the open timestamp. now_ms wraps at ~49.7 days;
 * a single misjudged block at wrap is accepted. */
#include "ps_audio.h"

#include <math.h>

#define EGATE_NEEDS_QUIET 0xFFFFFFFFu

void ps_egate_init(ps_egate_t *g, int threshold_dbfs, uint32_t hang_ms, uint32_t max_utterance_ms)
{
    double ms = pow(10.0, (double)threshold_dbfs / 10.0) * 32768.0 * 32768.0;

    if (ms > 2147483647.0) {
        ms = 2147483647.0;
    }
    if (ms < 1.0) {
        ms = 1.0;
    }
    g->threshold_q = (int32_t)ms;
    g->hang_ms = hang_ms;
    g->max_utterance_ms = max_utterance_ms;
    g->open = false;
    g->opened_ms = 0;
    g->last_voiced_ms = 0;
}

bool ps_egate_process(ps_egate_t *g, const int16_t *pcm, size_t n, uint32_t now_ms)
{
    int64_t acc = 0;
    int32_t block_ms;
    bool voiced;
    size_t i;

    if (n == 0) {
        return g->open;
    }
    for (i = 0; i < n; i++) {
        int32_t s = pcm[i];
        acc += (int64_t)s * (int64_t)s;
    }
    block_ms = (int32_t)(acc / (int64_t)n);
    voiced = (block_ms >= g->threshold_q);

    if (g->open) {
        if (voiced) {
            g->last_voiced_ms = now_ms;
        }
        if (now_ms - g->opened_ms >= g->max_utterance_ms) {
            g->open = false;
            g->opened_ms = EGATE_NEEDS_QUIET; /* re-arm requires a quiet block */
        } else if (!voiced && now_ms - g->last_voiced_ms >= g->hang_ms) {
            g->open = false;
            g->opened_ms = 0;
        }
    } else {
        if (g->opened_ms == EGATE_NEEDS_QUIET) {
            if (!voiced) {
                g->opened_ms = 0; /* quiet seen: armed again */
            }
        } else if (voiced) {
            g->open = true;
            g->opened_ms = now_ms;
            g->last_voiced_ms = now_ms;
        }
    }
    return g->open;
}
