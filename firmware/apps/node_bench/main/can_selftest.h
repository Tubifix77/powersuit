/* can_selftest — one board, no wiring: ps_can's transmit path on real silicon.
 * See can_selftest.c. */
#pragma once

#include <stdbool.h>

/* Runs every check, prints "SELFTEST PASS|FAIL <name>: ..." per check and a final
 * "SELFTEST_DONE OK|FAIL <n>". Returns true when every check passed. */
bool bench_can_selftest(void);
