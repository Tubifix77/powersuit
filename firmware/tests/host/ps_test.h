/* ps_test — minimal single-header host test framework for the pure-C modules.
 * One executable per test_*.c file: define tests with PS_TEST(name), finish the
 * file with PS_TEST_MAIN(). Integrates with CTest via exit code. */
#ifndef PS_TEST_H
#define PS_TEST_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef void (*ps_test_fn_t)(void);

typedef struct {
    const char *name;
    ps_test_fn_t fn;
} ps_test_case_t;

#define PS_TEST_MAX_CASES 128

static ps_test_case_t ps_test_cases[PS_TEST_MAX_CASES];
static int ps_test_count = 0;
static int ps_test_failures = 0;
static const char *ps_test_current = "";

static void ps_test_register(const char *name, ps_test_fn_t fn)
{
    if (ps_test_count < PS_TEST_MAX_CASES) {
        ps_test_cases[ps_test_count].name = name;
        ps_test_cases[ps_test_count].fn = fn;
        ps_test_count++;
    }
}

#define PS_TEST(name)                                                        \
    static void ps_test_body_##name(void);                                   \
    __attribute__((constructor)) static void ps_test_reg_##name(void)        \
    {                                                                        \
        ps_test_register(#name, ps_test_body_##name);                        \
    }                                                                        \
    static void ps_test_body_##name(void)

#define PS_FAIL(fmt, ...)                                                    \
    do {                                                                     \
        fprintf(stderr, "FAIL %s (%s:%d): " fmt "\n", ps_test_current,       \
                __FILE__, __LINE__, ##__VA_ARGS__);                          \
        ps_test_failures++;                                                  \
        return;                                                              \
    } while (0)

#define PS_ASSERT_TRUE(cond)                                                 \
    do {                                                                     \
        if (!(cond)) PS_FAIL("expected true: %s", #cond);                    \
    } while (0)

#define PS_ASSERT_FALSE(cond) PS_ASSERT_TRUE(!(cond))

#define PS_ASSERT_EQ_INT(a, b)                                               \
    do {                                                                     \
        long long pa = (long long)(a), pb = (long long)(b);                  \
        if (pa != pb) PS_FAIL("%s == %lld, expected %s == %lld", #a, pa, #b, pb); \
    } while (0)

#define PS_ASSERT_EQ_MEM(a, b, n)                                            \
    do {                                                                     \
        if (memcmp((a), (b), (n)) != 0) PS_FAIL("memory mismatch: %s vs %s (%zu bytes)", #a, #b, (size_t)(n)); \
    } while (0)

#define PS_ASSERT_NEAR(a, b, tol)                                            \
    do {                                                                     \
        double pa = (double)(a), pb = (double)(b);                           \
        if (fabs(pa - pb) > (double)(tol)) PS_FAIL("%s == %g, expected within %g of %g", #a, pa, (double)(tol), pb); \
    } while (0)

#define PS_TEST_MAIN()                                                       \
    int main(void)                                                           \
    {                                                                        \
        for (int i = 0; i < ps_test_count; i++) {                            \
            ps_test_current = ps_test_cases[i].name;                         \
            ps_test_cases[i].fn();                                           \
        }                                                                    \
        printf("%s: %d tests, %d failures\n",                                \
               ps_test_count ? ps_test_cases[0].name : "?", ps_test_count,   \
               ps_test_failures);                                            \
        return ps_test_failures ? 1 : 0;                                     \
    }

#endif /* PS_TEST_H */
