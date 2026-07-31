#include "powersuit_proto/crc16.h"
#include "ps_test.h"

PS_TEST(known_answer)
{
    PS_ASSERT_EQ_INT(ps_crc16((const uint8_t *)"123456789", 9), 0x29B1);
}

PS_TEST(empty_is_init)
{
    PS_ASSERT_EQ_INT(ps_crc16(NULL, 0), 0xFFFF);
}

PS_TEST(incremental_equals_oneshot)
{
    uint8_t data[768];
    for (size_t i = 0; i < sizeof(data); i++) {
        data[i] = (uint8_t)i;
    }
    uint16_t whole = ps_crc16(data, sizeof(data));
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < sizeof(data); i += 7) {
        size_t n = sizeof(data) - i < 7 ? sizeof(data) - i : 7;
        crc = ps_crc16_update(crc, data + i, n);
    }
    PS_ASSERT_EQ_INT(crc, whole);
}

PS_TEST(bit_sensitivity)
{
    uint8_t a[] = "powersuit heartbeat";
    uint16_t base = ps_crc16(a, sizeof(a));
    a[3] ^= 0x01;
    PS_ASSERT_TRUE(ps_crc16(a, sizeof(a)) != base);
}

PS_TEST_MAIN()
