from powersuit_proto.crc16 import crc16, crc16_update


def test_known_answer():
    # CRC16/CCITT-FALSE catalog value.
    assert crc16(b"123456789") == 0x29B1


def test_empty_is_init():
    assert crc16(b"") == 0xFFFF


def test_incremental_equals_oneshot():
    data = bytes(range(256)) * 3
    whole = crc16(data)
    crc = 0xFFFF
    for i in range(0, len(data), 7):
        crc = crc16_update(crc, data[i:i + 7])
    assert crc == whole


def test_sensitivity():
    a = bytearray(b"powersuit heartbeat")
    base = crc16(bytes(a))
    a[3] ^= 0x01
    assert crc16(bytes(a)) != base
