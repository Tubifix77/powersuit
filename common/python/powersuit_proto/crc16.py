"""CRC16-CCITT-FALSE — mirror of common/c/src/crc16.c. KAT: b"123456789" -> 0x29B1."""

from __future__ import annotations


def crc16_update(crc: int, data: bytes | bytearray | memoryview) -> int:
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def crc16(data: bytes | bytearray | memoryview) -> int:
    return crc16_update(0xFFFF, data)
