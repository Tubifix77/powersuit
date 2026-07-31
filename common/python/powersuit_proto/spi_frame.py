"""SPI bridge framing — mirror of common/c/src/spi_frame.c (docs/network-map.md §6)."""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

from .crc16 import crc16_update

SPI_MAGIC = 0xA55A
SPI_VER = 1
SPI_XFER_SIZE = 512
SPI_HDR_SIZE = 8
SPI_REC_SIZE = 16
SPI_MAX_RECORDS = 31

# Header flag bits.
SPIF_ESTOP_LATCHED = 1 << 0
SPIF_OVERFLOW = 1 << 1
SPIF_MORE_PENDING = 1 << 2
SPIF_HUB_FAULT = 1 << 3

# Record bus nibble values.
BUS_CAN1 = 0
BUS_CAN2 = 1
BUS_HUB_LOCAL = 3

_HDR = struct.Struct("<HBBBB H".replace(" ", ""))  # magic, ver, flags, count, seq, crc
_REC = struct.Struct("<IBBH8s")


class SpiFrameError(ValueError):
    """Raised on malformed SPI frames; `code` mirrors the C PS_SPI_E* values."""

    def __init__(self, code: int, message: str):
        super().__init__(message)
        self.code = code


EBADMAGIC = -1
EBADVER = -2
EBADCOUNT = -3
EBADCRC = -4
ESHORT = -5


@dataclass
class CanRecord:
    id: int
    bus: int = BUS_CAN1
    dlc: int = 8
    ts_ms: int = 0
    data: bytes = field(default=b"\x00" * 8)

    def packed(self) -> bytes:
        if not 0 <= self.dlc <= 8:
            raise SpiFrameError(EBADCOUNT, f"dlc {self.dlc} out of range")
        payload = bytes(self.data)[:8].ljust(8, b"\x00")
        return _REC.pack(
            self.id & 0x1FFF_FFFF,
            ((self.bus & 0xF) << 4) | (self.dlc & 0xF),
            0,
            self.ts_ms & 0xFFFF,
            payload,
        )

    @classmethod
    def from_bytes(cls, raw: bytes | memoryview) -> "CanRecord":
        rid, bus_dlc, _rsvd, ts_ms, data = _REC.unpack_from(raw)
        dlc = bus_dlc & 0xF
        if dlc > 8:
            raise SpiFrameError(EBADCOUNT, f"dlc {dlc} out of range")
        return cls(id=rid & 0x1FFF_FFFF, bus=bus_dlc >> 4, dlc=dlc, ts_ms=ts_ms, data=bytes(data))


def build_frame(flags: int, seq: int, records: list[CanRecord]) -> bytes:
    if len(records) > SPI_MAX_RECORDS:
        raise SpiFrameError(EBADCOUNT, f"{len(records)} records > {SPI_MAX_RECORDS}")
    body = b"".join(r.packed() for r in records)
    meta = bytes((SPI_VER, flags & 0xFF, len(records), seq & 0xFF))
    crc = crc16_update(crc16_update(0xFFFF, meta), body)
    frame = struct.pack("<H", SPI_MAGIC) + meta + struct.pack("<H", crc) + body
    return frame.ljust(SPI_XFER_SIZE, b"\x00")


def parse_frame(buf: bytes | memoryview) -> tuple[int, int, list[CanRecord]]:
    """Returns (flags, seq, records). Raises SpiFrameError on any validation failure."""
    if len(buf) < SPI_HDR_SIZE:
        raise SpiFrameError(ESHORT, "buffer shorter than header")
    magic, ver, flags, count, seq, crc = _HDR.unpack_from(buf)
    if magic != SPI_MAGIC:
        raise SpiFrameError(EBADMAGIC, f"bad magic 0x{magic:04X}")
    if ver != SPI_VER:
        raise SpiFrameError(EBADVER, f"bad version {ver}")
    if count > SPI_MAX_RECORDS:
        raise SpiFrameError(EBADCOUNT, f"count {count} > {SPI_MAX_RECORDS}")
    need = SPI_HDR_SIZE + count * SPI_REC_SIZE
    if len(buf) < need:
        raise SpiFrameError(ESHORT, f"need {need} bytes, have {len(buf)}")
    body = bytes(buf[SPI_HDR_SIZE:need])
    meta = bytes((ver, flags, count, seq))
    calc = crc16_update(crc16_update(0xFFFF, meta), body)
    if calc != crc:
        raise SpiFrameError(EBADCRC, f"crc mismatch: frame 0x{crc:04X} calc 0x{calc:04X}")
    records = [
        CanRecord.from_bytes(body[i * SPI_REC_SIZE:(i + 1) * SPI_REC_SIZE]) for i in range(count)
    ]
    return flags, seq, records


def scan(buf: bytes | memoryview, start: int = 0) -> int | None:
    """Offset of the next plausible frame start (magic bytes), or None."""
    lo = SPI_MAGIC & 0xFF
    hi = SPI_MAGIC >> 8
    for i in range(start, len(buf) - 1):
        if buf[i] == lo and buf[i + 1] == hi:
            return i
    return None
