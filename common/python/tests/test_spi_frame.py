import pytest

from powersuit_proto import can_id, spi_frame
from powersuit_proto.can_id import Cls, MsgType, Node
from powersuit_proto.spi_frame import (
    EBADCRC,
    EBADMAGIC,
    EBADCOUNT,
    CanRecord,
    SpiFrameError,
    build_frame,
    parse_frame,
    scan,
)


def _records(n: int) -> list[CanRecord]:
    return [
        CanRecord(
            id=can_id.pack(Cls.TELEM, 1 + (i % 6), Node.ORCH, MsgType.JOINT_STATE, i & 0xFF),
            bus=i % 2,
            dlc=8,
            ts_ms=1000 + i,
            data=bytes((i * 16 + j) & 0xFF for j in range(8)),
        )
        for i in range(n)
    ]


def test_roundtrip_full():
    recs = _records(31)
    frame = build_frame(spi_frame.SPIF_MORE_PENDING, 42, recs)
    assert len(frame) == spi_frame.SPI_XFER_SIZE
    flags, seq, out = parse_frame(frame)
    assert flags == spi_frame.SPIF_MORE_PENDING
    assert seq == 42
    assert out == recs


def test_roundtrip_empty():
    frame = build_frame(0, 0, [])
    flags, seq, out = parse_frame(frame)
    assert (flags, seq, out) == (0, 0, [])
    # tail is zero padding
    assert frame[spi_frame.SPI_HDR_SIZE:] == b"\x00" * (512 - 8)


def test_too_many_records_rejected():
    with pytest.raises(SpiFrameError) as ei:
        build_frame(0, 0, _records(32))
    assert ei.value.code == EBADCOUNT


def test_crc_detects_corruption():
    frame = bytearray(build_frame(0, 7, _records(5)))
    frame[20] ^= 0x40  # flip a bit inside record 0
    with pytest.raises(SpiFrameError) as ei:
        parse_frame(bytes(frame))
    assert ei.value.code == EBADCRC


def test_header_corruption_is_bad_magic():
    frame = bytearray(build_frame(0, 7, _records(2)))
    frame[0] = 0x00
    with pytest.raises(SpiFrameError) as ei:
        parse_frame(bytes(frame))
    assert ei.value.code == EBADMAGIC


def test_scan_recovers_offset():
    frame = build_frame(0, 9, _records(3))
    garbage = b"\xde\xad\xbe\xef" * 5
    stream = garbage + frame
    off = scan(stream)
    assert off == len(garbage)
    flags, seq, recs = parse_frame(stream[off:])
    assert seq == 9 and len(recs) == 3


def test_dlc_out_of_range_rejected():
    rec = CanRecord(id=1, dlc=9)
    with pytest.raises(SpiFrameError):
        rec.packed()


def test_short_data_padded():
    rec = CanRecord(id=1, dlc=3, data=b"\x01\x02\x03")
    raw = rec.packed()
    assert len(raw) == spi_frame.SPI_REC_SIZE
    back = CanRecord.from_bytes(raw)
    assert back.dlc == 3
    assert back.data == b"\x01\x02\x03" + b"\x00" * 5
