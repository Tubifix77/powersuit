"""Fixed-record mmap ring logger — PURE python (no rclpy; natively tested).

File layout: one 64-byte header + capacity x 64-byte records.
Record: {f64 ts, u16 code, 6 x f32 vals, 22 B tag, 8 B pad}.
The write index is monotonic (never wraps); the slot is write_index % capacity,
so chronological order is recoverable after wrap. snapshot() writes an ordered,
non-ring copy. freeze() latches the ring read-only (post-incident preservation).
"""

from __future__ import annotations

import mmap
import os
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

MAGIC = b"PSBB"
VERSION = 1

_HDR = struct.Struct("<4sHHQQB39x")   # magic, version, rsvd, capacity, write_index, frozen
_REC = struct.Struct("<dH6f22s8x")    # ts, code, vals[6], tag, pad
HEADER_SIZE = _HDR.size
RECORD_SIZE = _REC.size
assert HEADER_SIZE == 64 and RECORD_SIZE == 64

DEFAULT_SIZE = 64 * 1024 * 1024


@dataclass(frozen=True)
class Record:
    ts: float
    code: int
    vals: tuple[float, float, float, float, float, float]
    tag: bytes  # up to 22 bytes, NUL-stripped

    @classmethod
    def unpack(cls, raw: bytes) -> "Record":
        ts, code, v0, v1, v2, v3, v4, v5, tag = _REC.unpack(raw)
        return cls(ts, code, (v0, v1, v2, v3, v4, v5), tag.rstrip(b"\x00"))


class RingLog:
    def __init__(self, path: str | Path, size_bytes: int = DEFAULT_SIZE):
        if size_bytes < HEADER_SIZE + RECORD_SIZE:
            raise ValueError(f"size_bytes {size_bytes} too small for one record")
        self._path = Path(path)
        self._capacity = (size_bytes - HEADER_SIZE) // RECORD_SIZE
        total = HEADER_SIZE + self._capacity * RECORD_SIZE

        existing = self._path.is_file() and self._path.stat().st_size == total
        self._fd = os.open(self._path, os.O_RDWR | os.O_CREAT, 0o644)
        os.ftruncate(self._fd, total)
        self._mm = mmap.mmap(self._fd, total)

        if existing and self._mm[:4] == MAGIC:
            magic, ver, _rsvd, cap, widx, frozen = _HDR.unpack(self._mm[:HEADER_SIZE])
            if ver == VERSION and cap == self._capacity:
                self._write_index = widx
                self._frozen = bool(frozen)
                return
        self._write_index = 0
        self._frozen = False
        self._flush_header()

    # ------------------------------------------------------------------ header
    def _flush_header(self) -> None:
        self._mm[:HEADER_SIZE] = _HDR.pack(
            MAGIC, VERSION, 0, self._capacity, self._write_index, int(self._frozen))

    # ------------------------------------------------------------------ API
    @property
    def capacity(self) -> int:
        return self._capacity

    @property
    def frozen(self) -> bool:
        return self._frozen

    @property
    def write_index(self) -> int:
        return self._write_index

    def __len__(self) -> int:
        return min(self._write_index, self._capacity)

    def append(self, ts: float, code: int, vals=(), tag: bytes | str = b"") -> bool:
        """Append one record; returns False (and writes nothing) while frozen."""
        if self._frozen:
            return False
        v = tuple(float(x) for x in vals)[:6]
        v = v + (0.0,) * (6 - len(v))
        if isinstance(tag, str):
            tag = tag.encode("utf-8", "replace")
        slot = self._write_index % self._capacity
        off = HEADER_SIZE + slot * RECORD_SIZE
        self._mm[off:off + RECORD_SIZE] = _REC.pack(float(ts), int(code) & 0xFFFF, *v, tag[:22])
        self._write_index += 1
        self._flush_header()
        return True

    def records(self) -> Iterator[Record]:
        """Chronological (oldest -> newest) iteration of the live ring."""
        n = len(self)
        start = self._write_index - n
        for i in range(start, self._write_index):
            slot = i % self._capacity
            off = HEADER_SIZE + slot * RECORD_SIZE
            yield Record.unpack(bytes(self._mm[off:off + RECORD_SIZE]))

    def snapshot(self, path: str | Path) -> int:
        """Copy the ring, in chronological order, to `path`. Returns record count.

        Snapshot layout: header (capacity == write_index == n, frozen=1) + n records —
        a plain sequential log readable with the same Record format.
        """
        recs = list(self.records())
        tmp = Path(str(path) + ".tmp")
        with open(tmp, "wb") as fh:
            fh.write(_HDR.pack(MAGIC, VERSION, 0, len(recs), len(recs), 1))
            for r in recs:
                fh.write(_REC.pack(r.ts, r.code, *r.vals, r.tag))
        os.replace(tmp, path)
        return len(recs)

    def freeze(self) -> bool:
        """Latch the ring read-only. Idempotent: True first time, False after."""
        if self._frozen:
            return False
        self._frozen = True
        self._flush_header()
        return True

    def thaw(self) -> None:
        self._frozen = False
        self._flush_header()

    def close(self) -> None:
        if self._mm is not None:
            self._mm.flush()
            self._mm.close()
            self._mm = None  # type: ignore[assignment]
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None  # type: ignore[assignment]

    def __enter__(self) -> "RingLog":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def read_snapshot(path: str | Path) -> list[Record]:
    """Read a snapshot file produced by RingLog.snapshot()."""
    raw = Path(path).read_bytes()
    magic, ver, _rsvd, cap, widx, _frozen = _HDR.unpack(raw[:HEADER_SIZE])
    if magic != MAGIC or ver != VERSION:
        raise ValueError(f"{path} is not a powersuit blackbox snapshot")
    out = []
    for i in range(min(cap, widx)):
        off = HEADER_SIZE + i * RECORD_SIZE
        out.append(Record.unpack(raw[off:off + RECORD_SIZE]))
    return out
