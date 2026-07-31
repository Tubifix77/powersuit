"""Powersuit protocol contract — Python mirror of common/c.

Normative references: docs/network-map.md, docs/safety.md, docs/link-protocol.md.
The C implementation is the firmware-side twin; both are locked together by the
shared vectors under tests/vectors/ (regenerate with tests/gen_vectors.py).
"""

from . import adpcm, can_id, crc16, link, spi_frame, wire

__all__ = ["adpcm", "can_id", "crc16", "link", "spi_frame", "wire"]
__version__ = "0.1.0"
