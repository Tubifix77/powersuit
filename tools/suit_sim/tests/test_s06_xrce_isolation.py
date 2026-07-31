"""Scenario 6 — XRCE streams from different nodes must not bleed into each other.

Six micro-ROS clients share one CAN plane and one SPI link. The bridge separates
them by source node before handing each to its own agent PTY; if that demux ever
interleaves two clients' bytes, both sessions corrupt silently. This proves the
reassembly is exact (docs/network-map.md §3.4, §7).
"""

from __future__ import annotations

import pytest
from powersuit_proto.can_id import Node

from suit_sim.harness import SuitHarness, until

pytestmark = pytest.mark.asyncio


async def test_interleaved_streams_reassemble_per_source():
    async with SuitHarness() as suit:
        # Distinct, self-identifying payloads so any cross-talk is obvious.
        payloads = {
            Node.ARM_R: bytes(range(0, 60)),
            Node.LEG_L: bytes(range(200, 260)),
        }

        # Interleave at 8-byte granularity — the worst case for a demux that
        # mistakenly concatenates by arrival order.
        for off in range(0, 60, 8):
            for node, blob in payloads.items():
                suit.limbs[node].send_xrce_bytes(blob[off:off + 8])

        assert await until(
            lambda: all(
                len(suit.bridge.xrce_streams.get(n, b"")) >= len(p)
                for n, p in payloads.items()
            ),
            timeout=2.0,
        ), {n: len(v) for n, v in suit.bridge.xrce_streams.items()}

        for node, blob in payloads.items():
            assert bytes(suit.bridge.xrce_streams[node]) == blob, f"stream {node} corrupted"


async def test_xrce_never_crosses_between_buses():
    async with SuitHarness() as suit:
        # An arm is on bus 1, a leg on bus 2. Neither may see the other's XRCE
        # traffic; only the SPI link does (§5 routing policy).
        seen: list[int] = []
        suit.bus2.attach(31, lambda f: seen.append(f.id))

        suit.limbs[Node.ARM_R].send_xrce_bytes(b"bus1-only")
        assert await until(lambda: Node.ARM_R in suit.bridge.xrce_streams, timeout=1.0)

        from powersuit_proto import can_id
        from powersuit_proto.can_id import Cls

        assert not [i for i in seen if can_id.unpack(i).cls == Cls.XRCE], \
            "XRCE traffic crossed to the other bus"
