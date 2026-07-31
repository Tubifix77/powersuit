"""Virtual CAN bus for tools/suit_sim.

NOTE — arbitration is NOT simulated. Real CAN arbitrates by identifier
priority when multiple nodes contend for the bus at the same instant; this
bus has no notion of contention at all: every `send()` is delivered
synchronously, in call order, to every other attached node. That is faithful
enough to prove protocol/routing/safety correctness (the thing this harness
exists for) but says nothing about bus loading or worst-case latency under
real contention — see docs/network-map.md §10 for the real budget analysis.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from powersuit_proto import can_id

FrameCallback = Callable[["CanFrame"], None]


@dataclass
class CanFrame:
    """One classic-CAN extended frame: 29-bit `id` (see `powersuit_proto.can_id`)
    plus up to 8 bytes of payload."""

    id: int
    data: bytes = field(default=b"")


class VirtualBus:
    """A named virtual CAN segment. Nodes attach with their node id and a
    synchronous callback; `send()` fans a frame out to every attached node
    other than the frame's own source (decoded from the CAN id itself, so
    callers never have to pass the sender separately)."""

    def __init__(self, name: str = "bus") -> None:
        self.name = name
        self._nodes: dict[int, FrameCallback] = {}

    def attach(self, node_id: int, callback: FrameCallback) -> None:
        self._nodes[node_id] = callback

    def detach(self, node_id: int) -> None:
        self._nodes.pop(node_id, None)

    def send(self, frame: CanFrame) -> None:
        src = can_id.unpack(frame.id).src
        for node_id, callback in list(self._nodes.items()):
            if node_id == src:
                continue
            callback(frame)
