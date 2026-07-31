"""suit_sim — end-to-end integration simulator for the Powersuit stack.

Fake limb/helmet/flight nodes on virtual CAN buses, a fake chest hub speaking
the real 512-byte SPI framing, a Node-8 bridge stand-in, and the real Node 9
`cloud_ai_core` server running in-process. Every wire format is imported from
`powersuit_proto` — nothing here re-implements protocol logic.
"""

from __future__ import annotations

__version__ = "0.1.0"
