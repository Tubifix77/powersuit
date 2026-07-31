"""Run the simulated suit as a standalone server so the REAL Node 8 stack can
drive it.

The scenario tests use `bridge_lite`, a Python stand-in for the orchestrator.
This entrypoint leaves that out and exposes the hub's SPI socket instead, so the
actual C++ `suit_canspi_bridge` — with its mock transport pointed here — talks to
simulated limbs over the real 512-byte framing:

    python -m suit_sim.serve --port 9700
    ros2 launch suit_bringup sim.launch.py mock_port:=9700

That combination exercises everything except the silicon: real ROS graph, real
bridge, real framing, real protocol codecs, simulated hardware.

    --cloud     also run the real Node 9 service in-process
    --soc N     start the pack at N% (try 9 to watch an advisory fire)
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging

from powersuit_proto import wire

from .harness import SuitHarness

LOG = logging.getLogger("suit_sim")


async def _run(args: argparse.Namespace) -> None:
    async with SuitHarness(
        with_cloud=args.cloud, with_bridge=False, hub_port=args.port
    ) as suit:
        suit.hub.soc_pct = args.soc

        LOG.info("hub SPI socket listening on 127.0.0.1:%d", suit.hub.port)
        if args.cloud:
            LOG.info("cloud (Node 9) on ws://127.0.0.1:%d, token 'dev-token'",
                     suit.cloud_port)
        LOG.info("limbs: %s", ", ".join(str(n) for n in suit.limbs))
        LOG.info("waiting for a bridge to connect — nothing actuates until it "
                 "starts beating (docs/safety.md §2)")

        last_state: dict[int, int] = {}
        while True:
            await asyncio.sleep(1.0)
            states = {n: limb.state for n, limb in suit.limbs.items()}
            if states != last_state:
                LOG.info("limb states: %s",
                         {n: wire.State(s).name for n, s in states.items()})
                last_state = states


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="suit_sim.serve", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=9700,
                        help="TCP port for the hub's SPI socket (default 9700)")
    parser.add_argument("--cloud", action="store_true",
                        help="also run the real Node 9 service in-process")
    parser.add_argument("--soc", type=int, default=84,
                        help="initial pack state of charge in %% (default 84)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(_run(args))


if __name__ == "__main__":
    main()
