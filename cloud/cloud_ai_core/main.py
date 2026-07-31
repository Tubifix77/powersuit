"""CLI entrypoint: `cloud-ai-core` (see [project.scripts])."""

from __future__ import annotations

import argparse
import asyncio
import logging

from .config import Settings
from .gateway.server import CloudServer


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(prog="cloud-ai-core", description="Powersuit Node 9 cloud AI core")
    ap.add_argument("--host", help="bind host (overrides CLOUD_BIND)")
    ap.add_argument("--port", type=int, help="bind port (overrides CLOUD_BIND)")
    ap.add_argument("--engine", choices=["mock", "openai"], help="LLM engine (overrides ENGINE)")
    ap.add_argument("--model", help="model name (overrides OPENAI_MODEL)")
    ap.add_argument("--base-url", help="OpenAI-compatible endpoint (overrides OPENAI_BASE_URL)")
    ap.add_argument("--tts", choices=["mock", "piper"], help="TTS backend (overrides TTS)")
    ap.add_argument("--docs", help="RAG manuals dir (overrides RAG_DOCS_DIR)")
    ap.add_argument("--token", action="append", default=None,
                    help="allowed bearer token (repeatable; overrides CLOUD_LINK_TOKENS)")
    ap.add_argument("-v", "--verbose", action="store_true")
    return ap.parse_args(argv)


def settings_from_args(args: argparse.Namespace) -> Settings:
    s = Settings.from_env()
    if args.host:
        s.host = args.host
    if args.port is not None:
        s.port = args.port
    if args.engine:
        s.engine = args.engine
    if args.model:
        s.openai_model = args.model
    if args.base_url:
        s.openai_base_url = args.base_url
    if args.tts:
        s.tts = args.tts
    if args.docs:
        s.rag_docs_dir = args.docs
    if args.token:
        s.tokens = tuple(args.token)
    return s


async def _run(settings: Settings) -> None:
    server = CloudServer(settings)
    await server.serve_forever()


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    settings = settings_from_args(args)
    if not settings.tokens:
        logging.warning("no CLOUD_LINK_TOKENS configured; all connections will be rejected")
    try:
        asyncio.run(_run(settings))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
