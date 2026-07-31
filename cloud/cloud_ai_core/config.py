"""Settings for cloud_ai_core, sourced from the environment (12-factor style)."""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

_DEFAULT_DOCS_DIR = Path(__file__).resolve().parent.parent / "docs" / "manuals"


@dataclass(slots=True)
class RuleThresholds:
    """Tunables for telemetry/rules.py (ADVISORY_* env vars)."""

    soc_notice: int = 20
    soc_warning: int = 10
    temp_critical: float = 60.0
    dropped_notice: int = 50


@dataclass(slots=True)
class Settings:
    host: str = "127.0.0.1"
    port: int = 8765
    tokens: tuple[str, ...] = ()
    tls_cert: str | None = None
    tls_key: str | None = None
    engine: str = "mock"  # mock | openai
    openai_base_url: str = "http://localhost:8000/v1"  # vLLM default; Ollama: http://localhost:11434/v1
    openai_model: str = ""
    openai_api_key: str | None = None
    rag_docs_dir: str = str(_DEFAULT_DOCS_DIR)
    tts: str = "mock"  # mock | piper
    piper_model: str | None = None
    rules: RuleThresholds = field(default_factory=RuleThresholds)
    # Link behavior knobs (protocol constants stay in docs/link-protocol.md).
    initial_credits: int = 32
    session_ttl_s: float = 120.0
    hello_timeout_s: float = 5.0
    asr_silence_s: float = 1.5
    heartbeat_s: int = 5

    @classmethod
    def from_env(cls, env: Mapping[str, str] | None = None) -> "Settings":
        env = os.environ if env is None else env
        s = cls()
        bind = env.get("CLOUD_BIND")
        if bind:
            host, _, port = bind.rpartition(":")
            if host:
                s.host = host
            if port:
                s.port = int(port)
        tokens = env.get("CLOUD_LINK_TOKENS")
        if tokens:
            s.tokens = tuple(t.strip() for t in tokens.split(",") if t.strip())
        s.tls_cert = env.get("CLOUD_TLS_CERT") or None
        s.tls_key = env.get("CLOUD_TLS_KEY") or None
        s.engine = env.get("ENGINE", s.engine).lower()
        s.openai_base_url = env.get("OPENAI_BASE_URL", s.openai_base_url)
        s.openai_model = env.get("OPENAI_MODEL", s.openai_model)
        s.openai_api_key = env.get("OPENAI_API_KEY") or None
        s.rag_docs_dir = env.get("RAG_DOCS_DIR", s.rag_docs_dir)
        s.tts = env.get("TTS", s.tts).lower()
        s.piper_model = env.get("PIPER_MODEL") or None
        s.rules = RuleThresholds(
            soc_notice=int(env.get("ADVISORY_SOC_NOTICE", s.rules.soc_notice)),
            soc_warning=int(env.get("ADVISORY_SOC_WARNING", s.rules.soc_warning)),
            temp_critical=float(env.get("ADVISORY_TEMP_CRITICAL", s.rules.temp_critical)),
            dropped_notice=int(env.get("ADVISORY_DROPPED_NOTICE", s.rules.dropped_notice)),
        )
        s.initial_credits = int(env.get("CLOUD_INITIAL_CREDITS", s.initial_credits))
        s.session_ttl_s = float(env.get("CLOUD_SESSION_TTL_S", s.session_ttl_s))
        return s
