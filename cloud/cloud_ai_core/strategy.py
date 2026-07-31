"""Voice query pipeline (link-protocol §7):

RAG retrieve -> engine.generate -> advisory(in_reply_to) -> tts_meta start ->
synth -> resample 8 kHz -> IMA ADPCM (fresh state per stream) -> credit-gated
320-byte TTS_CHUNK binary frames -> tts_meta end.
"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np

from powersuit_proto import adpcm

from .gateway.session import Session
from .inference.engine import EngineBase, EngineUnavailable
from .protocol import (
    BTYPE_TTS_CHUNK,
    CODEC_ADPCM_8K,
    T_ADVISORY,
    T_TTS_META,
    TTS_CHUNK_BYTES,
    pack_audio_frame,
)
from .rag.store import VectorStore
from .tts.base import TtsBase, TtsUnavailable
from .tts.resample import resample_pcm16

log = logging.getLogger(__name__)

SYSTEM_PROMPT = (
    "You are the Powersuit cloud advisor (Node 9). Answer briefly and factually "
    "from the manual excerpts and telemetry. You can only advise; you cannot "
    "actuate the suit."
)


def _encode_stream(pcm8k: bytes) -> bytes:
    """ADPCM-encode a whole stream with fresh codec state, padded so the byte
    count is a multiple of 8 (16-sample blocks — keeps every chunk 8-aligned)."""
    samples = np.frombuffer(pcm8k, dtype="<i2").tolist()
    pad = (-len(samples)) % 16
    if pad:
        samples.extend([0] * pad)
    return adpcm.encode(adpcm.AdpcmState(), samples)


def chunk_adpcm(data: bytes, chunk_bytes: int = TTS_CHUNK_BYTES) -> list[bytes]:
    if chunk_bytes % 8 != 0:
        raise ValueError("chunk size must be a multiple of 8 ADPCM bytes")
    return [data[i : i + chunk_bytes] for i in range(0, len(data), chunk_bytes)]


async def handle_voice_query(
    session: Session,
    payload: dict[str, Any],
    engine: EngineBase,
    store: VectorStore,
    tts: TtsBase,
) -> None:
    query_id = str(payload.get("query_id", ""))
    text = str(payload.get("text", "") or "")
    context = payload.get("context") if isinstance(payload.get("context"), dict) else {}

    hits = store.search(text, k=3) if text else []
    context_docs = [f"{h.title}\n{h.text}" for h in hits]

    facts = session.telemetry.facts()
    if context:
        ctx_str = " ".join(f"{k}={v}" for k, v in sorted(context.items()))
        facts = f"{facts} {ctx_str}".strip()
    user_prompt = f"Query: {text}\nTelemetry: {facts}"

    severity = "info"
    try:
        answer = await engine.generate(SYSTEM_PROMPT, user_prompt, context_docs)
    except EngineUnavailable as exc:
        log.warning("engine unavailable for %s: %s", query_id, exc)
        answer = "Cloud reasoning unavailable. Local voice control remains active."
        severity = "notice"

    await session.send(
        T_ADVISORY,
        {
            "advisory_id": f"a-{query_id}" if query_id else "a-unsolicited",
            "severity": severity,
            "title": "Voice response",
            "body": answer,
            "in_reply_to": query_id,
        },
    )

    # --- TTS stream -----------------------------------------------------------
    stream_id = session.alloc_stream()
    await session.send(
        T_TTS_META,
        {"stream_id": stream_id, "query_id": query_id, "state": "start",
         "codec": "adpcm8k", "text": answer},
    )
    try:
        pcm, rate = await tts.synth(answer)
    except TtsUnavailable as exc:
        log.warning("tts unavailable for %s: %s", query_id, exc)
        await session.send(
            T_TTS_META,
            {"stream_id": stream_id, "query_id": query_id, "state": "abort",
             "codec": "adpcm8k", "text": ""},
        )
        session.close_stream(stream_id)
        return

    encoded = _encode_stream(resample_pcm16(pcm, rate, 8000))
    state = "end"
    try:
        for seq, chunk in enumerate(chunk_adpcm(encoded)):
            await session.consume_credit(stream_id)
            if session.ws is None:
                state = "abort"  # audio is never replayed (§1); drop the stream
                break
            await session.send_binary(
                pack_audio_frame(BTYPE_TTS_CHUNK, CODEC_ADPCM_8K, stream_id, seq, chunk)
            )
    except ConnectionError:
        state = "abort"
    finally:
        session.close_stream(stream_id)
    await session.send(
        T_TTS_META,
        {"stream_id": stream_id, "query_id": query_id, "state": state,
         "codec": "adpcm8k", "text": ""},
    )
