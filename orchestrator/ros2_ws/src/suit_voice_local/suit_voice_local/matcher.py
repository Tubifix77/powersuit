"""Intent matcher — PURE python (no rclpy; natively tested).

Grammar comes from config/grammar.yaml: per intent a list of keyword patterns
(all keywords must be present in the normalized utterance) plus optional typed
slots. Longest matching pattern wins; ties resolve to the first-declared intent.
Unmatched text falls through to the intent marked `fallback: true` (cloud_query).
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import yaml

FALLBACK_CONFIDENCE = 0.2

_NORM_RE = re.compile(r"[^a-z0-9. ]+")
_NUM_RE = re.compile(r"^\d+(\.\d+)?$")


def normalize(text: str) -> list[str]:
    """Lowercase, strip punctuation to spaces, collapse whitespace, tokenize."""
    lowered = _NORM_RE.sub(" ", text.lower().replace("-", " "))
    return [s for s in (t.strip(".") for t in lowered.split()) if s]


def _extract_number(tokens: list[str]) -> float | None:
    for t in tokens:
        if _NUM_RE.match(t):
            return float(t)
    return None


def load_grammar(path: str | Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh)
    if not isinstance(doc, dict) or "intents" not in doc:
        raise ValueError(f"grammar file {path} missing 'intents' table")
    return doc


def default_grammar_path() -> Path:
    """Installed share/ path when available, else the source-tree config/."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("suit_voice_local"))
        cand = share / "config" / "grammar.yaml"
        if cand.is_file():
            return cand
    except Exception:
        pass
    return Path(__file__).resolve().parents[1] / "config" / "grammar.yaml"


class Matcher:
    def __init__(self, grammar: dict[str, Any]):
        self._intents: dict[str, dict] = grammar["intents"]
        self._fallback = next(
            (name for name, spec in self._intents.items()
             if isinstance(spec, dict) and spec.get("fallback")),
            None,
        )

    @classmethod
    def from_file(cls, path: str | Path) -> "Matcher":
        return cls(load_grammar(path))

    def match(self, text: str) -> tuple[str | None, dict[str, Any], float]:
        """Returns (intent, slots, confidence). intent is None for empty input."""
        tokens = normalize(text)
        if not tokens:
            return (None, {}, 0.0)
        token_set = set(tokens)

        best_intent: str | None = None
        best_len = 0
        for name, spec in self._intents.items():
            if not isinstance(spec, dict):
                continue
            for pattern in spec.get("patterns") or []:
                words = [str(w).lower() for w in pattern]
                if words and all(w in token_set for w in words):
                    if len(words) > best_len:
                        best_len = len(words)
                        best_intent = name

        if best_intent is None:
            if self._fallback is None:
                return (None, {"text": text}, 0.0)
            return (self._fallback, {"text": text}, FALLBACK_CONFIDENCE)

        slots: dict[str, Any] = {}
        for slot_name, slot_type in (self._intents[best_intent].get("slots") or {}).items():
            if slot_type == "number":
                value = _extract_number(tokens)
                if value is not None:
                    slots[slot_name] = value
        # Exact commands score 1.0; keyword hits embedded in chatter score lower,
        # never below 0.5 (the pattern did fully match).
        confidence = 0.5 + 0.5 * min(1.0, best_len / len(tokens))
        return (best_intent, slots, confidence)
