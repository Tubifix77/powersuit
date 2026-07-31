"""Deterministic engine used in tests and offline bench runs. NO randomness."""

from __future__ import annotations


class MockEngine:
    """Template answer embedding the query, the top RAG doc title, and any
    telemetry facts present in the user prompt (strategy puts them on a
    'Telemetry:' line). Deterministic by construction."""

    async def generate(self, system: str, user: str, context_docs: list[str]) -> str:
        query = ""
        facts = ""
        for line in user.splitlines():
            if line.startswith("Query:"):
                query = line[len("Query:"):].strip()
            elif line.startswith("Telemetry:"):
                facts = line[len("Telemetry:"):].strip()
        top_title = ""
        if context_docs:
            top_title = context_docs[0].split("\n", 1)[0].strip()
        parts = [f"Acknowledged: {query}." if query else "Acknowledged."]
        if facts:
            parts.append(f"Current telemetry: {facts}.")
        if top_title:
            parts.append(f"Reference: {top_title}.")
        parts.append("All advisory, no actuation.")
        return " ".join(parts)
