"""Manual ingestion: *.md files -> chunked VectorStore."""

from __future__ import annotations

from pathlib import Path

from .store import VectorStore

CHUNK_CHARS = 800
CHUNK_OVERLAP = 120


def chunk_text(text: str, size: int = CHUNK_CHARS, overlap: int = CHUNK_OVERLAP) -> list[str]:
    if size <= overlap:
        raise ValueError("chunk size must exceed overlap")
    chunks: list[str] = []
    start = 0
    while start < len(text):
        piece = text[start : start + size]
        if piece.strip():
            chunks.append(piece)
        if start + size >= len(text):
            break
        start += size - overlap
    return chunks


def _first_heading(text: str, fallback: str) -> str:
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("#"):
            return stripped.lstrip("#").strip()
    return fallback


def ingest_dir(path: str | Path) -> VectorStore:
    store = VectorStore()
    root = Path(path)
    if not root.is_dir():
        return store
    for md in sorted(root.glob("*.md")):
        text = md.read_text(encoding="utf-8")
        title = _first_heading(text, md.stem)
        store.add(md.stem, title, text, chunk_text(text))
    return store
