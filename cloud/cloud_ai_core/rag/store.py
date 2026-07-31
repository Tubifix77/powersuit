"""In-memory cosine vector store over manual chunks."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .embed_hash import DIM, embed


@dataclass(frozen=True)
class Hit:
    score: float
    doc_id: str
    title: str
    text: str


class VectorStore:
    def __init__(self) -> None:
        self._vecs: list[np.ndarray] = []
        self._meta: list[tuple[str, str, str]] = []  # (doc_id, title, chunk_text)
        self._matrix: np.ndarray | None = None

    def add(self, doc_id: str, title: str, text: str, chunks: list[str]) -> None:
        """Register a document's chunks. `text` is kept only via its chunks."""
        for chunk in chunks:
            self._vecs.append(embed(chunk))
            self._meta.append((doc_id, title, chunk))
        self._matrix = None

    def __len__(self) -> int:
        return len(self._meta)

    def _mat(self) -> np.ndarray:
        if self._matrix is None:
            if not self._vecs:
                self._matrix = np.zeros((0, DIM), dtype=np.float32)
            else:
                self._matrix = np.vstack(self._vecs)
        return self._matrix

    def search(self, query: str, k: int = 3) -> list[Hit]:
        mat = self._mat()
        if mat.shape[0] == 0:
            return []
        q = embed(query)
        scores = mat @ q  # rows and q are l2-normalized -> cosine
        order = np.argsort(-scores, kind="stable")[:k]
        return [
            Hit(score=float(scores[i]), doc_id=self._meta[i][0],
                title=self._meta[i][1], text=self._meta[i][2])
            for i in order
        ]
