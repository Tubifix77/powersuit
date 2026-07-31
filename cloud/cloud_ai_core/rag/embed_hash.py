"""Pure hashing embedder: text -> l2-normalized float32[256].

Features are whole tokens plus character 3-grams of each padded token, hashed
with zlib.crc32 (stable across runs and processes — never the built-in hash(),
which is salted per process).
"""

from __future__ import annotations

import re
import zlib

import numpy as np

DIM = 256
_TOKEN_RE = re.compile(r"[a-z0-9]+")


def _tokens(text: str) -> list[str]:
    return _TOKEN_RE.findall(text.lower())


def embed(text: str) -> np.ndarray:
    vec = np.zeros(DIM, dtype=np.float32)
    for tok in _tokens(text):
        # Whole-token feature (strong exact-word signal).
        h = zlib.crc32(b"w:" + tok.encode())
        vec[h % DIM] += 2.0
        # Character 3-grams of the padded token (typo/morphology tolerance).
        padded = f"#{tok}#"
        for i in range(len(padded) - 2):
            g = zlib.crc32(b"g:" + padded[i : i + 3].encode())
            vec[g % DIM] += 1.0
    norm = float(np.linalg.norm(vec))
    if norm > 0.0:
        vec /= norm
    return vec
