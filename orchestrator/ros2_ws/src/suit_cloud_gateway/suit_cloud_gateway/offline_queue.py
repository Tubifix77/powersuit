"""Bounded offline store-and-forward queue — PURE python.

Holds advisories/queries while the link is down; drains in FIFO order on resume.
Bounded at 512 entries with drop-oldest (fresh context beats stale backlog).
"""

from __future__ import annotations

from collections import deque
from typing import Any

DEFAULT_MAXLEN = 512


class OfflineQueue:
    def __init__(self, maxlen: int = DEFAULT_MAXLEN):
        if maxlen < 1:
            raise ValueError("maxlen must be >= 1")
        self._q: deque[Any] = deque(maxlen=maxlen)
        self._dropped = 0

    def put(self, item: Any) -> None:
        if len(self._q) == self._q.maxlen:
            self._dropped += 1  # deque(maxlen) evicts the oldest on append
        self._q.append(item)

    def drain(self) -> list[Any]:
        """All queued items, oldest first; the queue is left empty."""
        items = list(self._q)
        self._q.clear()
        return items

    def peek_all(self) -> list[Any]:
        return list(self._q)

    def __len__(self) -> int:
        return len(self._q)

    @property
    def maxlen(self) -> int:
        return int(self._q.maxlen or 0)

    @property
    def dropped(self) -> int:
        return self._dropped
