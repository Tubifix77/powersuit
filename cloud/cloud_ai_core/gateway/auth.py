"""Bearer-token auth (docs/link-protocol.md §4) and session id generation."""

from __future__ import annotations

import hmac
import secrets
from collections.abc import Iterable


def check_token(presented: object, valid_tokens: Iterable[str]) -> bool:
    """Constant-time comparison against every configured token (no early exit)."""
    if not isinstance(presented, str) or not presented:
        return False
    raw = presented.encode()
    ok = False
    for token in valid_tokens:
        ok |= hmac.compare_digest(raw, token.encode())
    return ok


def new_session_id() -> str:
    return secrets.token_hex(16)
