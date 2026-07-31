"""OpenAI-compatible chat.completions client: vLLM, Ollama, LM Studio, ...

Only the common subset is used (model, messages, max_tokens, temperature) so any
/v1-compatible endpoint works. Connection-level failures raise EngineUnavailable
so the strategy can degrade to a 'cloud reasoning unavailable' advisory.
"""

from __future__ import annotations

import httpx

from .engine import EngineError, EngineUnavailable


class OpenAiCompatEngine:
    def __init__(
        self,
        base_url: str,
        model: str,
        api_key: str | None = None,
        timeout_s: float = 60.0,
        max_tokens: int = 400,
        temperature: float = 0.4,
    ):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.api_key = api_key
        self.timeout_s = timeout_s
        self.max_tokens = max_tokens
        self.temperature = temperature

    async def generate(self, system: str, user: str, context_docs: list[str]) -> str:
        content = user
        if context_docs:
            joined = "\n\n---\n\n".join(context_docs)
            content = f"{user}\n\nRelevant manual excerpts:\n{joined}"
        headers = {}
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"
        body = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": content},
            ],
            "max_tokens": self.max_tokens,
            "temperature": self.temperature,
        }
        try:
            async with httpx.AsyncClient(timeout=self.timeout_s) as client:
                resp = await client.post(
                    f"{self.base_url}/chat/completions", json=body, headers=headers
                )
        except (httpx.ConnectError, httpx.ConnectTimeout, httpx.NetworkError) as exc:
            raise EngineUnavailable(f"engine endpoint unreachable: {exc}") from exc
        if resp.status_code != 200:
            raise EngineError(f"engine HTTP {resp.status_code}: {resp.text[:200]}")
        try:
            return resp.json()["choices"][0]["message"]["content"]
        except (KeyError, IndexError, ValueError) as exc:
            raise EngineError(f"malformed engine response: {exc}") from exc
