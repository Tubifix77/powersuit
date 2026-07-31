"""Piper TTS backend via subprocess (`piper --model … --output_raw`).

Optional: guarded so that a missing binary/model raises TtsUnavailable at
construction/synthesis time instead of crashing the server.
"""

from __future__ import annotations

import asyncio
import shutil
from pathlib import Path

from .base import TtsUnavailable

# piper --output_raw emits s16le mono at the voice's native rate; the common
# medium voices are 22 050 Hz. Override via constructor when using another voice.
DEFAULT_PIPER_RATE = 22_050


class PiperTts:
    def __init__(self, model_path: str, exe: str = "piper", sample_rate: int = DEFAULT_PIPER_RATE):
        self.model_path = model_path
        self.exe = exe
        self.sample_rate = sample_rate
        if shutil.which(exe) is None:
            raise TtsUnavailable(f"piper executable {exe!r} not found on PATH")
        if not Path(model_path).exists():
            raise TtsUnavailable(f"piper model {model_path!r} does not exist")

    async def synth(self, text: str) -> tuple[bytes, int]:
        try:
            proc = await asyncio.create_subprocess_exec(
                self.exe,
                "--model",
                self.model_path,
                "--output_raw",
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
        except OSError as exc:
            raise TtsUnavailable(f"failed to launch piper: {exc}") from exc
        stdout, stderr = await proc.communicate(text.encode("utf-8"))
        if proc.returncode != 0:
            raise TtsUnavailable(f"piper exited {proc.returncode}: {stderr.decode(errors='replace')[:200]}")
        if len(stdout) < 2:
            raise TtsUnavailable("piper produced no audio")
        return stdout, self.sample_rate
