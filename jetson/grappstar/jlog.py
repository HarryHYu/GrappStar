"""JSONL mission log: one JSON object per line, every line has t (time.time()) and kind.

Kinds: step, info, warn, error, prompt, answer, base_tx, base_rx (telemetry, events, replies), dart_tx, dart (status
polls), result, file. A run folder holds mission.jsonl plus the recordings and analysis outputs, so a mission can be
replayed later with `python3 -m grappstar replay <folder>`.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Any, Iterator, Optional


def _clean(o: Any) -> Any:
    """Make numpy values, NaN and tuples JSON safe."""
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if hasattr(o, "item") and callable(getattr(o, "item")):
        try:
            return _clean(o.item())
        except Exception:
            return str(o)
    if hasattr(o, "tolist"):
        return _clean(o.tolist())
    if isinstance(o, (str, int, bool)) or o is None:
        return o
    return str(o)


class JLog:
    def __init__(self, path: Optional[str] = None, echo=None, tel_every: int = 1):
        self.path = path
        self.echo = echo            # callable(str) for human lines (print or a logger)
        self.lock = threading.Lock()
        self.f = None
        self.tail: list = []        # last human lines for the UI
        self.tel_every = max(1, tel_every)
        self._tel_n = 0
        if path:
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            self.f = open(path, "a", buffering=1)

    def write(self, kind: str, **kw: Any) -> None:
        if kind == "base_rx" and "tel" in kw.get("obj", {}):
            self._tel_n += 1
            if self._tel_n % self.tel_every:
                return
        rec = {"t": round(time.time(), 3), "kind": kind}
        rec.update(_clean(kw))
        line = json.dumps(rec, separators=(",", ":"))
        with self.lock:
            if self.f:
                self.f.write(line + "\n")
            if kind in ("step", "info", "warn", "error", "prompt", "answer", "result"):
                msg = kw.get("msg") or kw.get("name") or kw.get("text") or ""
                human = f"{time.strftime('%H:%M:%S')} {kind.upper():6} {msg}"
                self.tail.append(human)
                del self.tail[:-200]
                if self.echo:
                    try:
                        self.echo(human)
                    except Exception:
                        pass

    def info(self, msg: str, **kw) -> None: self.write("info", msg=msg, **kw)
    def warn(self, msg: str, **kw) -> None: self.write("warn", msg=msg, **kw)
    def error(self, msg: str, **kw) -> None: self.write("error", msg=msg, **kw)

    def base_line(self, direction: str, obj: dict) -> None:
        self.write("base_tx" if direction == "tx" else "base_rx", obj=obj)

    def dart_line(self, kind: str, obj: dict) -> None:
        self.write(kind, obj=obj)

    def close(self) -> None:
        with self.lock:
            if self.f:
                self.f.close()
                self.f = None


def read(path: str) -> Iterator[dict]:
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    yield json.loads(line)
                except ValueError:
                    continue
