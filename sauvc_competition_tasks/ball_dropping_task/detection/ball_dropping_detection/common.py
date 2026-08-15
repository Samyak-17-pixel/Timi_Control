from __future__ import annotations

import json
from typing import Any


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def status_json(**kwargs: Any) -> str:
    return json.dumps(kwargs, separators=(",", ":"), sort_keys=True)

