from __future__ import annotations

import time
from typing import Dict, Iterable, List, Optional


def build_ee_targets(
    arms: Iterable[Dict],
    frame: str = "world",
    precision: int = 0,
    seq: Optional[int] = None,
    t: Optional[float] = None,
) -> Dict:
    return {
        "v": 1,
        "type": "ee_targets",
        "seq": int(seq if seq is not None else time.time_ns()),
        "t": float(t if t is not None else time.time()),
        "frame": frame,
        "precision": int(precision),
        "arms": list(arms),
    }
