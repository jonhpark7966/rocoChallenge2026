from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Dict, Optional, Tuple


HAND_IDS = ("L", "R")


@dataclass(frozen=True)
class VpHand:
    tracked: bool
    wrist_p: Optional[Tuple[float, float, float]]
    wrist_q: Optional[Tuple[float, float, float, float]]
    pinch: Optional[float]


@dataclass(frozen=True)
class VpHandsFrame:
    seq: int
    t: float
    hands: Dict[str, VpHand]
    raw: Dict


def parse_vp_hands_json(payload: str | bytes) -> Optional[VpHandsFrame]:
    try:
        if isinstance(payload, bytes):
            payload = payload.decode("utf-8")
        msg = json.loads(payload)
    except Exception:
        return None
    return parse_vp_hands(msg)


def parse_vp_hands(msg: Dict) -> Optional[VpHandsFrame]:
    if msg.get("v") != 1 or msg.get("type") != "vp_hands":
        return None
    try:
        seq = int(msg["seq"])
        ts = float(msg["t"])
    except (KeyError, TypeError, ValueError):
        return None
    hands_raw = msg.get("hands")
    if not isinstance(hands_raw, dict):
        return None
    parsed: Dict[str, VpHand] = {}
    for hand_id in HAND_IDS:
        if hand_id not in hands_raw:
            continue
        hand = _parse_hand(hands_raw.get(hand_id))
        if hand:
            parsed[hand_id] = hand
    if not parsed:
        return None
    return VpHandsFrame(seq=seq, t=ts, hands=parsed, raw=msg)


def _parse_hand(hand_raw: Dict) -> Optional[VpHand]:
    if not isinstance(hand_raw, dict):
        return None
    if "tracked" not in hand_raw:
        return None
    tracked = bool(hand_raw["tracked"])

    wrist_p = _parse_vec(hand_raw.get("wrist_p"), 3)
    wrist_q = _parse_vec(hand_raw.get("wrist_q"), 4)

    pinch = None
    if "pinch" in hand_raw and hand_raw["pinch"] is not None:
        try:
            pinch = float(hand_raw["pinch"])
        except (TypeError, ValueError):
            return None
        pinch = max(0.0, min(1.0, pinch))

    return VpHand(tracked=tracked, wrist_p=wrist_p, wrist_q=wrist_q, pinch=pinch)


def _parse_vec(value: object, length: int) -> Optional[Tuple[float, ...]]:
    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != length:
        return None
    try:
        return tuple(float(x) for x in value)
    except (TypeError, ValueError):
        return None
