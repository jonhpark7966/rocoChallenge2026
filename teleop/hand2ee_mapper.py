from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

from .math_utils import quat_multiply, quat_normalize, quat_slerp
from .vp_protocol import VpHand, VpHandsFrame


ARM_IDS = ("L", "R")


@dataclass
class ArmAnchor:
    p: Tuple[float, float, float]
    q: Tuple[float, float, float, float]
    grip: float


@dataclass
class HandAnchor:
    p: Tuple[float, float, float]
    q: Optional[Tuple[float, float, float, float]]


class HandToEEMapper:
    def __init__(
        self,
        pos_scale: float = 1.0,
        axis_map: Optional[Tuple[Tuple[float, float, float], ...]] = None,
        ee_frame_map: Optional[Dict[str, str]] = None,
        use_orientation: bool = False,
        rot_scale: float = 1.0,
        q_map: Optional[Tuple[float, float, float, float]] = None,
        grip_lpf_alpha: float = 0.0,
    ) -> None:
        self.pos_scale = float(pos_scale)
        self.axis_map = axis_map or identity_axis_map()
        self.ee_frame_map = ee_frame_map or {"L": "left_gripper_tcp", "R": "right_gripper_tcp"}
        self.use_orientation = bool(use_orientation)
        self.rot_scale = float(rot_scale)
        self.q_map = q_map or (1.0, 0.0, 0.0, 0.0)
        self.grip_lpf_alpha = max(0.0, min(1.0, float(grip_lpf_alpha)))

        self._robot_anchor: Dict[str, ArmAnchor] = {}
        self._hand_anchor: Dict[str, HandAnchor] = {}
        self._last_grip: Dict[str, float] = {}

    def reset_hand_anchors(self) -> None:
        self._hand_anchor = {}

    def get_robot_anchor_snapshot(self) -> Dict[str, Dict]:
        snapshot: Dict[str, Dict] = {}
        for arm_id, anchor in self._robot_anchor.items():
            snapshot[arm_id] = {
                "p": list(anchor.p),
                "q": list(anchor.q),
                "grip": anchor.grip,
            }
        return snapshot

    def get_hand_anchor_snapshot(self) -> Dict[str, Dict]:
        snapshot: Dict[str, Dict] = {}
        for arm_id, anchor in self._hand_anchor.items():
            snapshot[arm_id] = {
                "p": list(anchor.p),
                "q": list(anchor.q) if anchor.q else None,
            }
        return snapshot

    def set_robot_anchor(self, arm_id: str, p: Iterable[float], q: Iterable[float], grip: float) -> None:
        if arm_id not in ARM_IDS:
            return
        p_t = tuple(float(x) for x in p)
        q_t = tuple(float(x) for x in q)
        if len(p_t) != 3 or len(q_t) != 4:
            return
        self._robot_anchor[arm_id] = ArmAnchor(p=p_t, q=quat_normalize(q_t), grip=float(grip))
        self._last_grip[arm_id] = float(grip)

    def update_robot_anchor_from_state(self, ee_state: Dict) -> None:
        for arm in ee_state.get("arms", []):
            if not isinstance(arm, dict):
                continue
            arm_id = arm.get("id")
            if arm_id not in ARM_IDS:
                continue
            p = arm.get("p")
            q = arm.get("q")
            grip = arm.get("grip", 1.0)
            if p is None or q is None:
                continue
            self.set_robot_anchor(arm_id, p, q, grip)

    def map_frame(self, frame: VpHandsFrame) -> List[Dict]:
        arms: List[Dict] = []
        for arm_id, hand in frame.hands.items():
            target = self.map_hand(arm_id, hand)
            if target:
                arms.append(target)
        return arms

    def map_hand(self, arm_id: str, hand: VpHand) -> Optional[Dict]:
        target, _ = self._map_hand(arm_id, hand, debug=False)
        return target

    def map_frame_debug(self, frame: VpHandsFrame) -> Tuple[List[Dict], List[Dict]]:
        arms: List[Dict] = []
        debug: List[Dict] = []
        for arm_id, hand in frame.hands.items():
            target, info = self._map_hand(arm_id, hand, debug=True)
            if info:
                debug.append(info)
            if target:
                arms.append(target)
        return arms, debug

    def map_hand_debug(self, arm_id: str, hand: VpHand) -> Tuple[Optional[Dict], Optional[Dict]]:
        return self._map_hand(arm_id, hand, debug=True)

    def _map_hand(self, arm_id: str, hand: VpHand, debug: bool) -> Tuple[Optional[Dict], Optional[Dict]]:
        debug_info: Optional[Dict] = None
        if debug:
            debug_info = {
                "arm": arm_id,
                "tracked": hand.tracked,
                "wrist_p": list(hand.wrist_p) if hand.wrist_p else None,
                "wrist_q": list(hand.wrist_q) if hand.wrist_q else None,
                "pinch": hand.pinch,
                "pos_scale": self.pos_scale,
                "axis_map": _as_list(self.axis_map),
                "use_orientation": self.use_orientation,
                "rot_scale": self.rot_scale,
                "q_map": list(self.q_map),
            }

        if arm_id not in ARM_IDS:
            if debug_info is not None:
                debug_info.update({"status": "skip", "reason": "invalid_arm"})
            return None, debug_info
        if not hand.tracked or hand.wrist_p is None:
            if debug_info is not None:
                debug_info.update({"status": "skip", "reason": "not_tracked_or_missing_wrist"})
            return None, debug_info
        if arm_id not in self._robot_anchor:
            if debug_info is not None:
                debug_info.update({"status": "skip", "reason": "missing_robot_anchor"})
            return None, debug_info

        if arm_id not in self._hand_anchor:
            self._hand_anchor[arm_id] = HandAnchor(p=hand.wrist_p, q=hand.wrist_q)
            if debug_info is not None:
                debug_info.update(
                    {
                        "status": "anchor_set",
                        "hand_anchor": {"p": list(hand.wrist_p), "q": list(hand.wrist_q) if hand.wrist_q else None},
                    }
                )
            return None, debug_info

        hand_anchor = self._hand_anchor[arm_id]
        robot_anchor = self._robot_anchor[arm_id]

        dp = (
            hand.wrist_p[0] - hand_anchor.p[0],
            hand.wrist_p[1] - hand_anchor.p[1],
            hand.wrist_p[2] - hand_anchor.p[2],
        )
        dp_sim = apply_axis_map(dp, self.axis_map)
        p_target = (
            robot_anchor.p[0] + self.pos_scale * dp_sim[0],
            robot_anchor.p[1] + self.pos_scale * dp_sim[1],
            robot_anchor.p[2] + self.pos_scale * dp_sim[2],
        )

        q_target = robot_anchor.q
        if self.use_orientation and hand.wrist_q and hand_anchor.q:
            dq_vp = quat_multiply(hand.wrist_q, quat_inverse(hand_anchor.q))
            if abs(self.rot_scale - 1.0) > 1e-6:
                dq_vp = quat_slerp((1.0, 0.0, 0.0, 0.0), dq_vp, self.rot_scale)
            dq_sim = quat_multiply(quat_multiply(self.q_map, dq_vp), quat_inverse(self.q_map))
            q_target = quat_multiply(dq_sim, robot_anchor.q)

        grip = self._compute_grip(arm_id, hand)
        target = {
            "id": arm_id,
            "ee_frame": self.ee_frame_map.get(arm_id, ""),
            "p": list(p_target),
            "q": list(quat_normalize(q_target)),
            "grip": grip,
        }
        if debug_info is not None:
            debug_info.update(
                {
                    "status": "mapped",
                    "hand_anchor": {"p": list(hand_anchor.p), "q": list(hand_anchor.q) if hand_anchor.q else None},
                    "robot_anchor": {"p": list(robot_anchor.p), "q": list(robot_anchor.q), "grip": robot_anchor.grip},
                    "dp_vp": list(dp),
                    "dp_sim": list(dp_sim),
                    "p_target": list(p_target),
                    "q_target": list(quat_normalize(q_target)),
                    "grip": grip,
                }
            )
        return target, debug_info

    def _compute_grip(self, arm_id: str, hand: VpHand) -> float:
        if hand.pinch is None:
            grip = self._last_grip.get(arm_id, 1.0)
        else:
            grip = 1.0 - float(hand.pinch)
        grip = max(0.0, min(1.0, grip))
        if self.grip_lpf_alpha > 0.0:
            prev = self._last_grip.get(arm_id, grip)
            grip = prev + self.grip_lpf_alpha * (grip - prev)
        self._last_grip[arm_id] = grip
        return grip


def identity_axis_map() -> Tuple[Tuple[float, float, float], ...]:
    return (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )


def apply_axis_map(
    vec: Tuple[float, float, float],
    axis_map: Tuple[Tuple[float, float, float], ...],
) -> Tuple[float, float, float]:
    return (
        axis_map[0][0] * vec[0] + axis_map[0][1] * vec[1] + axis_map[0][2] * vec[2],
        axis_map[1][0] * vec[0] + axis_map[1][1] * vec[1] + axis_map[1][2] * vec[2],
        axis_map[2][0] * vec[0] + axis_map[2][1] * vec[1] + axis_map[2][2] * vec[2],
    )


def parse_axis_map(text: str) -> Tuple[Tuple[float, float, float], ...]:
    lowered = text.strip().lower()
    if lowered in ("identity", "eye", "i"):
        return identity_axis_map()
    rows = [row.strip() for row in text.split(";") if row.strip()]
    if len(rows) != 3:
        raise ValueError("axis-map must have 3 rows separated by ';'")
    matrix = []
    for row in rows:
        cols = [col.strip() for col in row.split(",") if col.strip()]
        if len(cols) != 3:
            raise ValueError("axis-map rows must have 3 comma-separated values")
        matrix.append(tuple(float(x) for x in cols))
    return tuple(matrix)  # type: ignore[return-value]


def quat_inverse(q: Iterable[float]) -> Tuple[float, float, float, float]:
    q_t = tuple(float(x) for x in q)
    if len(q_t) != 4:
        return (1.0, 0.0, 0.0, 0.0)
    w, x, y, z = q_t
    norm = w * w + x * x + y * y + z * z
    if norm < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, -x / norm, -y / norm, -z / norm)


def _as_list(value: object) -> object:
    if isinstance(value, tuple):
        return [_as_list(item) for item in value]
    return value
