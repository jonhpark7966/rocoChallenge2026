from __future__ import annotations

import argparse
import gzip
import json
import socket
import time
import sys
import os
from typing import Iterable, List, Tuple

if __package__ is None and __name__ == "__main__":
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

from teleop.ee_targets_builder import build_ee_targets
from teleop.hand2ee_mapper import HandToEEMapper, parse_axis_map
from teleop.io_udp import request_ee_state, send_udp_json
from teleop.vp_protocol import VpHandsFrame, parse_vp_hands


DEFAULT_ANCHORS = {
    "L": {"p": (0.45, 0.10, 0.85), "q": (1.0, 0.0, 0.0, 0.0), "grip": 1.0},
    "R": {"p": (0.45, -0.10, 0.85), "q": (1.0, 0.0, 0.0, 0.0), "grip": 1.0},
}


def seed_default_anchor(mapper: HandToEEMapper) -> None:
    for arm_id, anchor in DEFAULT_ANCHORS.items():
        mapper.set_robot_anchor(arm_id, anchor["p"], anchor["q"], anchor["grip"])
    print("[INFO] ee_state unavailable; using default anchors for dry-run")


def load_frames(path: str) -> List[Tuple[float, VpHandsFrame]]:
    opener = gzip.open if path.endswith(".gz") else open
    frames: List[Tuple[float, VpHandsFrame]] = []
    with opener(path, "rt", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if record.get("type") != "vp_hands":
                continue
            msg = record.get("msg") if isinstance(record, dict) else None
            if not isinstance(msg, dict):
                continue
            frame = parse_vp_hands(msg)
            if not frame:
                continue
            t_recv = record.get("t_recv", frame.t)
            try:
                t_recv = float(t_recv)
            except (TypeError, ValueError):
                t_recv = frame.t
            frames.append((t_recv, frame))
    return frames


def replay(
    frames: List[Tuple[float, VpHandsFrame]],
    mapper: HandToEEMapper,
    args: argparse.Namespace,
) -> None:
    if not frames:
        print("[WARN] no vp_hands frames in log")
        return

    base_t = frames[0][0]
    normalized = [(t - base_t, frame) for t, frame in frames]

    if args.t0 is not None or args.t1 is not None:
        t0 = args.t0 or 0.0
        t1 = args.t1 if args.t1 is not None else float("inf")
        normalized = [(t, f) for t, f in normalized if t0 <= t <= t1]
        if not normalized:
            print("[WARN] no frames in requested time window")
            return

    udp_sock = None
    if not args.dry_run:
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _run_once() -> None:
        mapper.reset_hand_anchors()
        state = request_ee_state(args.udp_ip, args.state_port, args.state_timeout)
        if state:
            mapper.update_robot_anchor_from_state(state)
            print("[INFO] robot anchor updated from ee_state")
        elif args.dry_run:
            seed_default_anchor(mapper)
        else:
            print("[WARN] ee_state request timed out; mapping disabled until anchor is set")

        prev_t = None
        for t_rel, frame in normalized:
            if prev_t is not None:
                delay = max(0.0, (t_rel - prev_t) / args.speed)
                if delay > 0.0:
                    time.sleep(delay)
            prev_t = t_rel

            arms = mapper.map_frame(frame)
            if not arms:
                continue
            msg = build_ee_targets(arms=arms, frame=args.frame, precision=0)
            if args.dry_run:
                print(f"[DRY] seq={msg['seq']} arms={len(arms)}")
            else:
                send_udp_json(msg, args.udp_ip, args.udp_port, sock=udp_sock)

    try:
        while True:
            _run_once()
            if not args.loop:
                break
    finally:
        if udp_sock:
            udp_sock.close()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Replay vp_hands logs into ee_targets UDP")
    parser.add_argument("--log", required=True, help="Path to JSONL (or .gz) log")
    parser.add_argument("--speed", type=float, default=1.0, help="Replay speed factor")
    parser.add_argument("--t0", type=float, default=None, help="Start offset seconds")
    parser.add_argument("--t1", type=float, default=None, help="End offset seconds")
    parser.add_argument("--loop", action="store_true", help="Loop replay")
    parser.add_argument("--dry-run", action="store_true", help="Do not send UDP")
    parser.add_argument("--udp-ip", default="127.0.0.1", help="IsaacSim UDP target IP")
    parser.add_argument("--udp-port", type=int, default=5005, help="IsaacSim UDP target port")
    parser.add_argument("--state-port", type=int, default=5006, help="ee_state UDP port")
    parser.add_argument("--state-timeout", type=float, default=1.0, help="ee_state timeout seconds")
    parser.add_argument("--frame", default="world", help="ee_targets frame")
    parser.add_argument("--pos-scale", type=float, default=1.0, help="Position scale factor")
    parser.add_argument("--rot-scale", type=float, default=1.0, help="Rotation scale factor")
    parser.add_argument("--axis-map", default="", help="Axis map matrix 'a,b,c;d,e,f;g,h,i'")
    parser.add_argument("--use-orient", dest="use_orientation", action="store_true", help="Enable orientation mapping")
    parser.add_argument("--no-orient", dest="use_orientation", action="store_false", help="Disable orientation mapping")
    parser.set_defaults(use_orientation=False)
    parser.add_argument("--grip-lpf", type=float, default=0.0, help="Grip LPF alpha (0 disables)")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    frames = load_frames(args.log)
    axis_map = parse_axis_map(args.axis_map) if args.axis_map else None
    mapper = HandToEEMapper(
        pos_scale=args.pos_scale,
        axis_map=axis_map,
        use_orientation=args.use_orientation,
        rot_scale=args.rot_scale,
        grip_lpf_alpha=args.grip_lpf,
    )
    replay(frames, mapper, args)


if __name__ == "__main__":
    main()
