from __future__ import annotations

import argparse
import asyncio
import gzip
import json
import os
import socket
import sys
import threading
import time
from typing import Optional

if __package__ is None and __name__ == "__main__":
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

try:
    import websockets
except ImportError:  # pragma: no cover - runtime dependency
    websockets = None

from teleop.ee_targets_builder import build_ee_targets
from teleop.hand2ee_mapper import HandToEEMapper, parse_axis_map
from teleop.io_udp import request_ee_state, send_udp_json
from teleop.vp_protocol import parse_vp_hands_json


class JsonlLogger:
    def __init__(self, path: str, gzip_enabled: bool) -> None:
        self.path = path
        if gzip_enabled:
            self._fh = gzip.open(path, "at")
        else:
            self._fh = open(path, "a", encoding="utf-8")

    def write(self, record: dict) -> None:
        self._fh.write(json.dumps(record) + "\n")
        self._fh.flush()

    def close(self) -> None:
        self._fh.close()


class BridgeRuntime:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.logger = JsonlLogger(args.log, args.gzip)
        self.forward = args.mode == "forward"
        self.send_udp = self.forward and not args.no_udp_send
        self.log_map = args.log_map
        self.log_map_every = max(1, args.log_map_every)
        axis_map = parse_axis_map(args.axis_map) if args.axis_map else None
        self.mapper = HandToEEMapper(
            pos_scale=args.pos_scale,
            axis_map=axis_map,
            use_orientation=args.use_orientation,
            rot_scale=args.rot_scale,
            grip_lpf_alpha=args.grip_lpf,
        )
        self._udp_sock: Optional[socket.socket] = None
        if self.send_udp:
            self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._reanchor_event = threading.Event()
        self._last_print = time.time()
        self._msg_count = 0
        self._bad_count = 0
        self._frame_index = 0

    def start(self) -> None:
        self._write_meta()
        if self.forward:
            self._refresh_robot_anchor()
        self._start_reanchor_listener()

    def close(self) -> None:
        if self._udp_sock:
            self._udp_sock.close()
        self.logger.close()

    def _write_meta(self) -> None:
        self.logger.write(
            {
                "type": "meta",
                "v": 1,
                "created_at": time.time(),
                "schema": "vp_hands_v1",
                "config": {
                    "axis_map": self.args.axis_map or "identity",
                    "pos_scale": self.args.pos_scale,
                    "rot_scale": self.args.rot_scale,
                    "use_orientation": self.args.use_orientation,
                    "mode": self.args.mode,
                    "no_udp_send": self.args.no_udp_send,
                    "log_map": self.args.log_map,
                    "log_map_every": self.log_map_every,
                },
            }
        )

    def _start_reanchor_listener(self) -> None:
        if not self.forward:
            return

        def _listen() -> None:
            while True:
                line = sys.stdin.readline()
                if not line:
                    break
                if line.strip().lower() in ("reanchor", "r"):
                    self._reanchor_event.set()

        threading.Thread(target=_listen, daemon=True).start()

    def _refresh_robot_anchor(self) -> None:
        state = request_ee_state(self.args.udp_ip, self.args.state_port, self.args.state_timeout)
        if state:
            self.mapper.update_robot_anchor_from_state(state)
            print("[INFO] robot anchor updated from ee_state")
            self._log_robot_anchor("ee_state")
        elif self.args.no_udp_send:
            self._seed_default_anchor()
        else:
            print("[WARN] ee_state request timed out; mapping disabled until anchor is set")

    def _seed_default_anchor(self) -> None:
        defaults = {
            "L": {"p": (0.45, 0.10, 0.85), "q": (1.0, 0.0, 0.0, 0.0), "grip": 1.0},
            "R": {"p": (0.45, -0.10, 0.85), "q": (1.0, 0.0, 0.0, 0.0), "grip": 1.0},
        }
        for arm_id, anchor in defaults.items():
            self.mapper.set_robot_anchor(arm_id, anchor["p"], anchor["q"], anchor["grip"])
        print("[INFO] ee_state unavailable; using default anchors for logging only")
        self._log_robot_anchor("default")

    def _log_robot_anchor(self, source: str) -> None:
        snapshot = self.mapper.get_robot_anchor_snapshot()
        if snapshot:
            self.logger.write({"type": "robot_anchor", "t_recv": time.time(), "source": source, "arms": snapshot})

    def _maybe_reanchor(self) -> None:
        if not self._reanchor_event.is_set():
            return
        self._reanchor_event.clear()
        self.logger.write({"type": "event", "t_recv": time.time(), "name": "reanchor"})
        self.mapper.reset_hand_anchors()
        self._refresh_robot_anchor()

    def process_message(self, payload: str | bytes) -> None:
        now = time.time()
        frame = parse_vp_hands_json(payload)
        if not frame:
            self._bad_count += 1
            return
        self._msg_count += 1
        self.logger.write({"type": "vp_hands", "t_recv": now, "msg": frame.raw})

        if self.forward:
            self._maybe_reanchor()
            if self.log_map:
                arms, debug = self.mapper.map_frame_debug(frame)
                self._maybe_log_map(frame, debug, now)
            else:
                arms = self.mapper.map_frame(frame)
            if arms:
                msg = build_ee_targets(arms=arms, frame=self.args.frame, precision=0)
                if self.send_udp:
                    send_udp_json(msg, self.args.udp_ip, self.args.udp_port, sock=self._udp_sock)
                if self.args.log_ee_targets:
                    self.logger.write({"type": "ee_targets", "t_send": time.time(), "msg": msg})

        self._maybe_print_rate(now)

    def _maybe_print_rate(self, now: float) -> None:
        if now - self._last_print < 1.0:
            return
        good = self._msg_count
        bad = self._bad_count
        self._msg_count = 0
        self._bad_count = 0
        self._last_print = now
        print(f"[RATE] vp_hands={good}/s bad={bad}/s")

    def _maybe_log_map(self, frame, debug_entries, now: float) -> None:
        if not debug_entries:
            return
        self._frame_index += 1
        if self._frame_index % self.log_map_every != 0:
            return
        self.logger.write(
            {
                "type": "map_debug",
                "t_recv": now,
                "vp_seq": frame.seq,
                "vp_t": frame.t,
                "entries": debug_entries,
            }
        )


async def serve(args: argparse.Namespace) -> None:
    if websockets is None:
        raise RuntimeError("websockets is not installed. Run: pip install websockets")

    runtime = BridgeRuntime(args)
    runtime.start()

    async def handler(websocket) -> None:
        peer = websocket.remote_address
        print(f"[INFO] client connected: {peer}")
        try:
            async for message in websocket:
                runtime.process_message(message)
        except Exception as exc:
            print(f"[WARN] client error: {exc}")
        finally:
            print(f"[INFO] client disconnected: {peer}")

    async with websockets.serve(handler, args.ws_host, args.ws_port):
        print(f"[INFO] WebSocket server on ws://{args.ws_host}:{args.ws_port}")
        print("[INFO] type 'reanchor' + Enter to reset anchors")
        try:
            await asyncio.Future()
        finally:
            runtime.close()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Vision Pro hand stream bridge + logger")
    parser.add_argument("--ws-host", default="0.0.0.0", help="WebSocket bind host")
    parser.add_argument("--ws-port", type=int, default=8765, help="WebSocket bind port")
    parser.add_argument("--log", default=default_log_path(), help="JSONL log output path")
    parser.add_argument("--gzip", action="store_true", help="Gzip log output")
    parser.add_argument("--mode", choices=("log_only", "forward"), default="log_only")
    parser.add_argument("--udp-ip", default="127.0.0.1", help="IsaacSim UDP target IP")
    parser.add_argument("--udp-port", type=int, default=5005, help="IsaacSim UDP target port")
    parser.add_argument("--state-port", type=int, default=5006, help="ee_state UDP port")
    parser.add_argument("--state-timeout", type=float, default=1.0, help="ee_state timeout seconds")
    parser.add_argument("--no-udp-send", action="store_true", help="Do not send UDP (log only)")
    parser.add_argument("--log-map", action="store_true", help="Log mapping debug info")
    parser.add_argument("--log-map-every", type=int, default=1, help="Log mapping every N frames")
    parser.add_argument("--frame", default="world", help="ee_targets frame")
    parser.add_argument("--pos-scale", type=float, default=1.0, help="Position scale factor")
    parser.add_argument("--rot-scale", type=float, default=1.0, help="Rotation scale factor")
    parser.add_argument("--axis-map", default="", help="Axis map matrix 'a,b,c;d,e,f;g,h,i'")
    parser.add_argument("--use-orient", dest="use_orientation", action="store_true", help="Enable orientation mapping")
    parser.add_argument("--no-orient", dest="use_orientation", action="store_false", help="Disable orientation mapping")
    parser.set_defaults(use_orientation=False)
    parser.add_argument("--grip-lpf", type=float, default=0.0, help="Grip LPF alpha (0 disables)")
    parser.add_argument("--log-ee-targets", action="store_true", help="Log ee_targets alongside vp_hands")
    return parser


def default_log_path() -> str:
    ts = time.strftime("%Y%m%d_%H%M%S")
    folder = os.path.join("data", "vp_logs")
    os.makedirs(folder, exist_ok=True)
    return os.path.join(folder, f"vp_hands_{ts}.jsonl")


def main() -> None:
    args = build_arg_parser().parse_args()
    try:
        asyncio.run(serve(args))
    except KeyboardInterrupt:
        print("\n[INFO] bridge stopped")


if __name__ == "__main__":
    main()
