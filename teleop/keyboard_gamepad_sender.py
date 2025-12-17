#!/usr/bin/env python3
"""
Keyboard + optional gamepad sender for ee_targets v1.

This script maintains an absolute EE state per arm and emits UDP packets.
It is a pre-Vision-Pro path: keyboard/gamepad only.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import select
import socket
import sys
import termios
import time
import tty
from typing import Dict, Optional

# Allow running as a standalone script (without installing teleop as a package)
if __package__ is None or __package__ == "":
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from math_utils import quat_from_axis_angle, quat_multiply, quat_normalize
else:
    from .math_utils import quat_from_axis_angle, quat_multiply, quat_normalize

try:
    import pygame  # type: ignore

    _HAVE_PYGAME = True
except Exception:
    _HAVE_PYGAME = False


def _read_key(timeout: float) -> Optional[str]:
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return None


class _RawStdin:
    def __enter__(self):
        self._old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._old:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old)


def apply_translation(state, arm_id: str, dx: float, dy: float, dz: float):
    p = state[arm_id]["p"]
    state[arm_id]["p"] = [p[0] + dx, p[1] + dy, p[2] + dz]


def apply_rotation(state, arm_id: str, yaw: float, pitch: float, roll: float):
    q = state[arm_id]["q"]
    q_yaw = quat_from_axis_angle((0, 0, 1), yaw)
    q_pitch = quat_from_axis_angle((0, 1, 0), pitch)
    q_roll = quat_from_axis_angle((1, 0, 0), roll)
    # Roll -> Pitch -> Yaw
    q_new = quat_multiply(q, quat_multiply(q_yaw, quat_multiply(q_pitch, q_roll)))
    state[arm_id]["q"] = list(quat_normalize(q_new))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def maybe_init_gamepad():
    if not _HAVE_PYGAME:
        return None
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        return None
    pad = pygame.joystick.Joystick(0)
    pad.init()
    return pad


def poll_gamepad(pad, pos_step: float, rot_step: float, grip_step: float, state, active_arm: str):
    """Very small mapping to avoid surprises across controllers."""
    pygame.event.pump()
    # Axes: 0/1 left stick, 2/3 right stick, 4/5 triggers on some pads
    ax0 = pad.get_axis(0)
    ax1 = pad.get_axis(1)
    ax2 = pad.get_axis(2) if pad.get_numaxes() > 2 else 0.0
    ax3 = pad.get_axis(3) if pad.get_numaxes() > 3 else 0.0
    ax4 = pad.get_axis(4) if pad.get_numaxes() > 4 else 0.0
    ax5 = pad.get_axis(5) if pad.get_numaxes() > 5 else 0.0

    apply_translation(state, active_arm, pos_step * ax0, -pos_step * ax1, pos_step * (ax5 - ax4))
    apply_rotation(state, active_arm, rot_step * ax2, rot_step * -ax3, rot_step * (ax5 - ax4))

    buttons = {i: pad.get_button(i) for i in range(pad.get_numbuttons())}
    if buttons.get(4):  # LB
        state[active_arm]["grip"] = clamp(state[active_arm]["grip"] + grip_step, 0.0, 1.0)
    if buttons.get(5):  # RB
        state[active_arm]["grip"] = clamp(state[active_arm]["grip"] - grip_step, 0.0, 1.0)
    precision = 1 if buttons.get(1) else 0  # B
    return precision


def build_packet(seq: int, frame: str, precision: int, state: Dict) -> Dict:
    return {
        "v": 1,
        "type": "ee_targets",
        "seq": seq,
        "t": time.time(),
        "frame": frame,
        "precision": precision,
        "arms": [
            {
                "id": arm_id,
                "ee_frame": arm_state["ee_frame"],
                "p": arm_state["p"],
                "q": arm_state["q"],
                "grip": arm_state["grip"],
            }
            for arm_id, arm_state in sorted(state.items())
        ],
    }


def main():
    parser = argparse.ArgumentParser(description="Keyboard + optional gamepad ee_targets sender")
    parser.add_argument("--ip", default="127.0.0.1", help="Receiver IP")
    parser.add_argument("--port", type=int, default=5005, help="Receiver UDP port")
    parser.add_argument("--rate", type=float, default=60.0, help="Send rate Hz")
    parser.add_argument("--frame", default="torso", help="Reference frame name")
    parser.add_argument("--pos-step", type=float, default=0.005, help="Position increment per keypress (m)")
    parser.add_argument("--rot-step-deg", type=float, default=2.0, help="Rotation increment per keypress (deg)")
    parser.add_argument("--grip-step", type=float, default=0.05, help="Grip increment per input (0..1)")
    parser.add_argument("--start-arm", default="L", choices=["L", "R"], help="Active arm at start")
    args = parser.parse_args()

    rot_step = math.radians(args.rot_step_deg)
    seq = 0
    dest = (args.ip, args.port)
    active_arm = args.start_arm
    precision = 0

    arms_state: Dict[str, Dict] = {
        "L": {"p": [0.45, 0.10, 0.85], "q": [1.0, 0.0, 0.0, 0.0], "grip": 1.0, "ee_frame": "left_gripper_tcp"},
        "R": {"p": [0.45, -0.10, 0.85], "q": [1.0, 0.0, 0.0, 0.0], "grip": 1.0, "ee_frame": "right_gripper_tcp"},
    }

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_period = 1.0 / args.rate
    gamepad = maybe_init_gamepad()
    if gamepad:
        print("[INFO] Gamepad detected and enabled.")
    else:
        if _HAVE_PYGAME:
            print("[INFO] No gamepad detected; keyboard only.")
        else:
            print("[INFO] pygame not installed; gamepad disabled. Install pygame if needed.")

    print("Controls: Tab toggle arm, P precision toggle, WASD=XY, E/Q=Z, J/L=Yaw, I/K=Pitch, U/O=Roll, Z/X=Grip +/-")

    try:
        with _RawStdin():
            while True:
                start = time.time()
                key = _read_key(timeout=0.0)
                if key:
                    if key == "\x03":  # Ctrl+C
                        break
                    elif key == "\t":
                        active_arm = "R" if active_arm == "L" else "L"
                        print(f"[ACTIVE] {active_arm}")
                    elif key.lower() == "p":
                        precision = 0 if precision else 1
                        print(f"[PRECISION] {precision}")
                    elif key in ("w", "W"):
                        apply_translation(arms_state, active_arm, 0.0, args.pos_step, 0.0)
                    elif key in ("s", "S"):
                        apply_translation(arms_state, active_arm, 0.0, -args.pos_step, 0.0)
                    elif key in ("a", "A"):
                        apply_translation(arms_state, active_arm, -args.pos_step, 0.0, 0.0)
                    elif key in ("d", "D"):
                        apply_translation(arms_state, active_arm, args.pos_step, 0.0, 0.0)
                    elif key in ("e", "E"):
                        apply_translation(arms_state, active_arm, 0.0, 0.0, args.pos_step)
                    elif key in ("q", "Q"):
                        apply_translation(arms_state, active_arm, 0.0, 0.0, -args.pos_step)
                    elif key in ("j", "J"):
                        apply_rotation(arms_state, active_arm, -rot_step, 0.0, 0.0)
                    elif key in ("l", "L"):
                        apply_rotation(arms_state, active_arm, rot_step, 0.0, 0.0)
                    elif key in ("i", "I"):
                        apply_rotation(arms_state, active_arm, 0.0, rot_step, 0.0)
                    elif key in ("k", "K"):
                        apply_rotation(arms_state, active_arm, 0.0, -rot_step, 0.0)
                    elif key in ("u", "U"):
                        apply_rotation(arms_state, active_arm, 0.0, 0.0, rot_step)
                    elif key in ("o", "O"):
                        apply_rotation(arms_state, active_arm, 0.0, 0.0, -rot_step)
                    elif key in ("z", "Z"):
                        arms_state[active_arm]["grip"] = clamp(arms_state[active_arm]["grip"] + args.grip_step, 0.0, 1.0)
                    elif key in ("x", "X"):
                        arms_state[active_arm]["grip"] = clamp(arms_state[active_arm]["grip"] - args.grip_step, 0.0, 1.0)

                pad_precision = 0
                if gamepad:
                    pad_precision = poll_gamepad(
                        gamepad,
                        args.pos_step * (0.5 if precision else 1.0),
                        rot_step * (0.5 if precision else 1.0),
                        args.grip_step,
                        arms_state,
                        active_arm,
                    )
                precision = pad_precision or precision

                packet = build_packet(seq, args.frame, precision, arms_state)
                sock.sendto(json.dumps(packet).encode("utf-8"), dest)
                seq += 1

                elapsed = time.time() - start
                to_sleep = send_period - elapsed
                if to_sleep > 0:
                    time.sleep(to_sleep)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
