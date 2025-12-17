#!/usr/bin/env python3
"""
Hardcoded ee_targets packet sender for smoke testing receivers/bridges.

Use this before plugging in keyboard/gamepad/Vision Pro.
"""

from __future__ import annotations

import argparse
import json
import socket
import time


SCENARIOS = [
    {
        "desc": "Reset edge with clutch off (should hold)",
        "packet": {
            "v": 1,
            "type": "ee_targets",
            "seq": 1,
            "t": 0.0,
            "frame": "torso",
            "clutch": 0,
            "precision": 0,
            "reset": 1,
            "arms": [
                {"id": "L", "ee_frame": "left_gripper_tcp", "p": [0.4, 0.1, 0.85], "q": [1, 0, 0, 0], "grip": 1.0, "mode": "free"}
            ],
        },
    },
    {
        "desc": "Free mode, clutch on, left arm open grip",
        "packet": {
            "v": 1,
            "type": "ee_targets",
            "seq": 2,
            "t": 0.0,
            "frame": "torso",
            "clutch": 1,
            "precision": 0,
            "reset": 0,
            "arms": [
                {"id": "L", "ee_frame": "left_gripper_tcp", "p": [0.45, 0.1, 0.90], "q": [0.98, 0.0, 0.2, 0.0], "grip": 1.0, "mode": "free"}
            ],
        },
    },
    {
        "desc": "Insert mode, precision on, right arm closing grip",
        "packet": {
            "v": 1,
            "type": "ee_targets",
            "seq": 3,
            "t": 0.0,
            "frame": "torso",
            "clutch": 1,
            "precision": 1,
            "reset": 0,
            "arms": [
                {"id": "R", "ee_frame": "right_gripper_tcp", "p": [0.4, -0.1, 0.82], "q": [0.99, 0.0, 0.05, 0.0], "grip": 0.3, "mode": "insert"}
            ],
        },
    },
    {
        "desc": "Rotate mode roll twist",
        "packet": {
            "v": 1,
            "type": "ee_targets",
            "seq": 4,
            "t": 0.0,
            "frame": "torso",
            "clutch": 1,
            "precision": 0,
            "reset": 0,
            "arms": [
                {"id": "R", "ee_frame": "right_gripper_tcp", "p": [0.4, -0.1, 0.82], "q": [0.96, 0.0, 0.0, 0.28], "grip": 0.3, "mode": "rotate"}
            ],
        },
    },
]


def main():
    parser = argparse.ArgumentParser(description="Send hardcoded ee_targets packets to a receiver.")
    parser.add_argument("--ip", default="127.0.0.1", help="Receiver IP")
    parser.add_argument("--port", type=int, default=5005, help="Receiver UDP port")
    parser.add_argument("--delay", type=float, default=0.5, help="Delay between packets (s)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for scenario in SCENARIOS:
            packet = dict(scenario["packet"])
            packet["t"] = time.time()
            print(f"[SEND] {scenario['desc']} seq={packet['seq']}")
            sock.sendto((json.dumps(packet) + "\n").encode("utf-8"), (args.ip, args.port))
            time.sleep(args.delay)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
