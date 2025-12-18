from __future__ import annotations

import json
import socket
from typing import Dict, Optional, Tuple


def send_udp_json(payload: Dict, ip: str, port: int, sock: Optional[socket.socket] = None) -> None:
    data = json.dumps(payload).encode("utf-8")
    if sock is None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            udp.sendto(data, (ip, port))
    else:
        sock.sendto(data, (ip, port))


def request_ee_state(ip: str, port: int, timeout_s: float = 1.0) -> Optional[Dict]:
    payload = {"type": "ee_state_request"}
    data = json.dumps(payload).encode("utf-8")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.settimeout(timeout_s)
        udp.sendto(data, (ip, port))
        try:
            reply, _ = udp.recvfrom(4096)
        except socket.timeout:
            return None
    try:
        msg = json.loads(reply.decode("utf-8"))
    except Exception:
        return None
    if not isinstance(msg, dict) or msg.get("type") != "ee_state":
        return None
    return msg
