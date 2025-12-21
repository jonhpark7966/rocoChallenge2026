# Teleoperation (Keyboard UDP + Vision Pro Bridge + IsaacSim)

이 문서는 **현재 코드 상태** 기준으로 텔레옵 파이프라인을 정리한 사용 가이드입니다.
주요 흐름은 두 가지입니다:

1) **Keyboard UDP (joint degrees) -> assembly_scene_teleop_demo**
2) **Vision Pro (vp_hands v1) -> Bridge (ee_targets v1) -> teleop_agent**

---

## 0. 빠른 요약 (결론)
- `keyboard_udp_teleop.py`는 **joint-degree 포맷**을 보내며, **`assembly_scene_teleop_demo.py` 전용**입니다.
- Vision Pro 브릿지는 **ee_targets v1**을 만들어 `teleop_agent.py`로 보냅니다.
- `teleop_agent.py`는 **ee_targets v1만** 수신합니다. (keyboard_udp_teleop와는 포맷이 다릅니다)
- 로봇이 안 움직이면: **seq 증가**, **ee_state/anchor**, **좌표계 axis_map**을 먼저 확인하세요.

---

## 1. 코드 상태 요약 (실제 파일 기준)

### IsaacSim / gearboxAssembly
- `submodules/gearboxAssembly/scripts/basic/assembly_scene_teleop_demo.py`
  - **keyboard_udp_teleop** 포맷(좌/우 6-DOF joint degrees + gripper)을 받습니다.
- `submodules/gearboxAssembly/scripts/basic/keyboard_udp_teleop.py`
  - 위 demo용 **joint-degree UDP sender**입니다.
- `submodules/gearboxAssembly/scripts/teleop_agent.py`
  - **ee_targets v1**을 UDP로 받아 IK + 그리퍼 제어.
  - `--state-port`로 **ee_state** 응답.
  - `--log_payload`로 **raw payload 로그** 출력.

### Bridge / Teleop (repo root)
- `teleop/vp_bridge.py`
  - WebSocket 서버로 vp_hands v1 수신
  - JSONL 로깅
  - (옵션) ee_targets v1 생성 -> UDP 송신
- `teleop/vp_replay.py`
  - JSONL 로그 리플레이 -> ee_targets UDP 송신
  - `--dry-run` 시 **ee_state 없으면 default anchor로 맵핑**
- `teleop/hand2ee_mapper.py`
  - hand delta -> ee_targets 변환 (anchor 기반)
- `teleop/vp_protocol.py`
  - vp_hands v1 파서
- `teleop/ee_targets_builder.py`
  - ee_targets v1 메시지 생성

### Vision Pro 앱 (submodule)
- 프로젝트: `submodules/vision-lerobot-teleop/lerobot-teleoperator/lerobot-teleoperator.xcodeproj`
- 전송 경로: `HandTrackingSystem` -> `WebSocketManager.sendVPHHands` (약 30Hz)
- payload:
  - `wrist_p`: `forearm.wrist` 조인트 위치
  - `pinch`: thumb.tip ↔ index.tip 거리 기반 (0..1)
  - `wrist_q`: 현재는 **nil** (미전송)
  - `seq`: `uptimeNanoseconds` 사용
- 레거시 송신 함수(`sendHandData`, `sendBothHands`)는 존재하지만 **현재 기본 경로는 vp_hands**입니다.
- `submodules/vision-lerobot-teleop/scripts/websocket_server.py`는 **legacy(leftHand/rightHand) 포맷** 기준이므로 vp_hands 테스트는 `teleop/vp_bridge.py`를 사용하세요.

---

## 2. 메시지 스펙

### 2.1 keyboard_udp_teleop 포맷 (assembly_scene_teleop_demo 전용)
```json
{
  "ts": 1734500000.123,
  "left_arm_deg": [..6..],
  "right_arm_deg": [..6..],
  "left_gripper": 1.0,
  "right_gripper": 1.0
}
```

### 2.2 ee_targets v1 (teleop_agent / bridge)
```json
{
  "v": 1,
  "type": "ee_targets",
  "seq": 1734500000123456789,
  "t": 1734500000.123,
  "frame": "world",
  "precision": 0,
  "arms": [
    {"id":"L","ee_frame":"left_gripper_tcp","p":[0,0,0],"q":[1,0,0,0],"grip":1.0},
    {"id":"R","ee_frame":"right_gripper_tcp","p":[0,0,0],"q":[1,0,0,0],"grip":1.0}
  ]
}
```
- `seq`는 **항상 증가**해야 합니다. (sender 재시작 시에도)

### 2.3 vp_hands v1 (Vision Pro -> Bridge)
```json
{
  "v": 1,
  "type": "vp_hands",
  "seq": 123456789,
  "t": 1734500000.123,
  "hands": {
    "L": {"tracked": true, "wrist_p":[x,y,z], "wrist_q": null, "pinch": 0.2},
    "R": {"tracked": true, "wrist_p":[x,y,z], "wrist_q": null, "pinch": 0.9}
  }
}
```
- 현재 Vision Pro 앱은 `wrist_q`를 보내지 않습니다.

---

## 3. 실행 방법 (Quick Start)

### 3.1 IsaacSim + teleop_agent (ee_targets v1 수신)
```bash
python submodules/gearboxAssembly/scripts/teleop_agent.py \
  --task Template-Galaxea-Lab-External-Direct-v0 \
  --listen_ip 0.0.0.0 --port 5005 --state_port 5006
```
- `--log_payload`를 추가하면 raw ee_targets 로그를 출력합니다.

### 3.2 ee_targets sender (키보드/게임패드 테스트)
```bash
python teleop/keyboard_gamepad_sender.py --ip 127.0.0.1 --port 5005 --state-port 5006
```
- sender 재시작 시에도 seq가 유지되도록 time-based seq 사용.

### 3.3 keyboard_udp_teleop (assembly_scene_teleop_demo 전용)
```bash
python submodules/gearboxAssembly/scripts/basic/assembly_scene_teleop_demo.py
python submodules/gearboxAssembly/scripts/basic/keyboard_udp_teleop.py --ip 127.0.0.1 --port 5005
```
- **teleop_agent와는 포맷이 달라** 서로 호환되지 않습니다.

### 3.4 Bridge (Vision Pro -> ee_targets)
- 로그만:
```bash
python teleop/vp_bridge.py --mode log_only --log data/vp_logs/session.jsonl
```
- 실시간 forward:
```bash
python teleop/vp_bridge.py \
  --mode forward \
  --udp-ip 127.0.0.1 --udp-port 5005 --state-port 5006
```
- 매핑/디버그 로그 포함:
```bash
python teleop/vp_bridge.py \
  --mode forward --no-udp-send --log-ee-targets \
  --log-map --log-map-every 5 \
  --log-bad --log-bad-max 20 --log-bad-bytes 512 \
  --log data/vp_logs/vp_debug.jsonl
```

### 3.5 Vision Pro 앱 연결 (Xcode)
1) Xcode에서 `submodules/vision-lerobot-teleop/lerobot-teleoperator/lerobot-teleoperator.xcodeproj` 열기
2) 앱 실행 후 **WebSocket Control** ON
3) 서버 주소를 `ws://<PC_IP>:8765`로 설정 후 **Connect**
4) 브릿지 터미널에서 `[RATE] vp_hands=...` 로그 확인

### 3.6 Replay (로그 -> ee_targets)
```bash
python teleop/vp_replay.py --log data/vp_logs/session.jsonl --udp-ip 127.0.0.1 --udp-port 5005 --state-port 5006
```
- UDP 없이 동작 확인:
```bash
python teleop/vp_replay.py --log data/vp_logs/session.jsonl --dry-run
```
  - ee_state가 없으면 default anchor로 맵핑합니다.

---

## 4. 테스트 순서 (권장)
1) **teleop_agent가 ee_targets를 받는지** 먼저 확인
   - `teleop/test_hardcoded_sender.py` 또는 `teleop/keyboard_gamepad_sender.py`로 확인
2) **Bridge log_only**로 vp_hands 수신/로깅 확인
   - Vision Pro 앱 연결 후 `[RATE]` 로그 확인
3) **vp_replay**로 로그를 재생해 ee_targets가 정상 전송되는지 확인
4) 마지막에 **Vision Pro 실시간 forward** 연결

---

## 5. 로깅/디버깅 방법

### 5.1 Bridge 로그(JSONL) 타입
- `meta`: 실행 환경/옵션
- `vp_hands`: raw 수신 메시지
- `robot_anchor`: ee_state 기반 로봇 기준점
- `map_debug`: 매핑 디버그 (옵션)
- `ee_targets`: 브릿지에서 생성된 출력 (옵션)
- `vp_hands_bad`: 파싱 실패 raw payload (옵션)

### 5.2 주요 플래그
- `--log-map` / `--log-map-every N`: map_debug 기록
- `--log-ee-targets`: 생성된 ee_targets 기록
- `--log-bad`: 파싱 실패 payload 기록
- `--gzip`: JSONL gzip 압축
- `teleop_agent.py --log_payload`: 수신한 raw ee_targets 출력

### 5.3 디버깅 가이드
- 자세한 축/스케일/앵커 튜닝은 `docs/teleop_debugging.md` 참고.

---

## 6. 자주 막히는 포인트
- **seq 재시작 문제**: teleop_agent는 `seq` 증가만 수용합니다. sender 재시작 시에도 증가하도록 time-based seq 사용.
- **ee_state/anchor 없음**: 브릿지/리플레이는 ee_state가 없으면 매핑이 안 되거나 default anchor로만 동작.
- **좌표계 불일치**: `--axis-map`, `--pos-scale`로 튜닝 필요.
- **VP payload mismatch**: `vp_hands_bad` 로그가 많으면 Vision Pro 앱의 payload 스키마를 먼저 맞추세요.
- **legacy 서버 혼동**: `submodules/vision-lerobot-teleop/scripts/websocket_server.py`는 legacy 포맷을 가정합니다. vp_hands는 `teleop/vp_bridge.py`로 수신하세요.

---

## 7. 참고 자료
- 디버깅 튜닝 가이드: `docs/teleop_debugging.md`
- Vision Pro 앱: `submodules/vision-lerobot-teleop`
