# Vision Pro -> Bridge -> IsaacSim(teleop_agent) 텔레옵 설계안 + 단계별 Plan (UDP 기반)

이 문서는 현재 repo 코드와 `submodules/vision-lerobot-teleop` (예전에 만들어 둔 Vision Pro 앱) 기준으로 작성했습니다.

## 0. 결론부터 말씀드리겠습니다
- Vision Pro 앱은 "손 트래킹 데이터만" 스트리밍하는 방향이 맞습니다.
- 로봇/IK/프레임/필터/리플레이는 전부 PC 브릿지에서 처리해야 반복 개발 속도가 나옵니다.
- 브릿지는 (1) 수신 + 로깅을 먼저, 그 다음에 (2) ee_targets 변환 + UDP 송신을 붙이십시오.
- IsaacSim 쪽은 이미 UDP ee_targets 수신 -> 로봇 컨트롤이 연결되어 있으므로, 브릿지는 기존 ee_targets v1 trimmed 스펙을 그대로 맞춰서 내보내기만 하면 됩니다.

## 1. 현재 상태 요약 (repo 기준)
IsaacSim(gearboxAssembly teleop) 쪽
- `submodules/gearboxAssembly/scripts/teleop_agent.py`가 UDP 5005로 ee_targets를 받고, 5006으로 들어오는 ee_state_request에 ee_state를 응답합니다.
- ee_targets 스키마는 v1 "trimmed" 버전이며, 파서/검증은 `teleop/ee_targets.py`에 있습니다. precision은 optional이며 없으면 0 처리합니다.
- receiver는 seq가 최신보다 클 때만 채택합니다 (`teleop/ee_targets.py`의 `state.seq > latest.seq`).
- `teleop/keyboard_gamepad_sender.py`는 agent의 ee_state를 받아 초기 포즈를 시드합니다.
- 키보드 sender 기본 frame은 "torso" 입니다 (`teleop/keyboard_gamepad_sender.py`).

ee_frame / 팔 ID 규약
- 팔 ID는 "L", "R" 입니다 (`teleop/ee_targets.py`).
- 키보드 sender 기본 ee_frame은 아래와 같으며 동일하게 사용합니다 (`teleop/keyboard_gamepad_sender.py`).
  - L: `left_gripper_tcp`
  - R: `right_gripper_tcp`

그리퍼 값
- 0 = 닫힘, 1 = 열림 (키보드 텔레옵으로 검증 완료) - 이 규약 유지합니다.

## 2. 목표와 비목표
목표 (MVP)
- Vision Pro 앱이 손 트래킹 데이터를 계속 전송 (PC로)
- PC 브릿지가 데이터를 JSONL로 로깅
- PC 브릿지가 로그를 리플레이하면서 동일한 hand trajectory를 IsaacSim에 UDP ee_targets로 재생
- 좌/우 팔 1:1 매핑 (왼손 -> L, 오른손 -> R) 유지

비목표 (지금은 하지 않습니다)
- clutch/reset/mode 같은 복잡 UX 복원 (원하시면 나중에 넣으면 됩니다)
- Vision Pro에서 직접 ee_targets 생성 (이건 개발 속도만 망칩니다)

## 3. 전체 아키텍처 (권장)
```
Vision Pro (hand tracking)
  -> (WebSocket 권장, 기존 코드 재사용) -> PC Bridge
       - Logger (raw hand stream JSONL)
       - Mapper (hand -> ee_targets)
       - Forwarder (UDP ee_targets -> IsaacSim teleop_agent:5005)
       - Replayer (JSONL -> Mapper -> UDP)
```

VP -> Bridge는 WebSocket 추천: 과거 repo에 이미 WebSocketManager/서버 스켈레톤과 양손 조인트 스트림 코드가 있습니다.
- 앱: `submodules/vision-lerobot-teleop/lerobot-teleoperator/lerobot-teleoperator/Models/WebSocketManager.swift`
- 손 추적: `submodules/vision-lerobot-teleop/lerobot-teleoperator/lerobot-teleoperator/Systems/HandTrackingSystem.swift`
- 서버 테스트: `submodules/vision-lerobot-teleop/scripts/websocket_server.py`

Bridge -> IsaacSim은 UDP 유지: 이미 agent가 UDP ee_targets를 받는 구조라 그대로 갑니다 (`submodules/gearboxAssembly/scripts/teleop_agent.py` + `teleop/ee_targets.py`).

## 4. 통신 스펙
### 4.1 IsaacSim이 받는 ee_targets v1 (trimmed) - 브릿지가 만들어 보내는 값
현재 파서 스펙 (`teleop/ee_targets.py`)은 아래 형태입니다.
```json
{
  "v": 1,
  "type": "ee_targets",
  "seq": 123,
  "t": 1734500000.123,
  "frame": "world",
  "precision": 0,
  "arms": [
    {"id": "L", "ee_frame": "left_gripper_tcp",  "p":[0,0,0], "q":[1,0,0,0], "grip": 1.0},
    {"id": "R", "ee_frame": "right_gripper_tcp", "p":[0,0,0], "q":[1,0,0,0], "grip": 1.0}
  ]
}
```

- precision은 optional이며, 없으면 0 취급 (`teleop/ee_targets.py`).
- 제거된 필드: clutch, reset, mode, active (지금은 필요 없음).

seq 관련 매우 중요한 요구사항
- receiver는 seq가 증가할 때만 최신으로 채택합니다 (`teleop/ee_targets.py`).
- 브릿지/리플레이는 절대 seq를 0부터 다시 시작하면 안 됩니다.
- 강력 추천: `seq = time.time_ns()` 또는 `time.monotonic_ns()`처럼 재시작해도 증가하는 값을 쓰십시오.

### 4.2 IsaacSim 상태 조회: ee_state_request / ee_state
키보드 sender가 사용하는 방식 그대로 재사용합니다 (`teleop/keyboard_gamepad_sender.py`).

요청:
```json
{"type": "ee_state_request"}
```

응답 (teleop_agent가 world frame으로 응답, `submodules/gearboxAssembly/scripts/teleop_agent.py` 참고):
```json
{
  "type": "ee_state",
  "frame": "world",
  "arms": [
    {"id":"L", "p":[...], "q":[...], "grip": 1.0, "ee_frame":"left_gripper_tcp"},
    {"id":"R", "p":[...], "q":[...], "grip": 1.0, "ee_frame":"right_gripper_tcp"}
  ]
}
```

## 5. 브릿지 설계: "로그-우선" + "리플레이-우선"
### 5.1 브릿지가 받는 Vision Pro 손 스트림 메시지 (새로 정의)
권장 vp_hands_v1 (JSON)
```json
{
  "v": 1,
  "type": "vp_hands",
  "seq": 123456789,
  "t": 1734500000.123,
  "hands": {
    "L": {"tracked": true,  "wrist_p":[x,y,z], "wrist_q":[w,x,y,z], "pinch": 0.2},
    "R": {"tracked": true,  "wrist_p":[x,y,z], "wrist_q":[w,x,y,z], "pinch": 0.9}
  }
}
```

- 최소 MVP에서는 wrist_q 없이 position-only로 시작해도 됩니다.
- pinch는 0..1로 두고, 브릿지에서 그리퍼로 맵핑합니다.
- 과거 repo는 27개 조인트 position만 전송하는 compact struct가 있습니다. 그대로 쓰되 브릿지에서 wrist를 뽑아도 되고, 더 깔끔하게는 VP에서 wrist pose만 보내도록 간단히 바꾸셔도 됩니다 (아래 9.2 참고).

## 6. Hand -> ee_targets 변환 (핵심 로직)
### 6.1 좌/우 팔 1:1 매핑
- VP 왼손 -> arms[].id="L"
- VP 오른손 -> arms[].id="R"
- ee_frame은 키보드 sender 기본값 그대로:
  - L=left_gripper_tcp
  - R=right_gripper_tcp

### 6.2 "앵커 기반 델타" 방식 (캘리브레이션 최소화)
브릿지 시작 시 ee_state_request로 현재 로봇 EE pose를 읽습니다.
VP에서 손이 처음 안정적으로 들어오면 그 시점의 손 wrist pose를 hand_anchor로 저장합니다.
이후에는 손 변화량(델타)을 EE에 적용합니다.

포지션
```
dp_vp = wrist_p_now - wrist_p_anchor
dp_sim = M_pos @ dp_vp        # 축 매핑 행렬, config 조정
p_target = p_robot_anchor + pos_scale * dp_sim
```

오리엔테이션(나중 단계)
```
dq_vp = q_now * inverse(q_anchor)
dq_sim = q_map * dq_vp * inverse(q_map)   # basis change
q_target = dq_sim * q_robot_anchor
```

강한 의견: 처음엔 translation-only + gripper로 성공시키고, 그 다음에 orientation을 붙이십시오.

### 6.3 그리퍼 맵핑
VP에서 pinch=1이 닫힘 쪽이라고 가정하면:
```
grip = clamp(1.0 - pinch, 0.0, 1.0)
```

추가로, 브릿지에서 LPF 한 줄 넣으면 떨림이 크게 줄어듭니다 (옵션).

### 6.4 프레임(frame) 결정
브릿지에서 만들어내는 p/q가 로봇 EE world pose 기준이면 frame="world"로 보내십시오.
`submodules/gearboxAssembly/scripts/teleop_agent.py`는 world/torso를 지원하며, 그 외는 world로 처리하면서 경고를 출력합니다.

## 7. 로깅 설계
### 7.1 파일 포맷: JSONL (한 줄 = 한 메시지)
추천 구조:
```json
{"type":"meta","v":1,"created_at":1734500000.1,"schema":"vp_hands_v1","config":{...}}
{"type":"vp_hands","t_recv":1734500000.2,"msg":{...vp_hands_v1...}}
{"type":"vp_hands","t_recv":1734500000.216,"msg":{...}}
{"type":"event","t_recv":1734500010.0,"name":"reanchor"}
```

브릿지에서 생성한 ee_targets도 저장하고 싶으면:
```json
{"type":"ee_targets","t_send":1734500000.3, "msg":{...}}
```

다만 raw만 저장 + replay에서 재생성하는 쪽을 더 추천합니다 (매핑 바꾸며 실험할 때 유리).

### 7.2 로그 회전 / 크기
60Hz 양손이면 금방 커집니다. JSONL + gzip 옵션(--gzip) 넣으시면 충분합니다.

## 8. 리플레이 설계 (디버깅 루프)
### 8.1 리플레이가 반드시 해야 하는 것
- JSONL에서 vp_hands 스트림 읽기
- (리플레이 시작 시) IsaacSim에 ee_state_request -> robot_anchor 세팅
- 첫 vp_hands를 hand_anchor로 세팅
- 원본 타임스탬프 간격으로 sleep 하며 ee_targets 생성/송신

### 8.2 seq 처리 (중요)
- 로그에 기록된 seq를 그대로 쓰면 안 됩니다. (중간 스킵/역순/재시작 등으로)
- 리플레이 시마다 seq = time.time_ns() 같은 방식으로 새로 생성하십시오.

### 8.3 리플레이 옵션(필수급)
- --speed 0.5/1/2 : 재생 속도
- --t0 / --t1 : 구간 재생
- --loop
- --no-orient (translation-only 재생)
- --pos-scale, --rot-scale, --axis-map (축 맵핑/스케일 튜닝)

## 9. 코드 변경안 (파일/모듈 구조 제안)
### 9.1 rocoChallenge2026 (teleop 브랜치) 추가/수정
A) 브릿지 런타임 (실시간 forward + logging)
- `teleop/vp_bridge.py`
  - WebSocket(or UDP) 수신
  - JSONL 로깅
  - (옵션) 실시간 ee_targets 생성 -> UDP 송신

B) 리플레이 도구
- `teleop/vp_replay.py`
  - JSONL 읽기
  - ee_state_request로 시드
  - 변환 후 UDP 송신

C) 공용 모듈 (테스트 가능한 순수 로직)
- `teleop/vp_protocol.py` : vp_hands 파서/validator (버전/필드 체크)
- `teleop/ee_targets_builder.py` : ee_targets 메시지 빌더 (seq/t/frame/arms)
- `teleop/hand2ee_mapper.py` : 앵커 기반 델타 매핑 구현
- `teleop/io_udp.py` : UDP send + ee_state_request helper

"테스트 가능성"이 핵심입니다. Mapper는 IO 없이 pure function 스타일로 짜면 리플레이/실시간이 같은 코드를 씁니다.

### 9.2 vision-lerobot-teleop (Vision Pro 앱) 최소 수정
현재 코드 흐름
- `HandTrackingSystem`이 양손 업데이트마다 WebSocket으로 데이터를 보냅니다 (`submodules/vision-lerobot-teleop/lerobot-teleoperator/lerobot-teleoperator/Systems/HandTrackingSystem.swift`).
- WebSocket 연결은 `WebSocketManager`가 담당합니다 (`submodules/vision-lerobot-teleop/lerobot-teleoperator/lerobot-teleoperator/Models/WebSocketManager.swift`).

MVP 수정 2가지 옵션
옵션 1) "가장 적은 수정": 지금 보내는 27 joint position 그대로 보내기
- 브릿지에서 wrist joint index를 고정해서 wrist_p를 뽑습니다.
- orientation/핀치 없이 translation-only부터 시작합니다.

옵션 2) "권장": VP에서 wrist pose(+pinch)만 보내도록 payload 단순화
- 네트워크 비용/파싱 비용이 줄고, mapper 구현이 단순해집니다.
- pinch 값을 같이 보내면 gripper까지 바로 됩니다.

저는 옵션 2를 권합니다. 어차피 과거 코드 많은 부분을 버릴 거라면 "손목 pose + pinch"만 남기고 정리하는 게 맞습니다.

## 10. 단계별 Plan (실행 순서 + Definition of Done)
Stage 0 - 기준선 고정 (이미 완료했지만 체크리스트로 유지)
- 목표: IsaacSim teleop_agent가 UDP ee_targets를 문제 없이 받는 상태 유지.
- `submodules/gearboxAssembly/scripts/teleop_agent.py` 실행해서 UDP 5005 리슨 확인
- `teleop/keyboard_gamepad_sender.py`로 작동 + 그리퍼 0..1 정상 동작 확인
- ee_state_request가 5006으로 응답 오는지 확인 (`teleop/keyboard_gamepad_sender.py`로 확인 가능)
- DoD: "키보드로 L/R 팔, gripper 모두 움직임 + ee_state 시드가 정상"

Stage 1 - 브릿지 "수신+로깅"만 먼저
- 목표: Vision Pro 없이도 개발 가능한 루프. 브릿지가 raw hand stream을 받아 JSONL로 저장.
- 구현: `teleop/vp_bridge.py --mode log_only`
- 입력은 일단 PC에서 mock sender로 만들어도 됩니다.
- (가능하면) Vision Pro 앱의 기존 WebSocket 전송을 그대로 받아서 저장.
- 기능 요구:
  - 연결 상태 로그 (connected/disconnected)
  - JSONL 파일 생성
  - 초당 수신률/누락률 프린트
- DoD: 로그 파일이 안정적으로 쌓이고 재실행해도 잘 저장됨.

Stage 2 - 리플레이(로그 -> 화면 출력) + 시간 재생
- 목표: Vision Pro 없이도 로그를 시간축대로 재생할 수 있어야 함.
- 구현: `teleop/vp_replay.py --log session.jsonl --dry-run`
- 옵션: --speed, --t0/--t1, --loop
- 주의: seq는 로그값 쓰지 말고 새로 생성
- 출력: 현재 L/R 손 위치(혹은 wrist) 및 tracked 여부
- DoD: replay가 시간축을 유지하며 돌아가고, 중간 구간 재생/배속/루프가 됨.

Stage 3 - 리플레이 -> ee_targets 변환 -> IsaacSim UDP 송신 (translation-only)
- 목표: orientation 없이도 손 이동이 EE target position으로 반영되어 로봇 팔이 움직이게.
- 구현:
  - `hand2ee_mapper.py` (anchor delta)
  - ee_state_request로 robot_anchor 잡기
  - 첫 hand msg를 hand_anchor로 잡기
  - frame="world"로 ee_targets 생성해서 5005로 송신
- 옵션:
  - --pos-scale로 민감도 조절
  - --axis-map으로 축/부호 튜닝
- DoD: 리플레이만으로 IsaacSim 팔이 일관되게 움직임 (방향이 조금 틀려도 OK).

Stage 4 - 실시간 브릿지 forward 붙이기 (Vision Pro live -> IsaacSim)
- 목표: Vision Pro에서 실제 손을 움직이면 브릿지가 실시간으로 ee_targets를 만들어 5005로 보냄.
- 구현: Stage1(수신) + Stage3(매핑) 합치기
- 로깅은 항상 ON (debug 목적)
- UX: 브릿지 터미널에서 reanchor 입력하면 즉시 앵커 재설정
  - ee_state 다시 받아 robot_anchor 업데이트
  - 현재 손을 hand_anchor로 업데이트
- DoD: "실시간 + 로깅 + reanchor + 끊김 복구"가 됨.

Stage 5 - 그리퍼 + orientation 확장
그리퍼
- VP pinch를 받으면 grip=1-pinch
- 없으면 grip 유지 (혹은 1.0 고정)

오리엔테이션
- 처음에는 yaw만 넣는 식으로 단계적으로 붙이십시오.
- 축 맵핑(q_map)을 config로 노출.

DoD: assembly에 의미 있는 6DoF 조작이 가능.

## 11. 리스크/함정 (미리 방지)
- seq 재시작 문제: agent가 seq > latest.seq만 받습니다. 브릿지 재시작 후 seq=0이면 전부 드랍됩니다. 무조건 time 기반 seq 사용.
- 좌표계 축/부호: VisionOS/ARKit 좌표계와 IsaacSim 월드 축이 다를 수밖에 없습니다. anchor delta + axis-map 튜닝으로 해결.
- orientation부터 하면 망합니다: translation-only로 먼저 움직이게 만든 뒤 orientation 추가.
- 손 트래킹 dropout: tracked=false일 때는 해당 팔 업데이트를 빼거나 송신을 멈추는 쪽이 안전합니다.

## 12. (제가 깔아둔 가정) - 바꾸려면 여기만 수정
- VP -> Bridge는 WebSocket 기반 (기존 Vision Pro 코드 재사용)
- IsaacSim에 보내는 ee_targets는:
  - id는 "L"/"R"
  - ee_frame은 "left_gripper_tcp"/"right_gripper_tcp"
  - grip은 0..1 (0 닫힘/1 열림)
  - frame="world" (MVP)

## 13. 사용 방법 (Quick Start)
사전 준비
- Python 의존성: `pip install websockets` (또는 `pip install -e .`로 `pyproject.toml` 기준 설치)
- IsaacSim teleop_agent: `python submodules/gearboxAssembly/scripts/teleop_agent.py --task ...`

Vision Pro 앱 (vp_hands_v1 송신)
1) Xcode에서 `submodules/vision-lerobot-teleop/lerobot-teleoperator` 프로젝트 열기
2) 앱 실행 후 "WebSocket Control" 켜고, 서버 주소를 `ws://<PC_IP>:8765`로 설정
3) 손 추적이 켜지면 vp_hands 메시지가 브릿지로 전송됩니다

브릿지 (수신 + 로깅 / 실시간 forward)
- 로그만:
```
python teleop/vp_bridge.py --mode log_only --log data/vp_logs/session.jsonl
```
- 로봇으로 실시간 전송:
```
python teleop/vp_bridge.py --mode forward --udp-ip 127.0.0.1 --udp-port 5005 --state-port 5006
```
- UDP 없이 ee_targets 로그만:
```
python teleop/vp_bridge.py --mode forward --no-udp-send --log-ee-targets --log data/vp_logs/vp_forward.jsonl
```
- 터미널에서 `reanchor` 입력 시 앵커 재설정

리플레이 (로그 -> ee_targets)
- 드라이런:
```
python teleop/vp_replay.py --log data/vp_logs/session.jsonl --dry-run
```
- 실제 UDP 송신:
```
python teleop/vp_replay.py --log data/vp_logs/session.jsonl --udp-ip 127.0.0.1 --udp-port 5005 --state-port 5006
```

튜닝 옵션 (브릿지/리플레이 공통)
- `--speed 0.5/1/2`, `--t0/--t1`, `--loop`
- `--pos-scale 0.5` (민감도 조절)
- `--axis-map "1,0,0;0,1,0;0,0,1"` (축 맵핑)
- `--no-orient` (orientation 비활성, 기본값)

## 14. 매핑 디버깅 (손 움직임과 로봇 움직임이 다를 때)
브릿지에서 매핑 로그를 켜면 원인 추적이 훨씬 쉽습니다.

브릿지 실행 (UDP 없이 로그만)
```
python teleop/vp_bridge.py \
  --mode forward \
  --no-udp-send \
  --log-ee-targets \
  --log-map --log-map-every 5 \
  --log data/vp_logs/vp_debug.jsonl
```

로그에서 확인할 항목
- `robot_anchor` : ee_state로 받은 로봇 기준점 (없으면 default anchor 사용)
- `map_debug.entries[]` :
  - `status` = `mapped | anchor_set | skip`
  - `reason` (skip 사유: not_tracked, missing_robot_anchor 등)
  - `dp_vp` : 손 변화량 (wrist 기준)
  - `dp_sim` : axis_map 적용 후 변화량
  - `p_target` : 실제 ee_targets로 나가는 위치

자주 발생하는 원인과 해결 전략
- 축이 뒤집힘/교차됨: `dp_vp` 방향과 `p_target` 변화 방향이 다르면 axis_map을 튜닝합니다.
  - 예: X 반전 `--axis-map "-1,0,0;0,1,0;0,0,1"`
  - 예: Y/Z 스왑 `--axis-map "1,0,0;0,0,1;0,1,0"`
- 스케일이 과/소함: `dp_vp` 대비 `p_target` 변화량이 너무 크거나 작으면 `--pos-scale` 조절.
- 앵커가 어긋남: 로봇 기준점이 틀리면 전체가 밀립니다.
  - 실시간 모드에서는 `reanchor` 입력으로 재설정
  - 로그-only 디버그는 default anchor이므로 실제 로봇과는 다르게 보일 수 있음
- orientation부터 넣으면 축 튜닝이 어려워집니다. translation-only부터 맞추고 나중에 `--use-orient` 활성화.

원하시면 위 설계를 기준으로 "구체적인 작업 티켓(파일별 TODO + 함수 시그니처 + 테스트 케이스)" 형식으로 더 쪼개드리겠습니다.
