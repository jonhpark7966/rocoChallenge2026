# Teleop Debugging Guide (Vision Pro -> Bridge -> IsaacSim)

이 문서는 "손 움직임과 로봇 움직임이 다르게 보이는" 상황을 빠르게 추적하기 위한 디버깅 가이드입니다.
핵심 목표는 **축 맵핑 / 스케일 / 앵커** 문제를 로그로 확인하고 조정하는 것입니다.

## 1) 가장 중요한 개념 요약
- **브릿지는 손의 변화량(델타)을 로봇 EE에 적용**합니다.
- 기준점은 2개:
  - hand_anchor: 첫 손 입력 시의 손목 위치
  - robot_anchor: ee_state로 받은 로봇 EE 위치
- 따라서 로봇 타겟은 다음으로 계산됩니다:
  - `dp_vp = wrist_now - wrist_anchor`
  - `dp_sim = axis_map @ dp_vp`
  - `p_target = robot_anchor + pos_scale * dp_sim`

문제가 생기면 대부분 아래 중 하나입니다:
- 좌표계 축/부호가 맞지 않음 (axis_map)
- pos_scale 과/소
- robot_anchor가 없는 상태 (ee_state 미수신)

## 2) 디버그용 브릿지 실행 (로봇 없이 로그만)
아래 실행으로 **ee_targets + 매핑 로그**를 남깁니다.
```
python teleop/vp_bridge.py \
  --mode forward \
  --no-udp-send \
  --log-ee-targets \
  --log-map --log-map-every 5 \
  --log data/vp_logs/vp_debug.jsonl
```

로그에서 확인할 핵심 항목
- `robot_anchor` : ee_state로 받은 로봇 기준점 (없으면 default anchor)
- `map_debug.entries[]`:
  - `status`: `mapped | anchor_set | skip`
  - `reason`: skip 사유
  - `dp_vp`: 손 변화량
  - `dp_sim`: axis_map 적용 후 변화량
  - `p_target`: 실제 ee_targets로 나가는 위치

## 3) 빠른 원인 판별 체크리스트
- **축이 뒤집힘/스왑**: `dp_vp` 방향과 `p_target` 방향이 다르면 axis_map을 수정.
- **스케일 문제**: `dp_vp` 대비 `p_target` 변화량이 너무 크거나 작으면 `--pos-scale` 조정.
- **앵커 문제**: `robot_anchor`가 없으면 전체 동작이 밀립니다.
  - 실시간이면 `reanchor` 입력으로 재설정
  - 로깅만 하는 경우 default anchor가 사용됨

## 4) axis_map 튜닝 예시
- X 반전:
```
--axis-map "-1,0,0;0,1,0;0,0,1"
```
- Y/Z 스왑:
```
--axis-map "1,0,0;0,0,1;0,1,0"
```
- X/Y 스왑:
```
--axis-map "0,1,0;1,0,0;0,0,1"
```

## 5) 최소 튜닝 전략 (권장 순서)
1. translation-only 유지 (`--no-orient`)
2. **한 축씩 움직여** `dp_vp`와 `p_target` 방향 비교
3. axis_map 튜닝 (부호/스왑)
4. pos_scale 튜닝
5. 마지막에 orientation (`--use-orient`) 도입

## 6) 주요 코드 요약 (핵심 로직 위치)
- `teleop/hand2ee_mapper.py`
  - `HandToEEMapper`:
    - `map_frame()` -> vp_hands를 ee_targets로 변환
    - `map_frame_debug()` -> 변환 + debug 정보 반환
  - `apply_axis_map()` : axis_map 행렬 적용
  - `pos_scale`, `axis_map`, `use_orientation`이 주요 튜닝 파라미터
- `teleop/vp_bridge.py`
  - `--log-map`: map_debug 로그 생성
  - `--no-udp-send`: UDP 송신 없이 로그만
  - `robot_anchor` 로그 생성
- `teleop/vp_replay.py`
  - `--speed`: 느린 재생으로 디버깅 가능

## 7) 다음 단계
- axis_map 후보를 몇 개 시험하면서 손-로봇 방향을 맞춥니다.
- 로그에서 `dp_vp` vs `p_target`가 같은 방향이 되면, UDP 전송을 켭니다.

