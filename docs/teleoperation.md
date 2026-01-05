# Teleoperation (CloudXR + Apple Vision Pro)

---

## 0. 목표 (Baseline 먼저)
1) IsaacLab 공식 예제로 **CloudXR + AVP 샘플 연결**을 먼저 검증
2) 검증이 되면 Galaxea R1 환경으로 **teleop retarget/IK 연결** 확장

---

## 1. Baseline 테스트 (공식 예제)
### 1.1 CloudXR Runtime 실행
**Option 1 (권장, Docker Compose)**
```bash
./scripts/cloudxr/start_runtime_compose.sh
```

**Option 2 (Local IsaacLab + CloudXR Runtime)**
```bash
./scripts/cloudxr/run_runtime_local.sh
# 다른 터미널에서 아래 환경 변수 적용
source ./scripts/cloudxr/source_openxr_env.sh
```

방화벽 포트 (IsaacLab 워크스테이션):
```bash
sudo ufw allow 47998:48000,48005,48008,48012/udp
sudo ufw allow 48010/tcp
```

### 1.2 IsaacLab 공식 텔레옵 예제 실행
```bash
./scripts/cloudxr/run_isaaclab_baseline.sh
```
- 필요 시 CPU 물리: `--device cpu`
- 기본 task: `Isaac-PickPlace-GR1T2-Abs-v0`

### 1.3 Isaac Sim UI 설정
AR Panel에서:
- Selected Output Plugin: **OpenXR**
- OpenXR Runtime: **System OpenXR Runtime** (Option 2 사용 시)
- **Start AR** 클릭

### 1.4 Apple Vision Pro 샘플 클라이언트
- AVP 샘플 앱 연결 후 **Connect → Play**
- 네트워크 확인:
```bash
nc -vz <isaac-lab-ip> 48010
```

---

## 2. Apple Vision Pro 샘플 앱 (서브모듈 위치)
IsaacLab 2.3.0 기준 샘플 앱 버전: **v2.3.0**

권장 경로:
```
submodules/isaac-xr-teleop-sample-client-apple
```

예시 (사용자 실행):
```bash
git submodule add git@github.com:isaac-sim/isaac-xr-teleop-sample-client-apple.git \
  submodules/isaac-xr-teleop-sample-client-apple
cd submodules/isaac-xr-teleop-sample-client-apple
git checkout v2.3.0
```

---

## 3. Galaxea R1 통합 (OpenXR Teleop)

### 3.1 실행 (Template-Galaxea-Lab-Agent-Direct-v0)
```bash
./isaaclab.sh -p submodules/gearboxAssembly/scripts/teleop_r1_agent.py \
  --task Template-Galaxea-Lab-Agent-Direct-v0 \
  --teleop_device handtracking
```

### 3.2 CLI 옵션

| Option | Default | Description |
|--------|---------|-------------|
| `--task` | `Template-Galaxea-Lab-Agent-Direct-v0` | 환경 이름 |
| `--teleop_device` | `handtracking` | 입력 장치 (handtracking, keyboard) |
| `--view_mode` | `headcam` | XR 뷰 모드 (headcam, world) |
| `--anchor_mode` | `world` | XR 앵커 모드 (headcam, world) |
| `--gripper_open` | `0.04` | 그리퍼 열림 위치 (m) |
| `--gripper_close` | `0.0` | 그리퍼 닫힘 위치 (m) |
| `--record` | - | 데이터 녹화 활성화 |
| `--record_dir` | `./data/teleop_demos` | 녹화 저장 경로 |
| `--visualize_targets` | - | EE 타겟 마커 표시 |
| `--hand_markers` | - | 손 마커 표시 |
| `--no_randomize_objects` | - | 물체 고정 배치 |

### 3.3 데이터 녹화
```bash
./isaaclab.sh -p submodules/gearboxAssembly/scripts/teleop_r1_agent.py \
  --task Template-Galaxea-Lab-Agent-Direct-v0 \
  --teleop_device handtracking \
  --record \
  --record_dir ./data/teleop_demos
```

녹화 데이터는 HDF5 형식으로 저장됩니다:
```
episode_N.h5
├── observations/
│   ├── head_rgb              (T, H, W, 3) uint8
│   ├── left_hand_rgb         (T, H, W, 3) uint8
│   ├── right_hand_rgb        (T, H, W, 3) uint8
│   ├── left_arm_joint_pos    (T, 6) float32
│   ├── right_arm_joint_pos   (T, 6) float32
│   ├── left_gripper_pos      (T, 1) float32
│   └── right_gripper_pos     (T, 1) float32
├── actions/
│   ├── left_arm_action       (T, 6) float32
│   ├── right_arm_action      (T, 6) float32
│   ├── left_gripper          (T, 1) float32
│   └── right_gripper         (T, 1) float32
└── attrs: {sim: True, timestamp: ...}
```

### 3.4 XR 제스처
- **START**: 엄지와 검지를 핀치 후 release → 텔레옵 활성화 + 녹화 시작
- **STOP**: 다시 핀치 후 release → 텔레옵 비활성화 + 녹화 저장
- **RESET**: 별도 제스처 또는 키보드 'R' → 환경 리셋

### 3.5 설정 메모
- R1 IK 타겟 링크: `left_arm_link6`, `right_arm_link6`
- 그리퍼 조인트: `left_gripper_axis1`, `right_gripper_axis1` (0.0-0.04m)
- XR 앵커가 맞지 않으면 Isaac Sim XR 패널에서 Anchor 위치를 조정
- Reanchoring: START 시 현재 로봇 EE 포즈와 XR 손 위치의 오프셋을 자동 계산

---

## 4. 참고
- 공식 가이드: `submodules/IsaacLab/docs/source/how-to/cloudxr_teleoperation.rst`
- Baseline 스크립트: `scripts/cloudxr/run_isaaclab_baseline.sh`
- Teleop 스크립트: `submodules/gearboxAssembly/scripts/teleop_r1_agent.py`
- Data Recorder: `submodules/gearboxAssembly/scripts/teleop_data_recorder.py`
