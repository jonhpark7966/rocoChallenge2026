# Teleoperation (CloudXR + Apple Vision Pro)

이 문서는 기존 ws/UDP 텔레옵 파이프라인을 중단하고, **IsaacLab 공식 CloudXR + Apple Vision Pro 샘플** 흐름으로 재구성하기 위한 기준 문서입니다.
레거시 문서는 백업했습니다:
- `docs/teleoperation_legacy.md`
- `docs/teleop_debugging_legacy.md`

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

## 3. Galaxea R1 통합 계획 (초안)
- Galaxea R1 환경에서 XR 입력을 받기 위한 **teleop device 설정** 추가
- OpenXRDevice + Se3Abs/Se3Rel + GripperRetargeter 구성
- R1 IK 타겟 링크 기준: `left_arm_link6`, `right_arm_link6`
- 그리퍼 조인트: `left_gripper_axis1`, `right_gripper_axis1`
- 최종 목표: CloudXR + AVP 입력으로 R1 조립 가능하게 만들기

---

## 4. 참고
- 공식 가이드: `submodules/IsaacLab/docs/source/how-to/cloudxr_teleoperation.rst`
- Baseline 스크립트: `scripts/cloudxr/run_isaaclab_baseline.sh`
