# CloudXR Baseline (IsaacLab + Apple Vision Pro)

이 폴더는 IsaacLab 공식 CloudXR 텔레옵 예제를 빠르게 검증하기 위한 스크립트 모음입니다.

## 빠른 실행
1) CloudXR Runtime 실행
```bash
./scripts/cloudxr/start_runtime_compose.sh
```

2) IsaacLab 텔레옵 예제 실행
```bash
./scripts/cloudxr/run_isaaclab_baseline.sh
```

3) Isaac Sim AR Panel 설정
- Selected Output Plugin: OpenXR
- OpenXR Runtime: System OpenXR Runtime (Option 2 사용 시)
- Start AR 클릭

4) AVP 샘플 앱에서 Connect → Play

---

## 옵션별 런타임
### Option 1 (Docker Compose, 권장)
```bash
./scripts/cloudxr/start_runtime_compose.sh
```
중지:
```bash
./scripts/cloudxr/stop_runtime_compose.sh
```

### Option 2 (Local IsaacLab + CloudXR Runtime 컨테이너)
```bash
./scripts/cloudxr/run_runtime_local.sh
source ./scripts/cloudxr/source_openxr_env.sh
```

---

## 포트 체크
```bash
nc -vz <isaac-lab-ip> 48010
```

---

## AVP 샘플 앱 (IsaacLab 2.3.0 기준 v2.3.0)
권장 위치: `submodules/isaac-xr-teleop-sample-client-apple`

```bash
git submodule add git@github.com:isaac-sim/isaac-xr-teleop-sample-client-apple.git \
  submodules/isaac-xr-teleop-sample-client-apple
cd submodules/isaac-xr-teleop-sample-client-apple
git checkout v2.3.0
```

---

## 팁
- 네트워크는 Wifi 6 전용 라우터 권장
- 지연이 심하면 `--device cpu`로 물리 실행
- CloudXR 렌더링 템포가 흔들리면 `NV_PACER_FIXED_TIME_STEP_MS` 설정 고려
