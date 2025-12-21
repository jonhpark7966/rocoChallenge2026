# RoCoChallenge2026 Workspace

Lightweight wrapper repo: tracks submodules, small tools, and setup notes. Large assets/data are not committed.

## Quick start
```bash
git clone https://github.com/jonhpark7966/rocoChallenge2026.git
cd rocoChallenge2026
git submodule update --init --recursive
```

### Python env (for helper scripts)
```bash
uv venv .venv
uv sync               # installs deps from pyproject.toml
uv run python scripts/roco_hdf5_viewer.py --help
```
*pip 사용자*: `python -m venv .venv && source .venv/bin/activate && pip install -e .`

### Submodules
- `submodules/gearboxAssembly` — task code & Omniverse extension (fork).
- `submodules/IsaacLab` — upstream Isaac Lab.
- `submodules/isaac-xr-teleop-sample-client-apple` — Apple Vision Pro CloudXR teleop sample client.

필요하면 최신으로:
```bash
git submodule update --remote --merge submodules/gearboxAssembly
git submodule update --remote --merge submodules/IsaacLab
```

### Data (not in git)
- Source: Hugging Face `rocochallenge2025/rocochallenge2025`.
- 빠른 구조/프리뷰: `python scripts/roco_hdf5_viewer.py gearbox_assembly_demos/data_20251127_212217.hdf5 --seconds 5`
- 캐시/출력 기본 경로: `data/cache`, `data/exports` (ignored).

필요 내용은 이후에 추가합니다.
