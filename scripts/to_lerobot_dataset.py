#!/usr/bin/env python3
"""
RoCoChallenge HDF5 데이터셋을 LeRobotDataset 형식으로 변환합니다.

사용법:

0. huggingface_cli login을 통해 Hugging Face에 로그인합니다.

다음 4가지 케이스에 대한 실행 커맨드입니다:

A. Local HDF5 -> Local LeRobotDataset (새로운 디렉토리 생성)
   python to_lerobot_dataset.py --input path/to/local/hdf5_dir --output path/to/local/output_dir

B. Local HDF5 -> Remote Hugging Face LeRobotDataset (업로드)
   python to_lerobot_dataset.py --input path/to/local/hdf5_dir --output https://huggingface.co/datasets/user-name/dataset-name

C. Remote HDF5 -> Local LeRobotDataset (다운로드 및 변환)
   python to_lerobot_dataset.py --input https://huggingface.co/datasets/user-name/source-dataset --output path/to/local/output_dir

D. Remote HDF5 -> Remote Hugging Face LeRobotDataset (Hub에서 Hub로 변환)
   python to_lerobot_dataset.py --input https://huggingface.co/datasets/user-name/source-dataset --output https://huggingface.co/datasets/user-name/target-dataset

LeRobot Dataset Visualizer: https://huggingface.co/spaces/lerobot/visualize_dataset
"""

import argparse
import logging
import shutil
from pathlib import Path
from typing import Optional, List, Dict, Any

import h5py
import numpy as np
import torch
from huggingface_hub import HfApi, hf_hub_download, snapshot_download

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except ImportError:
    logger.error("lerobot이 설치되지 않았습니다. `pip install lerobot`으로 설치해주세요.")
    raise


def parse_args():
    parser = argparse.ArgumentParser(description="RoCo HDF5를 LeRobotDataset으로 변환합니다.")
    parser.add_argument(
        "--input",
        type=str,
        required=True,
        help="로컬 HDF5 파일/디렉토리 경로 또는 Hugging Face repo Url (예: 'https://huggingface.co/datasets/rocochallenge2025/rocochallenge2025').",
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="로컬 출력 디렉토리 경로 또는 업로드할 Hugging Face repo Url (예: 'https://huggingface.co/datasets/rocochallenge2025/rocochallenge2025').",
    )
    parser.add_argument(
        "--local-dir",
        type=str,
        default="data/cache",
        help="다운로드를 위한 로컬 캐시 디렉토리.",
    )
    parser.add_argument(
        "--robot-type",
        type=str,
        default="galaxea_r1", # RoCo2026
        help="메타데이터에 사용할 로봇 타입 이름.",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=15, # RoCo2026
        help="데이터셋의 초당 프레임 수 (FPS).",
    )

    return parser.parse_args()



def extract_repo_id(path_or_url: str) -> str:
    """Hugging Face URL에서 repo_id ('user/repo')를 추출하거나 원본 문자열을 반환합니다."""
    if path_or_url.startswith("http"):
        # 휴리스틱: '/'로 분리하고 'huggingface.co'가 포함된 경우 마지막 두 부분을 가져옵니다.
        # 예: https://huggingface.co/datasets/user/repo -> user/repo
        # 예: https://huggingface.co/user/repo -> user/repo
        parts = path_or_url.rstrip("/").split("/")
        if "huggingface.co" in path_or_url:
            if "datasets" in parts:
                idx = parts.index("datasets")
                if idx + 2 < len(parts):
                    return f"{parts[idx+1]}/{parts[idx+2]}"
            # datasets가 없는 경우 (예: model repo 등)
            # URL 구조가 https://huggingface.co/user/repo 라고 가정
            elif len(parts) >= 2:
                 return f"{parts[-2]}/{parts[-1]}"
    
    return path_or_url


# Arg로 받은 --input이 로컬 파일인지, Hugging Face Repo ID인지 확인하는 함수
def is_remote_repo(path_or_id: str) -> bool:
    """문자열이 Hugging Face repo ID 처럼 보이는지 확인합니다 ('/' 포함 여부 등)."""
    # 해당 경로의 파일이 로컬에 존재하는지 확인
    if Path(path_or_id).exists() or Path(path_or_id).parent.exists():
        return False
    # 상대경로로 시작하는지 확인
    if path_or_id.startswith("/") or path_or_id.startswith("./") or path_or_id.startswith("../"):
        return False
    # Hugging Face Repo ID로 시작하는지 확인 (http/https URL 포함)
    if path_or_id.startswith("http://") or path_or_id.startswith("https://"):
        return True
    return "/" in path_or_id


def get_local_hdf5_paths(input_path: str, local_cache_dir: Path) -> List[Path]:
    """입력 경로를 로컬 HDF5 파일 경로 리스트로 변환합니다."""
    # URL 입력을 처리하여 가능한 경우 repo_id를 추출하거나 그대로 사용합니다.
    input_path = extract_repo_id(input_path)
    
    path = Path(input_path)
    
    # Case 1: 로컬 파일인 경우
    if path.is_file() and path.suffix == ".hdf5":
        return [path]
    
    # Case 2: 로컬 디렉토리인 경우
    if path.is_dir():
        files = list(path.glob("**/*.hdf5"))
        if not files:
            raise ValueError(f"디렉토리에서 HDF5 파일을 찾을 수 없습니다: {input_path}")
        return files

    # Case 3: 원격 HF Repo인 경우
    if is_remote_repo(input_path):
        logger.info(f"원격 HF repo 감지됨: {input_path}")
        # 관련 파일 다운로드
        local_dir = local_cache_dir / input_path.replace("/", "_")
        try:
            # .hdf5 파일만 필터링합니다. 
            api = HfApi()
            files_in_repo = api.list_repo_files(repo_id=input_path, repo_type="dataset")
            hdf5_files = [f for f in files_in_repo if f.endswith(".hdf5")]
            
            if not hdf5_files:
                raise ValueError(f"원격 repo에서 HDF5 파일을 찾을 수 없습니다: {input_path}")

            downloaded_paths = []
            for file in hdf5_files:
                p = hf_hub_download(
                    repo_id=input_path,
                    filename=file,
                    repo_type="dataset",
                    local_dir=local_dir,
                )
                downloaded_paths.append(Path(p))
            return downloaded_paths

        except Exception as e:
            raise RuntimeError(f"HF repo {input_path}에서 다운로드 실패: {e}")

    raise ValueError(f"입력 '{input_path}'를 로컬에서 찾을 수 없으며 유효한 HF repo로 보이지 않습니다.")


def read_roco_hdf5(file_path: Path) -> Dict[str, np.ndarray]:
    """RoCo HDF5 파일을 읽어 LeRobot 형식으로 매핑합니다."""
    raw_data = {}
    
    def visitor(name, obj):
        if isinstance(obj, h5py.Dataset):
            raw_data[name] = np.array(obj)
            
    with h5py.File(file_path, "r") as f:
        f.visititems(visitor)

    data = {}
    
    # 1D 배열을 2D로 정리하는 헬퍼 함수
    def get_and_ensure_2d(key):
        val = raw_data.get(key)
        if val is not None:
             if val.ndim == 1:
                 return val[:, None]
             return val
        # logger.warning(f"키 {key}를 HDF5에서 찾을 수 없습니다")
        return None

    # Mapping
    # [actions] -> actions/key
    # [observations] -> observations/key
    
    # 1. 이미지 (Images)
    # observations/head_rgb, observations/left_hand_rgb, observations/right_hand_rgb
    image_map = {
        "observation.images.head": "observations/head_rgb",
        "observation.images.left_hand": "observations/left_hand_rgb",
        "observation.images.right_hand": "observations/right_hand_rgb",
        
        # depth 이미지
        "observation.images.head_depth": "observations/head_depth",
        "observation.images.left_hand_depth": "observations/left_hand_depth",
        "observation.images.right_hand_depth": "observations/right_hand_depth",
    }
    
    for lerobot_key, hdf5_key in image_map.items():
        if hdf5_key in raw_data:
            val = raw_data[hdf5_key]
            # 이미지(T, H, W, C). Depth 이미지는(T, H, W).
            if val.ndim == 3:
                val = val[..., None]
            data[lerobot_key] = val

    # 2. 액션 (Action)
    # [actions]
    # left_arm_action, left_gripper_action, right_arm_action, right_gripper_action
    
    action_components = [
        "actions/left_arm_action",  
        "actions/left_gripper_action", 
        "actions/right_arm_action",
        "actions/right_gripper_action",
    ]
    
    collected_actions = []
    
    for k in action_components:
        val = get_and_ensure_2d(k)
        if val is not None:
            collected_actions.append(val)
    
    if collected_actions:
        data["action"] = np.concatenate(collected_actions, axis=1)

    # 3. 상태 (State)
    # [observations]
    # left_arm_joint_pos, left_arm_joint_vel, left_gripper_joint_pos, left_gripper_joint_vel,
    # right_arm_joint_pos, right_arm_joint_vel, right_gripper_joint_pos, right_gripper_joint_vel
    
    state_keys = [
        "observations/left_arm_joint_pos",
        "observations/left_arm_joint_vel",
        "observations/left_gripper_joint_pos",
        "observations/left_gripper_joint_vel",
        
        "observations/right_arm_joint_pos",
        "observations/right_arm_joint_vel",
        "observations/right_gripper_joint_pos",
        "observations/right_gripper_joint_vel",
    ]
    
    collected_states = []
    for k in state_keys:
        val = get_and_ensure_2d(k)
        if val is not None:
             collected_states.append(val)
             
    if collected_states:
        data["observation.state"] = np.concatenate(collected_states, axis=1)

    return data


def main():
    args = parse_args()
    local_cache_dir = Path(args.local_dir)
    local_cache_dir.mkdir(parents=True, exist_ok=True)

    # 1. 입력 파일 목록 가져오기
    input_files = get_local_hdf5_paths(args.input, local_cache_dir)
    logger.info(f"{len(input_files)}개의 처리할 HDF5 파일을 찾았습니다.")

    # 2. 출력 모드 결정 (로컬 vs 원격)
    is_remote_output = is_remote_repo(args.output)
    
    if is_remote_output:
        repo_id = extract_repo_id(args.output)
        output_dir = local_cache_dir / "lerobot_build_temp"
        
        # 원격 업로드를 위한 임시 디렉토리이므로, 기존에 존재하면 삭제하고 새로 만듭니다.
        if output_dir.exists():
            shutil.rmtree(output_dir)
            
        create_repo_id = repo_id
        create_root = output_dir
    else:
        # 로컬 출력
        create_root = Path(args.output).resolve()
        create_repo_id = create_root.name

        # Safety Check
        # 1. Check if output is current working directory or parent
        cwd = Path.cwd().resolve()
        if create_root == cwd or cwd in create_root.parents:
             # Allowing subdirectory of CWD is fine, but not CWD itself or parent
             if create_root == cwd:
                 raise ValueError(f"출력 디렉토리({create_root})는 현재 작업 디렉토리와 같을 수 없습니다. 안전을 위해 별도의 서브 디렉토리를 지정해주세요.")
        
        # 2. Check if output is the script directory
        script_dir = Path(__file__).parent.resolve()
        if create_root == script_dir:
             raise ValueError(f"출력 디렉토리({create_root})는 스크립트가 있는 디렉토리와 같을 수 없습니다.")

    # Handle existing directory
    if create_root.exists():
        raise FileExistsError(f"출력 디렉토리 '{create_root}'가 이미 존재합니다. 신규 디렉토리를 지정해주세요.")

    
    # 3. LeRobot 데이터셋 생성
    first_data = read_roco_hdf5(input_files[0])
    
    features = {}
    for key, val in first_data.items():
        if key.startswith("observation.images."):
            shape = val.shape
            # (T, H, W, C)
            features[key] = {
                "dtype": "video",
                "shape": shape[1:], # (H, W, C)
                "names": ["height", "width", "channel"],
            }
        elif key.startswith("observation.depths."):
             shape = val.shape
             features[key] = {
                 "dtype": "float32",
                 "shape": shape[1:], # (H, W, 1)
                 "names": ["height", "width", "channel"],
             }
        elif key == "action":
            features[key] = {
                "dtype": "float32",
                "shape": (val.shape[1],),
                "names": [f"action_{i}" for i in range(val.shape[1])],
            }
        elif key == "observation.state":
            features[key] = {
                "dtype": "float32",
                "shape": (val.shape[1],),
                "names": [f"state_{i}" for i in range(val.shape[1])],
            }

    dataset = LeRobotDataset.create(
        repo_id=create_repo_id,
        fps=args.fps,
        robot_type=args.robot_type,
        features=features,
        root=create_root,
    )
    
    # 4. 데이터 반복 및 추가
    for file_path in input_files:
        logger.info(f"처리 중: {file_path}")
        data = read_roco_hdf5(file_path)
        
        # 프레임 수 확인
        # 모든 배열이 동일한 T를 가진다고 가정합니다.
        key0 = next(iter(data))
        num_frames = len(data[key0])
        
        # 프레임 반복 처리
        for i in range(num_frames):
            frame = {}
            for key, val in data.items():
                # val은 numpy 배열 (T, ...)
                # frame[key] = val[i] (numpy)
                # read_roco_hdf5는 numpy를 반환하고, add_frame은 dict(numpy or torch)를 받음.
                frame[key] = val[i]
            
            frame["task"] = "gearbox_assembly"
            dataset.add_frame(frame)
        
        dataset.save_episode()
        
    dataset.finalize()

    # 5. 최종 출력 처리
    if is_remote_output:
        logger.info(f"Hugging Face Hub로 데이터셋 푸시 중: {repo_id}")
        dataset.push_to_hub()
    else:
        logger.info(f"데이터셋이 로컬에 저장되었습니다: {dataset.root}")
        # repo_id가 주어졌을 때 데이터셋 클래스가 기본 캐시 디렉토리에 쓰는 경우를 대비해 
        # 이동이 필요할 수 있습니다.
        # 현재는 `LeRobotDataset.create(root=output_dir, ...)`가 해당 위치에 직접 쓴다고 가정합니다.


if __name__ == "__main__":
    main()
