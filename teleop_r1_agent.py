# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Teleoperate the Galaxea R1 agent environment with OpenXR hand tracking."""

import argparse
import logging
import os
import time

from isaaclab.app import AppLauncher


# add argparse arguments
parser = argparse.ArgumentParser(description="OpenXR teleoperation for Galaxea R1 agent environment.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate (teleop uses 1).")
parser.add_argument(
    "--task",
    type=str,
    default="Template-Galaxea-Lab-Agent-Direct-v0",
    help="Name of the task (expected: Template-Galaxea-Lab-Agent-Direct-v0).",
)
parser.add_argument(
    "--teleop_device",
    type=str,
    default="handtracking",
    help="Teleop device (only handtracking is supported in this script).",
)
parser.add_argument(
    "--view_mode",
    type=str,
    choices=["headcam", "world"],
    default="headcam",
    help="Viewport/XR view mode. 'headcam' follows the robot head camera, 'world' uses default view.",
)
parser.add_argument(
    "--anchor_mode",
    type=str,
    choices=["world", "headcam"],
    default="world",
    help="XR anchor mode. 'world' keeps the anchor fixed at world origin, 'headcam' follows headcam position.",
)
parser.add_argument(
    "--keep_env_cameras",
    action="store_true",
    default=False,
    help="Keep environment RTX camera sensors enabled (can reduce CloudXR frame-rate / cause render contention).",
)
parser.add_argument(
    "--reanchor_rotation",
    dest="reanchor_rotation",
    action="store_true",
    default=True,
    help="Reanchor wrist rotation to current robot EE orientation on teleop start (default: enabled).",
)
parser.add_argument(
    "--no_reanchor_rotation",
    dest="reanchor_rotation",
    action="store_false",
    help="Disable rotation reanchor (position-only reanchor).",
)
parser.add_argument("--gripper_open", type=float, default=None, help="Gripper open joint position target.")
parser.add_argument("--gripper_close", type=float, default=None, help="Gripper close joint position target.")
parser.add_argument(
    "--pinch_open",
    type=float,
    default=0.05,
    help="Pinch distance (m) treated as fully open.",
)
parser.add_argument(
    "--pinch_close",
    type=float,
    default=0.02,
    help="Pinch distance (m) treated as fully closed.",
)
parser.add_argument(
    "--pinch_calib",
    dest="pinch_calib",
    action="store_true",
    default=True,
    help="Enable adaptive pinch calibration (default: enabled).",
)
parser.add_argument(
    "--no_pinch_calib",
    dest="pinch_calib",
    action="store_false",
    help="Disable adaptive pinch calibration.",
)
parser.add_argument(
    "--pinch_offset_ratio",
    type=float,
    default=0.4,
    help="Fraction of observed pinch range used as offset from min/max when calibrating.",
)
parser.add_argument(
    "--pinch_open_ratio",
    type=float,
    default=0.7,
    help="Fraction of observed pinch range used as offset from max for full open (adaptive).",
)
parser.add_argument(
    "--pinch_close_ratio",
    type=float,
    default=0.2,
    help="Fraction of observed pinch range used as offset from min for full close (adaptive).",
)
parser.add_argument(
    "--pinch_open_offset",
    type=float,
    default=None,
    help="Absolute offset (m) from observed max distance for full open (overrides ratio when set).",
)
parser.add_argument(
    "--pinch_close_offset",
    type=float,
    default=None,
    help="Absolute offset (m) from observed min distance for full close (overrides ratio when set).",
)
parser.add_argument(
    "--pinch_min_range",
    type=float,
    default=0.015,
    help="Minimum observed pinch range (m) required before adaptive calibration is applied.",
)
parser.add_argument(
    "--pinch_min_decay",
    type=float,
    default=0.0,
    help="Leaky update rate for pinch min distance when dist > min (0 disables).",
)
parser.add_argument(
    "--pinch_max_decay",
    type=float,
    default=0.0,
    help="Leaky update rate for pinch max distance when dist < max (0 disables).",
)
parser.add_argument(
    "--left_wrist_offset_rpy",
    nargs=3,
    type=float,
    default=(0.0, 0.0, 0.0),
    help="Left wrist rotation offset in degrees (roll pitch yaw).",
)
parser.add_argument(
    "--right_wrist_offset_rpy",
    nargs=3,
    type=float,
    default=(0.0, 0.0, 0.0),
    help="Right wrist rotation offset in degrees (roll pitch yaw).",
)
parser.add_argument(
    "--pinch_log_path",
    type=str,
    default=None,
    help="Optional CSV log path for pinch distance/grip values.",
)
parser.add_argument(
    "--gripper_invert",
    action="store_true",
    default=False,
    help="Invert pinch mapping (swap open/close).",
)
parser.add_argument(
    "--hand_markers",
    action="store_true",
    default=False,
    help="Show wrist/thumb/index markers for both hands.",
)
parser.add_argument(
    "--no_randomize_objects",
    action="store_false",
    dest="randomize_objects",
    default=True,
    help="Disable object randomization on reset (default: randomize).",
)
parser.add_argument(
    "--visualize_targets",
    action="store_true",
    default=False,
    help="Visualize target poses produced by the retargeter.",
)
parser.add_argument(
    "--hold_pose",
    action="store_true",
    default=False,
    help="Hold current robot joint positions and ignore teleop commands (debug).",
)

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

is_handtracking = "handtracking" in args_cli.teleop_device.lower()
# NOTE: For XR/CloudXR teleop, keeping physics on GPU generally provides a much smoother visual experience.
# Users can still override explicitly with AppLauncher `--device cpu` if desired.

app_launcher_args = vars(args_cli)
if is_handtracking:
    app_launcher_args["xr"] = True

# launch omniverse app
app_launcher = AppLauncher(app_launcher_args)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import torch

from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.devices.device_base import DeviceBase, DevicesCfg
from isaaclab.devices.openxr import OpenXRDeviceCfg, XrAnchorRotationMode, XrCfg
from isaaclab.devices.openxr.retargeters import Se3AbsRetargeterCfg
from isaaclab.devices.retargeter_base import RetargeterBase, RetargeterCfg
from isaaclab.devices.teleop_device_factory import create_teleop_device
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import FRAME_MARKER_CFG, SPHERE_MARKER_CFG
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_apply, quat_from_euler_xyz, quat_inv, quat_mul, subtract_frame_transforms

import isaaclab_tasks
from isaaclab_tasks.utils import parse_env_cfg

import Galaxea_Lab_External.tasks

logger = logging.getLogger(__name__)

class PinchGripperRetargeter(RetargeterBase):
    """Continuous gripper retargeter based on pinch distance."""

    def __init__(self, cfg: "PinchGripperRetargeterCfg"):
        super().__init__(cfg)
        if cfg.bound_hand not in [DeviceBase.TrackingTarget.HAND_LEFT, DeviceBase.TrackingTarget.HAND_RIGHT]:
            raise ValueError(
                "bound_hand must be either DeviceBase.TrackingTarget.HAND_LEFT or DeviceBase.TrackingTarget.HAND_RIGHT"
            )
        self.bound_hand = cfg.bound_hand
        self.open_distance = cfg.open_distance
        self.close_distance = cfg.close_distance
        self.invert = cfg.invert
        self._adaptive = cfg.adaptive_calibration
        self._offset_ratio = cfg.offset_ratio
        self._open_ratio = cfg.open_ratio
        self._close_ratio = cfg.close_ratio
        self._open_offset = cfg.open_offset
        self._close_offset = cfg.close_offset
        self._min_range = cfg.min_range
        self._min_decay = cfg.min_decay
        self._max_decay = cfg.max_decay
        self._markers = None
        if cfg.enable_markers:
            marker_cfg = SPHERE_MARKER_CFG.copy()
            marker_cfg.prim_path = cfg.marker_prim_path
            marker_cfg.markers["sphere"].radius = cfg.marker_radius
            self._markers = VisualizationMarkers(marker_cfg)
        self.min_distance = None
        self.max_distance = None
        self.last_distance = None
        self.last_grip = None
        self.last_open_distance = None
        self.last_close_distance = None

    def _update_bounds(self, dist: float) -> None:
        if self.min_distance is None or self.max_distance is None:
            self.min_distance = dist
            self.max_distance = dist
            return
        if dist < self.min_distance:
            self.min_distance = dist
        elif self._min_decay > 0.0:
            self.min_distance += self._min_decay * (dist - self.min_distance)
        if dist > self.max_distance:
            self.max_distance = dist
        elif self._max_decay > 0.0:
            self.max_distance += self._max_decay * (dist - self.max_distance)

    def _compute_thresholds(self) -> tuple[float, float]:
        if not self._adaptive or self.min_distance is None or self.max_distance is None:
            self.last_open_distance = self.open_distance
            self.last_close_distance = self.close_distance
            return self.open_distance, self.close_distance

        dist_range = self.max_distance - self.min_distance
        if dist_range < self._min_range:
            self.last_open_distance = self.open_distance
            self.last_close_distance = self.close_distance
            return self.open_distance, self.close_distance

        if self._open_offset is not None:
            open_offset = self._open_offset
        elif self._open_ratio is not None:
            open_offset = dist_range * self._open_ratio
        else:
            open_offset = dist_range * self._offset_ratio

        if self._close_offset is not None:
            close_offset = self._close_offset
        elif self._close_ratio is not None:
            close_offset = dist_range * self._close_ratio
        else:
            close_offset = dist_range * self._offset_ratio

        ratio_sum = 0.0
        if self._open_ratio is not None:
            ratio_sum += self._open_ratio
        if self._close_ratio is not None:
            ratio_sum += self._close_ratio
        if ratio_sum >= 1.0 and dist_range > 0.0:
            scale = 0.9 / ratio_sum
            open_offset *= scale
            close_offset *= scale

        margin_total = open_offset + close_offset
        if margin_total > 0.0:
            available = max(dist_range - self._min_range, 0.0)
            scale = min(1.0, available / margin_total)
            open_offset *= scale
            close_offset *= scale

        open_distance = self.max_distance - open_offset
        close_distance = self.min_distance + close_offset
        if open_distance <= close_distance:
            open_distance = self.max_distance
            close_distance = self.min_distance

        self.last_open_distance = open_distance
        self.last_close_distance = close_distance
        return open_distance, close_distance

    def retarget(self, data: dict) -> torch.Tensor:
        hand_data = data[self.bound_hand]
        thumb_tip = hand_data.get("thumb_tip")
        index_tip = hand_data.get("index_tip")
        wrist = hand_data.get("wrist")

        thumb = torch.tensor(thumb_tip[:3], dtype=torch.float32)
        index = torch.tensor(index_tip[:3], dtype=torch.float32)
        dist = float(torch.norm(thumb - index).item())
        self._update_bounds(dist)
        open_distance, close_distance = self._compute_thresholds()
        denom = max(open_distance - close_distance, 1e-6)
        grip = (dist - close_distance) / denom
        grip = float(min(max(grip, 0.0), 1.0))
        if self.invert:
            grip = 1.0 - grip
        self.last_distance = dist
        self.last_grip = grip

        if self._markers is not None and wrist is not None:
            pts = torch.tensor(
                [wrist[:3], thumb_tip[:3], index_tip[:3]],
                dtype=torch.float32,
                device=self._sim_device,
            )
            self._markers.visualize(translations=pts)

        return torch.tensor([grip], dtype=torch.float32, device=self._sim_device)

    def get_requirements(self) -> list[RetargeterBase.Requirement]:
        return [RetargeterBase.Requirement.HAND_TRACKING]


class PinchGripperRetargeterCfg(RetargeterCfg):
    """Configuration for pinch-based gripper retargeter."""

    def __init__(
        self,
        *,
        bound_hand: DeviceBase.TrackingTarget,
        open_distance: float,
        close_distance: float,
        invert: bool,
        adaptive_calibration: bool,
        offset_ratio: float,
        open_offset: float | None,
        close_offset: float | None,
        open_ratio: float | None,
        close_ratio: float | None,
        min_range: float,
        min_decay: float,
        max_decay: float,
        enable_markers: bool,
        marker_prim_path: str,
        marker_radius: float,
        sim_device: str,
    ):
        super().__init__(sim_device=sim_device)
        self.bound_hand = bound_hand
        self.open_distance = open_distance
        self.close_distance = close_distance
        self.invert = invert
        self.adaptive_calibration = adaptive_calibration
        self.offset_ratio = offset_ratio
        self.open_offset = open_offset
        self.close_offset = close_offset
        self.open_ratio = open_ratio
        self.close_ratio = close_ratio
        self.min_range = min_range
        self.min_decay = min_decay
        self.max_decay = max_decay
        self.enable_markers = enable_markers
        self.marker_prim_path = marker_prim_path
        self.marker_radius = marker_radius
        self.retargeter_type = PinchGripperRetargeter


def _setup_arm_ik(env, arm_name: str):
    diff_ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls")
    controller = DifferentialIKController(diff_ik_cfg, num_envs=env.num_envs, device=env.device)
    arm_entity_cfg = SceneEntityCfg(
        "robot", joint_names=[f"{arm_name}_arm_joint.*"], body_names=[f"{arm_name}_arm_link6"]
    )
    arm_entity_cfg.resolve(env.scene)
    return controller, arm_entity_cfg


def _compute_arm_targets(robot, arm_entity_cfg: SceneEntityCfg, controller: DifferentialIKController,
                         target_pos_w: torch.Tensor, target_quat_w: torch.Tensor) -> torch.Tensor:
    arm_joint_ids = arm_entity_cfg.joint_ids
    arm_body_ids = arm_entity_cfg.body_ids

    if robot.is_fixed_base:
        ee_jacobi_idx = arm_body_ids[0] - 1
    else:
        ee_jacobi_idx = arm_body_ids[0]

    jacobian = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, arm_joint_ids]
    ee_pose_w = robot.data.body_state_w[:, arm_body_ids[0], 0:7]
    root_pose_w = robot.data.root_state_w[:, 0:7]
    joint_pos = robot.data.joint_pos[:, arm_joint_ids]

    ee_pos_b, ee_quat_b = subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
    )
    target_pos_b, target_quat_b = subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7], target_pos_w, target_quat_w
    )

    controller.set_command(torch.cat([target_pos_b, target_quat_b], dim=-1))
    return controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)


def _map_gripper_command(grip_cmd: torch.Tensor, open_pos: torch.Tensor, close_pos: torch.Tensor) -> torch.Tensor:
    grip = torch.clamp(grip_cmd, 0.0, 1.0)
    return close_pos + grip * (open_pos - close_pos)

def _infer_gripper_positions(env) -> tuple[torch.Tensor, torch.Tensor]:
    left_gripper_ids, _ = env.robot.find_joints(env.cfg.left_gripper_dof_name)
    if len(left_gripper_ids) == 0:
        raise RuntimeError("Unable to resolve gripper joint IDs for auto gripper limits.")

    gripper_id = left_gripper_ids[0]
    limits = env.robot.data.soft_joint_pos_limits[0, gripper_id]
    min_lim = float(limits[0].item())
    max_lim = float(limits[1].item())
    current = float(env.robot.data.joint_pos[0, gripper_id].item())
    if abs(current - max_lim) <= abs(current - min_lim):
        open_pos = max_lim
        close_pos = min_lim
    else:
        open_pos = min_lim
        close_pos = max_lim
    logger.info("Auto gripper range: open=%.4f close=%.4f", open_pos, close_pos)
    return torch.tensor(open_pos, device=env.device), torch.tensor(close_pos, device=env.device)

def _format_optional(value, digits=4) -> str:
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}"

def _log_gripper_limits(env) -> dict:
    left_gripper_ids, _ = env.robot.find_joints(env.cfg.left_gripper_dof_name)
    right_gripper_ids, _ = env.robot.find_joints(env.cfg.right_gripper_dof_name)
    if len(left_gripper_ids) == 0 or len(right_gripper_ids) == 0:
        logger.warning("Unable to resolve gripper joint IDs for limits logging.")
        return {}
    left_id = left_gripper_ids[0]
    right_id = right_gripper_ids[0]

    def _get_pos_limits(limits_tensor, joint_id):
        if limits_tensor is None:
            return None, None
        values = limits_tensor[0, joint_id]
        return float(values[0].item()), float(values[1].item())

    def _get_limit(limits_tensor, joint_id):
        if limits_tensor is None:
            return None
        return float(limits_tensor[0, joint_id].item())

    left_soft_min, left_soft_max = _get_pos_limits(env.robot.data.soft_joint_pos_limits, left_id)
    right_soft_min, right_soft_max = _get_pos_limits(env.robot.data.soft_joint_pos_limits, right_id)
    left_hard_min, left_hard_max = _get_pos_limits(env.robot.data.joint_pos_limits, left_id)
    right_hard_min, right_hard_max = _get_pos_limits(env.robot.data.joint_pos_limits, right_id)
    left_vel_limit = _get_limit(env.robot.data.joint_vel_limits, left_id)
    right_vel_limit = _get_limit(env.robot.data.joint_vel_limits, right_id)
    left_effort_limit = _get_limit(env.robot.data.joint_effort_limits, left_id)
    right_effort_limit = _get_limit(env.robot.data.joint_effort_limits, right_id)

    left_pos = float(env.robot.data.joint_pos[0, left_id].item())
    right_pos = float(env.robot.data.joint_pos[0, right_id].item())
    logger.info(
        "Left gripper limits soft=[%s, %s] hard=[%s, %s] vel=%s effort=%s current=%.4f",
        _format_optional(left_soft_min),
        _format_optional(left_soft_max),
        _format_optional(left_hard_min),
        _format_optional(left_hard_max),
        _format_optional(left_vel_limit),
        _format_optional(left_effort_limit),
        left_pos,
    )
    logger.info(
        "Right gripper limits soft=[%s, %s] hard=[%s, %s] vel=%s effort=%s current=%.4f",
        _format_optional(right_soft_min),
        _format_optional(right_soft_max),
        _format_optional(right_hard_min),
        _format_optional(right_hard_max),
        _format_optional(right_vel_limit),
        _format_optional(right_effort_limit),
        right_pos,
    )
    return {
        "left_soft_min": left_soft_min,
        "left_soft_max": left_soft_max,
        "right_soft_min": right_soft_min,
        "right_soft_max": right_soft_max,
        "left_hard_min": left_hard_min,
        "left_hard_max": left_hard_max,
        "right_hard_min": right_hard_min,
        "right_hard_max": right_hard_max,
        "left_vel_limit": left_vel_limit,
        "right_vel_limit": right_vel_limit,
        "left_effort_limit": left_effort_limit,
        "right_effort_limit": right_effort_limit,
    }

def _collect_pinch_metrics(retargeters):
    left = None
    right = None
    for retargeter in retargeters:
        if isinstance(retargeter, PinchGripperRetargeter):
            if retargeter.bound_hand == DeviceBase.TrackingTarget.HAND_LEFT:
                left = retargeter
            elif retargeter.bound_hand == DeviceBase.TrackingTarget.HAND_RIGHT:
                right = retargeter
    return left, right

def _first_value(value):
    if value is None:
        return None
    if torch.is_tensor(value):
        if value.numel() == 0:
            return None
        return float(value.view(-1)[0].item())
    return float(value)

def _get_joint_value(data, joint_ids):
    if data is None:
        return None
    try:
        return _first_value(data[:, joint_ids])
    except Exception:
        return None

def _format_log_value(value):
    if value is None:
        return ""
    return f"{float(value):.6f}"

def _write_pinch_log_line(
    pinch_log,
    left_vals,
    right_vals,
    gripper_open_pos,
    gripper_close_pos,
    left_gripper_target,
    right_gripper_target,
    left_gripper_qpos,
    right_gripper_qpos,
    left_gripper_qvel,
    right_gripper_qvel,
    left_sim_target,
    right_sim_target,
    left_vel_target,
    right_vel_target,
    left_effort_target,
    right_effort_target,
    left_applied_torque,
    right_applied_torque,
    action_source,
    gripper_limits=None,
):
    if pinch_log is None:
        return
    limits = gripper_limits or {}
    left_soft_min = limits.get("left_soft_min")
    left_soft_max = limits.get("left_soft_max")
    right_soft_min = limits.get("right_soft_min")
    right_soft_max = limits.get("right_soft_max")
    left_hard_min = limits.get("left_hard_min")
    left_hard_max = limits.get("left_hard_max")
    right_hard_min = limits.get("right_hard_min")
    right_hard_max = limits.get("right_hard_max")
    left_vel_limit = limits.get("left_vel_limit")
    right_vel_limit = limits.get("right_vel_limit")
    left_effort_limit = limits.get("left_effort_limit")
    right_effort_limit = limits.get("right_effort_limit")
    left_dist = left_vals.last_distance if left_vals is not None else None
    left_grip = left_vals.last_grip if left_vals is not None else None
    left_min = left_vals.min_distance if left_vals is not None else None
    left_max = left_vals.max_distance if left_vals is not None else None
    left_close = left_vals.last_close_distance if left_vals is not None else None
    left_open = left_vals.last_open_distance if left_vals is not None else None
    right_dist = right_vals.last_distance if right_vals is not None else None
    right_grip = right_vals.last_grip if right_vals is not None else None
    right_min = right_vals.min_distance if right_vals is not None else None
    right_max = right_vals.max_distance if right_vals is not None else None
    right_close = right_vals.last_close_distance if right_vals is not None else None
    right_open = right_vals.last_open_distance if right_vals is not None else None
    pinch_log.write(
        f"{time.time():.6f},"
        f"{_format_log_value(left_dist)},"
        f"{_format_log_value(left_grip)},"
        f"{_format_log_value(left_min)},"
        f"{_format_log_value(left_max)},"
        f"{_format_log_value(left_close)},"
        f"{_format_log_value(left_open)},"
        f"{_format_log_value(right_dist)},"
        f"{_format_log_value(right_grip)},"
        f"{_format_log_value(right_min)},"
        f"{_format_log_value(right_max)},"
        f"{_format_log_value(right_close)},"
        f"{_format_log_value(right_open)},"
        f"{_format_log_value(gripper_open_pos)},"
        f"{_format_log_value(gripper_close_pos)},"
        f"{_format_log_value(left_gripper_target)},"
        f"{_format_log_value(right_gripper_target)},"
        f"{_format_log_value(left_gripper_qpos)},"
        f"{_format_log_value(right_gripper_qpos)},"
        f"{_format_log_value(left_gripper_qvel)},"
        f"{_format_log_value(right_gripper_qvel)},"
        f"{_format_log_value(left_sim_target)},"
        f"{_format_log_value(right_sim_target)},"
        f"{_format_log_value(left_vel_target)},"
        f"{_format_log_value(right_vel_target)},"
        f"{_format_log_value(left_effort_target)},"
        f"{_format_log_value(right_effort_target)},"
        f"{_format_log_value(left_applied_torque)},"
        f"{_format_log_value(right_applied_torque)},"
        f"{_format_log_value(left_soft_min)},"
        f"{_format_log_value(left_soft_max)},"
        f"{_format_log_value(right_soft_min)},"
        f"{_format_log_value(right_soft_max)},"
        f"{_format_log_value(left_hard_min)},"
        f"{_format_log_value(left_hard_max)},"
        f"{_format_log_value(right_hard_min)},"
        f"{_format_log_value(right_hard_max)},"
        f"{_format_log_value(left_vel_limit)},"
        f"{_format_log_value(right_vel_limit)},"
        f"{_format_log_value(left_effort_limit)},"
        f"{_format_log_value(right_effort_limit)},"
        f"{action_source}\n"
    )

def _update_view_from_headcam(env) -> bool:
    if not hasattr(env, "head_camera"):
        return False
    if env.head_camera is None:
        return False
    cam_data = env.head_camera.data
    if cam_data.pos_w is None or cam_data.quat_w_world is None:
        return False
    pos_w = cam_data.pos_w[0]
    quat_w = cam_data.quat_w_world[0]
    # Camera forward axis depends on the sensor convention. Isaac Sim USD cameras use OpenGL convention (-Z forward).
    try:
        convention = env.cfg.head_camera_cfg.offset.convention
    except Exception:
        convention = "opengl"
    if convention == "ros":
        axis = (0.0, 0.0, 1.0)
    elif convention == "world":
        axis = (1.0, 0.0, 0.0)
    else:  # "opengl" (default)
        axis = (0.0, 0.0, -1.0)
    forward = quat_apply(quat_w.unsqueeze(0), torch.tensor([axis], device=env.device)).squeeze(0)
    eye = (pos_w).cpu().tolist()
    target = (pos_w + forward).cpu().tolist()
    env.sim.set_camera_view(eye=eye, target=target)
    return True


def main():
    if not is_handtracking:
        raise ValueError("This script only supports OpenXR handtracking teleop.")

    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )

    if env_cfg.xr is None:
        env_cfg.xr = XrCfg()
    # In XR/CloudXR, the view is driven by the XR headset pose. To "see from the robot head",
    # attach the XR anchor to the robot head camera prim.
    if args_cli.view_mode == "headcam" and args_cli.anchor_mode == "world":
        args_cli.anchor_mode = "headcam"
    if args_cli.anchor_mode == "headcam":
        env_cfg.xr.anchor_prim_path = "/World/envs/env_0/Robot/zed_link/head_cam/head_cam"
        # Follow the robot yaw smoothly so the XR view stays aligned while avoiding aggressive pitch/roll motion.
        env_cfg.xr.anchor_rotation_mode = XrAnchorRotationMode.FOLLOW_PRIM_SMOOTHED
        env_cfg.xr.anchor_rotation_smoothing_time = 0.25
        env_cfg.xr.fixed_anchor_height = False
        # Smooth XR requires frequent rendering (anchor sync runs in the render loop).
        env_cfg.sim.render_interval = 1
    env_cfg.sim.render.antialiasing_mode = "DLSS"
    if hasattr(env_cfg, "randomize_objects"):
        env_cfg.randomize_objects = args_cli.randomize_objects
    # Reduce env overhead for XR streaming unless explicitly requested.
    if hasattr(env_cfg, "debug_prints"):
        env_cfg.debug_prints = False
    if hasattr(env_cfg, "enable_cameras") and not getattr(args_cli, "keep_env_cameras", False):
        env_cfg.enable_cameras = False

    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    if env.num_envs != 1:
        logger.error("Teleop supports num_envs=1. Please re-run with --num_envs 1.")
        env.close()
        return

    gripper_limits = _log_gripper_limits(env)

    left_controller, left_arm_cfg = _setup_arm_ik(env, "left")
    right_controller, right_arm_cfg = _setup_arm_ik(env, "right")
    left_gripper_ids, _ = env.robot.find_joints(env.cfg.left_gripper_dof_name)
    right_gripper_ids, _ = env.robot.find_joints(env.cfg.right_gripper_dof_name)
    if len(left_gripper_ids) == 0 or len(right_gripper_ids) == 0:
        raise RuntimeError("Unable to resolve gripper joint IDs for hold-pose debug.")

    teleop_devices = DevicesCfg(
        devices={
            "handtracking": OpenXRDeviceCfg(
                xr_cfg=env_cfg.xr,
                sim_device=env.device,
                retargeters=[
                    Se3AbsRetargeterCfg(
                        bound_hand=DeviceBase.TrackingTarget.HAND_LEFT,
                        zero_out_xy_rotation=False,
                        use_wrist_rotation=True,
                        use_wrist_position=True,
                        enable_visualization=False,
                        sim_device=env.device,
                    ),
                    PinchGripperRetargeterCfg(
                        bound_hand=DeviceBase.TrackingTarget.HAND_LEFT,
                        open_distance=args_cli.pinch_open,
                        close_distance=args_cli.pinch_close,
                        invert=args_cli.gripper_invert,
                        adaptive_calibration=args_cli.pinch_calib,
                        offset_ratio=args_cli.pinch_offset_ratio,
                        open_offset=args_cli.pinch_open_offset,
                        close_offset=args_cli.pinch_close_offset,
                        open_ratio=args_cli.pinch_open_ratio,
                        close_ratio=args_cli.pinch_close_ratio,
                        min_range=args_cli.pinch_min_range,
                        min_decay=args_cli.pinch_min_decay,
                        max_decay=args_cli.pinch_max_decay,
                        enable_markers=args_cli.hand_markers,
                        marker_prim_path="/Visuals/hand_markers_left",
                        marker_radius=0.015,
                        sim_device=env.device,
                    ),
                    Se3AbsRetargeterCfg(
                        bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT,
                        zero_out_xy_rotation=False,
                        use_wrist_rotation=True,
                        use_wrist_position=True,
                        enable_visualization=False,
                        sim_device=env.device,
                    ),
                    PinchGripperRetargeterCfg(
                        bound_hand=DeviceBase.TrackingTarget.HAND_RIGHT,
                        open_distance=args_cli.pinch_open,
                        close_distance=args_cli.pinch_close,
                        invert=args_cli.gripper_invert,
                        adaptive_calibration=args_cli.pinch_calib,
                        offset_ratio=args_cli.pinch_offset_ratio,
                        open_offset=args_cli.pinch_open_offset,
                        close_offset=args_cli.pinch_close_offset,
                        open_ratio=args_cli.pinch_open_ratio,
                        close_ratio=args_cli.pinch_close_ratio,
                        min_range=args_cli.pinch_min_range,
                        min_decay=args_cli.pinch_min_decay,
                        max_decay=args_cli.pinch_max_decay,
                        enable_markers=args_cli.hand_markers,
                        marker_prim_path="/Visuals/hand_markers_right",
                        marker_radius=0.015,
                        sim_device=env.device,
                    ),
                ],
            ),
        }
    )

    should_reset = False
    teleoperation_active = False
    reanchor_pending = False
    left_pos_offset = None
    right_pos_offset = None
    left_rot_offset = None
    right_rot_offset = None
    left_target_marker = None
    right_target_marker = None
    left_ee_marker = None
    right_ee_marker = None

    def reset_cb() -> None:
        nonlocal should_reset, left_pos_offset, right_pos_offset, left_rot_offset, right_rot_offset
        should_reset = True
        left_pos_offset = None
        right_pos_offset = None
        left_rot_offset = None
        right_rot_offset = None
        print("Reset triggered - Environment will reset on next step")

    def start_cb() -> None:
        nonlocal teleoperation_active, reanchor_pending
        teleoperation_active = True
        reanchor_pending = True
        print("Teleoperation activated (re-anchoring on next frame)")

    def stop_cb() -> None:
        nonlocal teleoperation_active
        teleoperation_active = False
        print("Teleoperation deactivated")

    teleop_callbacks = {"RESET": reset_cb, "START": start_cb, "STOP": stop_cb}
    teleop_device = create_teleop_device("handtracking", teleop_devices.devices, teleop_callbacks)

    print(f"Using teleop device: {teleop_device}")

    env.reset()
    teleop_device.reset()
    # Only force the desktop viewport camera when not using XR. In XR, the view is controlled by the headset.
    if args_cli.view_mode == "headcam" and not app_launcher_args.get("xr", False):
        if hasattr(env, "head_camera") and env.head_camera is not None:
            env.head_camera.update(dt=env.sim.get_physics_dt())
        _update_view_from_headcam(env)

    if args_cli.visualize_targets:
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/ee_goal_left"
        left_target_marker = VisualizationMarkers(marker_cfg)
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/ee_goal_right"
        right_target_marker = VisualizationMarkers(marker_cfg)

        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.08, 0.08, 0.08)
        marker_cfg.prim_path = "/Visuals/ee_actual_left"
        left_ee_marker = VisualizationMarkers(marker_cfg)
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.08, 0.08, 0.08)
        marker_cfg.prim_path = "/Visuals/ee_actual_right"
        right_ee_marker = VisualizationMarkers(marker_cfg)

    left_offset_rpy = torch.tensor(args_cli.left_wrist_offset_rpy, device=env.device) * (torch.pi / 180.0)
    right_offset_rpy = torch.tensor(args_cli.right_wrist_offset_rpy, device=env.device) * (torch.pi / 180.0)
    left_wrist_offset_quat = quat_from_euler_xyz(
        left_offset_rpy[0].unsqueeze(0), left_offset_rpy[1].unsqueeze(0), left_offset_rpy[2].unsqueeze(0)
    )
    right_wrist_offset_quat = quat_from_euler_xyz(
        right_offset_rpy[0].unsqueeze(0), right_offset_rpy[1].unsqueeze(0), right_offset_rpy[2].unsqueeze(0)
    )

    if args_cli.gripper_open is None or args_cli.gripper_close is None:
        open_pos, close_pos = _infer_gripper_positions(env)
    else:
        open_pos = torch.tensor(args_cli.gripper_open, device=env.device)
        close_pos = torch.tensor(args_cli.gripper_close, device=env.device)

    pinch_log = None
    if args_cli.pinch_log_path:
        log_dir = os.path.dirname(args_cli.pinch_log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        pinch_log = open(args_cli.pinch_log_path, "w", buffering=1, encoding="utf-8")
        pinch_log.write(
            "timestamp,"
            "left_pinch_m,left_grip,left_min,left_max,left_close,left_open,"
            "right_pinch_m,right_grip,right_min,right_max,right_close,right_open,"
            "gripper_open_pos,gripper_close_pos,"
            "left_gripper_target,right_gripper_target,"
            "left_gripper_qpos,right_gripper_qpos,"
            "left_gripper_qvel,right_gripper_qvel,"
            "left_sim_target,right_sim_target,"
            "left_vel_target,right_vel_target,"
            "left_effort_target,right_effort_target,"
            "left_applied_torque,right_applied_torque,"
            "left_soft_min,left_soft_max,right_soft_min,right_soft_max,"
            "left_hard_min,left_hard_max,right_hard_min,right_hard_max,"
            "left_vel_limit,right_vel_limit,left_effort_limit,right_effort_limit,"
            "action_source\n"
        )

    while simulation_app.is_running():
        try:
            with torch.inference_mode():
                if teleoperation_active:
                    command = teleop_device.advance()
                    if command is None and not args_cli.hold_pose:
                        env.sim.render()
                        continue
                    left_vals = None
                    right_vals = None
                    if pinch_log is not None:
                        left_vals, right_vals = _collect_pinch_metrics(teleop_device._retargeters)

                    if args_cli.hold_pose:
                        hold_left = env.robot.data.joint_pos[:, left_arm_cfg.joint_ids]
                        hold_right = env.robot.data.joint_pos[:, right_arm_cfg.joint_ids]
                        hold_left_grip = env.robot.data.joint_pos[:, left_gripper_ids]
                        hold_right_grip = env.robot.data.joint_pos[:, right_gripper_ids]
                        actions = torch.cat([hold_left, hold_right, hold_left_grip, hold_right_grip], dim=-1)
                        env.step(actions)
                        if pinch_log is not None:
                            left_target = _first_value(actions[:, -2])
                            right_target = _first_value(actions[:, -1])
                            left_qpos = _first_value(env.robot.data.joint_pos[:, left_gripper_ids])
                            right_qpos = _first_value(env.robot.data.joint_pos[:, right_gripper_ids])
                            left_qvel = _first_value(env.robot.data.joint_vel[:, left_gripper_ids])
                            right_qvel = _first_value(env.robot.data.joint_vel[:, right_gripper_ids])
                            left_sim_target = _get_joint_value(env.robot.data.joint_pos_target, left_gripper_ids)
                            right_sim_target = _get_joint_value(env.robot.data.joint_pos_target, right_gripper_ids)
                            left_vel_target = _get_joint_value(env.robot.data.joint_vel_target, left_gripper_ids)
                            right_vel_target = _get_joint_value(env.robot.data.joint_vel_target, right_gripper_ids)
                            left_effort_target = _get_joint_value(env.robot.data.joint_effort_target, left_gripper_ids)
                            right_effort_target = _get_joint_value(env.robot.data.joint_effort_target, right_gripper_ids)
                            left_applied_torque = _get_joint_value(env.robot.data.applied_torque, left_gripper_ids)
                            right_applied_torque = _get_joint_value(env.robot.data.applied_torque, right_gripper_ids)
                            _write_pinch_log_line(
                                pinch_log,
                                left_vals,
                                right_vals,
                                _first_value(open_pos),
                                _first_value(close_pos),
                                left_target,
                                right_target,
                                left_qpos,
                                right_qpos,
                                left_qvel,
                                right_qvel,
                                left_sim_target,
                                right_sim_target,
                                left_vel_target,
                                right_vel_target,
                                left_effort_target,
                                right_effort_target,
                                left_applied_torque,
                                right_applied_torque,
                                "hold",
                                gripper_limits,
                            )
                        if args_cli.view_mode == "headcam" and not app_launcher_args.get("xr", False):
                            _update_view_from_headcam(env)
                        continue

                    command = command.view(-1)
                    if command.numel() != 16:
                        logger.error(
                            "Unexpected teleop command size: %d (expected 16).", command.numel()
                        )
                        env.sim.render()
                        continue

                    left_pose = command[0:7].view(1, 7)
                    left_grip = command[7:8]
                    right_pose = command[8:15].view(1, 7)
                    right_grip = command[15:16]

                    left_pos_w = left_pose[:, 0:3].repeat(env.num_envs, 1)
                    left_quat_w_raw = left_pose[:, 3:7].repeat(env.num_envs, 1)
                    right_pos_w = right_pose[:, 0:3].repeat(env.num_envs, 1)
                    right_quat_w_raw = right_pose[:, 3:7].repeat(env.num_envs, 1)

                    left_quat_w = left_quat_w_raw
                    right_quat_w = right_quat_w_raw

                    if reanchor_pending:
                        left_body_id = left_arm_cfg.body_ids[0]
                        right_body_id = right_arm_cfg.body_ids[0]
                        left_ee_pose_w = env.robot.data.body_state_w[:, left_body_id, 0:7]
                        right_ee_pose_w = env.robot.data.body_state_w[:, right_body_id, 0:7]
                        left_pos_offset = left_ee_pose_w[:, 0:3] - left_pos_w
                        right_pos_offset = right_ee_pose_w[:, 0:3] - right_pos_w
                        if args_cli.reanchor_rotation:
                            left_rot_offset = quat_mul(left_ee_pose_w[:, 3:7], quat_inv(left_quat_w_raw))
                            right_rot_offset = quat_mul(right_ee_pose_w[:, 3:7], quat_inv(right_quat_w_raw))
                        else:
                            left_rot_offset = None
                            right_rot_offset = None
                        reanchor_pending = False

                    if left_pos_offset is not None:
                        left_pos_w = left_pos_w + left_pos_offset
                        if left_rot_offset is not None:
                            left_quat_w = quat_mul(left_rot_offset, left_quat_w)
                    if right_pos_offset is not None:
                        right_pos_w = right_pos_w + right_pos_offset
                        if right_rot_offset is not None:
                            right_quat_w = quat_mul(right_rot_offset, right_quat_w)

                    left_quat_w = quat_mul(left_quat_w, left_wrist_offset_quat)
                    right_quat_w = quat_mul(right_quat_w, right_wrist_offset_quat)

                    if left_target_marker is not None:
                        left_target_marker.visualize(translations=left_pos_w, orientations=left_quat_w)
                    if right_target_marker is not None:
                        right_target_marker.visualize(translations=right_pos_w, orientations=right_quat_w)
                    if left_ee_marker is not None and right_ee_marker is not None:
                        left_body_id = left_arm_cfg.body_ids[0]
                        right_body_id = right_arm_cfg.body_ids[0]
                        left_ee_pose_w = env.robot.data.body_state_w[:, left_body_id, 0:7]
                        right_ee_pose_w = env.robot.data.body_state_w[:, right_body_id, 0:7]
                        left_ee_marker.visualize(
                            translations=left_ee_pose_w[:, 0:3], orientations=left_ee_pose_w[:, 3:7]
                        )
                        right_ee_marker.visualize(
                            translations=right_ee_pose_w[:, 0:3], orientations=right_ee_pose_w[:, 3:7]
                        )

                    left_joint_targets = _compute_arm_targets(
                        env.robot, left_arm_cfg, left_controller, left_pos_w, left_quat_w
                    )
                    right_joint_targets = _compute_arm_targets(
                        env.robot, right_arm_cfg, right_controller, right_pos_w, right_quat_w
                    )

                    left_gripper = _map_gripper_command(left_grip, open_pos, close_pos).view(1, 1)
                    right_gripper = _map_gripper_command(right_grip, open_pos, close_pos).view(1, 1)

                    actions = torch.cat(
                        [left_joint_targets, right_joint_targets, left_gripper, right_gripper], dim=-1
                    )
                    env.step(actions)
                    if pinch_log is not None:
                        left_target = _first_value(actions[:, -2])
                        right_target = _first_value(actions[:, -1])
                        left_qpos = _first_value(env.robot.data.joint_pos[:, left_gripper_ids])
                        right_qpos = _first_value(env.robot.data.joint_pos[:, right_gripper_ids])
                        left_qvel = _first_value(env.robot.data.joint_vel[:, left_gripper_ids])
                        right_qvel = _first_value(env.robot.data.joint_vel[:, right_gripper_ids])
                        left_sim_target = _get_joint_value(env.robot.data.joint_pos_target, left_gripper_ids)
                        right_sim_target = _get_joint_value(env.robot.data.joint_pos_target, right_gripper_ids)
                        left_vel_target = _get_joint_value(env.robot.data.joint_vel_target, left_gripper_ids)
                        right_vel_target = _get_joint_value(env.robot.data.joint_vel_target, right_gripper_ids)
                        left_effort_target = _get_joint_value(env.robot.data.joint_effort_target, left_gripper_ids)
                        right_effort_target = _get_joint_value(env.robot.data.joint_effort_target, right_gripper_ids)
                        left_applied_torque = _get_joint_value(env.robot.data.applied_torque, left_gripper_ids)
                        right_applied_torque = _get_joint_value(env.robot.data.applied_torque, right_gripper_ids)
                        _write_pinch_log_line(
                            pinch_log,
                            left_vals,
                            right_vals,
                            _first_value(open_pos),
                            _first_value(close_pos),
                            left_target,
                            right_target,
                            left_qpos,
                            right_qpos,
                            left_qvel,
                            right_qvel,
                            left_sim_target,
                            right_sim_target,
                            left_vel_target,
                            right_vel_target,
                            left_effort_target,
                            right_effort_target,
                            left_applied_torque,
                            right_applied_torque,
                            "teleop",
                            gripper_limits,
                        )
                    if args_cli.view_mode == "headcam" and not app_launcher_args.get("xr", False):
                        _update_view_from_headcam(env)
                else:
                    env.sim.render()
                    if args_cli.view_mode == "headcam" and not app_launcher_args.get("xr", False):
                        if hasattr(env, "head_camera") and env.head_camera is not None:
                            env.head_camera.update(dt=env.sim.get_physics_dt())
                        _update_view_from_headcam(env)

                if should_reset:
                    env.reset()
                    teleop_device.reset()
                    if args_cli.view_mode == "headcam" and not app_launcher_args.get("xr", False):
                        if hasattr(env, "head_camera") and env.head_camera is not None:
                            env.head_camera.update(dt=env.sim.get_physics_dt())
                        _update_view_from_headcam(env)
                    should_reset = False
        except Exception as e:
            logger.error("Error during teleop step: %s", e)
            break

    if pinch_log is not None:
        pinch_log.close()
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
