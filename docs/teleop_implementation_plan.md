# Galaxea R1 Teleoperation Implementation Plan

## 1. Current State Analysis

### 1.1 Identified Issues

| Issue | Root Cause | Priority |
|-------|-----------|----------|
| **Gears flying on reset** | Physics instability during object placement | High |
| **Gripper not reopening** | Pinch calibration or retargeting logic issue | High |
| **Wrist rotation mapping** | EE coordinate frame mismatch between XR and robot | High |

### 1.2 Key Findings

#### Gears Physics Issue
- Table height: `0.9m`
- Gear initial z position: `0.92m` (2cm above table)
- Gear physics settings:
  - `max_depenetration_velocity=0.5` (may be too aggressive)
  - `max_contact_impulse=0.5`
  - `solver_iteration_count=16`
- **Hypothesis**: Objects may penetrate during randomization and get ejected violently

#### Gripper Control Issue
- Gripper joint: `left_gripper_axis1`, `right_gripper_axis1`
- Range: `0.0` (closed) to `0.04` (open)
- IsaacLab default pinch thresholds: `3cm-5cm` hysteresis
- **Hypothesis**: Pinch mapping may be inverted or thresholds incorrect

#### Wrist Rotation Issue
- Se3AbsRetargeter applies 90-degree X-axis rotation
- Robot EE uses standard robotics convention
- Camera convention: OpenGL (Z-backward)
- **Hypothesis**: Need additional rotation offset for Galaxea R1 EE frame

---

## 2. Environment Configuration Summary

### 2.1 Robot Configuration
```
Robot: Galaxea R1 (r1_DVT_colored_cam_pos.usd)
- Left arm: left_arm_joint[1-6] + left_gripper_axis1
- Right arm: right_arm_joint[1-6] + right_gripper_axis1
- EE links: left_arm_link6, right_arm_link6
- Torso: torso_joint[1-3] (fixed during teleop)
```

### 2.2 Action Space (14D)
```
[0-5]:   Left arm joint positions (6 DOF)
[6-11]:  Right arm joint positions (6 DOF)
[12]:    Left gripper position
[13]:    Right gripper position
```

### 2.3 Cameras
| Camera | Link Path | Convention |
|--------|-----------|------------|
| Head | `zed_link/head_cam` | OpenGL |
| Left Hand | `left_realsense_link/left_hand_cam` | OpenGL |
| Right Hand | `right_realsense_link/right_hand_cam` | OpenGL |

---

## 3. Implementation Plan

### Phase 1: Fix Gear Physics (Priority: High)

**Goal**: Prevent gears from flying on reset

**Tasks**:
1. Increase physics stability settings:
   ```python
   max_depenetration_velocity = 0.1  # Reduce from 0.5
   solver_position_iteration_count = 32  # Increase from 16
   solver_velocity_iteration_count = 32
   ```

2. Add simulation settling steps after reset:
   ```python
   def _reset_idx(self, env_ids):
       # ... existing reset code ...

       # Settle physics before starting
       for _ in range(10):
           self.sim.step(render=False)
           for obj in self.obj_dict.values():
               obj.update(self.sim.get_physics_dt())
   ```

3. Verify object placement doesn't penetrate table:
   - Table surface z = 0.9m
   - Gear center z = 0.92m (2cm margin)
   - Check gear collision mesh extends below center

### Phase 2: Fix Gripper Control (Priority: High)

**Goal**: Proper pinch-to-gripper mapping with reliable open/close

**Tasks**:
1. Use IsaacLab's built-in GripperRetargeter as reference:
   ```python
   GRIPPER_CLOSE_METERS = 0.03  # 3cm
   GRIPPER_OPEN_METERS = 0.05   # 5cm
   ```

2. Verify gripper mapping direction:
   ```python
   # grip=0 -> open (0.04), grip=1 -> close (0.0)
   gripper_target = open_pos + grip * (close_pos - open_pos)
   # If inverted: gripper_target = close_pos + grip * (open_pos - close_pos)
   ```

3. Add debug logging for pinch values:
   ```python
   logger.info(f"Pinch dist: {dist:.4f}m, Grip: {grip:.2f}, Target: {gripper_target:.4f}")
   ```

### Phase 3: Fix Wrist Rotation Mapping (Priority: High)

**Goal**: Correct hand-to-EE rotation alignment

**Tasks**:
1. Analyze Se3AbsRetargeter rotation transform:
   ```python
   # Current: 90-degree X-axis rotation
   final_rot = base_rot * Rotation.from_euler("x", 90, degrees=True)
   ```

2. Determine Galaxea R1 EE frame convention:
   - Measure EE orientation at known poses
   - Compare with XR hand orientation

3. Add wrist offset calibration:
   ```python
   --left_wrist_offset_rpy 0 0 0   # Adjust as needed
   --right_wrist_offset_rpy 0 0 0
   ```

4. Implement runtime reanchoring:
   - On teleop start, capture current robot EE pose
   - Calculate offset between XR hand and robot EE
   - Apply offset to all subsequent commands

### Phase 4: Camera Data Collection (Priority: Medium)

**Goal**: Save head_cam and wrist_cam data during teleop

**Tasks**:
1. Create data recorder class:
   ```python
   class TeleopDataRecorder:
       def __init__(self, save_dir: str):
           self.save_dir = save_dir
           self.episode_idx = 0

       def save_frame(self, obs: dict, action: torch.Tensor, timestamp: float):
           # Save RGB, depth, joint states, action
   ```

2. Integrate with teleop loop:
   ```python
   if recording_enabled:
       recorder.save_frame(
           obs=env.obs,
           action=actions,
           timestamp=time.time()
       )
   ```

3. Save in HDF5 format (compatible with existing code):
   ```python
   with h5py.File(filename, 'w') as f:
       f.create_dataset('head_rgb', data=head_rgb_stack)
       f.create_dataset('left_hand_rgb', data=left_hand_rgb_stack)
       f.create_dataset('actions', data=action_stack)
   ```

---

## 4. File Structure

```
scripts/
└── teleop/
    ├── teleop_r1_xr.py          # Main teleop script (new)
    ├── data_recorder.py          # Camera/action data recording
    └── pinch_calibrator.py       # Pinch threshold calibration tool

submodules/gearboxAssembly/source/.../
├── tasks/direct/galaxea_lab_agent/
│   ├── galaxea_lab_agent_env.py       # Modify _reset_idx for physics settling
│   └── galaxea_lab_agent_env_cfg.py   # Add teleop_devices config
└── robots/
    └── gears_assets.py                # Adjust physics parameters
```

---

## 5. Testing Checklist

### Phase 1 Tests
- [ ] Reset environment 10 times, verify no gears fly off
- [ ] Check gear positions remain on table after reset
- [ ] Verify physics settling with visual inspection

### Phase 2 Tests
- [ ] Pinch fingers slowly: gripper closes smoothly
- [ ] Open fingers: gripper opens reliably
- [ ] Verify hysteresis prevents chattering
- [ ] Check pinch_log.csv for correct values

### Phase 3 Tests
- [ ] Rotate wrist in XR: robot wrist follows correctly
- [ ] Test all 3 axes: roll, pitch, yaw
- [ ] Verify reanchoring works on teleop start

### Phase 4 Tests
- [ ] Record 1-minute session
- [ ] Verify HDF5 file contains all data
- [ ] Check image quality and alignment

---

## 6. Reference Code Locations

| Component | File |
|-----------|------|
| IsaacLab teleop example | `submodules/IsaacLab/scripts/environments/teleoperation/teleop_se3_agent.py` |
| Se3AbsRetargeter | `submodules/IsaacLab/source/isaaclab/isaaclab/devices/openxr/retargeters/manipulator/se3_abs_retargeter.py` |
| GripperRetargeter | `submodules/IsaacLab/source/isaaclab/isaaclab/devices/openxr/retargeters/manipulator/gripper_retargeter.py` |
| Environment config | `submodules/gearboxAssembly/.../tasks/direct/galaxea_lab_agent/galaxea_lab_agent_env_cfg.py` |
| Robot config | `submodules/gearboxAssembly/.../robots/galaxea_robots.py` |
| Gear physics | `submodules/gearboxAssembly/.../robots/gears_assets.py` |

---

## 7. Questions for User

Before implementing, please clarify:

1. **Gear physics**: Should we modify the gear physics parameters in `gears_assets.py`, or create a teleop-specific environment variant?

2. **Data format**: For camera recording, should we use the existing HDF5 format from the environment, or a different format?

3. **Wrist rotation**: Did the IsaacLab baseline example (`Isaac-PickPlace-GR1T2-Abs-v0`) work correctly with wrist rotation? If so, we can compare the EE frame conventions.

4. **Gripper behavior**: When you say "gripper doesn't reopen", does the grip value in the log stay at 1.0, or does it return to 0.0 but the gripper joint doesn't move?
