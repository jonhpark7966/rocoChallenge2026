- [ ] Implement an `ee_targets` UDP receiver (packet parsing, seq/order checks, clutch/reset/precision flags, timeout/hold).
- [ ] Build a pose-to-joint converter layer (IK + mode handling) separate from UDP I/O; expose a clean API to the sim loop.
- [ ] Wire the converter into `assembly_scene_teleop_demo.py` and replace the joint-target path with EE-pose targets.
- [ ] Create a headless test harness that feeds hardcoded `ee_targets` packets (no keyboard/gamepad) and asserts joint target outputs.
- [ ] Update keyboard/gamepad sender to maintain EE state and emit `ee_targets` v1; keep a “canned” packet test mode.
- [ ] Add Vision Pro/6DoF client (optional) that reuses the same payload; map gesture→grip.
- [ ] Validate end-to-end in Isaac Sim (single/dual arm, modes, timeout/recovery) and document `frame`/`ee_frame` names.

## Goals
- Teleoperate the humanoid’s two arms and the parallel grippers inside Isaac Sim while assembling the gearbox.
- Use one unified UDP message (`ee_targets` v1) regardless of input device (keyboard, gamepad, Vision Pro).
- Keep the mid-term path practical: keyboard/gamepad parity first, Vision Pro/other 6DoF input later if needed.

## Snapshot of current code (does **not** match the target spec yet)
- `submodules/gearboxAssembly/scripts/basic/keyboard_udp_teleop.py`: sends joint deltas per keypress, accumulates absolute joint degrees, and streams `{left_arm_deg, right_arm_deg, left_gripper, right_gripper}` at ~30 Hz.
- `submodules/gearboxAssembly/scripts/basic/assembly_scene_teleop_demo.py`: UDP receiver (`UDPArmReceiver`) reads those joint-degree payloads, adds static joint offsets, sets joint position targets directly; no clutch/precision/reset/mode, and only time-gates packets with `max_age`.
- Result: the current loop is joint-space only, has no notion of EE pose, and can’t express insert/rotate constraints or the clutch/precision/reset semantics from the new spec.

## Target UDP message (ee_targets v1, summarized)
- State message (not deltas) at 30–60 Hz, UDP/IPv4, ≤1200B, latest `seq` wins.
- Header: `v=1`, `type="ee_targets"`, `seq` monotonic, `t` (sender timestamp, monotonic seconds recommended), `frame` (pose reference, prefer `torso`), `clutch` (0/1 deadman), `precision` (0/1 scale-down flag), `reset` (edge-trigger snap-to-current).
- Arms array (1–2 entries): `{id: "L"|"R", ee_frame: "<tcp_name>", p: [x,y,z] m, q: [w,x,y,z] unit quaternion, grip: 0..1 (0 close,1 open), mode: "free"|"insert"|"rotate"}`.
- Modes are advisory; receiver applies constraints (alignment, axis filtering, roll focus) per mode.
- Receiver rules: drop stale/out-of-order `seq`, hold on timeout, clutch=0 blocks updates, reset snaps target to current EE pose (and should re-seed IK), missing arms keep last target.

## Detailed plan (receiver, conversion, senders, tests)
1) **UDP receiver module**
   - Build a standalone class that parses `ee_targets` JSON, enforces `v/type`, monotonic `seq`, and drops stale packets.
   - Track `clutch/precision/reset` flags, `frame`, and per-arm targets; hold last valid state on timeout.
   - Expose a thread-safe getter that returns the latest state + an edge-triggered reset indicator.
2) **Pose→joint conversion layer**
   - API: `convert_targets(state) -> per-arm joint targets + gripper`, taking `frame` and `mode` into account.
   - Use `DifferentialIKController` for pose tracking; inject velocity limits per mode (`free/insert/rotate`) and precision scaling.
   - Add receiver-side LPFs (`alpha_p=0.2`, `alpha_q=0.2`, `alpha_grip=0.3`) and grip deadband (0.02).
   - Implement reset handling (snap to current EE pose, re-seed IK), and mode hooks for alignment/insertion-axis constraints.
3) **Simulation integration**
   - Replace the joint-degree path in `assembly_scene_teleop_demo.py` with the new receiver + converter outputs.
   - Keep scripted/rule-based mode as a fallback; gate teleop application with `clutch` and timeout/hold.
   - Log incoming targets, filtered targets, and final joint commands for debugging/recording.
4) **Test harness (no input devices)**
   - Add a headless test that feeds hardcoded `ee_targets` packets to the receiver (via UDP or direct call), exercises `seq`, timeout, clutch=0, reset edge, precision=1, and modes (`free/insert/rotate`).
   - Assert joint targets stay stable on timeout, snap on reset, and obey gripper deadband; measure latency of parsing→conversion.
   - Keep a canned-packet replay script to run inside Isaac Sim for visual verification before plugging keyboards/gamepads.
5) **Input clients**
   - Keyboard/gamepad: maintain local EE state; key/gamepad events nudge deltas, sender emits absolute state with `seq`++ at 60 Hz. Map: `Tab` active-arm toggle, `Space` clutch (hold), `Shift` precision, `Backspace` reset, translational/rotational/grip keys as below.
   - Vision Pro/6DoF (optional): stream head/hand poses, map to torso frame, apply gesture→grip; reuse the same sender payload.
6) **End-to-end validation**
   - Scenarios: single-arm free, dual-arm free, insert/rotate alignment/roll emphasis, clutch hold/release, precision toggle, reset edge, timeout hold, gripper deadband.
   - Verify `frame`/`ee_frame` names against the USD and document them.

## New pre–Vision Pro code
- Root repo (senders + tests): `teleop/keyboard_gamepad_sender.py`, `teleop/test_hardcoded_sender.py`, `teleop/math_utils.py`.
- gearboxAssembly submodule (agent side): `scripts/teleop_utils/ee_targets.py`, `scripts/teleop_utils/filters.py`, `scripts/teleop_utils/math_utils.py`.
- Isaac scene (teleop receiver): `scripts/basic/assembly_scene_teleop_demo.py` now consumes ee_targets v1 via `EETargetsReceiver` → `PoseCommandFilter` → DiffIK.

## Usage (pre–Vision Pro)
### 0) Hardcoded smoke test (no devices)
1. Start your receiver/bridge (e.g., Isaac scene listening on UDP 5005).
2. Run: `python teleop/test_hardcoded_sender.py --ip 127.0.0.1 --port 5005`
3. Observe seq/packet flow and confirm the receiver holds on clutch=0, applies reset edge, and handles insert/rotate payloads.

### 1) Keyboard + optional gamepad sender (ee_targets v1)
Run:
```
python teleop/keyboard_gamepad_sender.py --ip 127.0.0.1 --port 5005 --rate 60 --frame torso
```
Controls (latched toggles in this script):
- Tab: active arm toggle (L/R)
- Space: clutch toggle (1/0)
- P: precision toggle (1/0)
- Backspace: reset edge (1 frame)
- WASD: XY, E/Q: Z
- J/L: yaw, I/K: pitch, U/O: roll
- Z/X: grip +/- (0..1)
Gamepad (if pygame + controller present): left stick XY, triggers for ±Z/roll, A=clutch, B=precision, X=reset, LB/RB grip.

### 2) Isaac Sim teleop (gearboxAssembly/teleop branch)
Run the scene (after switching submodule to the `teleop` branch):
```
python submodules/gearboxAssembly/scripts/basic/assembly_scene_teleop_demo.py
```
The script now:
- Listens on UDP 5005 for ee_targets v1 (clutch/precision/reset/seq/arms)
- Filters via `PoseCommandFilter` (LPF + deadband + timeout/hold + reset snap)
- Feeds per-arm pose into DiffIK to set joint targets and grippers (0..1 scaled to ~0.04)
- Leaves scripted/rule-based mode intact; toggle teleop with `T` in the terminal

### 3) Validation ideas
- Use `teleop/test_hardcoded_sender.py` to prove seq ordering, clutch hold, reset edge, and per-mode payloads.
- Use `teleop/keyboard_gamepad_sender.py` to check continuous control and gripper deadband; verify the receiver drops stale packets and times out to hold.
- Log filtered pose targets + final joint commands to compare IK output vs. requested EE targets.

### 3) Validation ideas
- Use `teleop/test_hardcoded_sender.py` to prove seq ordering, clutch hold, reset edge, and per-mode payloads.
- Use `teleop/keyboard_gamepad_sender.py` to check continuous control and gripper deadband; verify the receiver drops stale packets and times out to hold.
- Log filtered pose targets + final joint commands to compare IK output vs. requested EE targets.

## Suggested key/gamepad mapping (sender side)
- Global: `Tab` toggle active arm (L/R), `Space` clutch (hold-to-send), `Shift` precision, `Backspace` reset.
- Pose nudges: WASD/Arrow for XY, `E/Q` for Z, `J/L` yaw, `I/K` pitch, `U/O` roll (adjust as needed).
- Gripper: `Z` open, `X` close (scale to 0..1 with deadband 0.02). Gamepad: bumpers for clutch/precision, sticks for translation/rotation, triggers for grip.

## Notes on inconsistencies & how to improve the provided spec
- Define the `precision` scale factors numerically in the spec (e.g., 0.05 m/s, 0.3 rad/s as “precision on”) to avoid divergent implementations.
- Clarify `frame`/`ee_frame` discovery: provide canonical names for the robot (e.g., `torso`, `left_gripper_tcp`, `right_gripper_tcp`) and a handshake/health message to confirm alignment.
- Specify `seq` behavior on wrap/restart (e.g., allow reset to 0 with a version bump or explicit “session_id”).
- Call out expected `t` clock domain (monotonic vs wall) and whether receivers should apply clock-skew correction or just use `seq`.
- Consider adding an optional `inputs` debug block (e.g., raw joystick/gesture values) to aid tooling, and an optional `heartbeat`/`estop` flag for safer teleop.
- For `insert/rotate`, include a short note on how the receiver determines the insertion axis (e.g., from model metadata) to keep sender implementations simple.
