# Teleoperation plan

## Goals
- Drive the Galaxea arms and parallel grippers from a single UDP message format (`ee_targets` v1) regardless of input device.
- Keep the agent launch UX identical to the other `*_agent.py` scripts (`--task` from `list_envs.py`, `--num_envs`, renderer flags).
- Build in small, testable steps: first prove packet ingress, then gripper-only actuation, then full 6DoF EE tracking.

## Current state (arms + grippers via IK)
- `scripts/teleop_agent.py` mirrors `random_agent.py` bring-up, stubs any built-in `rule_policy`, and **waits for the first UDP packet** before stepping.
- UDP receiver: `teleop_utils/ee_targets.py` parses `ee_targets` v1 and drops stale packets; filtering via `teleop_utils/filters.PoseCommandFilter`.
- Applied control: per-arm Differential IK (target link6) with gripper open/close (0..1 scaled to ~0.04 m). Supported frames: `world` (direct) and `torso` (transformed from robot root); others log a warning and are treated as world.
- Logs: every new `seq` prints frame/precision plus per-arm XYZ/quat/grip, plus 5 s heartbeats showing last packet age/seq. First packet gate keeps the sim paused until data arrives.

## Usage (today)
1) Launch the teleop agent (same shape as other agents):
```
python submodules/gearboxAssembly/scripts/teleop_agent.py \
  --task Template-Galaxea-Lab-External-Direct-v0 --num_envs 1 [--renderer RayTracedLighting ...]
```
Expected: observation/action space prints, `[INFO] teleop rule_policy installed (arms + grippers via IK).`, `[UDP] ...` then heartbeats while it waits. A state responder listens on `--state_port` (default 5006) for `ee_state_request` and replies with current world-frame EE poses + grippers.

2) Smoke-test UDP ingress with canned packets:
```
python teleop/test_hardcoded_sender.py --ip 127.0.0.1 --port 5005 --delay 0.5
```
Teleop terminal should print per-arm XYZ/quat/grip per packet, then heartbeats with `last_seq`.

3) Keyboard/gamepad sender for manual packets (arms and grip applied, seeded from live state):
```
python teleop/keyboard_gamepad_sender.py --ip 127.0.0.1 --port 5005 --state-port 5006 --rate 60 --frame torso
```
On startup the sender asks the agent for `ee_state` and uses that as its baseline (prevents arm drop); if it fails, it falls back to the baked defaults. Controls: Tab arm L/R, WASD/EQ for XYZ, J/L/I/K/U/O for yaw/pitch/roll, Z/X grip +/- (gamepad: sticks for pose, LB/RB grip).

## Next polish items
- Add optional velocity/position clamping and precision scaling inside the IK bridge.
- Improve frame handling for additional named frames (`left_gripper_tcp`, etc.) if the USD exposes them.
- Timeouts: currently we “hold last target” when packets stop; consider a configurable hold-or-freeze option.

## ee_targets v1 schema (current trimmed version)
- Header: `v=1`, `type="ee_targets"`, `seq` (monotonic), `t` (sender timestamp), `frame` (pose reference, e.g., `torso`), `precision` (optional int, defaults to 0).
- Arms array (1–2 entries): `{id: "L"|"R", ee_frame: "<tcp_name>", p: [x,y,z] meters, q: [w,x,y,z] unit quaternion, grip: 0..1}`.
- Fields removed for now: `clutch`, `reset`, `mode`, `active`. We can reintroduce them later if needed.

## Troubleshooting
- No `[UDP]` lines: check IP/port, firewalls; use `--log_payload` to print full JSON per datagram.
- Scripted motion appearing: ensure `teleop_agent.py` is launched (it stubs `rule_policy`); other demos may still run scripted policies.
- If arms collapse once 6DoF is wired in, verify frame alignment (`--frame world|torso`) and add clamping before IK.
