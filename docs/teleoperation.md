# Teleoperation bring-up

Goal: simplify first and prove that UDP teleop packets arrive. No IK or arm motion yet.

## Plan (staged)
- Stage 1 (now): launch Isaac Lab env, step with zero actions, log every UDP ee_targets packet.
- Stage 2 (next): wire the parsed packets into robot commands (precision/IK).
- Stage 3 (later): polish device UX, filtering, and add automated tests.

## Stage 1: UDP-only smoke test
1) Launch the minimal receiver + env (same CLI shape as the other *_agent scripts):  
`python submodules/gearboxAssembly/scripts/teleop_agent.py --task Template-Galaxea-Lab-External-Direct-v0 --num_envs 1 [--renderer RayTracedLighting ...]`  
What you should see in the console: observation/action spaces, `[UDP] Listening on ...`, and a `[HEARTBEAT]` every ~5 s showing packet counts.
   - It stubs out the built-in `rule_policy` so nothing scripted moves and waits for the first UDP packet before stepping.
2) Send canned packets to prove the UDP path:  
`python teleop/test_hardcoded_sender.py --ip 127.0.0.1 --port 5005 --delay 0.5`  
Expected on the teleop terminal: one `[UDP] seq=... arms=[...] from ('127.0.0.1', <port>)` per packet, then heartbeats like `udp_packets=4, last_seq=4`.
3) (Optional) Drive packets from a keyboard/gamepad to see higher rates (still logging only):  
`python teleop/keyboard_gamepad_sender.py --ip 127.0.0.1 --port 5005 --rate 60 --frame torso`  
Keyboard controls: Tab arm L/R, P precision toggle, WASD XY, E/Q Z, J/L yaw, I/K pitch, U/O roll, Z/X grip +/-.
Gamepad (pygame required, auto-detected): left stick XY, triggers drive ±Z and roll, right stick yaw/pitch, `B` precision, `LB/RB` grip +/-.
4) Troubleshooting: if no `[UDP]` lines appear, double-check IP/port/firewall; use `--log_payload` on teleop_agent to print full JSON for each datagram.

Notes
- The receiver listens on `0.0.0.0:5005` by default and steps the sim with zero actions so we can focus solely on UDP ingress.
- Packet schema used by the senders (`teleop/test_hardcoded_sender.py`, `teleop/keyboard_gamepad_sender.py`) is ee_targets v1:  
`{v:1,type:"ee_targets",seq:int,t:float,frame:str,precision:int,arms:[{id:"L|R",ee_frame:"<tcp>",p:[x,y,z],q:[w,x,y,z],grip:0..1}]}` (precision optional)
