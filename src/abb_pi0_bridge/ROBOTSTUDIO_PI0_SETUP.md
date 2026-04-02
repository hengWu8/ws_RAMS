# RobotStudio + pi0 Server-Side Setup

This file describes the server-side preparation that can be done from `ws_RAMS` for RobotStudio and pi0/openpi integration.

Important limitation:

- The server can prepare ROS2 launch/config/policy pieces.
- The server cannot directly click through RobotStudio on your local Windows machine.
- Your local RobotStudio virtual controller still needs to expose network access that this server can reach.

Important network note:

- ABB EGM uses UDP. A reverse SSH tunnel such as `ssh -R 27897:127.0.0.1:7897 ...` only forwards TCP and does not replace the direct UDP path required by EGM joint streaming.
- For EGM, the RobotStudio virtual controller should send UDP directly to this server's reachable IP and the configured EGM port, which remains `6515` by default.

## What is now available on the server

- `abb_pi0_bridge` can run with a real HTTP policy backend.
- A local HTTP hold-position stub is available as `pi0_policy_stub_server`.
- A dedicated RobotStudio launch is available:

```bash
ros2 launch abb_pi0_bridge robotstudio_pi0.launch.py robotstudio_rws_ip:=<YOUR_ROBOTSTUDIO_IP>
```

## Recommended bringup order

1. On your local PC:
   Start RobotStudio and the virtual controller.
2. On your local PC:
   Make sure the virtual controller IP is reachable from this server.
3. On your local PC:
   Configure the RobotStudio side so its EGM/RWS settings match the ROS launch values you pass in.
4. On this server:
   Source your ROS2 workspace and launch the RobotStudio bringup.

## Server-side commands

Optional preflight before launching:

```bash
# Optional but recommended when you use SSH proxy tunnels:
# unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy

ros2 run abb_pi0_bridge robotstudio_preflight \
  --rws-ip <YOUR_ROBOTSTUDIO_IP> \
  --rws-port <YOUR_RWS_TCP_PORT> \
  --policy-health-url http://127.0.0.1:8000/healthz
```

`robotstudio_preflight` ignores environment HTTP proxies by default to reduce false positives/negatives on local network checks. If you intentionally need proxy-based probing for the policy health URL, add:

```bash
--use-env-proxy
```

Launch the RobotStudio bringup plus the local policy stub:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash
ros2 launch abb_pi0_bridge robotstudio_pi0.launch.py \
  robotstudio_rws_ip:=<YOUR_ROBOTSTUDIO_IP> \
  robotstudio_rws_port:=<YOUR_RWS_TCP_PORT> \
  launch_policy_stub:=true \
  publish_commands:=false \
  policy_stub_mode:=left \
  policy_stub_step_rad:=0.1
```

Switch the bridge into streaming mode:

```bash
ros2 service call /abb_pi0_bridge/activate_streaming_mode std_srvs/srv/Trigger "{}"
```

Arm low-level streaming output only after you have confirmed RobotStudio is the target you want to drive:

```bash
ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool "{data: true}"
```

Return control to MoveIt / `click_to_move`:

```bash
ros2 service call /abb_pi0_bridge/activate_click_to_move_mode std_srvs/srv/Trigger "{}"
```

## RobotStudio-side checklist

The exact UI steps depend on your RobotStudio/ABB package version, but the key requirements are:

1. The virtual controller must expose an RWS IP that this server can reach.
2. The RWS TCP port must match the launch/config value, typically `80` unless you intentionally forward it to another port.
3. The EGM UDP port must match the launch/config value, currently default `6515`.
4. Windows firewall or host networking must allow traffic between your PC and this server.
5. The virtual controller should use the same robot model / joint layout expected by `workcell_description`.
6. Only enable streaming mode after verifying RobotStudio is connected and stable.
7. If you are unsure about the network path, run `ros2 run abb_pi0_bridge robotstudio_preflight ...` from the server first.

## RobotStudio RAPID / EGM configuration

Your RAPID program structure is compatible with the current server-side setup:

- `EGMSetupUC ROB_1, egm_id, "default", "ROB_1", \Joint;`
- `EGMActJoint ...`
- `EGMRunJoint ...`

On the RobotStudio side, align `ROB_1` with these values:

1. Set the UC device remote address to this server's IP. Based on your SSH example, that appears to be `192.168.1.102`.
2. Keep the UDP port at `6515` unless you intentionally change both RobotStudio and ROS2 together.
3. Keep the EGM mode as joint-space, which matches the current `abb_pi0_bridge` command path.

For RWS on the ROS2 side:

- `robotstudio_rws_ip` should be the virtual controller IP that the server can reach over TCP.
- `robotstudio_rws_port` should be the TCP port the server can actually reach, usually `80`.
- This is not necessarily `192.168.1.102`. `192.168.1.102` is the server IP from your SSH example, while `robotstudio_rws_ip` should point at the RobotStudio virtual controller or the Windows host address that exposes it.

## Known recovery case: direct commands stop moving until RAPID is restarted

Observed behavior during RobotStudio testing:

- RWS remained reachable.
- EGM UDP traffic on port `6515` remained bi-directional.
- ROS2 controllers stayed active and `/forward_command_controller_position/commands` continued to receive non-zero targets.
- Even direct test commands such as `[0.5, 0, 0, 0, 0, 0]` no longer produced motion.
- Restarting the RobotStudio RAPID program restored motion immediately.

What this most likely means:

- The failure is not necessarily a raw TCP/UDP connectivity problem.
- The virtual controller can remain "connected" while the current EGM motion session is no longer consuming incoming references.
- After repeated stop/start cycles, `41830` errors, or mismatched launch timing between ROS2 bringup and `EGMRunJoint`, the controller can end up in a stale EGM execution state.
- Restarting RAPID re-runs `EGMGetId -> EGMSetupUC -> EGMActJoint -> EGMRunJoint`, which effectively refreshes the EGM session and restores motion execution.

Practical interpretation:

- If RWS is up and UDP packets are still flowing but both `pi0` commands and direct controller commands stop moving the robot, do not assume the policy output is the root cause.
- First suspect that the active RAPID/EGM session is stale and needs to be restarted from the RobotStudio side.

Recommended recovery order:

1. Stop ROS2 streaming output or set `command_output_armed=false`.
2. In RobotStudio, stop the RAPID program.
3. Perform `PP to Main`.
4. Start the RAPID program again so it re-enters `EGMRunJoint`.
5. Re-run a direct controller command baseline before blaming `pi0`.

This recovery step is especially important before any A/B comparison between:

- direct `/forward_command_controller_position/commands` publishing, and
- `pi0 -> HTTP adapter -> abb_pi0_bridge -> controller`

Without resetting RAPID first, the comparison can be misleading because both paths may fail for the same stale-session reason.

Example launch if your virtual controller is reachable at `192.168.1.50`:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash
ros2 launch abb_pi0_bridge robotstudio_pi0.launch.py \
  robotstudio_rws_ip:=192.168.1.50 \
  robotstudio_rws_port:=80 \
  egm_port:=6515 \
  launch_policy_stub:=true \
  publish_commands:=false
```

Example if you expose RWS to the server through a TCP tunnel on port `27897`:

```bash
ros2 run abb_pi0_bridge robotstudio_preflight \
  --rws-ip 127.0.0.1 \
  --rws-port 27897 \
  --egm-port 6515 \
  --policy-health-url http://127.0.0.1:8000/healthz

ros2 launch abb_pi0_bridge robotstudio_pi0.launch.py \
  robotstudio_rws_ip:=127.0.0.1 \
  robotstudio_rws_port:=27897 \
  egm_port:=6515 \
  launch_policy_stub:=true \
  publish_commands:=false
```

## pi0 status

This is not a full pi0 deployment yet.

- The bridge-side HTTP contract is now defined.
- The local stub server lets you test the full request/response path safely.
- The next step is to replace the hold-position stub with a real pi0/openpi policy service.

## Minimal real openpi dry-run

This workspace now includes a minimal ABB-to-openpi HTTP adapter at:

`/home/heng/workspace/ws_RAMS/tools/openpi_http_adapter.py`

Recommended first step:

1. Keep `publish_commands:=false` so the robot does not move.
2. Start the adapter inside the `openpi_official` Python environment.
3. Point `abb_pi0_bridge` at `http://127.0.0.1:8001/infer`.
4. Inspect `/abb_pi0_bridge/status` and the adapter `/healthz` before allowing any motion.

Example adapter launch:

```bash
cd /home/heng/workspace/ws_RAMS
/home/heng/workspace/openpi_official/.venv/bin/python \
  tools/openpi_http_adapter.py \
  --host 127.0.0.1 \
  --port 8001 \
  --config-name pi05_libero \
  --checkpoint-dir /home/heng/workspace/openpi_official/checkpoints/pi05_libero_pytorch \
  --pytorch-device auto
```

Example RobotStudio bridge launch against the real-model adapter, still in dry-run mode:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash
ros2 launch abb_pi0_bridge robotstudio_pi0.launch.py \
  robotstudio_rws_ip:=127.0.0.1 \
  robotstudio_rws_port:=28080 \
  egm_port:=6515 \
  launch_policy_stub:=false \
  policy_server_url:=http://127.0.0.1:8001/infer \
  publish_commands:=false
```

Notes:

- The adapter currently uses ABB joint positions as a low-dimensional state, zero images, and a fixed prompt. This is only for validating the real-model request/response path.
- It does **not** mean the selected checkpoint is semantically aligned with the ABB IRB6700.
- Real camera inputs and an ABB-specific observation/action mapping are still needed before meaningful closed-loop robot control with pi0.
- If your machine has an NVIDIA GPU, prefer `--pytorch-device auto` or an explicit `cuda:N` device. CPU loading can be very slow and may exhaust host memory on large checkpoints.

## Automation idea for later

The observed RAPID restart requirement suggests a small recovery helper would be valuable. A future automation utility could:

1. Query RWS for `ctrlstate` and RAPID execution state.
2. Confirm the ROS2 side is listening on the expected EGM port.
3. Send a short direct controller baseline command and observe whether `/joint_states` changes.
4. If commands are being published but joint feedback stays flat, warn that the EGM session is likely stale.
5. Optionally drive a scripted recovery sequence:
   - disarm `abb_pi0_bridge`
   - request RAPID stop through RWS
   - request `PP to Main`
   - request RAPID start
   - re-run the direct-command baseline

In other words, the useful automation target is not just "launch pi0", but "verify motion execution and recover stale RAPID/EGM sessions before enabling pi0 output".
