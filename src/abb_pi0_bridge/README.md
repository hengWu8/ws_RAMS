# abb_pi0_bridge

`abb_pi0_bridge` is a new, additive package for preparing `ws_RAMS` for future pi0/openpi deployment on the ABB IRB6700 stack.

## Phase-1 scope

- Keep the existing `click_to_move` and MoveIt path unchanged.
- Subscribe to robot observations from `/joint_states`.
- Optionally subscribe to fixed and wrist camera RGB topics.
- Assemble a policy-friendly observation object.
- Support both a mock policy and a real HTTP JSON policy server stub.
- Apply safety filters before any command leaves the node.
- Publish low-level streaming-style commands to a configurable topic.
- Arbitrate between `click_to_move` trajectory mode and streaming mode with a shared control-mode channel.

## Current architecture

The bridge node is intentionally split into small layers:

1. Observation assembly:
   Order joint state data into a fixed ABB joint vector and optionally attach camera frames.
2. Policy adapter:
   `mock_hold` is available for dry-run work, and `http_json` is a real server client stub for future policy serving.
3. Mode arbitration:
   The bridge publishes a shared control mode on `/abb/control_mode`, and `click_to_move` only executes in `trajectory` mode.
4. Safety filter:
   Clamp to joint limits, reject invalid values, and limit per-step motion.
5. Command output:
   Publish `std_msgs/msg/Float64MultiArray` commands to a low-level controller topic.

This keeps the policy integration seam clear without forcing MoveIt planning on every control tick.

## Safety defaults

- `publish_commands` defaults to `false`.
- `control_mode` defaults to `trajectory`, so existing `click_to_move` behavior stays enabled unless you switch modes.
- Streaming output must be both opted in (`publish_commands=true`) and armed via service or parameter.
- The mock policy returns the current joint positions, so the default behavior is observe-only / hold-current.
- Commands are rate-limited by `max_position_step_rad`.
- Commands are clamped to ABB joint limits from the current workcell description.
- pi0 bridge commands are additionally constrained by a Cartesian TCP workspace guard: by default the TCP must remain within `0.5 m` of the pose captured when streaming mode/output is handed to pi0.
- Stale joint state data is rejected.

## Launch

Start only the bridge node:

```bash
ros2 launch abb_pi0_bridge abb_pi0_bridge.launch.py
```

Dry-run with explicit overrides:

```bash
ros2 launch abb_pi0_bridge abb_pi0_bridge.launch.py \
  publish_commands:=false \
  control_rate_hz:=20.0
```

Send local dual-RealSense images to a remote pi0 policy server over Tailscale:

```bash
ros2 launch abb_pi0_bridge abb_pi0_bridge.launch.py \
  policy_backend:=http_json \
  policy_server_url:=http://100.70.7.8:8001/infer \
  enable_front_camera_observation:=true \
  enable_wrist_camera_observation:=true \
  front_camera_image_topic:=/camera/fixed_cam/color/image_raw \
  wrist_camera_image_topic:=/camera/wrist_cam/color/image_raw \
  publish_commands:=false
```

One-command full workcell bringup with ABB + MoveIt + `click_to_move` + dual RealSense + `abb_pi0_bridge`:

```bash
ros2 launch abb_pi0_bridge workcell_pi0.launch.py \
  rws_ip:=192.168.125.1 \
  policy_server_url:=http://100.70.7.8:8001/infer \
  publish_commands:=false
```

For this ABB cell, the verified RWS address is `192.168.125.1`. The upper-controller USB Ethernet link `enxc8a362651863` must be configured as `192.168.125.109/24`, because the ABB `SIO/COM_TRP/ROB_1` transmission currently targets `192.168.125.109:6515` for EGM.

Left-step policy stub for a visible motion check:

```bash
ros2 launch abb_pi0_bridge robotstudio_pi0.launch.py \
  launch_policy_stub:=true \
  policy_stub_mode:=left \
  policy_stub_step_rad:=0.1 \
  policy_stub_joint_index:=0 \
  publish_commands:=true
```

If the arm moves the wrong way, flip the sign of `policy_stub_step_rad`.

Switch to streaming mode:

```bash
ros2 service call /abb_pi0_bridge/activate_streaming_mode std_srvs/srv/Trigger "{}"
```

Arm streaming output:

```bash
ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool "{data: true}"
```

Return control to `click_to_move` / MoveIt:

```bash
ros2 service call /abb_pi0_bridge/activate_click_to_move_mode std_srvs/srv/Trigger "{}"
```

RobotStudio-oriented bringup from the server:

```bash
ros2 launch abb_pi0_bridge robotstudio_pi0.launch.py \
  robotstudio_rws_ip:=<YOUR_ROBOTSTUDIO_IP>
```

One-command startup helper:

```bash
cd /home/rob/workspace/ws_RAMS
tools/start_robotstudio_pi0.sh \
  --robotstudio-rws-ip 127.0.0.1 \
  --robotstudio-rws-port 28080 \
  --egm-port 6515 \
  --pytorch-device cuda:1
```

Add `--arm` only when you want the script to hand motion output to `pi0` immediately.

For the full server-side checklist, see `ROBOTSTUDIO_PI0_SETUP.md`.

For the exact first-working RobotStudio pi0 procedure, see `ROBOTSTUDIO_PI0_RUNBOOK.md`.

## Expected integration pattern

For now, the bridge should run beside the current stack instead of replacing it:

1. Bring up your existing ABB / MoveIt system as usual.
2. Start `abb_pi0_bridge` in dry-run mode.
3. Inspect `/abb_pi0_bridge/safe_command` and `/abb_pi0_bridge/status`.
4. Switch to `streaming` mode only when you want policy control.
5. Arm command output only after confirming the downstream controller path is the one you want to test.
6. Switch back to `trajectory` mode before using `click_to_move` again.

## Future Phase-2 work

- Add richer observations, such as tool pose and perception outputs.
- Extend the current repo-level mode arbitration to controller-manager-backed switching when that interface is available in the deployment environment.
- Add integration tests against fake hardware and controller_manager.

## Local cameras + remote pi0

The intended deployment split for your current setup is now:

1. This ABB upper-controller machine runs ROS 2, `abb_pi0_bridge`, the ABB driver, and the two local RealSense cameras.
2. A second machine connected over Tailscale runs `tools/openpi_http_adapter.py` plus the actual pi0/openpi checkpoint.
   Current remote inference host: `100.70.7.8 (tjzs-desktop)`.
3. `abb_pi0_bridge` publishes ABB joint observations together with optional JPEG-compressed `front` and `wrist` images over HTTP to the remote policy endpoint.
4. The remote policy returns the next ABB joint target, and the bridge keeps responsibility for safety clamping and output arming.

## Extrinsics And Timing

- `click_to_move/launch/system_bringup.launch.py` now publishes camera TFs from `fixed_cam_mount -> fixed_cam_link` and `wrist_cam_mount -> wrist_cam_link` instead of anchoring them directly to `base_link` / `tool0`.
- The transform offsets are launch-configurable through `fixed_tf_*` and `wrist_tf_*`, which makes hand-eye calibration updates much easier to apply without editing code.
- `abb_pi0_bridge` now uses message-header time when available, checks camera freshness, checks camera/joint timestamp skew, and can auto-disarm streaming output when the observation set becomes invalid.

## Cartesian Pi0 Safety Guard

For the real workcell path, `system_bringup.launch.py` passes the expanded `robot_description` into `abb_pi0_bridge`. The bridge uses that URDF to compute FK from `base_link` to `tcp`.

Default behavior:

- `enable_cartesian_workspace_guard=true`
- `cartesian_workspace_radius_m=0.5`
- center resets to the current TCP pose when streaming mode is activated
- center resets again when bridge output is armed
- if a candidate pi0 joint target places `tcp` outside the radius, the command is rejected and bridge output is disarmed

The current center, candidate TCP, and distance are visible in `/abb_pi0_bridge/status` under `cartesian_workspace_guard`.
