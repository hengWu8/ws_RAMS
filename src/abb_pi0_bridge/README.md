# abb_pi0_bridge

`abb_pi0_bridge` is a new, additive package for preparing `ws_RAMS` for future pi0/openpi deployment on the ABB IRB6700 stack.

## Phase-1 scope

- Keep the existing `click_to_move` and MoveIt path unchanged.
- Subscribe to robot observations from `/joint_states`.
- Assemble a policy-friendly observation object.
- Support both a mock policy and a real HTTP JSON policy server stub.
- Apply safety filters before any command leaves the node.
- Publish low-level streaming-style commands to a configurable topic.
- Arbitrate between `click_to_move` trajectory mode and streaming mode with a shared control-mode channel.

## Current architecture

The bridge node is intentionally split into small layers:

1. Observation assembly:
   Order joint state data into a fixed ABB joint vector.
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
