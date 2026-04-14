# ABB EGM + pi05 Real Robot Runbook

This note records the real-cell path that successfully moved the ABB robot under the ws_RAMS bridge on 2026-04-13.

The goal is repeatability: future tests should not depend on memory, hidden RobotStudio defaults, or accidental controller state.

## Proven Result

A visible Cartesian motion test completed on the real controller:

- Commanded profile: `100 s * 0.002 m/s = 0.20 m`
- Policy backend: `cartesian_left_true_servo`
- Direction: base-frame `+Y`
- Start TCP: `x=1539.099 mm, y=3.849 mm, z=1352.181 mm`
- End TCP: `x=1539.113 mm, y=200.538 mm, z=1352.187 mm`
- Measured travel: about `196.69 mm` in `+Y`
- Final bridge state after script cleanup:
  - `publish_commands=false`
  - `command_output_active=false`
  - `command_output_armed=false`
  - `control_mode=trajectory`
  - `policy_backend=http_json`

## Final RAPID Pattern

Use the operator-defined ready-point program:

```rapid
rapid/TRob1Main_WS_RAMS_EGM_Joint_PrepMove_Blocking_CondTime600.mod
```

Key choices:

- `MoveAbsJ egm_ready, v10, fine, tool0;` is the required ordinary first movement after controller start.
- `egm_ready` is a `PERS jointtarget` that the operator must set manually to a known-safe pose.
- Do not use `MoveAbsJ home` to a hidden/default all-zero target.
- Do not use `MoveAbsJ CJointT()` for the prep move on this real controller; it produced unexpected pre-EGM behavior during debugging.
- Use one long blocking `EGMRunJoint` window:

```rapid
EGMRunJoint egm_id, EGM_STOP_HOLD, \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=600 \RampOutTime:=5;
```

Why `CondTime:=600`:

- `egm_condition := [-0.1, 0.1]` is a convergence condition, not a workspace limit.
- If the joint error remains inside the condition band, a short `CondTime` can make `EGMRunJoint` return while ROS is still streaming.
- A long blocking window keeps EGM available for slow visible tests while the ROS bridge enforces workspace and per-step safety.

## Controller / EGM Profile Values

The RobotStudio profile that moved clearly used:

- `ext_motion_Kp = 5`
- `ext_motion_filter_bandwidth = 20`
- `max_allowed_speed_factor = 0.25`
- `ramp_time = 0.5`
- `max_external_pos_adjustment = 0.2`
- `use_motion_process_mode = rob1_optimal_cycle_time_mode`
- `std_servo_queue_time = 0.193536`
- `group_queue_time = 0.048384`
- `ipol_prefetch_time = 0.1`
- `dynamic_resolution = 1`
- `path_resolution = 1`
- `interpolation_priority = 10`

If real-cell behavior diverges from RobotStudio, compare these fields first.

## ROS / Bridge Pattern That Worked

Use the tested script. It intentionally launches ROS first, seeds the forward position controller with the current real joints, waits for ABB EGM readiness, then arms output only for the requested window.

The general pi05 bringup now follows the same ordering:

```text
ROS workcell launch
  -> abb_pi0_bridge starts with publish_commands=false
  -> /abb_rws/joint_states true-state publisher starts
  -> seed_forward_position_hold.py publishes current joints to the forward command controller
  -> operator starts/restarts ABB RAPID EGM
  -> readiness check waits for EGM_RUNNING
  -> only then may streaming/arm be enabled
```

This order matters. Starting ABB EGM before ROS/forward-controller current-position seeding is ready can expose stale or default command behavior at the hardware-interface layer. The hold seeder is stopped automatically before an armed test window so it does not compete with `abb_pi0_bridge`.

Safe pi05 observation bringup:

```bash
cd /home/rob/workspace/ws_RAMS
tools/start_workcell_pi0_remote.sh --wait-for-egm-ready-sec 180
```

The script will start ROS first, seed a current-position hold command, then prompt you to turn Motors On and start/restart the ABB RAPID EGM program. It still leaves `publish_commands=false` unless `--arm` is explicitly requested.

Example 20 cm run:

```bash
cd /home/rob/workspace/ws_RAMS
tools/run_cartesian_left_test.sh \
  --execute \
  --policy-backend cartesian_left_true_servo \
  --direction +y \
  --duration-sec 100 \
  --speed-mps 0.002 \
  --tracking-gain 1.0 \
  --max-cartesian-error-m 0.02 \
  --max-joint-delta-rad 0.008 \
  --bridge-max-position-step-rad 0.1 \
  --bridge-joint-state-topic /joint_states \
  --bridge-feedback-joint-state-topic /abb_rws/joint_states \
  --wait-for-egm-ready-sec 180
```

While the script is waiting for EGM readiness, restart/start the RAPID EGM program on the ABB controller. The script will continue only after the readiness check passes.

## HMI Motion Control

The Qt HMI now includes a `Limited Motion Control` panel.

It exposes:

- Direction: `+Y`, `-Y`, `+X`, `-X`, `+Z`, `-Z`
- Speed in `mm/s`
- Duration in seconds
- Expected travel display
- Confirmation checkbox for the operator-defined RAPID EGM state
- `START LIMITED MOTION`
- `STOP / DISARM`

The HMI calls the same tested script and keeps the same safety posture:

- Uses `cartesian_left_true_servo`
- Uses `/abb_rws/joint_states` for true feedback
- Waits for ABB EGM readiness
- Arms output only during the requested window
- Restores the normal safe pi05 bringup afterwards

The HMI blocks requests above `250 mm` of expected travel. The lower-level bridge still enforces the workspace guard, per-step joint clamp, and auto-disarm behavior.

## Safe Operating Checklist

Before clicking start or running the script:

1. ABB controller motors on, E-stop clear, work area clear.
2. RAPID module loaded with the manually verified `egm_ready` target.
3. Start the script or HMI command first.
4. When it waits for EGM readiness, start/restart the RAPID EGM program.
5. Confirm readiness passes before the bridge arms output.
6. Watch the robot and be ready to stop from ABB side.
7. After completion, verify `publish_commands=false` and `command_output_armed=false`.

## Failure Modes We Saw

- Short `CondTime:=5` produced a repeatable running/stopped rhythm and made visible motion unreliable.
- `EGMRunJoint \NoWaitCond + EGMWaitCond` stalled on the real controller in this setup, even though similar logic can work in examples.
- `MoveAbsJ CJointT()` was not a safe substitute for a real operator ready point on this controller.
- Starting ABB EGM before the ROS/forward-controller side had seeded a valid hold command could expose stale/default command behavior.
- The bridge must use the RWS-backed true joint state for closed-loop Cartesian servo tests: `/abb_rws/joint_states`.

## Keep These Defaults Conservative

- HMI speed max: `3 mm/s`
- HMI travel max: `250 mm`
- Bridge workspace radius: `0.5 m`
- Bridge auto-disarm on observation/workspace fault: enabled
- Normal pi05 bringup after any test: `publish_commands=false`
