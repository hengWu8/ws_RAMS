# ws_RAMS

This workspace contains the existing ABB IRB6700 workcell description, MoveIt configuration, and `click_to_move` demo path.

## New additive package

`src/abb_pi0_bridge` is a Phase-1 bridge package for future pi0/openpi integration.

- It does not replace the current ABB stack.
- It does not modify `click_to_move`.
- It defaults to mock policy + dry-run command output.
- It is intended as the first low-level streaming-compatible policy bridge layer.
- It can now optionally attach fixed/wrist RealSense RGB frames to each HTTP policy request for remote pi0 inference.
- It now includes a one-command full workcell launch path for ABB + MoveIt + dual RealSense + remote pi0 bridge deployment.
- The integrated bringup now launches an ABB RWS-backed true-state publisher on `/abb_rws/joint_states`, and `abb_pi0_bridge` subscribes to that topic by default so pi0 uses the controller's real joint state even if `/joint_states` from ros2_control goes stale.

## Verified ABB Network Layout

For this real workcell, keep the ABB private network split as follows:

- ABB controller/RWS endpoint: `192.168.125.1`
- Upper-controller USB Ethernet interface `enxc8a362651863`: `192.168.125.109/24`
- ABB EGM `ROB_1` transmission target: `192.168.125.109:6515`

Do not use `192.168.125.88` as the ABB controller address on this setup. The controller configuration currently sends EGM traffic to `192.168.125.109`, so the local ABB-facing NIC must keep that address unless the ABB `SIO/COM_TRP/ROB_1` configuration is changed too.

## Build Note

This workspace is meant to build against the system ROS 2 Humble Python environment.

If Conda is active, `ament_cmake` may discover the Conda interpreter first and fail during package configuration with errors such as `ModuleNotFoundError: No module named 'catkin_pkg'`, even though the required ROS Python modules are installed for `/usr/bin/python3`.

Use the helper script below to force a system-Python build:

```bash
cd /home/rob/workspace/ws_RAMS
./tools/colcon_build_system_python.sh --packages-up-to abb_bringup abb_hardware_interface abb_pi0_bridge
```

The script removes Conda from `PATH`, unsets common Conda environment variables, sources `/opt/ros/humble/setup.bash`, and passes `-DPython3_EXECUTABLE=/usr/bin/python3` to CMake.

## Read-only workcell dashboard

For live situational awareness without starting the ABB control chain, run:

```bash
cd /home/rob/workspace/ws_RAMS
tools/workcell_dashboard.py --host 127.0.0.1 --port 8090
```

Open `http://127.0.0.1:8090` on the upper-controller machine.

The dashboard only reads status:

- ABB RWS `jointtarget` and `robtarget` for current joint/TCP pose.
- Local `v4l2-ctl` camera enumeration for the two RealSense devices.
- Remote pi0 adapter health on `http://100.70.7.8:8002/healthz`.
- Local ROS/ABB control-process watchlist.

It does not start ROS, EGM, controller manager, or any robot command publisher.

## Qt industrial HMI

For the local industrial-style HMI with live front/wrist previews, ABB read-only pose, remote pi0 health, and URDF-based 3D robot state:

```bash
cd /home/rob/workspace/ws_RAMS
python3 tools/workcell_hmi_qt.py
```

The HMI defaults to monitoring, but it now also contains a guarded `Limited Motion Control` panel. That panel calls the tested safe motion script, requires an operator confirmation checkbox, waits for ABB EGM readiness, arms output only for the requested time window, and then restores the normal `publish_commands:=false` pi05 bringup.

The real-robot EGM/pi05 runbook is documented here:

```text
docs/ABB_EGM_PI05_REAL_ROBOT_RUNBOOK.md
```

## One-click workcell + HMI startup

To launch the full workcell stack plus the Qt HMI in one command:

```bash
cd /home/rob/workspace/ws_RAMS
tools/start_workcell_hmi_remote.sh
```

Important:

- The underlying bringup still defaults to `publish_commands:=false`.
- This means the bridge can come up for observation and remote pi0 link checks without automatically commanding the ABB robot.
- The workcell launcher starts ROS first, seeds the forward position controller with the current ABB RWS joints, then optionally waits for you to start/restart the ABB RAPID EGM program. This preserves the real-cell startup order that made the visible 20 cm EGM test reliable.

To stop both the HMI and the workcell bringup:

```bash
cd /home/rob/workspace/ws_RAMS
tools/stop_workcell_hmi_remote.sh
```
