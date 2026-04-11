# RobotStudio pi0 Working Runbook

This is the first working RobotStudio deployment path for:

`openpi/pi0 -> openpi_http_adapter -> abb_pi0_bridge -> ros2_control -> ABB EGM -> RobotStudio robot`

Use this document when you want to reproduce the currently working setup where `pi0` controls the RobotStudio virtual robot.

## 0. Current Working Architecture

The working data path is:

```text
openpi checkpoint
  -> tools/openpi_http_adapter.py
  -> HTTP /infer on 127.0.0.1:8001
  -> abb_pi0_bridge
  -> /forward_command_controller_position/commands
  -> abb_hardware_interface
  -> EGM UDP 6515
  -> RobotStudio RAPID EGMRunJoint
```

The current adapter mode is:

- `config_name = pi05_libero`
- `mapping_mode = abb_libero`
- `synthetic_vision = true`
- `prompt = "move left"`
- `pytorch_device = cuda:1` in the validated run

The current RobotStudio/RWS path is:

- RWS is exposed on the Linux server as `127.0.0.1:28080`
- EGM UDP is direct between RobotStudio and the Linux server on port `6515`

## 1. RobotStudio Side Checklist

Before starting Linux commands, check RobotStudio.

1. Start the Virtual Controller.
2. Load the RAPID module that contains `EGMSetupUC`, `EGMActJoint`, and `EGMRunJoint`.
3. Make sure the task is `T_ROB1`.
4. Make sure Motors are on.
5. Make sure the RAPID program is running or ready to be restarted with `PP to Main`.

Your RAPID EGM section should match this shape:

```rapid
EGMGetId egm_id;
EGMSetupUC ROB_1, egm_id, "default", "ROB_1", \Joint;
EGMActJoint egm_id
            \J1:=egm_condition
            \J2:=egm_condition
            \J3:=egm_condition
            \J4:=egm_condition
            \J5:=egm_condition
            \J6:=egm_condition
            \MaxSpeedDeviation:=20.0;

WHILE TRUE DO
    EGMRunJoint egm_id, EGM_STOP_HOLD, \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=5 \RampOutTime:=5;
ENDWHILE
```

## 2. RobotStudio Communication Settings

In RobotStudio `Configuration -> Communication`, the `ROB_1` UDP UC device should point to the Linux server.

For the current working setup:

```text
Name: ROB_1
Type: UDPUC
Remote Address: 192.168.1.102
Remote port number: 6515
Local port number: 0
```

Important:

- `192.168.1.102` is the Linux server address that RobotStudio sends EGM UDP packets to.
- `6515` is the EGM UDP port.
- This is separate from RWS.
- Do not put `127.0.0.1` in `ROB_1` for EGM unless RobotStudio and the EGM receiver are on the same machine.

## 3. RWS SSH Tunnel From Windows

If RobotStudio RWS is only available on the Windows machine as `127.0.0.1:80`, expose it to the Linux server with this PowerShell command:

```powershell
ssh -o ExitOnForwardFailure=yes -o ServerAliveInterval=60 -N -R 28080:127.0.0.1:80 heng@192.168.1.102
```

After this tunnel is up:

- Linux reaches RWS at `127.0.0.1:28080`
- EGM still uses direct UDP to `192.168.1.102:6515`

This distinction matters:

```text
RWS TCP: Linux -> 127.0.0.1:28080 -> SSH tunnel -> Windows 127.0.0.1:80
EGM UDP: RobotStudio -> 192.168.1.102:6515
```

## 4. Recommended Startup: One Command

Use a `tmux` session so the process stays visible and easy to stop:

```bash
tmux new -A -s robotstudio_pi0
```

Then start the working stack without arming motion output:

```bash
cd /home/heng/workspace/ws_RAMS
tools/start_robotstudio_pi0.sh \
  --robotstudio-rws-ip 127.0.0.1 \
  --robotstudio-rws-port 28080 \
  --egm-port 6515 \
  --pytorch-device cuda:1
```

This starts:

- `tools/openpi_http_adapter.py`
- `robotstudio_pi0.launch.py`
- `abb_pi0_bridge` in streaming mode

Safe default:

- The script does not arm command output unless `--arm` is provided.

If you want `pi0` to control the robot immediately:

```bash
cd /home/heng/workspace/ws_RAMS
tools/start_robotstudio_pi0.sh \
  --robotstudio-rws-ip 127.0.0.1 \
  --robotstudio-rws-port 28080 \
  --egm-port 6515 \
  --pytorch-device cuda:1 \
  --arm
```

## 5. Manual Arm And Disarm

If you started without `--arm`, arm after you have confirmed RobotStudio is ready:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash

ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool "{data: true}"
```

Disarm:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash

ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool "{data: false}"
```

Return to trajectory mode:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash

ros2 service call /abb_pi0_bridge/activate_click_to_move_mode std_srvs/srv/Trigger "{}"
```

## 6. Verify The Stack

Check the openpi adapter:

```bash
curl --noproxy '*' http://127.0.0.1:8001/healthz
```

Expected important fields:

```json
{
  "ok": true,
  "policy_loaded": true,
  "mapping_mode": "abb_libero",
  "synthetic_vision": true
}
```

Check bridge status:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash

ros2 topic echo --once /abb_pi0_bridge/status
```

Expected when ready and armed:

```text
policy_source: http_json
policy_error: null
control_mode: streaming
command_output_active: true
state: command_ready
```

Check actual low-level commands:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash

ros2 topic echo /forward_command_controller_position/commands
```

Check robot feedback:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash

ros2 topic echo /joint_states
```

Check controllers:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash

ros2 control list_controllers
```

Expected:

```text
joint_state_broadcaster             ... active
forward_command_controller_position ... active
```

Check EGM UDP listener:

```bash
ss -u -a -n -p | rg 6515
```

Expected:

```text
0.0.0.0:6515
```

## 7. Manual Startup Fallback

If the one-command helper is not desired, use the manual sequence.

Terminal 1, start adapter:

```bash
cd /home/heng/workspace/ws_RAMS
/home/heng/workspace/openpi_official/.venv/bin/python \
  tools/openpi_http_adapter.py \
  --host 127.0.0.1 \
  --port 8001 \
  --config-name pi05_libero \
  --checkpoint-dir /home/heng/workspace/openpi_official/checkpoints/pi05_libero_pytorch \
  --pytorch-device cuda:1 \
  --mapping-mode abb_libero \
  --use-synthetic-vision
```

Terminal 2, start bridge:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash

ros2 launch abb_pi0_bridge robotstudio_pi0.launch.py \
  robotstudio_rws_ip:=127.0.0.1 \
  robotstudio_rws_port:=28080 \
  egm_port:=6515 \
  launch_policy_stub:=false \
  policy_server_url:=http://127.0.0.1:8001/infer \
  publish_commands:=true
```

Terminal 3, configure bridge:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash

ros2 param set /abb_pi0_bridge policy_request_timeout_sec 60.0
ros2 service call /abb_pi0_bridge/activate_streaming_mode std_srvs/srv/Trigger "{}"
ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool "{data: true}"
```

## 8. Stop Everything

If you used the helper:

```bash
cd /home/heng/workspace/ws_RAMS
tools/stop_robotstudio_pi0.sh
```

If you used manual terminals:

- Stop the `ros2 launch` terminal with `Ctrl-C`.
- Stop the adapter terminal with `Ctrl-C`.
- Optionally disarm first:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash

ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool "{data: false}"
```

## 9. Known Stale EGM Session Recovery

Confirmed behavior:

- RWS can still work.
- UDP packets can still flow.
- `/forward_command_controller_position/commands` can still receive non-zero commands.
- The robot may still not move.
- Restarting the RAPID program restores motion.

This means the EGM motion session can become stale even though communication is not fully broken.

Recovery:

1. Disarm bridge output.
2. In RobotStudio, stop RAPID.
3. Perform `PP to Main`.
4. Start RAPID again.
5. Confirm the robot can move again.
6. Arm `abb_pi0_bridge` again.

Disarm before recovery:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash

ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool "{data: false}"
```

Arm after RAPID restart:

```bash
source /opt/ros/humble/setup.bash
source /home/heng/workspace/ws_RAMS/install/setup.bash

ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool "{data: true}"
```

## 10. Current Success Criteria

The current first working version is considered healthy when:

- `curl http://127.0.0.1:8001/healthz` shows `policy_loaded=true`.
- `/abb_pi0_bridge/status` shows `policy_source=http_json`.
- `/abb_pi0_bridge/status` shows `policy_error=null`.
- `/abb_pi0_bridge/status` shows `command_output_active=true`.
- `/forward_command_controller_position/commands` is receiving non-zero arrays.
- `/joint_states` changes while `pi0` is armed.
- RobotStudio visibly shows continuous robot motion.

## 11. Files In This Version

Main runtime files:

- `/home/heng/workspace/ws_RAMS/tools/openpi_http_adapter.py`
- `/home/heng/workspace/ws_RAMS/tools/start_robotstudio_pi0.sh`
- `/home/heng/workspace/ws_RAMS/tools/stop_robotstudio_pi0.sh`
- `/home/heng/workspace/ws_RAMS/src/abb_pi0_bridge/launch/robotstudio_pi0.launch.py`
- `/home/heng/workspace/ws_RAMS/src/abb_pi0_bridge/config/pi0_robotstudio_http.params.yaml`

Useful documentation:

- `/home/heng/workspace/ws_RAMS/src/abb_pi0_bridge/ROBOTSTUDIO_PI0_SETUP.md`
- `/home/heng/workspace/ws_RAMS/src/abb_pi0_bridge/ROBOTSTUDIO_PI0_RUNBOOK.md`
