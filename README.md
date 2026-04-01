# ws_RAMS

This workspace contains the existing ABB IRB6700 workcell description, MoveIt configuration, and `click_to_move` demo path.

## New additive package

`src/abb_pi0_bridge` is a Phase-1 bridge package for future pi0/openpi integration.

- It does not replace the current ABB stack.
- It does not modify `click_to_move`.
- It defaults to mock policy + dry-run command output.
- It is intended as the first low-level streaming-compatible policy bridge layer.

## Build Note

This workspace is meant to build against the system ROS 2 Humble Python environment.

If Conda is active, `ament_cmake` may discover the Conda interpreter first and fail during package configuration with errors such as `ModuleNotFoundError: No module named 'catkin_pkg'`, even though the required ROS Python modules are installed for `/usr/bin/python3`.

Use the helper script below to force a system-Python build:

```bash
cd /home/heng/workspace/ws_RAMS
./tools/colcon_build_system_python.sh --packages-up-to abb_bringup abb_hardware_interface abb_pi0_bridge
```

The script removes Conda from `PATH`, unsets common Conda environment variables, sources `/opt/ros/humble/setup.bash`, and passes `-DPython3_EXECUTABLE=/usr/bin/python3` to CMake.
