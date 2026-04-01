#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS 2 Humble on this machine is installed for the system Python.
# If Conda is active, CMake may discover the Conda interpreter first,
# which breaks ament/catkin package parsing during configure.
unset CONDA_EXE CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_DEFAULT_ENV
unset CONDA_PYTHON_EXE _CONDA_EXE _CONDA_ROOT _CE_CONDA _CE_M
unset PYTHONHOME

clean_path=""
IFS=':' read -r -a path_parts <<< "${PATH:-}"
for entry in "${path_parts[@]}"; do
  case "$entry" in
    */miniconda3/bin|*/miniconda3/condabin)
      continue
      ;;
  esac
  if [ -z "$entry" ]; then
    continue
  fi
  if [ -z "$clean_path" ]; then
    clean_path="$entry"
  else
    clean_path="${clean_path}:$entry"
  fi
done

export PATH="/usr/bin:/bin:/usr/sbin:/sbin${clean_path:+:${clean_path}}"

source /opt/ros/humble/setup.bash
set -u

cd "$workspace_root"

exec colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
