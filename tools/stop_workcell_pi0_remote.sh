#!/usr/bin/env bash
set -euo pipefail

RUNTIME_DIR_DEFAULT="/tmp/ws_rams_workcell_pi0_remote"
RUNTIME_DIR="${RUNTIME_DIR_DEFAULT}"

if [[ $# -gt 0 ]]; then
  if [[ "$1" == "--runtime-dir" && $# -eq 2 ]]; then
    RUNTIME_DIR="$2"
  else
    echo "Usage: $(basename "$0") [--runtime-dir PATH]" >&2
    exit 2
  fi
fi

PID_FILE="${RUNTIME_DIR}/launch.pid"
HOLD_SEED_PID_FILE="${RUNTIME_DIR}/hold_seed.pid"

if [[ -f "${HOLD_SEED_PID_FILE}" ]]; then
  HOLD_SEED_PID="$(cat "${HOLD_SEED_PID_FILE}")"
  if [[ -n "${HOLD_SEED_PID}" ]] && kill -0 "${HOLD_SEED_PID}" >/dev/null 2>&1; then
    kill "${HOLD_SEED_PID}" >/dev/null 2>&1 || true
    sleep 1
    if kill -0 "${HOLD_SEED_PID}" >/dev/null 2>&1; then
      kill -9 "${HOLD_SEED_PID}" >/dev/null 2>&1 || true
    fi
    echo "Stopped hold seeder pid ${HOLD_SEED_PID}."
  else
    echo "Hold seeder pid ${HOLD_SEED_PID} is not running."
  fi
  rm -f "${HOLD_SEED_PID_FILE}"
fi

if [[ ! -f "${PID_FILE}" ]]; then
  echo "No launch pid file found under ${RUNTIME_DIR}."
  exit 0
fi

PID="$(cat "${PID_FILE}")"
PGID="${PID}"

if [[ -n "${PID}" ]] && kill -0 "${PID}" >/dev/null 2>&1; then
  kill -- "-${PGID}" >/dev/null 2>&1 || kill "${PID}" >/dev/null 2>&1 || true
  sleep 1
  if kill -0 "${PID}" >/dev/null 2>&1; then
    kill -9 -- "-${PGID}" >/dev/null 2>&1 || kill -9 "${PID}" >/dev/null 2>&1 || true
  fi
  echo "Stopped launch pid ${PID} (process group ${PGID})."
else
  echo "Launch pid ${PID} is not running."
fi

rm -f "${PID_FILE}"

# ros2 launch can occasionally leave children re-parented after the launch
# process group exits. Clean only this workspace's workcell bringup processes.
patterns=(
  "/home/rob/workspace/ws_RAMS/install/abb_pi0_bridge/lib/abb_pi0_bridge/abb_pi0_bridge_node"
  "/opt/ros/humble/lib/controller_manager/ros2_control_node"
  "/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node"
  "/opt/ros/humble/lib/moveit_ros_move_group/move_group"
  "/home/rob/workspace/ws_RAMS/install/click_to_move/lib/click_to_move/click_to_move_node"
  "/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher"
)

for pattern in "${patterns[@]}"; do
  mapfile -t child_pids < <(pgrep -f "${pattern}" || true)
  for child_pid in "${child_pids[@]}"; do
    [[ -z "${child_pid}" || "${child_pid}" == "$$" ]] && continue
    kill "${child_pid}" >/dev/null 2>&1 || true
  done
done
