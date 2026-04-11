#!/usr/bin/env bash
set -euo pipefail

RUNTIME_DIR="/tmp/ws_rams_robotstudio_pi0"

usage() {
  cat <<EOF
Usage: $(basename "$0") [--runtime-dir PATH]

Stops the adapter and ROS launch started by tools/start_robotstudio_pi0.sh.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --runtime-dir)
      RUNTIME_DIR="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

stop_pid_file() {
  local pid_file="$1"
  local label="$2"
  if [[ ! -f "${pid_file}" ]]; then
    return 0
  fi

  local pid
  pid="$(cat "${pid_file}")"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    echo "Stopping ${label} pid ${pid}..."
    kill "${pid}" >/dev/null 2>&1 || true
    for _ in $(seq 1 20); do
      if ! kill -0 "${pid}" >/dev/null 2>&1; then
        break
      fi
      sleep 0.5
    done
    if kill -0 "${pid}" >/dev/null 2>&1; then
      echo "Force-killing ${label} pid ${pid}..."
      kill -9 "${pid}" >/dev/null 2>&1 || true
    fi
  fi
  rm -f "${pid_file}"
}

stop_pid_file "${RUNTIME_DIR}/launch.pid" "launch"
stop_pid_file "${RUNTIME_DIR}/adapter.pid" "adapter"

echo "Stopped RobotStudio pi0 runtime in ${RUNTIME_DIR}"
