#!/usr/bin/env bash
set -euo pipefail

WS_RAMS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLS_DIR="${WS_RAMS_ROOT}/tools"
RUNTIME_DIR_DEFAULT="/tmp/ws_rams_hmi_remote"
RUNTIME_DIR="${RUNTIME_DIR_DEFAULT}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [--runtime-dir PATH]
EOF
}

if [[ $# -gt 0 ]]; then
  if [[ "$1" == "--runtime-dir" && $# -eq 2 ]]; then
    RUNTIME_DIR="$2"
  else
    usage >&2
    exit 2
  fi
fi

WORKCELL_RUNTIME_DIR="${RUNTIME_DIR}/workcell"
HMI_RUNTIME_DIR="${RUNTIME_DIR}/hmi"
HMI_PID_FILE="${HMI_RUNTIME_DIR}/hmi.pid"

if [[ -f "${HMI_PID_FILE}" ]]; then
  HMI_PID="$(cat "${HMI_PID_FILE}")"
  if [[ -n "${HMI_PID}" ]] && kill -0 "${HMI_PID}" >/dev/null 2>&1; then
    kill "${HMI_PID}" >/dev/null 2>&1 || true
    sleep 1
    if kill -0 "${HMI_PID}" >/dev/null 2>&1; then
      kill -9 "${HMI_PID}" >/dev/null 2>&1 || true
    fi
    echo "Stopped Qt HMI pid ${HMI_PID}."
  else
    echo "Qt HMI pid ${HMI_PID} is not running."
  fi
  rm -f "${HMI_PID_FILE}"
else
  echo "No Qt HMI pid file found under ${HMI_RUNTIME_DIR}."
fi

"${TOOLS_DIR}/stop_workcell_pi0_remote.sh" --runtime-dir "${WORKCELL_RUNTIME_DIR}"
