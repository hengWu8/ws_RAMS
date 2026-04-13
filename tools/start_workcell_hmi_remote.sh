#!/usr/bin/env bash
set -euo pipefail

WS_RAMS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLS_DIR="${WS_RAMS_ROOT}/tools"
RUNTIME_DIR_DEFAULT="/tmp/ws_rams_hmi_remote"
WORKCELL_RUNTIME_DIR="${RUNTIME_DIR_DEFAULT}/workcell"
HMI_RUNTIME_DIR="${RUNTIME_DIR_DEFAULT}/hmi"

DISPLAY_VALUE="${DISPLAY:-:0}"
HEADLESS_HMI="false"
WORKSPACE_RADIUS_M="0.5"
RUNTIME_DIR="${RUNTIME_DIR_DEFAULT}"
PASSTHROUGH_ARGS=()

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

One-click startup for:
  1. system_bringup + abb_pi0_bridge + remote pi0 HTTP bridge
  2. Qt industrial HMI

Safety default:
  - Underlying workcell launcher keeps publish_commands:=false
  - This script does not arm robot output unless you explicitly pass through --streaming/--arm

Options:
  --display DISPLAY        X11 display for the Qt HMI. Default: ${DISPLAY_VALUE}
  --headless-hmi           Run the HMI with QT_QPA_PLATFORM=offscreen for smoke/debug use.
  --workspace-radius-m M   HMI workspace cube half-extent in meters. Default: ${WORKSPACE_RADIUS_M}
  --runtime-dir PATH       Runtime/log directory. Default: ${RUNTIME_DIR_DEFAULT}
  -h, --help               Show this help text.

All other arguments are passed through to start_workcell_pi0_remote.sh.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --display)
      DISPLAY_VALUE="$2"
      shift 2
      ;;
    --headless-hmi)
      HEADLESS_HMI="true"
      shift
      ;;
    --workspace-radius-m)
      WORKSPACE_RADIUS_M="$2"
      shift 2
      ;;
    --runtime-dir)
      RUNTIME_DIR="$2"
      WORKCELL_RUNTIME_DIR="${RUNTIME_DIR}/workcell"
      HMI_RUNTIME_DIR="${RUNTIME_DIR}/hmi"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      PASSTHROUGH_ARGS+=("$1")
      shift
      ;;
  esac
done

mkdir -p "${WORKCELL_RUNTIME_DIR}" "${HMI_RUNTIME_DIR}"
HMI_PID_FILE="${HMI_RUNTIME_DIR}/hmi.pid"
HMI_LOG="${HMI_RUNTIME_DIR}/hmi.log"

"${TOOLS_DIR}/start_workcell_pi0_remote.sh" \
  --runtime-dir "${WORKCELL_RUNTIME_DIR}" \
  "${PASSTHROUGH_ARGS[@]}"

if [[ -f "${HMI_PID_FILE}" ]]; then
  EXISTING_PID="$(cat "${HMI_PID_FILE}")"
  if [[ -n "${EXISTING_PID}" ]] && kill -0 "${EXISTING_PID}" >/dev/null 2>&1; then
    echo "Qt HMI already running with pid ${EXISTING_PID}."
    echo "  log: ${HMI_LOG}"
    exit 0
  fi
  rm -f "${HMI_PID_FILE}"
fi

echo "Starting Qt HMI..."
if [[ "${HEADLESS_HMI}" == "true" ]]; then
  setsid env QT_QPA_PLATFORM=offscreen bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    source '${WS_RAMS_ROOT}/install/setup.bash'
    set -u
    exec python3 '${TOOLS_DIR}/workcell_hmi_qt.py' \
      --rws-ip '192.168.125.1' \
      --remote-host '100.70.7.8' \
      --workspace-radius-m '${WORKSPACE_RADIUS_M}'
  " >"${HMI_LOG}" 2>&1 < /dev/null &
else
  setsid env DISPLAY="${DISPLAY_VALUE}" QT_QPA_PLATFORM=xcb bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    source '${WS_RAMS_ROOT}/install/setup.bash'
    set -u
    exec python3 '${TOOLS_DIR}/workcell_hmi_qt.py' \
      --rws-ip '192.168.125.1' \
      --remote-host '100.70.7.8' \
      --workspace-radius-m '${WORKSPACE_RADIUS_M}'
  " >"${HMI_LOG}" 2>&1 < /dev/null &
fi

HMI_PID=$!
echo "${HMI_PID}" > "${HMI_PID_FILE}"

sleep 1
if ! kill -0 "${HMI_PID}" >/dev/null 2>&1; then
  echo "ERROR: Qt HMI exited immediately. See ${HMI_LOG}" >&2
  exit 1
fi

echo
echo "Industrial HMI startup complete."
echo "  workcell runtime: ${WORKCELL_RUNTIME_DIR}"
echo "  hmi pid: ${HMI_PID}"
echo "  hmi log: ${HMI_LOG}"
echo "  display: ${DISPLAY_VALUE}"
