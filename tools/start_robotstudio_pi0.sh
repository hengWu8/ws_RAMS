#!/usr/bin/env bash
set -euo pipefail

WS_RAMS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(cd "${WS_RAMS_ROOT}/.." && pwd)"
OPENPI_ROOT_DEFAULT="${WORKSPACE_ROOT}/openpi_official"
RUNTIME_DIR_DEFAULT="/tmp/ws_rams_robotstudio_pi0"

ROBOTSTUDIO_RWS_IP="127.0.0.1"
ROBOTSTUDIO_RWS_PORT="28080"
EGM_PORT="6515"
POLICY_PORT="8001"
POLICY_HOST="127.0.0.1"
POLICY_CONFIG="pi05_libero"
POLICY_CHECKPOINT="${OPENPI_ROOT_DEFAULT}/checkpoints/pi05_libero_pytorch"
PYTORCH_DEVICE="auto"
MAPPING_MODE="abb_libero"
PUBLISH_COMMANDS="true"
ARM_OUTPUT="false"
USE_SYNTHETIC_VISION="true"
POLICY_TIMEOUT_SEC="60.0"
OPENPI_ROOT="${OPENPI_ROOT_DEFAULT}"
RUNTIME_DIR="${RUNTIME_DIR_DEFAULT}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Starts the local openpi HTTP adapter and the RobotStudio ROS bridge, then
switches abb_pi0_bridge to streaming mode.

Options:
  --arm                    Arm low-level streaming output after startup.
  --robotstudio-rws-ip IP  RobotStudio RWS IP. Default: ${ROBOTSTUDIO_RWS_IP}
  --robotstudio-rws-port N RobotStudio RWS TCP port. Default: ${ROBOTSTUDIO_RWS_PORT}
  --egm-port N             ABB EGM UDP port. Default: ${EGM_PORT}
  --policy-port N          Local adapter HTTP port. Default: ${POLICY_PORT}
  --policy-host HOST       Local adapter bind host. Default: ${POLICY_HOST}
  --policy-config NAME     openpi config name. Default: ${POLICY_CONFIG}
  --checkpoint-dir PATH    openpi checkpoint directory.
  --pytorch-device DEV     PyTorch device, e.g. auto, cuda:0. Default: ${PYTORCH_DEVICE}
  --mapping-mode MODE      Adapter mapping mode. Default: ${MAPPING_MODE}
  --no-synthetic-vision    Disable synthetic vision in the adapter.
  --policy-timeout SEC     abb_pi0_bridge policy timeout. Default: ${POLICY_TIMEOUT_SEC}
  --runtime-dir PATH       Runtime/log directory. Default: ${RUNTIME_DIR_DEFAULT}
  --openpi-root PATH       openpi_official root. Default: ${OPENPI_ROOT_DEFAULT}
  -h, --help               Show this help text.
EOF
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "ERROR: required command not found: $1" >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --arm)
      ARM_OUTPUT="true"
      shift
      ;;
    --robotstudio-rws-ip)
      ROBOTSTUDIO_RWS_IP="$2"
      shift 2
      ;;
    --robotstudio-rws-port)
      ROBOTSTUDIO_RWS_PORT="$2"
      shift 2
      ;;
    --egm-port)
      EGM_PORT="$2"
      shift 2
      ;;
    --policy-port)
      POLICY_PORT="$2"
      shift 2
      ;;
    --policy-host)
      POLICY_HOST="$2"
      shift 2
      ;;
    --policy-config)
      POLICY_CONFIG="$2"
      shift 2
      ;;
    --checkpoint-dir)
      POLICY_CHECKPOINT="$2"
      shift 2
      ;;
    --pytorch-device)
      PYTORCH_DEVICE="$2"
      shift 2
      ;;
    --mapping-mode)
      MAPPING_MODE="$2"
      shift 2
      ;;
    --no-synthetic-vision)
      USE_SYNTHETIC_VISION="false"
      shift
      ;;
    --policy-timeout)
      POLICY_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --runtime-dir)
      RUNTIME_DIR="$2"
      shift 2
      ;;
    --openpi-root)
      OPENPI_ROOT="$2"
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

require_cmd curl

ADAPTER_PYTHON="${OPENPI_ROOT}/.venv/bin/python"
ADAPTER_SCRIPT="${WS_RAMS_ROOT}/tools/openpi_http_adapter.py"
ADAPTER_URL="http://${POLICY_HOST}:${POLICY_PORT}"
POLICY_URL="${ADAPTER_URL}/infer"
HEALTH_URL="${ADAPTER_URL}/healthz"

if [[ ! -x "${ADAPTER_PYTHON}" ]]; then
  echo "ERROR: adapter Python not found: ${ADAPTER_PYTHON}" >&2
  exit 2
fi
if [[ ! -f "${ADAPTER_SCRIPT}" ]]; then
  echo "ERROR: adapter script not found: ${ADAPTER_SCRIPT}" >&2
  exit 2
fi
if [[ ! -d "${POLICY_CHECKPOINT}" ]]; then
  echo "ERROR: checkpoint directory not found: ${POLICY_CHECKPOINT}" >&2
  exit 2
fi

mkdir -p "${RUNTIME_DIR}"
ADAPTER_PID_FILE="${RUNTIME_DIR}/adapter.pid"
LAUNCH_PID_FILE="${RUNTIME_DIR}/launch.pid"
ADAPTER_LOG="${RUNTIME_DIR}/adapter.log"
LAUNCH_LOG="${RUNTIME_DIR}/launch.log"
META_FILE="${RUNTIME_DIR}/meta.env"

check_existing_pid() {
  local pid_file="$1"
  local label="$2"
  if [[ -f "${pid_file}" ]]; then
    local pid
    pid="$(cat "${pid_file}")"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
      echo "ERROR: ${label} already appears to be running with pid ${pid}." >&2
      echo "Use tools/stop_robotstudio_pi0.sh first, or remove stale pid files in ${RUNTIME_DIR}." >&2
      exit 2
    fi
    rm -f "${pid_file}"
  fi
}

check_existing_pid "${ADAPTER_PID_FILE}" "adapter"
check_existing_pid "${LAUNCH_PID_FILE}" "launch"

echo "Starting openpi adapter..."
adapter_cmd=(
  "${ADAPTER_PYTHON}" "${ADAPTER_SCRIPT}"
  --host "${POLICY_HOST}"
  --port "${POLICY_PORT}"
  --config-name "${POLICY_CONFIG}"
  --checkpoint-dir "${POLICY_CHECKPOINT}"
  --pytorch-device "${PYTORCH_DEVICE}"
  --mapping-mode "${MAPPING_MODE}"
)
if [[ "${USE_SYNTHETIC_VISION}" == "true" ]]; then
  adapter_cmd+=(--use-synthetic-vision)
fi

(
  cd "${WS_RAMS_ROOT}"
  nohup "${adapter_cmd[@]}" >"${ADAPTER_LOG}" 2>&1 < /dev/null &
  echo $! > "${ADAPTER_PID_FILE}"
) >/dev/null 2>&1
ADAPTER_PID="$(cat "${ADAPTER_PID_FILE}")"

for _ in $(seq 1 60); do
  if curl --noproxy '*' --silent --show-error "${HEALTH_URL}" >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

if ! curl --noproxy '*' --silent --show-error "${HEALTH_URL}" >/dev/null 2>&1; then
  echo "ERROR: adapter did not become healthy. See ${ADAPTER_LOG}" >&2
  exit 1
fi

echo "Warming up openpi policy..."
warmup_payload='{"observation":{"joint_positions":[0,0,0,0,0,0]}}'
curl --noproxy '*' --silent --show-error \
  -X POST "${POLICY_URL}" \
  -H 'Content-Type: application/json' \
  -d "${warmup_payload}" >/dev/null || true

echo "Starting RobotStudio ROS launch..."
nohup bash -lc "
  set +u
  source /opt/ros/humble/setup.bash
  source '${WS_RAMS_ROOT}/install/setup.bash'
  set -u
  exec ros2 launch abb_pi0_bridge robotstudio_pi0.launch.py \
    robotstudio_rws_ip:='${ROBOTSTUDIO_RWS_IP}' \
    robotstudio_rws_port:='${ROBOTSTUDIO_RWS_PORT}' \
    egm_port:='${EGM_PORT}' \
    launch_policy_stub:=false \
    policy_server_url:='${POLICY_URL}' \
    publish_commands:='${PUBLISH_COMMANDS}'
" >"${LAUNCH_LOG}" 2>&1 < /dev/null &
LAUNCH_PID=$!
echo "${LAUNCH_PID}" > "${LAUNCH_PID_FILE}"

wait_for_service() {
  local service_name="$1"
  for _ in $(seq 1 60); do
    if bash -lc "set +u && source /opt/ros/humble/setup.bash && source '${WS_RAMS_ROOT}/install/setup.bash' && set -u && ros2 service list | grep -Fx '${service_name}'" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

if ! wait_for_service "/abb_pi0_bridge/activate_streaming_mode"; then
  echo "ERROR: abb_pi0_bridge service did not appear. See ${LAUNCH_LOG}" >&2
  exit 1
fi

echo "Configuring abb_pi0_bridge..."
bash -lc "set +u && source /opt/ros/humble/setup.bash && source '${WS_RAMS_ROOT}/install/setup.bash' && set -u && ros2 param set /abb_pi0_bridge policy_request_timeout_sec ${POLICY_TIMEOUT_SEC}" >/dev/null
bash -lc "set +u && source /opt/ros/humble/setup.bash && source '${WS_RAMS_ROOT}/install/setup.bash' && set -u && ros2 service call /abb_pi0_bridge/activate_streaming_mode std_srvs/srv/Trigger '{}'" >/dev/null
bash -lc "set +u && source /opt/ros/humble/setup.bash && source '${WS_RAMS_ROOT}/install/setup.bash' && set -u && ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool '{data: ${ARM_OUTPUT}}'" >/dev/null

if ! kill -0 "${ADAPTER_PID}" >/dev/null 2>&1; then
  echo "ERROR: adapter process exited unexpectedly. See ${ADAPTER_LOG}" >&2
  exit 1
fi
if ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
  echo "ERROR: launch process exited unexpectedly. See ${LAUNCH_LOG}" >&2
  exit 1
fi

cat > "${META_FILE}" <<EOF
WS_RAMS_ROOT='${WS_RAMS_ROOT}'
OPENPI_ROOT='${OPENPI_ROOT}'
RUNTIME_DIR='${RUNTIME_DIR}'
ADAPTER_PID='${ADAPTER_PID}'
LAUNCH_PID='${LAUNCH_PID}'
ADAPTER_LOG='${ADAPTER_LOG}'
LAUNCH_LOG='${LAUNCH_LOG}'
POLICY_URL='${POLICY_URL}'
HEALTH_URL='${HEALTH_URL}'
ROBOTSTUDIO_RWS_IP='${ROBOTSTUDIO_RWS_IP}'
ROBOTSTUDIO_RWS_PORT='${ROBOTSTUDIO_RWS_PORT}'
EGM_PORT='${EGM_PORT}'
ARM_OUTPUT='${ARM_OUTPUT}'
EOF

echo
echo "RobotStudio pi0 startup complete."
echo "  adapter pid: ${ADAPTER_PID}"
echo "  launch pid: ${LAUNCH_PID}"
echo "  adapter log: ${ADAPTER_LOG}"
echo "  launch log: ${LAUNCH_LOG}"
echo "  policy health: ${HEALTH_URL}"
echo "  streaming armed: ${ARM_OUTPUT}"
echo
echo "Stop everything with:"
echo "  ${WS_RAMS_ROOT}/tools/stop_robotstudio_pi0.sh --runtime-dir ${RUNTIME_DIR}"
