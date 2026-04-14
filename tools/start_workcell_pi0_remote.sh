#!/usr/bin/env bash
set -euo pipefail

WS_RAMS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR_DEFAULT="/tmp/ws_rams_workcell_pi0_remote"

RWS_IP="192.168.125.1"
RWS_PORT="80"
EGM_PORT="6515"
FIXED_SERIAL="_109522062115"
WRIST_SERIAL="_342522070232"
FIXED_USB_PORT_ID=""
WRIST_USB_PORT_ID=""
POLICY_SERVER_URL="http://100.70.7.8:8002/infer"
STREAMING_MODE="false"
ARM_OUTPUT="false"
WAIT_FOR_EGM_READY_SEC="0"
HOLD_SEED="true"
RUNTIME_DIR="${RUNTIME_DIR_DEFAULT}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Starts the full ABB workcell bringup with dual RealSense cameras and abb_pi0_bridge,
configured to call the remote Tailscale pi0/openpi server.

Options:
  --streaming              Switch abb_pi0_bridge into streaming mode after startup, without arming output.
  --arm                    Arm low-level streaming output after startup.
                           Requires ABB EGM readiness; if no wait is set, waits up to 180s.
  --wait-for-egm-ready-sec SECONDS
                           After ROS is up and the hold seeder is running, wait this
                           long for the operator to start/restart ABB RAPID EGM.
                           Default: ${WAIT_FOR_EGM_READY_SEC}
  --no-hold-seed           Do not seed the forward position controller with current
                           ABB joints. Not recommended for real-robot EGM startup.
  --rws-ip IP              ABB/RobotStudio RWS IP. Default: ${RWS_IP}
  --rws-port N             ABB/RobotStudio RWS TCP port. Default: ${RWS_PORT}
  --egm-port N             ABB EGM UDP port. Default: ${EGM_PORT}
  --fixed-serial SERIAL    Fixed RealSense serial. Default: ${FIXED_SERIAL}
  --wrist-serial SERIAL    Wrist RealSense serial. Default: ${WRIST_SERIAL}
  --fixed-usb-port ID      Fixed RealSense USB port id. Default: unset
  --wrist-usb-port ID      Wrist RealSense USB port id. Default: unset
  --policy-server-url URL  Remote pi0 policy URL. Default: ${POLICY_SERVER_URL}
  --runtime-dir PATH       Runtime/log directory. Default: ${RUNTIME_DIR_DEFAULT}
  -h, --help               Show this help text.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --streaming)
      STREAMING_MODE="true"
      shift
      ;;
    --arm)
      STREAMING_MODE="true"
      ARM_OUTPUT="true"
      shift
      ;;
    --wait-for-egm-ready-sec)
      WAIT_FOR_EGM_READY_SEC="$2"
      shift 2
      ;;
    --no-hold-seed)
      HOLD_SEED="false"
      shift
      ;;
    --rws-ip)
      RWS_IP="$2"
      shift 2
      ;;
    --rws-port)
      RWS_PORT="$2"
      shift 2
      ;;
    --egm-port)
      EGM_PORT="$2"
      shift 2
      ;;
    --fixed-serial)
      FIXED_SERIAL="$2"
      shift 2
      ;;
    --wrist-serial)
      WRIST_SERIAL="$2"
      shift 2
      ;;
    --fixed-usb-port)
      FIXED_USB_PORT_ID="$2"
      shift 2
      ;;
    --wrist-usb-port)
      WRIST_USB_PORT_ID="$2"
      shift 2
      ;;
    --policy-server-url)
      POLICY_SERVER_URL="$2"
      shift 2
      ;;
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

if [[ "${ARM_OUTPUT}" == "true" && "${WAIT_FOR_EGM_READY_SEC}" == "0" ]]; then
  WAIT_FOR_EGM_READY_SEC="180"
fi

mkdir -p "${RUNTIME_DIR}"
LAUNCH_PID_FILE="${RUNTIME_DIR}/launch.pid"
LAUNCH_LOG="${RUNTIME_DIR}/launch.log"
META_FILE="${RUNTIME_DIR}/meta.env"
HOLD_SEED_PID_FILE="${RUNTIME_DIR}/hold_seed.pid"
HOLD_SEED_LOG="${RUNTIME_DIR}/hold_seed.log"
EGM_READINESS_LOG="${RUNTIME_DIR}/egm_readiness.log"
HOLD_SEED_PID=""

if [[ -f "${LAUNCH_PID_FILE}" ]]; then
  EXISTING_PID="$(cat "${LAUNCH_PID_FILE}")"
  if [[ -n "${EXISTING_PID}" ]] && kill -0 "${EXISTING_PID}" >/dev/null 2>&1; then
    echo "ERROR: workcell pi0 remote bringup already appears to be running with pid ${EXISTING_PID}." >&2
    exit 2
  fi
  rm -f "${LAUNCH_PID_FILE}"
fi

echo "Starting workcell + remote pi0 bridge..."
launch_cmd=(
  "exec" "ros2" "launch" "abb_pi0_bridge" "workcell_pi0.launch.py"
  "rws_ip:='${RWS_IP}'"
  "rws_port:='${RWS_PORT}'"
  "egm_port:='${EGM_PORT}'"
  "initial_joint_controller:='forward_command_controller_position'"
  "fixed_serial:='${FIXED_SERIAL}'"
  "wrist_serial:='${WRIST_SERIAL}'"
  "policy_server_url:='${POLICY_SERVER_URL}'"
  "launch_moveit_rviz:='false'"
  "publish_commands:='false'"
  "control_mode:='trajectory'"
)

if [[ -n "${FIXED_USB_PORT_ID}" ]]; then
  launch_cmd+=("fixed_usb_port_id:='${FIXED_USB_PORT_ID}'")
fi
if [[ -n "${WRIST_USB_PORT_ID}" ]]; then
  launch_cmd+=("wrist_usb_port_id:='${WRIST_USB_PORT_ID}'")
fi

launch_cmd_string="$(printf "%s " "${launch_cmd[@]}")"

setsid bash -lc "
  set +u
  source /opt/ros/humble/setup.bash
  source '${WS_RAMS_ROOT}/install/setup.bash'
  set -u
  ${launch_cmd_string}
" >"${LAUNCH_LOG}" 2>&1 < /dev/null &
LAUNCH_PID=$!
echo "${LAUNCH_PID}" > "${LAUNCH_PID_FILE}"

wait_for_service() {
  local service_name="$1"
  for _ in $(seq 1 90); do
    if bash -lc "set +u && source /opt/ros/humble/setup.bash && source '${WS_RAMS_ROOT}/install/setup.bash' && set -u && ros2 service list | grep -Fx '${service_name}'" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

source_ros_inline="set +u && source /opt/ros/humble/setup.bash && source '${WS_RAMS_ROOT}/install/setup.bash' && set -u"

start_hold_seed() {
  if [[ "${HOLD_SEED}" != "true" ]]; then
    echo "WARNING: hold seeding disabled. Do not start ABB EGM unless another current-position hold source is active." >&2
    return 0
  fi
  echo "Seeding forward position controller from ABB RWS joints before ABB EGM starts..."
  bash -lc "${source_ros_inline} && exec python3 '${WS_RAMS_ROOT}/tools/seed_forward_position_hold.py' \
    --joint-state-topic /abb_rws/joint_states \
    --command-topic /forward_command_controller_position/commands \
    --rate-hz 20 \
    --startup-timeout-sec 12" >"${HOLD_SEED_LOG}" 2>&1 &
  HOLD_SEED_PID="$!"
  echo "${HOLD_SEED_PID}" > "${HOLD_SEED_PID_FILE}"
  sleep 1
  if ! kill -0 "${HOLD_SEED_PID}" >/dev/null 2>&1; then
    echo "ERROR: hold seeder failed to start. See ${HOLD_SEED_LOG}" >&2
    cat "${HOLD_SEED_LOG}" >&2 || true
    return 1
  fi
}

stop_hold_seed() {
  local pid="${HOLD_SEED_PID}"
  if [[ -z "${pid}" && -f "${HOLD_SEED_PID_FILE}" ]]; then
    pid="$(cat "${HOLD_SEED_PID_FILE}" 2>/dev/null || true)"
  fi
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    kill "${pid}" >/dev/null 2>&1 || true
    wait "${pid}" >/dev/null 2>&1 || true
    echo "Stopped hold seeder pid ${pid}."
  fi
  rm -f "${HOLD_SEED_PID_FILE}"
  HOLD_SEED_PID=""
}

check_egm_ready() {
  bash -lc "${source_ros_inline} && cd '${WS_RAMS_ROOT}' && python3 tools/check_abb_egm_motion_readiness.py \
    --rws-ip '${RWS_IP}' \
    --min-egm-window-deg 0.1 \
    --max-diag-age-sec 15 \
    --max-egm-cycle-gap-sec 45" >"${EGM_READINESS_LOG}" 2>&1
}

wait_for_egm_ready_if_requested() {
  local ready="false"
  if check_egm_ready; then
    ready="true"
  elif (( WAIT_FOR_EGM_READY_SEC > 0 )); then
    echo
    echo "ROS is ready and current-position hold seeding is active."
    echo "Now turn Motors On and Start/Restart the ABB RAPID EGM program."
    echo "Waiting up to ${WAIT_FOR_EGM_READY_SEC}s for ABB EGM_RUNNING..."
    local deadline_sec=$(( $(date +%s) + WAIT_FOR_EGM_READY_SEC ))
    while (( $(date +%s) < deadline_sec )); do
      sleep 5
      if check_egm_ready; then
        ready="true"
        break
      fi
      echo "Still waiting for ABB EGM_RUNNING..."
    done
  fi
  cat "${EGM_READINESS_LOG}" || true
  [[ "${ready}" == "true" ]]
}

if ! wait_for_service "/abb_pi0_bridge/activate_streaming_mode"; then
  echo "ERROR: abb_pi0_bridge did not come up. See ${LAUNCH_LOG}" >&2
  exit 1
fi

if ! start_hold_seed; then
  if [[ "${ARM_OUTPUT}" == "true" ]]; then
    echo "ERROR: refusing to arm without a current-position hold seed." >&2
    exit 1
  fi
fi

EGM_READY="false"
if wait_for_egm_ready_if_requested; then
  EGM_READY="true"
elif [[ "${ARM_OUTPUT}" == "true" ]]; then
  echo "ERROR: ABB EGM is not ready; refusing to arm output." >&2
  exit 1
fi

if [[ "${STREAMING_MODE}" == "true" ]]; then
  echo "Switching abb_pi0_bridge into streaming mode..."
  bash -lc "${source_ros_inline} && ros2 service call /abb_pi0_bridge/activate_streaming_mode std_srvs/srv/Trigger '{}'" >/dev/null
else
  echo "Leaving abb_pi0_bridge in trajectory mode."
fi
if [[ "${ARM_OUTPUT}" == "true" ]]; then
  stop_hold_seed
fi
bash -lc "${source_ros_inline} && ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool '{data: ${ARM_OUTPUT}}'" >/dev/null

cat > "${META_FILE}" <<EOF
WS_RAMS_ROOT='${WS_RAMS_ROOT}'
RUNTIME_DIR='${RUNTIME_DIR}'
LAUNCH_PID='${LAUNCH_PID}'
LAUNCH_LOG='${LAUNCH_LOG}'
RWS_IP='${RWS_IP}'
RWS_PORT='${RWS_PORT}'
EGM_PORT='${EGM_PORT}'
FIXED_SERIAL='${FIXED_SERIAL}'
WRIST_SERIAL='${WRIST_SERIAL}'
FIXED_USB_PORT_ID='${FIXED_USB_PORT_ID}'
WRIST_USB_PORT_ID='${WRIST_USB_PORT_ID}'
POLICY_SERVER_URL='${POLICY_SERVER_URL}'
STREAMING_MODE='${STREAMING_MODE}'
ARM_OUTPUT='${ARM_OUTPUT}'
WAIT_FOR_EGM_READY_SEC='${WAIT_FOR_EGM_READY_SEC}'
HOLD_SEED='${HOLD_SEED}'
HOLD_SEED_PID='${HOLD_SEED_PID}'
HOLD_SEED_LOG='${HOLD_SEED_LOG}'
EGM_READY='${EGM_READY}'
EGM_READINESS_LOG='${EGM_READINESS_LOG}'
EOF

echo
echo "Workcell pi0 remote startup complete."
echo "  launch pid: ${LAUNCH_PID}"
echo "  policy server: ${POLICY_SERVER_URL}"
echo "  hold seeder: ${HOLD_SEED} ${HOLD_SEED_PID:+pid ${HOLD_SEED_PID}}"
echo "  egm ready: ${EGM_READY}"
echo "  streaming mode: ${STREAMING_MODE}"
echo "  arm output: ${ARM_OUTPUT}"
echo "  log: ${LAUNCH_LOG}"
