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
POLICY_SERVER_URL="http://100.70.7.8:8001/infer"
STREAMING_MODE="false"
ARM_OUTPUT="false"
RUNTIME_DIR="${RUNTIME_DIR_DEFAULT}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Starts the full ABB workcell bringup with dual RealSense cameras and abb_pi0_bridge,
configured to call the remote Tailscale pi0/openpi server.

Options:
  --streaming              Switch abb_pi0_bridge into streaming mode after startup, without arming output.
  --arm                    Arm low-level streaming output after startup.
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

mkdir -p "${RUNTIME_DIR}"
LAUNCH_PID_FILE="${RUNTIME_DIR}/launch.pid"
LAUNCH_LOG="${RUNTIME_DIR}/launch.log"
META_FILE="${RUNTIME_DIR}/meta.env"

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

if ! wait_for_service "/abb_pi0_bridge/activate_streaming_mode"; then
  echo "ERROR: abb_pi0_bridge did not come up. See ${LAUNCH_LOG}" >&2
  exit 1
fi

if [[ "${STREAMING_MODE}" == "true" ]]; then
  echo "Switching abb_pi0_bridge into streaming mode..."
  bash -lc "set +u && source /opt/ros/humble/setup.bash && source '${WS_RAMS_ROOT}/install/setup.bash' && set -u && ros2 service call /abb_pi0_bridge/activate_streaming_mode std_srvs/srv/Trigger '{}'" >/dev/null
else
  echo "Leaving abb_pi0_bridge in trajectory mode."
fi
bash -lc "set +u && source /opt/ros/humble/setup.bash && source '${WS_RAMS_ROOT}/install/setup.bash' && set -u && ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool '{data: ${ARM_OUTPUT}}'" >/dev/null

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
EOF

echo
echo "Workcell pi0 remote startup complete."
echo "  launch pid: ${LAUNCH_PID}"
echo "  policy server: ${POLICY_SERVER_URL}"
echo "  streaming mode: ${STREAMING_MODE}"
echo "  arm output: ${ARM_OUTPUT}"
echo "  log: ${LAUNCH_LOG}"
