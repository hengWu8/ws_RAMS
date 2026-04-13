#!/usr/bin/env bash
set -euo pipefail

WS_RAMS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR="/tmp/ws_rams_cartesian_left_test"
SAFE_RUNTIME_DIR="/tmp/ws_rams_workcell_pi0_safe_8002"
RWS_IP="192.168.125.1"
RWS_PORT="80"
EGM_PORT="6515"
POLICY_SERVER_URL="http://100.70.7.8:8002/infer"
POLICY_BACKEND="cartesian_left_test"
DIRECTION_X="0.0"
DIRECTION_Y="1.0"
DIRECTION_Z="0.0"
DIRECTION_LABEL="+Y"
STEP_M="0.001"
SPEED_MPS="0.001"
TRACKING_GAIN="1.0"
MAX_CARTESIAN_ERROR_M="0.02"
MAX_JOINT_DELTA_RAD="0.005"
BRIDGE_MAX_POSITION_STEP_RAD="0.02"
BRIDGE_JOINT_STATE_TOPIC="/abb_rws/joint_states"
BRIDGE_FEEDBACK_JOINT_STATE_TOPIC="/abb_rws/joint_states"
BRIDGE_STEP_REFERENCE_MODE="current_observation"
DURATION_SEC="1.2"
EGM_READY_WAIT_SEC="0"
EXECUTE="false"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Runs the deterministic Cartesian-left test policy.

By default this is a dry-run: publish_commands=false and no robot motion output is armed.
Pass --execute to perform a short live motion window, then automatically restore
the normal remote pi05 safe bringup with publish_commands=false.

Options:
  --execute                    Actually open publish+arm for the live test window.
  --policy-backend NAME        Policy backend to use. Default: ${POLICY_BACKEND}
  --direction NAME             Cartesian direction shortcut: +x, -x, +y, -y, +z, -z.
                               Default: ${DIRECTION_LABEL}
  --direction-xyz X,Y,Z        Explicit Cartesian direction vector in base frame.
                               The bridge normalizes this vector before use.
  --duration-sec SECONDS       Live arm duration when --execute is set. Default: ${DURATION_SEC}
  --step-m METERS              TCP +Y step per bridge tick. Default: ${STEP_M}
  --speed-mps METERS_PER_SEC   Cartesian demo trajectory speed. Default: ${SPEED_MPS}
  --tracking-gain VALUE       Closed-loop Cartesian tracking gain. Default: ${TRACKING_GAIN}
  --max-cartesian-error-m M   Maximum Cartesian tracking error magnitude. Default: ${MAX_CARTESIAN_ERROR_M}
  --max-joint-delta-rad RAD    Policy-side per-joint delta clamp. Default: ${MAX_JOINT_DELTA_RAD}
  --bridge-max-position-step-rad RAD
                               Bridge safety-filter per-tick joint clamp. Default: ${BRIDGE_MAX_POSITION_STEP_RAD}
  --bridge-joint-state-topic TOPIC
                               JointState topic consumed by abb_pi0_bridge. Default: ${BRIDGE_JOINT_STATE_TOPIC}
  --bridge-feedback-joint-state-topic TOPIC
                               True-state JointState topic used by closed-loop demo policies. Default: ${BRIDGE_FEEDBACK_JOINT_STATE_TOPIC}
  --bridge-step-reference-mode MODE
                               Step clamp reference mode: current_observation or approved_command.
                               Default: ${BRIDGE_STEP_REFERENCE_MODE}
  --wait-for-egm-ready-sec SECONDS
                               When --execute is set, keep the test ROS stack
                               running and wait this long for ABB EGM readiness
                               before arming. Use this when you need to restart
                               the RAPID EGM program after the ROS stack is up.
                               Default: ${EGM_READY_WAIT_SEC}
  --rws-ip IP                  ABB RWS IP. Default: ${RWS_IP}
  --rws-port PORT              ABB RWS port. Default: ${RWS_PORT}
  --egm-port PORT              EGM UDP port. Default: ${EGM_PORT}
  --policy-server-url URL      Normal pi05 URL restored after test. Default: ${POLICY_SERVER_URL}
  --runtime-dir DIR            Runtime/log directory. Default: ${RUNTIME_DIR}
  --safe-runtime-dir DIR       Normal safe bringup runtime dir. Default: ${SAFE_RUNTIME_DIR}
  -h, --help                   Show this help.
EOF
}

set_direction_shortcut() {
  case "$1" in
    +x|x|forward)
      DIRECTION_X="1.0"; DIRECTION_Y="0.0"; DIRECTION_Z="0.0"; DIRECTION_LABEL="+X"
      ;;
    -x|backward)
      DIRECTION_X="-1.0"; DIRECTION_Y="0.0"; DIRECTION_Z="0.0"; DIRECTION_LABEL="-X"
      ;;
    +y|y|left)
      DIRECTION_X="0.0"; DIRECTION_Y="1.0"; DIRECTION_Z="0.0"; DIRECTION_LABEL="+Y"
      ;;
    -y|right)
      DIRECTION_X="0.0"; DIRECTION_Y="-1.0"; DIRECTION_Z="0.0"; DIRECTION_LABEL="-Y"
      ;;
    +z|z|up)
      DIRECTION_X="0.0"; DIRECTION_Y="0.0"; DIRECTION_Z="1.0"; DIRECTION_LABEL="+Z"
      ;;
    -z|down)
      DIRECTION_X="0.0"; DIRECTION_Y="0.0"; DIRECTION_Z="-1.0"; DIRECTION_LABEL="-Z"
      ;;
    *)
      echo "ERROR: unknown direction shortcut: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
}

set_direction_xyz() {
  local raw="$1"
  IFS=',' read -r DIRECTION_X DIRECTION_Y DIRECTION_Z extra <<<"${raw}"
  if [[ -n "${extra:-}" || -z "${DIRECTION_X:-}" || -z "${DIRECTION_Y:-}" || -z "${DIRECTION_Z:-}" ]]; then
    echo "ERROR: --direction-xyz must be formatted as X,Y,Z" >&2
    exit 2
  fi
  DIRECTION_LABEL="custom(${DIRECTION_X},${DIRECTION_Y},${DIRECTION_Z})"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)
      EXECUTE="true"
      shift
      ;;
    --policy-backend)
      POLICY_BACKEND="$2"
      shift 2
      ;;
    --direction)
      set_direction_shortcut "$2"
      shift 2
      ;;
    --direction-xyz)
      set_direction_xyz "$2"
      shift 2
      ;;
    --duration-sec)
      DURATION_SEC="$2"
      shift 2
      ;;
    --step-m)
      STEP_M="$2"
      shift 2
      ;;
    --speed-mps)
      SPEED_MPS="$2"
      shift 2
      ;;
    --tracking-gain)
      TRACKING_GAIN="$2"
      shift 2
      ;;
    --max-cartesian-error-m)
      MAX_CARTESIAN_ERROR_M="$2"
      shift 2
      ;;
    --max-joint-delta-rad)
      MAX_JOINT_DELTA_RAD="$2"
      shift 2
      ;;
    --bridge-max-position-step-rad)
      BRIDGE_MAX_POSITION_STEP_RAD="$2"
      shift 2
      ;;
    --bridge-joint-state-topic)
      BRIDGE_JOINT_STATE_TOPIC="$2"
      shift 2
      ;;
    --bridge-feedback-joint-state-topic)
      BRIDGE_FEEDBACK_JOINT_STATE_TOPIC="$2"
      shift 2
      ;;
    --bridge-step-reference-mode)
      BRIDGE_STEP_REFERENCE_MODE="$2"
      shift 2
      ;;
    --wait-for-egm-ready-sec)
      EGM_READY_WAIT_SEC="$2"
      shift 2
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
    --policy-server-url)
      POLICY_SERVER_URL="$2"
      shift 2
      ;;
    --runtime-dir)
      RUNTIME_DIR="$2"
      shift 2
      ;;
    --safe-runtime-dir)
      SAFE_RUNTIME_DIR="$2"
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
ACTIVE_SAMPLES="${RUNTIME_DIR}/active_window_samples.txt"
EGM_READINESS_AFTER_LAUNCH="${RUNTIME_DIR}/egm_readiness_after_launch.txt"
HOLD_SEED_LOG="${RUNTIME_DIR}/hold_seed.log"
HOLD_SEED_PID=""

source_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  source "${WS_RAMS_ROOT}/install/setup.bash"
  set -u
}

stop_launch_pidfile() {
  local pidfile="$1"
  if [[ ! -f "${pidfile}" ]]; then
    return 0
  fi
  local pid
  pid="$(cat "${pidfile}" 2>/dev/null || true)"
  if [[ -z "${pid}" ]]; then
    return 0
  fi
  if kill -0 "${pid}" >/dev/null 2>&1; then
    local pgid
    pgid="$(ps -o pgid= -p "${pid}" | tr -d ' ' || true)"
    if [[ -n "${pgid}" ]]; then
      kill -- -"${pgid}" >/dev/null 2>&1 || true
    fi
    kill "${pid}" >/dev/null 2>&1 || true
  fi
}

stop_hold_seed() {
  if [[ -n "${HOLD_SEED_PID}" ]] && kill -0 "${HOLD_SEED_PID}" >/dev/null 2>&1; then
    kill "${HOLD_SEED_PID}" >/dev/null 2>&1 || true
    wait "${HOLD_SEED_PID}" >/dev/null 2>&1 || true
  fi
  HOLD_SEED_PID=""
}

read_pose() {
  RWS_IP="${RWS_IP}" RWS_PORT="${RWS_PORT}" python3 - <<'PY'
import os
import json
import tools.workcell_dashboard as d

cfg = d.make_dashboard_config(
    rws_ip=os.environ["RWS_IP"],
    rws_port=os.environ["RWS_PORT"],
    rws_user=d.DEFAULT_RWS_USER,
    rws_password=d.DEFAULT_RWS_PASSWORD,
    rws_timeout_s=2.0,
)
j = d.read_rws_jointtarget(cfg)
c = d.read_rws_cartesian(cfg)
print(json.dumps({
    "joint_ok": j.get("ok"),
    "joints_deg": j.get("degrees"),
    "cart_ok": c.get("ok"),
    "tcp_mm": c.get("position_mm"),
}, ensure_ascii=False, sort_keys=True))
PY
}

check_egm_ready_after_launch() {
  python3 "${WS_RAMS_ROOT}/tools/check_abb_egm_motion_readiness.py" \
    --rws-ip "${RWS_IP}" \
    --min-egm-window-deg 0.1 \
    --max-diag-age-sec 15 \
    --max-egm-cycle-gap-sec 45 >"${EGM_READINESS_AFTER_LAUNCH}" 2>&1
}

restore_safe() {
  set +e
  stop_hold_seed
  source_ros >/dev/null 2>&1 || true
  timeout 5 ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  timeout 5 ros2 service call /abb_pi0_bridge/activate_click_to_move_mode std_srvs/srv/Trigger "{}" >/dev/null 2>&1
  stop_launch_pidfile "${LAUNCH_PID_FILE}"
  sleep 2
  "${WS_RAMS_ROOT}/tools/start_workcell_pi0_remote.sh" \
    --runtime-dir "${SAFE_RUNTIME_DIR}" \
    --policy-server-url "${POLICY_SERVER_URL}" >/tmp/ws_rams_safe_restore_after_cartesian_left.out 2>&1 || true
}
trap restore_safe EXIT

cd "${WS_RAMS_ROOT}"
source_ros

echo "Cartesian-left test starting."
echo "  mode: $([[ "${EXECUTE}" == "true" ]] && echo live-execute || echo dry-run)"
echo "  policy_backend: ${POLICY_BACKEND}"
echo "  direction: ${DIRECTION_LABEL} [${DIRECTION_X}, ${DIRECTION_Y}, ${DIRECTION_Z}]"
echo "  step_m: ${STEP_M}"
echo "  speed_mps: ${SPEED_MPS}"
echo "  tracking_gain: ${TRACKING_GAIN}"
echo "  max_cartesian_error_m: ${MAX_CARTESIAN_ERROR_M}"
echo "  max_joint_delta_rad: ${MAX_JOINT_DELTA_RAD}"
echo "  bridge_max_position_step_rad: ${BRIDGE_MAX_POSITION_STEP_RAD}"
echo "  bridge_joint_state_topic: ${BRIDGE_JOINT_STATE_TOPIC}"
echo "  bridge_feedback_joint_state_topic: ${BRIDGE_FEEDBACK_JOINT_STATE_TOPIC}"
echo "  bridge_step_reference_mode: ${BRIDGE_STEP_REFERENCE_MODE}"
echo "  duration_sec: ${DURATION_SEC}"
echo "  wait_for_egm_ready_sec: ${EGM_READY_WAIT_SEC}"
echo "  log: ${LAUNCH_LOG}"
printf "BEFORE_POSE "
read_pose | tee "${RUNTIME_DIR}/before_pose.json"

stop_launch_pidfile "${SAFE_RUNTIME_DIR}/launch.pid"
sleep 3

PUBLISH_COMMANDS="false"
if [[ "${EXECUTE}" == "true" ]]; then
  PUBLISH_COMMANDS="true"
fi

setsid bash -lc "
  set +u
  source /opt/ros/humble/setup.bash
  source '${WS_RAMS_ROOT}/install/setup.bash'
  set -u
  cd '${WS_RAMS_ROOT}'
  exec ros2 launch abb_pi0_bridge workcell_pi0.launch.py \
    rws_ip:='${RWS_IP}' \
    rws_port:='${RWS_PORT}' \
    egm_port:='${EGM_PORT}' \
    initial_joint_controller:=forward_command_controller_position \
    fixed_serial:=_109522062115 \
    wrist_serial:=_342522070232 \
    policy_backend:='${POLICY_BACKEND}' \
    launch_moveit_rviz:=false \
    publish_commands:='${PUBLISH_COMMANDS}' \
    control_mode:=trajectory \
    bridge_joint_state_timeout_sec:=1.0 \
    bridge_max_position_step_rad:='${BRIDGE_MAX_POSITION_STEP_RAD}' \
    bridge_joint_state_topic:='${BRIDGE_JOINT_STATE_TOPIC}' \
    bridge_feedback_joint_state_topic:='${BRIDGE_FEEDBACK_JOINT_STATE_TOPIC}' \
    bridge_feedback_joint_state_timeout_sec:=1.0 \
    bridge_step_reference_mode:='${BRIDGE_STEP_REFERENCE_MODE}' \
    bridge_max_camera_joint_skew_sec:=1.0 \
    bridge_enable_front_camera_observation:=false \
    bridge_enable_wrist_camera_observation:=false \
    cartesian_test_direction_x:='${DIRECTION_X}' \
    cartesian_test_direction_y:='${DIRECTION_Y}' \
    cartesian_test_direction_z:='${DIRECTION_Z}' \
    cartesian_test_step_m:='${STEP_M}' \
    cartesian_test_speed_mps:='${SPEED_MPS}' \
    cartesian_test_tracking_gain:='${TRACKING_GAIN}' \
    cartesian_test_max_cartesian_error_m:='${MAX_CARTESIAN_ERROR_M}' \
    cartesian_test_max_joint_delta_rad:='${MAX_JOINT_DELTA_RAD}'
" >"${LAUNCH_LOG}" 2>&1 < /dev/null &
echo "$!" > "${LAUNCH_PID_FILE}"

for _ in $(seq 1 90); do
  if ros2 node list 2>/dev/null | grep -Fx /abb_pi0_bridge >/dev/null; then
    break
  fi
  sleep 1
done
sleep 3

if [[ "${EXECUTE}" == "true" ]]; then
  echo "Seeding forward position controller with current robot joints before ABB EGM is restarted..."
  python3 "${WS_RAMS_ROOT}/tools/seed_forward_position_hold.py" \
    --joint-state-topic "${BRIDGE_FEEDBACK_JOINT_STATE_TOPIC}" \
    --command-topic /forward_command_controller_position/commands \
    --rate-hz 20 \
    --startup-timeout-sec 8 >"${HOLD_SEED_LOG}" 2>&1 &
  HOLD_SEED_PID="$!"
  sleep 1
  if ! kill -0 "${HOLD_SEED_PID}" >/dev/null 2>&1; then
    cat "${HOLD_SEED_LOG}" >&2 || true
    echo "ERROR: failed to start hold seeder; aborting before ABB EGM restart/arm." >&2
    exit 1
  fi

  echo "Checking ABB EGM readiness after launching the test ROS stack..."
  EGM_READY="false"
  if check_egm_ready_after_launch; then
    EGM_READY="true"
  elif (( EGM_READY_WAIT_SEC > 0 )); then
    echo "EGM is not ready yet. Keep this script running, restart the RAPID EGM program now, and I will wait up to ${EGM_READY_WAIT_SEC}s..."
    DEADLINE_SEC=$(( $(date +%s) + EGM_READY_WAIT_SEC ))
    while (( $(date +%s) < DEADLINE_SEC )); do
      sleep 5
      if check_egm_ready_after_launch; then
        EGM_READY="true"
        break
      fi
      echo "Still waiting for ABB EGM_RUNNING..."
    done
  fi

  cat "${EGM_READINESS_AFTER_LAUNCH}" || true
  if [[ "${EGM_READY}" != "true" ]]; then
    echo "ERROR: ABB EGM is not ready with this newly launched ROS stack; aborting before arming output." >&2
    echo "Hint: start/restart the RAPID EGM program after this ROS stack is up, then rerun the test." >&2
    exit 1
  fi
fi

if [[ "${EXECUTE}" == "true" ]]; then
  echo "Switching to streaming mode and arming output for ${DURATION_SEC}s..."
  ros2 service call /abb_pi0_bridge/activate_streaming_mode std_srvs/srv/Trigger "{}" >/dev/null
  sleep 0.5
  stop_hold_seed
  ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool "{data: true}" >/dev/null
  (
    sleep 0.8
    {
      echo "=== ACTIVE WINDOW SAMPLE $(date --iso-8601=seconds) ==="
      echo "--- /abb_pi0_bridge/status ---"
      timeout 3 ros2 topic echo /abb_pi0_bridge/status --once --field data --full-length || true
      echo "--- /abb_pi0_bridge/safe_command ---"
      timeout 3 ros2 topic echo /abb_pi0_bridge/safe_command --once || true
      echo "--- ${COMMAND_TOPIC:-/forward_command_controller_position/commands} ---"
      timeout 3 ros2 topic echo /forward_command_controller_position/commands --once || true
      echo "--- /abb/control_mode ---"
      timeout 3 ros2 topic echo /abb/control_mode --once || true
      echo "--- topic info: /forward_command_controller_position/commands ---"
      ros2 topic info /forward_command_controller_position/commands || true
    } >"${ACTIVE_SAMPLES}" 2>&1
  ) &
  SAMPLER_PID="$!"
  sleep "${DURATION_SEC}"
  ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool "{data: false}" >/dev/null
  ros2 service call /abb_pi0_bridge/activate_click_to_move_mode std_srvs/srv/Trigger "{}" >/dev/null
  wait "${SAMPLER_PID}" >/dev/null 2>&1 || true
  if [[ -f "${ACTIVE_SAMPLES}" ]]; then
    cat "${ACTIVE_SAMPLES}"
  fi
else
  echo "Dry-run only: publish_commands=false; output not armed."
fi

timeout 10 ros2 topic echo /abb_pi0_bridge/status --once --field data --full-length \
  | tee "${RUNTIME_DIR}/status.json"
timeout 10 ros2 topic echo /abb_pi0_bridge/safe_command --once \
  | tee "${RUNTIME_DIR}/safe_command.txt"
printf "AFTER_POSE "
read_pose | tee "${RUNTIME_DIR}/after_pose.json"

echo "Cartesian-left test finished; restoring normal safe pi05 bringup."
