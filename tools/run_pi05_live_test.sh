#!/usr/bin/env bash
set -euo pipefail

WS_RAMS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR="/tmp/ws_rams_pi05_live_test"
SAFE_RUNTIME_DIR="/tmp/ws_rams_workcell_pi0_safe_8002"
RWS_IP="192.168.125.1"
RWS_PORT="80"
EGM_PORT="6515"
POLICY_SERVER_URL="http://100.70.7.8:8002/infer"
DURATION_SEC="1.2"
EGM_READY_WAIT_SEC="180"
BRIDGE_MAX_POSITION_STEP_RAD="0.02"
BRIDGE_JOINT_STATE_TOPIC="/abb_rws/joint_states"
BRIDGE_FEEDBACK_JOINT_STATE_TOPIC="/abb_rws/joint_states"
EXECUTE="false"

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

Runs a bounded live official-pi05 test window.

Default is dry-run: launch the full chain with publish_commands=false and do not arm.
Pass --execute to arm bridge output only for --duration-sec, then auto-disarm and
restore the normal safe pi05 bringup.

Options:
  --execute                    Arm output for the bounded test window.
  --duration-sec SECONDS       Live pi05 arm duration. Default: ${DURATION_SEC}
  --wait-for-egm-ready-sec S   Wait for ABB EGM readiness after ROS/hold seed. Default: ${EGM_READY_WAIT_SEC}
  --bridge-max-position-step-rad RAD
                               Bridge per-tick joint clamp. Default: ${BRIDGE_MAX_POSITION_STEP_RAD}
  --policy-server-url URL      Official pi05 infer URL. Default: ${POLICY_SERVER_URL}
  --rws-ip IP                  ABB RWS IP. Default: ${RWS_IP}
  --rws-port PORT              ABB RWS port. Default: ${RWS_PORT}
  --egm-port PORT              EGM UDP port. Default: ${EGM_PORT}
  --runtime-dir DIR            Runtime/log directory. Default: ${RUNTIME_DIR}
  --safe-runtime-dir DIR       Safe restore runtime dir. Default: ${SAFE_RUNTIME_DIR}
  -h, --help                   Show this help.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)
      EXECUTE="true"
      shift
      ;;
    --duration-sec)
      DURATION_SEC="$2"
      shift 2
      ;;
    --wait-for-egm-ready-sec)
      EGM_READY_WAIT_SEC="$2"
      shift 2
      ;;
    --bridge-max-position-step-rad)
      BRIDGE_MAX_POSITION_STEP_RAD="$2"
      shift 2
      ;;
    --policy-server-url)
      POLICY_SERVER_URL="$2"
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
HOLD_SEED_POSITIONS_RAD=""

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
import json
import os
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

joints_rad_csv_from_pose_file() {
  python3 - "$1" <<'PY'
import json
import math
import sys

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    payload = json.load(stream)
degrees = payload.get("joints_deg")
if not isinstance(degrees, list) or len(degrees) != 6:
    raise SystemExit("pose file does not contain six joints_deg values")
radians = [math.radians(float(value)) for value in degrees]
if not all(math.isfinite(value) for value in radians):
    raise SystemExit("pose file contains non-finite joints_deg values")
print(",".join(f"{value:.12g}" for value in radians))
PY
}

start_hold_seed() {
  echo "Pre-seeding forward position controller with current RWS joints before ROS/EGM startup..."
  python3 "${WS_RAMS_ROOT}/tools/seed_forward_position_hold.py" \
    --joint-state-topic "${BRIDGE_FEEDBACK_JOINT_STATE_TOPIC}" \
    --command-topic /forward_command_controller_position/commands \
    --positions-rad "${HOLD_SEED_POSITIONS_RAD}" \
    --rate-hz 50 \
    --startup-timeout-sec 0.2 >"${HOLD_SEED_LOG}" 2>&1 &
  HOLD_SEED_PID="$!"
  sleep 1
  if ! kill -0 "${HOLD_SEED_PID}" >/dev/null 2>&1; then
    cat "${HOLD_SEED_LOG}" >&2 || true
    echo "ERROR: failed to start hold seeder; aborting before ROS/ABB EGM startup." >&2
    exit 1
  fi
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
    --policy-server-url "${POLICY_SERVER_URL}" >/tmp/ws_rams_safe_restore_after_pi05_live.out 2>&1 || true
}
trap restore_safe EXIT

cd "${WS_RAMS_ROOT}"
source_ros

echo "pi05 live test starting."
echo "  mode: $([[ "${EXECUTE}" == "true" ]] && echo live-execute || echo dry-run)"
echo "  policy_backend: http_json"
echo "  policy_server_url: ${POLICY_SERVER_URL}"
echo "  duration_sec: ${DURATION_SEC}"
echo "  wait_for_egm_ready_sec: ${EGM_READY_WAIT_SEC}"
echo "  bridge_max_position_step_rad: ${BRIDGE_MAX_POSITION_STEP_RAD}"
echo "  log: ${LAUNCH_LOG}"
printf "BEFORE_POSE "
read_pose | tee "${RUNTIME_DIR}/before_pose.json"
HOLD_SEED_POSITIONS_RAD="$(joints_rad_csv_from_pose_file "${RUNTIME_DIR}/before_pose.json")"

stop_launch_pidfile "${SAFE_RUNTIME_DIR}/launch.pid"
sleep 3

PUBLISH_COMMANDS="false"
if [[ "${EXECUTE}" == "true" ]]; then
  PUBLISH_COMMANDS="true"
  start_hold_seed
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
    policy_backend:=http_json \
    policy_server_url:='${POLICY_SERVER_URL}' \
    launch_moveit_rviz:=false \
    publish_commands:='${PUBLISH_COMMANDS}' \
    control_mode:=trajectory \
    bridge_joint_state_timeout_sec:=1.0 \
    bridge_max_position_step_rad:='${BRIDGE_MAX_POSITION_STEP_RAD}' \
    bridge_joint_state_topic:='${BRIDGE_JOINT_STATE_TOPIC}' \
    bridge_feedback_joint_state_topic:='${BRIDGE_FEEDBACK_JOINT_STATE_TOPIC}' \
    bridge_feedback_joint_state_timeout_sec:=1.0 \
    bridge_step_reference_mode:=current_observation \
    bridge_max_camera_joint_skew_sec:=1.0 \
    bridge_enable_front_camera_observation:=true \
    bridge_enable_wrist_camera_observation:=true
" >"${LAUNCH_LOG}" 2>&1 < /dev/null &
echo "$!" > "${LAUNCH_PID_FILE}"

BRIDGE_READY="false"
for _ in $(seq 1 90); do
  if ros2 service list 2>/dev/null | grep -Fx /abb_pi0_bridge/activate_streaming_mode >/dev/null; then
    BRIDGE_READY="true"
    break
  fi
  sleep 1
done
sleep 3
if [[ "${BRIDGE_READY}" != "true" ]]; then
  echo "ERROR: abb_pi0_bridge did not expose services. Refusing to continue." >&2
  echo "Last launch log lines:" >&2
  tail -n 80 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi

if [[ "${EXECUTE}" == "true" ]]; then
  if ! kill -0 "${HOLD_SEED_PID}" >/dev/null 2>&1; then
    cat "${HOLD_SEED_LOG}" >&2 || true
    echo "ERROR: hold seeder is not running; aborting before ABB EGM restart/arm." >&2
    exit 1
  fi

  echo "Checking ABB EGM readiness after launching the pi05 ROS stack..."
  EGM_READY="false"
  if check_egm_ready_after_launch; then
    EGM_READY="true"
  elif (( EGM_READY_WAIT_SEC > 0 )); then
    echo "ROS is up and current-position hold commands are already being published."
    echo "Now start/restart the RAPID EGM program, and I will wait up to ${EGM_READY_WAIT_SEC}s..."
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
    echo "ERROR: ABB EGM is not ready with this newly launched pi05 ROS stack; aborting before arming output." >&2
    exit 1
  fi
fi

if [[ "${EXECUTE}" == "true" ]]; then
  echo "Switching to streaming mode and arming pi05 output for ${DURATION_SEC}s..."
  ros2 service call /abb_pi0_bridge/activate_streaming_mode std_srvs/srv/Trigger "{}" >/dev/null
  sleep 0.5
  stop_hold_seed
  ros2 service call /abb_pi0_bridge/set_streaming_arm std_srvs/srv/SetBool "{data: true}" >/dev/null
  (
    sleep 0.8
    {
      echo "=== ACTIVE PI05 WINDOW SAMPLE $(date --iso-8601=seconds) ==="
      echo "--- /abb_pi0_bridge/status ---"
      timeout 3 ros2 topic echo /abb_pi0_bridge/status --once --field data --full-length || true
      echo "--- /abb_pi0_bridge/safe_command ---"
      timeout 3 ros2 topic echo /abb_pi0_bridge/safe_command --once || true
      echo "--- /forward_command_controller_position/commands ---"
      timeout 3 ros2 topic echo /forward_command_controller_position/commands --once || true
      echo "--- /abb/control_mode ---"
      timeout 3 ros2 topic echo /abb/control_mode --once || true
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
printf "AFTER_POSE "
read_pose | tee "${RUNTIME_DIR}/after_pose.json"

echo "pi05 live test finished; restoring normal safe pi05 bringup."
