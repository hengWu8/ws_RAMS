#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
from typing import Any

import requests
from requests.auth import HTTPDigestAuth


def _rws_get(session: requests.Session, base_url: str, path: str, timeout: float) -> tuple[bool, Any, str]:
    try:
        response = session.get(base_url + path, timeout=timeout)
        text = response.text
        response.raise_for_status()
        return True, response.json(), text
    except Exception as exc:
        return False, None, str(exc)


def _states(payload: Any) -> list[dict[str, Any]]:
    try:
        return payload.get("_embedded", {}).get("_state", []) or []
    except AttributeError:
        return []


def _first_value(payload: Any) -> str | None:
    for state in _states(payload):
        if "value" in state:
            return str(state["value"])
    return None


def _parse_minmax(raw: str | None) -> tuple[float, float] | None:
    if raw is None:
        return None
    numbers = [float(value) for value in re.findall(r"[-+]?\d+(?:\.\d+)?", raw)]
    if len(numbers) < 2:
        return None
    return numbers[0], numbers[1]


def _find_local_egm_activity_lines() -> list[dict[str, Any]]:
    candidates: list[Path] = []
    rapid_dir = Path("/home/rob/workspace/ws_RAMS/rapid")
    if rapid_dir.exists():
        candidates.extend(sorted(rapid_dir.glob("TRob1Main*.mod")))
    candidates.append(Path("/home/rob/workspace/ws_RAMS/src/abb_ros2_upstream/robot_studio_resources/TRob1Main.mod"))

    results: list[dict[str, Any]] = []
    for candidate in candidates:
        try:
            for lineno, line in enumerate(candidate.read_text(encoding="utf-8").splitlines(), start=1):
                stripped = line.lstrip()
                if stripped.startswith("!"):
                    continue
                if stripped.startswith("EGMRunJoint") or stripped.startswith("EGMWaitCond"):
                    results.append({"path": str(candidate), "line": lineno, "statement": stripped.split()[0]})
        except OSError:
            continue
    return results


def _read_ros_egm_diagnostics(topic: str, timeout_sec: float) -> tuple[bool, Any, str | None]:
    ros2 = shutil.which("ros2")
    if ros2 is None:
      return False, None, "ros2 CLI not found in PATH"

    try:
        completed = subprocess.run(
            [ros2, "topic", "echo", "--once", topic],
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            check=True,
        )
    except Exception as exc:
        return False, None, str(exc)

    output = completed.stdout
    match = re.search(r"data:\s*'(.*?)'\s*---", output, re.DOTALL)
    if not match:
        return False, None, "failed to parse ROS topic output"

    raw = match.group(1)
    try:
        return True, json.loads(raw.replace("NaN", "null")), None
    except json.JSONDecodeError as exc:
        return False, None, f"invalid diagnostics JSON: {exc}"


def _sample_ros_egm_diagnostics(topic: str, sample_count: int, timeout_sec: float) -> tuple[bool, Any, str | None]:
    samples: list[dict[str, Any]] = []
    errors: list[str] = []

    for _ in range(max(sample_count, 1)):
        ok, payload, error = _read_ros_egm_diagnostics(topic, timeout_sec)
        if ok and isinstance(payload, dict):
            payload = dict(payload)
            payload["sample_wall_time"] = time.time()
            samples.append(payload)
        elif error:
            errors.append(error)

    if not samples:
        return False, None, "; ".join(errors) if errors else "no diagnostics samples received"

    states = [sample.get("egm_state") for sample in samples]
    sequence_numbers = [sample.get("sequence_number") for sample in samples]
    return (
        True,
        {
            "sample_count": len(samples),
            "samples": samples,
            "states": states,
            "sequence_numbers": sequence_numbers,
            "running_count": sum(1 for state in states if state == "EGM_RUNNING"),
            "stopped_count": sum(1 for state in states if state == "EGM_STOPPED"),
            "all_running": all(state == "EGM_RUNNING" for state in states),
            "any_running": any(state == "EGM_RUNNING" for state in states),
            "state_changes": sum(1 for idx in range(1, len(states)) if states[idx] != states[idx - 1]),
            "sequence_changes": sum(
                1 for idx in range(1, len(sequence_numbers)) if sequence_numbers[idx] != sequence_numbers[idx - 1]
            ),
        },
        None,
    )


def _read_latest_ros2_control_egm_log(max_diag_age_sec: float, max_cycle_gap_sec: float) -> tuple[bool, Any, str | None]:
    log_dir = Path(os.path.expanduser("~/.ros/log"))
    candidates = sorted(log_dir.glob("ros2_control_node_*.log"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not candidates:
        return False, None, "no ros2_control_node log found"

    write_pattern = re.compile(
        r"\[(?P<stamp>\d+(?:\.\d+)?)\].*?EGM write diag group=(?P<group>.*?) active=(?P<active>true|false) seq=(?P<seq>\d+) "
        r"motor=(?P<motor>\S+) rapid=(?P<rapid>\S+) egm=(?P<egm>\S+) "
        r"max_cmd_minus_state_deg=(?P<cmd_state>[-+]?\d+(?:\.\d+)?) "
        r"max_planned_minus_state_deg=(?P<planned_state>[-+]?\d+(?:\.\d+)?) "
        r"max_cmd_minus_planned_deg=(?P<cmd_planned>[-+]?\d+(?:\.\d+)?)"
    )
    transition_pattern = re.compile(
        r"\[(?P<stamp>\d+(?:\.\d+)?)\].*?EGM status transition group=(?P<group>.*?) active=(?P<active>true|false) "
        r"seq=(?P<seq>\d+) motor=(?P<motor>\S+) rapid=(?P<rapid>\S+) egm=(?P<egm>\S+)"
    )
    latest_match = None
    timeline: list[dict[str, Any]] = []
    latest_file = candidates[0]
    try:
        for line in latest_file.read_text(encoding="utf-8", errors="replace").splitlines():
            match = write_pattern.search(line)
            if match:
                latest_match = match
                timeline.append(
                    {
                        "stamp": float(match.group("stamp")),
                        "kind": "write",
                        "egm_state": match.group("egm"),
                        "active": match.group("active") == "true",
                        "motor_state": match.group("motor"),
                        "rapid_execution_state": match.group("rapid"),
                        "sequence_number": int(match.group("seq")),
                    }
                )
                continue
            transition = transition_pattern.search(line)
            if transition:
                timeline.append(
                    {
                        "stamp": float(transition.group("stamp")),
                        "kind": "transition",
                        "egm_state": transition.group("egm"),
                        "active": transition.group("active") == "true",
                        "motor_state": transition.group("motor"),
                        "rapid_execution_state": transition.group("rapid"),
                        "sequence_number": int(transition.group("seq")),
                    }
                )
    except OSError as exc:
        return False, None, str(exc)

    if latest_match is None:
        return False, None, f"no EGM write diagnostics found in {latest_file}"

    latest_stamp = max(item["stamp"] for item in timeline) if timeline else None
    recent_window_sec = 15.0
    recent = [
        item for item in timeline if latest_stamp is not None and item["stamp"] >= latest_stamp - recent_window_sec
    ]
    recent_states = [item["egm_state"] for item in recent]

    latest_stamp_sec = float(latest_match.group("stamp"))
    now_sec = time.time()
    diagnostics_age_sec = max(0.0, now_sec - latest_stamp_sec)
    transition_events = [item for item in timeline if item.get("kind") == "transition"]
    running_transition_ages = [
        now_sec - item["stamp"] for item in transition_events if item.get("egm_state") == "EGM_RUNNING"
    ]
    last_running_transition_age_sec = min(running_transition_ages) if running_transition_ages else None

    running_durations_sec: list[float] = []
    active_running_start: float | None = None
    for item in transition_events:
        if item.get("rapid_execution_state") != "RAPID_RUNNING":
            continue
        if item.get("egm_state") == "EGM_RUNNING" and item.get("active"):
            active_running_start = item["stamp"]
        elif item.get("egm_state") == "EGM_STOPPED" and active_running_start is not None:
            running_durations_sec.append(item["stamp"] - active_running_start)
            active_running_start = None

    lifecycle_mode = "unknown"
    if latest_match.group("egm") == "EGM_RUNNING":
        lifecycle_mode = "currently_running"
    elif last_running_transition_age_sec is not None and last_running_transition_age_sec <= max_cycle_gap_sec:
        lifecycle_mode = "cyclic_completion"

    data = {
        "source": str(latest_file),
        "source_mtime_sec": latest_file.stat().st_mtime,
        "latest_stamp_sec": latest_stamp_sec,
        "diagnostics_age_sec": diagnostics_age_sec,
        "diagnostics_fresh_enough": diagnostics_age_sec <= max_diag_age_sec,
        "group": latest_match.group("group"),
        "active": latest_match.group("active") == "true",
        "sequence_number": int(latest_match.group("seq")),
        "motor_state": latest_match.group("motor"),
        "rapid_execution_state": latest_match.group("rapid"),
        "egm_state": latest_match.group("egm"),
        "max_cmd_minus_state_deg": float(latest_match.group("cmd_state")),
        "max_planned_minus_state_deg": float(latest_match.group("planned_state")),
        "max_cmd_minus_planned_deg": float(latest_match.group("cmd_planned")),
        "recent_window_sec": recent_window_sec,
        "recent_sample_count": len(recent),
        "recent_running_count": sum(1 for state in recent_states if state == "EGM_RUNNING"),
        "recent_stopped_count": sum(1 for state in recent_states if state == "EGM_STOPPED"),
        "recent_state_changes": sum(
            1 for idx in range(1, len(recent_states)) if recent_states[idx] != recent_states[idx - 1]
        ),
        "recent_states_tail": recent_states[-10:],
        "last_running_transition_age_sec": last_running_transition_age_sec,
        "max_allowed_cycle_gap_sec": max_cycle_gap_sec,
        "running_durations_sec_tail": running_durations_sec[-5:],
        "lifecycle_mode": lifecycle_mode,
    }
    return True, data, None


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only ABB RAPID/EGM readiness check for WS_RAMS motion tests.")
    parser.add_argument("--rws-ip", default="192.168.125.1")
    parser.add_argument("--rws-port", default="80")
    parser.add_argument("--rws-user", default="Default User")
    parser.add_argument("--rws-password", default="robotics")
    parser.add_argument("--timeout-sec", type=float, default=2.0)
    parser.add_argument(
        "--min-egm-window-deg",
        type=float,
        default=1.0,
        help="Required absolute min/max egm_condition range in degrees for this check to pass.",
    )
    parser.add_argument(
        "--ros-egm-diagnostic-topic",
        default="/abb_hardware_interface/egm_diagnostics",
        help="ROS topic publishing live hardware-level EGM diagnostics JSON.",
    )
    parser.add_argument(
        "--ros-echo-timeout-sec",
        type=float,
        default=3.0,
        help="Timeout for reading the live ROS EGM diagnostics topic.",
    )
    parser.add_argument(
        "--egm-sample-count",
        type=int,
        default=5,
        help="Number of live EGM diagnostic samples to collect before declaring readiness.",
    )
    parser.add_argument(
        "--max-diag-age-sec",
        type=float,
        default=5.0,
        help="Maximum allowed age for fallback ros2_control EGM diagnostics before treating them as stale.",
    )
    parser.add_argument(
        "--max-egm-cycle-gap-sec",
        type=float,
        default=45.0,
        help=(
            "Maximum time since the last EGM_RUNNING transition that is accepted as normal cyclic "
            "completion/restart behavior for RAPID loops using EGMWaitCond."
        ),
    )
    parser.add_argument(
        "--allow-cyclic-egm-completion",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Accept CondTime-driven EGM_RUNNING/EGM_STOPPED cycles as healthy when RAPID remains running.",
    )
    args = parser.parse_args()

    base_url = f"http://{args.rws_ip}:{args.rws_port}"
    session = requests.Session()
    session.trust_env = False
    session.auth = HTTPDigestAuth(args.rws_user, args.rws_password)
    session.headers.update({"Connection": "keep-alive"})

    checks: list[tuple[str, bool, Any]] = []

    ok, payload, error = _rws_get(session, base_url, "/rw/panel/opmode?json=1", args.timeout_sec)
    opmode = _states(payload)[0].get("opmode") if ok and _states(payload) else None
    checks.append(("opmode_auto", opmode == "AUTO", {"opmode": opmode, "error": None if ok else error}))

    ok, payload, error = _rws_get(session, base_url, "/rw/panel/ctrlstate?json=1", args.timeout_sec)
    ctrlstate = _states(payload)[0].get("ctrlstate") if ok and _states(payload) else None
    checks.append(("motors_on", ctrlstate == "motoron", {"ctrlstate": ctrlstate, "error": None if ok else error}))

    ok, payload, error = _rws_get(session, base_url, "/rw/rapid/tasks/T_ROB1?json=1", args.timeout_sec)
    task = _states(payload)[0] if ok and _states(payload) else {}
    checks.append(("rapid_started", task.get("excstate") == "started", {"task": task, "error": None if ok else error}))

    ok, payload, error = _rws_get(session, base_url, "/rw/rapid/tasks/T_ROB1/pcp?json=1", args.timeout_sec)
    pcp = _states(payload) if ok else []
    program_pointer = next((state for state in pcp if state.get("_title") == "progpointer"), {})
    local_egm_activity_lines = _find_local_egm_activity_lines()
    actual_beginposition = str(program_pointer.get("beginposition", ""))
    actual_line = None
    try:
        actual_line = int(actual_beginposition.split(",", 1)[0])
    except (TypeError, ValueError):
        actual_line = None
    expected_lines = [entry["line"] for entry in local_egm_activity_lines]
    line_matches = True
    if expected_lines:
        # RWS line numbers can be off by one compared with the local RAPID file
        # after comments/blank lines are edited locally. Accept a tiny tolerance
        # and rely on live EGM diagnostics below to prove the session is running.
        line_matches = actual_line is not None and any(
            abs(actual_line - int(line)) <= 1 for line in expected_lines
        )
    checks.append(
        (
            "program_pointer_on_egm_activity",
            program_pointer.get("modulemame") == "TRob1Main"
            and program_pointer.get("routinename") == "main"
            and line_matches,
            {
                "program_pointer": program_pointer,
                "actual_line": actual_line,
                "line_tolerance": 1,
                "expected_egm_activity_lines": local_egm_activity_lines,
                "error": None if ok else error,
            },
        )
    )

    ok, payload, error = _rws_get(
        session,
        base_url,
        "/rw/rapid/symbol/data/RAPID/T_ROB1/TRob1Main/egm_condition?json=1",
        args.timeout_sec,
    )
    egm_condition_raw = _first_value(payload) if ok else None
    egm_condition = _parse_minmax(egm_condition_raw)
    required_window = abs(float(args.min_egm_window_deg))
    condition_wide = (
        egm_condition is not None
        and egm_condition[0] <= -required_window
        and egm_condition[1] >= required_window
    )
    checks.append(
        (
            "egm_condition_wide_enough_for_visible_test",
            condition_wide,
            {
                "egm_condition": egm_condition_raw,
                "parsed_deg": egm_condition,
                "required_abs_window_deg": required_window,
                "error": None if ok else error,
            },
        )
    )

    ros2_control_pid = None
    try:
        completed = subprocess.run(
            ["pgrep", "-f", "/opt/ros/humble/lib/controller_manager/ros2_control_node"],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
        pids = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
        if pids:
            ros2_control_pid = pids[-1]
    except Exception:
        ros2_control_pid = None

    checks.append(
        (
            "ros2_control_process_alive",
            ros2_control_pid is not None,
            {
                "ros2_control_pid": ros2_control_pid,
            },
        )
    )

    ok, payload, error = _sample_ros_egm_diagnostics(
        args.ros_egm_diagnostic_topic, args.egm_sample_count, args.ros_echo_timeout_sec
    )
    source = "topic"
    if not ok:
        ok, payload, fallback_error = _read_latest_ros2_control_egm_log(
            args.max_diag_age_sec, args.max_egm_cycle_gap_sec
        )
        if ok:
            error = None
            source = "ros2_control_log"
        else:
            error = f"topic_error={error}; log_error={fallback_error}"
    egm_state = None
    lifecycle_acceptable = False
    if ok and isinstance(payload, dict):
        if source == "topic":
            states = payload.get("states", [])
            egm_state = states[-1] if states else None
            sequence_is_live = payload.get("sequence_changes", 0) > 0
            lifecycle_acceptable = sequence_is_live and (
                bool(payload.get("all_running"))
                or (bool(args.allow_cyclic_egm_completion) and bool(payload.get("any_running")))
            )
        else:
            egm_state = payload.get("egm_state")
            base_ok = (
                bool(payload.get("diagnostics_fresh_enough"))
                and payload.get("motor_state") == "MOTORS_ON"
                and payload.get("rapid_execution_state") == "RAPID_RUNNING"
                and bool(payload.get("active"))
            )
            stable_running = egm_state == "EGM_RUNNING"
            cyclic_completion = (
                bool(args.allow_cyclic_egm_completion)
                and payload.get("lifecycle_mode") == "cyclic_completion"
                and payload.get("last_running_transition_age_sec") is not None
                and payload.get("last_running_transition_age_sec") <= args.max_egm_cycle_gap_sec
            )
            lifecycle_acceptable = base_ok and (stable_running or cyclic_completion)
    checks.append(
        (
            "live_egm_lifecycle_acceptable",
            lifecycle_acceptable,
            {
                "source": source,
                "topic": args.ros_egm_diagnostic_topic,
                "diagnostics": payload,
                "error": error,
            },
        )
    )

    all_ok = True
    for name, passed, detail in checks:
        all_ok = all_ok and passed
        print(f"[{'PASS' if passed else 'FAIL'}] {name}")
        print(json.dumps(detail, ensure_ascii=False, indent=2))

    if not all_ok:
        print("\nMotion output is not ready for a visible test. Do not increase ROS command magnitudes until the failed items are fixed.")
        print("Recommended robot-side module: rapid/TRob1Main_WS_RAMS_EGM_Joint_PrepMove_Blocking_CondTime600.mod")
        return 1

    print("\nABB EGM appears ready for a visible ROS-side motion test.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
