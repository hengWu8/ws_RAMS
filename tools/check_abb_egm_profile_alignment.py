#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import socket
import subprocess
from pathlib import Path
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


def _instance_attributes(payload: Any) -> dict[str, str]:
    for state in _states(payload):
        attribs = state.get("attrib")
        if isinstance(attribs, list):
            return {
                str(item.get("_title", "")): str(item.get("value", ""))
                for item in attribs
            }
    return {}


def _read_local_ipv4_addresses() -> list[str]:
    try:
        output = subprocess.check_output(["ip", "-4", "addr", "show"], text=True)
    except Exception:
        return []
    return re.findall(r"inet (\d+\.\d+\.\d+\.\d+)/", output)


def _find_rapid_value(pattern: str) -> str | None:
    rapid_path = Path("/home/rob/workspace/ws_RAMS/rapid/TRob1Main_WS_RAMS_EGM_Joint.mod")
    try:
        text = rapid_path.read_text(encoding="utf-8")
    except OSError:
        return None
    match = re.search(pattern, text)
    if not match:
        return None
    return match.group(1)


def _parse_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _print_check(name: str, passed: bool | None, detail: dict[str, Any]) -> None:
    if passed is True:
        prefix = "PASS"
    elif passed is False:
        prefix = "FAIL"
    else:
        prefix = "INFO"
    print(f"[{prefix}] {name}")
    print(json.dumps(detail, ensure_ascii=False, indent=2, sort_keys=True))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only ABB EGM profile alignment check for WS_RAMS."
    )
    parser.add_argument("--rws-ip", default="192.168.125.1")
    parser.add_argument("--rws-port", default="80")
    parser.add_argument("--rws-user", default="Default User")
    parser.add_argument("--rws-password", default="robotics")
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument("--expected-egm-host", default="192.168.125.109")
    parser.add_argument("--expected-egm-port", type=int, default=6515)
    parser.add_argument("--expected-ext-motion-profile", default="default")
    parser.add_argument("--expected-uc-device", default="ROB_1")
    args = parser.parse_args()

    base_url = f"http://{args.rws_ip}:{args.rws_port}"
    session = requests.Session()
    session.trust_env = False
    session.auth = HTTPDigestAuth(args.rws_user, args.rws_password)
    session.headers.update({"Connection": "keep-alive"})

    all_required_ok = True

    local_ipv4 = _read_local_ipv4_addresses()
    _print_check(
        "local_ipv4_inventory",
        None,
        {
            "hostname": socket.gethostname(),
            "local_ipv4": local_ipv4,
            "expected_egm_host": args.expected_egm_host,
        },
    )

    ok, payload, error = _rws_get(
        session, base_url, "/rw/cfg/SIO/COM_TRP/instances/ROB_1?json=1", args.timeout_sec
    )
    com_trp_attrs = _instance_attributes(payload) if ok else {}
    remote_addr = com_trp_attrs.get("RemoteAdress")
    remote_port = com_trp_attrs.get("RemotePortNumber")
    remote_addr_ok = remote_addr == args.expected_egm_host and remote_addr in local_ipv4
    remote_port_ok = str(args.expected_egm_port) == remote_port
    _print_check(
        "com_trp_rob_1_target",
        remote_addr_ok and remote_port_ok,
        {
            "error": None if ok else error,
            "attributes": com_trp_attrs,
            "expected_remote_address": args.expected_egm_host,
            "expected_remote_port": args.expected_egm_port,
            "local_ipv4": local_ipv4,
        },
    )
    all_required_ok = all_required_ok and remote_addr_ok and remote_port_ok

    ok, payload, error = _rws_get(
        session,
        base_url,
        f"/rw/cfg/MOC/EXT_MOTION_DATA/instances/{args.expected_ext_motion_profile}?json=1",
        args.timeout_sec,
    )
    ext_motion_attrs = _instance_attributes(payload) if ok else {}
    _print_check(
        "ext_motion_profile_snapshot",
        ok if ok else False,
        {
            "error": None if ok else error,
            "profile_name": args.expected_ext_motion_profile,
            "attributes": ext_motion_attrs,
        },
    )
    all_required_ok = all_required_ok and ok

    max_allowed_speed_factor = _parse_float(ext_motion_attrs.get("max_allowed_speed_factor"))
    ext_motion_kp = _parse_float(ext_motion_attrs.get("ext_motion_Kp"))
    ext_motion_filter_bandwidth = _parse_float(ext_motion_attrs.get("ext_motion_filter_bandwidth"))
    ramp_time = _parse_float(ext_motion_attrs.get("ramp_time"))
    _print_check(
        "ext_motion_profile_interpretation",
        None,
        {
            "profile_name": args.expected_ext_motion_profile,
            "max_allowed_speed_factor": max_allowed_speed_factor,
            "ext_motion_Kp": ext_motion_kp,
            "ext_motion_filter_bandwidth": ext_motion_filter_bandwidth,
            "ramp_time": ramp_time,
            "note": (
                "These values likely govern how aggressively RAPID EGM profile "
                f"'{args.expected_ext_motion_profile}' accepts external motion correction."
            ),
        },
    )

    ok, payload, error = _rws_get(
        session,
        base_url,
        "/rw/cfg/MOC/MOTION_PLANNER/instances/motion_planner_1?json=1",
        args.timeout_sec,
    )
    motion_planner_attrs = _instance_attributes(payload) if ok else {}
    _print_check(
        "motion_planner_1_snapshot",
        ok if ok else False,
        {
            "error": None if ok else error,
            "attributes": {
                key: motion_planner_attrs.get(key)
                for key in (
                    "name",
                    "std_servo_queue_time",
                    "group_queue_time",
                    "dynamic_resolution",
                    "path_resolution",
                    "ipol_prefetch_time",
                    "interpolation_priority",
                )
            },
        },
    )

    rapid_profile = _find_rapid_value(r'EGMSetupUC\s+ROB_1,\s*egm_id,\s*"([^"]+)"')
    rapid_uc_device = _find_rapid_value(r'EGMSetupUC\s+ROB_1,\s*egm_id,\s*"[^"]+",\s*"([^"]+)"')
    rapid_cond_time = _find_rapid_value(r'EGMRunJoint[^\n]*\\CondTime:=(\d+)')
    rapid_ramp_out_time = _find_rapid_value(r'EGMRunJoint[^\n]*\\RampOutTime:=(\d+)')
    rapid_max_speed_deviation = _find_rapid_value(r'\\MaxSpeedDeviation:=(\d+(?:\.\d+)?)')
    _print_check(
        "local_rapid_template_snapshot",
        rapid_profile == args.expected_ext_motion_profile
        and rapid_uc_device == args.expected_uc_device,
        {
            "rapid_file": "/home/rob/workspace/ws_RAMS/rapid/TRob1Main_WS_RAMS_EGM_Joint.mod",
            "profile_name": rapid_profile,
            "uc_device": rapid_uc_device,
            "cond_time": rapid_cond_time,
            "ramp_out_time": rapid_ramp_out_time,
            "max_speed_deviation": rapid_max_speed_deviation,
            "expected_profile_name": args.expected_ext_motion_profile,
            "expected_uc_device": args.expected_uc_device,
        },
    )
    all_required_ok = all_required_ok and (
        rapid_profile == args.expected_ext_motion_profile
        and rapid_uc_device == args.expected_uc_device
    )

    ok, payload, error = _rws_get(
        session,
        base_url,
        "/rw/rapid/symbol/data/RAPID/T_ROB1/TRob1Main/egm_condition?json=1",
        args.timeout_sec,
    )
    egm_condition = None
    if ok:
        for state in _states(payload):
            if "value" in state:
                egm_condition = state["value"]
                break
    _print_check(
        "loaded_rapid_runtime_snapshot",
        None,
        {
            "error": None if ok else error,
            "egm_condition": egm_condition,
            "note": (
                "RWS exposes egm_condition directly, but exact EGMRunJoint "
                "instruction arguments cannot be reliably fetched via the same API path here."
            ),
        },
    )

    if all_required_ok:
        print(
            "\nRequired alignment checks passed. The remaining unknown is whether the controller's "
            "EXT_MOTION_DATA/default profile is equivalent to the RobotStudio profile that produced visible motion."
        )
        return 0

    print(
        "\nOne or more required alignment checks failed. Fix those first before comparing subtle "
        "EGM gain/filter behavior against RobotStudio."
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
