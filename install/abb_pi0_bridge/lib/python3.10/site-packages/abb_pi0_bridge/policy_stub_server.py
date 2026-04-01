import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from typing import Any


def build_hold_response(request_payload: dict[str, Any]) -> dict[str, Any]:
    observation = request_payload.get("observation", {})
    joint_positions = observation.get("joint_positions", [])
    return {
        "joint_positions": joint_positions,
        "stub_mode": "hold_current_joint_positions",
    }


def build_left_response(
    request_payload: dict[str, Any],
    step_rad: float,
    joint_index: int,
) -> dict[str, Any]:
    if step_rad == 0.0:
        raise ValueError("step_rad must be non-zero for left mode.")
    if joint_index < 0:
        raise ValueError("joint_index must be non-negative.")

    observation = request_payload.get("observation", {})
    joint_positions = list(observation.get("joint_positions", []))
    if len(joint_positions) == 0:
        raise ValueError("observation.joint_positions is missing or empty.")
    if joint_index >= len(joint_positions):
        raise ValueError("joint_index is out of range for the observation vector.")

    joint_positions[joint_index] = float(joint_positions[joint_index]) + float(step_rad)
    return {
        "joint_positions": joint_positions,
        "stub_mode": "left_joint_step",
        "joint_index": joint_index,
        "step_rad": step_rad,
    }


def build_policy_response(
    request_payload: dict[str, Any],
    mode: str,
    step_rad: float,
    joint_index: int,
) -> dict[str, Any]:
    if mode == "hold":
        return build_hold_response(request_payload)
    if mode == "left":
        return build_left_response(request_payload, step_rad=step_rad, joint_index=joint_index)
    raise ValueError(f"Unsupported policy stub mode: {mode}")


class _PolicyStubHandler(BaseHTTPRequestHandler):
    server_version = "Pi0PolicyStub/0.1"

    def do_GET(self) -> None:
        if self.path != "/healthz":
            self._write_json(404, {"error": "not_found"})
            return
        self._write_json(200, {"ok": True})

    def do_POST(self) -> None:
        if self.path != "/infer":
            self._write_json(404, {"error": "not_found"})
            return

        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length).decode("utf-8")

        try:
            request_payload = json.loads(raw_body) if raw_body else {}
        except json.JSONDecodeError:
            self._write_json(400, {"error": "invalid_json"})
            return

        try:
            response_payload = build_policy_response(
                request_payload,
                mode=getattr(self.server, "stub_mode", "hold"),
                step_rad=float(getattr(self.server, "stub_step_rad", 0.003)),
                joint_index=int(getattr(self.server, "stub_joint_index", 0)),
            )
        except ValueError as exc:
            self._write_json(400, {"error": str(exc)})
            return

        self._write_json(200, response_payload)

    def log_message(self, format: str, *args) -> None:
        return

    def _write_json(self, status_code: int, payload: dict[str, Any]) -> None:
        response = json.dumps(payload).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(response)))
        self.end_headers()
        self.wfile.write(response)


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Local HTTP stub for pi0/openpi policy serving.")
    parser.add_argument("--host", default="127.0.0.1", help="Bind address for the HTTP stub.")
    parser.add_argument("--port", type=int, default=8000, help="Bind port for the HTTP stub.")
    parser.add_argument(
        "--mode",
        choices=("hold", "left"),
        default="hold",
        help="Stub policy behavior. 'hold' echoes the current joint positions. 'left' nudges one joint.",
    )
    parser.add_argument(
        "--step-rad",
        type=float,
        default=0.003,
        help="Joint step used by left mode. Positive or negative values are allowed.",
    )
    parser.add_argument(
        "--joint-index",
        type=int,
        default=0,
        help="Zero-based joint index used by left mode.",
    )
    parsed = parser.parse_args(args=args)

    server = ThreadingHTTPServer((parsed.host, parsed.port), _PolicyStubHandler)
    server.stub_mode = parsed.mode
    server.stub_step_rad = parsed.step_rad
    server.stub_joint_index = parsed.joint_index
    print(f"pi0 policy stub listening on http://{parsed.host}:{parsed.port}")
    print(
        f"policy stub mode={parsed.mode}, step_rad={parsed.step_rad}, joint_index={parsed.joint_index}"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
