import json
from typing import Any, Callable
import urllib.error
import urllib.request

from .bridge_core import PolicyObservation
from .protocol import build_policy_request, extract_joint_positions


class MockHoldPolicy:
    """Local safety-first backend that simply holds the current joint state."""

    def compute_command(self, observation: PolicyObservation) -> tuple[float, ...]:
        return observation.positions


class HttpPolicyServerClient:
    """Minimal HTTP JSON policy client stub for future remote pi0/openpi serving."""

    def __init__(
        self,
        server_url: str,
        timeout_sec: float,
        urlopen: Callable[..., Any] | None = None,
    ) -> None:
        self.server_url = server_url
        self.timeout_sec = timeout_sec
        self._urlopen = urlopen or urllib.request.urlopen

    def compute_command(self, observation: PolicyObservation) -> tuple[float, ...]:
        if not self.server_url:
            raise ValueError("policy_server_url must be set when using the http_json backend.")
        if self.timeout_sec <= 0.0:
            raise ValueError("policy_request_timeout_sec must be positive.")

        payload = json.dumps(build_policy_request(observation)).encode("utf-8")
        request = urllib.request.Request(
            self.server_url,
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        try:
            with self._urlopen(request, timeout=self.timeout_sec) as response:
                response_body = response.read().decode("utf-8")
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise ValueError(f"Policy server request failed: {exc}") from exc

        try:
            response_payload = json.loads(response_body)
        except json.JSONDecodeError as exc:
            raise ValueError("Policy server returned invalid JSON.") from exc

        return extract_joint_positions(response_payload, observation.joint_names)
def build_policy_backend(
    policy_backend: str,
    policy_server_url: str = "",
    policy_request_timeout_sec: float = 0.2,
):
    if policy_backend == "mock_hold":
        return MockHoldPolicy()
    if policy_backend == "http_json":
        return HttpPolicyServerClient(
            server_url=policy_server_url,
            timeout_sec=policy_request_timeout_sec,
        )
    raise ValueError(
        f"Unsupported policy_backend '{policy_backend}'. "
        "Phase-2 currently supports 'mock_hold' and 'http_json'."
    )
