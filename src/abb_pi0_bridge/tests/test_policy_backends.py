import json

import pytest

from abb_pi0_bridge.bridge_core import PolicyObservation
from abb_pi0_bridge.mock_policy import HttpPolicyServerClient, extract_joint_positions


class _FakeResponse:
    def __init__(self, payload: dict):
        self._payload = payload

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False

    def read(self):
        return json.dumps(self._payload).encode("utf-8")


def test_extract_joint_positions_accepts_joint_dict():
    positions = extract_joint_positions(
        {"joint_positions": {"joint_1": 0.1, "joint_2": -0.2}},
        ("joint_1", "joint_2"),
    )

    assert positions == (0.1, -0.2)


def test_http_policy_server_client_parses_response():
    observation = PolicyObservation(
        stamp_sec=1.0,
        joint_names=("joint_1", "joint_2"),
        positions=(0.0, 0.0),
        velocities=(0.0, 0.0),
        efforts=(0.0, 0.0),
    )

    client = HttpPolicyServerClient(
        server_url="http://localhost:8000/infer",
        timeout_sec=0.2,
        urlopen=lambda request, timeout: _FakeResponse({"joint_positions": [0.3, -0.4]}),
    )

    assert client.compute_command(observation) == (0.3, -0.4)


def test_extract_joint_positions_rejects_missing_field():
    with pytest.raises(ValueError):
        extract_joint_positions({}, ("joint_1", "joint_2"))
