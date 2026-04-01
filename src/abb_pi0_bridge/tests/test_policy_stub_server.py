from abb_pi0_bridge.policy_stub_server import build_hold_response, build_left_response


def test_build_hold_response_echoes_joint_positions():
    payload = {
        "observation": {
            "joint_positions": [0.1, -0.2, 0.3],
        }
    }

    assert build_hold_response(payload) == {
        "joint_positions": [0.1, -0.2, 0.3],
        "stub_mode": "hold_current_joint_positions",
    }


def test_build_left_response_steps_one_joint():
    payload = {
        "observation": {
            "joint_positions": [0.1, -0.2, 0.3],
        }
    }

    assert build_left_response(payload, step_rad=0.05, joint_index=1) == {
        "joint_positions": [0.1, -0.15000000000000002, 0.3],
        "stub_mode": "left_joint_step",
        "joint_index": 1,
        "step_rad": 0.05,
    }
