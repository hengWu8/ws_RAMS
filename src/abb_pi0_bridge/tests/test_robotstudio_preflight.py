from abb_pi0_bridge.robotstudio_preflight import build_robotstudio_launch_command


def test_build_robotstudio_launch_command_contains_expected_arguments():
    command = build_robotstudio_launch_command(
        robotstudio_rws_ip="192.168.125.1",
        robotstudio_rws_port=8080,
        egm_port=6515,
        policy_server_url="http://127.0.0.1:8000/infer",
    )

    assert "robotstudio_rws_ip:=192.168.125.1" in command
    assert "robotstudio_rws_port:=8080" in command
    assert "egm_port:=6515" in command
    assert "policy_server_url:=http://127.0.0.1:8000/infer" in command
