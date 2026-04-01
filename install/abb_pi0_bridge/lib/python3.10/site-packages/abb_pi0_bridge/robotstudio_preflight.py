import argparse
import socket
import urllib.error
import urllib.request


def tcp_reachable(host: str, port: int, timeout_sec: float) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout_sec)
        return sock.connect_ex((host, port)) == 0


def http_health_ok(url: str, timeout_sec: float, use_env_proxy: bool = False) -> bool:
    try:
        if use_env_proxy:
            opener = urllib.request.build_opener()
        else:
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(url, timeout=timeout_sec) as response:
            return 200 <= response.status < 300
    except (urllib.error.URLError, TimeoutError, OSError):
        return False


def build_robotstudio_launch_command(
    robotstudio_rws_ip: str,
    robotstudio_rws_port: int,
    egm_port: int,
    policy_server_url: str,
) -> str:
    return (
        "ros2 launch abb_pi0_bridge robotstudio_pi0.launch.py "
        f"robotstudio_rws_ip:={robotstudio_rws_ip} "
        f"robotstudio_rws_port:={robotstudio_rws_port} "
        f"egm_port:={egm_port} "
        f"policy_server_url:={policy_server_url}"
    )


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Preflight checks for RobotStudio + pi0 bringup.")
    parser.add_argument("--rws-ip", required=True, help="RobotStudio virtual controller RWS IP.")
    parser.add_argument("--rws-port", type=int, default=80, help="RobotStudio RWS TCP port.")
    parser.add_argument("--egm-port", type=int, default=6515, help="Expected EGM UDP port.")
    parser.add_argument(
        "--policy-health-url",
        default="http://127.0.0.1:8000/healthz",
        help="HTTP health endpoint for the local or remote policy server.",
    )
    parser.add_argument(
        "--policy-server-url",
        default="http://127.0.0.1:8000/infer",
        help="HTTP inference endpoint for abb_pi0_bridge.",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=1.0,
        help="Timeout for each individual network probe.",
    )
    parser.add_argument(
        "--use-env-proxy",
        action="store_true",
        help="Use HTTP(S)_PROXY from the current environment for policy health probing.",
    )
    parsed = parser.parse_args(args=args)

    rws_ok = tcp_reachable(parsed.rws_ip, parsed.rws_port, parsed.timeout_sec)
    policy_ok = http_health_ok(parsed.policy_health_url, parsed.timeout_sec, parsed.use_env_proxy)

    print(f"RWS reachable ({parsed.rws_ip}:{parsed.rws_port}): {rws_ok}")
    print(f"Policy health reachable ({parsed.policy_health_url}): {policy_ok}")
    print(f"Expected EGM UDP port: {parsed.egm_port}")
    print("Suggested launch command:")
    print(
        build_robotstudio_launch_command(
            robotstudio_rws_ip=parsed.rws_ip,
            robotstudio_rws_port=parsed.rws_port,
            egm_port=parsed.egm_port,
            policy_server_url=parsed.policy_server_url,
        )
    )
