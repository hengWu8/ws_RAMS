import json
import math
from typing import Sequence
from typing import Any, Callable
import urllib.error
import urllib.request

from .bridge_core import PolicyObservation
from .protocol import build_policy_request, extract_joint_positions


class MockHoldPolicy:
    """Local safety-first backend that simply holds the current joint state."""

    def compute_command(self, observation: PolicyObservation) -> tuple[float, ...]:
        return observation.positions

    def reset(self) -> None:
        return None


class CartesianStepPolicy:
    """Local deterministic Cartesian smoke-test policy.

    It converts a tiny TCP translation into joint targets with a damped
    least-squares numerical position Jacobian. The bridge still applies the
    normal joint step limits and Cartesian workspace guard after this policy.
    """

    def __init__(
        self,
        *,
        forward_kinematics,
        direction_xyz: Sequence[float] = (0.0, 1.0, 0.0),
        step_m: float = 0.001,
        damping: float = 0.05,
        finite_difference_epsilon_rad: float = 1e-4,
        max_joint_delta_rad: float = 0.005,
    ) -> None:
        if forward_kinematics is None:
            raise ValueError("CartesianStepPolicy requires a forward_kinematics helper.")
        self.forward_kinematics = forward_kinematics
        self.direction_xyz = _normalize_vector(direction_xyz)
        self.step_m = _positive_finite(step_m, "step_m")
        self.damping = _positive_finite(damping, "damping")
        self.finite_difference_epsilon_rad = _positive_finite(
            finite_difference_epsilon_rad,
            "finite_difference_epsilon_rad",
        )
        self.max_joint_delta_rad = _positive_finite(max_joint_delta_rad, "max_joint_delta_rad")

    def compute_command(self, observation: PolicyObservation) -> tuple[float, ...]:
        current_positions = tuple(float(value) for value in observation.positions)
        joint_count = len(current_positions)
        if joint_count == 0:
            raise ValueError("CartesianStepPolicy requires at least one joint.")

        current_tcp = self.forward_kinematics.compute_tip_position(
            observation.joint_names,
            current_positions,
        )
        jacobian = self._numerical_position_jacobian(
            observation.joint_names,
            current_positions,
            current_tcp,
        )
        target_delta = tuple(self.step_m * value for value in self.direction_xyz)

        # Damped least-squares: dq = J^T (J J^T + lambda^2 I)^-1 dx.
        normal = _add_damping(_matmul_j_jt(jacobian), self.damping)
        workspace_weights = _solve_3x3(normal, target_delta)
        joint_delta = []
        for joint_index in range(joint_count):
            delta = sum(jacobian[row][joint_index] * workspace_weights[row] for row in range(3))
            delta = min(max(delta, -self.max_joint_delta_rad), self.max_joint_delta_rad)
            joint_delta.append(delta)

        return tuple(position + delta for position, delta in zip(current_positions, joint_delta))

    def reset(self) -> None:
        return None

    def _numerical_position_jacobian(
        self,
        joint_names: Sequence[str],
        current_positions: tuple[float, ...],
        current_tcp: tuple[float, float, float],
    ) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
        columns = []
        for joint_index in range(len(current_positions)):
            perturbed = list(current_positions)
            perturbed[joint_index] += self.finite_difference_epsilon_rad
            perturbed_tcp = self.forward_kinematics.compute_tip_position(joint_names, perturbed)
            columns.append(
                tuple(
                    (float(perturbed_tcp[row]) - float(current_tcp[row]))
                    / self.finite_difference_epsilon_rad
                    for row in range(3)
                )
            )
        return tuple(tuple(column[row] for column in columns) for row in range(3))  # type: ignore[return-value]


class CartesianRampPolicy:
    """Local deterministic Cartesian ramp policy.

    Unlike ``CartesianStepPolicy``, this policy keeps an accumulated TCP target
    in Cartesian space so long-duration demos continue making forward progress
    instead of repeatedly issuing only a tiny local correction around the latest
    observation.
    """

    def __init__(
        self,
        *,
        forward_kinematics,
        direction_xyz: Sequence[float] = (0.0, 1.0, 0.0),
        step_m: float = 0.001,
        damping: float = 0.05,
        finite_difference_epsilon_rad: float = 1e-4,
        max_joint_delta_rad: float = 0.005,
    ) -> None:
        if forward_kinematics is None:
            raise ValueError("CartesianRampPolicy requires a forward_kinematics helper.")
        self.forward_kinematics = forward_kinematics
        self.direction_xyz = _normalize_vector(direction_xyz)
        self.step_m = _positive_finite(step_m, "step_m")
        self.damping = _positive_finite(damping, "damping")
        self.finite_difference_epsilon_rad = _positive_finite(
            finite_difference_epsilon_rad,
            "finite_difference_epsilon_rad",
        )
        self.max_joint_delta_rad = _positive_finite(max_joint_delta_rad, "max_joint_delta_rad")
        self._target_tcp: tuple[float, float, float] | None = None

    def compute_command(self, observation: PolicyObservation) -> tuple[float, ...]:
        current_positions = tuple(float(value) for value in observation.positions)
        joint_count = len(current_positions)
        if joint_count == 0:
            raise ValueError("CartesianRampPolicy requires at least one joint.")

        current_tcp = self.forward_kinematics.compute_tip_position(
            observation.joint_names,
            current_positions,
        )
        if self._target_tcp is None:
            self._target_tcp = current_tcp
        self._target_tcp = tuple(
            float(self._target_tcp[index]) + self.step_m * self.direction_xyz[index]
            for index in range(3)
        )

        jacobian = self._numerical_position_jacobian(
            observation.joint_names,
            current_positions,
            current_tcp,
        )
        target_delta = tuple(
            float(self._target_tcp[index]) - float(current_tcp[index])
            for index in range(3)
        )

        normal = _add_damping(_matmul_j_jt(jacobian), self.damping)
        workspace_weights = _solve_3x3(normal, target_delta)
        joint_delta = []
        for joint_index in range(joint_count):
            delta = sum(jacobian[row][joint_index] * workspace_weights[row] for row in range(3))
            delta = min(max(delta, -self.max_joint_delta_rad), self.max_joint_delta_rad)
            joint_delta.append(delta)

        return tuple(position + delta for position, delta in zip(current_positions, joint_delta))

    def reset(self) -> None:
        self._target_tcp = None

    def _numerical_position_jacobian(
        self,
        joint_names: Sequence[str],
        current_positions: tuple[float, ...],
        current_tcp: tuple[float, float, float],
    ) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
        columns = []
        for joint_index in range(len(current_positions)):
            perturbed = list(current_positions)
            perturbed[joint_index] += self.finite_difference_epsilon_rad
            perturbed_tcp = self.forward_kinematics.compute_tip_position(joint_names, perturbed)
            columns.append(
                tuple(
                    (float(perturbed_tcp[row]) - float(current_tcp[row]))
                    / self.finite_difference_epsilon_rad
                    for row in range(3)
                )
            )
        return tuple(tuple(column[row] for column in columns) for row in range(3))  # type: ignore[return-value]


class CartesianTimedTrajectoryPolicy:
    """Time-parameterized Cartesian trajectory policy for stable long demos.

    The target TCP pose is computed from the pose at reset/first use plus a
    constant-velocity displacement along ``direction_xyz`` based on elapsed
    observation time. This avoids the short-horizon local-correction behavior of
    the step policy and provides a stable absolute reference for long
    demonstrations such as "100 s at 1 mm/s".
    """

    def __init__(
        self,
        *,
        forward_kinematics,
        direction_xyz: Sequence[float] = (0.0, 1.0, 0.0),
        speed_mps: float = 0.001,
        damping: float = 0.05,
        finite_difference_epsilon_rad: float = 1e-4,
        max_joint_delta_rad: float = 0.005,
    ) -> None:
        if forward_kinematics is None:
            raise ValueError("CartesianTimedTrajectoryPolicy requires a forward_kinematics helper.")
        self.forward_kinematics = forward_kinematics
        self.direction_xyz = _normalize_vector(direction_xyz)
        self.speed_mps = _positive_finite(speed_mps, "speed_mps")
        self.damping = _positive_finite(damping, "damping")
        self.finite_difference_epsilon_rad = _positive_finite(
            finite_difference_epsilon_rad,
            "finite_difference_epsilon_rad",
        )
        self.max_joint_delta_rad = _positive_finite(max_joint_delta_rad, "max_joint_delta_rad")
        self._start_tcp: tuple[float, float, float] | None = None
        self._start_stamp_sec: float | None = None

    def compute_command(self, observation: PolicyObservation) -> tuple[float, ...]:
        current_positions = tuple(float(value) for value in observation.positions)
        joint_count = len(current_positions)
        if joint_count == 0:
            raise ValueError("CartesianTimedTrajectoryPolicy requires at least one joint.")

        current_tcp = self.forward_kinematics.compute_tip_position(
            observation.joint_names,
            current_positions,
        )
        if self._start_tcp is None or self._start_stamp_sec is None:
            self._start_tcp = current_tcp
            self._start_stamp_sec = float(observation.stamp_sec)

        elapsed_sec = max(0.0, float(observation.stamp_sec) - float(self._start_stamp_sec))
        target_tcp = tuple(
            float(self._start_tcp[index]) + self.speed_mps * elapsed_sec * self.direction_xyz[index]
            for index in range(3)
        )
        target_delta = tuple(
            float(target_tcp[index]) - float(current_tcp[index])
            for index in range(3)
        )

        jacobian = _numerical_position_jacobian(
            self.forward_kinematics,
            observation.joint_names,
            current_positions,
            current_tcp,
            self.finite_difference_epsilon_rad,
        )
        normal = _add_damping(_matmul_j_jt(jacobian), self.damping)
        workspace_weights = _solve_3x3(normal, target_delta)
        joint_delta = []
        for joint_index in range(joint_count):
            delta = sum(jacobian[row][joint_index] * workspace_weights[row] for row in range(3))
            delta = min(max(delta, -self.max_joint_delta_rad), self.max_joint_delta_rad)
            joint_delta.append(delta)

        return tuple(position + delta for position, delta in zip(current_positions, joint_delta))

    def reset(self) -> None:
        self._start_tcp = None
        self._start_stamp_sec = None


class CartesianTrueFeedbackServoPolicy:
    """Closed-loop Cartesian demo policy driven by ABB true-state feedback.

    The reference trajectory is defined in Cartesian space from the true TCP
    pose captured at reset/first use. Each control tick compares that absolute
    reference against a true-state feedback observation (for example
    ``/abb_rws/joint_states``) and converts the resulting Cartesian error into a
    local joint correction around the current control observation. Long-horizon
    accumulation is handled by the bridge safety layer when it is configured to
    step from the previously approved command.
    """

    def __init__(
        self,
        *,
        forward_kinematics,
        feedback_observation_provider: Callable[[], PolicyObservation | None],
        direction_xyz: Sequence[float] = (0.0, 1.0, 0.0),
        speed_mps: float = 0.001,
        tracking_gain: float = 1.0,
        max_cartesian_error_m: float = 0.02,
        damping: float = 0.05,
        finite_difference_epsilon_rad: float = 1e-4,
        max_joint_delta_rad: float = 0.005,
    ) -> None:
        if forward_kinematics is None:
            raise ValueError(
                "CartesianTrueFeedbackServoPolicy requires a forward_kinematics helper."
            )
        self.forward_kinematics = forward_kinematics
        self.feedback_observation_provider = feedback_observation_provider
        self.direction_xyz = _normalize_vector(direction_xyz)
        self.speed_mps = _positive_finite(speed_mps, "speed_mps")
        self.tracking_gain = _positive_finite(tracking_gain, "tracking_gain")
        self.max_cartesian_error_m = _positive_finite(
            max_cartesian_error_m,
            "max_cartesian_error_m",
        )
        self.damping = _positive_finite(damping, "damping")
        self.finite_difference_epsilon_rad = _positive_finite(
            finite_difference_epsilon_rad,
            "finite_difference_epsilon_rad",
        )
        self.max_joint_delta_rad = _positive_finite(max_joint_delta_rad, "max_joint_delta_rad")
        self._start_tcp: tuple[float, float, float] | None = None
        self._start_stamp_sec: float | None = None
        self._last_debug_state: dict[str, object] = {}

    def compute_command(self, observation: PolicyObservation) -> tuple[float, ...]:
        current_positions = tuple(float(value) for value in observation.positions)
        joint_count = len(current_positions)
        if joint_count == 0:
            raise ValueError("CartesianTrueFeedbackServoPolicy requires at least one joint.")

        feedback_observation = self.feedback_observation_provider()
        if feedback_observation is None:
            raise ValueError("true_state_feedback_unavailable")

        current_tcp = self.forward_kinematics.compute_tip_position(
            observation.joint_names,
            current_positions,
        )
        feedback_tcp = self.forward_kinematics.compute_tip_position(
            feedback_observation.joint_names,
            feedback_observation.positions,
        )
        if self._start_tcp is None or self._start_stamp_sec is None:
            self._start_tcp = feedback_tcp
            self._start_stamp_sec = float(feedback_observation.stamp_sec)

        elapsed_sec = max(
            0.0,
            float(feedback_observation.stamp_sec) - float(self._start_stamp_sec),
        )
        target_tcp = tuple(
            float(self._start_tcp[index]) + self.speed_mps * elapsed_sec * self.direction_xyz[index]
            for index in range(3)
        )
        raw_error = tuple(
            float(target_tcp[index]) - float(feedback_tcp[index])
            for index in range(3)
        )
        limited_error = _limit_vector_norm(raw_error, self.max_cartesian_error_m)
        target_delta = tuple(self.tracking_gain * value for value in limited_error)

        jacobian = _numerical_position_jacobian(
            self.forward_kinematics,
            observation.joint_names,
            current_positions,
            current_tcp,
            self.finite_difference_epsilon_rad,
        )
        normal = _add_damping(_matmul_j_jt(jacobian), self.damping)
        workspace_weights = _solve_3x3(normal, target_delta)
        joint_delta = []
        for joint_index in range(joint_count):
            delta = sum(jacobian[row][joint_index] * workspace_weights[row] for row in range(3))
            delta = min(max(delta, -self.max_joint_delta_rad), self.max_joint_delta_rad)
            joint_delta.append(delta)

        requested_positions = tuple(
            position + delta for position, delta in zip(current_positions, joint_delta)
        )
        self._last_debug_state = {
            "elapsed_sec": elapsed_sec,
            "feedback_stamp_sec": float(feedback_observation.stamp_sec),
            "current_tcp_xyz": tuple(float(value) for value in current_tcp),
            "feedback_tcp_xyz": tuple(float(value) for value in feedback_tcp),
            "target_tcp_xyz": tuple(float(value) for value in target_tcp),
            "raw_error_xyz": tuple(float(value) for value in raw_error),
            "limited_error_xyz": tuple(float(value) for value in limited_error),
            "tracking_gain": self.tracking_gain,
            "max_cartesian_error_m": self.max_cartesian_error_m,
            "requested_positions": requested_positions,
        }
        return requested_positions

    def reset(self) -> None:
        self._start_tcp = None
        self._start_stamp_sec = None
        self._last_debug_state = {}

    def debug_state(self) -> dict[str, object]:
        return dict(self._last_debug_state)


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
    *,
    cartesian_fk=None,
    feedback_observation_provider: Callable[[], PolicyObservation | None] | None = None,
    cartesian_test_direction_xyz: Sequence[float] = (0.0, 1.0, 0.0),
    cartesian_test_step_m: float = 0.001,
    cartesian_test_speed_mps: float = 0.001,
    cartesian_test_tracking_gain: float = 1.0,
    cartesian_test_max_cartesian_error_m: float = 0.02,
    cartesian_test_damping: float = 0.05,
    cartesian_test_fd_epsilon_rad: float = 1e-4,
    cartesian_test_max_joint_delta_rad: float = 0.005,
):
    if policy_backend == "mock_hold":
        return MockHoldPolicy()
    if policy_backend == "cartesian_left_test":
        return CartesianStepPolicy(
            forward_kinematics=cartesian_fk,
            direction_xyz=cartesian_test_direction_xyz,
            step_m=cartesian_test_step_m,
            damping=cartesian_test_damping,
            finite_difference_epsilon_rad=cartesian_test_fd_epsilon_rad,
            max_joint_delta_rad=cartesian_test_max_joint_delta_rad,
        )
    if policy_backend == "cartesian_left_demo_traj":
        return CartesianTimedTrajectoryPolicy(
            forward_kinematics=cartesian_fk,
            direction_xyz=cartesian_test_direction_xyz,
            speed_mps=cartesian_test_speed_mps,
            damping=cartesian_test_damping,
            finite_difference_epsilon_rad=cartesian_test_fd_epsilon_rad,
            max_joint_delta_rad=cartesian_test_max_joint_delta_rad,
        )
    if policy_backend == "cartesian_left_true_servo":
        if feedback_observation_provider is None:
            raise ValueError(
                "feedback_observation_provider must be set for cartesian_left_true_servo."
            )
        return CartesianTrueFeedbackServoPolicy(
            forward_kinematics=cartesian_fk,
            feedback_observation_provider=feedback_observation_provider,
            direction_xyz=cartesian_test_direction_xyz,
            speed_mps=cartesian_test_speed_mps,
            tracking_gain=cartesian_test_tracking_gain,
            max_cartesian_error_m=cartesian_test_max_cartesian_error_m,
            damping=cartesian_test_damping,
            finite_difference_epsilon_rad=cartesian_test_fd_epsilon_rad,
            max_joint_delta_rad=cartesian_test_max_joint_delta_rad,
        )
    if policy_backend == "cartesian_left_ramp_test":
        return CartesianRampPolicy(
            forward_kinematics=cartesian_fk,
            direction_xyz=cartesian_test_direction_xyz,
            step_m=cartesian_test_step_m,
            damping=cartesian_test_damping,
            finite_difference_epsilon_rad=cartesian_test_fd_epsilon_rad,
            max_joint_delta_rad=cartesian_test_max_joint_delta_rad,
        )
    if policy_backend == "http_json":
        return HttpPolicyServerClient(
            server_url=policy_server_url,
            timeout_sec=policy_request_timeout_sec,
        )
    raise ValueError(
        f"Unsupported policy_backend '{policy_backend}'. "
        "Supported backends: 'mock_hold', 'http_json', 'cartesian_left_test', "
        "'cartesian_left_ramp_test', 'cartesian_left_demo_traj', "
        "'cartesian_left_true_servo'."
    )


def _numerical_position_jacobian(
    forward_kinematics,
    joint_names: Sequence[str],
    current_positions: tuple[float, ...],
    current_tcp: tuple[float, float, float],
    finite_difference_epsilon_rad: float,
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    columns = []
    for joint_index in range(len(current_positions)):
        perturbed = list(current_positions)
        perturbed[joint_index] += finite_difference_epsilon_rad
        perturbed_tcp = forward_kinematics.compute_tip_position(joint_names, perturbed)
        columns.append(
            tuple(
                (float(perturbed_tcp[row]) - float(current_tcp[row])) / finite_difference_epsilon_rad
                for row in range(3)
            )
        )
    return tuple(tuple(column[row] for column in columns) for row in range(3))  # type: ignore[return-value]


def _positive_finite(value: float, label: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{label} must be positive and finite.")
    return value


def _normalize_vector(vector: Sequence[float]) -> tuple[float, float, float]:
    if len(vector) != 3:
        raise ValueError("direction_xyz must contain exactly three values.")
    values = tuple(float(value) for value in vector)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("direction_xyz must contain only finite values.")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 0.0:
        raise ValueError("direction_xyz must be non-zero.")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def _matmul_j_jt(
    jacobian: tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    rows = []
    for row in range(3):
        values = []
        for col in range(3):
            values.append(sum(jacobian[row][k] * jacobian[col][k] for k in range(len(jacobian[row]))))
        rows.append(tuple(values))
    return tuple(rows)  # type: ignore[return-value]


def _add_damping(
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    damping: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    damping_squared = float(damping) * float(damping)
    rows = []
    for row in range(3):
        values = []
        for col in range(3):
            values.append(matrix[row][col] + (damping_squared if row == col else 0.0))
        rows.append(tuple(values))
    return tuple(rows)  # type: ignore[return-value]


def _solve_3x3(
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    vector: Sequence[float],
) -> tuple[float, float, float]:
    a = [[float(matrix[row][col]) for col in range(3)] for row in range(3)]
    b = [float(vector[row]) for row in range(3)]

    for pivot_col in range(3):
        pivot_row = max(range(pivot_col, 3), key=lambda row: abs(a[row][pivot_col]))
        if abs(a[pivot_row][pivot_col]) < 1e-12:
            raise ValueError("CartesianStepPolicy Jacobian solve became singular.")
        if pivot_row != pivot_col:
            a[pivot_col], a[pivot_row] = a[pivot_row], a[pivot_col]
            b[pivot_col], b[pivot_row] = b[pivot_row], b[pivot_col]

        pivot = a[pivot_col][pivot_col]
        for col in range(pivot_col, 3):
            a[pivot_col][col] /= pivot
        b[pivot_col] /= pivot

        for row in range(3):
            if row == pivot_col:
                continue
            factor = a[row][pivot_col]
            for col in range(pivot_col, 3):
                a[row][col] -= factor * a[pivot_col][col]
            b[row] -= factor * b[pivot_col]

    return (b[0], b[1], b[2])


def _limit_vector_norm(vector: Sequence[float], max_norm: float) -> tuple[float, float, float]:
    limited = tuple(float(value) for value in vector)
    norm = math.sqrt(sum(value * value for value in limited))
    if norm <= max_norm or norm <= 0.0:
        return limited  # type: ignore[return-value]
    scale = max_norm / norm
    return tuple(value * scale for value in limited)  # type: ignore[return-value]
