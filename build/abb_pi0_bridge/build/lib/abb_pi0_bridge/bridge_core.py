from dataclasses import dataclass
import math
from typing import Mapping, Sequence, Tuple


@dataclass(frozen=True)
class PolicyObservation:
    stamp_sec: float
    joint_names: Tuple[str, ...]
    positions: Tuple[float, ...]
    velocities: Tuple[float, ...]
    efforts: Tuple[float, ...]

    def as_policy_input(self) -> Mapping[str, object]:
        return {
            "stamp_sec": self.stamp_sec,
            "joint_names": list(self.joint_names),
            "joint_positions": list(self.positions),
            "joint_velocities": list(self.velocities),
            "joint_efforts": list(self.efforts),
        }


@dataclass(frozen=True)
class SafetyConfig:
    lower_limits: Tuple[float, ...]
    upper_limits: Tuple[float, ...]
    max_position_step_rad: float


@dataclass(frozen=True)
class SafeCommand:
    positions: Tuple[float, ...]
    was_clamped: bool
    reason: str


def build_observation(
    ordered_joint_names: Sequence[str],
    msg_joint_names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float] | None,
    efforts: Sequence[float] | None,
    stamp_sec: float,
) -> PolicyObservation:
    if len(msg_joint_names) != len(positions):
        raise ValueError("JointState name and position vectors must have the same length.")

    index_by_name = {}
    for index, name in enumerate(msg_joint_names):
        if name in index_by_name:
            raise ValueError(f"Duplicate joint name in JointState: {name}")
        index_by_name[name] = index

    missing = [name for name in ordered_joint_names if name not in index_by_name]
    if missing:
        raise ValueError(f"Missing joints in JointState: {missing}")

    ordered_positions = []
    ordered_velocities = []
    ordered_efforts = []
    velocities = velocities or []
    efforts = efforts or []

    for name in ordered_joint_names:
        index = index_by_name[name]
        ordered_positions.append(float(positions[index]))
        ordered_velocities.append(float(velocities[index]) if index < len(velocities) else 0.0)
        ordered_efforts.append(float(efforts[index]) if index < len(efforts) else 0.0)

    return PolicyObservation(
        stamp_sec=float(stamp_sec),
        joint_names=tuple(ordered_joint_names),
        positions=tuple(ordered_positions),
        velocities=tuple(ordered_velocities),
        efforts=tuple(ordered_efforts),
    )


def observation_is_fresh(now_sec: float, observation_stamp_sec: float, timeout_sec: float) -> bool:
    if timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be positive.")
    return (now_sec - observation_stamp_sec) <= timeout_sec


def apply_safety_filters(
    current_positions: Sequence[float],
    requested_positions: Sequence[float],
    safety_config: SafetyConfig,
) -> SafeCommand:
    _validate_vector_lengths(current_positions, requested_positions, safety_config)
    if safety_config.max_position_step_rad <= 0.0:
        raise ValueError("max_position_step_rad must be positive.")

    safe_positions = []
    was_clamped = False
    clamp_reasons = []

    for index, (current, requested, lower, upper) in enumerate(
        zip(
            current_positions,
            requested_positions,
            safety_config.lower_limits,
            safety_config.upper_limits,
        )
    ):
        _validate_finite(current, f"current_positions[{index}]")
        _validate_finite(requested, f"requested_positions[{index}]")
        _validate_finite(lower, f"lower_limits[{index}]")
        _validate_finite(upper, f"upper_limits[{index}]")

        if lower > upper:
            raise ValueError(f"Invalid joint limits at index {index}: {lower} > {upper}")

        bounded = min(max(float(requested), float(lower)), float(upper))
        if not math.isclose(bounded, float(requested), rel_tol=0.0, abs_tol=1e-12):
            was_clamped = True
            clamp_reasons.append(f"joint_{index}_limit")

        lower_step = float(current) - safety_config.max_position_step_rad
        upper_step = float(current) + safety_config.max_position_step_rad
        stepped = min(max(bounded, lower_step), upper_step)
        if not math.isclose(stepped, bounded, rel_tol=0.0, abs_tol=1e-12):
            was_clamped = True
            clamp_reasons.append(f"joint_{index}_step")

        safe_positions.append(stepped)

    reason = "ok"
    if clamp_reasons:
        reason = ",".join(clamp_reasons)

    return SafeCommand(positions=tuple(safe_positions), was_clamped=was_clamped, reason=reason)


def _validate_vector_lengths(
    current_positions: Sequence[float],
    requested_positions: Sequence[float],
    safety_config: SafetyConfig,
) -> None:
    vector_length = len(current_positions)
    if len(requested_positions) != vector_length:
        raise ValueError("current_positions and requested_positions must have the same length.")
    if len(safety_config.lower_limits) != vector_length:
        raise ValueError("lower_limits length does not match command vector length.")
    if len(safety_config.upper_limits) != vector_length:
        raise ValueError("upper_limits length does not match command vector length.")


def _validate_finite(value: float, label: str) -> None:
    if not math.isfinite(float(value)):
        raise ValueError(f"{label} must be finite.")
