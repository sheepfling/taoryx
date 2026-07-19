"""Canonical rotor command and quadrotor allocation contracts.

The contract is vehicle-agnostic: runtime controls provide four rotor-speed
commands in rad/s, while the allocation layer maps a collective speed and a
body moment request into bounded rotor commands.  Source-specific frame and
coefficient adapters remain outside this module.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from .contracts import Vector3


@dataclass(frozen=True, slots=True)
class RotorCommandSet:
    """Four individually commanded rotor speeds in rad/s."""

    rotor_1_rad_s: float
    rotor_2_rad_s: float
    rotor_3_rad_s: float
    rotor_4_rad_s: float

    @property
    def values(self) -> tuple[float, float, float, float]:
        return (self.rotor_1_rad_s, self.rotor_2_rad_s, self.rotor_3_rad_s, self.rotor_4_rad_s)
        ####

    @property
    def rms_speed_rad_s(self) -> float:
        return math.sqrt(sum(value * value for value in self.values) / 4.0)
        ####

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) and value >= 0.0 for value in self.values):
            raise ValueError("rotor speeds must be finite and nonnegative")
        ####
    ####


@dataclass(frozen=True, slots=True)
class QuadRotorAllocation:
    """X-configuration allocation and differential-wrench adapter."""

    arm_m: float
    thrust_coefficient_n_per_rad_s2: float
    reaction_torque_coefficient_nm_per_rad_s2: float
    directions: tuple[float, float, float, float] = (1.0, -1.0, 1.0, -1.0)
    minimum_speed_rad_s: float = 0.0
    maximum_speed_rad_s: float = 1500.0

    def __post_init__(self) -> None:
        if self.arm_m <= 0.0 or self.thrust_coefficient_n_per_rad_s2 <= 0.0 or self.reaction_torque_coefficient_nm_per_rad_s2 <= 0.0:
            raise ValueError("rotor geometry and coefficients must be positive")
        if len(self.directions) != 4 or any(direction not in {-1.0, 1.0} for direction in self.directions):
            raise ValueError("quadrotor directions must contain four +/-1 values")
        if not 0.0 <= self.minimum_speed_rad_s < self.maximum_speed_rad_s:
            raise ValueError("rotor speed bounds are invalid")
        ####
    ####

    @property
    def rotor_positions_m(self) -> tuple[tuple[float, float], ...]:
        diagonal = self.arm_m / math.sqrt(2.0)
        return ((diagonal, diagonal), (diagonal, -diagonal), (-diagonal, -diagonal), (-diagonal, diagonal))
        ####

    def allocate(self, collective_speed_rad_s: float, moment_body_source: Vector3) -> RotorCommandSet:
        """Allocate source-frame body moment demand around a collective speed."""

        if not math.isfinite(collective_speed_rad_s) or collective_speed_rad_s < 0.0:
            raise ValueError("collective speed must be finite and nonnegative")
        base_squared = collective_speed_rad_s * collective_speed_rad_s
        matrix = [
            [1.0, 1.0, 1.0, 1.0],
            [y * self.thrust_coefficient_n_per_rad_s2 for _, y in self.rotor_positions_m],
            [-x * self.thrust_coefficient_n_per_rad_s2 for x, _ in self.rotor_positions_m],
            [direction * self.reaction_torque_coefficient_nm_per_rad_s2 for direction in self.directions],
        ]
        rhs = [4.0 * base_squared, moment_body_source.x, moment_body_source.y, moment_body_source.z]
        squared = _solve_four_by_four(matrix, rhs)
        speeds = tuple(
            min(self.maximum_speed_rad_s, max(self.minimum_speed_rad_s, math.sqrt(max(0.0, value))))
            for value in squared
        )
        return RotorCommandSet(*speeds)
        ####

    def differential_moment_source(self, commands: RotorCommandSet) -> Vector3:
        """Return the source-frame moment relative to the command RMS thrust."""

        baseline = commands.rms_speed_rad_s**2
        thrust_delta = tuple(self.thrust_coefficient_n_per_rad_s2 * (speed * speed - baseline) for speed in commands.values)
        roll = sum(y * delta for (_, y), delta in zip(self.rotor_positions_m, thrust_delta, strict=True))
        pitch = sum(-x * delta for (x, _), delta in zip(self.rotor_positions_m, thrust_delta, strict=True))
        yaw = sum(direction * self.reaction_torque_coefficient_nm_per_rad_s2 * (speed * speed - baseline) for direction, speed in zip(self.directions, commands.values, strict=True))
        return Vector3(roll, pitch, yaw)
        ####
    ####


def _solve_four_by_four(matrix: Sequence[Sequence[float]], rhs: Sequence[float]) -> tuple[float, float, float, float]:
    """Solve a small dense system without coupling the runtime to NumPy."""

    augmented = [list(row) + [float(value)] for row, value in zip(matrix, rhs, strict=True)]
    for pivot in range(4):
        pivot_row = max(range(pivot, 4), key=lambda row: abs(augmented[row][pivot]))
        if abs(augmented[pivot_row][pivot]) <= 1.0e-15:
            raise ValueError("rotor allocation matrix is singular")
        augmented[pivot], augmented[pivot_row] = augmented[pivot_row], augmented[pivot]
        scale = augmented[pivot][pivot]
        augmented[pivot] = [value / scale for value in augmented[pivot]]
        for row in range(4):
            if row == pivot:
                continue
            factor = augmented[row][pivot]
            augmented[row] = [left - factor * right for left, right in zip(augmented[row], augmented[pivot], strict=True)]
    return tuple(float(augmented[row][4]) for row in range(4))  # type: ignore[return-value]
    ####


__all__ = ["QuadRotorAllocation", "RotorCommandSet"]
