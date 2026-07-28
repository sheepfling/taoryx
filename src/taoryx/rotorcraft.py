"""Canonical rotor command and quadrotor allocation contracts.

The contract is vehicle-agnostic: runtime controls provide four rotor-speed
commands in rad/s, while the allocation layer maps a collective speed and a
body moment request into bounded rotor commands.  A vehicle may additionally
declare the complete source individual-rotor load model.  That explicit mode
evaluates every rotor's thrust, drag, force arm, and reaction torque rather
than treating a requested moment as an injected wrench.
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
    rotor_drag_xy_coefficient_n_s_per_m: float | None = None
    rotor_drag_z_coefficient_n_s_per_m: float | None = None
    translational_lift_coefficient_n_s2_per_m2: float | None = None
    frame_drag_coefficients_n_s2_per_m2: Vector3 | None = None

    def __post_init__(self) -> None:
        if self.arm_m <= 0.0 or self.thrust_coefficient_n_per_rad_s2 <= 0.0 or self.reaction_torque_coefficient_nm_per_rad_s2 <= 0.0:
            raise ValueError("rotor geometry and coefficients must be positive")
        if len(self.directions) != 4 or any(direction not in {-1.0, 1.0} for direction in self.directions):
            raise ValueError("quadrotor directions must contain four +/-1 values")
        if not 0.0 <= self.minimum_speed_rad_s < self.maximum_speed_rad_s:
            raise ValueError("rotor speed bounds are invalid")
        full_model_values = (
            self.rotor_drag_xy_coefficient_n_s_per_m,
            self.rotor_drag_z_coefficient_n_s_per_m,
            self.translational_lift_coefficient_n_s2_per_m2,
            self.frame_drag_coefficients_n_s2_per_m2,
        )
        if any(value is not None for value in full_model_values) and any(value is None for value in full_model_values):
            raise ValueError("individual-rotor source model requires every declared drag and lift parameter")
        if self.individual_rotor_source_available:
            assert self.rotor_drag_xy_coefficient_n_s_per_m is not None
            assert self.rotor_drag_z_coefficient_n_s_per_m is not None
            assert self.translational_lift_coefficient_n_s2_per_m2 is not None
            scalar_values = (
                self.rotor_drag_xy_coefficient_n_s_per_m,
                self.rotor_drag_z_coefficient_n_s_per_m,
                self.translational_lift_coefficient_n_s2_per_m2,
            )
            if any(not math.isfinite(float(value)) or float(value) < 0.0 for value in scalar_values):
                raise ValueError("individual-rotor source coefficients must be finite and nonnegative")
            assert self.frame_drag_coefficients_n_s2_per_m2 is not None
            frame_drag = self.frame_drag_coefficients_n_s2_per_m2
            if any(not math.isfinite(value) or value < 0.0 for value in (frame_drag.x, frame_drag.y, frame_drag.z)):
                raise ValueError("individual-rotor frame-drag coefficients must be finite and nonnegative")
        ####
    ####

    @property
    def rotor_positions_m(self) -> tuple[tuple[float, float], ...]:
        diagonal = self.arm_m / math.sqrt(2.0)
        return ((diagonal, diagonal), (diagonal, -diagonal), (-diagonal, -diagonal), (-diagonal, diagonal))
        ####

    @property
    def individual_rotor_source_available(self) -> bool:
        """Whether this allocation also owns a complete source rotor-load model.

        A false value is intentional: legacy common-speed tables can use the
        allocation only for differential-moment reconstruction.  A true value
        is stronger evidence because the runtime evaluates individual rotor
        thrust, drag, force arms, and reaction torque from the declared source
        equations.
        """

        return all(
            value is not None
            for value in (
                self.rotor_drag_xy_coefficient_n_s_per_m,
                self.rotor_drag_z_coefficient_n_s_per_m,
                self.translational_lift_coefficient_n_s2_per_m2,
                self.frame_drag_coefficients_n_s2_per_m2,
            )
        )
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

    def source_force_moment(
        self,
        commands: RotorCommandSet,
        body_velocity_source_m_s: Vector3,
        body_rate_source_rad_s: Vector3,
    ) -> tuple[Vector3, Vector3]:
        """Evaluate the declared individual-rotor source equations.

        This is the RotorPy-style source equation used by the Hummingbird
        bundle.  It is deliberately opt-in through
        :attr:`individual_rotor_source_available`; a generic quadrotor cannot
        silently acquire unproven rotor drag or translational-lift terms.

        The inputs and outputs remain in the source body axes.  The runtime's
        frame adapter is responsible for converting this source Z-up model to
        the canonical rigid-body frame.
        """

        if not self.individual_rotor_source_available:
            raise ValueError("individual-rotor source loads were not declared for this allocation")
        assert self.rotor_drag_xy_coefficient_n_s_per_m is not None
        assert self.rotor_drag_z_coefficient_n_s_per_m is not None
        assert self.translational_lift_coefficient_n_s2_per_m2 is not None
        assert self.frame_drag_coefficients_n_s2_per_m2 is not None
        velocity = body_velocity_source_m_s
        rate = body_rate_source_rad_s
        total_force = Vector3(
            -velocity.x * abs(velocity.x) * self.frame_drag_coefficients_n_s2_per_m2.x,
            -velocity.y * abs(velocity.y) * self.frame_drag_coefficients_n_s2_per_m2.y,
            -velocity.z * abs(velocity.z) * self.frame_drag_coefficients_n_s2_per_m2.z,
        )
        total_moment = Vector3(0.0, 0.0, 0.0)
        for (x_position, y_position), direction, speed in zip(
            self.rotor_positions_m,
            self.directions,
            commands.values,
            strict=True,
        ):
            position = Vector3(x_position, y_position, 0.0)
            local_velocity = velocity + rate.cross(position)
            thrust_z = (
                self.thrust_coefficient_n_per_rad_s2 * speed * speed
                + self.translational_lift_coefficient_n_s2_per_m2
                * (local_velocity.x * local_velocity.x + local_velocity.y * local_velocity.y)
            )
            rotor_drag = Vector3(
                -speed * self.rotor_drag_xy_coefficient_n_s_per_m * local_velocity.x,
                -speed * self.rotor_drag_xy_coefficient_n_s_per_m * local_velocity.y,
                -speed * self.rotor_drag_z_coefficient_n_s_per_m * local_velocity.z,
            )
            rotor_force = rotor_drag + Vector3(0.0, 0.0, thrust_z)
            total_force = total_force + rotor_force
            total_moment = total_moment + position.cross(rotor_force)
            total_moment = Vector3(
                total_moment.x,
                total_moment.y,
                total_moment.z + direction * self.reaction_torque_coefficient_nm_per_rad_s2 * speed * speed,
            )
        return total_force, total_moment
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
