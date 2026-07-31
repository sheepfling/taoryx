"""Independent translational Hummingbird point-mass realization.

The point-mass tier intentionally has no attitude or rotor-state variables.
It accepts a requested world-frame thrust vector, applies the declared total
thrust and bounded-resource limits, and integrates only position, velocity,
contact, and battery-reserve state.  Yaw and motor-allocation claims belong to
the pseudo or native rigid-body tiers.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

Vector3 = tuple[float, float, float]


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
    ####


def _norm(value: Vector3) -> float:
    return math.sqrt(sum(component * component for component in value))
    ####


def _scale(value: Vector3, factor: float) -> Vector3:
    return tuple(component * factor for component in value)  # type: ignore[return-value]
    ####


def _add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a + b for a, b in zip(left, right, strict=True))  # type: ignore[return-value]
    ####


@dataclass(frozen=True, slots=True)
class Hummingbird3DOFCommand:
    """World-frame translational force command for the point-mass tier."""

    thrust_vector_n: Vector3 = (0.0, 0.0, 0.0)
    motors_enabled: bool = True
    ####


@dataclass(frozen=True, slots=True)
class Hummingbird3DOFState:
    """Translational and resource state exposed by the point-mass tier."""

    time_s: float
    position_m: Vector3
    velocity_m_s: Vector3
    battery_fraction: float
    thrust_n: float
    contact: bool = False
    ####


@dataclass(frozen=True, slots=True)
class Hummingbird3DOFModel:
    """Bounded force-model Hummingbird 3DOF plant."""

    mass_kg: float = 0.5
    gravity_m_s2: float = 9.80665
    maximum_thrust_n: float = 50.13
    battery_energy_j: float = 2_000.0

    def __post_init__(self) -> None:
        if self.mass_kg <= 0.0 or self.gravity_m_s2 <= 0.0 or self.maximum_thrust_n <= 0.0 or self.battery_energy_j <= 0.0:
            raise ValueError("Hummingbird 3DOF model parameters must be positive")
        ####
    ####

    def initial_state(self, *, altitude_m: float = 0.0) -> Hummingbird3DOFState:
        """Return a stationary point-mass state."""

        if altitude_m < 0.0:
            raise ValueError("altitude_m must be nonnegative")
        return Hummingbird3DOFState(
            0.0,
            (0.0, 0.0, altitude_m),
            (0.0, 0.0, 0.0),
            1.0,
            self.mass_kg * self.gravity_m_s2,
            altitude_m == 0.0,
        )
        ####

    def step(self, state: Hummingbird3DOFState, command: Hummingbird3DOFCommand, dt_s: float) -> tuple[Hummingbird3DOFState, dict[str, object]]:
        """Advance the translational plant with force and resource limits."""

        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        requested = command.thrust_vector_n if command.motors_enabled else (0.0, 0.0, 0.0)
        requested_norm = _norm(requested)
        available_norm = self.maximum_thrust_n * _clamp(state.battery_fraction, 0.0, 1.0)
        achieved_norm = min(requested_norm, available_norm)
        achieved = _scale(requested, achieved_norm / requested_norm) if requested_norm > 0.0 else (0.0, 0.0, 0.0)
        acceleration = _add(_scale(achieved, 1.0 / self.mass_kg), (0.0, 0.0, -self.gravity_m_s2))
        velocity = _add(state.velocity_m_s, _scale(acceleration, dt_s))
        position = _add(state.position_m, _scale(velocity, dt_s))
        contact = state.contact
        if position[2] <= 0.0 and velocity[2] <= 0.0:
            position = (position[0], position[1], 0.0)
            velocity = (velocity[0], velocity[1], 0.0)
            contact = True
        elif contact and achieved[2] > self.mass_kg * self.gravity_m_s2:
            contact = False
        battery_fraction = _clamp(state.battery_fraction - achieved_norm * dt_s / self.battery_energy_j, 0.0, 1.0)
        next_state = Hummingbird3DOFState(state.time_s + dt_s, position, velocity, battery_fraction, achieved_norm, contact)
        telemetry: dict[str, object] = {
            "time_s": next_state.time_s,
            "position_m": list(position),
            "velocity_m_s": list(velocity),
            "requested_thrust_vector_n": list(requested),
            "achieved_thrust_vector_n": list(achieved),
            "requested_thrust_n": requested_norm,
            "achieved_thrust_n": achieved_norm,
            "maximum_thrust_n": self.maximum_thrust_n,
            "motors_enabled": command.motors_enabled,
            "contact_state": contact,
            "shutdown": not command.motors_enabled,
            "battery_fraction": battery_fraction,
            "battery_energy_model": "bounded_engineering_reserve_not_source_electrical_model",
            "control_realization": "point_mass_force_model",
            "attitude_channels": "not_applicable_point_mass_3dof",
            "physical_motor_allocation": False,
        }
        return next_state, telemetry
        ####


__all__ = ["Hummingbird3DOFCommand", "Hummingbird3DOFModel", "Hummingbird3DOFState"]
####
