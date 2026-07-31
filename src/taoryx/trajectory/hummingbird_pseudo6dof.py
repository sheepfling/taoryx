"""Aggregate-thrust pseudo-6DOF realization for the Hummingbird family.

This module is intentionally smaller than the native rotor plant.  It carries
translation, attitude, bounded body-rate response, thrust, and battery
telemetry so mission planning can use a named pseudo tier.  It does not solve
individual motor allocation, rotor inflow, or aerodynamic moments; those
claims remain reserved for the native rigid-body Hummingbird runs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from .pseudo6dof_profiles import Pseudo6DOFProfile, load_pseudo6dof_catalog
from .response_laws import AxisResponseState, step_bounded_axis_response

Vector3 = tuple[float, float, float]


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
    ####


def _add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a + b for a, b in zip(left, right, strict=True))  # type: ignore[return-value]
    ####


def _scale(value: Vector3, factor: float) -> Vector3:
    return tuple(component * factor for component in value)  # type: ignore[return-value]
    ####


def _norm(value: Vector3) -> float:
    return math.sqrt(sum(component * component for component in value))
    ####


@dataclass(frozen=True, slots=True)
class HummingbirdPseudo6DOFCommand:
    """Semantic command accepted by the aggregate thrust-vector model."""

    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    thrust_ratio: float = 1.0
    motors_enabled: bool = True
    thrust_frame: Literal["world_euler", "body_euler"] = "world_euler"
    ####


@dataclass(frozen=True, slots=True)
class HummingbirdPseudo6DOFState:
    """Mission-level translation and response-law attitude state."""

    time_s: float
    position_m: Vector3
    velocity_m_s: Vector3
    attitude_rad: Vector3
    attitude_rate_rad_s: Vector3
    battery_fraction: float
    thrust_n: float
    contact: bool = False
    ####


@dataclass(frozen=True, slots=True)
class HummingbirdPseudo6DOFModel:
    """Bounded aggregate-thrust Hummingbird pseudo-6DOF model."""

    mass_kg: float = 0.5
    gravity_m_s2: float = 9.80665
    # Source-derived from the imported Hummingbird rotor fixture's
    # ``maximum_total_collective_thrust`` entry.  This is a capacity bound,
    # not a claim that the pseudo tier resolves four rotor effectors.
    maximum_thrust_n: float = 50.13
    battery_energy_j: float = 2_000.0
    profile: Pseudo6DOFProfile | None = None
    profile_id: str = "hummingbird.attitude_response_p6dof.v1"

    def __post_init__(self) -> None:
        if self.mass_kg <= 0.0 or self.gravity_m_s2 <= 0.0 or self.maximum_thrust_n <= 0.0 or self.battery_energy_j <= 0.0:
            raise ValueError("Hummingbird pseudo model parameters must be positive")
        selected = self.profile if self.profile is not None else self._catalog_profile()
        if selected.id != self.profile_id:
            raise ValueError(f"profile ID mismatch: {self.profile_id} != {selected.id}")
        if selected.model_kind != "thrust_vector_surrogate":
            raise ValueError("Hummingbird pseudo model requires a thrust-vector surrogate profile")
        if selected.family_id != "hummingbird":
            raise ValueError("Hummingbird pseudo model requires the hummingbird profile family")
        object.__setattr__(self, "profile", selected)
        ####

    def _catalog_profile(self) -> Pseudo6DOFProfile:
        """Resolve the canonical Hummingbird profile."""

        _, profile = load_pseudo6dof_catalog().for_family("hummingbird")
        return profile
        ####

    def initial_state(self, *, altitude_m: float = 0.0) -> HummingbirdPseudo6DOFState:
        """Return a stationary, upright starting state."""

        if altitude_m < 0.0:
            raise ValueError("altitude_m must be nonnegative")
        return HummingbirdPseudo6DOFState(
            0.0,
            (0.0, 0.0, altitude_m),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            1.0,
            self.mass_kg * self.gravity_m_s2,
            altitude_m == 0.0,
        )
        ####

    def step(self, state: HummingbirdPseudo6DOFState, command: HummingbirdPseudo6DOFCommand, dt_s: float) -> tuple[HummingbirdPseudo6DOFState, dict[str, object]]:
        """Advance translation and the declared bounded response law."""

        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        assert self.profile is not None
        axes = (self.profile.response["roll"], self.profile.response["pitch"], self.profile.response["yaw"])
        commands = (command.roll_rad, command.pitch_rad, command.yaw_rad)
        response = tuple(
            step_bounded_axis_response(axis, AxisResponseState(angle, rate), target, dt_s)
            for axis, angle, rate, target in zip(axes, state.attitude_rad, state.attitude_rate_rad_s, commands, strict=True)
        )
        roll_response, pitch_response, yaw_response = response
        attitude: Vector3 = (roll_response.angle_rad, pitch_response.angle_rad, yaw_response.angle_rad)
        rates: Vector3 = (roll_response.rate_rad_s, pitch_response.rate_rad_s, yaw_response.rate_rad_s)
        requested_ratio = _clamp(command.thrust_ratio, 0.0, 1.0) if command.motors_enabled else 0.0
        available_ratio = _clamp(state.battery_fraction, 0.0, 1.0)
        thrust_ratio = requested_ratio * available_ratio
        thrust = thrust_ratio * self.maximum_thrust_n
        roll, pitch, yaw = attitude
        if command.thrust_frame == "body_euler":
            # Map the aggregate body-z thrust through the achieved yaw, pitch,
            # and roll.  This remains a pseudo response law, but it prevents
            # the directional mission from silently treating body tilt as a
            # world-frame force command.
            body_vector = (
                math.sin(pitch) * math.cos(roll) * thrust,
                -math.sin(roll) * math.cos(pitch) * thrust,
                math.cos(roll) * math.cos(pitch) * thrust,
            )
            thrust_vector = (
                math.cos(yaw) * body_vector[0] - math.sin(yaw) * body_vector[1],
                math.sin(yaw) * body_vector[0] + math.cos(yaw) * body_vector[1],
                body_vector[2],
            )
        else:
            thrust_vector = (
                math.sin(pitch) * thrust,
                -math.sin(roll) * math.cos(pitch) * thrust,
                math.cos(roll) * math.cos(pitch) * thrust,
            )
        acceleration = _add(_scale(thrust_vector, 1.0 / self.mass_kg), (0.0, 0.0, -self.gravity_m_s2))
        velocity = _add(state.velocity_m_s, _scale(acceleration, dt_s))
        position = _add(state.position_m, _scale(velocity, dt_s))
        contact = state.contact
        if position[2] <= 0.0 and velocity[2] <= 0.0:
            position = (position[0], position[1], 0.0)
            velocity = (velocity[0], velocity[1], 0.0)
            contact = True
        elif contact and thrust_vector[2] > self.mass_kg * self.gravity_m_s2:
            contact = False
        energy_used = thrust * dt_s
        battery_fraction = _clamp(state.battery_fraction - energy_used / self.battery_energy_j, 0.0, 1.0)
        next_state = HummingbirdPseudo6DOFState(state.time_s + dt_s, position, velocity, attitude, rates, battery_fraction, thrust, contact)
        telemetry: dict[str, object] = {
            "time_s": next_state.time_s,
            "response_profile_id": self.profile_id,
            "position_m": list(position),
            "velocity_m_s": list(velocity),
            "commanded_attitude_rad": list(commands),
            "achieved_attitude_rad": list(attitude),
            "body_rate_rad_s": list(rates),
            "requested_thrust_ratio": requested_ratio,
            "achieved_thrust_n": thrust,
            "achieved_thrust_vector_n": list(thrust_vector),
            "maximum_thrust_n": self.maximum_thrust_n,
            "motors_enabled": command.motors_enabled,
            "contact_state": contact,
            "shutdown": not command.motors_enabled,
            "battery_fraction": battery_fraction,
            "battery_energy_model": "bounded_engineering_reserve_not_source_electrical_model",
            "allocation_mode": "aggregate_thrust_vector_surrogate",
            "thrust_vector_frame": command.thrust_frame,
            "physical_motor_allocation": False,
            "unsupported_claims": list(self.profile.unsupported_claims),
        }
        return next_state, telemetry
        ####

    def run(self, commands: tuple[HummingbirdPseudo6DOFCommand, ...], *, dt_s: float = 0.01, initial_state: HummingbirdPseudo6DOFState | None = None) -> tuple[HummingbirdPseudo6DOFState, tuple[dict[str, object], ...]]:
        """Run a deterministic command sequence and retain its telemetry."""

        state = self.initial_state() if initial_state is None else initial_state
        rows: list[dict[str, object]] = []
        for command in commands:
            state, row = self.step(state, command, dt_s)
            rows.append(row)
        return state, tuple(rows)
        ####


__all__ = ["HummingbirdPseudo6DOFCommand", "HummingbirdPseudo6DOFModel", "HummingbirdPseudo6DOFState"]
