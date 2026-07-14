"""Canonical state contracts for the TAOS 3-DOF point-mass model.

The manual's primary integration state is ECFC position and earth-relative
velocity.  TAOS augments those six translational components with mass, path
length, and ground range.  Attitude is deliberately absent: it is supplied by
the guidance/force evaluation layer and is not a rotational state.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import ClassVar

from .contracts import Frame, FrameVector3, Vector3
from .simulation.contracts import SimulationState


@dataclass(frozen=True, slots=True)
class PointMassState:
    """Time-tagged, augmented TAOS state in the ECFC frame.

    The packed order is the order printed by the manual's augmented state
    vector (equations 2-107 and 2-113).  It is intentionally explicit so an
    integrator, runtime adapter, and output writer cannot silently disagree.
    """

    STATE_NAMES: ClassVar[tuple[str, ...]] = (
        "x_ecfc",
        "y_ecfc",
        "z_ecfc",
        "xdot_ecfc",
        "ydot_ecfc",
        "zdot_ecfc",
        "mass",
        "path_length",
        "ground_range",
    )

    time: float
    position: FrameVector3
    earth_relative_velocity: FrameVector3
    mass: float
    path_length: float = 0.0
    ground_range: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.time):
            raise ValueError("state time must be finite")
        for name, vector in (("position", self.position), ("earth-relative velocity", self.earth_relative_velocity)):
            if vector.frame is not Frame.ECFC:
                raise ValueError(f"{name} must be expressed in the ECFC frame")
            ####
            if not all(math.isfinite(value) for value in (vector.vector.x, vector.vector.y, vector.vector.z)):
                raise ValueError(f"{name} components must be finite")
            ####
        if not math.isfinite(self.mass) or self.mass <= 0.0:
            raise ValueError("mass must be positive and finite")
        if not all(math.isfinite(value) for value in (self.path_length, self.ground_range)):
            raise ValueError("trajectory integrals must be finite")
        ####

    def to_values(self) -> tuple[float, ...]:
        """Pack the state in the canonical TAOS integration order."""

        position = self.position.vector
        velocity = self.earth_relative_velocity.vector
        return (position.x, position.y, position.z, velocity.x, velocity.y, velocity.z, self.mass, self.path_length, self.ground_range)
        ####

    def to_simulation_state(self) -> SimulationState:
        """Adapt this physical state to the generic integrator contract."""

        return SimulationState(self.time, self.to_values(), Frame.ECFC.value)
        ####

    @classmethod
    def from_values(cls, time: float, values: tuple[float, ...] | list[float]) -> PointMassState:
        """Unpack a canonical numeric vector and validate its dimension."""

        if len(values) != len(cls.STATE_NAMES):
            raise ValueError(f"TAOS point-mass state requires {len(cls.STATE_NAMES)} values")
        x, y, z, xdot, ydot, zdot, mass, path_length, ground_range = (float(value) for value in values)
        return cls(
            time,
            FrameVector3(Vector3(x, y, z), Frame.ECFC),
            FrameVector3(Vector3(xdot, ydot, zdot), Frame.ECFC),
            mass,
            path_length,
            ground_range,
        )
        ####

    @classmethod
    def from_simulation_state(cls, state: SimulationState) -> PointMassState:
        """Adapt a generic integrator state back to the typed physical state."""

        if state.frame not in {Frame.ECFC.value, Frame.ECFC.name, "ecfc"}:
            raise ValueError("TAOS point-mass state must use the ECFC frame")
        return cls.from_values(state.time, list(state.values))
        ####


@dataclass(frozen=True, slots=True)
class PointMassRates:
    """Derivative vector matching :class:`PointMassState` exactly."""

    velocity_derivative: FrameVector3
    acceleration_derivative: FrameVector3
    mass_rate: float
    path_length_rate: float
    ground_range_rate: float

    def to_values(self) -> tuple[float, ...]:
        """Pack derivatives in the same order as the state."""

        velocity = self.velocity_derivative.vector
        acceleration = self.acceleration_derivative.vector
        return (velocity.x, velocity.y, velocity.z, acceleration.x, acceleration.y, acceleration.z, self.mass_rate, self.path_length_rate, self.ground_range_rate)
        ####

    def __post_init__(self) -> None:
        if self.velocity_derivative.frame is not Frame.ECFC or self.acceleration_derivative.frame is not Frame.ECFC:
            raise ValueError("point-mass derivatives must be expressed in the ECFC frame")
        if not all(math.isfinite(value) for value in (*self.to_values(),)):
            raise ValueError("point-mass derivative values must be finite")
        ####
