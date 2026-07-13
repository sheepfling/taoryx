"""Earth-fixed point-mass dynamics from the TAOS equation catalog."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

from .contracts import Basis3, EarthModel, Frame, FrameVector3, Vector3


@dataclass(frozen=True, slots=True)
class EarthFixedDerivatives:
    """Position and velocity derivatives expressed in the ECFC frame."""

    position_derivative: FrameVector3
    velocity_derivative: FrameVector3
####


class ConstraintMode(StrEnum):
    """Directional behavior for a launch rail or sled track."""

    RAIL = "rail"
    SLED = "sled"
####


@dataclass(frozen=True, slots=True)
class RailConstraintResult:
    """Projected acceleration and the friction quantities used to obtain it."""

    acceleration: FrameVector3
    rail_acceleration_magnitude: float
    normal_acceleration: float
    friction_acceleration: float
    friction_coefficient: float


@dataclass(frozen=True, slots=True)
class SpecificLoadFactors:
    """Specific-load vector projections in body axes and their magnitudes."""

    specific_acceleration: FrameVector3
    body_x: float
    body_y: float
    body_z: float
    total: float
    normal: float


@dataclass(frozen=True, slots=True)
class AugmentedTrajectoryState:
    """ECFC state plus the scalar trajectory integrals in equations 2-107--2-113."""

    position: FrameVector3
    earth_relative_velocity: FrameVector3
    mass: float
    path_length: float
    ground_range: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.mass) or self.mass <= 0.0:
            raise ValueError("mass must be positive and finite")
        if not math.isfinite(self.path_length) or not math.isfinite(self.ground_range):
            raise ValueError("trajectory integrals must be finite")
        ####
    ####


@dataclass(frozen=True, slots=True)
class AugmentedTrajectoryDerivatives:
    """Named derivative components for an augmented trajectory state."""

    position_derivative: FrameVector3
    velocity_derivative: FrameVector3
    mass_rate: float
    path_length_rate: float
    ground_range_rate: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.mass_rate, self.path_length_rate, self.ground_range_rate)):
            raise ValueError("trajectory derivative scalars must be finite")
        ####
    ####


def earth_fixed_derivatives(
    position: FrameVector3,
    earth_relative_velocity: FrameVector3,
    total_force: FrameVector3,
    mass: float,
    earth: EarthModel,
) -> EarthFixedDerivatives:
    """Evaluate the rotating-Earth point-mass equations of motion.

    This implements TAOS-ALG-DYN-001 and equations 2-97 through 2-106. With
    omega aligned to the ECFC z axis, the acceleration is force/mass minus the
    Coriolis and centrifugal terms.
    """

    for name, vector in (("position", position), ("earth-relative velocity", earth_relative_velocity), ("total force", total_force)):
        if vector.frame is not Frame.ECFC:
            raise ValueError(f"{name} must be expressed in the ECFC frame")
        ####
    if not math.isfinite(mass) or mass <= 0.0:
        raise ValueError("mass must be positive and finite")
    rotation_rate = earth.rotation_rate.si_value
    omega = Vector3(0.0, 0.0, rotation_rate)
    coriolis = omega.cross(earth_relative_velocity.vector).scaled(-2.0)
    centrifugal = omega.cross(omega.cross(position.vector)).scaled(-1.0)
    acceleration = total_force.vector.scaled(1.0 / mass) + coriolis + centrifugal
    return EarthFixedDerivatives(
        position_derivative=FrameVector3(earth_relative_velocity.vector, Frame.ECFC),
        velocity_derivative=FrameVector3(acceleration, Frame.ECFC),
    )
####


def assemble_state_derivatives(
    state: AugmentedTrajectoryState,
    total_force: FrameVector3,
    mass_rate: float,
    ground_speed: float,
    earth: EarthModel,
) -> AugmentedTrajectoryDerivatives:
    """Assemble the complete augmented derivative vector from equations 2-107--2-113.

    This implements ``TAOS-ALG-DYN-002``. ``ground_speed`` is supplied by the
    ground-kinematics algorithm (catalog ``DYN-010``); the assembler only owns
    the state ordering and path-integral rates.
    """

    if not math.isfinite(mass_rate) or not math.isfinite(ground_speed) or ground_speed < 0.0:
        raise ValueError("mass_rate must be finite and ground_speed must be finite and non-negative")
    derivatives = earth_fixed_derivatives(
        state.position,
        state.earth_relative_velocity,
        total_force,
        state.mass,
        earth,
    )
    return AugmentedTrajectoryDerivatives(
        derivatives.position_derivative,
        derivatives.velocity_derivative,
        mass_rate,
        state.earth_relative_velocity.vector.norm(),
        ground_speed,
    )
####


def apply_rail_constraint(
    total_acceleration: FrameVector3,
    body_basis: Basis3,
    speed: float,
    static_friction_coefficient: float,
    kinetic_friction_coefficient: float,
    *,
    mode: ConstraintMode = ConstraintMode.RAIL,
    static_speed_threshold: float = 0.001,
) -> RailConstraintResult:
    """Restrict ECFC acceleration to the body-x rail or sled direction.

    This implements ``TAOS-ALG-DYN-004`` and equations 2-119 through 2-122.
    Static friction is selected below the documented speed threshold. Rail
    launches clamp nonpositive along-track acceleration to zero; sleds retain
    the signed value so deceleration is possible.
    """

    if total_acceleration.frame is not Frame.ECFC:
        raise ValueError("total acceleration must be expressed in ECFC")
    if body_basis.parent_frame is not Frame.ECFC or body_basis.child_frame is not Frame.BODY:
        raise ValueError("body basis must be an ECFC to BODY basis")
    if not body_basis.is_orthonormal():
        raise ValueError("body basis must be orthonormal")
    if not math.isfinite(speed) or speed < 0.0:
        raise ValueError("speed must be finite and non-negative")
    if not math.isfinite(static_speed_threshold) or static_speed_threshold < 0.0:
        raise ValueError("static_speed_threshold must be finite and non-negative")
    if not all(math.isfinite(value) and value >= 0.0 for value in (static_friction_coefficient, kinetic_friction_coefficient)):
        raise ValueError("friction coefficients must be finite and non-negative")
    if not isinstance(mode, ConstraintMode):
        raise ValueError("mode must be a ConstraintMode")
    body_x, body_y, body_z = body_basis.first, body_basis.second, body_basis.third
    axial_acceleration = total_acceleration.vector.dot(body_x)
    normal_acceleration = math.hypot(total_acceleration.vector.dot(body_y), total_acceleration.vector.dot(body_z))
    coefficient = static_friction_coefficient if speed < static_speed_threshold else kinetic_friction_coefficient
    friction_acceleration = coefficient * normal_acceleration
    rail_acceleration = axial_acceleration - friction_acceleration
    if mode is ConstraintMode.RAIL and rail_acceleration <= 0.0:
        rail_acceleration = 0.0
    constrained = body_x.scaled(rail_acceleration)
    return RailConstraintResult(
        FrameVector3(constrained, Frame.ECFC),
        rail_acceleration,
        normal_acceleration,
        friction_acceleration,
        coefficient,
    )
####


def specific_load_factors(
    inertial_acceleration: FrameVector3,
    gravity_acceleration: FrameVector3,
    body_basis: Basis3,
) -> SpecificLoadFactors:
    """Evaluate TAOS-ALG-DYN-012 and equations 2-172 through 2-174.

    Values retain the coherent acceleration units of the inputs; no implicit
    standard-gravity normalization is performed.
    """

    if inertial_acceleration.frame is not gravity_acceleration.frame:
        raise ValueError("inertial and gravity accelerations must share a frame")
    if body_basis.parent_frame is not inertial_acceleration.frame or body_basis.child_frame is not Frame.BODY:
        raise ValueError("body basis must match the acceleration frame and have BODY as its child frame")
    if not body_basis.is_orthonormal():
        raise ValueError("body basis must be orthonormal")
    specific = FrameVector3(inertial_acceleration.vector - gravity_acceleration.vector, inertial_acceleration.frame)
    components = (
        specific.vector.dot(body_basis.first),
        specific.vector.dot(body_basis.second),
        specific.vector.dot(body_basis.third),
    )
    total = specific.vector.norm()
    normal = math.hypot(components[1], components[2])
    return SpecificLoadFactors(specific, components[0], components[1], components[2], total, normal)
####


def combine_acceleration_contributions(
    aerodynamic_force: FrameVector3,
    propulsive_force: FrameVector3,
    gravity_force: FrameVector3,
    position: FrameVector3,
    earth_relative_velocity: FrameVector3,
    mass: float,
    earth: EarthModel,
) -> FrameVector3:
    """Evaluate TAOS-ALG-ENV-001 and equation 2-175.

    The three force contributions are summed before rotating-Earth Coriolis
    and centrifugal terms are applied by DYN-001.
    """

    for name, force in (("aerodynamic force", aerodynamic_force), ("propulsive force", propulsive_force), ("gravity force", gravity_force)):
        if force.frame is not Frame.ECFC:
            raise ValueError(f"{name} must be expressed in ECFC")
    total_force = FrameVector3(aerodynamic_force.vector + propulsive_force.vector + gravity_force.vector, Frame.ECFC)
    return earth_fixed_derivatives(position, earth_relative_velocity, total_force, mass, earth).velocity_derivative
####
