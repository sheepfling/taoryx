"""Trajectory equations of motion helpers."""

from __future__ import annotations

from dataclasses import dataclass

from .geodesy import CartesianVector3


@dataclass(frozen=True, slots=True)
class MotionState:
    """Compact earth-fixed position and velocity state."""

    position: CartesianVector3
    velocity: CartesianVector3
####


@dataclass(frozen=True, slots=True)
class MotionRates:
    """Earth-fixed acceleration and related derived terms."""

    force_acceleration: CartesianVector3
    inertial_acceleration: CartesianVector3
    earth_fixed_acceleration: CartesianVector3
    coriolis_acceleration: CartesianVector3
    centrifugal_acceleration: CartesianVector3
####


def _cross(left: CartesianVector3, right: CartesianVector3) -> CartesianVector3:
    return CartesianVector3(
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    )
####


def earth_spin_vector(rotation_rate_radians_per_second: float) -> CartesianVector3:
    """Return the earth spin vector aligned with the positive z axis."""

    return CartesianVector3(0.0, 0.0, rotation_rate_radians_per_second)
####


def newton_point_mass_acceleration(total_force: CartesianVector3, mass: float) -> CartesianVector3:
    """Return the inertial acceleration from Newton's second law."""

    if mass == 0.0:
        raise ValueError("mass must be nonzero")
    ####
    return CartesianVector3(total_force.x / mass, total_force.y / mass, total_force.z / mass)
####


def inertial_acceleration_from_earth_fixed(
    position: CartesianVector3,
    velocity: CartesianVector3,
    earth_fixed_acceleration: CartesianVector3,
    rotation_rate_radians_per_second: float,
) -> CartesianVector3:
    """Recover inertial acceleration from the earth-fixed state and spin rate."""

    spin = earth_spin_vector(rotation_rate_radians_per_second)
    coriolis = _cross(spin, velocity)
    centrifugal = _cross(spin, _cross(spin, position))
    return CartesianVector3(
        earth_fixed_acceleration.x + 2.0 * coriolis.x + centrifugal.x,
        earth_fixed_acceleration.y + 2.0 * coriolis.y + centrifugal.y,
        earth_fixed_acceleration.z + 2.0 * coriolis.z + centrifugal.z,
    )
####


def earth_fixed_equation_of_motion(
    total_force: CartesianVector3,
    mass: float,
    position: CartesianVector3,
    velocity: CartesianVector3,
    rotation_rate_radians_per_second: float,
) -> CartesianVector3:
    """Return the earth-fixed acceleration from the rotating-earth equation of motion."""

    force_acceleration = newton_point_mass_acceleration(total_force, mass)
    spin = earth_spin_vector(rotation_rate_radians_per_second)
    coriolis = _cross(spin, velocity)
    centrifugal = _cross(spin, _cross(spin, position))
    return CartesianVector3(
        force_acceleration.x - 2.0 * coriolis.x - centrifugal.x,
        force_acceleration.y - 2.0 * coriolis.y - centrifugal.y,
        force_acceleration.z - 2.0 * coriolis.z - centrifugal.z,
    )
####


def trajectory_state_vector(position: CartesianVector3, velocity: CartesianVector3) -> tuple[float, float, float, float, float, float]:
    """Return the six-component trajectory state vector."""

    return position.x, position.y, position.z, velocity.x, velocity.y, velocity.z
####


def trajectory_state_derivative_vector(
    velocity: CartesianVector3,
    acceleration: CartesianVector3,
) -> tuple[float, float, float, float, float, float]:
    """Return the six-component derivative vector for the trajectory state."""

    return velocity.x, velocity.y, velocity.z, acceleration.x, acceleration.y, acceleration.z
####


def first_order_equations_of_motion(
    total_force: CartesianVector3,
    mass: float,
    position: CartesianVector3,
    velocity: CartesianVector3,
    rotation_rate_radians_per_second: float,
) -> tuple[CartesianVector3, CartesianVector3]:
    """Return the first-order position and velocity rates."""

    acceleration = earth_fixed_equation_of_motion(
        total_force,
        mass,
        position,
        velocity,
        rotation_rate_radians_per_second,
    )
    return velocity, acceleration
####
