"""Trajectory equations of motion helpers."""

from __future__ import annotations

from collections.abc import Callable, Sequence
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


def earth_spin_rate_magnitude(rotation_rate_radians_per_second: float) -> float:
    """Return the magnitude of the earth spin rate vector."""

    return abs(rotation_rate_radians_per_second)
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


def ecfc_x_acceleration_state_form(
    total_force: CartesianVector3,
    mass: float,
    position: CartesianVector3,
    velocity: CartesianVector3,
    rotation_rate_radians_per_second: float,
) -> float:
    """Return the earth-fixed x acceleration from the TAOS state-form equation."""

    force_acceleration = newton_point_mass_acceleration(total_force, mass)
    return force_acceleration.x + rotation_rate_radians_per_second * (
        2.0 * velocity.y + rotation_rate_radians_per_second * position.x
    )
####


def ecfc_y_acceleration_state_form(
    total_force: CartesianVector3,
    mass: float,
    position: CartesianVector3,
    velocity: CartesianVector3,
    rotation_rate_radians_per_second: float,
) -> float:
    """Return the earth-fixed y acceleration from the TAOS state-form equation."""

    force_acceleration = newton_point_mass_acceleration(total_force, mass)
    return force_acceleration.y + rotation_rate_radians_per_second * (
        -2.0 * velocity.x + rotation_rate_radians_per_second * position.y
    )
####


def ecfc_z_acceleration_state_form(
    total_force: CartesianVector3,
    mass: float,
    position: CartesianVector3,
    velocity: CartesianVector3,
    rotation_rate_radians_per_second: float,
) -> float:
    """Return the earth-fixed z acceleration from the TAOS state-form equation."""

    _ = position, velocity, rotation_rate_radians_per_second
    return newton_point_mass_acceleration(total_force, mass).z
####


def augmented_trajectory_state_vector(
    position: CartesianVector3,
    velocity: CartesianVector3,
    mass: float,
    path_length: float,
    ground_range: float,
) -> tuple[float, float, float, float, float, float, float, float, float]:
    """Return the augmented trajectory state vector used by TAOS integration."""

    return (
        position.x,
        position.y,
        position.z,
        velocity.x,
        velocity.y,
        velocity.z,
        mass,
        path_length,
        ground_range,
    )
####


def augmented_trajectory_derivative_vector(
    velocity: CartesianVector3,
    acceleration: CartesianVector3,
    mass_rate: float,
    path_length_rate: float,
    ground_range_rate: float,
) -> tuple[float, float, float, float, float, float, float, float, float]:
    """Return the augmented trajectory derivative vector."""

    return (
        velocity.x,
        velocity.y,
        velocity.z,
        acceleration.x,
        acceleration.y,
        acceleration.z,
        mass_rate,
        path_length_rate,
        ground_range_rate,
    )
####


DerivativeFunction = Callable[[float, tuple[float, ...]], Sequence[float]]


def runge_kutta_stage_zero(
    derivative: DerivativeFunction,
    time_seconds: float,
    state: tuple[float, ...],
) -> tuple[float, ...]:
    """Return the first Runge-Kutta derivative evaluation."""

    return tuple(float(value) for value in derivative(time_seconds, state))
####


def runge_kutta_stage_one(
    derivative: DerivativeFunction,
    time_seconds: float,
    state: tuple[float, ...],
    step_size_seconds: float,
    stage_zero: tuple[float, ...],
) -> tuple[float, ...]:
    """Return the second Runge-Kutta derivative evaluation."""

    intermediate_state = tuple(
        value + 0.5 * step_size_seconds * rate
        for value, rate in zip(state, stage_zero, strict=True)
    )
    return tuple(float(value) for value in derivative(time_seconds + 0.5 * step_size_seconds, intermediate_state))
####


def runge_kutta_stage_two(
    derivative: DerivativeFunction,
    time_seconds: float,
    state: tuple[float, ...],
    step_size_seconds: float,
    stage_one: tuple[float, ...],
) -> tuple[float, ...]:
    """Return the third Runge-Kutta derivative evaluation."""

    intermediate_state = tuple(
        value + 0.5 * step_size_seconds * rate
        for value, rate in zip(state, stage_one, strict=True)
    )
    return tuple(float(value) for value in derivative(time_seconds + 0.5 * step_size_seconds, intermediate_state))
####


def runge_kutta_stage_three(
    derivative: DerivativeFunction,
    time_seconds: float,
    state: tuple[float, ...],
    step_size_seconds: float,
    stage_two: tuple[float, ...],
) -> tuple[float, ...]:
    """Return the fourth Runge-Kutta derivative evaluation."""

    intermediate_state = tuple(
        value + step_size_seconds * rate
        for value, rate in zip(state, stage_two, strict=True)
    )
    return tuple(float(value) for value in derivative(time_seconds + step_size_seconds, intermediate_state))
####


def runge_kutta_fourth_order_step(
    derivative: DerivativeFunction,
    time_seconds: float,
    state: tuple[float, ...],
    step_size_seconds: float,
) -> tuple[float, ...]:
    """Advance a state with a fourth-order Runge-Kutta step."""

    stage_zero = runge_kutta_stage_zero(derivative, time_seconds, state)
    stage_one = runge_kutta_stage_one(derivative, time_seconds, state, step_size_seconds, stage_zero)
    stage_two = runge_kutta_stage_two(derivative, time_seconds, state, step_size_seconds, stage_one)
    stage_three = runge_kutta_stage_three(derivative, time_seconds, state, step_size_seconds, stage_two)
    return tuple(
        value
        + (step_size_seconds / 6.0) * (k0 + 2.0 * k1 + 2.0 * k2 + k3)
        for value, k0, k1, k2, k3 in zip(state, stage_zero, stage_one, stage_two, stage_three, strict=True)
    )
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
