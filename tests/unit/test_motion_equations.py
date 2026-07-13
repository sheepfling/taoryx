from __future__ import annotations

import pytest

from taoryx.equations import (
    CartesianVector3,
    augmented_trajectory_derivative_vector,
    augmented_trajectory_state_vector,
    earth_fixed_equation_of_motion,
    earth_spin_rate_magnitude,
    earth_spin_vector,
    ecfc_x_acceleration_state_form,
    ecfc_y_acceleration_state_form,
    ecfc_z_acceleration_state_form,
    first_order_equations_of_motion,
    inertial_acceleration_from_earth_fixed,
    newton_point_mass_acceleration,
    runge_kutta_fourth_order_step,
    runge_kutta_stage_one,
    runge_kutta_stage_three,
    runge_kutta_stage_two,
    runge_kutta_stage_zero,
    trajectory_state_derivative_vector,
    trajectory_state_vector,
)


def test_newton_point_mass_and_state_vector_helpers_follow_the_manual_formulas() -> None:
    force = CartesianVector3(12.0, -6.0, 3.0)
    mass = 3.0
    position = CartesianVector3(10.0, 20.0, 30.0)
    velocity = CartesianVector3(-4.0, 5.0, -6.0)

    assert newton_point_mass_acceleration(force, mass) == CartesianVector3(4.0, -2.0, 1.0)
    assert trajectory_state_vector(position, velocity) == (10.0, 20.0, 30.0, -4.0, 5.0, -6.0)
    assert trajectory_state_derivative_vector(velocity, force) == (-4.0, 5.0, -6.0, 12.0, -6.0, 3.0)
    assert augmented_trajectory_state_vector(position, velocity, 12.0, 14.0, 16.0) == (
        10.0,
        20.0,
        30.0,
        -4.0,
        5.0,
        -6.0,
        12.0,
        14.0,
        16.0,
    )
    assert augmented_trajectory_derivative_vector(velocity, force, -1.0, 2.0, 3.0) == (
        -4.0,
        5.0,
        -6.0,
        12.0,
        -6.0,
        3.0,
        -1.0,
        2.0,
        3.0,
    )
####


def test_earth_fixed_equation_of_motion_matches_coriolis_and_centrifugal_terms() -> None:
    total_force = CartesianVector3(24.0, -18.0, 6.0)
    mass = 2.0
    position = CartesianVector3(10.0, -4.0, 3.0)
    velocity = CartesianVector3(2.0, 5.0, -1.0)
    rotation_rate = 0.25
    spin = earth_spin_vector(rotation_rate)

    force_acceleration = newton_point_mass_acceleration(total_force, mass)
    earth_fixed = earth_fixed_equation_of_motion(total_force, mass, position, velocity, rotation_rate)
    inertial = inertial_acceleration_from_earth_fixed(position, velocity, earth_fixed, rotation_rate)

    coriolis = CartesianVector3(
        2.0 * (spin.y * velocity.z - spin.z * velocity.y),
        2.0 * (spin.z * velocity.x - spin.x * velocity.z),
        2.0 * (spin.x * velocity.y - spin.y * velocity.x),
    )
    centrifugal = CartesianVector3(
        spin.y * (spin.x * position.y - spin.y * position.x) - spin.z * (spin.z * position.x - spin.x * position.z),
        spin.z * (spin.y * position.z - spin.z * position.y) - spin.x * (spin.x * position.y - spin.y * position.x),
        spin.x * (spin.z * position.x - spin.x * position.z) - spin.y * (spin.y * position.z - spin.z * position.y),
    )

    assert earth_fixed == CartesianVector3(
        force_acceleration.x - coriolis.x - centrifugal.x,
        force_acceleration.y - coriolis.y - centrifugal.y,
        force_acceleration.z - coriolis.z - centrifugal.z,
    )
    assert inertial == force_acceleration
####


def test_state_form_acceleration_components_and_spin_magnitude_follow_manual_formulas() -> None:
    total_force = CartesianVector3(24.0, -18.0, 6.0)
    mass = 2.0
    position = CartesianVector3(10.0, -4.0, 3.0)
    velocity = CartesianVector3(2.0, 5.0, -1.0)
    rotation_rate = 0.25

    assert earth_spin_rate_magnitude(rotation_rate) == 0.25
    assert ecfc_x_acceleration_state_form(total_force, mass, position, velocity, rotation_rate) == pytest.approx(12.0 + 0.25 * (2.0 * 5.0 + 0.25 * 10.0))
    assert ecfc_y_acceleration_state_form(total_force, mass, position, velocity, rotation_rate) == pytest.approx(-9.0 + 0.25 * (-2.0 * 2.0 + 0.25 * -4.0))
    assert ecfc_z_acceleration_state_form(total_force, mass, position, velocity, rotation_rate) == pytest.approx(3.0)
####


def test_first_order_equations_of_motion_return_position_and_velocity_rates() -> None:
    total_force = CartesianVector3(8.0, 4.0, 2.0)
    mass = 2.0
    position = CartesianVector3(7.0, 8.0, 9.0)
    velocity = CartesianVector3(-1.0, 2.0, -3.0)
    rate = 7.2921150e-5

    position_rate, velocity_rate = first_order_equations_of_motion(total_force, mass, position, velocity, rate)

    assert position_rate == velocity
    assert velocity_rate == earth_fixed_equation_of_motion(total_force, mass, position, velocity, rate)
####


def test_runge_kutta_fourth_order_step_matches_the_classic_scalar_ode_result() -> None:
    def derivative(time_seconds: float, state: tuple[float, ...]) -> tuple[float, ...]:
        _ = time_seconds
        return state

    stage_zero = runge_kutta_stage_zero(derivative, 0.0, (1.0,))
    stage_one = runge_kutta_stage_one(derivative, 0.0, (1.0,), 0.1, stage_zero)
    stage_two = runge_kutta_stage_two(derivative, 0.0, (1.0,), 0.1, stage_one)
    stage_three = runge_kutta_stage_three(derivative, 0.0, (1.0,), 0.1, stage_two)
    stepped = runge_kutta_fourth_order_step(derivative, 0.0, (1.0,), 0.1)

    assert stage_zero == (1.0,)
    assert stage_one == pytest.approx((1.05,))
    assert stage_two == pytest.approx((1.0525,))
    assert stage_three == pytest.approx((1.10525,))
    assert stepped == pytest.approx((1.1051708333333332,))
####
