from __future__ import annotations

import math

import pytest

from taoryx.equations import (
    CartesianVector3,
    Matrix,
    ballistic_coefficient,
    cubic_guidance_acceleration,
    cubic_guidance_coefficient_a,
    cubic_guidance_coefficient_b,
    cubic_guidance_state,
    guidance_gaussian_system,
    guidance_newton_update,
    iip_aerodynamic_acceleration_from_ballistic_coefficient,
    iip_aerodynamic_acceleration_from_density,
    iip_aerodynamic_acceleration_from_drag_coefficient,
    iip_aerodynamic_acceleration_from_dynamic_pressure,
    iip_aerodynamic_acceleration_vector,
    iip_wind_velocity_unit_vector,
    lift_to_drag_ratio,
    line_of_sight_pitch,
    line_of_sight_pitch_rate,
    line_of_sight_yaw,
    line_of_sight_yaw_rate,
    los_to_ecfc_acceleration_matrix,
    parabolic_guidance_coefficient_a,
    parabolic_guidance_coefficient_b,
    parabolic_guidance_rate,
    parabolic_guidance_state,
    predictive_intercept_point,
    predictive_intercept_position_equality,
    predictive_time_to_intercept,
    proportional_navigation_closure_velocity,
    proportional_navigation_ecfc_acceleration,
    proportional_navigation_pitch_acceleration,
    proportional_navigation_yaw_acceleration,
    source_los_pitch_rotation_matrix,
    source_los_yaw_rotation_matrix,
)


def test_iip_helpers_follow_the_manual_formulas() -> None:
    relative_velocity = CartesianVector3(0.0, 3.0, 4.0)
    unit_vector = iip_wind_velocity_unit_vector(relative_velocity)

    assert ballistic_coefficient(100.0, 2.0, 5.0) == pytest.approx(10.0)
    assert lift_to_drag_ratio(3.0, 2.0) == pytest.approx(1.5)
    assert unit_vector.x == pytest.approx(0.0)
    assert unit_vector.y == pytest.approx(0.6)
    assert unit_vector.z == pytest.approx(0.8)

    accel_from_ballistic = iip_aerodynamic_acceleration_from_ballistic_coefficient(0.5, 9.81, 2.0, unit_vector)
    accel_from_density = iip_aerodynamic_acceleration_from_density(9.81, 1.2, 2.0, relative_velocity)
    accel_from_dynamic_pressure = iip_aerodynamic_acceleration_from_dynamic_pressure(9.81, 1.2, 2.0, relative_velocity)
    accel_from_vector = iip_aerodynamic_acceleration_vector(9.81, 1.2, 2.0, relative_velocity)
    accel_from_drag = iip_aerodynamic_acceleration_from_drag_coefficient(0.5, 9.81, 2.0, 5.0, 20.0, unit_vector)
    assert accel_from_ballistic.x == pytest.approx(2.4525 * 0.0)
    assert accel_from_ballistic.y == pytest.approx(2.4525 * 0.6)
    assert accel_from_ballistic.z == pytest.approx(2.4525 * 0.8)
    assert accel_from_density == accel_from_vector
    assert accel_from_dynamic_pressure == accel_from_vector
    assert accel_from_drag.x == pytest.approx(accel_from_ballistic.x)
    assert accel_from_drag.y == pytest.approx(accel_from_ballistic.y)
    assert accel_from_drag.z == pytest.approx(accel_from_ballistic.z)
####


def test_parabolic_and_cubic_guidance_helpers_follow_the_manual_formulas() -> None:
    parabolic_a = parabolic_guidance_coefficient_a(5.0, 2.0, 1.0, 1.0)
    parabolic_b = parabolic_guidance_coefficient_b(0.0, parabolic_a, 1.0, 1.0)

    assert parabolic_a == pytest.approx(4.0)
    assert parabolic_b == pytest.approx(-7.0)
    assert parabolic_guidance_state(2.0, 4.0, -7.0, 3.0) == pytest.approx(5.0)
    assert parabolic_guidance_rate(2.0, 4.0, -7.0) == pytest.approx(9.0)

    cubic_b = cubic_guidance_coefficient_b(0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    cubic_a = cubic_guidance_coefficient_a(0.0, 1.0, 0.0, 1.0, cubic_b)

    assert cubic_b == pytest.approx(-1.0)
    assert cubic_a == pytest.approx(1.0)
    assert cubic_guidance_state(2.0, 1.0, -1.0, 0.0, 0.0) == pytest.approx(4.0)
    assert cubic_guidance_acceleration(2.0, 1.0, -1.0) == pytest.approx(10.0)
####


def test_predictive_intercept_helpers_follow_the_manual_formulas() -> None:
    interceptor_position = CartesianVector3(0.0, 0.0, 0.0)
    interceptor_velocity = CartesianVector3(1.0, 0.0, 0.0)
    target_position = CartesianVector3(10.0, 0.0, 0.0)
    target_velocity = CartesianVector3(0.0, 0.0, 0.0)

    time_to_intercept = predictive_time_to_intercept(
        interceptor_position,
        interceptor_velocity,
        target_position,
        target_velocity,
    )
    intercept_point = predictive_intercept_point(
        interceptor_position,
        interceptor_velocity,
        target_position,
        target_velocity,
    )
    left, right = predictive_intercept_position_equality(
        interceptor_position,
        interceptor_velocity,
        target_position,
        target_velocity,
        time_to_intercept,
        0.0,
    )

    assert time_to_intercept == pytest.approx(10.0)
    assert intercept_point == CartesianVector3(10.0, 0.0, 0.0)
    assert left == right == CartesianVector3(10.0, 0.0, 0.0)
####


def test_line_of_sight_and_navigation_helpers_follow_the_manual_formulas() -> None:
    relative_position = CartesianVector3(3.0, 4.0, 0.0)
    relative_velocity = CartesianVector3(0.0, 0.0, 1.0)

    assert line_of_sight_yaw(relative_position) == pytest.approx(math.atan2(4.0, 3.0))
    assert line_of_sight_pitch(CartesianVector3(3.0, 4.0, 12.0)) == pytest.approx(math.atan2(12.0, 5.0))
    assert line_of_sight_yaw_rate(relative_position, relative_velocity) == pytest.approx(0.0)
    assert line_of_sight_pitch_rate(relative_position, relative_velocity) == pytest.approx(0.2)
    assert proportional_navigation_closure_velocity(relative_position, CartesianVector3(-1.0, 0.0, 0.0)) == pytest.approx(0.6)
    assert proportional_navigation_yaw_acceleration(3.0, 2.0, 0.5) == pytest.approx(3.0)
    assert proportional_navigation_pitch_acceleration(3.0, 2.0, 0.25) == pytest.approx(1.5)
####


def test_line_of_sight_rotation_helpers_follow_the_manual_formulas() -> None:
    pitch_matrix = source_los_pitch_rotation_matrix(0.0)
    yaw_matrix = source_los_yaw_rotation_matrix(0.0)
    acceleration_matrix = los_to_ecfc_acceleration_matrix(0.0, 0.0)

    assert pitch_matrix == los_to_ecfc_acceleration_matrix(0.0, 0.0)
    assert yaw_matrix.row0 == CartesianVector3(1.0, 0.0, 0.0)
    assert yaw_matrix.row1 == CartesianVector3(0.0, 1.0, 0.0)
    assert yaw_matrix.row2 == CartesianVector3(0.0, 0.0, 1.0)
    assert acceleration_matrix.row0 == CartesianVector3(1.0, 0.0, 0.0)
    assert acceleration_matrix.row1 == CartesianVector3(0.0, 1.0, 0.0)
    assert acceleration_matrix.row2 == CartesianVector3(0.0, 0.0, 1.0)
    assert proportional_navigation_ecfc_acceleration(0.0, 0.0, 2.0, 3.0) == CartesianVector3(0.0, 2.0, 3.0)
####


def test_newton_and_gaussian_helpers_follow_the_manual_formulas() -> None:
    matrix = Matrix(((2.0, 0.0), (0.0, 4.0)))
    residual = (2.0, 8.0)

    assert guidance_gaussian_system(residual, matrix) == (-1.0, -2.0)
    assert guidance_newton_update((10.0, 20.0), residual, matrix) == (9.0, 18.0)
####
