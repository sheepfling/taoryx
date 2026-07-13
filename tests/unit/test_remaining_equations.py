from __future__ import annotations

import math

import pytest

from taoryx.equations import (
    CartesianVector3,
    atmospheric_density_rate,
    atmospheric_density_second_rate,
    dynamic_pressure_definition,
    dynamic_pressure_second_derivative,
    full_table_thrust,
    initial_ecic_rotation,
    mach_number_definition,
    mach_number_rate,
    optimization_central_difference,
    optimization_forward_difference,
    optimization_qmin_satisfied,
    optimization_qmin_violated,
    qmin_satisfied,
    qmin_violated,
    rail_acceleration_magnitude,
    rail_constrained_acceleration_components,
    rail_friction_acceleration,
    rail_launch_acceleration_components,
    rail_normal_acceleration,
    speed_of_sound_rate,
    stagnation_heating,
)


def _identity_axes() -> tuple[CartesianVector3, CartesianVector3, CartesianVector3]:
    return (
        CartesianVector3(1.0, 0.0, 0.0),
        CartesianVector3(0.0, 1.0, 0.0),
        CartesianVector3(0.0, 0.0, 1.0),
    )
####


def test_propulsive_and_initial_rotation_helpers_follow_the_manual_formulas() -> None:
    assert full_table_thrust(1200.0, 10.0, 3.0) == pytest.approx(1170.0)
    assert initial_ecic_rotation(7.2921150e-5, 10.0, 0.0, 25.3) == pytest.approx(25.3007292115)
####


def test_rail_and_rate_helpers_follow_the_manual_formulas() -> None:
    axes = _identity_axes()
    total_acceleration = CartesianVector3(3.0, 4.0, 5.0)

    assert rail_launch_acceleration_components(7.0, axes) == CartesianVector3(7.0, 0.0, 0.0)
    assert rail_constrained_acceleration_components(7.0, axes) == CartesianVector3(7.0, 0.0, 0.0)
    assert rail_acceleration_magnitude(total_acceleration, axes, 1.0) == pytest.approx(2.0)
    assert rail_friction_acceleration(0.25, 4.0) == pytest.approx(1.0)
    assert rail_normal_acceleration(total_acceleration, axes) == pytest.approx(math.sqrt(41.0))

    assert dynamic_pressure_definition(2.0, 3.0) == pytest.approx(9.0)
    assert atmospheric_density_rate(5.0, 0.2) == pytest.approx(1.0)
    assert atmospheric_density_second_rate(5.0, 0.2) == pytest.approx(1.0)
    assert dynamic_pressure_second_derivative(1.0, 2.0, 3.0, 4.0, 5.0, 6.0) == pytest.approx(140.0)
    assert mach_number_definition(600.0, 300.0) == pytest.approx(2.0)
    assert mach_number_rate(600.0, 30.0, 300.0, 3.0) == pytest.approx(0.08)
    assert speed_of_sound_rate(5.0, 0.2) == pytest.approx(1.0)
####


def test_optimization_and_stagnation_helpers_follow_the_manual_formulas() -> None:
    assert stagnation_heating(1.0, 0.0023769, 0.0023769, 26000.0) == pytest.approx(17600.0)
    assert qmin_satisfied() == 0.0
    assert qmin_violated(100.0, 90.0) == pytest.approx(100.0)
    assert optimization_qmin_satisfied(100.0, 120.0, 2000.0) == 0.0
    assert optimization_qmin_satisfied(100.0, 90.0, 2000.0) == pytest.approx(100.0)
    assert optimization_qmin_violated(500.0, 490.0) == pytest.approx(100.0)
    assert optimization_forward_difference(4.0, 1.0, 1.5) == pytest.approx(2.0)
    assert optimization_central_difference(4.0, 1.0, 1.5) == pytest.approx(1.0)
####
