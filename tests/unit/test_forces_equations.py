from __future__ import annotations

import math

import pytest

from taoryx.equations import (
    BodyAxes,
    CartesianVector3,
    WindAxes,
    aerodynamic_force_from_axial_and_normal_coefficients,
    aerodynamic_force_from_body_axis_coefficients,
    aerodynamic_force_from_lift_drag_side_coefficients,
    body_windward_meridian_unit_vector,
    normal_specific_load_magnitude,
    propulsive_force_vector,
    specific_load_components,
    specific_load_vector_from_accelerations,
    specific_load_vector_from_forces,
    wind_body_axes_from_aerodynamic_angles,
    wind_unit_vectors_from_velocity,
)


def _identity_axes() -> tuple[WindAxes, BodyAxes]:
    x_axis = CartesianVector3(1.0, 0.0, 0.0)
    y_axis = CartesianVector3(0.0, 1.0, 0.0)
    z_axis = CartesianVector3(0.0, 0.0, 1.0)
    axes = WindAxes(x_axis, y_axis, z_axis)
    return axes, BodyAxes(x_axis, y_axis, z_axis)
####


def test_body_windward_meridian_vector_and_axial_normal_force_follow_the_manual_formula() -> None:
    wind_axes = WindAxes(CartesianVector3(1.0, 0.0, 0.0), CartesianVector3(0.0, 1.0, 0.0), CartesianVector3(0.0, 0.0, 1.0))
    body_axes = wind_body_axes_from_aerodynamic_angles(wind_axes, math.radians(20.0), math.radians(15.0))

    phi_b = body_windward_meridian_unit_vector(wind_axes, body_axes)
    magnitude = math.hypot(
        wind_axes.x.x * body_axes.y.x + wind_axes.x.y * body_axes.y.y + wind_axes.x.z * body_axes.y.z,
        wind_axes.x.x * body_axes.z.x + wind_axes.x.y * body_axes.z.y + wind_axes.x.z * body_axes.z.z,
    )
    assert phi_b.x == pytest.approx(
        (
            (wind_axes.x.x * body_axes.y.x + wind_axes.x.y * body_axes.y.y + wind_axes.x.z * body_axes.y.z) * body_axes.y.x
            + (wind_axes.x.x * body_axes.z.x + wind_axes.x.y * body_axes.z.y + wind_axes.x.z * body_axes.z.z) * body_axes.z.x
        )
        / magnitude
    )
    assert phi_b.y == pytest.approx(
        (
            (wind_axes.x.x * body_axes.y.x + wind_axes.x.y * body_axes.y.y + wind_axes.x.z * body_axes.y.z) * body_axes.y.y
            + (wind_axes.x.x * body_axes.z.x + wind_axes.x.y * body_axes.z.y + wind_axes.x.z * body_axes.z.z) * body_axes.z.y
        )
        / magnitude
    )
    assert phi_b.z == pytest.approx(
        (
            (wind_axes.x.x * body_axes.y.x + wind_axes.x.y * body_axes.y.y + wind_axes.x.z * body_axes.y.z) * body_axes.y.z
            + (wind_axes.x.x * body_axes.z.x + wind_axes.x.y * body_axes.z.y + wind_axes.x.z * body_axes.z.z) * body_axes.z.z
        )
        / magnitude
    )

    force = aerodynamic_force_from_axial_and_normal_coefficients(wind_axes, body_axes, 2.0, 3.0, 4.0, 5.0)
    assert force == CartesianVector3(
        -(4.0 * body_axes.x.x + 5.0 * phi_b.x) * 6.0,
        -(4.0 * body_axes.x.y + 5.0 * phi_b.y) * 6.0,
        -(4.0 * body_axes.x.z + 5.0 * phi_b.z) * 6.0,
    )
####


def test_lift_drag_side_body_axis_and_propulsive_force_formulas_follow_the_manual() -> None:
    wind_axes, body_axes = _identity_axes()
    aero_force = aerodynamic_force_from_lift_drag_side_coefficients(wind_axes, 2.0, 3.0, 1.5, 4.0, -2.0)
    body_force = aerodynamic_force_from_body_axis_coefficients(body_axes, 2.0, 3.0, 1.0, 2.0, 3.0)
    thrust = propulsive_force_vector(100.0, math.radians(20.0), math.radians(-15.0))

    assert aero_force == CartesianVector3(-24.0, -12.0, -9.0)
    assert body_force == CartesianVector3(6.0, 12.0, 18.0)
    assert thrust == CartesianVector3(
        100.0 * math.cos(math.radians(20.0)),
        -100.0 * math.sin(math.radians(20.0)) * math.cos(math.radians(-15.0)),
        -100.0 * math.sin(math.radians(20.0)) * math.sin(math.radians(-15.0)),
    )
####


def test_specific_load_helpers_follow_the_manual_definitions() -> None:
    _, body_axes = _identity_axes()

    inertial_acceleration = CartesianVector3(9.0, 8.0, 7.0)
    gravity_acceleration = CartesianVector3(1.0, 2.0, 3.0)
    aerodynamic_force = CartesianVector3(6.0, -3.0, 9.0)
    propulsive_force = CartesianVector3(0.0, 6.0, -3.0)
    specific_load_vector = CartesianVector3(0.0, 5.0, 12.0)

    assert specific_load_vector_from_accelerations(inertial_acceleration, gravity_acceleration) == CartesianVector3(8.0, 6.0, 4.0)
    assert specific_load_vector_from_forces(aerodynamic_force, propulsive_force, 3.0) == CartesianVector3(2.0, 1.0, 2.0)
    assert specific_load_components(specific_load_vector, body_axes) == pytest.approx((0.0, 5.0, 12.0, 13.0))
    assert normal_specific_load_magnitude(specific_load_vector, body_axes) == pytest.approx(13.0)

    with pytest.raises(ValueError, match="mass must be nonzero"):
        specific_load_vector_from_forces(aerodynamic_force, propulsive_force, 0.0)
####


def test_wind_and_meridian_singularity_diagnostics_are_explicit() -> None:
    with pytest.raises(ValueError, match="wind axes are undefined at zero speed"):
        wind_unit_vectors_from_velocity(
            CartesianVector3(0.0, 0.0, 0.0),
            geodetic_up=CartesianVector3(0.0, 0.0, 1.0),
        )

    with pytest.raises(ValueError, match="wind axes are undefined when velocity is parallel to the geodetic up axis"):
        wind_unit_vectors_from_velocity(
            CartesianVector3(0.0, 0.0, 5.0),
            geodetic_up=CartesianVector3(0.0, 0.0, 1.0),
        )

    wind_axes = WindAxes(CartesianVector3(1.0, 0.0, 0.0), CartesianVector3(0.0, 1.0, 0.0), CartesianVector3(0.0, 0.0, 1.0))
    body_axes = BodyAxes(CartesianVector3(1.0, 0.0, 0.0), CartesianVector3(0.0, 1.0, 0.0), CartesianVector3(0.0, 0.0, 1.0))
    with pytest.raises(ValueError, match="windward meridian unit vector is undefined when total angle of attack is zero"):
        aerodynamic_force_from_axial_and_normal_coefficients(wind_axes, body_axes, 1.0, 1.0, 1.0, 1.0)
####
