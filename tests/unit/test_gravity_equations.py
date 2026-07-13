from __future__ import annotations

import math

import pytest

from taoryx.equations import (
    CartesianVector3,
    associated_legendre_function,
    first_four_legendre_functions,
    geopotential_spherical_harmonics,
    gravity_acceleration_from_geopotential_gradient,
    gravity_full_geocentric_components,
    gravity_geocentric_x_gradient,
    gravity_geocentric_y_gradient,
    gravity_geocentric_z_gradient,
    gravity_j2_geocentric_components,
    legendre_derivative_notation,
    legendre_order_zero_function,
    normalized_gravity_coefficient,
    normalized_legendre_function,
    point_mass_geopotential,
    zonal_geopotential_from_c_coefficients,
    zonal_geopotential_from_j_coefficients,
)


def test_legendre_and_normalization_helpers_follow_the_manual_formulas() -> None:
    sine_latitude = 0.5

    assert point_mass_geopotential(6.0, 2.0) == pytest.approx(3.0)
    assert legendre_order_zero_function(1, sine_latitude) == pytest.approx(0.5)
    assert first_four_legendre_functions(sine_latitude) == pytest.approx((0.5, -0.125, -0.4375, -0.2890625))
    assert associated_legendre_function(2, 1, sine_latitude) == pytest.approx(-1.299038105676658)
    assert legendre_derivative_notation(2, 0, sine_latitude) == pytest.approx(1.5)
    assert normalized_legendre_function(2, 0, -0.125) == pytest.approx(-0.2795084971874737)
    assert normalized_gravity_coefficient(2, 0, -0.1) == pytest.approx(-0.044721359549995794)
####


def test_geopotential_and_acceleration_helpers_follow_the_manual_formulas() -> None:
    gravitational_parameter = 3.986004418e14
    reference_radius = 6_378_137.0
    radius = 7_000_000.0
    latitude = math.radians(30.0)
    longitude = math.radians(40.0)

    coefficients = {(2, 0): (-1.08263e-3, 0.0)}
    expected_point_mass = gravitational_parameter / radius
    expected_zonal = zonal_geopotential_from_c_coefficients(
        gravitational_parameter,
        reference_radius,
        radius,
        latitude,
        {2: -1.08263e-3},
    )

    assert gravity_acceleration_from_geopotential_gradient(CartesianVector3(1.0, 2.0, 3.0)) == CartesianVector3(1.0, 2.0, 3.0)
    assert gravity_geocentric_x_gradient(radius, latitude, 14.0) == pytest.approx(2.0e-6)
    assert gravity_geocentric_y_gradient(radius, latitude, 21.0) == pytest.approx(21.0 / (radius * math.cos(latitude)))
    assert gravity_geocentric_z_gradient(-9.5) == pytest.approx(9.5)
    assert geopotential_spherical_harmonics(
        gravitational_parameter,
        reference_radius,
        radius,
        latitude,
        longitude,
        {},
    ) == pytest.approx(expected_point_mass)
    assert geopotential_spherical_harmonics(
        gravitational_parameter,
        reference_radius,
        radius,
        latitude,
        longitude,
        coefficients,
    ) == pytest.approx(expected_zonal)
    assert zonal_geopotential_from_j_coefficients(
        gravitational_parameter,
        reference_radius,
        radius,
        latitude,
        {2: 1.08263e-3},
    ) == pytest.approx(expected_zonal)

    full_components = gravity_full_geocentric_components(
        gravitational_parameter,
        reference_radius,
        radius,
        latitude,
        longitude,
        coefficients,
    )
    j2_components = gravity_j2_geocentric_components(
        gravitational_parameter,
        reference_radius,
        radius,
        latitude,
        1.08263e-3,
    )
    assert full_components.x == pytest.approx(j2_components.x, rel=1e-12, abs=1e-12)
    assert full_components.y == pytest.approx(j2_components.y, rel=1e-12, abs=1e-12)
    assert full_components.z == pytest.approx(j2_components.z, rel=1e-12, abs=1e-12)
####
