from __future__ import annotations

import math

import pytest

from taoryx.contracts import Latitude, Quantity, Unit
from taoryx.earth import ellipsoidal_surface_geometry, resolve_ellipsoid_parameters


def test_ellipsoid_parameter_conversion_preserves_units_and_equivalent_forms() -> None:
    radius = Quantity(6378.137, Unit.KILOMETER)
    by_flattening = resolve_ellipsoid_parameters(radius, flattening=1.0 / 298.257223563)
    by_eccentricity = resolve_ellipsoid_parameters(radius, eccentricity=by_flattening.eccentricity)

    assert by_flattening.polar_radius.value == pytest.approx(6356.752314245)
    assert by_flattening.polar_radius.unit is Unit.KILOMETER
    assert by_eccentricity.flattening == pytest.approx(by_flattening.flattening)
    assert by_eccentricity.polar_radius.value == pytest.approx(by_flattening.polar_radius.value)
####


def test_ellipsoidal_surface_geometry_matches_equation_definitions() -> None:
    parameters = resolve_ellipsoid_parameters(Quantity(6378.137, Unit.KILOMETER), flattening=1.0 / 298.257223563)
    latitude = Latitude(math.radians(45.0))
    geometry = ellipsoidal_surface_geometry(parameters, latitude)
    sine = math.sin(latitude.radians)
    expected_normal = parameters.equatorial_radius.value / math.sqrt(1.0 - parameters.eccentricity**2 * sine**2)

    assert geometry.normal_distance.value == pytest.approx(expected_normal)
    assert geometry.surface_equatorial_radius.value == pytest.approx(expected_normal * math.cos(latitude.radians))
    assert geometry.surface_polar_coordinate.value == pytest.approx(expected_normal * (1.0 - parameters.eccentricity**2) * sine)
    assert geometry.axis_intercept.value == pytest.approx(expected_normal * parameters.eccentricity**2 * sine)
####


def test_ellipsoid_parameter_conversion_rejects_invalid_or_inconsistent_inputs() -> None:
    with pytest.raises(ValueError, match="either eccentricity"):
        resolve_ellipsoid_parameters(Quantity(1.0, Unit.KILOMETER))
    with pytest.raises(ValueError, match="positive"):
        resolve_ellipsoid_parameters(Quantity(0.0, Unit.KILOMETER), flattening=0.1)
    with pytest.raises(ValueError, match="inconsistent"):
        resolve_ellipsoid_parameters(Quantity(1.0, Unit.KILOMETER), eccentricity=0.1, flattening=0.1)
####
