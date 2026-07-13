from __future__ import annotations

import math

import pytest

from taoryx.contracts import Frame, GeocentricCoordinates, Latitude, Longitude, Quantity, Unit
from taoryx.gravity import associated_legendre, geopotential, gravity_acceleration_full, gravity_acceleration_j2, harmonic_normalization_factor


def _position() -> GeocentricCoordinates:
    return GeocentricCoordinates(Quantity(7000.0, Unit.KILOMETER), Longitude(0.3), Latitude(0.4))


def test_gravity_wrappers_preserve_equation_registry_values_and_frames() -> None:
    position = _position()
    gravitational_parameter = Quantity(398600.4418, Unit.METER_CUBED_PER_SECOND_SQUARED)
    reference_radius = Quantity(6378.137, Unit.KILOMETER)
    coefficients = {(2, 0): (0.1, 0.0)}

    full = gravity_acceleration_full(position, gravitational_parameter, reference_radius, coefficients)
    j2 = gravity_acceleration_j2(position, gravitational_parameter, reference_radius, 0.00108262668)

    assert full.frame is Frame.GEOCENTRIC_HORIZON
    assert full.vector.norm() > 0.0
    assert j2.frame is Frame.GEOCENTRIC_HORIZON
    assert geopotential(position, gravitational_parameter, reference_radius, coefficients) > 0.0
####


def test_legendre_and_normalization_contracts_handle_valid_and_invalid_inputs() -> None:
    assert associated_legendre(2, 0, 0.0) == pytest.approx(-0.5)
    normalized, coefficient = harmonic_normalization_factor(2, 0, 0.1)
    assert normalized == pytest.approx(math.sqrt(5.0) * 0.1)
    assert math.isfinite(coefficient)
    with pytest.raises(ValueError, match="degree/order"):
        associated_legendre(1, 2, 0.0)
