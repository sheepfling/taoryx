"""Typed geocentric gravity-model bindings."""

from __future__ import annotations

import math
from collections.abc import Mapping

from .contracts import Frame, FrameVector3, GeocentricCoordinates, Quantity, Unit, Vector3
from .equations import (
    associated_legendre_function,
    geopotential_spherical_harmonics,
    gravity_full_geocentric_components,
    gravity_j2_geocentric_components,
    normalized_gravity_coefficient,
    normalized_legendre_function,
)


def geopotential(
    position: GeocentricCoordinates,
    gravitational_parameter: Quantity,
    reference_radius: Quantity,
    coefficients: Mapping[tuple[int, int], tuple[float, float]],
) -> float:
    """Evaluate TAOS-ALG-GRAV-001 and equations 2-201 through 2-205."""

    radius, reference = _normalize_geometry(position, gravitational_parameter, reference_radius)
    return geopotential_spherical_harmonics(
        gravitational_parameter.to(Unit.METER_CUBED_PER_SECOND_SQUARED).value,
        reference,
        radius,
        position.latitude.radians,
        position.longitude.radians,
        coefficients,
    )
####


def associated_legendre(degree: int, order: int, sine_latitude: float) -> float:
    """Evaluate TAOS-ALG-GRAV-002 and equations 2-206 through 2-209."""

    if degree < 0 or order < 0 or order > degree:
        raise ValueError("Legendre degree/order must satisfy 0 <= order <= degree")
    if not math.isfinite(sine_latitude) or not -1.0 <= sine_latitude <= 1.0:
        raise ValueError("sine latitude must be finite and within [-1, 1]")
    return associated_legendre_function(degree, order, sine_latitude)
####


def harmonic_normalization_factor(degree: int, order: int, coefficient: float) -> tuple[float, float]:
    """Evaluate TAOS-ALG-GRAV-003 and equations 2-210 through 2-211."""

    if degree < 0 or order < 0 or order > degree:
        raise ValueError("harmonic degree/order must satisfy 0 <= order <= degree")
    if not math.isfinite(coefficient):
        raise ValueError("harmonic coefficient must be finite")
    return (
        normalized_legendre_function(degree, order, coefficient),
        normalized_gravity_coefficient(degree, order, coefficient),
    )
####


def gravity_acceleration_full(
    position: GeocentricCoordinates,
    gravitational_parameter: Quantity,
    reference_radius: Quantity,
    coefficients: Mapping[tuple[int, int], tuple[float, float]],
) -> FrameVector3:
    """Evaluate TAOS-ALG-GRAV-004 and equations 2-212 through 2-217."""

    radius, reference = _normalize_geometry(position, gravitational_parameter, reference_radius)
    legacy = gravity_full_geocentric_components(
        gravitational_parameter.to(Unit.METER_CUBED_PER_SECOND_SQUARED).value,
        reference,
        radius,
        position.latitude.radians,
        position.longitude.radians,
        coefficients,
    )
    return FrameVector3(Vector3(legacy.x, legacy.y, legacy.z), Frame.GEOCENTRIC_HORIZON)
####


def gravity_acceleration_j2(
    position: GeocentricCoordinates,
    gravitational_parameter: Quantity,
    reference_radius: Quantity,
    j2_coefficient: float,
) -> FrameVector3:
    """Evaluate TAOS-ALG-GRAV-005 and equations 2-218 through 2-220."""

    radius, reference = _normalize_geometry(position, gravitational_parameter, reference_radius)
    if not math.isfinite(j2_coefficient):
        raise ValueError("J2 coefficient must be finite")
    legacy = gravity_j2_geocentric_components(
        gravitational_parameter.to(Unit.METER_CUBED_PER_SECOND_SQUARED).value,
        reference,
        radius,
        position.latitude.radians,
        j2_coefficient,
    )
    return FrameVector3(Vector3(legacy.x, legacy.y, legacy.z), Frame.GEOCENTRIC_HORIZON)
####


def _normalize_geometry(
    position: GeocentricCoordinates,
    gravitational_parameter: Quantity,
    reference_radius: Quantity,
) -> tuple[float, float]:
    if position.radius.value <= 0.0:
        raise ValueError("geocentric radius must be positive")
    if gravitational_parameter.unit.dimension != "length^3/time^2":
        raise ValueError("gravitational parameter requires length^3/time^2 units")
    if reference_radius.unit.dimension != "length" or reference_radius.value <= 0.0:
        raise ValueError("reference radius must be a positive length")
    return position.radius.to(Unit.METER).value, reference_radius.to(Unit.METER).value
####
