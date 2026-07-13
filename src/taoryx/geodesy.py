"""Typed Sodano ellipsoidal geodesy bindings."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .contracts import Angle, GeodeticCoordinates, Latitude, Longitude, Quantity
from .earth import EllipsoidParameters
from .equations import (
    sodano_backward_azimuth,
    sodano_central_angle,
    sodano_corrected_longitude_difference,
    sodano_delta_beta_series,
    sodano_direct_backward_azimuth,
    sodano_direct_cos_beta2,
    sodano_direct_l_correction,
    sodano_direct_latitude,
    sodano_direct_longitude,
    sodano_direct_m,
    sodano_direct_n,
    sodano_direct_phi,
    sodano_direct_sin_beta2,
    sodano_direct_xi,
    sodano_eprime,
    sodano_forward_azimuth,
    sodano_inverse_distance,
    sodano_inverse_m,
    sodano_reduced_latitude,
    sodano_third_flattening,
)


@dataclass(frozen=True, slots=True)
class SodanoInverseResult:
    """Distance and endpoint azimuths for an inverse geodesic."""

    distance: Quantity
    forward_azimuth: Angle
    backward_azimuth: Angle
####


@dataclass(frozen=True, slots=True)
class SodanoDirectResult:
    """Destination and reverse azimuth for a direct geodesic."""

    destination: GeodeticCoordinates
    backward_azimuth: Angle
####


def sodano_inverse(
    start: GeodeticCoordinates,
    destination: GeodeticCoordinates,
    parameters: EllipsoidParameters,
) -> SodanoInverseResult:
    """Evaluate TAOS-ALG-GEO-001 and equations 2-221 through 2-231.

    The historical direct and inverse series are independently truncated;
    they are not asserted to be exact numerical inverses of one another.
    """

    beta1 = sodano_reduced_latitude(start.latitude.radians, parameters.equatorial_radius.value, parameters.polar_radius.value)
    beta2 = sodano_reduced_latitude(destination.latitude.radians, parameters.equatorial_radius.value, parameters.polar_radius.value)
    delta_longitude = destination.longitude.radians - start.longitude.radians
    phi = sodano_central_angle(beta1, beta2, delta_longitude)
    inverse_m = sodano_inverse_m(beta1, beta2, phi, delta_longitude)
    distance = sodano_inverse_distance(
        parameters.equatorial_radius.value,
        parameters.polar_radius.value,
        parameters.flattening,
        beta1,
        beta2,
        phi,
        inverse_m,
    )
    corrected_longitude = sodano_corrected_longitude_difference(
        beta1,
        beta2,
        parameters.flattening,
        phi,
        inverse_m,
        delta_longitude,
    )
    delta_beta = sodano_delta_beta_series(
        start.latitude.radians,
        destination.latitude.radians,
        sodano_third_flattening(parameters.equatorial_radius.value, parameters.polar_radius.value),
    )
    forward = sodano_forward_azimuth(beta1, beta2, corrected_longitude, delta_beta)
    backward = sodano_backward_azimuth(beta1, beta2, corrected_longitude, delta_beta)
    return SodanoInverseResult(Quantity(distance, parameters.equatorial_radius.unit), Angle(forward), Angle(backward))
####


def sodano_direct(
    start: GeodeticCoordinates,
    surface_distance: Quantity,
    forward_azimuth: Angle,
    parameters: EllipsoidParameters,
) -> SodanoDirectResult:
    """Evaluate TAOS-ALG-GEO-003 and equations 2-235 through 2-245."""

    if surface_distance.unit.dimension != "length" or surface_distance.value < 0.0:
        raise ValueError("surface distance must be finite and non-negative")
    equatorial = parameters.equatorial_radius.value
    polar = parameters.polar_radius.value
    beta1 = sodano_reduced_latitude(start.latitude.radians, equatorial, polar)
    xi = sodano_direct_xi(surface_distance.to(parameters.polar_radius.unit).value, polar)
    eprime = sodano_eprime(parameters.eccentricity)
    m_value = sodano_direct_m(beta1, forward_azimuth.radians, eprime)
    n_value = sodano_direct_n(beta1, forward_azimuth.radians, xi, eprime)
    phi = sodano_direct_phi(xi, m_value, n_value, eprime)
    beta2 = math.atan2(
        sodano_direct_sin_beta2(beta1, forward_azimuth.radians, phi),
        sodano_direct_cos_beta2(beta1, forward_azimuth.radians, phi),
    )
    latitude = sodano_direct_latitude(beta2, equatorial, polar)
    correction = sodano_direct_l_correction(
        beta1,
        forward_azimuth.radians,
        phi,
        xi,
        parameters.flattening,
        m_value,
        n_value,
    )
    longitude = sodano_direct_longitude(
        start.longitude.radians,
        beta1,
        forward_azimuth.radians,
        phi,
        correction,
        beta2,
    )
    destination = GeodeticCoordinates(
        Longitude(longitude).wrapped(),
        Latitude(latitude),
        Quantity(start.altitude.to(parameters.equatorial_radius.unit).value, parameters.equatorial_radius.unit),
    )
    backward = sodano_direct_backward_azimuth(beta1, forward_azimuth.radians, phi)
    return SodanoDirectResult(destination, Angle(backward))
####
