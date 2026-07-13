"""Gravity equation helpers."""

from __future__ import annotations

import math
from collections.abc import Mapping
from functools import lru_cache

from .geodesy import CartesianVector3


def gravity_acceleration_from_geopotential_gradient(gradient: CartesianVector3) -> CartesianVector3:
    """Return the gravity acceleration vector from the geopotential gradient."""

    return gradient
####


def point_mass_geopotential(gravitational_parameter: float, radius: float) -> float:
    """Return the point-mass geopotential term."""

    return gravitational_parameter / radius
####


def _double_factorial(value: int) -> float:
    if value <= 0:
        return 1.0
    ####
    product = 1.0
    for factor in range(value, 0, -2):
        product *= factor
    ####
    return product
####


@lru_cache(maxsize=None)
def _associated_legendre_cached(degree: int, order: int, sine_latitude: float) -> float:
    if order < 0 or degree < 0 or order > degree:
        return 0.0
    ####
    x = sine_latitude
    p_mm = 1.0
    if order > 0:
        p_mm = ((-1.0) ** order) * _double_factorial(2 * order - 1) * (1.0 - x * x) ** (order / 2.0)
    ####
    if degree == order:
        return p_mm
    ####
    p_m1m = x * (2 * order + 1) * p_mm
    if degree == order + 1:
        return p_m1m
    ####
    p_prev_prev = p_mm
    p_prev = p_m1m
    for n in range(order + 2, degree + 1):
        p_curr = ((2 * n - 1) * x * p_prev - (n + order - 1) * p_prev_prev) / (n - order)
        p_prev_prev, p_prev = p_prev, p_curr
    ####
    return p_prev
####


def associated_legendre_function(degree: int, order: int, sine_latitude: float) -> float:
    """Return the unnormalized associated Legendre function."""

    return _associated_legendre_cached(degree, order, sine_latitude)
####


def legendre_order_zero_function(degree: int, sine_latitude: float) -> float:
    """Return the order-zero Legendre function."""

    return associated_legendre_function(degree, 0, sine_latitude)
####


def legendre_derivative_notation(degree: int, order: int, sine_latitude: float) -> float:
    """Return the derivative notation used for the Legendre functions."""

    x = sine_latitude
    p_nm = associated_legendre_function(degree, order, x)
    p_nminus1m = associated_legendre_function(degree - 1, order, x)
    denominator = x * x - 1.0
    if math.isclose(denominator, 0.0):
        return 0.0
    ####
    return (degree * x * p_nm - (degree + order) * p_nminus1m) / denominator
####


def first_four_legendre_functions(sine_latitude: float) -> tuple[float, float, float, float]:
    """Return the first four order-zero Legendre functions."""

    p1 = legendre_order_zero_function(1, sine_latitude)
    p2 = legendre_order_zero_function(2, sine_latitude)
    p3 = legendre_order_zero_function(3, sine_latitude)
    p4 = legendre_order_zero_function(4, sine_latitude)
    return p1, p2, p3, p4
####


def normalized_legendre_function(degree: int, order: int, legendre_value: float) -> float:
    """Return the normalized Legendre function."""

    normalization = math.sqrt(
        (2.0 - (1.0 if order == 0 else 0.0))
        * (2 * degree + 1)
        * math.factorial(degree - order)
        / math.factorial(degree + order)
    )
    return normalization * legendre_value
####


def normalized_gravity_coefficient(degree: int, order: int, coefficient: float) -> float:
    """Return the normalized spherical-harmonic coefficient."""

    normalization = math.sqrt(
        1.0
        / (
            (2.0 - (1.0 if order == 0 else 0.0))
            * (2 * degree + 1)
            * math.factorial(degree - order)
            / math.factorial(degree + order)
        )
    )
    return normalization * coefficient
####


def geopotential_spherical_harmonics(
    gravitational_parameter: float,
    reference_radius: float,
    radius: float,
    latitude_radians: float,
    longitude_radians: float,
    coefficients: Mapping[tuple[int, int], tuple[float, float]],
) -> float:
    """Return the geopotential spherical-harmonic expansion."""

    sine_latitude = math.sin(latitude_radians)
    total = 1.0
    for (degree, order), (c_coefficient, s_coefficient) in coefficients.items():
        factor = (reference_radius / radius) ** degree
        legendre = associated_legendre_function(degree, order, sine_latitude)
        total += factor * legendre * (
            c_coefficient * math.cos(order * longitude_radians)
            + s_coefficient * math.sin(order * longitude_radians)
        )
    ####
    return gravitational_parameter / radius * total
####


def zonal_geopotential_from_c_coefficients(
    gravitational_parameter: float,
    reference_radius: float,
    radius: float,
    latitude_radians: float,
    c_coefficients: Mapping[int, float],
) -> float:
    """Return the zonal geopotential using C coefficients."""

    sine_latitude = math.sin(latitude_radians)
    total = 1.0
    for degree, c_coefficient in c_coefficients.items():
        total += (reference_radius / radius) ** degree * legendre_order_zero_function(degree, sine_latitude) * c_coefficient
    ####
    return gravitational_parameter / radius * total
####


def zonal_geopotential_from_j_coefficients(
    gravitational_parameter: float,
    reference_radius: float,
    radius: float,
    latitude_radians: float,
    j_coefficients: Mapping[int, float],
) -> float:
    """Return the zonal geopotential using J coefficients."""

    sine_latitude = math.sin(latitude_radians)
    total = 1.0
    for degree, j_coefficient in j_coefficients.items():
        total -= (reference_radius / radius) ** degree * legendre_order_zero_function(degree, sine_latitude) * j_coefficient
    ####
    return gravitational_parameter / radius * total
####


def gravity_geocentric_x_gradient(
    radius: float,
    latitude_radians: float,
    partial_latitude_derivative: float,
) -> float:
    """Return the north-component acceleration from the geopotential gradient."""

    return partial_latitude_derivative / radius
####


def gravity_geocentric_y_gradient(
    radius: float,
    latitude_radians: float,
    partial_longitude_derivative: float,
) -> float:
    """Return the east-component acceleration from the geopotential gradient."""

    cosine_latitude = math.cos(latitude_radians)
    if math.isclose(cosine_latitude, 0.0):
        return 0.0
    ####
    return partial_longitude_derivative / (radius * cosine_latitude)
####


def gravity_geocentric_z_gradient(partial_radius_derivative: float) -> float:
    """Return the inward-component acceleration from the geopotential gradient."""

    return -partial_radius_derivative
####


def _spherical_harmonic_partials(
    gravitational_parameter: float,
    reference_radius: float,
    radius: float,
    latitude_radians: float,
    longitude_radians: float,
    coefficients: Mapping[tuple[int, int], tuple[float, float]],
) -> tuple[float, float, float]:
    sine_latitude = math.sin(latitude_radians)
    cosine_latitude = math.cos(latitude_radians)
    partial_latitude = 0.0
    partial_longitude = 0.0
    for (degree, order), (c_coefficient, s_coefficient) in coefficients.items():
        factor = (reference_radius / radius) ** degree
        legendre = associated_legendre_function(degree, order, sine_latitude)
        harmonic = c_coefficient * math.cos(order * longitude_radians) + s_coefficient * math.sin(order * longitude_radians)
        partial_longitude += factor * legendre * order * (
            -c_coefficient * math.sin(order * longitude_radians)
            + s_coefficient * math.cos(order * longitude_radians)
        )
        partial_latitude += factor * legendre_derivative_notation(degree, order, sine_latitude) * harmonic
    ####
    partial_latitude *= gravitational_parameter / radius * cosine_latitude
    partial_longitude *= gravitational_parameter / radius
    partial_radius = -gravitational_parameter / (radius * radius)
    for (degree, order), (c_coefficient, s_coefficient) in coefficients.items():
        factor = (reference_radius / radius) ** degree
        legendre = associated_legendre_function(degree, order, sine_latitude)
        harmonic = c_coefficient * math.cos(order * longitude_radians) + s_coefficient * math.sin(order * longitude_radians)
        partial_radius -= gravitational_parameter / (radius * radius) * (degree + 1) * factor * legendre * harmonic
    ####
    return partial_latitude, partial_longitude, partial_radius
####


def gravity_full_geocentric_components(
    gravitational_parameter: float,
    reference_radius: float,
    radius: float,
    latitude_radians: float,
    longitude_radians: float,
    coefficients: Mapping[tuple[int, int], tuple[float, float]],
) -> CartesianVector3:
    """Return the full spherical-harmonic geocentric acceleration components."""

    partial_latitude, partial_longitude, partial_radius = _spherical_harmonic_partials(
        gravitational_parameter,
        reference_radius,
        radius,
        latitude_radians,
        longitude_radians,
        coefficients,
    )
    return CartesianVector3(
        gravity_geocentric_x_gradient(radius, latitude_radians, partial_latitude),
        gravity_geocentric_y_gradient(radius, latitude_radians, partial_longitude),
        gravity_geocentric_z_gradient(partial_radius),
    )
####


def gravity_j2_geocentric_components(
    gravitational_parameter: float,
    reference_radius: float,
    radius: float,
    latitude_radians: float,
    j2_coefficient: float,
) -> CartesianVector3:
    """Return the J2-only geocentric acceleration components."""

    sine_latitude = math.sin(latitude_radians)
    cosine_latitude = math.cos(latitude_radians)
    radial_factor = (reference_radius / radius) ** 2
    north = -gravitational_parameter / (radius * radius) * radial_factor * (3.0 * j2_coefficient * sine_latitude * cosine_latitude)
    inward = gravitational_parameter / (radius * radius) * (
        1.0 - radial_factor * 1.5 * j2_coefficient * (3.0 * sine_latitude * sine_latitude - 1.0)
    )
    return CartesianVector3(north, 0.0, inward)
####
