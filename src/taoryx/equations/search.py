"""One-dimensional search equation helpers from the TAOS manual."""

from __future__ import annotations

import math


def newton_linearization(function_value: float, derivative_value: float, x_value: float, x_estimate: float) -> float:
    """Return the Newton linearization of a function."""

    return function_value + (x_value - x_estimate) * derivative_value
####


def newton_update_search(x_estimate: float, function_value: float, derivative_value: float) -> float:
    """Return the Newton--Raphson search update."""

    if derivative_value == 0.0:
        raise ValueError("derivative must be nonzero")
    ####
    return x_estimate - function_value / derivative_value
####


def secant_root_estimate(
    lower_x: float,
    lower_value: float,
    upper_x: float,
    upper_value: float,
) -> float:
    """Return the secant-method root estimate."""

    denominator = upper_value - lower_value
    if denominator == 0.0:
        raise ValueError("secant estimate is undefined when the function values are equal")
    ####
    return lower_x - lower_value * ((upper_x - lower_x) / denominator)
####


def parabolic_search_polynomial(x_value: float, coefficient_a: float, coefficient_b: float, coefficient_c: float) -> float:
    """Return the parabolic-search polynomial value."""

    return coefficient_a * x_value * x_value + coefficient_b * x_value + coefficient_c
####


def parabolic_search_b(
    x1: float,
    f1: float,
    x2: float,
    f2: float,
    x3: float,
    f3: float,
) -> float:
    """Return the parabolic-search coefficient b."""

    numerator = (x2 * x2 - x3 * x3) * (f1 - f3) - (x1 * x1 - x3 * x3) * (f2 - f3)
    denominator = (x2 * x2 - x3 * x3) * (x1 - x3) - (x1 * x1 - x3 * x3) * (x2 - x3)
    if denominator == 0.0:
        raise ValueError("parabolic fit is singular")
    ####
    return numerator / denominator
####


def parabolic_search_a(
    x2: float,
    f2: float,
    x3: float,
    f3: float,
    coefficient_b: float,
) -> float:
    """Return the parabolic-search coefficient a."""

    denominator = x2 * x2 - x3 * x3
    if denominator == 0.0:
        raise ValueError("parabolic fit is singular")
    ####
    return (f2 - f3 - coefficient_b * (x2 - x3)) / denominator
####


def parabolic_search_c(x2: float, f2: float, coefficient_a: float, coefficient_b: float) -> float:
    """Return the parabolic-search coefficient c."""

    return f2 - coefficient_a * x2 * x2 - coefficient_b * x2
####


def parabolic_search_roots(
    coefficient_a: float,
    coefficient_b: float,
    coefficient_c: float,
) -> tuple[float, float]:
    """Return the two roots of the fitted parabola."""

    if coefficient_a == 0.0:
        raise ValueError("parabolic roots are undefined when the quadratic term is zero")
    ####
    discriminant = coefficient_b * coefficient_b - 4.0 * coefficient_a * coefficient_c
    if discriminant < 0.0:
        raise ValueError("parabolic roots are complex")
    ####
    sqrt_discriminant = math.sqrt(discriminant)
    root1 = (-coefficient_b - sqrt_discriminant) / (2.0 * coefficient_a)
    root2 = (-coefficient_b + sqrt_discriminant) / (2.0 * coefficient_a)
    return (root1, root2) if root1 <= root2 else (root2, root1)
####


def golden_section_x1(x_lower: float, x_upper: float) -> float:
    """Return the first golden-section interior point."""

    alpha = (math.sqrt(5.0) - 1.0) / 2.0
    return x_upper - alpha * (x_upper - x_lower)
####


def golden_section_x2(x_lower: float, x_upper: float) -> float:
    """Return the second golden-section interior point."""

    alpha = (math.sqrt(5.0) - 1.0) / 2.0
    return x_lower + alpha * (x_upper - x_lower)
####


def parabolic_minimum(coefficient_a: float, coefficient_b: float) -> float:
    """Return the stationary point of the fitted parabola."""

    if coefficient_a == 0.0:
        raise ValueError("parabolic minimum is undefined when the quadratic term is zero")
    ####
    return -coefficient_b / (2.0 * coefficient_a)
####
