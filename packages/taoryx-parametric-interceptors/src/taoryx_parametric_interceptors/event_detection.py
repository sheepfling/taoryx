"""Shared within-step event localization for interceptor translation kernels."""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Literal

import numpy as np

TranslationEventKind = Literal["objective_capture", "ground_contact"]


@dataclass(frozen=True, slots=True)
class TranslationStepEvent:
    """Earliest terminal or mission event on one held-acceleration step."""

    kind: TranslationEventKind
    time_from_step_start_s: float

    ####


def objective_is_captured(range_m: float, capture_radius_m: float) -> bool:
    """Return capture status with a small scale-aware roundoff allowance."""

    if not math.isfinite(range_m) or range_m < 0.0:
        raise ValueError("objective range must be finite and nonnegative")
    if not math.isfinite(capture_radius_m) or capture_radius_m <= 0.0:
        raise ValueError("capture radius must be finite and positive")
    return range_m <= capture_radius_m + max(1.0e-9, capture_radius_m * 1.0e-10)
    ####


def first_capture_time_within_step(
    *,
    relative_position_m: tuple[float, float, float],
    relative_velocity_mps: tuple[float, float, float],
    interceptor_acceleration_mps2: tuple[float, float, float],
    capture_radius_m: float,
    duration_s: float,
) -> float | None:
    """Locate the first capture-volume intersection on a semi-implicit step.

    The kernels hold interceptor acceleration over one accepted step and use
    ``x(t) = x0 + (v0 + a*t)*t``. A constant-velocity target therefore has the
    relative path ``r(t) = r0 + v_rel*t - a*t**2``. The squared range is a
    quartic. Its derivative partitions the step into monotone intervals, after
    which deterministic bisection finds the first entry without cadence-sized
    sampling holes.
    """

    _validate_step_inputs(
        relative_position_m,
        relative_velocity_mps,
        interceptor_acceleration_mps2,
        capture_radius_m,
        duration_s,
    )
    quadratic = (
        -interceptor_acceleration_mps2[0],
        -interceptor_acceleration_mps2[1],
        -interceptor_acceleration_mps2[2],
    )
    coefficients = _squared_range_coefficients(
        relative_position_m,
        relative_velocity_mps,
        quadratic,
        capture_radius_m,
    )
    start_value = _polynomial_value(coefficients, 0.0)
    tolerance = max(1.0e-12, capture_radius_m**2 * 1.0e-12)
    if start_value <= tolerance:
        return 0.0

    derivative_descending = np.asarray(
        (4.0 * coefficients[4], 3.0 * coefficients[3], 2.0 * coefficients[2], coefficients[1]),
        dtype=float,
    )
    critical = _real_roots_in_interval(
        derivative_descending,
        lower=0.0,
        upper=duration_s,
    )
    boundaries = _deduplicate_sorted((0.0, *critical, duration_s))
    for left, right in zip(boundaries, boundaries[1:]):
        left_value = _polynomial_value(coefficients, left)
        right_value = _polynomial_value(coefficients, right)
        if left_value <= tolerance:
            return left
        if right_value > tolerance:
            continue
        if right_value >= 0.0:
            return right
        return _bisect_first_nonpositive(coefficients, left, right)
    return None
    ####


def first_ground_contact_time_within_step(
    *,
    altitude_m: float,
    vertical_velocity_mps: float,
    vertical_acceleration_mps2: float,
    duration_s: float,
) -> float | None:
    """Locate the first descending altitude-floor contact on the same step."""

    values = (altitude_m, vertical_velocity_mps, vertical_acceleration_mps2, duration_s)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("ground-contact localization inputs must be finite")
    if duration_s <= 0.0:
        raise ValueError("ground-contact duration must be positive")
    if altitude_m < 0.0 or (altitude_m <= 1.0e-12 and vertical_velocity_mps < 0.0):
        return 0.0
    roots = _real_roots_in_interval(
        np.asarray((vertical_acceleration_mps2, vertical_velocity_mps, altitude_m), dtype=float),
        lower=0.0,
        upper=duration_s,
    )
    for root in roots:
        if root <= 1.0e-12:
            continue
        committed_vertical_velocity = vertical_velocity_mps + vertical_acceleration_mps2 * root
        if committed_vertical_velocity < 0.0:
            return root
    return None
    ####


def localize_translation_step_event(
    *,
    relative_position_m: tuple[float, float, float],
    relative_velocity_mps: tuple[float, float, float],
    interceptor_acceleration_mps2: tuple[float, float, float],
    capture_radius_m: float,
    altitude_m: float,
    vertical_velocity_mps: float,
    duration_s: float,
) -> TranslationStepEvent | None:
    """Return the earliest capture or descending ground contact on one step."""

    capture_time = first_capture_time_within_step(
        relative_position_m=relative_position_m,
        relative_velocity_mps=relative_velocity_mps,
        interceptor_acceleration_mps2=interceptor_acceleration_mps2,
        capture_radius_m=capture_radius_m,
        duration_s=duration_s,
    )
    ground_time = first_ground_contact_time_within_step(
        altitude_m=altitude_m,
        vertical_velocity_mps=vertical_velocity_mps,
        vertical_acceleration_mps2=interceptor_acceleration_mps2[2],
        duration_s=duration_s,
    )
    if ground_time is not None and (capture_time is None or ground_time <= capture_time + 1.0e-12):
        return TranslationStepEvent("ground_contact", ground_time)
    if capture_time is not None:
        return TranslationStepEvent("objective_capture", capture_time)
    return None
    ####


def _validate_step_inputs(
    relative_position_m: tuple[float, float, float],
    relative_velocity_mps: tuple[float, float, float],
    interceptor_acceleration_mps2: tuple[float, float, float],
    capture_radius_m: float,
    duration_s: float,
) -> None:
    values = (
        *relative_position_m,
        *relative_velocity_mps,
        *interceptor_acceleration_mps2,
        capture_radius_m,
        duration_s,
    )
    if any(not math.isfinite(value) for value in values):
        raise ValueError("capture localization inputs must be finite")
    if capture_radius_m <= 0.0 or duration_s <= 0.0:
        raise ValueError("capture radius and step duration must be positive")
    ####


def _squared_range_coefficients(
    relative_position: tuple[float, float, float],
    relative_velocity: tuple[float, float, float],
    relative_quadratic: tuple[float, float, float],
    capture_radius_m: float,
) -> tuple[float, float, float, float, float]:
    return (
        _dot(relative_position, relative_position) - capture_radius_m**2,
        2.0 * _dot(relative_position, relative_velocity),
        _dot(relative_velocity, relative_velocity) + 2.0 * _dot(relative_position, relative_quadratic),
        2.0 * _dot(relative_velocity, relative_quadratic),
        _dot(relative_quadratic, relative_quadratic),
    )
    ####


def _real_roots_in_interval(
    coefficients_descending: np.ndarray,
    *,
    lower: float,
    upper: float,
) -> tuple[float, ...]:
    scale = max(float(np.max(np.abs(coefficients_descending))), 1.0)
    first = 0
    while first < len(coefficients_descending) - 1 and abs(coefficients_descending[first]) <= scale * 1.0e-14:
        first += 1
    polynomial = coefficients_descending[first:]
    if len(polynomial) <= 1:
        return ()
    roots = np.roots(polynomial)
    real = (
        min(max(float(root.real), lower), upper)
        for root in roots
        if abs(float(root.imag)) <= 1.0e-9 * max(1.0, abs(float(root.real))) and lower - 1.0e-12 <= float(root.real) <= upper + 1.0e-12
    )
    return _deduplicate_sorted(real)
    ####


def _deduplicate_sorted(values: Iterable[float]) -> tuple[float, ...]:
    ordered = sorted(float(value) for value in values)
    result: list[float] = []
    for value in ordered:
        if not result or abs(value - result[-1]) > 1.0e-10 * max(1.0, abs(value), abs(result[-1])):
            result.append(value)
    return tuple(result)
    ####


def _dot(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    return sum(a * b for a, b in zip(left, right, strict=True))
    ####


def _polynomial_value(coefficients_ascending: tuple[float, ...], time_s: float) -> float:
    value = 0.0
    for coefficient in reversed(coefficients_ascending):
        value = value * time_s + coefficient
    return value
    ####


def _bisect_first_nonpositive(
    coefficients: tuple[float, ...],
    lower: float,
    upper: float,
) -> float:
    low = lower
    high = upper
    for _ in range(80):
        midpoint = 0.5 * (low + high)
        if _polynomial_value(coefficients, midpoint) > 0.0:
            low = midpoint
        else:
            high = midpoint
    return high
    ####


__all__ = [
    "TranslationEventKind",
    "TranslationStepEvent",
    "first_capture_time_within_step",
    "first_ground_contact_time_within_step",
    "localize_translation_step_event",
    "objective_is_captured",
]
####
