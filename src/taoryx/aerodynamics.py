"""Typed aerodynamic performance metrics and coefficient searches."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from .equations import ballistic_coefficient, lift_to_drag_ratio
from .searches import golden_section_minimize


@dataclass(frozen=True, slots=True)
class AerodynamicPerformanceMetrics:
    """Ballistic coefficient and lift-to-drag ratio at one coefficient state."""

    ballistic_coefficient: float
    lift_to_drag_ratio: float
####


@dataclass(frozen=True, slots=True)
class MaximumLiftToDragResult:
    """Angle and ratio returned by the bounded L/D search."""

    angle: float
    lift_to_drag_ratio: float
    iterations: int
####


def aerodynamic_performance_metrics(
    weight: float,
    lift_coefficient: float,
    drag_coefficient: float,
    reference_area: float,
) -> AerodynamicPerformanceMetrics:
    """Evaluate TAOS-ALG-AERO-001 and equations 2-271 through 2-272."""

    if not all(math.isfinite(value) for value in (weight, lift_coefficient, drag_coefficient, reference_area)):
        raise ValueError("aerodynamic metric inputs must be finite")
    if drag_coefficient == 0.0:
        raise ValueError("aerodynamic metrics are undefined at zero drag coefficient")
    if reference_area <= 0.0:
        raise ValueError("reference area must be positive")
    return AerodynamicPerformanceMetrics(
        ballistic_coefficient(weight, drag_coefficient, reference_area),
        lift_to_drag_ratio(lift_coefficient, drag_coefficient),
    )
####


def maximum_lift_to_drag(
    coefficient_function: Callable[[float], tuple[float, float]],
    angle_bounds: tuple[float, float],
    angle_tolerance: float,
) -> MaximumLiftToDragResult:
    """Evaluate TAOS-ALG-AERO-002 using bounded golden-section search."""

    def negative_ratio(angle: float) -> float:
        lift, drag = coefficient_function(angle)
        if not math.isfinite(lift) or not math.isfinite(drag) or drag == 0.0:
            raise ValueError("L/D search requires finite coefficients and nonzero drag")
        return -lift / drag
    ####

    result = golden_section_minimize(negative_ratio, angle_bounds, angle_tolerance)
    return MaximumLiftToDragResult(result.minimum, -result.value, result.iterations)
####
