"""Small explicit unit boundary for interceptor catalogue authoring."""

from __future__ import annotations

import math
from typing import Self

from pydantic import BaseModel, ConfigDict, Field, model_validator


class InterceptorUnitConversion(BaseModel):
    """One auditable scalar conversion into a canonical interceptor unit."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    contract: str = "taoryx.parametric-interceptors.unit-conversion/v1"
    parameter_id: str
    source_value: float
    source_unit: str | None
    canonical_value: float
    canonical_unit: str
    scale_factor: float = Field(gt=0.0)
    converted: bool
    method: str

    @model_validator(mode="after")
    def validate_finite_values(self) -> Self:
        if any(not math.isfinite(value) for value in (self.source_value, self.canonical_value, self.scale_factor)):
            raise ValueError("interceptor unit-conversion values must be finite")
        return self
        ####

    ####


_CANONICAL_PARAMETER_UNITS: dict[str, str] = {
    "launch_mass_kg": "kg",
    "burnout_mass_kg": "kg",
    "propellant_fraction": "1",
    "effective_specific_impulse_s": "s",
    "length_m": "m",
    "body_diameter_m": "m",
    "wingspan_m": "m",
    "reference_area_m2": "m^2",
    "reference_length_m": "m",
    "slenderness_ratio": "1",
    "burn_time_s": "s",
    "first_pulse_burn_time_s": "s",
    "inter_pulse_coast_time_s": "s",
    "second_pulse_burn_time_s": "s",
    "second_pulse_thrust_ratio": "1",
    "second_pulse_propellant_fraction": "1",
    "nominal_thrust_n": "N",
    "thrust_scale": "1",
    "drag_scale": "1",
    "maneuverability_scale": "1",
    "guidance_time_constant_scale": "1",
    "navigation_constant": "1",
    "attitude_bandwidth_rad_s": "rad/s",
    "attitude_damping_ratio": "1",
    "max_body_rate_rad_s": "rad/s",
    "max_body_acceleration_rad_s2": "rad/s^2",
    "max_bank_angle_rad": "rad",
    "normal_force_coefficient_limit": "1",
    "max_thrust_vector_angle_rad": "rad",
    "maneuver_drag_factor": "1",
    "applicability_altitude_min_m": "m",
    "applicability_altitude_max_m": "m",
    "applicability_mach_min": "1",
    "applicability_mach_max": "1",
    "reported_max_speed_mps": "m/s",
    "reported_max_range_m": "m",
    "reported_max_altitude_m": "m",
}

_UNIT_FACTORS: dict[str, dict[str, float]] = {
    "kg": {
        "kg": 1.0,
        "g": 1.0e-3,
        "lb": 0.45359237,
        "oz": 0.028349523125,
    },
    "m": {
        "m": 1.0,
        "km": 1_000.0,
        "cm": 1.0e-2,
        "mm": 1.0e-3,
        "ft": 0.3048,
        "in": 0.0254,
        "nmi": 1_852.0,
    },
    "m^2": {
        "m^2": 1.0,
        "cm^2": 1.0e-4,
        "mm^2": 1.0e-6,
        "ft^2": 0.09290304,
        "in^2": 0.00064516,
    },
    "s": {
        "s": 1.0,
        "ms": 1.0e-3,
        "min": 60.0,
    },
    "m/s": {
        "m/s": 1.0,
        "km/h": 1.0 / 3.6,
        "mph": 0.44704,
        "ft/s": 0.3048,
        "kn": 0.5144444444444445,
    },
    "N": {
        "N": 1.0,
        "n": 1.0,
        "kN": 1_000.0,
        "kn": 1_000.0,
        "lbf": 4.4482216152605,
    },
    "rad": {
        "rad": 1.0,
        "deg": math.pi / 180.0,
    },
    "rad/s": {
        "rad/s": 1.0,
        "deg/s": math.pi / 180.0,
    },
    "rad/s^2": {
        "rad/s^2": 1.0,
        "deg/s^2": math.pi / 180.0,
    },
    "1": {
        "1": 1.0,
        "%": 0.01,
    },
}

_UNIT_ALIASES: dict[str, str] = {
    "kilogram": "kg",
    "kilograms": "kg",
    "gram": "g",
    "grams": "g",
    "lbm": "lb",
    "lbs": "lb",
    "pound": "lb",
    "pounds": "lb",
    "ounce": "oz",
    "ounces": "oz",
    "meter": "m",
    "meters": "m",
    "metre": "m",
    "metres": "m",
    "kilometer": "km",
    "kilometers": "km",
    "kilometre": "km",
    "kilometres": "km",
    "centimeter": "cm",
    "centimeters": "cm",
    "millimeter": "mm",
    "millimeters": "mm",
    "feet": "ft",
    "foot": "ft",
    "inch": "in",
    "inches": "in",
    "nauticalmile": "nmi",
    "nauticalmiles": "nmi",
    "sec": "s",
    "second": "s",
    "seconds": "s",
    "millisecond": "ms",
    "milliseconds": "ms",
    "minute": "min",
    "minutes": "min",
    "mps": "m/s",
    "kph": "km/h",
    "kmph": "km/h",
    "knot": "kn",
    "knots": "kn",
    "kt": "kn",
    "kts": "kn",
    "newton": "N",
    "newtons": "N",
    "kilonewton": "kN",
    "kilonewtons": "kN",
    "poundforce": "lbf",
    "poundsforce": "lbf",
    "radian": "rad",
    "radians": "rad",
    "degree": "deg",
    "degrees": "deg",
    "percent": "%",
    "percentage": "%",
    "ratio": "1",
    "dimensionless": "1",
}


def canonical_unit_for_interceptor_parameter(parameter_id: str) -> str | None:
    """Return the runtime unit for one public authoring parameter."""

    return _CANONICAL_PARAMETER_UNITS.get(parameter_id)
    ####


def supported_interceptor_units(parameter_id: str) -> tuple[str, ...]:
    """Return normalized source units accepted for one parameter."""

    canonical = _required_canonical_unit(parameter_id)
    return tuple(_UNIT_FACTORS[canonical])
    ####


def canonicalize_interceptor_value(
    parameter_id: str,
    value: float | int,
    unit: str | None,
) -> InterceptorUnitConversion:
    """Convert a finite source scalar while retaining its exact source form.

    A missing unit is interpreted as the parameter's canonical unit for
    compatibility with the compact profile format. Catalogue authors should
    provide source units whenever they are known.
    """

    if isinstance(value, bool):
        raise ValueError(f"numeric interceptor parameter {parameter_id!r} cannot use a boolean value")
    source_value = float(value)
    if not math.isfinite(source_value):
        raise ValueError(f"numeric interceptor parameter {parameter_id!r} must be finite")
    canonical = _required_canonical_unit(parameter_id)
    normalized = canonical if unit is None else _normalize_unit(unit)
    factors = _UNIT_FACTORS[canonical]
    try:
        factor = factors[normalized]
    except KeyError as error:
        raise ValueError(
            f"parameter {parameter_id!r} requires a unit compatible with {canonical!r}; accepted normalized units are {tuple(factors)!r}, got {unit!r}"
        ) from error
    converted = unit is not None and normalized != canonical
    method = f"multiply {normalized} by {factor:.17g} to obtain {canonical}" if converted else f"value already expressed in canonical unit {canonical}"
    if unit is None:
        method = f"unit omitted; value interpreted in canonical unit {canonical}"
    return InterceptorUnitConversion(
        parameter_id=parameter_id,
        source_value=source_value,
        source_unit=unit,
        canonical_value=source_value * factor,
        canonical_unit=canonical,
        scale_factor=factor,
        converted=converted,
        method=method,
    )
    ####


def _required_canonical_unit(parameter_id: str) -> str:
    try:
        return _CANONICAL_PARAMETER_UNITS[parameter_id]
    except KeyError as error:
        raise KeyError(f"interceptor parameter {parameter_id!r} has no registered canonical unit") from error
    ####


def _normalize_unit(unit: str) -> str:
    case_sensitive = unit.strip().replace("²", "^2").replace("·", "").replace(" ", "")
    if case_sensitive in {"N", "kN"}:
        return case_sensitive
    compact = case_sensitive.lower().replace("per", "/")
    compact = compact.replace("sec", "s") if compact in {"rad/sec", "deg/sec", "ft/sec"} else compact
    return _UNIT_ALIASES.get(compact, compact)
    ####


__all__ = [
    "InterceptorUnitConversion",
    "canonical_unit_for_interceptor_parameter",
    "canonicalize_interceptor_value",
    "supported_interceptor_units",
]
####
