"""Frame-aware aerodynamic and propulsive force conversions."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

from .contracts import Angle, Basis3, Frame, FrameVector3, Vector3
from .tables import PreparedTable, interpolate_nd


def force_from_axial_normal_coefficients(
    wind_basis: Basis3,
    body_basis: Basis3,
    dynamic_pressure: float,
    reference_area: float,
    axial_coefficient: float,
    normal_coefficient: float,
) -> FrameVector3:
    """Evaluate TAOS-ALG-FORCE-001 and equations 2-196 through 2-197."""

    _require_aero_bases(wind_basis, body_basis)
    _require_scale(dynamic_pressure, reference_area)
    meridian = body_basis.first
    projection_y = wind_basis.first.dot(body_basis.second)
    projection_z = wind_basis.first.dot(body_basis.third)
    magnitude = math.hypot(projection_y, projection_z)
    if magnitude == 0.0:
        raise ValueError("windward meridian is undefined for zero total angle of attack")
    meridian = body_basis.second.scaled(projection_y / magnitude) + body_basis.third.scaled(projection_z / magnitude)
    force = body_basis.first.scaled(-axial_coefficient) + meridian.scaled(-normal_coefficient)
    return FrameVector3(force.scaled(dynamic_pressure * reference_area), wind_basis.parent_frame)
####


def force_from_wind_coefficients(
    wind_basis: Basis3,
    dynamic_pressure: float,
    reference_area: float,
    lift_coefficient: float,
    drag_coefficient: float,
    side_force_coefficient: float,
) -> FrameVector3:
    """Evaluate TAOS-ALG-FORCE-002 and equation 2-198."""

    _require_basis(wind_basis, Frame.WIND)
    _require_scale(dynamic_pressure, reference_area)
    force = (
        wind_basis.first.scaled(-drag_coefficient)
        + wind_basis.second.scaled(side_force_coefficient)
        + wind_basis.third.scaled(-lift_coefficient)
    )
    return FrameVector3(force.scaled(dynamic_pressure * reference_area), wind_basis.parent_frame)
####


def force_from_body_coefficients(
    body_basis: Basis3,
    dynamic_pressure: float,
    reference_area: float,
    x_coefficient: float,
    y_coefficient: float,
    z_coefficient: float,
) -> FrameVector3:
    """Evaluate TAOS-ALG-FORCE-003 and equation 2-199."""

    _require_basis(body_basis, Frame.BODY)
    _require_scale(dynamic_pressure, reference_area)
    force = (
        body_basis.first.scaled(x_coefficient)
        + body_basis.second.scaled(y_coefficient)
        + body_basis.third.scaled(z_coefficient)
    )
    return FrameVector3(force.scaled(dynamic_pressure * reference_area), body_basis.parent_frame)
####


def propulsive_force(
    thrust_magnitude: float,
    first_direction_angle: Angle,
    second_direction_angle: Angle,
) -> FrameVector3:
    """Evaluate TAOS-ALG-FORCE-005 and equation 2-200 in BODY axes."""

    if not math.isfinite(thrust_magnitude) or thrust_magnitude < 0.0:
        raise ValueError("thrust magnitude must be finite and non-negative")
    cosine_first = math.cos(first_direction_angle.radians)
    sine_first = math.sin(first_direction_angle.radians)
    vector = Vector3(
        thrust_magnitude * cosine_first,
        -thrust_magnitude * sine_first * math.cos(second_direction_angle.radians),
        -thrust_magnitude * sine_first * math.sin(second_direction_angle.radians),
    )
    return FrameVector3(vector, Frame.BODY)
####


def evaluate_aerodynamic_forces(
    wind_basis: Basis3,
    body_basis: Basis3,
    dynamic_pressure: float,
    reference_area: float,
    coefficient_tables: Mapping[str, PreparedTable],
    query: Sequence[float],
) -> FrameVector3:
    """Evaluate TAOS-ALG-FORCE-004 and accumulate active coefficient tables.

    Supported coefficient families are ``CA/CN``, ``CL/CD/CS``, and
    ``CX/CY/CZ``. Names are case-insensitive, and each complete family is
    evaluated and summed in the common parent frame.
    """

    _require_aero_bases(wind_basis, body_basis)
    _require_scale(dynamic_pressure, reference_area)
    tables = {name.casefold(): table for name, table in coefficient_tables.items()}
    total = FrameVector3(Vector3(0.0, 0.0, 0.0), wind_basis.parent_frame)
    if {"ca", "cn"}.issubset(tables):
        total = total.add(
            force_from_axial_normal_coefficients(
                wind_basis,
                body_basis,
                dynamic_pressure,
                reference_area,
                interpolate_nd(tables["ca"], query),
                interpolate_nd(tables["cn"], query),
            )
        )
    if {"cl", "cd", "cs"}.issubset(tables):
        total = total.add(
            force_from_wind_coefficients(
                wind_basis,
                dynamic_pressure,
                reference_area,
                interpolate_nd(tables["cl"], query),
                interpolate_nd(tables["cd"], query),
                interpolate_nd(tables["cs"], query),
            )
        )
    if {"cx", "cy", "cz"}.issubset(tables):
        total = total.add(
            force_from_body_coefficients(
                body_basis,
                dynamic_pressure,
                reference_area,
                interpolate_nd(tables["cx"], query),
                interpolate_nd(tables["cy"], query),
                interpolate_nd(tables["cz"], query),
            )
        )
    complete_families = (
        {"ca", "cn"}.issubset(tables),
        {"cl", "cd", "cs"}.issubset(tables),
        {"cx", "cy", "cz"}.issubset(tables),
    )
    if not any(complete_families):
        raise ValueError("coefficient tables do not contain a complete supported family")
    return total
####


def _require_aero_bases(wind_basis: Basis3, body_basis: Basis3) -> None:
    _require_basis(wind_basis, Frame.WIND)
    _require_basis(body_basis, Frame.BODY)
    if wind_basis.parent_frame is not body_basis.parent_frame:
        raise ValueError("wind and body bases must share a parent frame")
    ####
####


def _require_basis(basis: Basis3, child_frame: Frame) -> None:
    if basis.child_frame is not child_frame or not basis.is_orthonormal():
        raise ValueError(f"basis must be an orthonormal {child_frame} basis")
    ####
####


def _require_scale(dynamic_pressure: float, reference_area: float) -> None:
    if not math.isfinite(dynamic_pressure) or dynamic_pressure < 0.0:
        raise ValueError("dynamic pressure must be finite and non-negative")
    if not math.isfinite(reference_area) or reference_area < 0.0:
        raise ValueError("reference area must be finite and non-negative")
    ####
####
