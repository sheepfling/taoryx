from __future__ import annotations

import math

import pytest

from taoryx.contracts import Angle, Basis3, Frame, Vector3
from taoryx.forces import (
    evaluate_aerodynamic_forces,
    force_from_axial_normal_coefficients,
    force_from_body_coefficients,
    force_from_wind_coefficients,
    propulsive_force,
)
from taoryx.tables import prepare_table


def _wind_basis() -> Basis3:
    return Basis3(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), Vector3(0.0, 0.0, 1.0), Frame.ECFC, Frame.WIND)


def _body_basis() -> Basis3:
    return Basis3(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), Vector3(0.0, 0.0, 1.0), Frame.ECFC, Frame.BODY)


def _rotated_body_basis() -> Basis3:
    return Basis3(Vector3(0.0, 0.0, 1.0), Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), Frame.ECFC, Frame.BODY)


def test_force_conversions_preserve_named_parent_frame_and_scale() -> None:
    wind = _wind_basis()
    body = _body_basis()

    axial_normal = force_from_axial_normal_coefficients(wind, _rotated_body_basis(), 2.0, 3.0, 0.5, 0.25)
    wind_force = force_from_wind_coefficients(wind, 2.0, 3.0, 0.5, 0.25, 0.1)
    body_force = force_from_body_coefficients(body, 2.0, 3.0, 0.5, 0.25, 0.1)

    assert axial_normal.frame is Frame.ECFC
    assert axial_normal.vector == Vector3(-1.5, 0.0, -3.0)
    assert (wind_force.vector.x, wind_force.vector.y, wind_force.vector.z) == pytest.approx((-1.5, 0.6, -3.0))
    assert (body_force.vector.x, body_force.vector.y, body_force.vector.z) == pytest.approx((3.0, 1.5, 0.6))
####


def test_axial_normal_conversion_rejects_zero_meridian() -> None:
    with pytest.raises(ValueError, match="meridian"):
        force_from_axial_normal_coefficients(_wind_basis(), _body_basis(), 1.0, 1.0, 0.0, 1.0)
####


def test_propulsive_force_uses_body_axis_direction_convention() -> None:
    result = propulsive_force(100.0, Angle(math.radians(20.0)), Angle(math.radians(-15.0)))

    assert result.frame is Frame.BODY
    assert result.vector.x == pytest.approx(100.0 * math.cos(math.radians(20.0)))
    assert result.vector.y == pytest.approx(-100.0 * math.sin(math.radians(20.0)) * math.cos(math.radians(-15.0)))
    assert result.vector.z == pytest.approx(-100.0 * math.sin(math.radians(20.0)) * math.sin(math.radians(-15.0)))
####


def test_evaluate_aerodynamic_forces_interpolates_and_accumulates_wind_tables() -> None:
    table = prepare_table(((0.0, 1.0),), (0.0, 1.0))
    result = evaluate_aerodynamic_forces(
        _wind_basis(),
        _body_basis(),
        2.0,
        3.0,
        {"cl": table, "cd": table, "cs": table},
        (0.5,),
    )

    assert result.vector == Vector3(-3.0, 3.0, -3.0)
####
