"""Tests for the explicit bounded direct-wrench fidelity seam."""

from __future__ import annotations

import pytest

from taoryx.contracts import Vector3
from taoryx.direct_wrench import DirectWrenchLimits, compose_direct_wrench_load
from taoryx.rigid_body import RigidBodyForceMoment


def _limits() -> DirectWrenchLimits:
    names = (
        "force_x_n",
        "force_y_n",
        "force_z_n",
        "moment_x_nm",
        "moment_y_nm",
        "moment_z_nm",
    )
    return DirectWrenchLimits(
        lower={name: -10.0 for name in names},
        upper={name: 10.0 for name in names},
        rate_limit_per_s={name: None for name in names},
    )
####


def _wrench(value: float) -> dict[str, float]:
    return {name: value for name in _limits().axes}
####


def test_direct_wrench_projection_is_explicit_and_bounded() -> None:
    projection = _limits().project(_wrench(25.0), _wrench(0.0), 0.1)

    assert projection.status == "partially_achievable"
    assert projection.position_saturated == _limits().axes
    assert projection.rate_limited == ()
    assert projection.residual_norm == pytest.approx((6 * 15.0**2) ** 0.5)
    assert projection.as_dict()["physical_effector_allocation"] is False
    ####


def test_direct_wrench_rate_limit_is_reported() -> None:
    limits = DirectWrenchLimits(
        lower={name: -10.0 for name in _limits().axes},
        upper={name: 10.0 for name in _limits().axes},
        rate_limit_per_s={name: 2.0 for name in _limits().axes},
    )
    projection = limits.project(_wrench(10.0), _wrench(0.0), 0.5)

    assert projection.status == "partially_achievable"
    assert projection.rate_limited == limits.axes
    assert set(projection.achieved.values()) == {1.0}
    ####


def test_direct_wrench_composition_preserves_source_loads_and_provenance() -> None:
    base = RigidBodyForceMoment(
        force_body=Vector3(1.0, 2.0, 3.0),
        moment_body=Vector3(4.0, 5.0, 6.0),
        aero_force_body=Vector3(1.0, 2.0, 3.0),
        aero_moment_body=Vector3(4.0, 5.0, 6.0),
    )
    projection = _limits().project(_wrench(1.0), _wrench(0.0), 0.1)
    result = compose_direct_wrench_load(base, projection)

    assert result.force_body == Vector3(2.0, 3.0, 4.0)
    assert result.moment_body == Vector3(5.0, 6.0, 7.0)
    assert result.aero_force_body == base.aero_force_body
    assert result.control_force_body == Vector3(1.0, 1.0, 1.0)
    assert result.control_moment_body == Vector3(1.0, 1.0, 1.0)
    ####
