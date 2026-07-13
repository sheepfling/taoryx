from __future__ import annotations

import math

import pytest

from taoryx.contracts import Basis3, EarthModel, Frame, FrameVector3, Quantity, Unit, Vector3
from taoryx.dynamics import (
    AugmentedTrajectoryState,
    ConstraintMode,
    apply_rail_constraint,
    assemble_state_derivatives,
    combine_acceleration_contributions,
    earth_fixed_derivatives,
    specific_load_factors,
)


def _earth(rotation_rate: float = 2.0) -> EarthModel:
    return EarthModel(
        equatorial_radius=Quantity(6378.137, Unit.KILOMETER),
        flattening=1.0 / 298.257223563,
        gravitational_parameter=Quantity(398600.4418, Unit.METER_CUBED_PER_SECOND_SQUARED),
        rotation_rate=Quantity(rotation_rate, Unit.RADIAN_PER_SECOND),
    )


def test_earth_fixed_derivatives_include_force_coriolis_and_centrifugal_terms() -> None:
    position = FrameVector3(Vector3(3.0, 4.0, 5.0), Frame.ECFC)
    velocity = FrameVector3(Vector3(7.0, 11.0, 13.0), Frame.ECFC)
    force = FrameVector3(Vector3(20.0, 30.0, 40.0), Frame.ECFC)
    derivatives = earth_fixed_derivatives(position, velocity, force, 2.0, _earth())

    assert derivatives.position_derivative.vector == velocity.vector
    assert derivatives.velocity_derivative.vector.x == pytest.approx(10.0 + 44.0 + 12.0)
    assert derivatives.velocity_derivative.vector.y == pytest.approx(15.0 - 28.0 + 16.0)
    assert derivatives.velocity_derivative.vector.z == pytest.approx(20.0)
    assert derivatives.position_derivative.frame is Frame.ECFC
    assert derivatives.velocity_derivative.frame is Frame.ECFC
####


def test_earth_fixed_derivatives_reduce_to_force_acceleration_without_rotation() -> None:
    derivatives = earth_fixed_derivatives(
        FrameVector3(Vector3(1.0, 2.0, 3.0), Frame.ECFC),
        FrameVector3(Vector3(4.0, 5.0, 6.0), Frame.ECFC),
        FrameVector3(Vector3(8.0, 10.0, 12.0), Frame.ECFC),
        2.0,
        _earth(0.0),
    )

    assert derivatives.velocity_derivative.vector == Vector3(4.0, 5.0, 6.0)
####


def test_earth_fixed_derivatives_reject_wrong_frames_and_invalid_mass() -> None:
    ecfc = FrameVector3(Vector3(1.0, 0.0, 0.0), Frame.ECFC)
    ecic = FrameVector3(Vector3(1.0, 0.0, 0.0), Frame.ECIC)
    with pytest.raises(ValueError, match="position must be expressed"):
        earth_fixed_derivatives(ecic, ecfc, ecfc, 1.0, _earth())
    with pytest.raises(ValueError, match="mass"):
        earth_fixed_derivatives(ecfc, ecfc, ecfc, 0.0, _earth())
####


def test_augmented_state_derivative_assembly_preserves_documented_order() -> None:
    state = AugmentedTrajectoryState(
        FrameVector3(Vector3(3.0, 4.0, 5.0), Frame.ECFC),
        FrameVector3(Vector3(6.0, 8.0, 0.0), Frame.ECFC),
        10.0,
        120.0,
        40.0,
    )
    derivatives = assemble_state_derivatives(
        state,
        FrameVector3(Vector3(10.0, 20.0, 30.0), Frame.ECFC),
        -0.5,
        7.5,
        _earth(0.0),
    )

    assert derivatives.position_derivative.vector == state.earth_relative_velocity.vector
    assert derivatives.mass_rate == -0.5
    assert derivatives.path_length_rate == pytest.approx(10.0)
    assert derivatives.ground_range_rate == 7.5
####


def test_augmented_state_contract_rejects_invalid_integrals_and_rates() -> None:
    with pytest.raises(ValueError, match="mass"):
        AugmentedTrajectoryState(
            FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
            FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
            0.0,
            0.0,
            0.0,
        )
    state = AugmentedTrajectoryState(
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        1.0,
        0.0,
        0.0,
    )
    with pytest.raises(ValueError, match="ground_speed"):
        assemble_state_derivatives(state, FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC), 0.0, -1.0, _earth())
####


def test_rail_constraint_selects_friction_and_projects_to_body_x() -> None:
    body_basis = Basis3(
        Vector3(1.0, 0.0, 0.0),
        Vector3(0.0, 1.0, 0.0),
        Vector3(0.0, 0.0, 1.0),
        Frame.ECFC,
        Frame.BODY,
    )
    result = apply_rail_constraint(
        FrameVector3(Vector3(10.0, 3.0, 4.0), Frame.ECFC),
        body_basis,
        speed=0.01,
        static_friction_coefficient=0.5,
        kinetic_friction_coefficient=0.2,
    )

    assert result.normal_acceleration == pytest.approx(5.0)
    assert result.friction_coefficient == pytest.approx(0.2)
    assert result.friction_acceleration == pytest.approx(1.0)
    assert result.rail_acceleration_magnitude == pytest.approx(9.0)
    assert result.acceleration.vector == Vector3(9.0, 0.0, 0.0)
####


def test_rail_clamps_backward_motion_but_sled_preserves_signed_acceleration() -> None:
    body_basis = Basis3(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), Vector3(0.0, 0.0, 1.0), Frame.ECFC, Frame.BODY)
    acceleration = FrameVector3(Vector3(-2.0, 0.0, 0.0), Frame.ECFC)
    rail = apply_rail_constraint(acceleration, body_basis, 0.0, 0.1, 0.2, mode=ConstraintMode.RAIL)
    sled = apply_rail_constraint(acceleration, body_basis, 0.0, 0.1, 0.2, mode=ConstraintMode.SLED)

    assert rail.acceleration.vector == Vector3(0.0, 0.0, 0.0)
    assert sled.acceleration.vector == Vector3(-2.0, 0.0, 0.0)
    assert sled.friction_coefficient == pytest.approx(0.1)
####


def test_rail_constraint_rejects_invalid_frame_basis_and_coefficients() -> None:
    body_basis = Basis3(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), Vector3(0.0, 0.0, 1.0), Frame.ECFC, Frame.BODY)
    with pytest.raises(ValueError, match="ECFC"):
        apply_rail_constraint(FrameVector3(Vector3(1.0, 0.0, 0.0), Frame.ECIC), body_basis, 0.0, 0.1, 0.1)
    with pytest.raises(ValueError, match="coefficients"):
        apply_rail_constraint(FrameVector3(Vector3(1.0, 0.0, 0.0), Frame.ECFC), body_basis, 0.0, -0.1, 0.1)
####


def test_specific_load_factors_project_specific_acceleration_into_body_axes() -> None:
    body_basis = Basis3(
        Vector3(1.0, 0.0, 0.0),
        Vector3(0.0, 1.0, 0.0),
        Vector3(0.0, 0.0, 1.0),
        Frame.ECFC,
        Frame.BODY,
    )
    result = specific_load_factors(
        FrameVector3(Vector3(9.0, 8.0, 7.0), Frame.ECFC),
        FrameVector3(Vector3(1.0, 2.0, 3.0), Frame.ECFC),
        body_basis,
    )

    assert result.specific_acceleration.vector == Vector3(8.0, 6.0, 4.0)
    assert (result.body_x, result.body_y, result.body_z) == pytest.approx((8.0, 6.0, 4.0))
    assert result.total == pytest.approx(math.sqrt(116.0))
    assert result.normal == pytest.approx(math.sqrt(52.0))
####


def test_combine_acceleration_contributions_sums_forces_before_rotating_terms() -> None:
    result = combine_acceleration_contributions(
        FrameVector3(Vector3(2.0, 0.0, 0.0), Frame.ECFC),
        FrameVector3(Vector3(0.0, 3.0, 0.0), Frame.ECFC),
        FrameVector3(Vector3(0.0, 0.0, 4.0), Frame.ECFC),
        FrameVector3(Vector3(1.0, 2.0, 3.0), Frame.ECFC),
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        2.0,
        _earth(0.0),
    )

    assert result == FrameVector3(Vector3(1.0, 1.5, 2.0), Frame.ECFC)
####
