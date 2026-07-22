from __future__ import annotations

import math

import pytest

from taoryx.contracts import Vector3
from taoryx.runtime.guidance_control import (
    AttitudeCommand,
    ControlOutput,
    CoordinatedTurnController,
    GuidanceDemand,
    TargetState,
    allocate_alpha_bank,
    limit_vector_norm,
)


def test_alpha_bank_allocator_tracks_unsaturated_transverse_demand() -> None:
    command = allocate_alpha_bank(
        Vector3(2.0, 3.0, 4.0),
        lift_acceleration_per_radian=10.0,
        maximum_angle_of_attack_radians=1.0,
    )

    assert command.angle_of_attack_radians == pytest.approx(0.5)
    assert command.bank_radians == pytest.approx(math.atan2(3.0, 4.0))
    assert (command.achieved_acceleration.x, command.achieved_acceleration.y, command.achieved_acceleration.z) == pytest.approx((2.0, 3.0, 4.0))
    assert command.residual_acceleration.norm() == pytest.approx(0.0)
    assert not command.saturated
    ####


def test_alpha_bank_allocator_reports_alpha_saturation_residual() -> None:
    command = allocate_alpha_bank(
        Vector3(0.0, 0.0, 20.0),
        lift_acceleration_per_radian=10.0,
        maximum_angle_of_attack_radians=0.5,
    )

    assert command.angle_of_attack_radians == pytest.approx(0.5)
    assert command.achieved_acceleration.z == pytest.approx(5.0)
    assert command.residual_acceleration.z == pytest.approx(15.0)
    assert command.saturated
    ####


def test_alpha_bank_allocator_applies_declared_bank_limit() -> None:
    command = allocate_alpha_bank(
        Vector3(0.0, 10.0, 0.0),
        lift_acceleration_per_radian=10.0,
        maximum_angle_of_attack_radians=1.0,
        maximum_bank_radians=math.radians(30.0),
    )

    assert command.bank_radians == pytest.approx(math.radians(30.0))
    assert command.saturated
    ####


def test_alpha_bank_allocator_rejects_invalid_limits() -> None:
    with pytest.raises(ValueError, match="lift slope"):
        allocate_alpha_bank(Vector3(0.0, 0.0, 1.0), lift_acceleration_per_radian=0.0, maximum_angle_of_attack_radians=0.5)
    with pytest.raises(ValueError, match="bank limit"):
        allocate_alpha_bank(Vector3(0.0, 0.0, 1.0), lift_acceleration_per_radian=1.0, maximum_angle_of_attack_radians=0.5, maximum_bank_radians=0.0)
    ####


def test_native_guidance_contracts_are_slotted_and_frame_bounded() -> None:
    target = TargetState(Vector3(1.0, 2.0, 3.0), Vector3(0.0, 1.0, 0.0))
    demand = GuidanceDemand(Vector3(0.0, 4.0, 5.0), target, navigation_gain=4.0, source="ca-hi")
    command = AttitudeCommand(0.1, 0.2)
    output = ControlOutput(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 1.0), demand.acceleration, demand.acceleration, Vector3(0.0, 0.0, 0.0))

    assert demand.target.frame == "ecic"
    assert command.bank_radians == pytest.approx(0.2)
    assert output.residual_acceleration.norm() == pytest.approx(0.0)
    assert getattr(target, "__slots__")
    ####


def test_coordinated_turn_controller_maps_errors_to_bounded_body_moment() -> None:
    controller = CoordinatedTurnController(heading_gain=4.0, bank_gain=6.0, heading_rate_damping=1.0, bank_rate_damping=2.0, maximum_moment=2.0)

    command = controller.command(heading_error=0.5, bank_error=-0.25, body_rates=Vector3(0.1, 0.0, -0.2))

    assert command.moment_body.norm() == pytest.approx(2.0)
    assert command.moment_body.y == pytest.approx(0.0)
    assert command.saturated
    ####


def test_limit_vector_norm_preserves_direction_and_handles_zero_limit() -> None:
    vector = limit_vector_norm(Vector3(3.0, 4.0, 0.0), 2.0)
    assert vector.norm() == pytest.approx(2.0)
    assert (vector.x, vector.y, vector.z) == pytest.approx((1.2, 1.6, 0.0))
    assert limit_vector_norm(Vector3(1.0, 2.0, 3.0), 0.0).norm() == pytest.approx(0.0)
    with pytest.raises(ValueError, match="finite and nonnegative"):
        limit_vector_norm(Vector3(1.0, 0.0, 0.0), -1.0)
    ####
