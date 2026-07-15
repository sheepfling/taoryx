from __future__ import annotations

import pytest

from taoryx.contracts import EarthModel, Frame, FrameVector3, Quantity, Unit, Vector3
from taoryx.rigid_body_frames import EarthRotationAdapter


def _earth() -> EarthModel:
    return EarthModel(
        equatorial_radius=Quantity(6378.137, Unit.KILOMETER),
        flattening=1.0 / 298.257223563,
        gravitational_parameter=Quantity(398600.4418, Unit.METER_CUBED_PER_SECOND_SQUARED),
        rotation_rate=Quantity(7.2921150e-5, Unit.RADIAN_PER_SECOND),
    )
    ####


def test_ecic_ecfc_kinematics_round_trip_preserves_state() -> None:
    adapter = EarthRotationAdapter(_earth(), initial_angle_radians=0.4, reference_time_seconds=10.0)
    position = FrameVector3(Vector3(6_300_000.0, -1_200_000.0, 2_100_000.0), Frame.ECIC)
    velocity = FrameVector3(Vector3(1200.0, 7300.0, -400.0), Frame.ECIC)

    ecfc_position, ecfc_velocity = adapter.ecic_to_ecfc(position, velocity, time_seconds=125.0)
    restored_position, restored_velocity = adapter.ecfc_to_ecic(ecfc_position, ecfc_velocity, time_seconds=125.0)

    assert restored_position.vector.x == pytest.approx(position.vector.x)
    assert restored_position.vector.y == pytest.approx(position.vector.y)
    assert restored_position.vector.z == pytest.approx(position.vector.z)
    assert restored_velocity.vector.x == pytest.approx(velocity.vector.x)
    assert restored_velocity.vector.y == pytest.approx(velocity.vector.y)
    assert restored_velocity.vector.z == pytest.approx(velocity.vector.z)
    ####


def test_stationary_ecfc_point_has_zero_air_relative_velocity_without_wind() -> None:
    adapter = EarthRotationAdapter(_earth())
    position_ecfc = FrameVector3(Vector3(6_378_137.0, 0.0, 0.0), Frame.ECFC)
    position_ecic, inertial_velocity = adapter.ecfc_to_ecic(
        position_ecfc,
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        time_seconds=500.0,
    )

    air_relative = adapter.air_relative_velocity_ecic(
        position_ecic,
        inertial_velocity,
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        time_seconds=500.0,
    )

    assert air_relative.vector.norm() == pytest.approx(0.0, abs=1e-9)
    ####


def test_adapter_rejects_mixed_frames() -> None:
    adapter = EarthRotationAdapter(_earth())
    with pytest.raises(ValueError, match="position must be expressed in ecic"):
        adapter.ecic_to_ecfc(
            FrameVector3(Vector3(1.0, 0.0, 0.0), Frame.ECFC),
            FrameVector3(Vector3(0.0, 1.0, 0.0), Frame.ECIC),
            time_seconds=0.0,
        )
    ####
