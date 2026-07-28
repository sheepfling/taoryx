from __future__ import annotations

import math

import pytest

from taoryx.contracts import EarthModel, Frame, FrameVector3, Latitude, Longitude, Quantity, Unit, Vector3
from taoryx.rigid_body_frames import EarthOperatingPoint, EarthRelativeVelocityStateAdapter, EarthRotationAdapter


def _earth() -> EarthModel:
    return EarthModel(
        equatorial_radius=Quantity(6378.137, Unit.KILOMETER),
        flattening=1.0 / 298.257223563,
        gravitational_parameter=Quantity(3.986004418e14, Unit.METER_CUBED_PER_SECOND_SQUARED),
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


def test_operating_point_converts_local_velocity_with_earth_transport() -> None:
    operating_point = EarthOperatingPoint(
        _earth(),
        longitude=Longitude(0.0),
        latitude=Latitude(0.0),
        altitude_m=0.0,
    )

    inertial = operating_point.inertial_velocity_from_local_ecfc(Vector3(0.0, 0.0, 0.0))

    assert inertial.frame is Frame.ECIC
    assert inertial.vector.x == pytest.approx(0.0)
    assert inertial.vector.y == pytest.approx(7.2921150e-5 * 6_378_137.0)
    assert operating_point.to_metadata()["latitude_rad"] == pytest.approx(0.0)
    ####


def test_operating_point_reuses_one_local_trim_across_latitudes() -> None:
    equator = EarthOperatingPoint(_earth(), Longitude(0.0), Latitude(0.0), 100.0)
    mid_latitude = EarthOperatingPoint(_earth(), Longitude(0.0), Latitude(0.7), 100.0)

    equator_speed = equator.earth_transport_velocity_ecfc().vector.norm()
    mid_latitude_speed = mid_latitude.earth_transport_velocity_ecfc().vector.norm()

    assert mid_latitude_speed < equator_speed
    assert mid_latitude_speed == pytest.approx(equator_speed * math.cos(0.7), rel=2.0e-3)
    ####


def test_operating_point_latitude_sensitivity_marks_reuse_by_declared_tolerance() -> None:
    operating_point = EarthOperatingPoint(_earth(), Longitude(0.0), Latitude(0.0), 100.0)

    samples = operating_point.latitude_sensitivity((Latitude(0.0), Latitude(0.7)), gravity_tolerance_fraction=0.01)

    assert samples[0]["reuse_local_trim"] is True
    assert samples[1]["reuse_local_trim"] is True
    assert samples[1]["earth_transport_speed_mps"] < samples[0]["earth_transport_speed_mps"]
    ####


def test_operating_point_converts_inertial_velocity_back_to_local_channels() -> None:
    operating_point = EarthOperatingPoint(_earth(), Longitude(0.0), Latitude(0.0), 0.0)
    inertial = operating_point.inertial_velocity_from_local_ecfc(Vector3(12.0, 3.0, -1.0))

    local = operating_point.earth_relative_velocity_from_inertial_ecic(inertial.vector)

    assert local.frame is Frame.ECFC
    assert local.vector.x == pytest.approx(12.0)
    assert local.vector.y == pytest.approx(3.0)
    assert local.vector.z == pytest.approx(-1.0)
    assert operating_point.earth_rotation_angular_rate_ecic().vector.z == pytest.approx(7.2921150e-5)
    ####


def test_controller_state_adapter_removes_earth_transport() -> None:
    operating_point = EarthOperatingPoint(_earth(), Longitude(0.0), Latitude(0.0), 0.0)
    adapter = EarthRelativeVelocityStateAdapter(
        EarthRotationAdapter(_earth()),
        position_names=("px", "py", "pz"),
        velocity_names=("vx", "vy", "vz"),
        time_name="t",
    )
    inertial = operating_point.inertial_velocity_from_local_ecfc(Vector3(12.0, 3.0, -1.0))

    state = adapter(
        {
            "px": 6_378_137.0,
            "py": 0.0,
            "pz": 0.0,
            "vx": inertial.vector.x,
            "vy": inertial.vector.y,
            "vz": inertial.vector.z,
            "t": 0.0,
        }
    )

    assert state["vx"] == pytest.approx(12.0)
    assert state["vy"] == pytest.approx(3.0)
    assert state["vz"] == pytest.approx(-1.0)
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
