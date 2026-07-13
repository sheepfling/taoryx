from __future__ import annotations

import math

import pytest

from taoryx.attitude import (
    AerodynamicAngles,
    EulerAngles,
    body_basis_to_euler_angles,
    body_basis_to_wind_aerodynamic,
    euler_angles_to_body_basis,
    wind_to_body_aerodynamic,
)
from taoryx.contracts import Angle, Basis3, Frame, Latitude, Longitude, Vector3
from taoryx.coordinates import geodetic_unit_vectors


def test_euler_body_basis_round_trip_preserves_rotation_sequence() -> None:
    reference = geodetic_unit_vectors(Longitude(0.8), Latitude(-0.4))
    source = EulerAngles(Angle(0.7), Angle(-0.3), Angle(1.1))
    body = euler_angles_to_body_basis(reference, source)
    recovered = body_basis_to_euler_angles(body, reference)

    assert recovered.yaw.radians == pytest.approx(source.yaw.radians)
    assert recovered.pitch.radians == pytest.approx(source.pitch.radians)
    assert recovered.roll.radians == pytest.approx(source.roll.radians)
    assert body.parent_frame is Frame.ECFC
    assert body.child_frame is Frame.BODY
    assert body.is_orthonormal()
####


@pytest.mark.parametrize("pitch", [math.pi / 2.0, -math.pi / 2.0])
def test_body_basis_inverse_uses_manual_pole_convention(pitch: float) -> None:
    reference = geodetic_unit_vectors(Longitude(0.0), Latitude(0.0))
    body = euler_angles_to_body_basis(reference, EulerAngles(Angle(0.9), Angle(pitch), Angle(-1.2)))
    recovered = body_basis_to_euler_angles(body, reference)

    assert recovered.yaw.radians == 0.0
    assert recovered.roll.radians == 0.0
    assert recovered.pitch.radians == pytest.approx(pitch)
####


def test_attitude_transforms_reject_invalid_frames_and_bases() -> None:
    reference = geodetic_unit_vectors(Longitude(0.0), Latitude(0.0))
    body = euler_angles_to_body_basis(reference, EulerAngles(Angle(0.0), Angle(0.0), Angle(0.0)))
    wrong_reference = type(reference)(reference.first, reference.second, reference.third, Frame.ECIC, Frame.GEODETIC_HORIZON)

    with pytest.raises(ValueError, match="reference basis"):
        euler_angles_to_body_basis(type(reference)(reference.first, reference.second, reference.third, Frame.ECFC, Frame.BODY), EulerAngles(Angle(0.0), Angle(0.0), Angle(0.0)))
    with pytest.raises(ValueError, match="BODY basis"):
        body_basis_to_euler_angles(body, wrong_reference)
    with pytest.raises(ValueError, match="singularity_tolerance"):
        body_basis_to_euler_angles(body, reference, singularity_tolerance=0.0)
####


def test_wind_body_aerodynamic_transform_round_trips_angles() -> None:
    reference = geodetic_unit_vectors(Longitude(0.8), Latitude(-0.4))
    wind = Basis3(reference.first, reference.second, reference.third, Frame.ECFC, Frame.WIND)
    body = wind_to_body_aerodynamic(wind, Angle(0.35), Angle(-0.22))
    recovered = body_basis_to_wind_aerodynamic(body, wind)

    assert recovered.angle_of_attack.radians == pytest.approx(0.35)
    assert recovered.euler_sideslip.radians == pytest.approx(-0.22)
    assert body.is_orthonormal()
####


def test_wind_body_aerodynamic_transform_preserves_full_angle_ranges() -> None:
    wind = Basis3(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), Vector3(0.0, 0.0, 1.0), Frame.ECFC, Frame.WIND)
    source = AerodynamicAngles(Angle(2.4), Angle(-1.7))
    recovered = body_basis_to_wind_aerodynamic(wind_to_body_aerodynamic(wind, source.angle_of_attack, source.euler_sideslip), wind)

    assert recovered.angle_of_attack.radians == pytest.approx(source.angle_of_attack.radians)
    assert recovered.euler_sideslip.radians == pytest.approx(source.euler_sideslip.radians)
####
