from __future__ import annotations

import math

import pytest

from taoryx.equations import (
    AerodynamicAngles,
    BodyAxes,
    CartesianVector3,
    aerodynamic_angles_from_body_axes,
    body_force_to_ecfc,
    earth_rotation_rate,
    ecfc_force_to_body,
    ecfc_to_ecic_acceleration,
    ecfc_to_ecic_position,
    ecfc_to_ecic_velocity,
    ecic_rotation_angle,
    geodetic_body_axes_from_euler_angles,
    geodetic_euler_angles_from_body_axes,
    geodetic_velocity_axes_from_angles,
    wind_axes_from_bank_angle,
    wind_body_axes_from_aerodynamic_angles,
    wind_corrected_velocity,
    wind_unit_vectors_from_velocity,
)


def test_ecic_rotation_and_frame_transforms_follow_manual_formulas() -> None:
    position = CartesianVector3(10.0, -4.0, 3.0)
    velocity = CartesianVector3(2.0, 5.0, -1.0)
    force = CartesianVector3(12.0, -8.0, 4.0)
    angle = ecic_rotation_angle(0.25, earth_rotation_rate(), 200.0, 20.0)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    rotation_rate = earth_rotation_rate()

    rotated_position = ecfc_to_ecic_position(position, angle)
    rotated_velocity = ecfc_to_ecic_velocity(position, velocity, angle, rotation_rate)
    rotated_acceleration = ecfc_to_ecic_acceleration(force, 2.0, angle)

    assert rotated_position == CartesianVector3(
        position.x * cosine - position.y * sine,
        position.x * sine + position.y * cosine,
        position.z,
    )
    assert rotated_velocity == CartesianVector3(
        velocity.x * cosine - velocity.y * sine - rotation_rate * (position.x * sine + position.y * cosine),
        velocity.x * sine + velocity.y * cosine + rotation_rate * (position.x * cosine - position.y * sine),
        velocity.z,
    )
    assert rotated_acceleration == CartesianVector3(
        (force.x * cosine - force.y * sine) / 2.0,
        (force.x * sine + force.y * cosine) / 2.0,
        force.z / 2.0,
    )
####


def test_geodetic_body_axes_round_trip_through_euler_angles() -> None:
    longitude = math.radians(-63.0)
    latitude = math.radians(41.0)
    yaw = math.radians(23.0)
    pitch = math.radians(-17.0)
    roll = math.radians(12.0)

    body_axes = geodetic_body_axes_from_euler_angles(longitude, latitude, yaw, pitch, roll)
    recovered = geodetic_euler_angles_from_body_axes(longitude, latitude, body_axes)

    assert recovered.yaw_radians == pytest.approx(yaw)
    assert recovered.pitch_radians == pytest.approx(pitch)
    assert recovered.roll_radians == pytest.approx(roll)
####


def test_geodetic_body_axes_are_orthonormal() -> None:
    body_axes = geodetic_body_axes_from_euler_angles(
        math.radians(12.0),
        math.radians(-33.0),
        math.radians(15.0),
        math.radians(20.0),
        math.radians(-10.0),
    )

    def dot(left: CartesianVector3, right: CartesianVector3) -> float:
        return left.x * right.x + left.y * right.y + left.z * right.z

    vectors = [body_axes.x, body_axes.y, body_axes.z]
    for vector in vectors:
        assert dot(vector, vector) == pytest.approx(1.0)
    assert dot(vectors[0], vectors[1]) == pytest.approx(0.0)
    assert dot(vectors[0], vectors[2]) == pytest.approx(0.0)
    assert dot(vectors[1], vectors[2]) == pytest.approx(0.0)
####


def test_geodetic_pole_convention_sets_yaw_and_roll_to_zero() -> None:
    longitude = math.radians(8.0)
    latitude = math.radians(44.0)
    body_axes = geodetic_body_axes_from_euler_angles(
        longitude,
        latitude,
        0.0,
        math.pi / 2,
        0.0,
    )

    recovered = geodetic_euler_angles_from_body_axes(longitude, latitude, body_axes)

    assert recovered.yaw_radians == pytest.approx(0.0)
    assert recovered.pitch_radians == pytest.approx(-math.pi / 2)
    assert recovered.roll_radians == pytest.approx(0.0)
####


def test_force_projection_round_trips_through_body_axes() -> None:
    body_axes = BodyAxes(
        CartesianVector3(0.0, 1.0, 0.0),
        CartesianVector3(-1.0, 0.0, 0.0),
        CartesianVector3(0.0, 0.0, 1.0),
    )
    force = CartesianVector3(3.0, -4.0, 5.0)

    body_force = ecfc_force_to_body(force, body_axes)
    assert body_force == CartesianVector3(-4.0, -3.0, 5.0)
    assert body_force_to_ecfc(body_force, body_axes) == force
####


def test_wind_corrected_velocity_and_wind_axes_follow_manual_formulas() -> None:
    vehicle_velocity = CartesianVector3(300.0, -20.0, 15.0)
    wind_velocity = CartesianVector3(12.0, 3.0, -5.0)
    corrected_velocity = wind_corrected_velocity(vehicle_velocity, wind_velocity)

    assert corrected_velocity == CartesianVector3(288.0, -23.0, 20.0)

    geodetic_up = CartesianVector3(0.0, 0.0, 1.0)
    wind_axes = wind_unit_vectors_from_velocity(corrected_velocity, geodetic_up=geodetic_up, bank_radians=0.0)

    speed = math.sqrt(288.0 * 288.0 + 23.0 * 23.0 + 20.0 * 20.0)
    assert wind_axes.x == CartesianVector3(288.0 / speed, -23.0 / speed, 20.0 / speed)
    assert math.isclose(wind_axes.y.x * wind_axes.x.x + wind_axes.y.y * wind_axes.x.y + wind_axes.y.z * wind_axes.x.z, 0.0, abs_tol=1e-12)
    assert math.isclose(wind_axes.z.x * wind_axes.x.x + wind_axes.z.y * wind_axes.x.y + wind_axes.z.z * wind_axes.x.z, 0.0, abs_tol=1e-12)
####


def test_geodetic_velocity_axes_from_angles_match_the_manual_expansion() -> None:
    geodetic_basis = (
        CartesianVector3(1.0, 0.0, 0.0),
        CartesianVector3(0.0, 1.0, 0.0),
        CartesianVector3(0.0, 0.0, 1.0),
    )

    x_axis, y_axis, z_axis = geodetic_velocity_axes_from_angles(
        geodetic_basis,
        1.0,
        math.radians(12.0),
        math.radians(-33.0),
    )

    assert x_axis == CartesianVector3(
        math.cos(math.radians(12.0)) * math.cos(math.radians(-33.0)),
        math.cos(math.radians(12.0)) * math.sin(math.radians(-33.0)),
        -math.sin(math.radians(12.0)),
    )
    assert y_axis == CartesianVector3(
        -math.sin(math.radians(-33.0)),
        math.cos(math.radians(-33.0)),
        0.0,
    )
    assert z_axis == CartesianVector3(
        math.sin(math.radians(12.0)) * math.cos(math.radians(-33.0)),
        math.sin(math.radians(12.0)) * math.sin(math.radians(-33.0)),
        math.cos(math.radians(12.0)),
    )
####


def test_aerodynamic_angle_round_trip_handles_forward_and_90_degree_alpha_cases() -> None:
    wind_axes = wind_axes_from_bank_angle(
        (
            CartesianVector3(1.0, 0.0, 0.0),
            CartesianVector3(0.0, 1.0, 0.0),
            CartesianVector3(0.0, 0.0, 1.0),
        ),
        math.radians(20.0),
    )
    target_angles = AerodynamicAngles(
        alpha_radians=math.radians(18.0),
        beta_e_radians=math.radians(-11.0),
        beta_radians=0.0,
        total_alpha_radians=0.0,
        windward_meridian_radians=0.0,
    )

    body_axes = wind_body_axes_from_aerodynamic_angles(
        wind_axes,
        target_angles.alpha_radians,
        target_angles.beta_e_radians,
    )
    recovered = aerodynamic_angles_from_body_axes(wind_axes, body_axes)

    assert recovered.alpha_radians == pytest.approx(target_angles.alpha_radians)
    assert recovered.beta_e_radians == pytest.approx(target_angles.beta_e_radians)
    assert recovered.beta_radians == pytest.approx(
        math.atan2(
            math.sin(target_angles.beta_e_radians),
            math.cos(target_angles.alpha_radians) * math.cos(target_angles.beta_e_radians),
        )
    )
    assert recovered.total_alpha_radians == pytest.approx(math.atan2(
        math.hypot(
            math.sin(target_angles.beta_e_radians),
            math.sin(target_angles.alpha_radians) * math.cos(target_angles.beta_e_radians),
        ),
        math.cos(target_angles.alpha_radians) * math.cos(target_angles.beta_e_radians),
    ))

    ninety_degree_body_axes = wind_body_axes_from_aerodynamic_angles(wind_axes, math.pi / 2, math.radians(30.0))
    ninety_recovered = aerodynamic_angles_from_body_axes(wind_axes, ninety_degree_body_axes)
    assert ninety_recovered.beta_radians == pytest.approx(math.radians(30.0))
####
