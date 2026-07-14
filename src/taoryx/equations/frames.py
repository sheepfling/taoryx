"""Coordinate-frame transforms and body-attitude helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .geodesy import CartesianVector3, geocentric_unit_vectors, geodetic_unit_vectors

FrameBasis = tuple[CartesianVector3, CartesianVector3, CartesianVector3]


@dataclass(frozen=True, slots=True)
class EulerAngles:
    """Yaw, pitch, and roll angles in radians."""

    yaw_radians: float
    pitch_radians: float
    roll_radians: float
####


@dataclass(frozen=True, slots=True)
class BodyAxes:
    """Body-axis unit vectors expressed in the chosen reference frame."""

    x: CartesianVector3
    y: CartesianVector3
    z: CartesianVector3
####


@dataclass(frozen=True, slots=True)
class WindAxes:
    """Wind-axis unit vectors expressed in the chosen reference frame."""

    x: CartesianVector3
    y: CartesianVector3
    z: CartesianVector3
####


@dataclass(frozen=True, slots=True)
class AerodynamicAngles:
    """Aerodynamic angles derived from wind and body axes."""

    alpha_radians: float
    beta_e_radians: float
    beta_radians: float
    total_alpha_radians: float
    windward_meridian_radians: float
####


def _dot(left: CartesianVector3, right: CartesianVector3) -> float:
    return left.x * right.x + left.y * right.y + left.z * right.z
####


def _cross(left: CartesianVector3, right: CartesianVector3) -> CartesianVector3:
    return CartesianVector3(
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    )
####


def _compose_from_basis(
    basis: FrameBasis,
    components: CartesianVector3,
) -> CartesianVector3:
    x_axis, y_axis, z_axis = basis
    return CartesianVector3(
        components.x * x_axis.x + components.y * y_axis.x + components.z * z_axis.x,
        components.x * x_axis.y + components.y * y_axis.y + components.z * z_axis.y,
        components.x * x_axis.z + components.y * y_axis.z + components.z * z_axis.z,
    )
####


def _decompose_to_basis(
    basis: FrameBasis,
    vector: CartesianVector3,
) -> CartesianVector3:
    x_axis, y_axis, z_axis = basis
    return CartesianVector3(
        _dot(vector, x_axis),
        _dot(vector, y_axis),
        _dot(vector, z_axis),
    )
####


def earth_rotation_rate() -> float:
    """Return the nominal TAOS Earth rotation rate in radians per second."""

    return 7.2921150e-5
####


def ecic_rotation_angle(
    initial_angle_radians: float,
    rotation_rate_radians_per_second: float,
    time_seconds: float,
    reference_time_seconds: float,
) -> float:
    """Return the ECIC/ECFC rotation angle from TAOS equation 2-1."""

    return initial_angle_radians + rotation_rate_radians_per_second * (time_seconds - reference_time_seconds)
####


def ecfc_to_ecic_position(position: CartesianVector3, rotation_angle_radians: float) -> CartesianVector3:
    """Rotate an ECFC position into the ECIC frame."""

    cosine = math.cos(rotation_angle_radians)
    sine = math.sin(rotation_angle_radians)
    return CartesianVector3(
        position.x * cosine - position.y * sine,
        position.x * sine + position.y * cosine,
        position.z,
    )
####


def ecfc_to_ecic_velocity(
    position: CartesianVector3,
    velocity: CartesianVector3,
    rotation_angle_radians: float,
    rotation_rate_radians_per_second: float,
) -> CartesianVector3:
    """Rotate an ECFC velocity into the ECIC frame, including transport terms."""

    cosine = math.cos(rotation_angle_radians)
    sine = math.sin(rotation_angle_radians)
    return CartesianVector3(
        velocity.x * cosine
        - velocity.y * sine
        - rotation_rate_radians_per_second * (position.x * sine + position.y * cosine),
        velocity.x * sine
        + velocity.y * cosine
        + rotation_rate_radians_per_second * (position.x * cosine - position.y * sine),
        velocity.z,
    )
####


def ecfc_to_ecic_acceleration(force: CartesianVector3, mass: float, rotation_angle_radians: float) -> CartesianVector3:
    """Project an ECFC force into ECIC acceleration components."""

    if mass == 0.0:
        raise ValueError("mass must be nonzero")
    ####
    return ecfc_to_ecic_position(CartesianVector3(force.x / mass, force.y / mass, force.z / mass), rotation_angle_radians)
####


def initial_ecic_rotation(rotation_rate_radians_per_second: float, time_seconds: float, reference_time_seconds: float, initial_rotation_radians: float) -> float:
    """Return the initial ECIC rotation angle."""

    return ecic_rotation_angle(initial_rotation_radians, rotation_rate_radians_per_second, time_seconds, reference_time_seconds)
####


def wind_corrected_velocity(vehicle_velocity: CartesianVector3, wind_velocity: CartesianVector3) -> CartesianVector3:
    """Return the wind-corrected air-relative velocity vector."""

    return CartesianVector3(
        vehicle_velocity.x - wind_velocity.x,
        vehicle_velocity.y - wind_velocity.y,
        vehicle_velocity.z - wind_velocity.z,
    )
####


def wind_unit_vectors_from_velocity(
    velocity_vector: CartesianVector3,
    *,
    geodetic_up: CartesianVector3,
    bank_radians: float = 0.0,
) -> WindAxes:
    """Return wind axes from a velocity vector and a geodetic up axis."""

    speed = math.sqrt(velocity_vector.x * velocity_vector.x + velocity_vector.y * velocity_vector.y + velocity_vector.z * velocity_vector.z)
    if speed == 0.0:
        raise ValueError("wind axes are undefined at zero speed")
    ####
    x_axis = CartesianVector3(velocity_vector.x / speed, velocity_vector.y / speed, velocity_vector.z / speed)
    up_cross = _cross(geodetic_up, x_axis)
    cross_norm = math.sqrt(up_cross.x * up_cross.x + up_cross.y * up_cross.y + up_cross.z * up_cross.z)
    if cross_norm == 0.0:
        raise ValueError("wind axes are undefined when velocity is parallel to the geodetic up axis")
    ####
    y_velocity = CartesianVector3(up_cross.x / cross_norm, up_cross.y / cross_norm, up_cross.z / cross_norm)
    z_velocity = _cross(x_axis, y_velocity)
    cosine_bank = math.cos(bank_radians)
    sine_bank = math.sin(bank_radians)
    y_axis = CartesianVector3(
        cosine_bank * y_velocity.x + sine_bank * z_velocity.x,
        cosine_bank * y_velocity.y + sine_bank * z_velocity.y,
        cosine_bank * y_velocity.z + sine_bank * z_velocity.z,
    )
    z_axis = CartesianVector3(
        -sine_bank * y_velocity.x + cosine_bank * z_velocity.x,
        -sine_bank * y_velocity.y + cosine_bank * z_velocity.y,
        -sine_bank * y_velocity.z + cosine_bank * z_velocity.z,
    )
    return WindAxes(x_axis, y_axis, z_axis)
####


def geodetic_velocity_axes_from_angles(
    geodetic_basis: FrameBasis,
    speed: float,
    gamma_radians: float,
    psi_radians: float,
) -> FrameBasis:
    """Return the geodetic velocity axes from speed and flight-path angles."""

    _ = speed
    x_axis = _compose_from_basis(
        geodetic_basis,
        CartesianVector3(
            math.cos(gamma_radians) * math.cos(psi_radians),
            math.cos(gamma_radians) * math.sin(psi_radians),
            -math.sin(gamma_radians),
        ),
    )
    y_axis = _compose_from_basis(
        geodetic_basis,
        CartesianVector3(-math.sin(psi_radians), math.cos(psi_radians), 0.0),
    )
    z_axis = _compose_from_basis(
        geodetic_basis,
        CartesianVector3(
            math.sin(gamma_radians) * math.cos(psi_radians),
            math.sin(gamma_radians) * math.sin(psi_radians),
            math.cos(gamma_radians),
        ),
    )
    return x_axis, y_axis, z_axis
####


def wind_axes_from_bank_angle(velocity_axes: FrameBasis, bank_radians: float) -> WindAxes:
    """Rotate velocity axes into wind axes with a bank angle."""

    x_axis, y_axis, z_axis = velocity_axes
    cosine_bank = math.cos(bank_radians)
    sine_bank = math.sin(bank_radians)
    return WindAxes(
        x_axis,
        CartesianVector3(
            cosine_bank * y_axis.x + sine_bank * z_axis.x,
            cosine_bank * y_axis.y + sine_bank * z_axis.y,
            cosine_bank * y_axis.z + sine_bank * z_axis.z,
        ),
        CartesianVector3(
            -sine_bank * y_axis.x + cosine_bank * z_axis.x,
            -sine_bank * y_axis.y + cosine_bank * z_axis.y,
            -sine_bank * y_axis.z + cosine_bank * z_axis.z,
        ),
    )
####


def wind_body_axes_from_aerodynamic_angles(
    wind_axes: WindAxes,
    alpha_radians: float,
    beta_e_radians: float,
) -> BodyAxes:
    """Return body axes from wind axes and aerodynamic angles."""

    cosine_alpha = math.cos(alpha_radians)
    sine_alpha = math.sin(alpha_radians)
    cosine_beta = math.cos(beta_e_radians)
    sine_beta = math.sin(beta_e_radians)
    x_axis, y_axis, z_axis = wind_axes.x, wind_axes.y, wind_axes.z
    body_x = _compose_from_basis(
        (x_axis, y_axis, z_axis),
        CartesianVector3(
            cosine_alpha * cosine_beta,
            -cosine_alpha * sine_beta,
            -sine_alpha,
        ),
    )
    body_y = _compose_from_basis(
        (x_axis, y_axis, z_axis),
        CartesianVector3(sine_beta, cosine_beta, 0.0),
    )
    body_z = _compose_from_basis(
        (x_axis, y_axis, z_axis),
        CartesianVector3(
            sine_alpha * cosine_beta,
            -sine_alpha * sine_beta,
            cosine_alpha,
        ),
    )
    return BodyAxes(body_x, body_y, body_z)
####


def aerodynamic_angles_from_body_axes(
    wind_axes: WindAxes,
    body_axes: BodyAxes,
) -> AerodynamicAngles:
    """Recover aerodynamic angles from wind and body axes."""

    x_projection = _decompose_to_basis((wind_axes.x, wind_axes.y, wind_axes.z), body_axes.x)
    beta_e = math.atan2(_dot(body_axes.y, wind_axes.x), _dot(body_axes.y, wind_axes.y))
    alpha = math.atan2(-x_projection.z, math.hypot(x_projection.x, x_projection.y))
    total_alpha = math.atan2(math.hypot(_dot(body_axes.x, wind_axes.y), _dot(body_axes.x, wind_axes.z)), _dot(body_axes.x, wind_axes.x))
    if math.isclose(x_projection.x, 0.0, abs_tol=1e-12):
        beta = math.atan2(_dot(body_axes.y, wind_axes.x), _dot(body_axes.z, wind_axes.x))
    else:
        beta = math.atan2(_dot(body_axes.y, wind_axes.x), x_projection.x)
    windward_meridian = math.atan2(-_dot(body_axes.x, wind_axes.y), _dot(body_axes.x, wind_axes.z))
    if math.isclose(abs(alpha), math.pi / 2.0, abs_tol=1e-12):
        windward_meridian = -beta
    return AerodynamicAngles(alpha, beta_e, beta, total_alpha, windward_meridian)
####




def geodetic_body_axes_from_euler_angles(
    longitude_radians: float,
    latitude_radians: float,
    yaw_radians: float,
    pitch_radians: float,
    roll_radians: float,
) -> BodyAxes:
    """Return body axes from geodetic yaw, pitch, and roll angles."""

    return body_axes_from_euler_angles(
        geodetic_unit_vectors(longitude_radians, latitude_radians),
        yaw_radians,
        pitch_radians,
        roll_radians,
    )
####


def geocentric_body_axes_from_euler_angles(
    longitude_radians: float,
    latitude_radians: float,
    yaw_radians: float,
    pitch_radians: float,
    roll_radians: float,
) -> BodyAxes:
    """Return body axes from geocentric yaw, pitch, and roll angles."""

    return body_axes_from_euler_angles(
        geocentric_unit_vectors(longitude_radians, latitude_radians),
        yaw_radians,
        pitch_radians,
        roll_radians,
    )
####


def body_axes_from_euler_angles(
    reference_basis: FrameBasis,
    yaw_radians: float,
    pitch_radians: float,
    roll_radians: float,
) -> BodyAxes:
    """Return body-axis unit vectors for a yaw-pitch-roll rotation sequence."""

    cosine_yaw = math.cos(yaw_radians)
    sine_yaw = math.sin(yaw_radians)
    cosine_pitch = math.cos(pitch_radians)
    sine_pitch = math.sin(pitch_radians)
    cosine_roll = math.cos(roll_radians)
    sine_roll = math.sin(roll_radians)

    body_x = _compose_from_basis(
        reference_basis,
        CartesianVector3(
            cosine_pitch * cosine_yaw,
            cosine_pitch * sine_yaw,
            -sine_pitch,
        ),
    )
    body_y = _compose_from_basis(
        reference_basis,
        CartesianVector3(
            sine_roll * sine_pitch * cosine_yaw - cosine_roll * sine_yaw,
            sine_roll * sine_pitch * sine_yaw + cosine_roll * cosine_yaw,
            sine_roll * cosine_pitch,
        ),
    )
    body_z = _compose_from_basis(
        reference_basis,
        CartesianVector3(
            cosine_roll * sine_pitch * cosine_yaw + sine_roll * sine_yaw,
            cosine_roll * sine_pitch * sine_yaw - sine_roll * cosine_yaw,
            cosine_roll * cosine_pitch,
        ),
    )
    return BodyAxes(body_x, body_y, body_z)
####


def temporary_yaw_x_axis(reference_basis: FrameBasis, yaw_radians: float) -> CartesianVector3:
    """Return the intermediate x-axis used while recovering body Euler angles."""

    cosine_yaw = math.cos(yaw_radians)
    sine_yaw = math.sin(yaw_radians)
    return _compose_from_basis(reference_basis, CartesianVector3(cosine_yaw, sine_yaw, 0.0))
####


def temporary_yaw_y_axis(reference_basis: FrameBasis, yaw_radians: float) -> CartesianVector3:
    """Return the intermediate y-axis used while recovering body Euler angles."""

    _, _, z_axis = reference_basis
    return _cross(z_axis, temporary_yaw_x_axis(reference_basis, yaw_radians))
####


def euler_angles_from_body_axes(reference_basis: FrameBasis, body_axes: BodyAxes) -> EulerAngles:
    """Recover yaw, pitch, and roll from body axes."""

    _, _, z_axis = reference_basis
    x_projection = _decompose_to_basis(reference_basis, body_axes.x)
    horizontal_projection = math.hypot(x_projection.x, x_projection.y)
    if math.isclose(horizontal_projection, 0.0, abs_tol=1e-12):
        pitch = math.pi * 0.5 * x_projection.z
        return EulerAngles(0.0, pitch, 0.0)
    ####
    yaw = math.atan2(x_projection.y, x_projection.x)
    pitch = math.asin(-x_projection.z)
    roll = math.atan2(_dot(body_axes.y, z_axis), _dot(body_axes.z, z_axis))
    ####
    return EulerAngles(yaw, pitch, roll)
####


def geodetic_euler_angles_from_body_axes(
    longitude_radians: float,
    latitude_radians: float,
    body_axes: BodyAxes,
) -> EulerAngles:
    """Recover geodetic yaw, pitch, and roll from body axes."""

    return euler_angles_from_body_axes(geodetic_unit_vectors(longitude_radians, latitude_radians), body_axes)
####


def geocentric_euler_angles_from_body_axes(
    longitude_radians: float,
    latitude_radians: float,
    body_axes: BodyAxes,
) -> EulerAngles:
    """Recover geocentric yaw, pitch, and roll from body axes."""

    return euler_angles_from_body_axes(geocentric_unit_vectors(longitude_radians, latitude_radians), body_axes)
####


def ecfc_force_to_body(force: CartesianVector3, body_axes: BodyAxes) -> CartesianVector3:
    """Project an ECFC force vector into body coordinates."""

    return _decompose_to_basis((body_axes.x, body_axes.y, body_axes.z), force)
####


def body_force_to_ecfc(force: CartesianVector3, body_axes: BodyAxes) -> CartesianVector3:
    """Project a body-coordinate force vector back into ECFC coordinates."""

    return _compose_from_basis((body_axes.x, body_axes.y, body_axes.z), force)
####
