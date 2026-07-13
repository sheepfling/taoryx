"""Typed attitude and Euler-angle transformations from the TAOS catalog."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .contracts import Angle, Basis3, Frame


@dataclass(frozen=True, slots=True)
class EulerAngles:
    """Named yaw, pitch, and roll angles in the TAOS rotation sequence."""

    yaw: Angle
    pitch: Angle
    roll: Angle


@dataclass(frozen=True, slots=True)
class AerodynamicAngles:
    """Angle of attack and Euler sideslip angle."""

    angle_of_attack: Angle
    euler_sideslip: Angle


def euler_angles_to_body_basis(reference_basis: Basis3, angles: EulerAngles) -> Basis3:
    """Construct body axes from reference-frame Euler angles.

    This implements ``TAOS-ALG-COORD-014`` and equations 2-50 through 2-53:
    yaw about reference z, pitch about intermediate y, and roll about
    intermediate x. The returned body basis is expressed in the reference
    parent frame.
    """

    _require_reference_basis(reference_basis)
    cosine_yaw = math.cos(angles.yaw.radians)
    sine_yaw = math.sin(angles.yaw.radians)
    cosine_pitch = math.cos(angles.pitch.radians)
    sine_pitch = math.sin(angles.pitch.radians)
    cosine_roll = math.cos(angles.roll.radians)
    sine_roll = math.sin(angles.roll.radians)
    x_axis = reference_basis.first
    y_axis = reference_basis.second
    z_axis = reference_basis.third
    body_x = (
        x_axis.scaled(cosine_pitch * cosine_yaw)
        + y_axis.scaled(cosine_pitch * sine_yaw)
        + z_axis.scaled(-sine_pitch)
    )
    body_y = (
        x_axis.scaled(sine_roll * sine_pitch * cosine_yaw - cosine_roll * sine_yaw)
        + y_axis.scaled(sine_roll * sine_pitch * sine_yaw + cosine_roll * cosine_yaw)
        + z_axis.scaled(sine_roll * cosine_pitch)
    )
    body_z = (
        x_axis.scaled(cosine_roll * sine_pitch * cosine_yaw + sine_roll * sine_yaw)
        + y_axis.scaled(cosine_roll * sine_pitch * sine_yaw - sine_roll * cosine_yaw)
        + z_axis.scaled(cosine_roll * cosine_pitch)
    )
    return Basis3(body_x, body_y, body_z, reference_basis.parent_frame, Frame.BODY)
####


def body_basis_to_euler_angles(
    body_basis: Basis3,
    reference_basis: Basis3,
    *,
    singularity_tolerance: float = 1e-12,
) -> EulerAngles:
    """Recover Euler angles from a body basis using equations 2-54 through 2-61.

    At the pole, body x is parallel or antiparallel to reference z and yaw is
    undefined. The manual convention is used: yaw and roll are zero, with
    pitch equal to ``-pi/2`` for alignment and ``+pi/2`` for anti-alignment.
    """

    _require_reference_basis(reference_basis)
    if body_basis.parent_frame is not reference_basis.parent_frame or body_basis.child_frame is not Frame.BODY:
        raise ValueError("body basis must be a BODY basis expressed in the reference parent frame")
    if not body_basis.is_orthonormal():
        raise ValueError("body basis must be orthonormal")
    if not math.isfinite(singularity_tolerance) or singularity_tolerance <= 0.0:
        raise ValueError("singularity_tolerance must be positive and finite")
    body_x = body_basis.first
    reference_x = reference_basis.first
    reference_y = reference_basis.second
    reference_z = reference_basis.third
    x_projection = body_x.dot(reference_x)
    y_projection = body_x.dot(reference_y)
    z_projection = body_x.dot(reference_z)
    horizontal_projection = math.hypot(x_projection, y_projection)
    if horizontal_projection <= singularity_tolerance:
        pitch = -math.pi / 2.0 if z_projection >= 0.0 else math.pi / 2.0
        return EulerAngles(Angle(0.0), Angle(pitch), Angle(0.0))
    yaw = math.atan2(y_projection, x_projection)
    pitch = math.atan2(-z_projection, horizontal_projection)
    temporary_x = reference_x.scaled(math.cos(yaw)) + reference_y.scaled(math.sin(yaw))
    temporary_y = reference_z.cross(temporary_x)
    roll = math.atan2(-body_basis.third.dot(temporary_y), body_basis.second.dot(temporary_y))
    return EulerAngles(Angle(yaw), Angle(pitch), Angle(roll))
####


def wind_to_body_aerodynamic(
    wind_basis: Basis3,
    angle_of_attack: Angle,
    euler_sideslip: Angle,
) -> Basis3:
    """Construct body axes from WIND axes and aerodynamic angles.

    This implements ``TAOS-ALG-COORD-017`` and equations 2-72 through 2-75.
    The sequence is Euler sideslip about wind z followed by angle of attack
    about the body negative-y axis.
    """

    _require_wind_basis(wind_basis)
    cosine_alpha = math.cos(angle_of_attack.radians)
    sine_alpha = math.sin(angle_of_attack.radians)
    cosine_beta = math.cos(euler_sideslip.radians)
    sine_beta = math.sin(euler_sideslip.radians)
    x_axis = wind_basis.first
    y_axis = wind_basis.second
    z_axis = wind_basis.third
    body_x = x_axis.scaled(cosine_alpha * cosine_beta) + y_axis.scaled(-cosine_alpha * sine_beta) + z_axis.scaled(-sine_alpha)
    body_y = x_axis.scaled(sine_beta) + y_axis.scaled(cosine_beta)
    body_z = x_axis.scaled(sine_alpha * cosine_beta) + y_axis.scaled(-sine_alpha * sine_beta) + z_axis.scaled(cosine_alpha)
    return Basis3(body_x, body_y, body_z, wind_basis.parent_frame, Frame.BODY)
####


def body_basis_to_wind_aerodynamic(body_basis: Basis3, wind_basis: Basis3) -> AerodynamicAngles:
    """Recover angle of attack and Euler sideslip from body and WIND bases.

    This is the derived inverse of ``wind_to_body_aerodynamic``. ``atan2``
    retains the full rotation range permitted by the manual.
    """

    _require_wind_basis(wind_basis)
    if body_basis.parent_frame is not wind_basis.parent_frame or body_basis.child_frame is not Frame.BODY:
        raise ValueError("body basis must be a BODY basis expressed in the wind parent frame")
    if not body_basis.is_orthonormal():
        raise ValueError("body basis must be orthonormal")
    angle_of_attack = math.atan2(-body_basis.first.dot(wind_basis.third), body_basis.third.dot(wind_basis.third))
    euler_sideslip = math.atan2(body_basis.second.dot(wind_basis.first), body_basis.second.dot(wind_basis.second))
    return AerodynamicAngles(Angle(angle_of_attack), Angle(euler_sideslip))
####


def _require_reference_basis(basis: Basis3) -> None:
    if not basis.is_orthonormal():
        raise ValueError("reference basis must be orthonormal")
    if basis.child_frame is Frame.BODY:
        raise ValueError("reference basis child frame cannot be BODY")
    ####


def _require_wind_basis(basis: Basis3) -> None:
    if basis.child_frame is not Frame.WIND:
        raise ValueError("basis must be a WIND basis")
    if not basis.is_orthonormal():
        raise ValueError("wind basis must be orthonormal")
    ####
