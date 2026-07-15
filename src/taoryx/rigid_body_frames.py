"""Explicit frame adapters for the ECIC rigid-body extension.

The rigid-body equations are integrated in ECIC.  Earth-fixed environment
models remain free to consume ECFC position and Earth-relative velocity; this
module is the deliberate boundary between those two conventions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .contracts import EarthModel, Frame, FrameVector3, Vector3
from .equations.frames import ecic_rotation_angle


def _rotate_z(vector: Vector3, angle_radians: float) -> Vector3:
    cosine = math.cos(angle_radians)
    sine = math.sin(angle_radians)
    return Vector3(
        cosine * vector.x - sine * vector.y,
        sine * vector.x + cosine * vector.y,
        vector.z,
    )
####


def _require_frame(value: FrameVector3, expected: Frame, name: str) -> None:
    if value.frame is not expected:
        raise ValueError(f"{name} must be expressed in {expected.value}")
    ####
####


@dataclass(frozen=True, slots=True)
class EarthRotationAdapter:
    """Convert ECIC inertial kinematics to ECFC environment inputs.

    ``initial_angle_radians`` and ``reference_time_seconds`` use the same
    convention as the manual's ECIC/ECFC rotation equation.  Velocities are
    converted with the transport term, so a vehicle fixed to the rotating
    Earth has zero Earth-relative velocity in ECFC.
    """

    earth: EarthModel
    initial_angle_radians: float = 0.0
    reference_time_seconds: float = 0.0

    def angle(self, time_seconds: float) -> float:
        """Return the ECIC-to-ECFC rotation angle at ``time_seconds``."""

        return ecic_rotation_angle(
            self.initial_angle_radians,
            self.earth.rotation_rate.si_value,
            time_seconds,
            self.reference_time_seconds,
        )
        ####

    def ecic_to_ecfc(
        self,
        position_ecic: FrameVector3,
        inertial_velocity_ecic: FrameVector3,
        *,
        time_seconds: float,
    ) -> tuple[FrameVector3, FrameVector3]:
        """Return ECFC position and Earth-relative velocity."""

        _require_frame(position_ecic, Frame.ECIC, "position")
        _require_frame(inertial_velocity_ecic, Frame.ECIC, "velocity")
        theta = self.angle(time_seconds)
        position = _rotate_z(position_ecic.vector, -theta)
        omega_cross_position = Vector3(
            -self.earth.rotation_rate.si_value * position_ecic.vector.y,
            self.earth.rotation_rate.si_value * position_ecic.vector.x,
            0.0,
        )
        earth_relative_velocity = _rotate_z(inertial_velocity_ecic.vector - omega_cross_position, -theta)
        return FrameVector3(position, Frame.ECFC), FrameVector3(earth_relative_velocity, Frame.ECFC)
        ####

    def ecfc_to_ecic(
        self,
        position_ecfc: FrameVector3,
        earth_relative_velocity_ecfc: FrameVector3,
        *,
        time_seconds: float,
    ) -> tuple[FrameVector3, FrameVector3]:
        """Return ECIC position and inertial velocity."""

        _require_frame(position_ecfc, Frame.ECFC, "position")
        _require_frame(earth_relative_velocity_ecfc, Frame.ECFC, "velocity")
        theta = self.angle(time_seconds)
        position = _rotate_z(position_ecfc.vector, theta)
        omega_cross_position = Vector3(
            -self.earth.rotation_rate.si_value * position.y,
            self.earth.rotation_rate.si_value * position.x,
            0.0,
        )
        inertial_velocity = _rotate_z(earth_relative_velocity_ecfc.vector, theta) + omega_cross_position
        return FrameVector3(position, Frame.ECIC), FrameVector3(inertial_velocity, Frame.ECIC)
        ####

    def air_relative_velocity_ecic(
        self,
        position_ecic: FrameVector3,
        inertial_velocity_ecic: FrameVector3,
        wind_ecfc: FrameVector3,
        *,
        time_seconds: float,
    ) -> FrameVector3:
        """Return air-relative velocity in ECIC after Earth-rotation correction."""

        _require_frame(wind_ecfc, Frame.ECFC, "wind")
        _, earth_relative_velocity = self.ecic_to_ecfc(
            position_ecic,
            inertial_velocity_ecic,
            time_seconds=time_seconds,
        )
        relative_ecfc = earth_relative_velocity.vector - wind_ecfc.vector
        return FrameVector3(_rotate_z(relative_ecfc, self.angle(time_seconds)), Frame.ECIC)
        ####
####
