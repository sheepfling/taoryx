"""Explicit frame adapters for the ECIC rigid-body extension.

The rigid-body equations are integrated in ECIC.  Earth-fixed environment
models remain free to consume ECFC position and Earth-relative velocity; this
module is the deliberate boundary between those two conventions.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from .contracts import EarthModel, Frame, FrameVector3, GeodeticCoordinates, Latitude, Longitude, Quantity, Unit, Vector3
from .coordinates import geodetic_position_to_ecfc
from .earth import resolve_ellipsoid_parameters
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


@dataclass(frozen=True, slots=True)
class EarthRelativeVelocityStateAdapter:
    """Map inertial telemetry to local controller velocity channels."""

    earth_rotation: EarthRotationAdapter
    position_names: tuple[str, str, str] = ("position_x", "position_y", "position_z")
    velocity_names: tuple[str, str, str] = ("velocity_x", "velocity_y", "velocity_z")
    time_name: str = "time"

    def __call__(self, state: Mapping[str, float]) -> dict[str, float]:
        """Return a copy with inertial velocity channels replaced by ECFC values."""

        missing = [name for name in (*self.position_names, *self.velocity_names, self.time_name) if name not in state]
        if missing:
            raise KeyError(f"Earth-relative controller state is missing: {', '.join(missing)}")
        position = FrameVector3(Vector3(*(float(state[name]) for name in self.position_names)), Frame.ECIC)
        velocity = FrameVector3(Vector3(*(float(state[name]) for name in self.velocity_names)), Frame.ECIC)
        _, earth_relative_velocity = self.earth_rotation.ecic_to_ecfc(
            position,
            velocity,
            time_seconds=float(state[self.time_name]),
        )
        result = dict(state)
        for name, value in zip(
            self.velocity_names,
            (earth_relative_velocity.vector.x, earth_relative_velocity.vector.y, earth_relative_velocity.vector.z),
            strict=True,
        ):
            result[name] = value
        return result
        ####
    ####


@dataclass(frozen=True, slots=True)
class EarthOperatingPoint:
    """Location and transport context shared by trim and controller adapters.

    Vehicle trim variables remain local/air-relative. This object supplies the
    inertial transport needed to place that local operating point in ECIC,
    avoiding a separate hand-authored trim for every latitude.
    """

    earth: EarthModel
    longitude: Longitude
    latitude: Latitude
    altitude_m: float
    initial_angle_radians: float = 0.0
    reference_time_seconds: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.altitude_m) or self.altitude_m < 0.0:
            raise ValueError("operating-point altitude must be finite and nonnegative")
        if not math.isfinite(self.initial_angle_radians) or not math.isfinite(self.reference_time_seconds):
            raise ValueError("operating-point Earth-angle metadata must be finite")

    @property
    def earth_rotation_rate_rad_s(self) -> float:
        """Return the declared Earth rotation rate in SI units."""

        return self.earth.rotation_rate.si_value

    @property
    def position_ecfc(self) -> FrameVector3:
        """Return the operating-point geodetic location in ECFC."""

        parameters = resolve_ellipsoid_parameters(self.earth.equatorial_radius, flattening=self.earth.flattening)
        position = geodetic_position_to_ecfc(
            GeodeticCoordinates(self.longitude, self.latitude, Quantity(self.altitude_m, Unit.METER)),
            parameters,
        )
        return FrameVector3(position.to(Unit.METER).vector, Frame.ECFC)

    def earth_transport_velocity_ecfc(self) -> FrameVector3:
        """Return the ECFC velocity of a point fixed to the rotating Earth."""

        position = self.position_ecfc.vector
        omega_cross_position = Vector3(-self.earth_rotation_rate_rad_s * position.y, self.earth_rotation_rate_rad_s * position.x, 0.0)
        return FrameVector3(omega_cross_position, Frame.ECFC)

    def inertial_velocity_from_local_ecfc(self, local_velocity_ecfc: Vector3, *, time_seconds: float = 0.0) -> FrameVector3:
        """Convert local Earth-relative ECFC velocity into inertial ECIC velocity."""

        adapter = EarthRotationAdapter(
            self.earth,
            initial_angle_radians=self.initial_angle_radians,
            reference_time_seconds=self.reference_time_seconds,
        )
        _, velocity = adapter.ecfc_to_ecic(
            self.position_ecfc,
            FrameVector3(local_velocity_ecfc, Frame.ECFC),
            time_seconds=time_seconds,
        )
        return velocity

    def position_ecic(self, *, time_seconds: float = 0.0) -> FrameVector3:
        """Return this fixed geodetic location expressed in ECIC."""

        adapter = EarthRotationAdapter(
            self.earth,
            initial_angle_radians=self.initial_angle_radians,
            reference_time_seconds=self.reference_time_seconds,
        )
        position, _ = adapter.ecfc_to_ecic(
            self.position_ecfc,
            FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
            time_seconds=time_seconds,
        )
        return position

    def earth_relative_velocity_from_inertial_ecic(
        self,
        inertial_velocity_ecic: Vector3,
        *,
        time_seconds: float = 0.0,
    ) -> FrameVector3:
        """Convert inertial velocity telemetry into local Earth-relative velocity."""

        adapter = EarthRotationAdapter(
            self.earth,
            initial_angle_radians=self.initial_angle_radians,
            reference_time_seconds=self.reference_time_seconds,
        )
        _, velocity = adapter.ecic_to_ecfc(
            self.position_ecic(time_seconds=time_seconds),
            FrameVector3(inertial_velocity_ecic, Frame.ECIC),
            time_seconds=time_seconds,
        )
        return velocity

    def earth_rotation_angular_rate_ecic(self) -> FrameVector3:
        """Return the common Earth-rate feed-forward vector in ECIC."""

        return FrameVector3(Vector3(0.0, 0.0, self.earth_rotation_rate_rad_s), Frame.ECIC)

    def air_relative_velocity_ecic(
        self,
        local_velocity_ecfc: Vector3,
        wind_ecfc: Vector3,
        *,
        time_seconds: float = 0.0,
    ) -> FrameVector3:
        """Convert a local air-relative velocity into inertial air velocity."""

        adapter = EarthRotationAdapter(
            self.earth,
            initial_angle_radians=self.initial_angle_radians,
            reference_time_seconds=self.reference_time_seconds,
        )
        inertial_velocity = self.inertial_velocity_from_local_ecfc(local_velocity_ecfc, time_seconds=time_seconds)
        return adapter.air_relative_velocity_ecic(self.position_ecfc, inertial_velocity, FrameVector3(wind_ecfc, Frame.ECFC), time_seconds=time_seconds)

    def at_latitude(self, latitude: Latitude) -> EarthOperatingPoint:
        """Return this operating point transported to another latitude."""

        return EarthOperatingPoint(
            earth=self.earth,
            longitude=self.longitude,
            latitude=latitude,
            altitude_m=self.altitude_m,
            initial_angle_radians=self.initial_angle_radians,
            reference_time_seconds=self.reference_time_seconds,
        )

    def effective_gravity_mps2(self) -> float:
        """Return central gravity plus centrifugal acceleration magnitude."""

        position = self.position_ecfc.vector
        radius = max(position.norm(), 1.0e-12)
        gravitational_acceleration = position.scaled(-self.earth.gravitational_parameter.si_value / radius**3)
        centrifugal_acceleration = Vector3(
            self.earth_rotation_rate_rad_s**2 * position.x,
            self.earth_rotation_rate_rad_s**2 * position.y,
            0.0,
        )
        return (gravitational_acceleration + centrifugal_acceleration).norm()

    def latitude_sensitivity(
        self,
        latitudes: Sequence[Latitude],
        *,
        gravity_tolerance_fraction: float = 0.01,
    ) -> tuple[dict[str, float | bool], ...]:
        """Assess local-trim reuse across latitude samples.

        This is a screening metric, not a substitute for a vehicle residual
        solve when atmosphere, wind, or table bounds change materially.
        """

        if not math.isfinite(gravity_tolerance_fraction) or gravity_tolerance_fraction < 0.0:
            raise ValueError("gravity tolerance fraction must be finite and nonnegative")
        anchor_gravity = self.effective_gravity_mps2()
        results: list[dict[str, float | bool]] = []
        for latitude in latitudes:
            point = self.at_latitude(latitude)
            gravity = point.effective_gravity_mps2()
            delta_fraction = abs(gravity - anchor_gravity) / max(anchor_gravity, 1.0e-12)
            results.append(
                {
                    "latitude_rad": latitude.radians,
                    "earth_transport_speed_mps": point.earth_transport_velocity_ecfc().vector.norm(),
                    "effective_gravity_mps2": gravity,
                    "gravity_delta_fraction": delta_fraction,
                    "reuse_local_trim": delta_fraction <= gravity_tolerance_fraction,
                }
            )
        return tuple(results)

    def to_metadata(self) -> dict[str, float | str]:
        """Return JSON-safe provenance for a trim/controller artifact."""

        return {
            "longitude_rad": self.longitude.radians,
            "latitude_rad": self.latitude.radians,
            "altitude_m": self.altitude_m,
            "earth_rotation_rate_rad_s": self.earth_rotation_rate_rad_s,
            "initial_angle_rad": self.initial_angle_radians,
            "reference_time_s": self.reference_time_seconds,
        }
####
