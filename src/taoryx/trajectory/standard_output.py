"""Portable ECEF/ECFC pose projection for Mission Composition results.

Mission Composition providers retain their native output channels because those
channels are part of their source evidence.  This module adds one uniform,
consumer-facing pose alongside them: WGS-84 Earth-Centred, Earth-Fixed (ECEF;
``ecfc`` in the TAOS vocabulary) position, Earth-relative velocity, and a
right-handed forward/right/down orientation quaternion.

Some low-fidelity and source-replay providers expose only local translation.
For those cases the projection uses the documented WGS-84 equatorial tangent
embedding rather than silently relabeling local coordinates as ECEF.  Likewise,
the universal orientation is a kinematic velocity-aligned reference when a
provider has not made physical attitude truth available.  The provenance fields
make both cases explicit to consumers.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

_WGS84_EQUATORIAL_RADIUS_M = 6_378_137.0
_WGS84_FLATTENING = 1.0 / 298.257223563
_WGS84_ECCENTRICITY_SQUARED = _WGS84_FLATTENING * (2.0 - _WGS84_FLATTENING)
_EARTH_ROTATION_RADPS = 7.2921150e-5
_EPSILON = 1.0e-12

EcefProjectionKind = Literal[
    "native_ecfc",
    "geodetic_wgs84",
    "ecic_to_ecfc_zero_epoch",
    "local_ned_wgs84_equatorial_embedding",
    "local_cartesian_wgs84_equatorial_embedding",
    "reference_origin_wgs84_equatorial_embedding",
]
EcefOrientationKind = Literal[
    "native_ecef_from_body",
    "native_ned_from_body",
    "native_ecic_from_body",
    "native_body_from_local_ned",
    "native_inertial_to_body",
    "kinematic_velocity_aligned",
    "held_kinematic_orientation",
    "local_ned_reference",
]
EcefAccelerationKind = Literal[
    "native_earth_relative_acceleration",
    "finite_difference_earth_relative_velocity",
]
EcefAngularVelocityKind = Literal["native_body_rate", "orientation_finite_difference"]
Vector3 = tuple[float, float, float]
QuaternionWxyz = tuple[float, float, float, float]


class StandardEcefState(BaseModel):
    """Required cross-provider ECEF kinematics and world-from-body attitude.

    ``ecef_from_body_wxyz`` maps a forward/right/down body-reference vector
    into the ECEF world frame. ``orientation_kind`` distinguishes native
    physical attitude truth from the velocity-aligned kinematic orientation
    used when a provider does not expose rigid-body attitude. Acceleration is
    either native Earth-relative truth or the finite-difference derivative of
    the standardized Earth-relative velocity at accepted sample boundaries.
    Angular velocity uses documented body rates when present and otherwise is
    derived from that quaternion.
    """

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.standard-ecef-state/v1"] = Field(
        default="taoryx.standard-ecef-state/v1",
        alias="schema",
        serialization_alias="schema",
    )
    frame_id: Literal["ecfc"] = "ecfc"
    position_ecef_m: Vector3
    velocity_ecef_mps: Vector3
    acceleration_ecef_mps2: Vector3
    acceleration_kind: EcefAccelerationKind = "finite_difference_earth_relative_velocity"
    angular_velocity_body_radps: Vector3
    angular_velocity_kind: EcefAngularVelocityKind
    ecef_from_body_wxyz: QuaternionWxyz
    position_projection: EcefProjectionKind
    source_frame: str = Field(min_length=1)
    orientation_kind: EcefOrientationKind

    @model_validator(mode="after")
    def validate_state(self) -> StandardEcefState:
        values = (
            *self.position_ecef_m,
            *self.velocity_ecef_mps,
            *self.acceleration_ecef_mps2,
            *self.angular_velocity_body_radps,
            *self.ecef_from_body_wxyz,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("standard ECEF state requires finite position, velocity, and orientation values")
        norm = math.sqrt(sum(value * value for value in self.ecef_from_body_wxyz))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-9):
            raise ValueError("standard ECEF orientation quaternion must have unit norm")
        return self
        ####

    ####


@dataclass(frozen=True, slots=True)
class _Pose:
    """Projected translation before absent velocities are differenced."""

    position_ecef_m: Vector3
    velocity_ecef_mps: Vector3 | None
    projection: EcefProjectionKind
    source_frame: str

    ####


def project_standard_ecef_samples(
    samples: Sequence[tuple[float, Mapping[str, Any]]],
    *,
    channel_frames: Mapping[str, str | None] | None = None,
    channel_units: Mapping[str, str | None] | None = None,
    existing: Sequence[StandardEcefState | None] | None = None,
) -> tuple[StandardEcefState, ...]:
    """Return one standard ECEF pose for every accepted source sample.

    The function is intentionally tolerant of the repository's current
    native-channel vocabulary.  It prefers native ECFC, then ECIC, geodetic,
    local-NED, and local-Cartesian truth.  A model with no position channel is
    still given an explicit WGS-84 equatorial reference-origin pose instead of
    receiving an unlabeled local-coordinate reinterpretation.
    """

    if not samples:
        return ()
    if existing is not None and len(existing) != len(samples):
        raise ValueError("existing standard ECEF states must align with source samples")
    frames = dict(channel_frames or {})
    units = dict(channel_units or {})
    poses = tuple(_extract_pose(time_s, values, frames, units) for time_s, values in samples)
    times = tuple(float(time_s) for time_s, _ in samples)
    velocities = tuple(
        pose.velocity_ecef_mps if pose.velocity_ecef_mps is not None else _position_difference(poses, times, index)
        for index, pose in enumerate(poses)
    )
    native_accelerations = tuple(_native_ecfc_acceleration(values, frames) for _, values in samples)
    accelerations = tuple(
        native if native is not None else _vector_difference(velocities, times, index)
        for index, native in enumerate(native_accelerations)
    )
    acceleration_kinds: tuple[EcefAccelerationKind, ...] = tuple(
        "native_earth_relative_acceleration" if native is not None else "finite_difference_earth_relative_velocity"
        for native in native_accelerations
    )
    orientations: list[QuaternionWxyz] = []
    orientation_kinds: list[EcefOrientationKind] = []
    preserved: list[StandardEcefState | None] = []
    prior_orientation: QuaternionWxyz | None = None
    for index, ((time_s, values), pose, velocity) in enumerate(zip(samples, poses, velocities, strict=True)):
        prior = None if existing is None else existing[index]
        if prior is not None and pose.projection == "reference_origin_wgs84_equatorial_embedding":
            orientation = prior.ecef_from_body_wxyz
            orientation_kind = prior.orientation_kind
            preserved.append(prior)
        else:
            native_orientation = _native_orientation(values, pose, time_s)
            if native_orientation is None:
                orientation, orientation_kind = _velocity_aligned_orientation(
                    pose.position_ecef_m,
                    velocity,
                    previous=prior_orientation,
                )
            else:
                orientation, orientation_kind = native_orientation
                if prior_orientation is not None and _dot(orientation, prior_orientation) < 0.0:
                    orientation = tuple(-value for value in orientation)  # type: ignore[assignment]
            preserved.append(None)
        orientations.append(orientation)
        orientation_kinds.append(orientation_kind)
        prior_orientation = orientation

    angular_velocities: list[Vector3] = []
    angular_velocity_kinds: list[EcefAngularVelocityKind] = []
    for index, ((_, values), prior) in enumerate(zip(samples, preserved, strict=True)):
        if prior is not None:
            angular_velocities.append(prior.angular_velocity_body_radps)
            angular_velocity_kinds.append(prior.angular_velocity_kind)
            continue
        native_body_rate = _native_body_angular_velocity(values)
        if native_body_rate is not None:
            angular_velocities.append(native_body_rate)
            angular_velocity_kinds.append("native_body_rate")
            continue
        angular_velocities.append(_body_angular_velocity_difference(tuple(orientations), times, index))
        angular_velocity_kinds.append("orientation_finite_difference")

    result: list[StandardEcefState] = []
    for pose, velocity, acceleration, acceleration_kind, orientation, orientation_kind, angular_velocity, angular_velocity_kind, prior in (
        zip(
            poses,
            velocities,
            accelerations,
            acceleration_kinds,
            orientations,
            orientation_kinds,
            angular_velocities,
            angular_velocity_kinds,
            preserved,
            strict=True,
        )
    ):
        if prior is not None:
            result.append(prior)
            continue
        state = StandardEcefState(
            position_ecef_m=pose.position_ecef_m,
            velocity_ecef_mps=velocity,
            acceleration_ecef_mps2=acceleration,
            acceleration_kind=acceleration_kind,
            angular_velocity_body_radps=angular_velocity,
            angular_velocity_kind=angular_velocity_kind,
            ecef_from_body_wxyz=orientation,
            position_projection=pose.projection,
            source_frame=pose.source_frame,
            orientation_kind=orientation_kind,
        )
        result.append(state)
    return tuple(result)
    ####


def standard_ecef_state_from_values(
    time_s: float,
    values: Mapping[str, Any],
    *,
    existing: StandardEcefState | None = None,
) -> StandardEcefState:
    """Project a single session observation into the standard ECEF pose."""

    return project_standard_ecef_samples(((time_s, values),), existing=(existing,))[0]
    ####


def _extract_pose(
    time_s: float,
    values: Mapping[str, Any],
    frames: Mapping[str, str | None],
    units: Mapping[str, str | None],
) -> _Pose:
    ecfc_position = _vector_from_components(values, (("position.ecfc.x", "position.ecfc.y", "position.ecfc.z"), ("position.ecef.x", "position.ecef.y", "position.ecef.z")))
    if ecfc_position is None:
        ecfc_position = _vector_value(values, ("position.ecfc", "position.ecef", "position_ecfc_m", "position_ecef_m"))
    if ecfc_position is None:
        ecfc_position = _framed_vector_value(values, frames, role="position", frame_kind="ecfc")
    if ecfc_position is not None:
        velocity = _vector_from_components(values, (("velocity.ecfc.x", "velocity.ecfc.y", "velocity.ecfc.z"), ("velocity.ecef.x", "velocity.ecef.y", "velocity.ecef.z")))
        if velocity is None:
            velocity = _vector_value(values, ("velocity.ecfc", "velocity.ecef", "velocity_ecfc_mps", "velocity_ecef_mps"))
        if velocity is None:
            velocity = _framed_vector_value(values, frames, role="velocity", frame_kind="ecfc")
        return _Pose(ecfc_position, velocity, "native_ecfc", "ecfc")

    ecic_position = _vector_from_components(values, (("position.eci.x", "position.eci.y", "position.eci.z"), ("position.ecic.x", "position.ecic.y", "position.ecic.z")))
    if ecic_position is None:
        ecic_position = _vector_value(
            values,
            ("position.eci", "position.ecic", "position_eci_m", "position_ecic_m", "position_inertial_m"),
        )
    if ecic_position is None:
        ecic_position = _framed_vector_value(values, frames, role="position", frame_kind="ecic")
    if ecic_position is not None:
        ecic_velocity = _vector_from_components(values, (("velocity.eci.x", "velocity.eci.y", "velocity.eci.z"), ("velocity.ecic.x", "velocity.ecic.y", "velocity.ecic.z")))
        if ecic_velocity is None:
            ecic_velocity = _vector_value(
                values,
                ("velocity.eci", "velocity.ecic", "velocity_eci_mps", "velocity_ecic_mps", "velocity_inertial_mps"),
            )
        if ecic_velocity is None:
            ecic_velocity = _framed_vector_value(values, frames, role="velocity", frame_kind="ecic")
        position, velocity = _ecic_to_ecfc(ecic_position, ecic_velocity, time_s)
        return _Pose(position, velocity, "ecic_to_ecfc_zero_epoch", "ecic")

    geodetic = _geodetic_position(values, units)
    if geodetic is not None:
        return _Pose(geodetic, _local_velocity(values, frames, kind="geodetic"), "geodetic_wgs84", "geodetic")

    local_ned = _local_ned_position(values, frames)
    if local_ned is None:
        local_ned_raw = _framed_vector_value(values, frames, role="position", frame_kind="local_ned")
        if local_ned_raw is not None:
            local_ned = (local_ned_raw[0], local_ned_raw[1], -local_ned_raw[2])
    if local_ned is not None:
        north, east, up = local_ned
        velocity = _local_velocity(values, frames, kind="local_ned")
        if velocity is None:
            local_ned_velocity = _framed_vector_value(values, frames, role="velocity", frame_kind="local_ned")
            if local_ned_velocity is not None:
                velocity = (local_ned_velocity[0], local_ned_velocity[1], -local_ned_velocity[2])
        return _Pose(
            _local_neu_position_to_ecfc(north, east, up),
            None if velocity is None else _local_neu_vector_to_ecfc(*velocity),
            "local_ned_wgs84_equatorial_embedding",
            "local_ned",
        )

    local_cartesian = _local_cartesian_position(values, frames)
    if local_cartesian is None:
        local_cartesian = _framed_vector_value(values, frames, role="position", frame_kind="local_cartesian")
    if local_cartesian is not None:
        x, y, z = local_cartesian
        velocity = _local_velocity(values, frames, kind="local_cartesian")
        if velocity is None:
            velocity = _framed_vector_value(values, frames, role="velocity", frame_kind="local_cartesian")
        return _Pose(
            _local_neu_position_to_ecfc(x, y, z),
            None if velocity is None else _local_neu_vector_to_ecfc(*velocity),
            "local_cartesian_wgs84_equatorial_embedding",
            "local_cartesian",
        )

    return _Pose(
        (_WGS84_EQUATORIAL_RADIUS_M, 0.0, 0.0),
        None,
        "reference_origin_wgs84_equatorial_embedding",
        "unlocated_reference_origin",
    )
    ####


def _geodetic_position(values: Mapping[str, Any], units: Mapping[str, str | None]) -> Vector3 | None:
    latitude_ids = ("position.geodetic.latitude", "position.latitude.geodetic", "latitude_deg", "latitude")
    longitude_ids = ("position.geodetic.longitude", "position.longitude.geodetic", "position.longitude", "longitude_deg", "longitude")
    altitude_ids = ("position.geodetic.altitude", "position.altitude.geodetic", "position.geometric.altitude", "altitude_m", "altitude")
    latitude = _scalar_value(values, latitude_ids)
    longitude = _scalar_value(values, longitude_ids)
    if latitude is None or longitude is None:
        return None
    altitude = _scalar_value(values, altitude_ids)
    latitude_unit = _first_unit(latitude_ids, values, units)
    longitude_unit = _first_unit(longitude_ids, values, units)
    return _geodetic_to_ecfc(
        _angle_to_radians(latitude, latitude_unit, latitude_ids),
        _angle_to_radians(longitude, longitude_unit, longitude_ids),
        0.0 if altitude is None else altitude,
    )
    ####


def _local_ned_position(values: Mapping[str, Any], frames: Mapping[str, str | None]) -> Vector3 | None:
    vector = _vector_value(values, ("position_ned_m", "position.local.ned", "position.local_ned"))
    if vector is not None:
        return (vector[0], vector[1], -vector[2])
    north = _scalar_value(values, ("position.local.north", "position.north_m", "north_m"))
    east = _scalar_value(values, ("position.local.east", "position.east_m", "east_m"))
    if north is not None and east is not None:
        down = _scalar_value(values, ("position.local.down", "position.down_m", "down_m"))
        if down is not None:
            return (north, east, -down)
        up = _scalar_value(values, ("position.geometric.altitude", "position.altitude_m", "position.altitude", "altitude_m"))
        if up is not None:
            return (north, east, up)
    local_xyz = _vector_from_components(values, (("position.local.x", "position.local.y", "position.local.z"),))
    if local_xyz is not None and _is_ned_frame(_frame_for("position.local.x", frames)):
        # Native local x/y/z adapters use north/east/positive-up values even
        # where their enclosing local tangent frame is named NED.
        return local_xyz
    vector = _vector_value(values, ("position", "position_m"))
    if vector is not None and _is_ned_frame(_first_present_frame(values, frames, ("position", "position_m"))):
        return (vector[0], vector[1], -vector[2])
    return None
    ####


def _local_cartesian_position(values: Mapping[str, Any], frames: Mapping[str, str | None]) -> Vector3 | None:
    local_xyz = _vector_from_components(values, (("position.local.x", "position.local.y", "position.local.z"),))
    if local_xyz is not None and not _is_ned_frame(_frame_for("position.local.x", frames)):
        return local_xyz
    vector = _vector_value(values, ("position", "position_m", "position.local"))
    if vector is not None:
        frame = _first_present_frame(values, frames, ("position", "position_m", "position.local"))
        if frame is None or not _is_ned_frame(frame):
            return vector
    # The legacy neutral-provider contract reports scalar downrange and
    # altitude.  It has no claimed geographic origin, so keep it as a local
    # Cartesian x/z pair and let the documented equatorial tangent embedding
    # below make its ECEF meaning explicit.
    downrange = _scalar_value(values, ("position.downrange_m", "downrange_m"))
    if downrange is not None:
        crossrange = _scalar_value(values, ("position.crossrange_m", "crossrange_m"))
        altitude = _scalar_value(values, ("position.altitude_m", "altitude_m", "altitude"))
        return (downrange, 0.0 if crossrange is None else crossrange, 0.0 if altitude is None else altitude)
    return None
    ####


def _local_velocity(
    values: Mapping[str, Any],
    frames: Mapping[str, str | None],
    *,
    kind: Literal["geodetic", "local_ned", "local_cartesian"],
) -> Vector3 | None:
    if kind == "local_ned":
        vector = _vector_value(values, ("velocity_ned_mps", "velocity.local.ned", "velocity.local_ned"))
        if vector is not None:
            return (vector[0], vector[1], -vector[2])
        north = _scalar_value(values, ("velocity.local.north", "velocity.north_m_s", "velocity.north_mps", "north_velocity_mps"))
        east = _scalar_value(values, ("velocity.local.east", "velocity.east_m_s", "velocity.east_mps", "east_velocity_mps"))
        if north is not None and east is not None:
            down = _scalar_value(values, ("velocity.local.down", "velocity.down_m_s", "velocity.down_mps", "vertical_velocity_down_mps"))
            if down is not None:
                return (north, east, -down)
            up = _scalar_value(values, ("velocity.local.vertical", "velocity.vertical_m_s", "velocity.vertical_mps", "vertical_velocity_mps"))
            if up is not None:
                return (north, east, up)
        local_xyz = _vector_from_components(values, (("velocity.local.x", "velocity.local.y", "velocity.local.z"),))
        if local_xyz is not None and _is_ned_frame(_frame_for("velocity.local.x", frames)):
            return local_xyz
        vector = _vector_value(values, ("velocity", "velocity_mps"))
        if vector is not None and _is_ned_frame(_first_present_frame(values, frames, ("velocity", "velocity_mps"))):
            return (vector[0], vector[1], -vector[2])
    elif kind == "local_cartesian":
        local_xyz = _vector_from_components(values, (("velocity.local.x", "velocity.local.y", "velocity.local.z"),))
        if local_xyz is not None and not _is_ned_frame(_frame_for("velocity.local.x", frames)):
            return local_xyz
        vector = _vector_value(values, ("velocity", "velocity_mps", "velocity.local"))
        if vector is not None:
            frame = _first_present_frame(values, frames, ("velocity", "velocity_mps", "velocity.local"))
            if frame is None or not _is_ned_frame(frame):
                return vector
        downrange_velocity = _scalar_value(values, ("velocity.downrange_m_s", "velocity.downrange_mps", "velocity.m_s"))
        if downrange_velocity is not None:
            crossrange_velocity = _scalar_value(values, ("velocity.crossrange_m_s", "velocity.crossrange_mps"))
            vertical_velocity = _scalar_value(values, ("velocity.vertical_m_s", "velocity.vertical_mps"))
            return (
                downrange_velocity,
                0.0 if crossrange_velocity is None else crossrange_velocity,
                0.0 if vertical_velocity is None else vertical_velocity,
            )
    return None
    ####


def _native_orientation(
    values: Mapping[str, Any],
    pose: _Pose,
    time_s: float,
) -> tuple[QuaternionWxyz, EcefOrientationKind] | None:
    """Project exact CADAC attitude truth when its source convention is known.

    The common result vocabulary does not treat arbitrary four-vectors as
    attitudes.  A provider must publish an explicit frame-and-direction name,
    or use the established CADAC ``quaternion_wxyz`` convention: local-NED
    results use body-from-local and inertial results use inertial-to-body.
    Other models keep their native quaternion in ``values`` and receive the
    honest kinematic reference below until they publish an equally explicit
    binding.
    """

    direct_ecef = _quaternion_value(values, ("ecef_from_body_wxyz", "quaternion_ecef_from_body_wxyz"))
    if direct_ecef is not None and pose.source_frame == "ecfc":
        return direct_ecef, "native_ecef_from_body"

    direct_ned = _quaternion_value(values, ("ned_from_body_wxyz", "quaternion_ned_from_body_wxyz"))
    if direct_ned is not None and pose.source_frame == "local_ned":
        ecef_from_ned = _quaternion_from_axes(
            (0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0),
            (-1.0, 0.0, 0.0),
        )
        return _normalize_quaternion(_quaternion_multiply(ecef_from_ned, direct_ned)), "native_ned_from_body"

    direct_ecic = _quaternion_value(values, ("ecic_from_body_wxyz", "quaternion_ecic_from_body_wxyz"))
    if direct_ecic is not None and pose.source_frame == "ecic":
        theta = _EARTH_ROTATION_RADPS * time_s
        ecef_from_ecic = (math.cos(0.5 * theta), 0.0, 0.0, -math.sin(0.5 * theta))
        return _normalize_quaternion(_quaternion_multiply(ecef_from_ecic, direct_ecic)), "native_ecic_from_body"

    source_quaternion = _quaternion_value(values, ("quaternion_wxyz",))
    if source_quaternion is None:
        return None
    if pose.source_frame == "local_ned":
        ecef_from_ned = _quaternion_from_axes(
            (0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0),
            (-1.0, 0.0, 0.0),
        )
        ecef_from_body = _quaternion_multiply(ecef_from_ned, _quaternion_conjugate(source_quaternion))
        return _normalize_quaternion(ecef_from_body), "native_body_from_local_ned"
    if pose.source_frame == "ecic":
        theta = _EARTH_ROTATION_RADPS * time_s
        ecef_from_ecic = (math.cos(0.5 * theta), 0.0, 0.0, -math.sin(0.5 * theta))
        ecef_from_body = _quaternion_multiply(ecef_from_ecic, _quaternion_conjugate(source_quaternion))
        return _normalize_quaternion(ecef_from_body), "native_inertial_to_body"
    return None
    ####


def _native_ecfc_acceleration(
    values: Mapping[str, Any],
    frames: Mapping[str, str | None],
) -> Vector3 | None:
    """Return a directly published Earth-relative ECEF acceleration vector."""

    acceleration = _vector_from_components(
        values,
        (
            ("acceleration.ecfc.x", "acceleration.ecfc.y", "acceleration.ecfc.z"),
            ("acceleration.ecef.x", "acceleration.ecef.y", "acceleration.ecef.z"),
        ),
    )
    if acceleration is None:
        acceleration = _vector_value(
            values,
            ("acceleration.ecfc", "acceleration.ecef", "acceleration_ecfc_mps2", "acceleration_ecef_mps2"),
        )
    if acceleration is None:
        acceleration = _framed_vector_value(values, frames, role="acceleration", frame_kind="ecfc")
    return acceleration
    ####


def _native_body_angular_velocity(values: Mapping[str, Any]) -> Vector3 | None:
    """Return an explicitly Earth-relative body rate when a provider exposes one."""

    vector = _vector_value(
        values,
        (
            "angular_velocity_body_radps",
            "angular_velocity_body_rad_s",
            "body_rates_earth_rad_s",
            "body_rates_rad_s",
        ),
    )
    if vector is not None:
        return vector
    return _vector_from_components(
        values,
        (
            ("angular_velocity.body.x", "angular_velocity.body.y", "angular_velocity.body.z"),
            ("angular_rate.body.x", "angular_rate.body.y", "angular_rate.body.z"),
        ),
    )
    ####


def _framed_vector_value(
    values: Mapping[str, Any],
    frames: Mapping[str, str | None],
    *,
    role: Literal["position", "velocity", "acceleration"],
    frame_kind: Literal["ecfc", "ecic", "local_ned", "local_cartesian"],
) -> Vector3 | None:
    """Read a vector channel using its advertised frame rather than its spelling."""

    for identifier, value in values.items():
        normalized_identifier = identifier.casefold()
        if not (normalized_identifier == role or normalized_identifier.startswith(f"{role}.") or normalized_identifier.startswith(f"{role}_")):
            continue
        if _frame_kind(frames.get(identifier)) != frame_kind:
            continue
        vector = _vector_value({identifier: value}, (identifier,))
        if vector is not None:
            return vector
    return None
    ####


def _frame_kind(frame: str | None) -> Literal["ecfc", "ecic", "local_ned", "local_cartesian"] | None:
    if frame is None:
        return None
    normalized = frame.casefold().replace("-", "_")
    if "ecfc" in normalized or "ecef" in normalized or "earth_fixed" in normalized:
        return "ecfc"
    if "ecic" in normalized or normalized == "eci" or "inertial" in normalized:
        return "ecic"
    if "ned" in normalized or "north_east_down" in normalized:
        return "local_ned"
    if "local" in normalized or "cartesian" in normalized or "tangent" in normalized:
        return "local_cartesian"
    return None
    ####


def _ecic_to_ecfc(position_ecic: Vector3, velocity_ecic: Vector3 | None, time_s: float) -> tuple[Vector3, Vector3 | None]:
    theta = _EARTH_ROTATION_RADPS * time_s
    position = _rotate_z(position_ecic, -theta)
    if velocity_ecic is None:
        return position, None
    omega_cross_position = (
        -_EARTH_ROTATION_RADPS * position_ecic[1],
        _EARTH_ROTATION_RADPS * position_ecic[0],
        0.0,
    )
    return position, _rotate_z(_subtract(velocity_ecic, omega_cross_position), -theta)
    ####


def _geodetic_to_ecfc(latitude_rad: float, longitude_rad: float, altitude_m: float) -> Vector3:
    sine_latitude = math.sin(latitude_rad)
    cosine_latitude = math.cos(latitude_rad)
    normal_radius = _WGS84_EQUATORIAL_RADIUS_M / math.sqrt(1.0 - _WGS84_ECCENTRICITY_SQUARED * sine_latitude * sine_latitude)
    return (
        (normal_radius + altitude_m) * cosine_latitude * math.cos(longitude_rad),
        (normal_radius + altitude_m) * cosine_latitude * math.sin(longitude_rad),
        (normal_radius * (1.0 - _WGS84_ECCENTRICITY_SQUARED) + altitude_m) * sine_latitude,
    )
    ####


def _local_neu_position_to_ecfc(north_m: float, east_m: float, up_m: float) -> Vector3:
    # The fixed equatorial origin is intentional and exposed in
    # ``position_projection``.  At latitude=longitude=0, north=+Z, east=+Y,
    # and up=+X in ECFC.
    return (_WGS84_EQUATORIAL_RADIUS_M + up_m, east_m, north_m)
    ####


def _local_neu_vector_to_ecfc(north_mps: float, east_mps: float, up_mps: float) -> Vector3:
    return (up_mps, east_mps, north_mps)
    ####


def _position_difference(poses: Sequence[_Pose], times: Sequence[float], index: int) -> Vector3:
    return _vector_difference(tuple(pose.position_ecef_m for pose in poses), times, index)
    ####


def _vector_difference(vectors: Sequence[Vector3], times: Sequence[float], index: int) -> Vector3:
    if len(vectors) == 1:
        return (0.0, 0.0, 0.0)
    before = next((candidate for candidate in range(index - 1, -1, -1) if times[index] - times[candidate] > _EPSILON), None)
    after = next((candidate for candidate in range(index + 1, len(vectors)) if times[candidate] - times[index] > _EPSILON), None)
    if before is not None and after is not None:
        return _scale(_subtract(vectors[after], vectors[before]), 1.0 / (times[after] - times[before]))
    if after is not None:
        return _scale(_subtract(vectors[after], vectors[index]), 1.0 / (times[after] - times[index]))
    if before is not None:
        return _scale(_subtract(vectors[index], vectors[before]), 1.0 / (times[index] - times[before]))
    return (0.0, 0.0, 0.0)
    ####


def _body_angular_velocity_difference(
    orientations: Sequence[QuaternionWxyz],
    times: Sequence[float],
    index: int,
) -> Vector3:
    """Differentiate ECEF-from-body quaternions into a body-frame rate."""

    if len(orientations) == 1:
        return (0.0, 0.0, 0.0)
    before = next((candidate for candidate in range(index - 1, -1, -1) if times[index] - times[candidate] > _EPSILON), None)
    after = next((candidate for candidate in range(index + 1, len(orientations)) if times[candidate] - times[index] > _EPSILON), None)
    if before is not None and after is not None:
        quaternion_rate = _scale_quaternion(
            _subtract_quaternion(orientations[after], orientations[before]),
            1.0 / (times[after] - times[before]),
        )
    elif after is not None:
        quaternion_rate = _scale_quaternion(
            _subtract_quaternion(orientations[after], orientations[index]),
            1.0 / (times[after] - times[index]),
        )
    elif before is not None:
        quaternion_rate = _scale_quaternion(
            _subtract_quaternion(orientations[index], orientations[before]),
            1.0 / (times[index] - times[before]),
        )
    else:
        return (0.0, 0.0, 0.0)
    body_rate_quaternion = _quaternion_multiply(_quaternion_conjugate(orientations[index]), quaternion_rate)
    return (2.0 * body_rate_quaternion[1], 2.0 * body_rate_quaternion[2], 2.0 * body_rate_quaternion[3])
    ####


def _velocity_aligned_orientation(
    position: Vector3,
    velocity: Vector3,
    *,
    previous: QuaternionWxyz | None,
) -> tuple[QuaternionWxyz, EcefOrientationKind]:
    forward = _normalize(velocity)
    if forward is None:
        if previous is not None:
            return previous, "held_kinematic_orientation"
        forward = _local_north(position)
        return _quaternion_from_axes(forward, _local_right(position, forward), _negate(_normalize_or(position, (1.0, 0.0, 0.0)))), "local_ned_reference"
    down = _negate(_normalize_or(position, (1.0, 0.0, 0.0)))
    right = _normalize(_cross(down, forward))
    if right is None:
        right = _local_right(position, forward)
    down = _normalize_or(_cross(forward, right), down)
    orientation = _quaternion_from_axes(forward, right, down)
    if previous is not None and _dot(orientation, previous) < 0.0:
        orientation = tuple(-value for value in orientation)  # type: ignore[assignment]
    return orientation, "kinematic_velocity_aligned"
    ####


def _local_north(position: Vector3) -> Vector3:
    up = _normalize_or(position, (1.0, 0.0, 0.0))
    candidate = _subtract((0.0, 0.0, 1.0), _scale(up, _dot((0.0, 0.0, 1.0), up)))
    north = _normalize(candidate)
    if north is not None:
        return north
    return (0.0, 1.0, 0.0)
    ####


def _local_right(position: Vector3, forward: Vector3) -> Vector3:
    down = _negate(_normalize_or(position, (1.0, 0.0, 0.0)))
    right = _normalize(_cross(down, forward))
    if right is not None:
        return right
    fallback = _normalize(_cross((0.0, 1.0, 0.0), forward))
    if fallback is not None:
        return fallback
    return (0.0, 1.0, 0.0)
    ####


def _quaternion_from_axes(forward: Vector3, right: Vector3, down: Vector3) -> QuaternionWxyz:
    # Columns are the forward/right/down body axes expressed in ECFC.
    matrix = (
        (forward[0], right[0], down[0]),
        (forward[1], right[1], down[1]),
        (forward[2], right[2], down[2]),
    )
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (0.25 * scale, (m21 - m12) / scale, (m02 - m20) / scale, (m10 - m01) / scale)
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(max(0.0, 1.0 + m00 - m11 - m22)) * 2.0
        quaternion = ((m21 - m12) / scale, 0.25 * scale, (m01 + m10) / scale, (m02 + m20) / scale)
    elif m11 > m22:
        scale = math.sqrt(max(0.0, 1.0 + m11 - m00 - m22)) * 2.0
        quaternion = ((m02 - m20) / scale, (m01 + m10) / scale, 0.25 * scale, (m12 + m21) / scale)
    else:
        scale = math.sqrt(max(0.0, 1.0 + m22 - m00 - m11)) * 2.0
        quaternion = ((m10 - m01) / scale, (m02 + m20) / scale, (m12 + m21) / scale, 0.25 * scale)
    norm = math.sqrt(sum(value * value for value in quaternion))
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]
    ####


def _vector_from_components(values: Mapping[str, Any], groups: Sequence[tuple[str, str, str]]) -> Vector3 | None:
    for identifiers in groups:
        vector = tuple(_finite_number(values.get(identifier)) for identifier in identifiers)
        if all(value is not None for value in vector):
            return (float(vector[0]), float(vector[1]), float(vector[2]))
    return None
    ####


def _vector_value(values: Mapping[str, Any], identifiers: Sequence[str]) -> Vector3 | None:
    for identifier in identifiers:
        raw = values.get(identifier)
        if isinstance(raw, Sequence) and not isinstance(raw, str | bytes) and len(raw) == 3:
            vector = tuple(_finite_number(item) for item in raw)
            if all(value is not None for value in vector):
                return (float(vector[0]), float(vector[1]), float(vector[2]))
    return None
    ####


def _quaternion_value(values: Mapping[str, Any], identifiers: Sequence[str]) -> QuaternionWxyz | None:
    for identifier in identifiers:
        raw = values.get(identifier)
        if isinstance(raw, Sequence) and not isinstance(raw, str | bytes) and len(raw) == 4:
            components = tuple(_finite_number(item) for item in raw)
            if all(component is not None for component in components):
                quaternion = tuple(float(component) for component in components)
                norm = math.sqrt(sum(component * component for component in quaternion))
                if norm > _EPSILON:
                    return tuple(component / norm for component in quaternion)  # type: ignore[return-value]
    return None
    ####


def _quaternion_conjugate(quaternion: QuaternionWxyz) -> QuaternionWxyz:
    return (quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3])
    ####


def _quaternion_multiply(left: QuaternionWxyz, right: QuaternionWxyz) -> QuaternionWxyz:
    left_w, left_x, left_y, left_z = left
    right_w, right_x, right_y, right_z = right
    return (
        left_w * right_w - left_x * right_x - left_y * right_y - left_z * right_z,
        left_w * right_x + left_x * right_w + left_y * right_z - left_z * right_y,
        left_w * right_y - left_x * right_z + left_y * right_w + left_z * right_x,
        left_w * right_z + left_x * right_y - left_y * right_x + left_z * right_w,
    )
    ####


def _subtract_quaternion(left: QuaternionWxyz, right: QuaternionWxyz) -> QuaternionWxyz:
    return tuple(first - second for first, second in zip(left, right, strict=True))  # type: ignore[return-value]
    ####


def _scale_quaternion(quaternion: QuaternionWxyz, factor: float) -> QuaternionWxyz:
    return tuple(component * factor for component in quaternion)  # type: ignore[return-value]
    ####


def _normalize_quaternion(quaternion: QuaternionWxyz) -> QuaternionWxyz:
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm <= _EPSILON or not math.isfinite(norm):
        raise ValueError("standard ECEF orientation requires a finite nonzero quaternion")
    return tuple(component / norm for component in quaternion)  # type: ignore[return-value]
    ####


def _scalar_value(values: Mapping[str, Any], identifiers: Sequence[str]) -> float | None:
    for identifier in identifiers:
        value = _finite_number(values.get(identifier))
        if value is not None:
            return value
    return None
    ####


def _finite_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, int | float):
        return None
    numeric = float(value)
    return numeric if math.isfinite(numeric) else None
    ####


def _first_unit(identifiers: Sequence[str], values: Mapping[str, Any], units: Mapping[str, str | None]) -> str | None:
    for identifier in identifiers:
        if identifier in values:
            return units.get(identifier)
    return None
    ####


def _angle_to_radians(value: float, unit: str | None, identifiers: Sequence[str]) -> float:
    normalized = "" if unit is None else unit.casefold()
    if normalized in {"rad", "radian", "radians"}:
        return value
    if normalized in {"deg", "degree", "degrees"} or any("deg" in identifier for identifier in identifiers):
        return math.radians(value)
    # Existing portable geodetic schemas use degrees.  Retain that convention
    # where a legacy provider did not carry a channel-unit annotation.
    return math.radians(value)
    ####


def _frame_for(identifier: str, frames: Mapping[str, str | None]) -> str | None:
    return frames.get(identifier)
    ####


def _first_present_frame(values: Mapping[str, Any], frames: Mapping[str, str | None], identifiers: Sequence[str]) -> str | None:
    for identifier in identifiers:
        if identifier in values:
            return frames.get(identifier)
    return None
    ####


def _is_ned_frame(frame: str | None) -> bool:
    if frame is None:
        return False
    normalized = frame.casefold()
    return normalized == "ned" or "local_ned" in normalized or normalized.endswith(".ned")
    ####


def _rotate_z(vector: Vector3, angle_rad: float) -> Vector3:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return (cosine * vector[0] - sine * vector[1], sine * vector[0] + cosine * vector[1], vector[2])
    ####


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])
    ####


def _scale(vector: Vector3, factor: float) -> Vector3:
    return (vector[0] * factor, vector[1] * factor, vector[2] * factor)
    ####


def _negate(vector: Vector3) -> Vector3:
    return (-vector[0], -vector[1], -vector[2])
    ####


def _dot(left: Sequence[float], right: Sequence[float]) -> float:
    return sum(first * second for first, second in zip(left, right, strict=True))
    ####


def _cross(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )
    ####


def _normalize(vector: Vector3) -> Vector3 | None:
    norm = math.sqrt(_dot(vector, vector))
    if norm <= _EPSILON or not math.isfinite(norm):
        return None
    return _scale(vector, 1.0 / norm)
    ####


def _normalize_or(vector: Vector3, fallback: Vector3) -> Vector3:
    return _normalize(vector) or fallback
    ####


__all__ = ["StandardEcefState", "project_standard_ecef_samples", "standard_ecef_state_from_values"]
