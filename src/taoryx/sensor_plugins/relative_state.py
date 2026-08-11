"""Direct committed-geometry relative-state tracking sensor.

The tracker is intentionally a geometric sensor model.  It projects a
declared scene entity into the sensor frame and makes its range, angles,
relative velocity, closing speed, and line-of-sight rate explicit.  It does
not claim radar propagation, target signatures, seeker gimbal dynamics, or a
fire-control track manager; those belong to a model that composes this
measurement with the relevant specialised state.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, cast

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, field_validator

from taoryx.sensor_api import (
    MeasurementPacket,
    PayloadCodec,
    SensorBuildContext,
    SensorContext,
    SensorOutputPort,
    SensorPluginDescriptor,
    SensorPluginManifest,
    SensorSampleRequest,
)


class RelativeStateTrackerConfig(BaseModel):
    """Configuration for a direct, typed relative-state measurement.

    The default full sphere field of regard permits a model to layer its own
    gimbal and acquisition logic above a raw sensor-frame observation.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    target_id: str
    horizontal_fov_rad: float = Field(default=math.tau, gt=0.0, le=math.tau)
    vertical_fov_rad: float = Field(default=math.pi, gt=0.0, le=math.pi)
    maximum_range_m: float | None = Field(default=None, gt=0.0)
    range_bias_m: float = 0.0
    range_noise_stddev_m: float = Field(default=0.0, ge=0.0)
    azimuth_bias_rad: float = 0.0
    elevation_bias_rad: float = 0.0
    angular_noise_stddev_rad: float = Field(default=0.0, ge=0.0)
    relative_velocity_bias_sensor_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    relative_velocity_noise_stddev_mps: float = Field(default=0.0, ge=0.0)

    @field_validator("target_id")
    @classmethod
    def validate_target_id(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("relative-state target_id must not be empty")
        return value
        ####

    @field_validator(
        "range_bias_m",
        "range_noise_stddev_m",
        "azimuth_bias_rad",
        "elevation_bias_rad",
        "angular_noise_stddev_rad",
        "relative_velocity_noise_stddev_mps",
    )
    @classmethod
    def validate_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("relative-state tracker error values must be finite")
        return value
        ####

    @field_validator("relative_velocity_bias_sensor_mps", mode="before")
    @classmethod
    def validate_velocity_bias(cls, value: object) -> tuple[float, float, float]:
        vector = np.asarray(value, dtype=float)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError("relative_velocity_bias_sensor_mps must be a finite 3-vector")
        return float(vector[0]), float(vector[1]), float(vector[2])
        ####

    ####


@dataclass(frozen=True, slots=True)
class RelativeStateTrack:
    """One sensor-frame relative-state observation of a declared entity."""

    target_id: str
    range_m: float
    azimuth_rad: float
    elevation_rad: float
    closing_speed_mps: float
    relative_position_sensor_m: tuple[float, float, float]
    relative_velocity_sensor_mps: tuple[float, float, float]
    unit_los_sensor: tuple[float, float, float]
    line_of_sight_rate_sensor_rad_s: tuple[float, float, float]
    frame_id: str = "sensor"

    def __post_init__(self) -> None:
        values = (
            self.range_m,
            self.azimuth_rad,
            self.elevation_rad,
            self.closing_speed_mps,
            *self.relative_position_sensor_m,
            *self.relative_velocity_sensor_mps,
            *self.unit_los_sensor,
            *self.line_of_sight_rate_sensor_rad_s,
        )
        if not self.target_id.strip() or not self.frame_id.strip():
            raise ValueError("relative-state track target_id and frame_id must not be empty")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("relative-state track values must be finite")
        if self.range_m <= 0.0:
            raise ValueError("relative-state track range_m must be positive")
        unit = np.asarray(self.unit_los_sensor, dtype=float)
        if not np.isclose(float(np.linalg.norm(unit)), 1.0, atol=1.0e-8):
            raise ValueError("relative-state track unit_los_sensor must be unit length")
        ####

    ####


def relative_state_track_from_context(
    context: SensorContext,
    config: RelativeStateTrackerConfig,
    *,
    body_from_sensor: np.ndarray | None = None,
    lever_arm_body_m: np.ndarray | None = None,
    rng: np.random.Generator | None = None,
) -> RelativeStateTrack | str:
    """Return a track payload or an explicit invalid-measurement reason.

    This pure helper is shared by the registered plug-in and source adapters.
    Calling it does not turn the host coordinates into an Earth model: it only
    requires one consistent, right-handed Euclidean truth basis.
    """

    target = context.entities.get(config.target_id)
    if target is None:
        return "target-unavailable"
    orientation = context.host.orientation_eci_from_body
    if orientation is None:
        return "host-orientation-unavailable"
    return relative_state_track_from_geometry(
        target_id=config.target_id,
        host_position_world_m=context.host.position_eci_m,
        host_velocity_world_mps=context.host.velocity_eci_mps,
        orientation_world_from_body=orientation,
        target_position_world_m=target.position_eci_m,
        target_velocity_world_mps=target.velocity_eci_mps,
        host_body_rate_rad_s=context.host.angular_rate_body_radps,
        config=config,
        body_from_sensor=body_from_sensor,
        lever_arm_body_m=lever_arm_body_m,
        rng=rng,
    )
    ####


def relative_state_track_from_geometry(
    *,
    target_id: str,
    host_position_world_m: np.ndarray,
    host_velocity_world_mps: np.ndarray,
    orientation_world_from_body: np.ndarray,
    target_position_world_m: np.ndarray,
    target_velocity_world_mps: np.ndarray,
    host_body_rate_rad_s: np.ndarray | None,
    config: RelativeStateTrackerConfig,
    body_from_sensor: np.ndarray | None = None,
    lever_arm_body_m: np.ndarray | None = None,
    rng: np.random.Generator | None = None,
) -> RelativeStateTrack | str:
    """Evaluate the native projection from already accepted truth geometry.

    This is the lower-overhead entry point for a source-ordered family that
    has already established its own committed truth boundary.  It has the
    exact same projection and corruption semantics as the registered sensor;
    only ``SensorContext`` construction is outside its hot execution loop.
    """

    sensor_from_body = np.eye(3, dtype=float) if body_from_sensor is None else np.asarray(body_from_sensor, dtype=float)
    lever_arm = np.zeros(3, dtype=float) if lever_arm_body_m is None else np.asarray(lever_arm_body_m, dtype=float)
    orientation = np.asarray(orientation_world_from_body, dtype=float)
    sensor_position = np.asarray(host_position_world_m, dtype=float) + orientation @ lever_arm
    sensor_velocity = np.asarray(host_velocity_world_mps, dtype=float).copy()
    if host_body_rate_rad_s is not None:
        sensor_velocity += orientation @ np.cross(host_body_rate_rad_s, lever_arm)
    sensor_from_world = sensor_from_body.T @ orientation.T
    relative_position = sensor_from_world @ (np.asarray(target_position_world_m, dtype=float) - sensor_position)
    range_m = float(np.linalg.norm(relative_position))
    if range_m <= 0.0:
        return "coincident-target"
    if config.maximum_range_m is not None and range_m > config.maximum_range_m:
        return "outside-range"
    unit_los = relative_position / range_m
    forward = float(unit_los[0])
    azimuth = math.atan2(float(unit_los[1]), forward)
    elevation = math.atan2(float(unit_los[2]), math.hypot(forward, float(unit_los[1])))
    if abs(azimuth) > 0.5 * config.horizontal_fov_rad or abs(elevation) > 0.5 * config.vertical_fov_rad:
        return "outside-field-of-view"
    generator = rng if rng is not None else np.random.default_rng()
    measured_range = max(0.0, range_m + config.range_bias_m + float(generator.normal(0.0, config.range_noise_stddev_m)))
    if measured_range <= 0.0:
        return "nonpositive-measured-range"
    measured_azimuth = azimuth + config.azimuth_bias_rad + float(generator.normal(0.0, config.angular_noise_stddev_rad))
    measured_elevation = elevation + config.elevation_bias_rad + float(generator.normal(0.0, config.angular_noise_stddev_rad))
    measured_unit = np.asarray(
        (
            math.cos(measured_elevation) * math.cos(measured_azimuth),
            math.cos(measured_elevation) * math.sin(measured_azimuth),
            math.sin(measured_elevation),
        ),
        dtype=float,
    )
    relative_velocity = sensor_from_world @ (np.asarray(target_velocity_world_mps, dtype=float) - sensor_velocity)
    velocity_noise = generator.normal(0.0, config.relative_velocity_noise_stddev_mps, size=3)
    measured_velocity = relative_velocity + np.asarray(config.relative_velocity_bias_sensor_mps, dtype=float) + velocity_noise
    line_of_sight_rate = np.cross(measured_unit, measured_velocity) / measured_range
    return RelativeStateTrack(
        target_id=target_id,
        range_m=measured_range,
        azimuth_rad=measured_azimuth,
        elevation_rad=measured_elevation,
        closing_speed_mps=-float(measured_unit @ measured_velocity),
        relative_position_sensor_m=_tuple3(measured_unit * measured_range),
        relative_velocity_sensor_mps=_tuple3(measured_velocity),
        unit_los_sensor=_tuple3(measured_unit),
        line_of_sight_rate_sensor_rad_s=_tuple3(line_of_sight_rate),
    )
    ####


def _tuple3(vector: np.ndarray) -> tuple[float, float, float]:
    return float(vector[0]), float(vector[1]), float(vector[2])
    ####


class RelativeStateTrackerSensor:
    """Context-aware wrapper for the direct relative-state tracker."""

    def __init__(self, config: RelativeStateTrackerConfig, context: SensorBuildContext) -> None:
        self.config = config
        self.body_from_sensor = context.body_from_sensor
        self.lever_arm_body_m = context.lever_arm_body_m
        self.sample_count = 0
        self.provenance = {
            "provider": "relative-state-track",
            "fidelity": "direct-committed-geometry",
            "observable": "sensor-frame-relative-state",
            "truth_position_output": False,
            "target_selector": config.target_id,
            "claim_boundary": "No propagation, target signature, gimbal, tracker, or fire-control state is represented.",
        }

    def sample_request(self, request: SensorSampleRequest) -> MeasurementPacket[RelativeStateTrack]:
        if request.point is None:
            raise ValueError("relative-state tracker requires instantaneous committed context")
        self.sample_count += 1
        payload_or_reason = relative_state_track_from_context(
            request.point,
            self.config,
            body_from_sensor=self.body_from_sensor,
            lever_arm_body_m=self.lever_arm_body_m,
            rng=request.rng,
        )
        if isinstance(payload_or_reason, str):
            return self._invalid(request, payload_or_reason)
        return MeasurementPacket(
            request.sampled_at_s,
            request.sampled_at_s,
            request.interval_start_s,
            payload_or_reason,
            schema_id="taoryx.tracking.relative-state/v1",
            port="track",
        )
        ####

    @staticmethod
    def _invalid(request: SensorSampleRequest, reason: str) -> MeasurementPacket[RelativeStateTrack]:
        return MeasurementPacket(
            request.sampled_at_s,
            request.sampled_at_s,
            request.interval_start_s,
            None,
            valid=False,
            schema_id="taoryx.tracking.relative-state/v1",
            port="track",
            invalid_reason=reason,
        )
        ####

    def reset(self) -> None:
        self.sample_count = 0
        ####

    def snapshot(self) -> Mapping[str, object]:
        return {
            "schema_version": 1,
            "adapter_type": "taoryx.RelativeStateTrackerSensor",
            "sample_count": self.sample_count,
        }
        ####

    def restore(self, checkpoint: Mapping[str, object]) -> None:
        if checkpoint.get("schema_version") != 1 or checkpoint.get("adapter_type") != "taoryx.RelativeStateTrackerSensor":
            raise ValueError("relative-state tracker checkpoint does not match the configured sensor")
        sample_count = int(cast(Any, checkpoint.get("sample_count", 0)))
        if sample_count < 0:
            raise ValueError("relative-state tracker checkpoint sample_count must be nonnegative")
        self.sample_count = sample_count
        ####

    ####


def _build_relative_state_tracker(config: BaseModel, context: SensorBuildContext) -> RelativeStateTrackerSensor:
    return RelativeStateTrackerSensor(cast(RelativeStateTrackerConfig, config), context)
    ####


RELATIVE_STATE_SENSOR_PLUGINS = (
    SensorPluginDescriptor(
        SensorPluginManifest(
            kind="relative-state-track",
            family="tracking",
            language_kinds=frozenset({"radar", "seeker", "tracking", "rf", "infrared"}),
            clock_kind="tracking",
            outputs=(SensorOutputPort("track", "taoryx.tracking.relative-state/v1"),),
            required_truth=frozenset({"position", "velocity", "orientation", "entities"}),
            supported_truth_modes=frozenset({"vehicle", "pseudo-6dof", "hybrid-6dof"}),
            default_cadence_s=0.02,
            description="Direct sensor-frame relative-state projection for a declared scene entity.",
        ),
        RelativeStateTrackerConfig,
        _build_relative_state_tracker,
    ),
)


def _encode_track(value: object) -> Mapping[str, object]:
    payload = cast(RelativeStateTrack, value)
    return {
        "kind": "relative_state_track",
        "target_id": payload.target_id,
        "range_m": payload.range_m,
        "azimuth_rad": payload.azimuth_rad,
        "elevation_rad": payload.elevation_rad,
        "closing_speed_mps": payload.closing_speed_mps,
        "relative_position_sensor_m": list(payload.relative_position_sensor_m),
        "relative_velocity_sensor_mps": list(payload.relative_velocity_sensor_mps),
        "unit_los_sensor": list(payload.unit_los_sensor),
        "line_of_sight_rate_sensor_rad_s": list(payload.line_of_sight_rate_sensor_rad_s),
        "frame_id": payload.frame_id,
    }
    ####


def _decode_vector(value: Mapping[str, object], key: str) -> tuple[float, float, float]:
    vector = np.asarray(value[key], dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"relative-state track {key} must be a finite 3-vector")
    return float(vector[0]), float(vector[1]), float(vector[2])
    ####


def _decode_track(value: Mapping[str, object]) -> RelativeStateTrack:
    return RelativeStateTrack(
        target_id=str(value["target_id"]),
        range_m=float(cast(Any, value["range_m"])),
        azimuth_rad=float(cast(Any, value["azimuth_rad"])),
        elevation_rad=float(cast(Any, value["elevation_rad"])),
        closing_speed_mps=float(cast(Any, value["closing_speed_mps"])),
        relative_position_sensor_m=_decode_vector(value, "relative_position_sensor_m"),
        relative_velocity_sensor_mps=_decode_vector(value, "relative_velocity_sensor_mps"),
        unit_los_sensor=_decode_vector(value, "unit_los_sensor"),
        line_of_sight_rate_sensor_rad_s=_decode_vector(value, "line_of_sight_rate_sensor_rad_s"),
        frame_id=str(value.get("frame_id", "sensor")),
    )
    ####


RELATIVE_STATE_PAYLOAD_CODECS = (
    PayloadCodec(
        "taoryx.tracking.relative-state/v1",
        RelativeStateTrack,
        _encode_track,
        _decode_track,
        {
            "measurement": "relative-state-track",
            "frame": "sensor",
            "range_unit": "m",
            "velocity_unit": "m/s",
            "angular_rate_unit": "rad/s",
            "truth_position_output": False,
            "claim_boundary": "Direct geometry only; no propagation, signature, gimbal, tracker, or fire-control state.",
        },
        legacy_kind="relative_state_track",
    ),
)


__all__ = [
    "RELATIVE_STATE_PAYLOAD_CODECS",
    "RELATIVE_STATE_SENSOR_PLUGINS",
    "RelativeStateTrack",
    "RelativeStateTrackerConfig",
    "RelativeStateTrackerSensor",
    "relative_state_track_from_geometry",
    "relative_state_track_from_context",
]
####
