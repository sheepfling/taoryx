"""Low-fidelity infrared bearing and point-source sensor plug-ins.

These models deliberately emit sensor-frame observables, not truth-derived
world positions. Higher-fidelity focal-plane implementations can share the
family while emitting array-frame or detection payload schemas.
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
    SensorOutputPort,
    SensorPluginDescriptor,
    SensorPluginManifest,
    SensorSampleRequest,
)


class BearingOnlyIrConfig(BaseModel):
    """Configuration for a direct committed-geometry IR bearing model."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    target_id: str
    horizontal_fov_rad: float = Field(default=math.radians(60.0), gt=0.0, le=math.pi)
    vertical_fov_rad: float = Field(default=math.radians(45.0), gt=0.0, le=math.pi)
    maximum_range_m: float | None = Field(default=None, gt=0.0)
    azimuth_bias_rad: float = 0.0
    elevation_bias_rad: float = 0.0
    angular_noise_stddev_rad: float = Field(default=0.0, ge=0.0)

    @field_validator("target_id")
    @classmethod
    def validate_target_id(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("IR target_id must not be empty")
        return value
        ####

    @field_validator("azimuth_bias_rad", "elevation_bias_rad", "angular_noise_stddev_rad")
    @classmethod
    def validate_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("IR angular error values must be finite")
        return value
        ####
    ####


class PointSourceFocalPlaneConfig(BaseModel):
    """Configuration for a pinhole point-source centroid measurement."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    target_id: str
    width_pixels: int = Field(default=640, gt=1)
    height_pixels: int = Field(default=480, gt=1)
    horizontal_fov_rad: float = Field(default=math.radians(60.0), gt=0.0, le=math.pi)
    vertical_fov_rad: float = Field(default=math.radians(45.0), gt=0.0, le=math.pi)
    maximum_range_m: float | None = Field(default=None, gt=0.0)
    centroid_bias_pixels: tuple[float, float] = (0.0, 0.0)
    centroid_noise_stddev_pixels: float = Field(default=0.0, ge=0.0)

    @field_validator("target_id")
    @classmethod
    def validate_target_id(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("focal-plane target_id must not be empty")
        return value
        ####

    @field_validator("centroid_bias_pixels", mode="before")
    @classmethod
    def validate_centroid_bias(cls, value: object) -> tuple[float, float]:
        array = np.asarray(value, dtype=float)
        if array.shape != (2,) or not np.all(np.isfinite(array)):
            raise ValueError("focal-plane centroid_bias_pixels must be a finite 2-vector")
        return float(array[0]), float(array[1])
        ####

    @field_validator("centroid_noise_stddev_pixels")
    @classmethod
    def validate_centroid_noise(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("focal-plane centroid noise must be finite")
        return value
        ####
    ####


@dataclass(frozen=True, slots=True)
class BearingDetection:
    """One unresolved sensor-frame angular detection."""

    azimuth_rad: float
    elevation_rad: float
    covariance_rad2: tuple[tuple[float, float], tuple[float, float]]
    frame_id: str = "sensor"

    def __post_init__(self) -> None:
        values = (
            self.azimuth_rad,
            self.elevation_rad,
            self.covariance_rad2[0][0],
            self.covariance_rad2[0][1],
            self.covariance_rad2[1][0],
            self.covariance_rad2[1][1],
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("bearing detection values must be finite")
        if self.covariance_rad2[0][0] < 0.0 or self.covariance_rad2[1][1] < 0.0:
            raise ValueError("bearing detection covariance diagonal must be nonnegative")
        covariance: np.ndarray = np.asarray(self.covariance_rad2, dtype=float)
        if not np.allclose(covariance, covariance.T, atol=1.0e-15) or np.linalg.eigvalsh(covariance).min() < -1.0e-15:
            raise ValueError("bearing detection covariance must be symmetric positive semidefinite")
        if not self.frame_id.strip():
            raise ValueError("bearing detection frame_id must not be empty")
        ####
    ####


@dataclass(frozen=True, slots=True)
class FocalPlaneDetection:
    """One unresolved point-source centroid in sensor focal-plane pixels."""

    column_px: float
    row_px: float
    covariance_px2: tuple[tuple[float, float], tuple[float, float]]
    image_size_pixels: tuple[int, int]
    frame_id: str = "sensor-focal-plane"

    def __post_init__(self) -> None:
        covariance: np.ndarray = np.asarray(self.covariance_px2, dtype=float)
        if not math.isfinite(self.column_px) or not math.isfinite(self.row_px):
            raise ValueError("focal-plane centroid values must be finite")
        if covariance.shape != (2, 2) or not np.all(np.isfinite(covariance)):
            raise ValueError("focal-plane covariance must be finite and 2x2")
        if not np.allclose(covariance, covariance.T, atol=1.0e-15) or np.linalg.eigvalsh(covariance).min() < -1.0e-15:
            raise ValueError("focal-plane covariance must be symmetric positive semidefinite")
        if len(self.image_size_pixels) != 2 or any(dimension <= 1 for dimension in self.image_size_pixels):
            raise ValueError("focal-plane image dimensions must exceed one pixel")
        if not self.frame_id.strip():
            raise ValueError("focal-plane frame_id must not be empty")
        ####
    ####


class BearingOnlyIrSensor:
    """Project one selected scene entity into noisy sensor-frame angles."""

    def __init__(self, config: BearingOnlyIrConfig, context: SensorBuildContext) -> None:
        self.config = config
        self.body_from_sensor = context.body_from_sensor
        self.lever_arm_body_m = context.lever_arm_body_m
        self.sample_count = 0
        self.provenance = {
            "provider": "ir-bearing",
            "fidelity": "bearing-only",
            "observable": "sensor-frame-azimuth-elevation",
            "truth_position_output": False,
            "target_selector": config.target_id,
        }

    def sample_request(self, request: SensorSampleRequest) -> MeasurementPacket[BearingDetection]:
        if request.point is None:
            raise ValueError("bearing-only IR currently requires instantaneous committed context")
        self.sample_count += 1
        context = request.point
        target = context.entities.get(self.config.target_id)
        if target is None:
            return self._invalid(request, "target-unavailable")
        orientation = context.host.orientation_eci_from_body
        if orientation is None:
            return self._invalid(request, "host-orientation-unavailable")
        sensor_position_eci = context.host.position_eci_m + orientation @ self.lever_arm_body_m
        relative_eci = target.position_eci_m - sensor_position_eci
        range_m = float(np.linalg.norm(relative_eci))
        if range_m <= 0.0:
            return self._invalid(request, "coincident-target")
        if self.config.maximum_range_m is not None and range_m > self.config.maximum_range_m:
            return self._invalid(request, "outside-range")
        line_of_sight_sensor = self.body_from_sensor.T @ orientation.T @ (relative_eci / range_m)
        forward = float(line_of_sight_sensor[0])
        azimuth = math.atan2(float(line_of_sight_sensor[1]), forward)
        elevation = math.atan2(
            float(line_of_sight_sensor[2]),
            math.hypot(forward, float(line_of_sight_sensor[1])),
        )
        if forward <= 0.0 or abs(azimuth) > 0.5 * self.config.horizontal_fov_rad or abs(elevation) > 0.5 * self.config.vertical_fov_rad:
            return self._invalid(request, "outside-field-of-view")
        noise = request.rng.normal(0.0, self.config.angular_noise_stddev_rad, 2)
        measured_azimuth = azimuth + self.config.azimuth_bias_rad + float(noise[0])
        measured_elevation = elevation + self.config.elevation_bias_rad + float(noise[1])
        variance = self.config.angular_noise_stddev_rad**2
        payload = BearingDetection(
            measured_azimuth,
            measured_elevation,
            ((variance, 0.0), (0.0, variance)),
        )
        return MeasurementPacket(
            request.sampled_at_s,
            request.sampled_at_s,
            request.interval_start_s,
            payload,
            schema_id="taoryx.ir.bearing-detection/v1",
            port="detections",
        )
        ####

    @staticmethod
    def _invalid(
        request: SensorSampleRequest,
        reason: str,
    ) -> MeasurementPacket[BearingDetection]:
        return MeasurementPacket(
            request.sampled_at_s,
            request.sampled_at_s,
            request.interval_start_s,
            None,
            valid=False,
            schema_id="taoryx.ir.bearing-detection/v1",
            port="detections",
            invalid_reason=reason,
        )
        ####

    def reset(self) -> None:
        self.sample_count = 0
        ####

    def snapshot(self) -> Mapping[str, object]:
        return {
            "schema_version": 1,
            "adapter_type": "taoryx.BearingOnlyIrSensor",
            "sample_count": self.sample_count,
        }
        ####

    def restore(self, checkpoint: Mapping[str, object]) -> None:
        if checkpoint.get("schema_version") != 1 or checkpoint.get("adapter_type") != "taoryx.BearingOnlyIrSensor":
            raise ValueError("bearing-only IR checkpoint does not match the configured sensor")
        sample_count = int(cast(Any, checkpoint.get("sample_count", 0)))
        if sample_count < 0:
            raise ValueError("bearing-only IR checkpoint sample_count must be nonnegative")
        self.sample_count = sample_count
        ####
    ####


class PointSourceFocalPlaneSensor:
    """Project one committed point target to a noisy pinhole centroid."""

    def __init__(self, config: PointSourceFocalPlaneConfig, context: SensorBuildContext) -> None:
        self.config = config
        self.body_from_sensor = context.body_from_sensor
        self.lever_arm_body_m = context.lever_arm_body_m
        self.sample_count = 0
        self.provenance = {
            "provider": "ir-point-source",
            "fidelity": "point-source-focal-plane",
            "observable": "sensor-focal-plane-centroid",
            "raw_frame_output": False,
            "radiometry": False,
            "target_selector": config.target_id,
        }

    def sample_request(self, request: SensorSampleRequest) -> MeasurementPacket[FocalPlaneDetection]:
        if request.point is None:
            raise ValueError("point-source focal-plane sampling requires instantaneous committed context")
        self.sample_count += 1
        context = request.point
        target = context.entities.get(self.config.target_id)
        if target is None:
            return self._invalid(request, "target-unavailable")
        orientation = context.host.orientation_eci_from_body
        if orientation is None:
            return self._invalid(request, "host-orientation-unavailable")
        sensor_position_eci = context.host.position_eci_m + orientation @ self.lever_arm_body_m
        relative_eci = target.position_eci_m - sensor_position_eci
        range_m = float(np.linalg.norm(relative_eci))
        if range_m <= 0.0:
            return self._invalid(request, "coincident-target")
        if self.config.maximum_range_m is not None and range_m > self.config.maximum_range_m:
            return self._invalid(request, "outside-range")
        line_of_sight_sensor = self.body_from_sensor.T @ orientation.T @ (relative_eci / range_m)
        forward = float(line_of_sight_sensor[0])
        if forward <= 0.0:
            return self._invalid(request, "outside-field-of-view")
        focal_x = 0.5 * self.config.width_pixels / math.tan(0.5 * self.config.horizontal_fov_rad)
        focal_y = 0.5 * self.config.height_pixels / math.tan(0.5 * self.config.vertical_fov_rad)
        ideal_column = 0.5 * (self.config.width_pixels - 1) + focal_x * float(line_of_sight_sensor[1]) / forward
        ideal_row = 0.5 * (self.config.height_pixels - 1) - focal_y * float(line_of_sight_sensor[2]) / forward
        if not (0.0 <= ideal_column < self.config.width_pixels and 0.0 <= ideal_row < self.config.height_pixels):
            return self._invalid(request, "outside-field-of-view")
        noise = request.rng.normal(0.0, self.config.centroid_noise_stddev_pixels, 2)
        variance = self.config.centroid_noise_stddev_pixels**2
        payload = FocalPlaneDetection(
            ideal_column + self.config.centroid_bias_pixels[0] + float(noise[0]),
            ideal_row + self.config.centroid_bias_pixels[1] + float(noise[1]),
            ((variance, 0.0), (0.0, variance)),
            (self.config.width_pixels, self.config.height_pixels),
        )
        return MeasurementPacket(
            request.sampled_at_s,
            request.sampled_at_s,
            request.interval_start_s,
            payload,
            schema_id="taoryx.ir.focal-plane-detection/v1",
            port="detections",
        )
        ####

    @staticmethod
    def _invalid(
        request: SensorSampleRequest,
        reason: str,
    ) -> MeasurementPacket[FocalPlaneDetection]:
        return MeasurementPacket(
            request.sampled_at_s,
            request.sampled_at_s,
            request.interval_start_s,
            None,
            valid=False,
            schema_id="taoryx.ir.focal-plane-detection/v1",
            port="detections",
            invalid_reason=reason,
        )
        ####

    def reset(self) -> None:
        self.sample_count = 0
        ####

    def snapshot(self) -> Mapping[str, object]:
        return {
            "schema_version": 1,
            "adapter_type": "taoryx.PointSourceFocalPlaneSensor",
            "sample_count": self.sample_count,
        }
        ####

    def restore(self, checkpoint: Mapping[str, object]) -> None:
        if checkpoint.get("schema_version") != 1 or checkpoint.get("adapter_type") != "taoryx.PointSourceFocalPlaneSensor":
            raise ValueError("point-source focal-plane checkpoint does not match the configured sensor")
        sample_count = int(cast(Any, checkpoint.get("sample_count", 0)))
        if sample_count < 0:
            raise ValueError("point-source focal-plane checkpoint sample_count must be nonnegative")
        self.sample_count = sample_count
        ####
    ####


def _build_bearing_only_ir(config: BaseModel, context: SensorBuildContext) -> BearingOnlyIrSensor:
    return BearingOnlyIrSensor(cast(BearingOnlyIrConfig, config), context)
    ####


def _build_point_source_focal_plane(
    config: BaseModel,
    context: SensorBuildContext,
) -> PointSourceFocalPlaneSensor:
    return PointSourceFocalPlaneSensor(cast(PointSourceFocalPlaneConfig, config), context)
    ####


INFRARED_SENSOR_PLUGINS = (
    SensorPluginDescriptor(
        SensorPluginManifest(
            kind="ir-bearing",
            family="infrared",
            language_kinds=frozenset({"infrared", "camera"}),
            clock_kind="ir-seeker",
            outputs=(SensorOutputPort("detections", "taoryx.ir.bearing-detection/v1"),),
            required_truth=frozenset({"position", "orientation", "entities"}),
            supported_truth_modes=frozenset({"vehicle", "pseudo-6dof", "hybrid-6dof"}),
            default_cadence_s=0.02,
            description="Direct target-geometry projection to a noisy sensor-frame bearing.",
        ),
        BearingOnlyIrConfig,
        _build_bearing_only_ir,
    ),
    SensorPluginDescriptor(
        SensorPluginManifest(
            kind="ir-point-source",
            family="infrared",
            language_kinds=frozenset({"infrared", "camera"}),
            clock_kind="ir-focal-plane",
            outputs=(SensorOutputPort("detections", "taoryx.ir.focal-plane-detection/v1"),),
            required_truth=frozenset({"position", "orientation", "entities"}),
            supported_truth_modes=frozenset({"vehicle", "pseudo-6dof", "hybrid-6dof"}),
            default_cadence_s=0.02,
            description="Pinhole point-target projection to a noisy focal-plane centroid.",
        ),
        PointSourceFocalPlaneConfig,
        _build_point_source_focal_plane,
    ),
)


def _encode_bearing(value: object) -> Mapping[str, object]:
    payload = cast(BearingDetection, value)
    return {
        "kind": "bearing_detection",
        "azimuth_rad": payload.azimuth_rad,
        "elevation_rad": payload.elevation_rad,
        "covariance_rad2": [list(row) for row in payload.covariance_rad2],
        "frame_id": payload.frame_id,
    }
    ####


def _decode_bearing(value: Mapping[str, object]) -> BearingDetection:
    raw_covariance = np.asarray(value["covariance_rad2"], dtype=float)
    if raw_covariance.shape != (2, 2):
        raise ValueError("bearing detection covariance must be 2x2")
    covariance = (
        (float(raw_covariance[0, 0]), float(raw_covariance[0, 1])),
        (float(raw_covariance[1, 0]), float(raw_covariance[1, 1])),
    )
    return BearingDetection(
        float(cast(Any, value["azimuth_rad"])),
        float(cast(Any, value["elevation_rad"])),
        covariance,
        str(value.get("frame_id", "sensor")),
    )
    ####


def _encode_focal_plane(value: object) -> Mapping[str, object]:
    payload = cast(FocalPlaneDetection, value)
    return {
        "kind": "focal_plane_detection",
        "column_px": payload.column_px,
        "row_px": payload.row_px,
        "covariance_px2": [list(row) for row in payload.covariance_px2],
        "image_size_pixels": list(payload.image_size_pixels),
        "frame_id": payload.frame_id,
    }
    ####


def _decode_focal_plane(value: Mapping[str, object]) -> FocalPlaneDetection:
    raw_covariance = np.asarray(value["covariance_px2"], dtype=float)
    raw_size = np.asarray(value["image_size_pixels"], dtype=int)
    if raw_covariance.shape != (2, 2) or raw_size.shape != (2,):
        raise ValueError("focal-plane detection requires 2x2 covariance and two image dimensions")
    return FocalPlaneDetection(
        float(cast(Any, value["column_px"])),
        float(cast(Any, value["row_px"])),
        (
            (float(raw_covariance[0, 0]), float(raw_covariance[0, 1])),
            (float(raw_covariance[1, 0]), float(raw_covariance[1, 1])),
        ),
        (int(raw_size[0]), int(raw_size[1])),
        str(value.get("frame_id", "sensor-focal-plane")),
    )
    ####


INFRARED_PAYLOAD_CODECS = (
    PayloadCodec(
        "taoryx.ir.bearing-detection/v1",
        BearingDetection,
        _encode_bearing,
        _decode_bearing,
        {
            "measurement": "bearing",
            "frame": "sensor",
            "angle_unit": "rad",
            "truth_position_output": False,
        },
        legacy_kind="bearing_detection",
    ),
    PayloadCodec(
        "taoryx.ir.focal-plane-detection/v1",
        FocalPlaneDetection,
        _encode_focal_plane,
        _decode_focal_plane,
        {
            "measurement": "point-source-centroid",
            "frame": "sensor-focal-plane",
            "coordinate_unit": "pixel",
            "raw_frame_output": False,
            "truth_position_output": False,
        },
        legacy_kind="focal_plane_detection",
    ),
)


__all__ = [
    "BearingDetection",
    "BearingOnlyIrConfig",
    "BearingOnlyIrSensor",
    "FocalPlaneDetection",
    "INFRARED_PAYLOAD_CODECS",
    "INFRARED_SENSOR_PLUGINS",
    "PointSourceFocalPlaneConfig",
    "PointSourceFocalPlaneSensor",
]
####
