"""Receiver-level GNSS fix plug-in over committed host truth."""

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


def _tuple3(value: object, name: str) -> tuple[float, float, float]:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite 3-vector")
    return cast(tuple[float, float, float], tuple(float(item) for item in array))
    ####


def _matrix3(value: object, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (3, 3) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    array = array.copy()
    array.setflags(write=False)
    return array
    ####


class GnssFixConfig(BaseModel):
    """Receiver-level position/velocity fix errors in the declared ECI frame."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    position_bias_eci_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity_bias_eci_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    position_stddev_m: float = Field(default=0.0, ge=0.0)
    velocity_stddev_mps: float = Field(default=0.0, ge=0.0)
    outage_probability: float = Field(default=0.0, ge=0.0, le=1.0)

    @field_validator("position_bias_eci_m", "velocity_bias_eci_mps", mode="before")
    @classmethod
    def validate_bias(cls, value: object) -> tuple[float, float, float]:
        return _tuple3(value, "GNSS bias")
        ####

    @field_validator("position_stddev_m", "velocity_stddev_mps", "outage_probability")
    @classmethod
    def validate_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("GNSS error values must be finite")
        return value
        ####
    ####


@dataclass(frozen=True, slots=True)
class GnssFix:
    """One receiver-level ECI position/velocity fix and covariance."""

    position_eci_m: np.ndarray
    velocity_eci_mps: np.ndarray
    position_covariance_eci_m2: np.ndarray
    velocity_covariance_eci_m2ps2: np.ndarray
    fix_status: str = "simulated-3d"
    frame_id: str = "ECI"

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_eci_m", np.asarray(_tuple3(self.position_eci_m, "GNSS position")))
        object.__setattr__(self, "velocity_eci_mps", np.asarray(_tuple3(self.velocity_eci_mps, "GNSS velocity")))
        object.__setattr__(
            self,
            "position_covariance_eci_m2",
            _matrix3(self.position_covariance_eci_m2, "GNSS position covariance"),
        )
        object.__setattr__(
            self,
            "velocity_covariance_eci_m2ps2",
            _matrix3(self.velocity_covariance_eci_m2ps2, "GNSS velocity covariance"),
        )
        self.position_eci_m.setflags(write=False)
        self.velocity_eci_mps.setflags(write=False)
        if np.any(np.diag(self.position_covariance_eci_m2) < 0.0) or np.any(
            np.diag(self.velocity_covariance_eci_m2ps2) < 0.0
        ):
            raise ValueError("GNSS covariance diagonals must be nonnegative")
        for covariance in (self.position_covariance_eci_m2, self.velocity_covariance_eci_m2ps2):
            if not np.allclose(covariance, covariance.T, atol=1.0e-12) or np.linalg.eigvalsh(covariance).min() < -1.0e-12:
                raise ValueError("GNSS covariance must be symmetric positive semidefinite")
        if not self.fix_status.strip() or self.frame_id != "ECI":
            raise ValueError("GNSS fix requires a nonempty status and the explicit ECI frame")
        ####
    ####


class GnssFixSensor:
    """Apply receiver-level bias, white error, outage, and fix timing."""

    def __init__(self, config: GnssFixConfig) -> None:
        self.config = config
        self.sample_count = 0
        self.provenance = {
            "provider": "gnss-fix",
            "fidelity": "receiver-level-fix",
            "frame": "ECI",
            "raw_observables": False,
            "constellation_geometry": False,
        }

    def sample_request(self, request: SensorSampleRequest) -> MeasurementPacket[GnssFix]:
        if request.point is None:
            raise ValueError("GNSS fix sampling currently requires instantaneous committed context")
        self.sample_count += 1
        if self.config.outage_probability > 0.0 and request.rng.random() < self.config.outage_probability:
            return MeasurementPacket(
                request.sampled_at_s,
                request.sampled_at_s,
                request.interval_start_s,
                None,
                valid=False,
                schema_id="taoryx.gnss.fix/v1",
                port="fix",
                invalid_reason="simulated-outage",
            )
        host = request.point.host
        position_bias = np.asarray(self.config.position_bias_eci_m)
        velocity_bias = np.asarray(self.config.velocity_bias_eci_mps)
        position = host.position_eci_m + position_bias + request.rng.normal(
            0.0,
            self.config.position_stddev_m,
            3,
        )
        velocity = host.velocity_eci_mps + velocity_bias + request.rng.normal(
            0.0,
            self.config.velocity_stddev_mps,
            3,
        )
        payload = GnssFix(
            position,
            velocity,
            np.eye(3) * self.config.position_stddev_m**2,
            np.eye(3) * self.config.velocity_stddev_mps**2,
        )
        return MeasurementPacket(
            request.sampled_at_s,
            request.sampled_at_s,
            request.interval_start_s,
            payload,
            schema_id="taoryx.gnss.fix/v1",
            port="fix",
        )
        ####

    def reset(self) -> None:
        self.sample_count = 0
        ####

    def snapshot(self) -> Mapping[str, object]:
        return {
            "schema_version": 1,
            "adapter_type": "taoryx.GnssFixSensor",
            "sample_count": self.sample_count,
        }
        ####

    def restore(self, checkpoint: Mapping[str, object]) -> None:
        if checkpoint.get("schema_version") != 1 or checkpoint.get("adapter_type") != "taoryx.GnssFixSensor":
            raise ValueError("GNSS fix checkpoint does not match the configured sensor")
        sample_count = int(cast(Any, checkpoint.get("sample_count", 0)))
        if sample_count < 0:
            raise ValueError("GNSS fix checkpoint sample_count must be nonnegative")
        self.sample_count = sample_count
        ####
    ####


def _build_gnss_fix(config: BaseModel, context: SensorBuildContext) -> GnssFixSensor:
    del context
    return GnssFixSensor(cast(GnssFixConfig, config))
    ####


GNSS_SENSOR_PLUGINS = (
    SensorPluginDescriptor(
        SensorPluginManifest(
            kind="gnss-fix",
            family="gnss",
            language_kinds=frozenset({"gnss", "gps"}),
            clock_kind="gnss-receiver",
            outputs=(SensorOutputPort("fix", "taoryx.gnss.fix/v1"),),
            required_truth=frozenset({"position", "velocity"}),
            supported_truth_modes=frozenset({"vehicle", "translation-only", "pseudo-6dof", "hybrid-6dof"}),
            default_cadence_s=1.0,
            description="Receiver-level ECI position/velocity fix model.",
        ),
        GnssFixConfig,
        _build_gnss_fix,
    ),
)


def _encode_gnss_fix(value: object) -> Mapping[str, object]:
    payload = cast(GnssFix, value)
    return {
        "kind": "gnss_fix",
        "position_eci_m": payload.position_eci_m.tolist(),
        "velocity_eci_mps": payload.velocity_eci_mps.tolist(),
        "position_covariance_eci_m2": payload.position_covariance_eci_m2.tolist(),
        "velocity_covariance_eci_m2ps2": payload.velocity_covariance_eci_m2ps2.tolist(),
        "fix_status": payload.fix_status,
        "frame_id": payload.frame_id,
    }
    ####


def _decode_gnss_fix(value: Mapping[str, object]) -> GnssFix:
    return GnssFix(
        np.asarray(value["position_eci_m"], dtype=float),
        np.asarray(value["velocity_eci_mps"], dtype=float),
        np.asarray(value["position_covariance_eci_m2"], dtype=float),
        np.asarray(value["velocity_covariance_eci_m2ps2"], dtype=float),
        str(value.get("fix_status", "simulated-3d")),
        str(value.get("frame_id", "ECI")),
    )
    ####


GNSS_PAYLOAD_CODECS = (
    PayloadCodec(
        "taoryx.gnss.fix/v1",
        GnssFix,
        _encode_gnss_fix,
        _decode_gnss_fix,
        {
            "measurement": "receiver-fix",
            "frame": "ECI",
            "position_unit": "m",
            "velocity_unit": "m/s",
            "raw_observables": False,
        },
        legacy_kind="gnss_fix",
    ),
)


__all__ = [
    "GNSS_PAYLOAD_CODECS",
    "GNSS_SENSOR_PLUGINS",
    "GnssFix",
    "GnssFixConfig",
    "GnssFixSensor",
]
####
