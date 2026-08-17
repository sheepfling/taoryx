"""Versioned bridge from interceptor tiers to Taoryx sensor plug-ins."""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass, replace
from typing import Any, Protocol, cast

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, computed_field, model_validator

from taoryx.runtime.sensor_contracts import SensorProviderConfig
from taoryx.sensor_api import (
    MeasurementPacket,
    SensorBuildContext,
    SensorContext,
    SensorSampleRequest,
    TruthPoint,
    packet_from_record,
    packet_to_record,
    parse_measurement_packet_record,
    sensor_plugin_registry,
)
from taoryx.sensor_plugins.relative_state import RelativeStateTrack
from taoryx.sensors import AccelerationIncrement, ImuIncrement

DEFAULT_SENSOR_SUITE_ID = "taoryx.sensors.interceptor-navigation.standard-v1"
DEFAULT_SENSOR_SUITE_VERSION = "1.1.0"
TARGET_TRACK_ENTITY_ID = "guidance-target"


class TranslationAccelerationSensor(Protocol):
    """Checkpointable translation-only sensor accepted by the point-mass tier."""

    provenance: Mapping[str, object]

    def reset(self) -> None: ...

    def snapshot(self) -> Mapping[str, object]: ...

    def restore(self, checkpoint: Mapping[str, object]) -> None: ...

    def sample(self, truth: TruthPoint) -> MeasurementPacket[AccelerationIncrement]: ...


class ImuSensor(Protocol):
    """Checkpointable inertial sensor accepted by the pseudo-6DOF tier."""

    provenance: Mapping[str, object]

    def reset(self) -> None: ...

    def snapshot(self) -> Mapping[str, object]: ...

    def restore(self, checkpoint: Mapping[str, object]) -> None: ...

    def sample(self, truth: TruthPoint) -> MeasurementPacket[ImuIncrement]: ...


class TargetTrackSensor(Protocol):
    """Checkpointable standard relative-state sensor used by target guidance."""

    provenance: Mapping[str, object]
    delivery_fresh: bool

    def reset(self) -> None: ...

    def snapshot(self) -> Mapping[str, object]: ...

    def restore(self, checkpoint: Mapping[str, object]) -> None: ...

    def sample_context(self, context: SensorContext) -> MeasurementPacket[RelativeStateTrack]: ...

    def held_packet(self, current_time_s: float) -> MeasurementPacket[RelativeStateTrack]: ...


@dataclass(frozen=True, slots=True)
class TargetTrackTelemetry:
    """Stable sample projection shared by both interceptor fidelity tiers."""

    applicable: bool = False
    valid: bool = False
    sampled_at_s: float = 0.0
    available_at_s: float = 0.0
    latency_s: float = 0.0
    delivery_fresh: bool = False
    sequence: int = -1
    schema_id: str = "taoryx.tracking.relative-state/v1"
    invalid_reason: str = "not-applicable"
    target_id: str = TARGET_TRACK_ENTITY_ID
    frame_id: str = "sensor"
    range_m: float = 0.0
    azimuth_rad: float = 0.0
    elevation_rad: float = 0.0
    closing_speed_mps: float = 0.0
    relative_position_sensor_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    relative_velocity_sensor_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    line_of_sight_rate_sensor_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)

    ####


def target_track_telemetry(
    packet: MeasurementPacket[RelativeStateTrack],
    *,
    delivery_fresh: bool,
) -> TargetTrackTelemetry:
    """Convert one registered packet into stable Composition-facing fields."""

    payload = packet.payload if packet.valid else None
    if payload is None:
        return TargetTrackTelemetry(
            applicable=True,
            sampled_at_s=packet.sampled_at_s,
            available_at_s=packet.available_at_s,
            latency_s=packet.available_at_s - packet.sampled_at_s,
            delivery_fresh=delivery_fresh,
            sequence=-1 if packet.sequence is None else packet.sequence,
            schema_id=packet.schema_id or "taoryx.tracking.relative-state/v1",
            invalid_reason=packet.invalid_reason or "invalid-measurement",
        )
    return TargetTrackTelemetry(
        applicable=True,
        valid=True,
        sampled_at_s=packet.sampled_at_s,
        available_at_s=packet.available_at_s,
        latency_s=packet.available_at_s - packet.sampled_at_s,
        delivery_fresh=delivery_fresh,
        sequence=-1 if packet.sequence is None else packet.sequence,
        schema_id=packet.schema_id or "taoryx.tracking.relative-state/v1",
        invalid_reason="none",
        target_id=payload.target_id,
        frame_id=payload.frame_id,
        range_m=payload.range_m,
        azimuth_rad=payload.azimuth_rad,
        elevation_rad=payload.elevation_rad,
        closing_speed_mps=payload.closing_speed_mps,
        relative_position_sensor_m=payload.relative_position_sensor_m,
        relative_velocity_sensor_mps=payload.relative_velocity_sensor_mps,
        line_of_sight_rate_sensor_rad_s=payload.line_of_sight_rate_sensor_rad_s,
    )
    ####


class InterceptorSensorSuite(BaseModel):
    """One versioned pair of standard sensor-provider configurations.

    A suite keeps Composition selection compact while retaining the ordinary
    Taoryx sensor registry as the implementation authority. The point-mass and
    pseudo-6DOF providers are validated against their required truth mode and
    payload schema before a vehicle can advertise the suite.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    version: str = Field(min_length=1)
    point_mass_provider: SensorProviderConfig = Field(default_factory=lambda: SensorProviderConfig(kind="translation-acceleration"))
    pseudo6_provider: SensorProviderConfig = Field(default_factory=lambda: SensorProviderConfig(kind="ideal"))
    target_track_provider: SensorProviderConfig = Field(
        default_factory=lambda: SensorProviderConfig(
            kind="relative-state-track",
            config={"target_id": TARGET_TRACK_ENTITY_ID},
        )
    )
    provenance: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_provider_compatibility(self) -> InterceptorSensorSuite:
        registry = sensor_plugin_registry()
        point = registry.descriptor(self.point_mass_provider.kind).manifest
        pseudo6 = registry.descriptor(self.pseudo6_provider.kind).manifest
        target_track = registry.descriptor(self.target_track_provider.kind).manifest
        _require_sensor_contract(
            suite_id=self.id,
            tier="point_mass_3dof",
            provider_kind=point.kind,
            supported_truth_modes=point.supported_truth_modes,
            required_truth_mode="translation-only",
            supported_sample_modes=point.sample_modes,
            output_schema_ids=tuple(item.schema_id for item in point.outputs),
            required_schema_id="taoryx.acceleration.increment/v1",
        )
        _require_sensor_contract(
            suite_id=self.id,
            tier="attitude_response_pseudo_6dof",
            provider_kind=pseudo6.kind,
            supported_truth_modes=pseudo6.supported_truth_modes,
            required_truth_mode="pseudo-6dof",
            supported_sample_modes=pseudo6.sample_modes,
            output_schema_ids=tuple(item.schema_id for item in pseudo6.outputs),
            required_schema_id="taoryx.imu.increment/v1",
        )
        _require_sensor_contract(
            suite_id=self.id,
            tier="target_track_guidance",
            provider_kind=target_track.kind,
            supported_truth_modes=target_track.supported_truth_modes,
            required_truth_mode="pseudo-6dof",
            supported_sample_modes=target_track.sample_modes,
            output_schema_ids=tuple(item.schema_id for item in target_track.outputs),
            required_schema_id="taoryx.tracking.relative-state/v1",
        )
        if self.target_track_provider.config.get("target_id") != TARGET_TRACK_ENTITY_ID:
            raise ValueError(f"sensor suite {self.id!r} target-track provider must select entity {TARGET_TRACK_ENTITY_ID!r}")
        return self
        ####

    @computed_field  # type: ignore[prop-decorator]
    @property
    def fingerprint(self) -> str:
        """Stable identity over suite version and both provider configurations."""

        payload = self.model_dump(mode="json", exclude={"fingerprint"})
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()
        ####

    def build_point_mass_sensor(self, *, seed: int = 0) -> TranslationAccelerationSensor:
        """Construct a fresh registered translation sensor for one execution."""

        instance = sensor_plugin_registry().create(
            self.point_mass_provider.kind,
            self.point_mass_provider.config,
            SensorBuildContext(sensor_id=f"{self.id}:point-mass", seed=seed),
        )
        return cast(
            TranslationAccelerationSensor,
            cast(
                Any,
                _CausalSensorProjection(
                    cast(_CheckpointableSensor, cast(Any, instance)),
                    expected_schema_id="taoryx.acceleration.increment/v1",
                    seed=seed,
                ),
            ),
        )
        ####

    def build_pseudo6_sensor(self, *, seed: int = 0) -> ImuSensor:
        """Construct a fresh registered IMU sensor for one execution."""

        instance = sensor_plugin_registry().create(
            self.pseudo6_provider.kind,
            self.pseudo6_provider.config,
            SensorBuildContext(sensor_id=f"{self.id}:pseudo6", seed=seed),
        )
        return cast(
            ImuSensor,
            cast(
                Any,
                _CausalSensorProjection(
                    cast(_CheckpointableSensor, cast(Any, instance)),
                    expected_schema_id="taoryx.imu.increment/v1",
                    seed=seed,
                ),
            ),
        )
        ####

    def build_target_track_sensor(self, *, seed: int = 0) -> TargetTrackSensor:
        """Construct the registered direct-geometry relative-state tracker."""

        instance = sensor_plugin_registry().create(
            self.target_track_provider.kind,
            self.target_track_provider.config,
            SensorBuildContext(sensor_id=f"{self.id}:target-track", seed=seed),
        )
        return cast(
            TargetTrackSensor,
            cast(
                Any,
                _CausalSensorProjection(
                    cast(_CheckpointableSensor, cast(Any, instance)),
                    expected_schema_id="taoryx.tracking.relative-state/v1",
                    seed=seed,
                ),
            ),
        )
        ####

    def provider_kind_for_fidelity(self, fidelity: str) -> str:
        """Return the registered provider kind selected by one runtime tier."""

        if fidelity == "point_mass_3dof":
            return self.point_mass_provider.kind
        if fidelity == "attitude_response_pseudo_6dof":
            return self.pseudo6_provider.kind
        raise ValueError(f"unsupported interceptor fidelity {fidelity!r}")
        ####

    ####


class _CheckpointableSensor(Protocol):
    provenance: Mapping[str, object]

    def reset(self) -> None: ...

    def snapshot(self) -> Mapping[str, object]: ...

    def restore(self, checkpoint: Mapping[str, object]) -> None: ...

    def sample(self, truth: TruthPoint) -> MeasurementPacket[Any]: ...

    def sample_request(self, request: SensorSampleRequest) -> MeasurementPacket[Any]: ...


class _CausalSensorProjection:
    """Release registered packets only after their declared availability time."""

    def __init__(self, sensor: _CheckpointableSensor, *, expected_schema_id: str, seed: int) -> None:
        self._sensor = sensor
        self._expected_schema_id = expected_schema_id
        self._seed = seed
        self._rng = np.random.default_rng(seed)
        self.provenance = {
            **dict(sensor.provenance),
            "delivery_projection": "causal-latest-delivered-hold/v1",
        }
        self._pending: list[MeasurementPacket[Any]] = []
        self._last_delivered: MeasurementPacket[Any] | None = None
        self._next_sequence = 0
        self.delivery_fresh = False
        ####

    def reset(self) -> None:
        self._sensor.reset()
        self._pending.clear()
        self._last_delivered = None
        self._next_sequence = 0
        self._rng = np.random.default_rng(self._seed)
        self.delivery_fresh = False
        ####

    def sample(self, truth: TruthPoint) -> MeasurementPacket[Any]:
        generated = self._sensor.sample(truth)
        return self._project(generated, truth.time_s)
        ####

    def sample_context(self, context: SensorContext) -> MeasurementPacket[Any]:
        generated = self._sensor.sample_request(SensorSampleRequest(point=context, rng=self._rng))
        return self._project(generated, context.host.time_s)
        ####

    def held_packet(self, current_time_s: float) -> MeasurementPacket[Any]:
        """Return the last causal delivery without generating another sample."""

        self.delivery_fresh = False
        if self._last_delivered is not None:
            return self._last_delivered
        return MeasurementPacket(
            sampled_at_s=current_time_s,
            available_at_s=current_time_s,
            interval_start_s=None,
            payload=None,
            valid=False,
            schema_id=self._expected_schema_id,
            invalid_reason="no-delivered-measurement",
        )
        ####

    def _project(self, generated: MeasurementPacket[Any], current_time_s: float) -> MeasurementPacket[Any]:
        schema_id = generated.schema_id or self._expected_schema_id
        if schema_id != self._expected_schema_id:
            raise ValueError(f"sensor emitted payload schema {schema_id!r}; expected {self._expected_schema_id!r}")
        generated = replace(
            generated,
            sequence=self._next_sequence,
            schema_id=schema_id,
        )
        self._next_sequence += 1
        self._pending.append(generated)
        self._pending.sort(
            key=lambda item: (
                item.available_at_s,
                item.sampled_at_s,
                -1 if item.sequence is None else item.sequence,
            )
        )
        delivered: list[MeasurementPacket[Any]] = []
        while self._pending and self._pending[0].available_at_s <= current_time_s + 1.0e-12:
            delivered.append(self._pending.pop(0))
        self.delivery_fresh = bool(delivered)
        if delivered:
            self._last_delivered = delivered[-1]
        if self._last_delivered is not None:
            return self._last_delivered
        return MeasurementPacket(
            sampled_at_s=current_time_s,
            available_at_s=current_time_s,
            interval_start_s=None,
            payload=None,
            valid=False,
            sequence=None,
            schema_id=self._expected_schema_id,
            invalid_reason="no-delivered-measurement",
        )
        ####

    def snapshot(self) -> Mapping[str, object]:
        return {
            "schema_version": 1,
            "projection_type": "taoryx.parametric-interceptors.causal-sensor-projection",
            "expected_schema_id": self._expected_schema_id,
            "sensor_checkpoint": dict(self._sensor.snapshot()),
            "pending": [packet_to_record(item, provenance=self.provenance) for item in self._pending],
            "last_delivered": (None if self._last_delivered is None else packet_to_record(self._last_delivered, provenance=self.provenance)),
            "next_sequence": self._next_sequence,
            "rng_state": deepcopy(self._rng.bit_generator.state),
        }
        ####

    def restore(self, checkpoint: Mapping[str, object]) -> None:
        if checkpoint.get("schema_version") != 1:
            raise ValueError("unsupported causal sensor projection checkpoint schema")
        if checkpoint.get("projection_type") != "taoryx.parametric-interceptors.causal-sensor-projection":
            raise ValueError("causal sensor projection checkpoint type does not match")
        if checkpoint.get("expected_schema_id") != self._expected_schema_id:
            raise ValueError("causal sensor projection payload schema does not match")
        sensor_checkpoint = checkpoint.get("sensor_checkpoint")
        if not isinstance(sensor_checkpoint, Mapping):
            raise ValueError("causal sensor projection checkpoint is missing sensor state")
        raw_pending = checkpoint.get("pending")
        if not isinstance(raw_pending, list):
            raise ValueError("causal sensor projection checkpoint pending packets must be a list")
        raw_last = checkpoint.get("last_delivered")
        if raw_last is not None and not isinstance(raw_last, Mapping):
            raise ValueError("causal sensor projection checkpoint last packet must be a mapping")
        raw_sequence = checkpoint.get("next_sequence")
        if isinstance(raw_sequence, bool) or not isinstance(raw_sequence, int) or raw_sequence < 0:
            raise ValueError("causal sensor projection checkpoint sequence must be nonnegative")
        self._sensor.restore(sensor_checkpoint)
        self._pending = [packet_from_record(parse_measurement_packet_record(item)) for item in raw_pending]
        self._last_delivered = None if raw_last is None else packet_from_record(parse_measurement_packet_record(raw_last))
        self._next_sequence = raw_sequence
        raw_rng_state = checkpoint.get("rng_state")
        if not isinstance(raw_rng_state, Mapping):
            raise ValueError("causal sensor projection checkpoint is missing RNG state")
        self._rng.bit_generator.state = deepcopy(dict(raw_rng_state))
        self.delivery_fresh = False
        ####

    ####


def standard_interceptor_sensor_suite() -> InterceptorSensorSuite:
    """Return the portable standard suite without sharing mutable sensor state."""

    return InterceptorSensorSuite(
        id=DEFAULT_SENSOR_SUITE_ID,
        version=DEFAULT_SENSOR_SUITE_VERSION,
        point_mass_provider=SensorProviderConfig(kind="translation-acceleration"),
        pseudo6_provider=SensorProviderConfig(kind="ideal"),
        target_track_provider=SensorProviderConfig(
            kind="relative-state-track",
            config={"target_id": TARGET_TRACK_ENTITY_ID},
        ),
        provenance="Taoryx built-in sensor plug-in registry API v1",
        claim_boundary=(
            "Ideal accepted-truth navigation plus direct committed-geometry relative-state tracking. The suite does not represent propagation, signatures, gimbals, a track manager, datalink, hardware qualification, or weapon performance."
        ),
    )
    ####


def _require_sensor_contract(
    *,
    suite_id: str,
    tier: str,
    provider_kind: str,
    supported_truth_modes: frozenset[str],
    required_truth_mode: str,
    supported_sample_modes: frozenset[str],
    output_schema_ids: tuple[str, ...],
    required_schema_id: str,
) -> None:
    if required_truth_mode not in supported_truth_modes:
        raise ValueError(f"sensor suite {suite_id!r} provider {provider_kind!r} does not support {tier} truth mode {required_truth_mode!r}")
    if required_schema_id not in output_schema_ids:
        raise ValueError(f"sensor suite {suite_id!r} provider {provider_kind!r} does not emit required payload schema {required_schema_id!r}")
    if "instantaneous" not in supported_sample_modes:
        raise ValueError(f"sensor suite {suite_id!r} provider {provider_kind!r} does not support accepted-boundary instantaneous sampling")
    ####


__all__ = [
    "DEFAULT_SENSOR_SUITE_ID",
    "DEFAULT_SENSOR_SUITE_VERSION",
    "ImuSensor",
    "InterceptorSensorSuite",
    "TARGET_TRACK_ENTITY_ID",
    "TargetTrackSensor",
    "TargetTrackTelemetry",
    "TranslationAccelerationSensor",
    "standard_interceptor_sensor_suite",
    "target_track_telemetry",
]
####
