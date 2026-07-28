"""Composable measurement corruption models applied after sensor sampling.

Plant truth is deliberately not mutable here. Each stage receives a typed
measurement packet and returns a new packet with explicit provenance.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from copy import deepcopy
from typing import Annotated, Any, Literal, Protocol, TypeAlias, cast

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, field_validator

from taoryx.sensors import ImuIncrement, MeasurementPacket, TruthPoint, TruthSegment


class _StageConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


def _matrix3(value: object, name: str) -> tuple[tuple[float, float, float], ...]:
    array = np.asarray(value, dtype=float)
    if array.shape != (3, 3) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    return cast(tuple[tuple[float, float, float], ...], tuple(tuple(float(item) for item in row) for row in array))


class ScaleMisalignmentConfig(_StageConfig):
    kind: Literal["scale-misalignment"] = "scale-misalignment"
    accelerometer_matrix: tuple[tuple[float, float, float], ...] = Field(default_factory=lambda: _matrix3(np.eye(3), "accelerometer_matrix"))
    gyroscope_matrix: tuple[tuple[float, float, float], ...] = Field(default_factory=lambda: _matrix3(np.eye(3), "gyroscope_matrix"))

    @field_validator("accelerometer_matrix", "gyroscope_matrix", mode="before")
    @classmethod
    def validate_matrices(cls, value: object) -> tuple[tuple[float, float, float], ...]:
        return _matrix3(value, "misalignment matrix")


class BiasConfig(_StageConfig):
    kind: Literal["bias"] = "bias"
    accelerometer_bias_mps2: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyroscope_bias_radps: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @field_validator("accelerometer_bias_mps2", "gyroscope_bias_radps", mode="before")
    @classmethod
    def validate_vectors(cls, value: object) -> tuple[float, float, float]:
        array = np.asarray(value, dtype=float)
        if array.shape != (3,) or not np.all(np.isfinite(array)):
            raise ValueError("bias values must be finite 3-vectors")
        return cast(tuple[float, float, float], tuple(float(item) for item in array))


class GaussianNoiseConfig(_StageConfig):
    kind: Literal["gaussian-noise"] = "gaussian-noise"
    accelerometer_density_mps2_sqrt_hz: float = Field(default=0.0, ge=0.0)
    gyroscope_density_radps_sqrt_hz: float = Field(default=0.0, ge=0.0)
    seed: int = 0


class QuantizationConfig(_StageConfig):
    kind: Literal["quantization"] = "quantization"
    accelerometer_step_mps: float = Field(default=0.0, ge=0.0)
    gyroscope_step_rad: float = Field(default=0.0, ge=0.0)


class DropoutConfig(_StageConfig):
    kind: Literal["dropout"] = "dropout"
    every_n: int = Field(gt=0)
    phase: int = Field(default=0, ge=0)


ObservationStageConfig: TypeAlias = Annotated[
    ScaleMisalignmentConfig | BiasConfig | GaussianNoiseConfig | QuantizationConfig | DropoutConfig,
    Field(discriminator="kind"),
]


class ObservationModelConfig(BaseModel):
    """An ordered, serializable measurement corruption pipeline."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    kind: Literal["compose"] = "compose"
    stages: tuple[ObservationStageConfig, ...] = ()


_OBSERVATION_ADAPTER = TypeAdapter(ObservationModelConfig)


def parse_observation_config(value: Mapping[str, object] | None) -> ObservationModelConfig | None:
    """Parse an optional sidecar observation pipeline."""

    if value is None:
        return None
    payload = dict(value)
    payload.setdefault("kind", "compose")
    return _OBSERVATION_ADAPTER.validate_python(payload)


class MeasurementTransform(Protocol):
    def apply(self, packet: MeasurementPacket[Any]) -> MeasurementPacket[Any]:
        ...

    def reset(self) -> None:
        ...

    def metadata(self) -> dict[str, object]:
        ...


def _replace_imu(packet: MeasurementPacket[Any], transform: Callable[[ImuIncrement], ImuIncrement]) -> MeasurementPacket[Any]:
    if not packet.valid or packet.payload is None:
        return packet
    if not isinstance(packet.payload, ImuIncrement):
        return packet
    return MeasurementPacket(
        packet.sampled_at_s,
        packet.available_at_s,
        packet.interval_start_s,
        transform(packet.payload),
        packet.valid,
    )


class _ScaleMisalignmentTransform:
    def __init__(self, config: ScaleMisalignmentConfig) -> None:
        self.config = config
        self.accelerometer_matrix = np.asarray(config.accelerometer_matrix, dtype=float)
        self.gyroscope_matrix = np.asarray(config.gyroscope_matrix, dtype=float)

    def apply(self, packet: MeasurementPacket[Any]) -> MeasurementPacket[Any]:
        return _replace_imu(
            packet,
            lambda increment: ImuIncrement(
                self.accelerometer_matrix @ increment.delta_v_body_mps,
                self.gyroscope_matrix @ increment.delta_theta_body_rad,
                increment.start_time_s,
                increment.end_time_s,
                increment.temperature_celsius,
            ),
        )

    def reset(self) -> None:
        return None

    def metadata(self) -> dict[str, object]:
        return self.config.model_dump(mode="json")


class _BiasTransform:
    def __init__(self, config: BiasConfig) -> None:
        self.config = config
        self.accelerometer_bias = np.asarray(config.accelerometer_bias_mps2, dtype=float)
        self.gyroscope_bias = np.asarray(config.gyroscope_bias_radps, dtype=float)

    def apply(self, packet: MeasurementPacket[Any]) -> MeasurementPacket[Any]:
        return _replace_imu(
            packet,
            lambda increment: ImuIncrement(
                increment.delta_v_body_mps + self.accelerometer_bias * increment.dt_s,
                increment.delta_theta_body_rad + self.gyroscope_bias * increment.dt_s,
                increment.start_time_s,
                increment.end_time_s,
                increment.temperature_celsius,
            ),
        )

    def reset(self) -> None:
        return None

    def metadata(self) -> dict[str, object]:
        return self.config.model_dump(mode="json")


class _GaussianNoiseTransform:
    def __init__(self, config: GaussianNoiseConfig) -> None:
        self.config = config
        self.rng = np.random.default_rng(config.seed)

    def apply(self, packet: MeasurementPacket[Any]) -> MeasurementPacket[Any]:
        def perturb(increment: ImuIncrement) -> ImuIncrement:
            scale = np.sqrt(increment.dt_s)
            return ImuIncrement(
                increment.delta_v_body_mps
                + self.rng.normal(0.0, self.config.accelerometer_density_mps2_sqrt_hz * scale, 3),
                increment.delta_theta_body_rad
                + self.rng.normal(0.0, self.config.gyroscope_density_radps_sqrt_hz * scale, 3),
                increment.start_time_s,
                increment.end_time_s,
                increment.temperature_celsius,
            )

        return _replace_imu(packet, perturb)

    def reset(self) -> None:
        self.rng = np.random.default_rng(self.config.seed)

    def metadata(self) -> dict[str, object]:
        return self.config.model_dump(mode="json")


class _QuantizationTransform:
    def __init__(self, config: QuantizationConfig) -> None:
        self.config = config

    @staticmethod
    def _quantize(value: np.ndarray, step: float) -> np.ndarray:
        return value if step == 0.0 else np.round(value / step) * step

    def apply(self, packet: MeasurementPacket[Any]) -> MeasurementPacket[Any]:
        return _replace_imu(
            packet,
            lambda increment: ImuIncrement(
                self._quantize(increment.delta_v_body_mps, self.config.accelerometer_step_mps),
                self._quantize(increment.delta_theta_body_rad, self.config.gyroscope_step_rad),
                increment.start_time_s,
                increment.end_time_s,
                increment.temperature_celsius,
            ),
        )

    def reset(self) -> None:
        return None

    def metadata(self) -> dict[str, object]:
        return self.config.model_dump(mode="json")


class _DropoutTransform:
    def __init__(self, config: DropoutConfig) -> None:
        self.config = config
        self.count = 0

    def apply(self, packet: MeasurementPacket[Any]) -> MeasurementPacket[Any]:
        self.count += 1
        if (self.count - 1 - self.config.phase) % self.config.every_n != 0:
            return packet
        return MeasurementPacket(packet.sampled_at_s, packet.available_at_s, packet.interval_start_s, None, valid=False)

    def reset(self) -> None:
        self.count = 0

    def metadata(self) -> dict[str, object]:
        return self.config.model_dump(mode="json")


def _build_transform(config: ObservationStageConfig) -> MeasurementTransform:
    if isinstance(config, ScaleMisalignmentConfig):
        return _ScaleMisalignmentTransform(config)
    if isinstance(config, BiasConfig):
        return _BiasTransform(config)
    if isinstance(config, GaussianNoiseConfig):
        return _GaussianNoiseTransform(config)
    if isinstance(config, QuantizationConfig):
        return _QuantizationTransform(config)
    if isinstance(config, DropoutConfig):
        return _DropoutTransform(config)
    raise TypeError(f"unsupported observation stage {type(config).__name__}")


class ObservationPipeline:
    """Wrap a base sensor model with ordered measurement transforms."""

    def __init__(self, base_model: object, config: ObservationModelConfig) -> None:
        self.base_model = base_model
        self.config = config
        self.transforms = [_build_transform(stage) for stage in config.stages]
        base_provenance = getattr(base_model, "provenance", {})
        normalized_provenance = dict(base_provenance) if isinstance(base_provenance, Mapping) else {}
        self.provenance = {
            **normalized_provenance,
            "observation_model": config.model_dump(mode="json"),
            "observation_stages": [transform.metadata() for transform in self.transforms],
        }

    def sample(self, truth: TruthPoint) -> MeasurementPacket[Any]:
        packet = cast(Any, self.base_model).sample(truth)
        return self._apply(packet)

    def sample_segment(self, segment: TruthSegment) -> MeasurementPacket[Any]:
        sampler = getattr(self.base_model, "sample_segment", None)
        packet = sampler(segment) if callable(sampler) else cast(Any, self.base_model).sample(segment.end)
        return self._apply(packet)

    def _apply(self, packet: MeasurementPacket[Any]) -> MeasurementPacket[Any]:
        for transform in self.transforms:
            packet = transform.apply(packet)
        return packet

    def reset(self) -> None:
        reset = getattr(self.base_model, "reset", None)
        if callable(reset):
            reset()
        for transform in self.transforms:
            transform.reset()

    def snapshot(self) -> dict[str, object]:
        """Return a JSON-compatible checkpoint for model and transform state."""

        model_snapshot = getattr(self.base_model, "snapshot", None)
        payload: dict[str, object] = {
            "schema_version": 1,
            "observation_model": self.config.model_dump(mode="json"),
            "base_model": None if not callable(model_snapshot) else model_snapshot().model_dump(mode="json"),
            "transforms": [self._transform_snapshot(transform) for transform in self.transforms],
        }
        return payload

    def restore(self, checkpoint: Mapping[str, object]) -> None:
        if int(cast(Any, checkpoint.get("schema_version", 0))) != 1:
            raise ValueError("unsupported observation checkpoint schema")
        model_checkpoint = checkpoint.get("base_model")
        restore = getattr(self.base_model, "restore", None)
        if model_checkpoint is not None and callable(restore):
            current = getattr(self.base_model, "snapshot", lambda: None)()
            if current is None:
                raise ValueError("base model does not expose a checkpoint type")
            restore(type(current).model_validate(model_checkpoint))
        transform_states = checkpoint.get("transforms", ())
        if not isinstance(transform_states, Sequence) or isinstance(transform_states, (str, bytes)):
            raise ValueError("observation transform checkpoint is missing transforms")
        for transform, state in zip(self.transforms, transform_states, strict=True):
            self._restore_transform(transform, state)

    @staticmethod
    def _transform_snapshot(transform: MeasurementTransform) -> dict[str, object]:
        if isinstance(transform, _DropoutTransform):
            return {"kind": "dropout", "count": transform.count}
        if isinstance(transform, _GaussianNoiseTransform):
            return {"kind": "gaussian-noise", "rng_state": deepcopy(transform.rng.bit_generator.state)}
        return {"kind": str(transform.metadata().get("kind", "unknown"))}

    @staticmethod
    def _restore_transform(transform: MeasurementTransform, state: object) -> None:
        if not isinstance(state, Mapping):
            raise ValueError("observation transform checkpoint must be a mapping")
        if isinstance(transform, _DropoutTransform):
            transform.count = int(state.get("count", 0))
        elif isinstance(transform, _GaussianNoiseTransform):
            rng_state = state.get("rng_state")
            if not isinstance(rng_state, Mapping):
                raise ValueError("gaussian-noise checkpoint is missing rng_state")
            transform.rng.bit_generator.state = cast(Any, dict(rng_state))


def build_observation_pipeline(base_model: Any, config: ObservationModelConfig | None) -> Any:
    """Return the base model or an observation pipeline when stages are declared."""

    if config is None or not config.stages:
        return base_model
    return ObservationPipeline(base_model, config)
