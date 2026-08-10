"""Pydantic contracts for sensor providers and truth-policy variants."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Annotated, Literal, TypeAlias

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, model_validator

from taoryx.sensor_api import sensor_plugin_registry


class _ContractModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class SensorProviderConfig(_ContractModel):
    """Open provider envelope with plug-in-owned nested configuration."""

    kind: str
    config: dict[str, object] = Field(default_factory=dict)

    @model_validator(mode="before")
    @classmethod
    def validate_plugin_config(cls, value: object) -> dict[str, object]:
        if isinstance(value, str):
            payload: dict[str, object] = {"kind": value}
        elif isinstance(value, Mapping):
            payload = {str(key): item for key, item in value.items()}
        else:
            raise ValueError("sensor provider must be a name or mapping")
        if "kind" not in payload and "provider" in payload:
            payload["kind"] = payload.pop("provider")
        kind_value = payload.get("kind")
        if kind_value is None:
            declared_default = cls.model_fields["kind"].default
            kind_value = declared_default if isinstance(declared_default, str) else ""
        kind = str(kind_value).strip()
        if not kind:
            raise ValueError("sensor provider kind must not be empty")
        raw_config = payload.pop("config", {})
        if raw_config is None:
            raw_config = {}
        if not isinstance(raw_config, Mapping):
            raise ValueError("sensor provider config must be a mapping")
        flattened = {key: item for key, item in payload.items() if key != "kind"}
        config_payload = {str(key): item for key, item in raw_config.items()}
        overlap = set(config_payload) & set(flattened)
        if overlap:
            raise ValueError(f"sensor provider config repeats field(s): {sorted(overlap)}")
        config_payload.update(flattened)
        validated = sensor_plugin_registry().validate_config(kind, config_payload)
        return {
            "kind": kind,
            "config": validated.model_dump(
                mode="python",
                exclude_defaults=True,
                exclude_none=True,
            ),
        }
        ####

    def to_metadata(self) -> dict[str, object]:
        metadata: dict[str, object] = {"kind": self.kind}
        if self.config:
            metadata["config"] = dict(self.config)
        return metadata
        ####
    ####


class ImuErrorModelProviderConfig(SensorProviderConfig):
    kind: Literal["imu-error-model"] = "imu-error-model"
    ####


class IdealImuProviderConfig(SensorProviderConfig):
    kind: Literal["ideal"] = "ideal"
    ####


class IdealGyroscopeProviderConfig(SensorProviderConfig):
    kind: Literal["ideal-gyroscope"] = "ideal-gyroscope"
    ####


class TranslationAccelerationProviderConfig(SensorProviderConfig):
    kind: Literal["translation-acceleration"] = "translation-acceleration"
    ####


class VelocityAlignedAttitudeConfig(_ContractModel):
    kind: Literal["velocity-aligned"] = "velocity-aligned"
    alignment: Literal["velocity"] = "velocity"
    speed_threshold_mps: float = Field(default=1.0, gt=0.0)
    bank_source: Literal["controller", "zero"] = "controller"
    bank_default_rad: float = 0.0
    yaw_source: Literal["controller-then-heading", "heading", "hold"] = "controller-then-heading"
    zero_speed_policy: Literal["hold-then-initial-frame", "nadir-frame"] = "hold-then-initial-frame"
    pole_crossing_policy: Literal["parallel-transport"] = "parallel-transport"


class RotorcraftAttitudeConfig(_ContractModel):
    kind: Literal["rotorcraft"] = "rotorcraft"
    vehicle_type: Literal["quadcopter", "tilt-rotor"]
    forward_source: Literal["velocity", "body-axis"] = "velocity"
    alignment: Literal["velocity"] = "velocity"
    speed_threshold_mps: float = Field(default=1.0, gt=0.0)
    bank_source: Literal["controller", "zero"] = "controller"
    bank_default_rad: float = 0.0
    yaw_source: Literal["controller-then-heading", "heading", "hold"] = "controller-then-heading"
    zero_speed_policy: Literal["hold-then-initial-frame", "nadir-frame"] = "hold-then-initial-frame"
    pole_crossing_policy: Literal["parallel-transport"] = "parallel-transport"


AttitudePolicyConfig: TypeAlias = Annotated[
    VelocityAlignedAttitudeConfig | RotorcraftAttitudeConfig,
    Field(discriminator="kind"),
]


class VehicleTruthConfig(_ContractModel):
    mode: Literal["vehicle"] = "vehicle"


class TranslationOnlyTruthConfig(_ContractModel):
    mode: Literal["translation-only"] = "translation-only"


class RotationOnlyTruthConfig(_ContractModel):
    mode: Literal["rotation-only"] = "rotation-only"


class Pseudo6DofTruthConfig(_ContractModel):
    mode: Literal["pseudo-6dof"] = "pseudo-6dof"
    orientation: AttitudePolicyConfig = Field(default_factory=VelocityAlignedAttitudeConfig)


class Hybrid6DofTruthConfig(_ContractModel):
    mode: Literal["hybrid-6dof"] = "hybrid-6dof"


TruthConfig: TypeAlias = Annotated[
    VehicleTruthConfig
    | TranslationOnlyTruthConfig
    | RotationOnlyTruthConfig
    | Pseudo6DofTruthConfig
    | Hybrid6DofTruthConfig,
    Field(discriminator="mode"),
]


_TRUTH_ADAPTER: TypeAdapter[TruthConfig] = TypeAdapter(TruthConfig)


def parse_truth_config(value: Mapping[str, object] | None) -> TruthConfig:
    """Validate and discriminate one sidecar truth definition."""

    payload = dict(value or {})
    if payload.get("mode") == "pseudo-6dof":
        orientation_value = payload.get("orientation")
        if orientation_value is None:
            payload["orientation"] = {"kind": "velocity-aligned"}
        elif isinstance(orientation_value, Mapping):
            orientation = {str(key): item for key, item in orientation_value.items()}
            orientation.setdefault("kind", "velocity-aligned")
            payload["orientation"] = orientation
    return _TRUTH_ADAPTER.validate_python(payload or {"mode": "vehicle"})


def parse_provider_config(value: object) -> SensorProviderConfig:
    """Validate and discriminate one provider name or provider mapping."""

    return SensorProviderConfig.model_validate(value)
    ####
