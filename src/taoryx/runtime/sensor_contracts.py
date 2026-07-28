"""Pydantic contracts for sensor providers and truth-policy variants."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Annotated, Literal, TypeAlias

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter


class _ContractModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class ImuErrorModelProviderConfig(_ContractModel):
    kind: Literal["imu-error-model"] = "imu-error-model"


class IdealImuProviderConfig(_ContractModel):
    kind: Literal["ideal"] = "ideal"


class IdealGyroscopeProviderConfig(_ContractModel):
    kind: Literal["ideal-gyroscope"] = "ideal-gyroscope"


class TranslationAccelerationProviderConfig(_ContractModel):
    kind: Literal["translation-acceleration"] = "translation-acceleration"


SensorProviderConfig: TypeAlias = Annotated[
    ImuErrorModelProviderConfig
    | IdealImuProviderConfig
    | IdealGyroscopeProviderConfig
    | TranslationAccelerationProviderConfig,
    Field(discriminator="kind"),
]


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
_PROVIDER_ADAPTER: TypeAdapter[SensorProviderConfig] = TypeAdapter(SensorProviderConfig)


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

    if isinstance(value, str):
        value = {"kind": value}
    elif isinstance(value, Mapping):
        payload: dict[str, object] = {str(key): item for key, item in value.items()}
        if "kind" not in payload and "provider" in payload:
            payload["kind"] = payload.pop("provider")
        value = payload
    return _PROVIDER_ADAPTER.validate_python(value)
