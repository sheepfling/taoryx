"""Bundled inertial sensor plug-ins and their versioned payload codecs."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any, cast

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from taoryx.sensor_api import (
    PayloadCodec,
    SensorBuildContext,
    SensorOutputPort,
    SensorPluginDescriptor,
    SensorPluginManifest,
)
from taoryx.sensors import (
    AccelerationIncrement,
    GyroIncrement,
    IdealGyroscopeAdapter,
    IdealImuAdapter,
    ImuErrorModelAdapter,
    ImuIncrement,
    TranslationAccelerationAdapter,
)


class _ImuPluginConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    ####


class ImuErrorModelPluginConfig(_ImuPluginConfig):
    profile_path: str | None = None
    profile_name: str | None = None
    profile_category: str = "hardware_estimates"
    delivery_delay_s: float = Field(default=0.0, ge=0.0)
    ####


class IdealImuPluginConfig(_ImuPluginConfig):
    pass
    ####


class IdealGyroscopePluginConfig(_ImuPluginConfig):
    delivery_delay_s: float = Field(default=0.0, ge=0.0)
    ####


class TranslationAccelerationPluginConfig(_ImuPluginConfig):
    delivery_delay_s: float = Field(default=0.0, ge=0.0)
    ####


def _resource_text(context: SensorBuildContext, name: str) -> str | None:
    value = context.resources.get(name)
    return None if value is None else str(value)
    ####


def _build_imu_error_model(config: BaseModel, context: SensorBuildContext) -> ImuErrorModelAdapter:
    selected = cast(ImuErrorModelPluginConfig, config)
    profile_path = selected.profile_path or _resource_text(context, "profile_path")
    profile_name = selected.profile_name or _resource_text(context, "profile_name")
    profile_category = selected.profile_category
    legacy_category = _resource_text(context, "profile_category")
    if selected.profile_category == "hardware_estimates" and legacy_category is not None:
        profile_category = legacy_category
    if profile_path is not None:
        selected_path = Path(profile_path)
        source_path = _resource_text(context, "scenario_source_path")
        if not selected_path.is_absolute() and source_path is not None:
            selected_path = Path(source_path).parent / selected_path
        return ImuErrorModelAdapter.from_profile(
            selected_path,
            seed=context.seed,
            body_from_sensor=context.body_from_sensor,
            lever_arm_body_m=context.lever_arm_body_m,
            delivery_delay_s=selected.delivery_delay_s,
        )
    if profile_name is not None:
        return ImuErrorModelAdapter.from_example_profile(
            profile_name,
            category=profile_category,
            seed=context.seed,
            body_from_sensor=context.body_from_sensor,
            lever_arm_body_m=context.lever_arm_body_m,
            delivery_delay_s=selected.delivery_delay_s,
        )
    return ImuErrorModelAdapter.from_config(
        seed=context.seed,
        body_from_sensor=context.body_from_sensor,
        lever_arm_body_m=context.lever_arm_body_m,
        delivery_delay_s=selected.delivery_delay_s,
    )
    ####


def _build_ideal_imu(config: BaseModel, context: SensorBuildContext) -> IdealImuAdapter:
    del config
    return IdealImuAdapter(
        body_from_sensor=context.body_from_sensor,
        lever_arm_body_m=context.lever_arm_body_m,
    )
    ####


def _build_ideal_gyroscope(config: BaseModel, context: SensorBuildContext) -> IdealGyroscopeAdapter:
    del context
    selected = cast(IdealGyroscopePluginConfig, config)
    return IdealGyroscopeAdapter(delivery_delay_s=selected.delivery_delay_s)
    ####


def _build_translation_acceleration(
    config: BaseModel,
    context: SensorBuildContext,
) -> TranslationAccelerationAdapter:
    del context
    selected = cast(TranslationAccelerationPluginConfig, config)
    return TranslationAccelerationAdapter(delivery_delay_s=selected.delivery_delay_s)
    ####


IMU_SENSOR_PLUGINS = (
    SensorPluginDescriptor(
        SensorPluginManifest(
            kind="imu-error-model",
            family="inertial",
            language_kinds=frozenset({"imu"}),
            clock_kind="imu",
            outputs=(SensorOutputPort("increments", "taoryx.imu.increment/v1"),),
            required_truth=frozenset({"position", "velocity", "orientation", "body-rate", "gravity"}),
            supported_truth_modes=frozenset({"vehicle", "pseudo-6dof", "hybrid-6dof"}),
            sample_modes=frozenset({"instantaneous", "interval"}),
            default_cadence_s=0.01,
            description="External imu-error-model adapter over committed vehicle truth.",
        ),
        ImuErrorModelPluginConfig,
        _build_imu_error_model,
    ),
    SensorPluginDescriptor(
        SensorPluginManifest(
            kind="ideal",
            family="inertial",
            language_kinds=frozenset({"imu"}),
            clock_kind="imu",
            outputs=(SensorOutputPort("increments", "taoryx.imu.increment/v1"),),
            required_truth=frozenset({"position", "velocity", "orientation", "body-rate", "gravity"}),
            supported_truth_modes=frozenset({"vehicle", "pseudo-6dof", "hybrid-6dof"}),
            sample_modes=frozenset({"instantaneous", "interval"}),
            default_cadence_s=0.01,
            description="Ideal committed-truth IMU comparison model.",
        ),
        IdealImuPluginConfig,
        _build_ideal_imu,
    ),
    SensorPluginDescriptor(
        SensorPluginManifest(
            kind="ideal-gyroscope",
            family="inertial",
            language_kinds=frozenset({"gyroscope"}),
            clock_kind="gyroscope",
            outputs=(SensorOutputPort("increments", "taoryx.gyro.increment/v1"),),
            required_truth=frozenset({"orientation", "body-rate"}),
            supported_truth_modes=frozenset({"rotation-only"}),
            sample_modes=frozenset({"instantaneous", "interval"}),
            default_cadence_s=0.01,
            description="Ideal body-rate increment model.",
        ),
        IdealGyroscopePluginConfig,
        _build_ideal_gyroscope,
    ),
    SensorPluginDescriptor(
        SensorPluginManifest(
            kind="translation-acceleration",
            family="inertial",
            language_kinds=frozenset({"accelerometer"}),
            clock_kind="accelerometer",
            outputs=(SensorOutputPort("increments", "taoryx.acceleration.increment/v1"),),
            required_truth=frozenset({"position", "velocity", "acceleration", "gravity"}),
            supported_truth_modes=frozenset({"translation-only"}),
            sample_modes=frozenset({"instantaneous", "interval"}),
            default_cadence_s=0.01,
            description="Translation-only specific-force increment model.",
        ),
        TranslationAccelerationPluginConfig,
        _build_translation_acceleration,
    ),
)


def _encode_imu(value: object) -> Mapping[str, object]:
    payload = cast(ImuIncrement, value)
    return {
        "kind": "imu_increment",
        "delta_v_body_mps": payload.delta_v_body_mps.tolist(),
        "delta_theta_body_rad": payload.delta_theta_body_rad.tolist(),
        "start_time_s": payload.start_time_s,
        "end_time_s": payload.end_time_s,
        "temperature_celsius": payload.temperature_celsius,
    }
    ####


def _decode_imu(value: Mapping[str, object]) -> ImuIncrement:
    return ImuIncrement(
        np.asarray(value["delta_v_body_mps"], dtype=float),
        np.asarray(value["delta_theta_body_rad"], dtype=float),
        float(cast(Any, value["start_time_s"])),
        float(cast(Any, value["end_time_s"])),
        None if value.get("temperature_celsius") is None else float(cast(Any, value["temperature_celsius"])),
    )
    ####


def _encode_gyro(value: object) -> Mapping[str, object]:
    payload = cast(GyroIncrement, value)
    return {
        "kind": "gyro_increment",
        "delta_theta_body_rad": payload.delta_theta_body_rad.tolist(),
        "start_time_s": payload.start_time_s,
        "end_time_s": payload.end_time_s,
    }
    ####


def _decode_gyro(value: Mapping[str, object]) -> GyroIncrement:
    return GyroIncrement(
        np.asarray(value["delta_theta_body_rad"], dtype=float),
        float(cast(Any, value["start_time_s"])),
        float(cast(Any, value["end_time_s"])),
    )
    ####


def _encode_acceleration(value: object) -> Mapping[str, object]:
    payload = cast(AccelerationIncrement, value)
    return {
        "kind": "acceleration_increment",
        "delta_v_eci_mps": payload.delta_v_eci_mps.tolist(),
        "start_time_s": payload.start_time_s,
        "end_time_s": payload.end_time_s,
    }
    ####


def _decode_acceleration(value: Mapping[str, object]) -> AccelerationIncrement:
    return AccelerationIncrement(
        np.asarray(value["delta_v_eci_mps"], dtype=float),
        float(cast(Any, value["start_time_s"])),
        float(cast(Any, value["end_time_s"])),
    )
    ####


IMU_PAYLOAD_CODECS = (
    PayloadCodec(
        "taoryx.imu.increment/v1",
        ImuIncrement,
        _encode_imu,
        _decode_imu,
        {
            "frame": "body",
            "delta_v_unit": "m/s",
            "delta_theta_unit": "rad",
        },
        legacy_kind="imu_increment",
    ),
    PayloadCodec(
        "taoryx.gyro.increment/v1",
        GyroIncrement,
        _encode_gyro,
        _decode_gyro,
        {
            "frame": "body",
            "delta_theta_unit": "rad",
            "measurement": "angular-rate",
        },
        legacy_kind="gyro_increment",
    ),
    PayloadCodec(
        "taoryx.acceleration.increment/v1",
        AccelerationIncrement,
        _encode_acceleration,
        _decode_acceleration,
        {
            "frame": "ECI",
            "delta_v_unit": "m/s",
            "measurement": "specific-force",
        },
        legacy_kind="acceleration_increment",
    ),
)


__all__ = [
    "IMU_PAYLOAD_CODECS",
    "IMU_SENSOR_PLUGINS",
    "IdealGyroscopePluginConfig",
    "IdealImuPluginConfig",
    "ImuErrorModelPluginConfig",
    "TranslationAccelerationPluginConfig",
]
####
