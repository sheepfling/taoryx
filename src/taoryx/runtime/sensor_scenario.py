"""Provider-neutral scenario sensor bindings and navigation artifacts."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, ClassVar, Protocol, cast

import numpy as np
import yaml

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.modes import Quaternion
from taoryx.navigation import (
    AttitudeNavigationState,
    AttitudeOnlyNavigator,
    DeadReckoningNavigator,
    MultiplicativeEkf,
    NavigationState,
    TranslationNavigationState,
    TranslationOnlyNavigator,
)
from taoryx.rigid_body import RIGID_BODY_STATE_NAMES, RigidBody6DofState
from taoryx.sensors import (
    AccelerationIncrement,
    GyroIncrement,
    IdealGyroscopeAdapter,
    IdealImuAdapter,
    ImuErrorModelAdapter,
    MeasurementPacket,
    TranslationAccelerationAdapter,
)

from .common import RuntimeProblem
from .navigation_feedback import NavigationFeedbackConfig, parse_navigation_feedback
from .observation_models import ObservationModelConfig, build_observation_pipeline, parse_observation_config
from .sensor_bus import SensorBinding, SensorBus
from .sensor_clock import SensorClockSpec
from .sensor_contracts import (
    Pseudo6DofTruthConfig,
    SensorProviderConfig,
    TruthConfig,
    parse_provider_config,
    parse_truth_config,
)
from .truth import AttitudePolicyFactory, CompositeTruthProvider, Pseudo6DofTruthProvider, RotationTruthProvider, truth_provider_contract


class _Navigator(Protocol):
    state: Any

    def propagate(self, packet: MeasurementPacket[Any]) -> Any:
        ...


def _as_vector(value: object, name: str) -> tuple[float, ...] | None:
    if value is None:
        return None
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must be a numeric sequence")
    values = tuple(float(item) for item in value)
    if len(values) != 3 or not all(math.isfinite(item) for item in values):
        raise ValueError(f"{name} must be a finite 3-vector")
    return values


def _as_rotation(value: object, name: str) -> np.ndarray | None:
    if value is None:
        return None
    array = np.asarray(value, dtype=float)
    if array.shape == (9,):
        array = array.reshape((3, 3))
    if array.shape != (3, 3) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite 3x3 rotation")
    if not np.allclose(array.T @ array, np.eye(3), atol=1.0e-8) or not np.isclose(np.linalg.det(array), 1.0, atol=1.0e-8):
        raise ValueError(f"{name} must be a proper rotation")
    return array


@dataclass(frozen=True, slots=True)
class SensorScenarioSpec:
    """Serializable binding configuration kept outside the `.prb` grammar."""

    sensor_name: str
    vehicle_name: str
    scenario_id: str = "sensor-scenario-v1"
    provider: str = "imu-error-model"
    truth_mode: str = "vehicle"
    alignment: str = "velocity"
    speed_threshold_mps: float = 1.0
    bank_source: str = "controller"
    bank_default_rad: float = 0.0
    yaw_source: str = "controller-then-heading"
    zero_speed_policy: str = "hold-then-initial-frame"
    pole_crossing_policy: str = "parallel-transport"
    cadence_s: float | None = None
    phase_s: float = 0.0
    sample_mode: str = "instantaneous"
    delivery_s: float = 0.0
    truth_policy: str = "boundary"
    rate_policy: str = "split"
    earth_rate_mode: str = "source"
    earth_omega_rad_s: float | None = None
    drop_every_n: int | None = None
    profile_path: Path | None = None
    profile_name: str | None = None
    profile_category: str = "hardware_estimates"
    seed: int | None = None
    body_from_sensor: np.ndarray | None = None
    lever_arm_body_m: tuple[float, ...] | None = None
    estimator_modes: tuple[str, ...] = ("dead-reckoning", "mekf")
    source_path: Path | None = None
    truth_config: TruthConfig | None = None
    provider_config: SensorProviderConfig | None = None
    observation_model: ObservationModelConfig | None = None
    feedback: NavigationFeedbackConfig = field(default_factory=NavigationFeedbackConfig)

    def __post_init__(self) -> None:
        if isinstance(self.observation_model, Mapping):
            object.__setattr__(self, "observation_model", parse_observation_config(self.observation_model))
        if isinstance(self.feedback, Mapping):
            object.__setattr__(self, "feedback", parse_navigation_feedback(self.feedback))
        if not self.sensor_name.strip() or not self.vehicle_name.strip():
            raise ValueError("sensor and vehicle names must not be empty")
        if not self.scenario_id.strip():
            raise ValueError("sensor scenario id must not be empty")
        if self.provider not in {"imu-error-model", "ideal", "ideal-gyroscope", "translation-acceleration"}:
            raise ValueError("sensor provider must be imu-error-model, ideal, ideal-gyroscope, or translation-acceleration")
        if self.truth_mode not in {"vehicle", "translation-only", "rotation-only", "pseudo-6dof", "hybrid-6dof"}:
            raise ValueError("truth_mode must be vehicle, translation-only, rotation-only, pseudo-6dof, or hybrid-6dof")
        if self.truth_mode == "translation-only" and self.provider != "translation-acceleration":
            raise ValueError("translation-only truth requires the translation-acceleration provider")
        if self.truth_mode == "rotation-only" and self.provider != "ideal-gyroscope":
            raise ValueError("rotation-only truth requires the ideal-gyroscope provider")
        if self.provider == "ideal-gyroscope" and self.truth_mode != "rotation-only":
            raise ValueError("ideal-gyroscope provider requires rotation-only truth")
        if self.truth_mode == "hybrid-6dof" and self.provider == "translation-acceleration":
            raise ValueError("hybrid-6dof truth requires a full IMU provider")
        if not math.isfinite(self.speed_threshold_mps) or self.speed_threshold_mps <= 0.0:
            raise ValueError("speed_threshold_mps must be positive and finite")
        if self.alignment != "velocity":
            raise ValueError("alignment must be velocity")
        if self.bank_source not in {"controller", "zero"}:
            raise ValueError("bank_source must be controller or zero")
        if not math.isfinite(self.bank_default_rad):
            raise ValueError("bank_default_rad must be finite")
        if self.yaw_source not in {"controller-then-heading", "heading", "hold"}:
            raise ValueError("yaw_source is invalid")
        if self.zero_speed_policy not in {"hold-then-initial-frame", "nadir-frame"}:
            raise ValueError("zero_speed_policy is invalid")
        if self.pole_crossing_policy != "parallel-transport":
            raise ValueError("pole_crossing_policy must be parallel-transport")
        if self.cadence_s is not None and (not math.isfinite(self.cadence_s) or self.cadence_s <= 0.0):
            raise ValueError("sensor cadence must be positive and finite")
        if not math.isfinite(self.phase_s) or self.phase_s < 0.0:
            raise ValueError("sensor phase must be finite and nonnegative")
        if not math.isfinite(self.delivery_s) or self.delivery_s < 0.0:
            raise ValueError("sensor delivery must be finite and nonnegative")
        if self.sample_mode not in {"instantaneous", "interval"}:
            raise ValueError("sensor sample_mode must be instantaneous or interval")
        if self.truth_policy not in {"boundary", "accepted-segment"}:
            raise ValueError("sensor truth_policy is invalid")
        if self.rate_policy not in {"split", "accumulate"}:
            raise ValueError("sensor rate_policy is invalid")
        if self.earth_rate_mode not in {"source", "nominal", "explicit"}:
            raise ValueError("earth_rate_mode must be source, nominal, or explicit")
        if self.earth_rate_mode == "explicit" and self.earth_omega_rad_s is None:
            raise ValueError("explicit earth_rate_mode requires earth_omega_rad_s")
        if self.earth_omega_rad_s is not None and not math.isfinite(self.earth_omega_rad_s):
            raise ValueError("earth_omega_rad_s must be finite when supplied")
        if self.drop_every_n is not None and self.drop_every_n <= 0:
            raise ValueError("drop_every_n must be positive when supplied")
        if not self.estimator_modes:
            raise ValueError("at least one estimator mode is required")
        if self.lever_arm_body_m is not None and len(self.lever_arm_body_m) != 3:
            raise ValueError("lever arm must be a 3-vector")
        if self.truth_config is not None and self.truth_config.mode != self.truth_mode:
            raise ValueError("truth_config mode must match truth_mode")
        if self.provider_config is not None and self.provider_config.kind != self.provider:
            raise ValueError("provider_config kind must match provider")

    def resolved_truth_config(self) -> TruthConfig:
        if self.truth_config is not None:
            return self.truth_config
        payload: dict[str, object] = {"mode": self.truth_mode}
        if self.truth_mode == "pseudo-6dof":
            payload["orientation"] = {
                "kind": "velocity-aligned",
                "alignment": self.alignment,
                "speed_threshold_mps": self.speed_threshold_mps,
                "bank_source": self.bank_source,
                "bank_default_rad": self.bank_default_rad,
                "yaw_source": self.yaw_source,
                "zero_speed_policy": self.zero_speed_policy,
                "pole_crossing_policy": self.pole_crossing_policy,
            }
        return parse_truth_config(payload)

    def resolved_provider_config(self) -> SensorProviderConfig:
        return self.provider_config or parse_provider_config(self.provider)

    @classmethod
    def from_file(cls, path: str | Path) -> SensorScenarioSpec:
        source = Path(path).resolve()
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("sensor scenario specification must be a mapping")
        schema_version = int(payload.get("schema_version", 1))
        if schema_version != 1:
            raise ValueError(f"unsupported sensor scenario schema version {schema_version}")
        sensor = payload.get("sensor", payload)
        if not isinstance(sensor, Mapping):
            raise ValueError("sensor scenario specification requires a sensor mapping")
        profile = sensor.get("profile")
        profile_path: Path | None = None
        profile_name: str | None = None
        profile_category = str(sensor.get("profile_category", "hardware_estimates"))
        if profile is not None:
            profile_text = str(profile)
            if profile_text.startswith("package:"):
                package_profile = profile_text.removeprefix("package:")
                if "/" in package_profile:
                    profile_category, profile_name = package_profile.split("/", 1)
                else:
                    profile_name = package_profile
            else:
                candidate = (source.parent / profile_text).resolve()
                if candidate.is_file():
                    profile_path = candidate
                else:
                    profile_name = profile_text
        drop_policy = payload.get("drop_policy", sensor.get("drop_policy", {}))
        if drop_policy is None:
            drop_policy = {}
        if not isinstance(drop_policy, Mapping):
            raise ValueError("drop_policy must be a mapping")
        earth = payload.get("earth", sensor.get("earth", {}))
        if earth is None:
            earth = {}
        if not isinstance(earth, Mapping):
            raise ValueError("earth must be a mapping")
        truth = payload.get("truth", {})
        if truth is None:
            truth = {}
        if not isinstance(truth, Mapping):
            raise ValueError("truth must be a mapping")
        orientation = truth.get("orientation", {})
        if orientation is None:
            orientation = {}
        if not isinstance(orientation, Mapping):
            raise ValueError("truth.orientation must be a mapping")
        mounting = _as_rotation(sensor.get("body_from_sensor"), "body_from_sensor")
        lever_arm = _as_vector(sensor.get("lever_arm_body_m"), "lever_arm_body_m")
        truth_config = parse_truth_config(truth)
        observation = payload.get("observation", sensor.get("observation"))
        if observation is not None and not isinstance(observation, Mapping):
            raise ValueError("observation must be a mapping")
        navigation = payload.get("navigation", sensor.get("navigation", {}))
        if navigation is None:
            navigation = {}
        if not isinstance(navigation, Mapping):
            raise ValueError("navigation must be a mapping")
        feedback = payload.get("feedback", navigation.get("feedback", {}))
        if feedback is None:
            feedback = {}
        if not isinstance(feedback, Mapping):
            raise ValueError("feedback must be a mapping")
        provider_config = parse_provider_config(sensor.get("provider", payload.get("provider", "imu-error-model")))
        truth_mode = truth_config.mode
        orientation_config = truth_config.orientation if isinstance(truth_config, Pseudo6DofTruthConfig) else None
        default_estimators = {
            "translation-only": ("translation-dead-reckoning",),
            "rotation-only": ("attitude-dead-reckoning",),
        }.get(truth_mode, ("dead-reckoning", "mekf"))
        estimator = payload.get("estimators", sensor.get("estimators", navigation.get("estimators", default_estimators)))
        if "estimator" in navigation and "estimators" not in payload and "estimators" not in sensor:
            estimator = navigation["estimator"]
        estimator_modes: tuple[str, ...]
        if isinstance(estimator, str):
            estimator_modes = (estimator,)
        elif isinstance(estimator, Sequence):
            estimator_modes = tuple(str(item) for item in estimator)
        else:
            raise ValueError("estimators must be a string or sequence")
        return cls(
            sensor_name=str(sensor.get("name", "imu")),
            vehicle_name=str(sensor.get("vehicle", "1")),
            scenario_id=str(payload.get("scenario_id", source.stem)),
            provider=provider_config.kind,
            truth_mode=truth_mode,
            alignment="velocity" if orientation_config is None else orientation_config.alignment,
            speed_threshold_mps=1.0 if orientation_config is None else orientation_config.speed_threshold_mps,
            bank_source="controller" if orientation_config is None else orientation_config.bank_source,
            bank_default_rad=0.0 if orientation_config is None else orientation_config.bank_default_rad,
            yaw_source="controller-then-heading" if orientation_config is None else orientation_config.yaw_source,
            zero_speed_policy="hold-then-initial-frame" if orientation_config is None else orientation_config.zero_speed_policy,
            pole_crossing_policy="parallel-transport" if orientation_config is None else orientation_config.pole_crossing_policy,
            cadence_s=None if sensor.get("cadence_s") is None else float(sensor["cadence_s"]),
            phase_s=float(sensor.get("phase_s", 0.0)),
            sample_mode=str(sensor.get("sample", sensor.get("sample_mode", "instantaneous"))),
            delivery_s=float(sensor.get("delivery_s", 0.0)),
            truth_policy=str(sensor.get("truth", sensor.get("truth_policy", "boundary"))),
            rate_policy=str(sensor.get("rate_policy", "split")),
            earth_rate_mode=str(earth.get("rate_mode", "source")),
            earth_omega_rad_s=None if earth.get("omega_rad_s") is None else float(earth["omega_rad_s"]),
            drop_every_n=None if drop_policy.get("every_n") is None else int(drop_policy["every_n"]),
            profile_path=profile_path,
            profile_name=profile_name,
            profile_category=profile_category,
            seed=None if sensor.get("seed") is None else int(sensor["seed"]),
            body_from_sensor=mounting,
            lever_arm_body_m=lever_arm,
            estimator_modes=estimator_modes,
            source_path=source,
            truth_config=truth_config,
            provider_config=provider_config,
            observation_model=parse_observation_config(observation),
            feedback=parse_navigation_feedback(feedback),
        )

    def to_metadata(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "scenario_id": self.scenario_id,
            "provider": self.provider,
            "truth_mode": self.truth_mode,
            "provider_config": self.resolved_provider_config().model_dump(mode="json"),
            "truth_config": self.resolved_truth_config().model_dump(mode="json"),
            "channel_sources": {
                "translation": "not-consumed-by-rotation-only" if self.truth_mode == "rotation-only" else "vehicle-or-substituted-eci-truth",
                "rotation": "external-rotational-provider" if self.truth_mode == "hybrid-6dof" else "vehicle-or-external-truth",
            },
            "orientation_policy": {
                "alignment": self.alignment,
                "speed_threshold_mps": self.speed_threshold_mps,
                "bank_source": self.bank_source,
                "bank_default_rad": self.bank_default_rad,
                "yaw_source": self.yaw_source,
                "zero_speed_policy": self.zero_speed_policy,
                "pole_crossing_policy": self.pole_crossing_policy,
            },
            "sensor_name": self.sensor_name,
            "vehicle_name": self.vehicle_name,
            "cadence_s": self.cadence_s,
            "phase_s": self.phase_s,
            "sample_mode": self.sample_mode,
            "delivery_s": self.delivery_s,
            "truth_policy": self.truth_policy,
            "rate_policy": self.rate_policy,
            "earth_rate_mode": self.earth_rate_mode,
            "earth_omega_rad_s": self.earth_omega_rad_s,
            "drop_every_n": self.drop_every_n,
            "profile_path": None if self.profile_path is None else str(self.profile_path),
            "profile_name": self.profile_name,
            "profile_category": self.profile_category,
            "seed": self.seed,
            "body_from_sensor": None if self.body_from_sensor is None else self.body_from_sensor.tolist(),
            "lever_arm_body_m": self.lever_arm_body_m,
            "estimator_modes": list(self.estimator_modes),
            "observation_model": None if self.observation_model is None else self.observation_model.model_dump(mode="json"),
            "feedback": self.feedback.model_dump(mode="json"),
            "source_path": None if self.source_path is None else str(self.source_path),
        }


SensorAdapterBuilder = Callable[[SensorScenarioSpec], Any]


class SensorAdapterFactory:
    """Registry-backed factory for provider implementations."""

    _builders: ClassVar[dict[str, SensorAdapterBuilder]] = {}

    @classmethod
    def register(cls, kind: str, builder: SensorAdapterBuilder) -> None:
        normalized = kind.strip()
        if not normalized:
            raise ValueError("sensor provider kind must not be empty")
        if normalized in cls._builders:
            raise ValueError(f"sensor provider kind {normalized!r} is already registered")
        cls._builders[normalized] = builder

    @classmethod
    def create(cls, spec: SensorScenarioSpec) -> Any:
        kind = spec.resolved_provider_config().kind
        builder = cls._builders.get(kind)
        if builder is None:
            raise ValueError(f"unsupported sensor provider kind {kind!r}")
        return builder(spec)


def _mounting_kwargs(spec: SensorScenarioSpec) -> dict[str, object]:
    return {
        "body_from_sensor": spec.body_from_sensor,
        "lever_arm_body_m": None if spec.lever_arm_body_m is None else np.asarray(spec.lever_arm_body_m),
    }


def _build_translation_acceleration_adapter(spec: SensorScenarioSpec) -> Any:
    return TranslationAccelerationAdapter()


def _build_gyro_adapter(spec: SensorScenarioSpec) -> Any:
    return IdealGyroscopeAdapter()


def _build_ideal_imu_adapter(spec: SensorScenarioSpec) -> Any:
    return IdealImuAdapter(
        body_from_sensor=spec.body_from_sensor,
        lever_arm_body_m=None if spec.lever_arm_body_m is None else np.asarray(spec.lever_arm_body_m),
    )


def _build_imu_error_model_adapter(spec: SensorScenarioSpec) -> Any:
    if spec.profile_path is not None:
        return ImuErrorModelAdapter.from_profile(
            spec.profile_path,
            seed=spec.seed,
            body_from_sensor=spec.body_from_sensor,
            lever_arm_body_m=None if spec.lever_arm_body_m is None else np.asarray(spec.lever_arm_body_m),
        )
    if spec.profile_name is not None:
        return ImuErrorModelAdapter.from_example_profile(
            spec.profile_name,
            category=spec.profile_category,
            seed=spec.seed,
            body_from_sensor=spec.body_from_sensor,
            lever_arm_body_m=None if spec.lever_arm_body_m is None else np.asarray(spec.lever_arm_body_m),
        )
    return ImuErrorModelAdapter.from_config(
        seed=spec.seed,
        body_from_sensor=spec.body_from_sensor,
        lever_arm_body_m=None if spec.lever_arm_body_m is None else np.asarray(spec.lever_arm_body_m),
    )


SensorAdapterFactory.register("translation-acceleration", _build_translation_acceleration_adapter)
SensorAdapterFactory.register("ideal-gyroscope", _build_gyro_adapter)
SensorAdapterFactory.register("ideal", _build_ideal_imu_adapter)
SensorAdapterFactory.register("imu-error-model", _build_imu_error_model_adapter)


def _packet_record(packet: MeasurementPacket[Any], provenance: Mapping[str, object] | None = None) -> dict[str, object]:
    payload_contract: dict[str, object]
    if isinstance(packet.payload, AccelerationIncrement) or (provenance or {}).get("provider") == "translation-acceleration":
        payload_contract = {
            "frame": "ECI",
            "delta_v_unit": "m/s",
            "measurement": "specific-force",
        }
    elif isinstance(packet.payload, GyroIncrement) or (provenance or {}).get("provider") == "ideal-gyroscope":
        payload_contract = {
            "frame": "body",
            "delta_theta_unit": "rad",
            "measurement": "angular-rate",
        }
    else:
        payload_contract = {
            "frame": "body",
            "delta_v_unit": "m/s",
            "delta_theta_unit": "rad",
        }
    record: dict[str, object] = {
        "sampled_at_s": packet.sampled_at_s,
        "available_at_s": packet.available_at_s,
        "interval_start_s": packet.interval_start_s,
        "valid": packet.valid,
        "payload_contract": payload_contract,
        "provenance": dict(provenance or {}),
    }
    if packet.payload is None:
        record["payload"] = None
    elif isinstance(packet.payload, AccelerationIncrement):
        record["payload"] = {
            "delta_v_eci_mps": packet.payload.delta_v_eci_mps.tolist(),
            "start_time_s": packet.payload.start_time_s,
            "end_time_s": packet.payload.end_time_s,
        }
    elif isinstance(packet.payload, GyroIncrement):
        record["payload"] = {
            "delta_theta_body_rad": packet.payload.delta_theta_body_rad.tolist(),
            "start_time_s": packet.payload.start_time_s,
            "end_time_s": packet.payload.end_time_s,
        }
    else:
        record["payload"] = {
            "delta_v_body_mps": np.asarray(packet.payload.delta_v_body_mps).tolist(),
            "delta_theta_body_rad": np.asarray(packet.payload.delta_theta_body_rad).tolist(),
            "start_time_s": packet.payload.start_time_s,
            "end_time_s": packet.payload.end_time_s,
        }
    return record


def _navigation_record(state: Any) -> dict[str, object]:
    if isinstance(state, AttitudeNavigationState):
        return {
            "time_s": state.time_s,
            "position_eci_m": None,
            "velocity_eci_mps": None,
            "orientation_eci_from_body": state.orientation_eci_from_body.tolist(),
        }
    record: dict[str, object] = {
        "time_s": state.time_s,
        "position_eci_m": state.position_eci_m.tolist(),
        "velocity_eci_mps": state.velocity_eci_mps.tolist(),
    }
    if isinstance(state, NavigationState):
        record.update(
            {
                "orientation_eci_from_body": state.orientation_eci_from_body.tolist(),
                "accelerometer_bias_body_mps2": state.accelerometer_bias_body_mps2.tolist(),
                "gyroscope_bias_body_radps": state.gyroscope_bias_body_radps.tolist(),
                "body_rate_body_radps": state.body_rate_body_radps.tolist(),
            }
        )
    else:
        record["orientation_eci_from_body"] = None
    return record


@dataclass(slots=True)
class SensorScenarioRuntime:
    """Live bus, estimators, and records for one lowered runtime problem."""

    spec: SensorScenarioSpec
    bus: SensorBus
    problem: RuntimeProblem | None = None
    estimators: dict[str, _Navigator] = field(default_factory=dict)
    histories: dict[str, list[dict[str, object]]] = field(default_factory=dict)
    source_inputs: dict[str, object] = field(default_factory=dict)
    completed: bool | None = None
    stop_reason: str | None = None
    max_steps: int | None = None
    plot_manifest: dict[str, object] = field(default_factory=dict)
    estimator_gaps: dict[str, list[dict[str, float]]] = field(default_factory=dict)
    truth_contract: dict[str, object] = field(default_factory=dict)
    feedback_selected_count: int = 0
    feedback_last_available_s: float | None = None

    def finalize(self, *, completed: bool, stop_reason: str | None, max_steps: int) -> None:
        self.completed = completed
        self.stop_reason = stop_reason
        self.max_steps = max_steps

    def _apply_feedback(self, vehicle: Any, state: NavigationState, available_at_s: float) -> None:
        """Publish an estimator state to the controller without changing plant truth."""

        vehicle.controller_state = _feedback_rigid_state(vehicle, state)
        self.feedback_selected_count += 1
        self.feedback_last_available_s = float(available_at_s)

    def sensor_checkpoint(self) -> dict[str, object]:
        """Return the bound sensor model checkpoint for deterministic replay."""

        binding = next(item for item in self.bus.bindings if item.name == self.spec.sensor_name)
        snapshot = getattr(binding.model, "snapshot", None)
        if not callable(snapshot):
            raise TypeError(f"sensor model {binding.name!r} does not support checkpointing")
        checkpoint = snapshot()
        if not isinstance(checkpoint, Mapping):
            raise TypeError("sensor checkpoint must be a mapping")
        return dict(checkpoint)

    def restore_sensor_checkpoint(self, checkpoint: Mapping[str, object]) -> None:
        """Restore the bound sensor model checkpoint after an explicit rebind."""

        binding = next(item for item in self.bus.bindings if item.name == self.spec.sensor_name)
        restore = getattr(binding.model, "restore", None)
        if not callable(restore):
            raise TypeError(f"sensor model {binding.name!r} does not support checkpointing")
        restore(checkpoint)

    def artifact(self) -> dict[str, object]:
        packets = self.bus.packets(self.spec.sensor_name)
        binding = next(item for item in self.bus.bindings if item.name == self.spec.sensor_name)
        queued = self.bus.packets(self.spec.sensor_name, delivered=False)
        vehicle = None if self.problem is None else self.problem.vehicles.get(self.spec.vehicle_name)
        last_truth_time = None if vehicle is None or not vehicle.history else vehicle.history[-1].time
        next_requested_time = None
        if last_truth_time is not None:
            next_requested_time = binding.clock.next_truth_time(last_truth_time)
        valid_packets = tuple(packet for packet in packets if packet.valid)
        measurements = {self.spec.sensor_name: [_packet_record(packet, binding.provenance) for packet in packets]}
        dropped_measurements = [_packet_record(packet, binding.provenance) for packet in binding.dropped_packets]
        estimates = {
            name: {
                "sample_count": len(history),
                "history": history,
                "final": history[-1] if history else None,
            }
            for name, history in self.histories.items()
        }
        packet_hash = _stable_hash(measurements)
        estimator_hash = _stable_hash(estimates)
        return {
            "schema_version": 1,
            "execution": "accepted-truth-measurement-bus",
            "scenario_identity": self.spec.scenario_id,
            "source_inputs": dict(self.source_inputs),
            "spec": self.spec.to_metadata(),
            "truth_contract": {
                **dict(self.truth_contract),
                "frame": "ECI",
                "channel_usage": {
                    "translation": "unused" if self.spec.truth_mode == "rotation-only" else "used",
                    "rotation": "used" if self.spec.truth_mode in {"vehicle", "pseudo-6dof", "hybrid-6dof", "rotation-only"} else "unavailable",
                },
                "earth_rate": {
                    "mode": self.spec.earth_rate_mode,
                    "omega_rad_s": self.spec.earth_omega_rad_s if self.spec.earth_rate_mode == "explicit" else (7.2921151467e-5 if self.spec.earth_rate_mode == "nominal" else None),
                },
                "support_mode": self.truth_contract.get("support_mode", "free-flight"),
                "velocity_without_gravity": "accepted-acceleration-minus-gravity-integrated-when-available",
                "solver_stage_samples": False,
            },
            "sensor_bindings": self.bus.to_metadata()["bindings"],
            "checkpointing": {
                "sensor_model": binding.to_metadata()["checkpointing"],
                "runtime_rebind_required": True,
            },
            "measurement_summary": {
                "emitted": binding.samples_emitted,
                "valid": len(valid_packets),
                "invalid": binding.invalid_samples,
                "dropped": binding.dropped_samples,
                "delivered": len(packets),
                "queued": len(queued),
                "timeout": self.stop_reason == "max_steps",
                "last_accepted_truth_time_s": last_truth_time,
                "next_requested_sensor_time_s": next_requested_time,
            },
            "termination": {"completed": self.completed, "stop_reason": self.stop_reason, "max_steps": self.max_steps},
            "rerun": self._rerun_hint(),
            "measurements": measurements,
            "dropped_measurements": {self.spec.sensor_name: dropped_measurements},
            "estimators": estimates,
            "estimator_gaps": {name: list(gaps) for name, gaps in self.estimator_gaps.items()},
            "feedback": {
                **self.spec.feedback.model_dump(mode="json"),
                "selected_count": self.feedback_selected_count,
                "last_available_s": self.feedback_last_available_s,
                "startup_fallback": "plant-truth-until-first-delivered-estimate",
                "plant_truth_immutable": True,
            },
            "reproducibility": {"measurement_sha256": packet_hash, "estimator_sha256": estimator_hash},
            "plots": dict(self.plot_manifest) if self.plot_manifest else {"status": "not-rendered"},
            "limitations": [
                "Research estimator implementation; not flight qualified.",
                "Checkpoint load requires explicit sensor and estimator rebind.",
            ],
        }

    def write_artifacts(self, output_dir: str | Path, case_index: int) -> tuple[Path, ...]:
        destination = Path(output_dir) / "sensor" / f"case-{case_index}"
        destination.mkdir(parents=True, exist_ok=True)
        artifact_path = destination / "sensor-execution.json"
        artifact_path.write_text(json.dumps(self.artifact(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        paths = [artifact_path]
        binding = next(item for item in self.bus.bindings if item.name == self.spec.sensor_name)
        packets = self.bus.packets(self.spec.sensor_name)
        packet_path = destination / f"{self.spec.sensor_name}-measurements.jsonl"
        packet_path.write_text("".join(json.dumps(_packet_record(packet, binding.provenance), sort_keys=True) + "\n" for packet in packets), encoding="utf-8")
        paths.append(packet_path)
        dropped_path = destination / f"{self.spec.sensor_name}-dropped.jsonl"
        dropped_path.write_text("".join(json.dumps(_packet_record(packet, binding.provenance), sort_keys=True) + "\n" for packet in binding.dropped_packets), encoding="utf-8")
        paths.append(dropped_path)
        for name, history in self.histories.items():
            estimate_path = destination / f"{name}-estimates.jsonl"
            estimate_path.write_text("".join(json.dumps(record, sort_keys=True) + "\n" for record in history), encoding="utf-8")
            paths.append(estimate_path)
        return tuple(paths)

    def write_manifest(self, output_dir: str | Path, case_index: int) -> Path:
        destination = Path(output_dir) / "sensor" / f"case-{case_index}" / "sensor-execution.json"
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(self.artifact(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return destination

    def _rerun_hint(self) -> dict[str, object] | None:
        if self.stop_reason != "max_steps" or self.max_steps is None:
            return None
        problem = self.source_inputs.get("problem", {})
        tables = self.source_inputs.get("tables", ())
        sidecar = self.source_inputs.get("sidecar")
        problem_path = problem.get("path") if isinstance(problem, Mapping) else None
        table_paths = [item.get("path") for item in tables if isinstance(item, Mapping)] if isinstance(tables, Sequence) else []
        sidecar_path = sidecar.get("path") if isinstance(sidecar, Mapping) else None
        recommended = max(self.max_steps * 2, self.max_steps + 1)
        command = ["taoryx", "run", str(problem_path or "<problem.prb>"), *(str(path) for path in table_paths)]
        if sidecar_path is not None:
            command.extend(("--sensor-spec", str(sidecar_path)))
        command.extend(("--max-steps", str(recommended)))
        return {
            "reason": "max_steps",
            "recommended_max_steps": recommended,
            "command": " ".join(command),
        }


def attach_sensor_scenario(
    problem: RuntimeProblem,
    spec: SensorScenarioSpec,
    *,
    rotational_truth_provider: RotationTruthProvider | None = None,
) -> SensorScenarioRuntime:
    """Attach one configured sensor and packet-only navigation consumers."""

    vehicle = problem.vehicles.get(spec.vehicle_name)
    if vehicle is None:
        raise KeyError(f"sensor scenario references unknown vehicle {spec.vehicle_name!r}")
    if vehicle.truth_provider is None:
        raise RuntimeError(f"vehicle {vehicle.name!r} has no lowered truth provider")
    truth_provider = vehicle.truth_provider
    if spec.truth_mode == "pseudo-6dof":
        truth_definition = spec.resolved_truth_config()
        if not isinstance(truth_definition, Pseudo6DofTruthConfig):
            raise TypeError("pseudo-6dof truth must resolve to a Pseudo6DofTruthConfig")
        policy = AttitudePolicyFactory.create(truth_definition.orientation.model_dump(mode="python"))
        truth_provider = Pseudo6DofTruthProvider(truth_provider, policy)
    elif spec.truth_mode in {"hybrid-6dof", "rotation-only"} and rotational_truth_provider is not None:
        truth_provider = CompositeTruthProvider(truth_provider, rotational_truth_provider)
    elif spec.truth_mode == "hybrid-6dof":
        if rotational_truth_provider is None:
            raise ValueError("hybrid-6dof truth requires an injected rotational_truth_provider")
    elif rotational_truth_provider is not None:
        raise ValueError("rotational_truth_provider is only valid for hybrid-6dof or rotation-only truth")
    adapter = build_observation_pipeline(SensorAdapterFactory.create(spec), spec.observation_model)
    existing_clock = next((clock for clock in problem.sensor_clocks if clock.name == spec.sensor_name), None)
    profile_period = adapter.provenance.get("sample_period_s", 0.01)
    if not isinstance(profile_period, (int, float)):
        profile_period = 0.01
    cadence_s = spec.cadence_s or (existing_clock.cadence_s if existing_clock is not None else None) or float(profile_period)
    clock = SensorClockSpec(
        spec.sensor_name,
        "accelerometer" if spec.truth_mode == "translation-only" else ("gyroscope" if spec.truth_mode == "rotation-only" else "imu"),
        cadence_s=cadence_s,
        phase_s=spec.phase_s,
        sample_mode=cast(Any, spec.sample_mode),
        delivery_s=spec.delivery_s,
        truth_policy=cast(Any, spec.truth_policy),
        rate_policy=cast(Any, spec.rate_policy),
    )
    adapter_provenance = getattr(adapter, "provenance", {})
    provenance = {
        **(dict(adapter_provenance) if isinstance(adapter_provenance, Mapping) else {}),
        "scenario_spec": spec.to_metadata(),
    }
    bus = SensorBus()
    bus.register(SensorBinding(spec.sensor_name, spec.vehicle_name, clock, adapter, provenance=provenance, truth_provider=truth_provider))
    bus.attach(problem)
    initial_truth = truth_provider(vehicle.state)
    if spec.truth_mode == "translation-only":
        initial_state: Any = TranslationNavigationState(
            initial_truth.time_s,
            initial_truth.position_eci_m,
            initial_truth.velocity_eci_mps,
        )
    elif spec.truth_mode == "rotation-only":
        if initial_truth.orientation_eci_from_body is None:
            raise ValueError("rotation-only truth requires committed orientation truth")
        initial_state = AttitudeNavigationState(
            initial_truth.time_s,
            initial_truth.orientation_eci_from_body,
        )
    else:
        if initial_truth.orientation_eci_from_body is None or initial_truth.angular_rate_body_radps is None:
            raise ValueError(f"truth mode {spec.truth_mode!r} requires orientation and body-rate channels")
        initial_state = NavigationState(
            initial_truth.time_s,
            initial_truth.position_eci_m,
            initial_truth.velocity_eci_mps,
            initial_truth.orientation_eci_from_body,
        )
    runtime = SensorScenarioRuntime(spec, bus, problem=problem)
    runtime.truth_contract = truth_provider_contract(truth_provider)
    if spec.feedback.source != "plant-truth":
        if tuple(vehicle.state.value_names) != RIGID_BODY_STATE_NAMES:
            raise ValueError("navigation feedback requires a rigid-body 6-DOF vehicle state")
        feedback_start_time = initial_truth.time_s

        def feedback_guard(time_s: float) -> None:
            if spec.feedback.stale_policy != "fail":
                return
            last_available = runtime.feedback_last_available_s
            age = time_s - feedback_start_time if last_available is None else time_s - last_available
            if spec.feedback.max_age_s is not None and age > spec.feedback.max_age_s + 1.0e-12:
                last_available_text = "none" if last_available is None else f"{last_available:g}s"
                raise RuntimeError(
                    f"navigation feedback source {spec.feedback.source!r} is stale at {time_s:g}s; "
                    f"last delivered estimate was {last_available_text}"
                )

        problem.feedback_guard = feedback_guard
    if spec.drop_every_n is not None:
        drop_every_n = spec.drop_every_n
        emission_index = 0

        def drop_packet(_: MeasurementPacket[Any]) -> bool:
            nonlocal emission_index
            emission_index += 1
            return emission_index % drop_every_n == 0

        binding = bus.bindings[0]
        binding.drop_predicate = drop_packet
    for mode in spec.estimator_modes:
        normalized = mode.casefold().replace("_", "-")
        estimator: _Navigator
        if spec.truth_mode == "translation-only" and normalized in {"dead-reckoning", "translation-dead-reckoning", "translation-dr", "dr"}:
            estimator = TranslationOnlyNavigator(initial_state, initial_truth.gravity_eci_mps2)
            name = "translation_dead_reckoning"
        elif spec.truth_mode == "rotation-only" and normalized in {"attitude-dead-reckoning", "rotation-dead-reckoning", "gyro-dead-reckoning", "gyro-dr"}:
            estimator = AttitudeOnlyNavigator(initial_state)
            name = "attitude_dead_reckoning"
        elif normalized in {"dead-reckoning", "dr"}:
            estimator = DeadReckoningNavigator(initial_state, initial_truth.gravity_eci_mps2)
            name = "dead_reckoning"
        elif normalized == "mekf":
            if spec.truth_mode in {"translation-only", "rotation-only"}:
                raise ValueError("MEKF requires both acceleration and rotation channels; use the mode-specific dead-reckoner")
            estimator = MultiplicativeEkf(initial_state, np.eye(15), initial_truth.gravity_eci_mps2)
            name = "mekf"
        else:
            raise ValueError(f"unsupported estimator mode {mode!r}")
        runtime.estimators[name] = estimator
        runtime.histories[name] = []
        runtime.estimator_gaps[name] = []

    def consume(packet: MeasurementPacket[Any]) -> None:
        for name, estimator in runtime.estimators.items():
            if packet.valid and packet.payload is not None:
                start_time = packet.payload.start_time_s
                if start_time > estimator.state.time_s + 1.0e-10:
                    runtime.estimator_gaps[name].append(
                        {
                            "from_time_s": estimator.state.time_s,
                            "to_time_s": start_time,
                        }
                    )
                    prior = estimator.state
                    if isinstance(prior, TranslationNavigationState):
                        estimator.state = TranslationNavigationState(start_time, prior.position_eci_m, prior.velocity_eci_mps)
                    elif isinstance(prior, AttitudeNavigationState):
                        estimator.state = AttitudeNavigationState(start_time, prior.orientation_eci_from_body)
                    else:
                        estimator.state = NavigationState(
                            start_time,
                            prior.position_eci_m,
                            prior.velocity_eci_mps,
                            prior.orientation_eci_from_body,
                            prior.accelerometer_bias_body_mps2,
                            prior.gyroscope_bias_body_radps,
                            prior.body_rate_body_radps,
                        )
                elif start_time < estimator.state.time_s - 1.0e-10:
                    runtime.estimator_gaps[name].append(
                        {
                            "from_time_s": estimator.state.time_s,
                            "to_time_s": start_time,
                        }
                    )
                    return
            state = estimator.propagate(packet)
            if packet.valid:
                runtime.histories[name].append(_navigation_record(state))
                if name == _feedback_estimator_name(spec.feedback.source) and isinstance(state, NavigationState):
                    runtime._apply_feedback(vehicle, state, packet.available_at_s)

    bus.subscribe(spec.sensor_name, consume)
    bus.initialize(problem)
    return runtime


def _feedback_estimator_name(source: str) -> str | None:
    return {"mekf": "mekf", "dead-reckoning": "dead_reckoning"}.get(source)


def _quaternion_from_rotation(rotation: np.ndarray) -> Quaternion:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = 0.5 / np.sqrt(trace + 1.0)
        return Quaternion(
            0.25 / scale,
            (rotation[2, 1] - rotation[1, 2]) * scale,
            (rotation[0, 2] - rotation[2, 0]) * scale,
            (rotation[1, 0] - rotation[0, 1]) * scale,
        ).normalized()
    index = int(np.argmax(np.diag(rotation)))
    if index == 0:
        scale = 2.0 * np.sqrt(max(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2], 1.0e-15))
        return Quaternion(
            (rotation[2, 1] - rotation[1, 2]) / scale,
            0.25 * scale,
            (rotation[0, 1] + rotation[1, 0]) / scale,
            (rotation[0, 2] + rotation[2, 0]) / scale,
        ).normalized()
    if index == 1:
        scale = 2.0 * np.sqrt(max(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2], 1.0e-15))
        return Quaternion(
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[0, 1] + rotation[1, 0]) / scale,
            0.25 * scale,
            (rotation[1, 2] + rotation[2, 1]) / scale,
        ).normalized()
    scale = 2.0 * np.sqrt(max(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1], 1.0e-15))
    return Quaternion(
        (rotation[1, 0] - rotation[0, 1]) / scale,
        (rotation[0, 2] + rotation[2, 0]) / scale,
        (rotation[1, 2] + rotation[2, 1]) / scale,
        0.25 * scale,
    ).normalized()


def _feedback_rigid_state(vehicle: Any, state: NavigationState) -> RigidBody6DofState:
    plant = RigidBody6DofState.from_values(vehicle.state.time, tuple(vehicle.state.values))
    return RigidBody6DofState(
        state.time_s,
        FrameVector3(Vector3(*state.position_eci_m), Frame.ECIC),
        FrameVector3(Vector3(*state.velocity_eci_mps), Frame.ECIC),
        _quaternion_from_rotation(state.orientation_eci_from_body),
        Vector3(*state.body_rate_body_radps),
        plant.mass,
        plant.propellant_mass,
        plant.heat_load,
        plant.peak_heat_rate,
    )


def _stable_hash(value: object) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def render_sensor_scenario_plots(
    artifact: Any,
    execution: Mapping[str, object],
    directory: str | Path,
    *,
    dpi: int = 140,
) -> dict[str, object]:
    """Render the standard sensor evidence bundle without reopening source files."""

    if dpi <= 0:
        raise ValueError("plot dpi must be positive")
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    raw_spec = execution.get("spec", {})
    spec = raw_spec if isinstance(raw_spec, Mapping) else {}
    sensor_name = str(spec.get("sensor_name", "imu"))
    raw_measurements = execution.get("measurements", {})
    measurements = raw_measurements if isinstance(raw_measurements, Mapping) else {}
    raw_packets = measurements.get(sensor_name, ())
    packets = tuple(item for item in raw_packets if isinstance(item, Mapping))
    raw_dropped = execution.get("dropped_measurements", {})
    dropped_measurements = raw_dropped if isinstance(raw_dropped, Mapping) else {}
    raw_dropped_packets = dropped_measurements.get(sensor_name, ())
    dropped_packets = tuple(item for item in raw_dropped_packets if isinstance(item, Mapping))
    estimators = execution.get("estimators", {})
    estimator_records = estimators if isinstance(estimators, Mapping) else {}
    vehicle_name = str(spec.get("vehicle_name", "1"))
    vehicles = getattr(artifact, "vehicles", {})
    vehicle: Any = vehicles.get(vehicle_name)
    rendered: list[dict[str, object]] = []

    def save(name: str, figure: Any, title: str) -> None:
        path = destination / f"{name}.png"
        figure.savefig(path, format="png", dpi=dpi, bbox_inches="tight")
        plt.close(figure)
        rendered.append({"name": name, "path": path.name, "title": title})

    sampled = [float(item["sampled_at_s"]) for item in packets if isinstance(item.get("sampled_at_s"), (int, float))]
    available = [float(item["available_at_s"]) for item in packets if isinstance(item.get("available_at_s"), (int, float))]
    valid = [bool(item.get("valid", False)) for item in packets]
    dropped_times = [float(item["sampled_at_s"]) for item in dropped_packets if isinstance(item.get("sampled_at_s"), (int, float))]
    latency = [available[index] - sampled[index] for index in range(min(len(sampled), len(available)))]

    def quaternion_matrix(values: Sequence[float]) -> np.ndarray:
        w, x, y, z = values
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
                [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
                [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=float,
        )

    def rotation_angle(rotation: np.ndarray) -> float:
        return float(np.arccos(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)))

    def channel_values(name: str) -> list[float] | None:
        channels = getattr(vehicle, "channels", {}) if vehicle is not None else {}
        channel = channels.get(name) or channels.get(f"taos.{name}")
        values = getattr(channel, "values", None)
        if not isinstance(values, Sequence) or any(not isinstance(value, (int, float)) for value in values):
            return None
        return [float(value) for value in values]

    figure, axis = plt.subplots(figsize=(7.2, 3.8), layout="constrained")
    if sampled:
        axis.plot(sampled, available, color="#0f766e", linewidth=1.6, label="available time")
        axis.plot(sampled, sampled, color="#94a3b8", linewidth=1.0, linestyle="--", label="sampled time")
    axis.set_title("Packet cadence and delivery latency", loc="left", fontweight="semibold")
    axis.set_xlabel("sampled time (s)")
    axis.set_ylabel("time (s)")
    axis.grid(True, color="#cbd5e1", linewidth=0.8)
    if sampled:
        axis.legend(loc="best")
    save("packet-cadence-latency", figure, "Packet cadence and delivery latency")

    figure, axis = plt.subplots(figsize=(7.2, 3.8), layout="constrained")
    if sampled:
        axis.scatter(sampled, [1.0 if item else 0.0 for item in valid], s=10, c=["#0f766e" if item else "#dc2626" for item in valid])
    if dropped_times:
        axis.scatter(dropped_times, [0.5] * len(dropped_times), marker="x", s=30, c="#dc2626", label="dropped")
    axis.set_title("Packet validity and delivered samples", loc="left", fontweight="semibold")
    axis.set_xlabel("sampled time (s)")
    axis.set_yticks((0.0, 1.0), labels=("invalid", "valid"))
    axis.grid(True, axis="x", color="#cbd5e1", linewidth=0.8)
    if dropped_times:
        axis.legend(loc="best")
    save("packet-validity", figure, "Packet validity and delivered samples")

    truth_times: list[float] = []
    truth_position: list[list[float]] = [[], [], []]
    truth_rate: list[list[float]] = [[], [], []]
    truth_orientation: list[np.ndarray] = []
    if vehicle is not None:
        truth_times = [float(value) for value in vehicle.times]
        for index, channel_name in enumerate(("position.ecfc.x", "position.ecfc.y", "position.ecfc.z")):
            channel = vehicle.channels.get(channel_name)
            truth_position[index] = [] if channel is None else [float(value) for value in channel.values if value is not None]
        for index, channel_name in enumerate(("taos.wx", "taos.wy", "taos.wz")):
            channel = vehicle.channels.get(channel_name)
            truth_rate[index] = [] if channel is None else [float(value) for value in channel.values if value is not None]
        quaternion_channels = [channel_values(name) for name in ("qw", "qx", "qy", "qz")]
        if all(values is not None for values in quaternion_channels):
            truth_orientation = [quaternion_matrix(values) for values in zip(*quaternion_channels, strict=True)]

    def records(name: str) -> list[Mapping[str, object]]:
        raw = estimator_records.get(name, {})
        history = raw.get("history", ()) if isinstance(raw, Mapping) else ()
        return [item for item in history if isinstance(item, Mapping)] if isinstance(history, Sequence) else []

    def vector(record: Mapping[str, object], key: str) -> np.ndarray:
        value = record.get(key)
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise ValueError(f"estimator record {key!r} is not a vector")
        return np.asarray(tuple(float(item) for item in value), dtype=float)

    def rotation(record: Mapping[str, object], key: str) -> np.ndarray:
        value = record.get(key)
        matrix = np.asarray(value, dtype=float)
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise ValueError(f"estimator record {key!r} is not a finite 3x3 rotation")
        return matrix

    def has_vector(record: Mapping[str, object], key: str) -> bool:
        value = record.get(key)
        return isinstance(value, Sequence) and not isinstance(value, (str, bytes))

    def record_time(record: Mapping[str, object]) -> float:
        value = record.get("time_s")
        if not isinstance(value, (int, float)):
            raise ValueError("estimator record time_s is not numeric")
        return float(value)

    figure, axis = plt.subplots(figsize=(7.2, 3.8), layout="constrained")
    if truth_position[0] and truth_position[1]:
        axis.plot(truth_position[0], truth_position[1], color="#0f172a", linewidth=1.8, label="truth")
    for name, color in (("dead_reckoning", "#2563eb"), ("mekf", "#d97706")):
        history = [item for item in records(name) if has_vector(item, "position_eci_m")]
        if history:
            axis.plot([vector(item, "position_eci_m")[0] for item in history], [vector(item, "position_eci_m")[1] for item in history], color=color, linewidth=1.2, label=name)
    axis.set_title("Flown trajectory and estimator paths", loc="left", fontweight="semibold")
    axis.set_xlabel("ECI/ECFC x (m)")
    axis.set_ylabel("ECI/ECFC y (m)")
    axis.grid(True, color="#cbd5e1", linewidth=0.8)
    if truth_position[0] or records("dead_reckoning") or records("mekf"):
        axis.legend(loc="best")
    save("trajectory-estimates", figure, "Flown trajectory and estimator paths")

    figure, axes = plt.subplots(2, 1, figsize=(7.2, 5.4), sharex=True, layout="constrained")
    error_plotted = False
    for name, color in (("dead_reckoning", "#2563eb"), ("mekf", "#d97706")):
        history = [item for item in records(name) if has_vector(item, "position_eci_m")]
        if history:
            estimate_times = [record_time(item) for item in history]
            for index, label in enumerate(("x", "y", "z")):
                axes[0].plot(estimate_times, [vector(item, "position_eci_m")[index] for item in history], color=color, alpha=0.75, label=f"{name} {label}")
    if truth_times and truth_position[0]:
        for index, label in enumerate(("x", "y", "z")):
            axes[0].plot(truth_times[: len(truth_position[index])], truth_position[index], color="#0f172a", linewidth=1.0, linestyle="--", label=f"truth {label}")
    axes[0].set_title("Truth versus estimated position", loc="left", fontweight="semibold")
    axes[0].set_ylabel("position (m)")
    axes[0].grid(True, color="#cbd5e1", linewidth=0.8)
    for name, color in (("dead_reckoning", "#2563eb"), ("mekf", "#d97706")):
        history = [item for item in records(name) if has_vector(item, "position_eci_m")]
        if history and truth_times and truth_position[0]:
            error_times = np.asarray([record_time(item) for item in history])
            errors = []
            for item_time, item in zip(error_times, history, strict=True):
                truth = np.array([np.interp(item_time, truth_times[: len(truth_position[index])], truth_position[index]) for index in range(3)])
                estimate = vector(item, "position_eci_m")
                errors.append(float(np.linalg.norm(estimate - truth)))
            axes[1].plot(error_times, errors, color=color, label=name)
            error_plotted = True
    axes[1].set_title("Estimator position error growth", loc="left", fontweight="semibold")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("error norm (m)")
    axes[1].grid(True, color="#cbd5e1", linewidth=0.8)
    if error_plotted:
        axes[1].legend(loc="best")
    save("truth-estimate-error", figure, "Truth versus estimated position and error growth")

    figure, axis = plt.subplots(figsize=(7.2, 3.8), layout="constrained")
    if truth_times and truth_rate[0]:
        for index, label in enumerate(("p", "q", "r")):
            axis.plot(truth_times[: len(truth_rate[index])], truth_rate[index], linewidth=1.2, label=f"truth {label}")
    axis.set_title("Flown attitude-rate evidence", loc="left", fontweight="semibold")
    axis.set_xlabel("time (s)")
    axis.set_ylabel("body rate (rad/s)")
    axis.grid(True, color="#cbd5e1", linewidth=0.8)
    if truth_rate[0]:
        axis.legend(loc="best")
    save("attitude-rate", figure, "Flown attitude-rate evidence")

    figure, axis = plt.subplots(figsize=(7.2, 3.8), layout="constrained")
    attitude_plotted = False
    if truth_orientation:
        reference = truth_orientation[0]
        axis.plot(
            truth_times[: len(truth_orientation)],
            [np.degrees(rotation_angle(reference.T @ orientation)) for orientation in truth_orientation],
            color="#0f172a",
            linewidth=1.2,
            label="truth from initial attitude",
        )
        attitude_plotted = True
    for name, color in (("dead_reckoning", "#2563eb"), ("mekf", "#d97706"), ("attitude_dead_reckoning", "#059669")):
        history = [item for item in records(name) if has_vector(item, "orientation_eci_from_body")]
        if not history:
            continue
        first = rotation(history[0], "orientation_eci_from_body")
        axis.plot(
            [record_time(item) for item in history],
            [
                np.degrees(
                    rotation_angle(
                        first.T @ rotation(item, "orientation_eci_from_body"),
                    )
                )
                for item in history
            ],
            color=color,
            linewidth=1.2,
            label=name,
        )
        attitude_plotted = True
    axis.set_title("Attitude orientation evidence", loc="left", fontweight="semibold")
    axis.set_xlabel("time (s)")
    axis.set_ylabel("rotation from initial attitude (deg)")
    axis.grid(True, color="#cbd5e1", linewidth=0.8)
    if attitude_plotted:
        axis.legend(loc="best")
    save("attitude-estimates", figure, "Attitude orientation evidence")

    figure, axis = plt.subplots(figsize=(7.2, 3.8), layout="constrained")
    events = getattr(artifact, "events", ())
    for event in events:
        if isinstance(event, Mapping) and isinstance(event.get("time"), (int, float)):
            axis.axvline(float(event["time"]), color="#7c3aed", linewidth=1.0)
    if truth_times:
        axis.plot(truth_times, [0.0] * len(truth_times), color="#64748b", linewidth=2.0)
    axis.set_title("Scenario phase and committed boundaries", loc="left", fontweight="semibold")
    axis.set_xlabel("time (s)")
    axis.set_yticks(())
    axis.grid(True, axis="x", color="#cbd5e1", linewidth=0.8)
    save("phase-boundaries", figure, "Scenario phase and committed boundaries")

    raw_summary = execution.get("measurement_summary", {})
    summary = raw_summary if isinstance(raw_summary, Mapping) else {}
    manifest = {
        "schema_version": 1,
        "renderer": "taoryx.runtime.sensor_scenario.render_sensor_scenario_plots",
        "dpi": dpi,
        "plots": rendered,
        "packet_latency_s": {"min": min(latency) if latency else None, "max": max(latency) if latency else None},
        "drop_count": summary.get("dropped", 0),
    }
    (destination / "plot-manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest
