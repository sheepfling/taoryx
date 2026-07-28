"""Committed-truth sensor contracts and the first external IMU adapter."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Generic, Protocol, TypeVar, cast

import numpy as np

Array3 = np.ndarray
MeasurementT = TypeVar("MeasurementT")


def _vector3(value: Array3, name: str) -> Array3:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3-vector")
    result = result.copy()
    result.setflags(write=False)
    return result


def _rotation(value: Array3, name: str) -> Array3:
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    if not np.allclose(result.T @ result, np.eye(3), atol=1.0e-8) or not np.isclose(np.linalg.det(result), 1.0, atol=1.0e-8):
        raise ValueError(f"{name} must be a proper rotation matrix")
    result = result.copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True, slots=True)
class TruthPoint:
    """One immutable, sensor-visible committed truth boundary.

    ``velocity_without_gravity_eci_mps`` is an explicit EOM-produced
    quantity. It is not ``velocity_eci_mps - gravity_eci_mps2``. ECI is the
    navigation frame so attitude changes include Earth rotation when the body
    is fixed to Earth. Translation-only providers may leave attitude and body
    rate unset; those providers expose acceleration channels without claiming
    a physical body frame or gyro measurement.
    """

    time_s: float
    position_eci_m: Array3
    velocity_eci_mps: Array3
    velocity_without_gravity_eci_mps: Array3
    orientation_eci_from_body: Array3 | None
    gravity_eci_mps2: Array3
    angular_rate_body_radps: Array3 | None
    acceleration_eci_mps2: Array3 | None = None
    angular_acceleration_body_radps2: Array3 | None = None
    temperature_celsius: float | None = None

    def __post_init__(self) -> None:
        if not np.isfinite(self.time_s):
            raise ValueError("truth time must be finite")
        object.__setattr__(self, "position_eci_m", _vector3(self.position_eci_m, "position_eci_m"))
        object.__setattr__(self, "velocity_eci_mps", _vector3(self.velocity_eci_mps, "velocity_eci_mps"))
        object.__setattr__(
            self,
            "velocity_without_gravity_eci_mps",
            _vector3(self.velocity_without_gravity_eci_mps, "velocity_without_gravity_eci_mps"),
        )
        if self.orientation_eci_from_body is not None:
            object.__setattr__(self, "orientation_eci_from_body", _rotation(self.orientation_eci_from_body, "orientation_eci_from_body"))
        object.__setattr__(self, "gravity_eci_mps2", _vector3(self.gravity_eci_mps2, "gravity_eci_mps2"))
        if self.angular_rate_body_radps is not None:
            object.__setattr__(self, "angular_rate_body_radps", _vector3(self.angular_rate_body_radps, "angular_rate_body_radps"))
        if self.acceleration_eci_mps2 is not None:
            object.__setattr__(self, "acceleration_eci_mps2", _vector3(self.acceleration_eci_mps2, "acceleration_eci_mps2"))
        if self.angular_acceleration_body_radps2 is not None:
            object.__setattr__(
                self,
                "angular_acceleration_body_radps2",
                _vector3(self.angular_acceleration_body_radps2, "angular_acceleration_body_radps2"),
            )
        if self.temperature_celsius is not None and not np.isfinite(self.temperature_celsius):
            raise ValueError("temperature_celsius must be finite when supplied")


@dataclass(frozen=True, slots=True)
class TruthSegment:
    """Accepted truth over one interval between committed boundaries."""

    start: TruthPoint
    end: TruthPoint

    def __post_init__(self) -> None:
        if self.end.time_s <= self.start.time_s:
            raise ValueError("truth segment end must be later than its start")


@dataclass(frozen=True, slots=True)
class MeasurementPacket(Generic[MeasurementT]):
    """Timestamped measurement with explicit delivery and validity state."""

    sampled_at_s: float
    available_at_s: float
    interval_start_s: float | None
    payload: MeasurementT | None
    valid: bool = True

    def __post_init__(self) -> None:
        if not np.isfinite(self.sampled_at_s) or not np.isfinite(self.available_at_s):
            raise ValueError("measurement timestamps must be finite")
        if self.available_at_s < self.sampled_at_s:
            raise ValueError("measurement cannot be available before it is sampled")
        if self.interval_start_s is not None and self.interval_start_s > self.sampled_at_s:
            raise ValueError("measurement interval start cannot be after sample time")
        if self.valid and self.payload is None:
            raise ValueError("valid measurements require a payload")


@dataclass(frozen=True, slots=True)
class ImuIncrement:
    """Body-frame IMU increments over one accepted truth interval."""

    delta_v_body_mps: Array3
    delta_theta_body_rad: Array3
    start_time_s: float
    end_time_s: float
    temperature_celsius: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "delta_v_body_mps", _vector3(self.delta_v_body_mps, "delta_v_body_mps"))
        object.__setattr__(self, "delta_theta_body_rad", _vector3(self.delta_theta_body_rad, "delta_theta_body_rad"))
        if not np.isfinite(self.start_time_s) or not np.isfinite(self.end_time_s) or self.end_time_s <= self.start_time_s:
            raise ValueError("IMU increment requires a positive finite interval")
        if self.temperature_celsius is not None and not np.isfinite(self.temperature_celsius):
            raise ValueError("temperature_celsius must be finite when supplied")

    @property
    def dt_s(self) -> float:
        return self.end_time_s - self.start_time_s


@dataclass(frozen=True, slots=True)
class GyroIncrement:
    """Body-frame angular increment over one accepted truth interval."""

    delta_theta_body_rad: Array3
    start_time_s: float
    end_time_s: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "delta_theta_body_rad", _vector3(self.delta_theta_body_rad, "delta_theta_body_rad"))
        if not np.isfinite(self.start_time_s) or not np.isfinite(self.end_time_s) or self.end_time_s <= self.start_time_s:
            raise ValueError("gyro increment requires a positive finite interval")

    @property
    def dt_s(self) -> float:
        return self.end_time_s - self.start_time_s


@dataclass(frozen=True, slots=True)
class AccelerationIncrement:
    """ECI specific-force increment for a translation-only scenario."""

    delta_v_eci_mps: Array3
    start_time_s: float
    end_time_s: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "delta_v_eci_mps", _vector3(self.delta_v_eci_mps, "delta_v_eci_mps"))
        if not np.isfinite(self.start_time_s) or not np.isfinite(self.end_time_s) or self.end_time_s <= self.start_time_s:
            raise ValueError("acceleration increment requires a positive finite interval")

    @property
    def dt_s(self) -> float:
        return self.end_time_s - self.start_time_s


class ImuModelProtocol(Protocol):
    """Structural interface implemented by ``imu-error-model`` and fakes."""

    def reset(self) -> None:
        ...

    def measure(
        self,
        timestamp: float,
        velocity_without_gravity: Array3,
        orientation_eci_from_body: Array3,
        temperature_celsius: float | None = None,
    ) -> object:
        ...


class ImuErrorModelAdapter:
    """Adapt ``imu-error-model`` to Taoryx committed-truth semantics."""

    def __init__(
        self,
        model: ImuModelProtocol,
        *,
        body_from_sensor: Array3 | None = None,
        lever_arm_body_m: Array3 | None = None,
        delivery_delay_s: float = 0.0,
        accelerometer_output_scale: float = 1.0,
        gyroscope_output_scale: float = 1.0,
        provenance: Mapping[str, object] | None = None,
    ) -> None:
        self.model = model
        self.body_from_sensor = _rotation(
            np.eye(3) if body_from_sensor is None else body_from_sensor,
            "body_from_sensor",
        )
        self.lever_arm_body_m = _vector3(
            np.zeros(3) if lever_arm_body_m is None else lever_arm_body_m,
            "lever_arm_body_m",
        )
        if not np.isfinite(delivery_delay_s) or delivery_delay_s < 0.0:
            raise ValueError("delivery_delay_s must be finite and nonnegative")
        self.delivery_delay_s = float(delivery_delay_s)
        if not np.isfinite(accelerometer_output_scale) or accelerometer_output_scale <= 0.0:
            raise ValueError("accelerometer_output_scale must be finite and positive")
        if not np.isfinite(gyroscope_output_scale) or gyroscope_output_scale <= 0.0:
            raise ValueError("gyroscope_output_scale must be finite and positive")
        self.accelerometer_output_scale = float(accelerometer_output_scale)
        self.gyroscope_output_scale = float(gyroscope_output_scale)
        self.provenance = {
            **dict(provenance or {}),
            "accelerometer_output_scale": self.accelerometer_output_scale,
            "gyroscope_output_scale": self.gyroscope_output_scale,
            "velocity_without_gravity_source": "truth-field-or-accepted-acceleration-minus-gravity",
        }
        self._last_truth_time_s: float | None = None
        self._last_velocity_without_gravity_eci_mps: Array3 | None = None
        self._last_specific_acceleration_eci_mps2: Array3 | None = None

    @classmethod
    def from_config(
        cls,
        config: object | None = None,
        *,
        seed: int | None = None,
        body_from_sensor: Array3 | None = None,
        lever_arm_body_m: Array3 | None = None,
        delivery_delay_s: float = 0.0,
        provenance: Mapping[str, object] | None = None,
    ) -> ImuErrorModelAdapter:
        """Construct the adapter using the optional PyPI dependency."""

        try:
            from imu_error_model import ImuModel
        except ImportError as exc:  # pragma: no cover - depends on installation extras
            raise ImportError("install the `sensors` extra to use imu-error-model") from exc
        model = ImuModel(config=config, rng=np.random.default_rng(seed))
        accelerometer_config = getattr(config, "accelerometer", None)
        gyroscope_config = getattr(config, "gyroscope", None)
        accelerometer_scale = float(getattr(config, "output_scale_accelerometer", 1.0)) * float(getattr(accelerometer_config, "output_scale", 1.0))
        gyroscope_scale = float(getattr(config, "output_scale_gyroscope", 1.0)) * float(getattr(gyroscope_config, "output_scale", 1.0))
        return cls(
            model,
            body_from_sensor=body_from_sensor,
            lever_arm_body_m=lever_arm_body_m,
            delivery_delay_s=delivery_delay_s,
            accelerometer_output_scale=accelerometer_scale,
            gyroscope_output_scale=gyroscope_scale,
            provenance=provenance,
        )

    @classmethod
    def from_profile(
        cls,
        path: str | Path,
        *,
        seed: int | None = None,
        body_from_sensor: Array3 | None = None,
        lever_arm_body_m: Array3 | None = None,
        delivery_delay_s: float = 0.0,
    ) -> ImuErrorModelAdapter:
        """Load a repository profile and preserve its declared provenance."""

        try:
            from imu_error_model import load_profile_document
        except ImportError as exc:  # pragma: no cover - depends on installation extras
            raise ImportError("install the `sensors` extra to use imu-error-model") from exc
        document = load_profile_document(path)
        return cls._from_loaded_profile(
            document,
            seed=seed,
            body_from_sensor=body_from_sensor,
            lever_arm_body_m=lever_arm_body_m,
            delivery_delay_s=delivery_delay_s,
        )

    @classmethod
    def from_example_profile(
        cls,
        name: str,
        *,
        category: str = "hardware_estimates",
        seed: int | None = None,
        body_from_sensor: Array3 | None = None,
        lever_arm_body_m: Array3 | None = None,
        delivery_delay_s: float = 0.0,
    ) -> ImuErrorModelAdapter:
        """Load a profile shipped inside ``imu-error-model`` 0.1.3 or newer."""

        try:
            from imu_error_model import load_example_profile
        except ImportError as exc:  # pragma: no cover - depends on installation extras
            raise ImportError("install the `sensors` extra to use imu-error-model") from exc
        document = load_example_profile(name, category=category)
        return cls._from_loaded_profile(
            document,
            seed=seed,
            body_from_sensor=body_from_sensor,
            lever_arm_body_m=lever_arm_body_m,
            delivery_delay_s=delivery_delay_s,
        )

    @classmethod
    def _from_loaded_profile(
        cls,
        document: Any,
        *,
        seed: int | None,
        body_from_sensor: Array3 | None,
        lever_arm_body_m: Array3 | None,
        delivery_delay_s: float,
    ) -> ImuErrorModelAdapter:
        """Construct an adapter from either a filesystem or packaged profile."""

        provenance = {
            "profile_path": str(document.source_path),
            "model_name": document.model_name,
            "sample_period_s": document.sample_period_s,
            "metadata": document.metadata.model_dump(mode="json"),
        }
        return cls.from_config(
            document.config,
            seed=seed,
            body_from_sensor=body_from_sensor,
            lever_arm_body_m=lever_arm_body_m,
            delivery_delay_s=delivery_delay_s,
            provenance=provenance,
        )

    def reset(self) -> None:
        self.model.reset()
        self._last_truth_time_s = None
        self._last_velocity_without_gravity_eci_mps = None
        self._last_specific_acceleration_eci_mps2 = None

    def snapshot(self) -> dict[str, object]:
        """Return a JSON-compatible adapter and upstream-model checkpoint."""

        snapshot = getattr(self.model, "snapshot", None)
        if not callable(snapshot):
            raise TypeError("the configured IMU model does not support checkpointing")
        model_checkpoint = snapshot()
        return {
            "schema_version": 1,
            "adapter_type": "taoryx.ImuErrorModelAdapter",
            "model_checkpoint": model_checkpoint.model_dump(mode="json"),
            "last_truth_time_s": self._last_truth_time_s,
            "last_velocity_without_gravity_eci_mps": None
            if self._last_velocity_without_gravity_eci_mps is None
            else self._last_velocity_without_gravity_eci_mps.tolist(),
            "last_specific_acceleration_eci_mps2": None
            if self._last_specific_acceleration_eci_mps2 is None
            else self._last_specific_acceleration_eci_mps2.tolist(),
        }

    def restore(self, checkpoint: Mapping[str, object]) -> None:
        """Restore a checkpoint produced by :meth:`snapshot`."""

        if int(cast(Any, checkpoint.get("schema_version", 0))) != 1:
            raise ValueError("unsupported Taoryx IMU adapter checkpoint schema")
        model_checkpoint = checkpoint.get("model_checkpoint")
        if not isinstance(model_checkpoint, Mapping):
            raise ValueError("IMU adapter checkpoint is missing model_checkpoint")
        snapshot = getattr(self.model, "snapshot", None)
        restore = getattr(self.model, "restore", None)
        if not callable(snapshot) or not callable(restore):
            raise TypeError("the configured IMU model does not support checkpointing")
        current = snapshot()
        restore(type(current).model_validate(model_checkpoint))
        time_value = checkpoint.get("last_truth_time_s")
        self._last_truth_time_s = None if time_value is None else float(cast(Any, time_value))
        self._last_velocity_without_gravity_eci_mps = self._restore_vector(
            checkpoint.get("last_velocity_without_gravity_eci_mps"),
        )
        self._last_specific_acceleration_eci_mps2 = self._restore_vector(
            checkpoint.get("last_specific_acceleration_eci_mps2"),
        )

    @staticmethod
    def _restore_vector(value: object) -> Array3 | None:
        if value is None:
            return None
        return _vector3(np.asarray(value, dtype=float), "checkpoint vector")

    def sample(self, truth: TruthPoint) -> MeasurementPacket[ImuIncrement]:
        """Sample one committed endpoint; the first call is explicitly invalid."""

        if truth.orientation_eci_from_body is None or truth.angular_rate_body_radps is None:
            raise ValueError("IMU sampling requires committed orientation and body-rate truth")
        previous_time = self._last_truth_time_s
        if previous_time is not None and truth.time_s <= previous_time:
            raise ValueError("IMU truth samples must be strictly chronological")
        velocity_without_gravity = self._resolve_velocity_without_gravity(truth, previous_time)
        output = self.model.measure(
            truth.time_s,
            velocity_without_gravity,
            truth.orientation_eci_from_body @ self.body_from_sensor,
            truth.temperature_celsius,
        )
        self._last_truth_time_s = truth.time_s
        start_time = float(getattr(output, "start_time"))
        end_time = float(getattr(output, "end_time"))
        is_initial = previous_time is None or end_time <= start_time
        payload = None
        if not is_initial:
            payload = ImuIncrement(
                self.body_from_sensor @ (np.asarray(getattr(output, "delta_v"), dtype=float) / self.accelerometer_output_scale),
                self.body_from_sensor @ (np.asarray(getattr(output, "delta_theta"), dtype=float) / self.gyroscope_output_scale),
                start_time,
                end_time,
                getattr(output, "temperature_celsius", truth.temperature_celsius),
            )
        return MeasurementPacket(
            sampled_at_s=truth.time_s,
            available_at_s=truth.time_s + self.delivery_delay_s,
            interval_start_s=None if is_initial else start_time,
            payload=payload,
            valid=not is_initial,
        )

    def sample_segment(self, segment: TruthSegment) -> MeasurementPacket[ImuIncrement]:
        """Consume an accepted segment without interpolating its endpoints."""

        if self._last_truth_time_s is None:
            self.sample(segment.start)
        elif not np.isclose(self._last_truth_time_s, segment.start.time_s, atol=1.0e-12):
            raise ValueError("truth segment does not continue the IMU adapter history")
        return self.sample(segment.end)

    def _sensor_velocity_without_gravity(self, truth: TruthPoint) -> Array3:
        angular_rate = np.zeros(3) if truth.angular_rate_body_radps is None else truth.angular_rate_body_radps
        lever_velocity_body = np.cross(angular_rate, self.lever_arm_body_m)
        return truth.velocity_without_gravity_eci_mps + truth.orientation_eci_from_body @ lever_velocity_body

    def _resolve_velocity_without_gravity(self, truth: TruthPoint, previous_time: float | None) -> Array3:
        """Build the package input from accepted specific acceleration when available.

        ``imu-error-model`` expects a velocity whose derivative is specific
        force, not the inertial velocity whose derivative includes gravity.
        Keeping this conversion at the adapter boundary makes hovering and
        powered flight observable without asking the external model to know
        Taoryx force conventions.
        """

        velocity = self._sensor_velocity_without_gravity(truth)
        specific_acceleration = None
        if truth.acceleration_eci_mps2 is not None:
            specific_acceleration = truth.acceleration_eci_mps2 - truth.gravity_eci_mps2
        if (
            previous_time is not None
            and self._last_velocity_without_gravity_eci_mps is not None
            and self._last_specific_acceleration_eci_mps2 is not None
            and specific_acceleration is not None
        ):
            dt = truth.time_s - previous_time
            velocity = self._last_velocity_without_gravity_eci_mps + 0.5 * (
                self._last_specific_acceleration_eci_mps2 + specific_acceleration
            ) * dt
        self._last_velocity_without_gravity_eci_mps = np.asarray(velocity, dtype=float).copy()
        self._last_specific_acceleration_eci_mps2 = None if specific_acceleration is None else specific_acceleration.copy()
        return velocity


class IdealImuAdapter:
    """Perfect-information IMU adapter for explicitly requested comparisons."""

    def __init__(
        self,
        *,
        body_from_sensor: Array3 | None = None,
        lever_arm_body_m: Array3 | None = None,
    ) -> None:
        self.body_from_sensor = _rotation(np.eye(3) if body_from_sensor is None else body_from_sensor, "body_from_sensor")
        self.lever_arm_body_m = _vector3(np.zeros(3) if lever_arm_body_m is None else lever_arm_body_m, "lever_arm_body_m")
        self.provenance = {
            "provider": "ideal",
            "authority": "Taoryx accepted truth",
            "accelerometer_output_scale": 1.0,
            "gyroscope_output_scale": 1.0,
        }
        self._last_truth: TruthPoint | None = None

    def reset(self) -> None:
        self._last_truth = None

    def sample(self, truth: TruthPoint) -> MeasurementPacket[ImuIncrement]:
        previous = self._last_truth
        if previous is not None and truth.time_s <= previous.time_s:
            raise ValueError("ideal IMU truth samples must be strictly chronological")
        if truth.orientation_eci_from_body is None or truth.angular_rate_body_radps is None:
            raise ValueError("ideal IMU sampling requires committed orientation and body-rate truth")
        self._last_truth = truth
        if previous is None:
            return MeasurementPacket(truth.time_s, truth.time_s, None, None, valid=False)
        dt = truth.time_s - previous.time_s
        specific_acceleration = self._specific_acceleration(truth, previous, dt)
        body_specific = truth.orientation_eci_from_body.T @ specific_acceleration
        angular_rate = truth.angular_rate_body_radps
        angular_acceleration = (
            np.zeros(3)
            if truth.angular_acceleration_body_radps2 is None
            else truth.angular_acceleration_body_radps2
        )
        lever_acceleration = np.cross(angular_acceleration, self.lever_arm_body_m) + np.cross(
            angular_rate,
            np.cross(angular_rate, self.lever_arm_body_m),
        )
        sensor_specific = self.body_from_sensor.T @ (body_specific + lever_acceleration)
        sensor_rate = self.body_from_sensor.T @ angular_rate
        increment = ImuIncrement(
            self.body_from_sensor @ sensor_specific * dt,
            self.body_from_sensor @ sensor_rate * dt,
            previous.time_s,
            truth.time_s,
            truth.temperature_celsius,
        )
        return MeasurementPacket(truth.time_s, truth.time_s, previous.time_s, increment, valid=True)

    @staticmethod
    def _specific_acceleration(truth: TruthPoint, previous: TruthPoint, dt: float) -> Array3:
        if truth.acceleration_eci_mps2 is not None:
            return truth.acceleration_eci_mps2 - truth.gravity_eci_mps2
        return (truth.velocity_without_gravity_eci_mps - previous.velocity_without_gravity_eci_mps) / dt


class IdealGyroscopeAdapter:
    """Perfect gyro-only adapter for rotational SWIL/HWIL comparisons."""

    def __init__(self, *, delivery_delay_s: float = 0.0) -> None:
        if not np.isfinite(delivery_delay_s) or delivery_delay_s < 0.0:
            raise ValueError("delivery delay must be finite and nonnegative")
        self.delivery_delay_s = float(delivery_delay_s)
        self.provenance = {
            "provider": "ideal-gyroscope",
            "authority": "Taoryx rotational truth",
            "channels": ["angular_rate"],
            "frame": "body",
            "translation_usage": "not-consumed",
        }
        self._last_truth: TruthPoint | None = None

    def reset(self) -> None:
        self._last_truth = None

    def sample(self, truth: TruthPoint) -> MeasurementPacket[GyroIncrement]:
        previous = self._last_truth
        if previous is not None and truth.time_s <= previous.time_s:
            raise ValueError("gyro truth samples must be strictly chronological")
        if truth.angular_rate_body_radps is None:
            raise ValueError("gyro sampling requires committed body-rate truth")
        self._last_truth = truth
        if previous is None:
            return MeasurementPacket(truth.time_s, truth.time_s + self.delivery_delay_s, None, None, valid=False)
        if previous.angular_rate_body_radps is None:
            raise ValueError("gyro sampling requires body-rate truth at every interval endpoint")
        dt = truth.time_s - previous.time_s
        increment = GyroIncrement(
            0.5 * (previous.angular_rate_body_radps + truth.angular_rate_body_radps) * dt,
            previous.time_s,
            truth.time_s,
        )
        return MeasurementPacket(
            truth.time_s,
            truth.time_s + self.delivery_delay_s,
            previous.time_s,
            increment,
            valid=True,
        )

    def sample_segment(self, segment: TruthSegment) -> MeasurementPacket[GyroIncrement]:
        """Consume an accepted segment without interpolating its endpoints."""

        if self._last_truth is None:
            self.sample(segment.start)
        elif not np.isclose(self._last_truth.time_s, segment.start.time_s, atol=1.0e-12):
            raise ValueError("truth segment does not continue the gyro adapter history")
        return self.sample(segment.end)


class TranslationAccelerationAdapter:
    """Emit acceleration-only packets without inventing an attitude channel."""

    def __init__(self, *, delivery_delay_s: float = 0.0) -> None:
        if not np.isfinite(delivery_delay_s) or delivery_delay_s < 0.0:
            raise ValueError("delivery delay must be finite and nonnegative")
        self.delivery_delay_s = float(delivery_delay_s)
        self.provenance = {
            "provider": "translation-acceleration",
            "authority": "Taoryx accepted truth",
            "channels": ["specific_force"],
            "frame": "ECI",
            "startup_policy": "zero-order-hold-current-specific-force",
        }
        self._last_truth: TruthPoint | None = None

    def reset(self) -> None:
        self._last_truth = None

    def sample(self, truth: TruthPoint) -> MeasurementPacket[AccelerationIncrement]:
        previous = self._last_truth
        if previous is not None and truth.time_s <= previous.time_s:
            raise ValueError("translation acceleration truth samples must be strictly chronological")
        self._last_truth = truth
        if previous is None:
            return MeasurementPacket(truth.time_s, truth.time_s + self.delivery_delay_s, None, None, valid=False)
        if truth.acceleration_eci_mps2 is None:
            raise ValueError("translation acceleration sampling requires committed acceleration truth")
        dt = truth.time_s - previous.time_s
        current_specific = truth.acceleration_eci_mps2 - truth.gravity_eci_mps2
        previous_specific = current_specific if previous.acceleration_eci_mps2 is None else previous.acceleration_eci_mps2 - previous.gravity_eci_mps2
        increment = AccelerationIncrement(
            0.5 * (previous_specific + current_specific) * dt,
            previous.time_s,
            truth.time_s,
        )
        return MeasurementPacket(
            truth.time_s,
            truth.time_s + self.delivery_delay_s,
            previous.time_s,
            increment,
            valid=True,
        )
