"""Accepted runtime-state projections for sensor truth providers."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, replace
from typing import ClassVar, Protocol

import numpy as np

from taoryx.rigid_body import RigidBody6DofModel, RigidBody6DofState
from taoryx.sensors import TruthPoint

from .common import RuntimeState


def _proper_rotation(value: np.ndarray, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3x3 rotation matrix")
    if not np.allclose(result.T @ result, np.eye(3), atol=1.0e-8) or not np.isclose(np.linalg.det(result), 1.0, atol=1.0e-8):
        raise ValueError(f"{name} must be a proper rotation matrix")
    return result.copy()


def _finite_vector(value: np.ndarray, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3-vector")
    return result.copy()


@dataclass(frozen=True, slots=True)
class RotationTruth:
    """External rotational truth, independent of translational authority."""

    time_s: float
    orientation_eci_from_body: np.ndarray
    angular_rate_body_radps: np.ndarray
    angular_acceleration_body_radps2: np.ndarray | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.time_s):
            raise ValueError("rotation truth time must be finite")
        object.__setattr__(self, "orientation_eci_from_body", _proper_rotation(self.orientation_eci_from_body, "orientation_eci_from_body"))
        object.__setattr__(self, "angular_rate_body_radps", _finite_vector(self.angular_rate_body_radps, "angular_rate_body_radps"))
        if self.angular_acceleration_body_radps2 is not None:
            object.__setattr__(self, "angular_acceleration_body_radps2", _finite_vector(self.angular_acceleration_body_radps2, "angular_acceleration_body_radps2"))


RotationTruthProvider = Callable[[RuntimeState], RotationTruth]


def rigid_body_truth_provider(model: RigidBody6DofModel) -> Callable[[RuntimeState], TruthPoint]:
    """Project an accepted ECIC rigid-body state into the ECI sensor contract."""

    def provider(runtime_state: RuntimeState) -> TruthPoint:
        state = RigidBody6DofState.from_values(runtime_state.time, runtime_state.values)
        observables = model.observables(state)
        attitude = state.attitude.normalized()
        w, x, y, z = attitude.w, attitude.x, attitude.y, attitude.z
        orientation_eci_from_body = np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
                [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
                [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
            ]
        )
        gravity = model.gravity(state)
        acceleration = np.array(
            [
                observables["acceleration_ecic_x_m_s2"],
                observables["acceleration_ecic_y_m_s2"],
                observables["acceleration_ecic_z_m_s2"],
            ]
        )
        angular_acceleration = np.array(
            [
                observables["angular_acceleration_body_x_rad_s2"],
                observables["angular_acceleration_body_y_rad_s2"],
                observables["angular_acceleration_body_z_rad_s2"],
            ]
        )
        return TruthPoint(
            time_s=state.time,
            position_eci_m=np.array([state.position.vector.x, state.position.vector.y, state.position.vector.z]),
            velocity_eci_mps=np.array([state.velocity.vector.x, state.velocity.vector.y, state.velocity.vector.z]),
            velocity_without_gravity_eci_mps=np.array([state.velocity.vector.x, state.velocity.vector.y, state.velocity.vector.z]),
            orientation_eci_from_body=orientation_eci_from_body,
            gravity_eci_mps2=np.array([gravity.x, gravity.y, gravity.z]),
            angular_rate_body_radps=np.array([state.body_rate.x, state.body_rate.y, state.body_rate.z]),
            acceleration_eci_mps2=acceleration,
            angular_acceleration_body_radps2=angular_acceleration,
        )

    contract = {
        "mode": "rigid-body-6dof",
        "translation": {"available": True, "source": "accepted-rigid-body-state"},
        "orientation": {"available": True, "source": "accepted-rigid-body-state"},
        "angular_rate": {"available": True, "source": "accepted-rigid-body-state"},
        "acceleration": {"available": True, "source": "accepted-rigid-body-observables"},
    }
    setattr(provider, "contract", contract)
    return provider


def _state_named(state: RuntimeState) -> dict[str, float]:
    values = dict(state.named)
    values.update({name: float(value) for name, value in zip(state.value_names, state.values, strict=False)})
    return values


def _rotate_z(vector: np.ndarray, angle_rad: float) -> np.ndarray:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.array(
        [cosine * vector[0] - sine * vector[1], sine * vector[0] + cosine * vector[1], vector[2]],
        dtype=float,
    )


def _translation_eci(
    state: RuntimeState,
    *,
    earth_rotation_rate_rad_s: float,
    length_scale_to_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    values = _state_named(state)
    required = ("x", "y", "z", "xdt", "ydt", "zdt")
    if not all(name in values for name in required):
        raise ValueError("translation sensor truth requires x, y, z, xdt, ydt, and zdt state channels")
    position = np.array([values[name] for name in required[:3]], dtype=float)
    velocity = np.array([values[name] for name in required[3:]], dtype=float)
    frame = getattr(state.frame, "value", str(state.frame)).casefold()
    if frame in {"ecic", "frame.ecic"}:
        return position * length_scale_to_m, velocity * length_scale_to_m
    angle = earth_rotation_rate_rad_s * state.time
    position_eci_internal = _rotate_z(position, angle)
    omega_cross_position = np.cross(np.array([0.0, 0.0, earth_rotation_rate_rad_s]), position_eci_internal)
    velocity_eci_internal = _rotate_z(velocity, angle) + omega_cross_position
    return position_eci_internal * length_scale_to_m, velocity_eci_internal * length_scale_to_m


class TranslationTruthProvider:
    """Project accepted point-mass translation into the ECI truth contract."""

    def __init__(self, gravitational_parameter: float, earth_rotation_rate_rad_s: float, *, length_scale_to_m: float = 1.0) -> None:
        if not math.isfinite(gravitational_parameter) or gravitational_parameter < 0.0:
            raise ValueError("point-mass gravitational parameter must be finite and nonnegative")
        if not math.isfinite(earth_rotation_rate_rad_s):
            raise ValueError("Earth rotation rate must be finite")
        if not math.isfinite(length_scale_to_m) or length_scale_to_m <= 0.0:
            raise ValueError("point-mass length scale must be finite and positive")
        self.gravitational_parameter = gravitational_parameter
        self.earth_rotation_rate_rad_s = earth_rotation_rate_rad_s
        self.length_scale_to_m = length_scale_to_m
        self._last_time_s: float | None = None
        self._last_velocity_eci_mps: np.ndarray | None = None
        self._last_acceleration_eci_mps2: np.ndarray | None = None
        self.contract = {
            "mode": "translation-only",
            "frame": "ECI",
            "translation": {"available": True, "source": "accepted-translation-state"},
            "orientation": {"available": False, "source": None},
            "angular_rate": {"available": False, "source": None},
            "acceleration": {"available": True, "source": "accepted-translation-velocity-difference"},
            "gravity": "central-point-mass",
        }

    def __call__(self, state: RuntimeState) -> TruthPoint:
        position, velocity = _translation_eci(
            state,
            earth_rotation_rate_rad_s=self.earth_rotation_rate_rad_s,
            length_scale_to_m=self.length_scale_to_m,
        )
        acceleration: np.ndarray | None = None
        if self._last_time_s is None:
            pass
        elif state.time < self._last_time_s:
            self._last_time_s = None
            self._last_velocity_eci_mps = None
            self._last_acceleration_eci_mps2 = None
        elif state.time > self._last_time_s and self._last_velocity_eci_mps is not None:
            acceleration = (velocity - self._last_velocity_eci_mps) / (state.time - self._last_time_s)
        elif self._last_acceleration_eci_mps2 is not None:
            acceleration = self._last_acceleration_eci_mps2.copy()
        self._last_time_s = state.time
        self._last_velocity_eci_mps = velocity.copy()
        self._last_acceleration_eci_mps2 = None if acceleration is None else acceleration.copy()
        gravitational_parameter_si = self.gravitational_parameter * self.length_scale_to_m**3
        radius = float(np.linalg.norm(position))
        gravity = np.zeros(3)
        if gravitational_parameter_si > 0.0 and radius > 1.0:
            gravity = -gravitational_parameter_si * position / radius**3
        return TruthPoint(
            time_s=state.time,
            position_eci_m=position,
            velocity_eci_mps=velocity,
            velocity_without_gravity_eci_mps=velocity,
            orientation_eci_from_body=None,
            gravity_eci_mps2=gravity,
            angular_rate_body_radps=None,
            acceleration_eci_mps2=acceleration,
        )


def _quaternion_matrix(values: Mapping[str, float]) -> np.ndarray:
    w, x, y, z = (float(values.get(name, default)) for name, default in (("qw", 1.0), ("qx", 0.0), ("qy", 0.0), ("qz", 0.0)))
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError("kinematic truth quaternion must have a finite nonzero norm")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1.0e-8:
        return np.array([(rotation[2, 1] - rotation[1, 2]) * 0.5, (rotation[0, 2] - rotation[2, 0]) * 0.5, (rotation[1, 0] - rotation[0, 1]) * 0.5])
    scale = angle / (2.0 * math.sin(angle))
    return scale * np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]])


class KinematicTruthProvider:
    """Add the accepted kinematic quaternion to a translation projection."""

    def __init__(self, base: Callable[[RuntimeState], TruthPoint]) -> None:
        self.base = base
        self._last_time_s: float | None = None
        self._last_orientation: np.ndarray | None = None
        self.contract = {
            **dict(getattr(base, "contract", {})),
            "mode": "kinematic-6dof",
            "orientation": {"available": True, "source": "accepted-kinematic-quaternion"},
            "angular_rate": {"available": True, "source": "accepted-quaternion-difference"},
        }

    def __call__(self, state: RuntimeState) -> TruthPoint:
        truth = self.base(state)
        orientation = _quaternion_matrix(_state_named(state))
        rate = np.zeros(3)
        if self._last_time_s is not None and self._last_orientation is not None and state.time > self._last_time_s:
            rate = _rotation_vector(self._last_orientation.T @ orientation) / (state.time - self._last_time_s)
        self._last_time_s = state.time
        self._last_orientation = orientation.copy()
        return replace(truth, orientation_eci_from_body=orientation, angular_rate_body_radps=rate)


class CompositeTruthProvider:
    """Combine independent translational and rotational truth authorities."""

    def __init__(
        self,
        translation_provider: Callable[[RuntimeState], TruthPoint],
        rotation_provider: RotationTruthProvider,
        *,
        rotation_source: str = "external-rotational-provider",
    ) -> None:
        if not rotation_source.strip():
            raise ValueError("rotation source must not be empty")
        self.translation_provider = translation_provider
        self.rotation_provider = rotation_provider
        self.rotation_source = rotation_source
        self.contract = {
            **dict(getattr(translation_provider, "contract", {})),
            "mode": "hybrid-6dof",
            "translation": {
                **dict(getattr(translation_provider, "contract", {}).get("translation", {})),
                "available": True,
                "substitution_allowed": True,
            },
            "orientation": {"available": True, "source": rotation_source},
            "angular_rate": {"available": True, "source": rotation_source},
            "channel_sources": {
                "translation": "simulation-or-substituted-eci-truth",
                "rotation": rotation_source,
            },
        }

    def __call__(self, state: RuntimeState) -> TruthPoint:
        translation = self.translation_provider(state)
        rotation = self.rotation_provider(state)
        if not np.isclose(translation.time_s, rotation.time_s, atol=1.0e-10):
            raise ValueError("translation and rotational truth must share an accepted timestamp")
        return replace(
            translation,
            orientation_eci_from_body=rotation.orientation_eci_from_body,
            angular_rate_body_radps=rotation.angular_rate_body_radps,
            angular_acceleration_body_radps2=rotation.angular_acceleration_body_radps2,
        )


@dataclass(frozen=True, slots=True)
class Pseudo6DofPolicy:
    """Deterministic attitude policy for a translation-only vehicle."""

    alignment: str = "velocity"
    speed_threshold_mps: float = 1.0
    bank_source: str = "controller"
    bank_default_rad: float = 0.0
    yaw_source: str = "controller-then-heading"
    zero_speed_policy: str = "hold-then-initial-frame"
    pole_crossing_policy: str = "parallel-transport"
    policy_id: ClassVar[str] = "velocity-aligned"

    def __post_init__(self) -> None:
        if self.alignment != "velocity":
            raise ValueError("pseudo-6dof alignment must be velocity")
        if not math.isfinite(self.speed_threshold_mps) or self.speed_threshold_mps <= 0.0:
            raise ValueError("pseudo-6dof speed threshold must be positive and finite")
        if self.bank_source not in {"controller", "zero"}:
            raise ValueError("pseudo-6dof bank source must be controller or zero")
        if self.yaw_source not in {"controller-then-heading", "heading", "hold"}:
            raise ValueError("pseudo-6dof yaw source is invalid")
        if self.zero_speed_policy not in {"hold-then-initial-frame", "nadir-frame"}:
            raise ValueError("pseudo-6dof zero-speed policy is invalid")
        if self.pole_crossing_policy != "parallel-transport":
            raise ValueError("pseudo-6dof pole-crossing policy must be parallel-transport")

    def project(
        self,
        truth: TruthPoint,
        state: RuntimeState,
        previous_orientation: np.ndarray | None,
    ) -> np.ndarray:
        return _project_pseudo_attitude(truth, state, self, previous_orientation)


class AttitudeProjectionPolicy(Protocol):
    """Interface for policies that project accepted truth into an attitude."""

    def project(
        self,
        truth: TruthPoint,
        state: RuntimeState,
        previous_orientation: np.ndarray | None,
    ) -> np.ndarray:
        ...


def _config_float(config: Mapping[str, object], name: str, default: float) -> float:
    value = config.get(name, default)
    if not isinstance(value, (int, float, str)):
        raise ValueError(f"attitude policy field {name!r} must be numeric")
    return float(value)


def _angle_from_state(values: Mapping[str, float], keys: tuple[str, ...], default: float) -> float:
    for key in keys:
        if key in values:
            value = float(values[key])
            return value if key.endswith("_rad") else math.radians(value)
    return default


def _local_heading_forward(position: np.ndarray, heading_rad: float) -> np.ndarray:
    radial = position / max(float(np.linalg.norm(position)), 1.0e-12)
    reference = np.array([0.0, 0.0, 1.0])
    north = reference - np.dot(reference, radial) * radial
    if np.linalg.norm(north) < 1.0e-8:
        reference = np.array([1.0, 0.0, 0.0])
        north = reference - np.dot(reference, radial) * radial
    north /= np.linalg.norm(north)
    east = np.cross(north, radial)
    return north * math.cos(heading_rad) + east * math.sin(heading_rad)


def _project_pseudo_attitude(
    truth: TruthPoint,
    state: RuntimeState,
    policy: Pseudo6DofPolicy,
    previous_orientation: np.ndarray | None,
    *,
    forward_override: np.ndarray | None = None,
) -> np.ndarray:
    values = _state_named(state)
    speed = float(np.linalg.norm(truth.velocity_eci_mps))
    if speed < policy.speed_threshold_mps and previous_orientation is not None and policy.zero_speed_policy == "hold-then-initial-frame":
        return previous_orientation.copy()

    forward = None if forward_override is None else _finite_vector(forward_override, "attitude forward axis")
    if forward is None:
        forward = truth.velocity_eci_mps / max(speed, 1.0e-12) if speed >= policy.speed_threshold_mps else None
    if forward is None:
        heading = 0.0
        if policy.yaw_source != "hold":
            heading = _angle_from_state(
                values,
                ("command.yaw_rad", "yaw_rad", "yawi", "yawgd", "psi") if policy.yaw_source == "controller-then-heading" else ("psi",),
                0.0,
            )
        forward = _local_heading_forward(truth.position_eci_m, heading)
    forward /= max(float(np.linalg.norm(forward)), 1.0e-12)
    down = -truth.position_eci_m / max(float(np.linalg.norm(truth.position_eci_m)), 1.0e-12)
    z_axis = down - float(np.dot(down, forward)) * forward
    if np.linalg.norm(z_axis) < 1.0e-8 and previous_orientation is not None and policy.pole_crossing_policy == "parallel-transport":
        prior_z = previous_orientation[:, 2]
        z_axis = prior_z - float(np.dot(prior_z, forward)) * forward
    if np.linalg.norm(z_axis) < 1.0e-8:
        reference = np.array([1.0, 0.0, 0.0]) if abs(forward[0]) < 0.8 else np.array([0.0, 1.0, 0.0])
        z_axis = reference - float(np.dot(reference, forward)) * forward
    z_axis /= np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, forward)
    y_axis /= np.linalg.norm(y_axis)
    if previous_orientation is not None and np.dot(y_axis, previous_orientation[:, 1]) < 0.0:
        y_axis = -y_axis
        z_axis = -z_axis
    bank = 0.0
    if policy.bank_source == "controller":
        bank = _angle_from_state(values, ("command.bank_rad", "bank_rad", "command.bank", "bank", "rolli", "rollgd"), policy.bank_default_rad)
    bank_y = math.cos(bank) * y_axis + math.sin(bank) * z_axis
    bank_z = -math.sin(bank) * y_axis + math.cos(bank) * z_axis
    return np.column_stack((forward, bank_y, bank_z))


@dataclass(frozen=True, slots=True)
class RotorcraftAttitudePolicy:
    """Attitude policy for rotorcraft-specific forward-axis authority."""

    vehicle_type: str
    forward_source: str = "velocity"
    base: Pseudo6DofPolicy = Pseudo6DofPolicy()
    policy_id: ClassVar[str] = "rotorcraft"

    def __post_init__(self) -> None:
        if self.vehicle_type not in {"quadcopter", "tilt-rotor"}:
            raise ValueError("rotorcraft vehicle_type must be quadcopter or tilt-rotor")
        if self.forward_source not in {"velocity", "body-axis"}:
            raise ValueError("rotorcraft forward_source must be velocity or body-axis")

    @property
    def alignment(self) -> str:
        return self.base.alignment

    @property
    def speed_threshold_mps(self) -> float:
        return self.base.speed_threshold_mps

    @property
    def zero_speed_policy(self) -> str:
        return self.base.zero_speed_policy

    @property
    def pole_crossing_policy(self) -> str:
        return self.base.pole_crossing_policy

    def project(
        self,
        truth: TruthPoint,
        state: RuntimeState,
        previous_orientation: np.ndarray | None,
    ) -> np.ndarray:
        forward_override = None
        if self.forward_source == "body-axis":
            values = _state_named(state)
            if not all(name in values for name in ("qw", "qx", "qy", "qz")):
                raise ValueError("rotorcraft body-axis policy requires qw, qx, qy, and qz state channels")
            forward_override = _quaternion_matrix(values)[:, 0]
        return _project_pseudo_attitude(truth, state, self.base, previous_orientation, forward_override=forward_override)


class Pseudo6DofTruthProvider:
    """Lift translation into continuous attitude and derived body rate."""

    def __init__(self, base: Callable[[RuntimeState], TruthPoint], policy: AttitudeProjectionPolicy) -> None:
        self.base = base
        self.policy = policy
        self._last_time_s: float | None = None
        self._last_orientation: np.ndarray | None = None
        policy_id = str(getattr(policy, "policy_id", policy.__class__.__name__))
        self.contract = {
            **dict(getattr(base, "contract", {})),
            "mode": "pseudo-6dof",
            "orientation": {
                "available": True,
                "source": f"synthesized-{policy_id}",
                "policy": policy_id,
                "alignment": getattr(policy, "alignment", "velocity"),
                "zero_speed_policy": getattr(policy, "zero_speed_policy", None),
                "pole_crossing_policy": getattr(policy, "pole_crossing_policy", None),
            },
            "angular_rate": {"available": True, "source": "quaternion-difference"},
        }

    def __call__(self, state: RuntimeState) -> TruthPoint:
        truth = self.base(state)
        previous = self._last_orientation
        orientation = self.policy.project(truth, state, previous)
        rate = np.zeros(3)
        if self._last_time_s is not None and previous is not None and state.time > self._last_time_s:
            rate = _rotation_vector(previous.T @ orientation) / (state.time - self._last_time_s)
        self._last_time_s = state.time
        self._last_orientation = orientation.copy()
        return replace(truth, orientation_eci_from_body=orientation, angular_rate_body_radps=rate)


AttitudePolicyBuilder = Callable[[Mapping[str, object]], AttitudeProjectionPolicy]


class AttitudePolicyFactory:
    """Registry-backed factory for extensible attitude projection policies."""

    _builders: ClassVar[dict[str, AttitudePolicyBuilder]] = {}

    @classmethod
    def register(cls, kind: str, builder: AttitudePolicyBuilder) -> None:
        normalized = kind.strip()
        if not normalized:
            raise ValueError("attitude policy kind must not be empty")
        if normalized in cls._builders:
            raise ValueError(f"attitude policy kind {normalized!r} is already registered")
        cls._builders[normalized] = builder

    @classmethod
    def create(cls, config: Mapping[str, object]) -> AttitudeProjectionPolicy:
        kind = str(config.get("kind", "velocity-aligned"))
        builder = cls._builders.get(kind)
        if builder is None:
            raise ValueError(f"unsupported attitude policy kind {kind!r}")
        return builder(config)


def _base_policy_from_config(config: Mapping[str, object]) -> Pseudo6DofPolicy:
    return Pseudo6DofPolicy(
        alignment=str(config.get("alignment", "velocity")),
        speed_threshold_mps=_config_float(config, "speed_threshold_mps", 1.0),
        bank_source=str(config.get("bank_source", "controller")),
        bank_default_rad=_config_float(config, "bank_default_rad", 0.0),
        yaw_source=str(config.get("yaw_source", "controller-then-heading")),
        zero_speed_policy=str(config.get("zero_speed_policy", "hold-then-initial-frame")),
        pole_crossing_policy=str(config.get("pole_crossing_policy", "parallel-transport")),
    )


def _build_velocity_aligned_policy(config: Mapping[str, object]) -> AttitudeProjectionPolicy:
    return _base_policy_from_config(config)


def _build_rotorcraft_policy(config: Mapping[str, object]) -> AttitudeProjectionPolicy:
    return RotorcraftAttitudePolicy(
        vehicle_type=str(config.get("vehicle_type", "")),
        forward_source=str(config.get("forward_source", "velocity")),
        base=_base_policy_from_config(config),
    )


AttitudePolicyFactory.register("velocity-aligned", _build_velocity_aligned_policy)
AttitudePolicyFactory.register("rotorcraft", _build_rotorcraft_policy)


def truth_provider_contract(provider: Callable[[RuntimeState], TruthPoint]) -> dict[str, object]:
    """Return a JSON-safe capability declaration for a truth provider."""

    contract = getattr(provider, "contract", None)
    return dict(contract) if isinstance(contract, Mapping) else {}
