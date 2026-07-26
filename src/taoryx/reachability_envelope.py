"""Small deterministic rocket-to-glider reachability experiments.

This module is deliberately separate from the historical TAOS runtime.  It is
the first Alpha 3 workbench slice: a single-stage boost phase hands off to a
simple glide phase, and a batch of launch decisions is propagated into a
terminal footprint.  The model is a process-stabilization fixture, not a
validated vehicle or a weapons-performance model.
"""

from __future__ import annotations

import json
import math
import multiprocessing
from collections.abc import Iterable, Mapping, Sequence
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field, fields
from enum import StrEnum
from pathlib import Path

from .contracts import Frame, FrameVector3
from .contracts import Vector3 as ContractVector3
from .modes import Quaternion
from .rigid_body import RigidBody6DofModel, RigidBody6DofState, RigidBodyForceMoment
from .vehicle import (
    DetachedBodyDefinition,
    DetachedBodyShape,
    ImpulseFrame,
    TumblingPolicy,
)
from .vehicle import (
    StagedVehicleDefinition as StagedRocketSpec,
)
from .vehicle import (
    StageMassDefinition as RocketStageSpec,
)
from .vehicle import (
    StageSeparationEvent as StageSeparationSpec,
)

Vector3 = tuple[float, float, float]


class ReachabilityFidelity(StrEnum):
    """Reduced-order fidelity supported by the first envelope workbench."""

    POINT_MASS_3DOF = "point_mass_3dof"
    PSEUDO_6DOF = "pseudo_6dof"
    RIGID_BODY_6DOF = "rigid_body_6dof"
    ####


class EnvelopeTermination(StrEnum):
    """Reason one candidate trajectory stopped."""

    HORIZON = "horizon"
    GROUND_CONTACT = "ground_contact"
    INVALID = "invalid"
    ####


def _validate_finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    ####


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(component * component for component in vector))
    ####


def _add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a + b for a, b in zip(left, right, strict=True))  # type: ignore[return-value]
    ####


def _scale(vector: Vector3, factor: float) -> Vector3:
    return tuple(factor * component for component in vector)  # type: ignore[return-value]
    ####


def _dot(left: Vector3, right: Vector3) -> float:
    return sum(a * b for a, b in zip(left, right, strict=True))
    ####


def _cross(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )
    ####


def _unit(vector: Vector3, fallback: Vector3 = (1.0, 0.0, 0.0)) -> Vector3:
    length = _norm(vector)
    if length <= 1.0e-12:
        return fallback
    return _scale(vector, 1.0 / length)
    ####


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
    ####


def _optional_float(value: object) -> float | None:
    return None if value is None else float(value)


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
    ####


def _euler_quaternion(attitude_rad: Vector3) -> Quaternion:
    """Convert reduced roll, pitch, yaw state into a native attitude."""

    roll, pitch, yaw = (0.5 * value for value in attitude_rad)
    return Quaternion(
        math.cos(yaw) * math.cos(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(pitch) * math.sin(roll),
        math.cos(yaw) * math.cos(pitch) * math.sin(roll) - math.sin(yaw) * math.sin(pitch) * math.cos(roll),
        math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.cos(pitch) * math.sin(roll),
        math.sin(yaw) * math.cos(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(pitch) * math.sin(roll),
    ).normalized()


@dataclass(frozen=True, slots=True)
class RocketGlideVehicle:
    """A transparent rocket/glide body with an optional attached booster.

    With the booster fields at their defaults this remains the original
    single-stage rocket-to-glide fixture.  A nonzero booster configuration
    adds explicit ``boost``, ``coast``, and ``glide`` phases and drops booster
    mass at release.
    """

    vehicle_id: str = "alpha3-single-stage-rocket-glide"
    dry_mass_kg: float = 40.0
    propellant_mass_kg: float = 20.0
    thrust_n: float = 2_000.0
    burn_time_s: float = 8.0
    reference_area_m2: float = 0.12
    drag_coefficient: float = 0.22
    lift_to_drag: float = 4.0
    initial_speed_m_s: float = 25.0
    initial_altitude_m: float = 1.0
    gravity_m_s2: float = 9.80665
    sea_level_density_kg_m3: float = 1.225
    density_scale_height_m: float = 8_500.0
    attitude_time_constant_s: float = 0.35
    max_attitude_rate_rad_s: float = math.radians(90.0)
    booster_dry_mass_kg: float = 0.0
    booster_propellant_mass_kg: float = 0.0
    booster_thrust_n: float = 0.0
    booster_burn_time_s: float = 0.0
    booster_release_time_s: float = 0.0
    booster_detached_body: DetachedBodyDefinition | None = None
    booster_separation_impulse_n_s: ContractVector3 = ContractVector3(0.0, 0.0, 0.0)
    booster_separation_impulse_frame: ImpulseFrame = ImpulseFrame.BODY

    @classmethod
    def from_staged_spec(cls, spec: StagedRocketSpec, **overrides: object) -> RocketGlideVehicle:
        """Build the current reduced solver vehicle from an explicit stage stack."""

        core = spec.core_stage
        parameters: dict[str, object] = {
            "dry_mass_kg": core.dry_mass_kg,
            "propellant_mass_kg": core.propellant_mass_kg,
            "thrust_n": core.thrust_n,
            "burn_time_s": core.burn_time_s,
        }
        if spec.attached_stages:
            if len(spec.attached_stages) != 1:
                raise ValueError("the reduced reachability solver currently supports one attached stage")
            booster = spec.attached_stages[0]
            separation = spec.separation_events[0]
            parameters.update(
                {
                    "booster_dry_mass_kg": booster.dry_mass_kg,
                    "booster_propellant_mass_kg": booster.propellant_mass_kg,
                    "booster_thrust_n": booster.thrust_n,
                    "booster_burn_time_s": booster.burn_time_s,
                    "booster_release_time_s": separation.time_s,
                    "booster_detached_body": separation.detached_body,
                    "booster_separation_impulse_n_s": separation.impulse_body_n_s,
                    "booster_separation_impulse_frame": separation.impulse_frame,
                }
            )
        parameters.update(overrides)
        return cls(**parameters)

    def __post_init__(self) -> None:
        positive = (
            "dry_mass_kg",
            "reference_area_m2",
            "drag_coefficient",
            "lift_to_drag",
            "gravity_m_s2",
            "sea_level_density_kg_m3",
            "density_scale_height_m",
            "attitude_time_constant_s",
            "max_attitude_rate_rad_s",
        )
        for name in positive:
            value = float(getattr(self, name))
            _validate_finite(name, value)
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if not all(math.isfinite(value) for value in (
            self.booster_separation_impulse_n_s.x,
            self.booster_separation_impulse_n_s.y,
            self.booster_separation_impulse_n_s.z,
        )):
            raise ValueError("booster separation impulse must be finite")
        for name in (
            "propellant_mass_kg",
            "initial_speed_m_s",
            "initial_altitude_m",
            "booster_dry_mass_kg",
            "booster_propellant_mass_kg",
        ):
            value = float(getattr(self, name))
            _validate_finite(name, value)
            if value < 0.0:
                raise ValueError(f"{name} must be non-negative")
        if self.propellant_mass_kg > 0.0:
            if self.thrust_n <= 0.0 or self.burn_time_s <= 0.0:
                raise ValueError("a propellant-bearing vehicle stage requires thrust and burn time")
        elif self.thrust_n != 0.0 or self.burn_time_s != 0.0:
            raise ValueError("a propellant-less vehicle stage must have zero thrust and burn time")
        for name in ("booster_thrust_n", "booster_burn_time_s", "booster_release_time_s"):
            _validate_finite(name, float(getattr(self, name)))
        has_booster_mass = self.booster_dry_mass_kg > 0.0 or self.booster_propellant_mass_kg > 0.0
        if has_booster_mass:
            if self.booster_thrust_n <= 0.0 or self.booster_burn_time_s <= 0.0:
                raise ValueError("an attached booster requires thrust and burn time")
            if self.booster_release_time_s < self.booster_burn_time_s:
                raise ValueError("booster release cannot precede booster burnout")
        elif any((self.booster_thrust_n, self.booster_burn_time_s, self.booster_release_time_s)):
            raise ValueError("booster thrust and timing require booster mass")
        if self.booster_detached_body is not None:
            if not self.has_booster:
                raise ValueError("a detached booster body requires an attached booster")
            if not math.isclose(self.booster_detached_body.mass_kg, self.booster_dry_mass_kg, rel_tol=1.0e-8, abs_tol=1.0e-8):
                raise ValueError("detached booster body mass must match post-burn booster dry mass")
        ####

    @property
    def initial_mass_kg(self) -> float:
        return self.dry_mass_kg + self.propellant_mass_kg + self.booster_dry_mass_kg + self.booster_propellant_mass_kg
        ####

    @property
    def has_booster(self) -> bool:
        return self.booster_dry_mass_kg > 0.0 or self.booster_propellant_mass_kg > 0.0

    @property
    def burnout_mass_kg(self) -> float:
        return self.dry_mass_kg + self.propellant_mass_kg + self.booster_dry_mass_kg

    @property
    def release_mass_kg(self) -> float:
        return self.dry_mass_kg + self.propellant_mass_kg if self.has_booster else self.dry_mass_kg

    def phase_at(self, time_s: float) -> str:
        if self.has_booster:
            if time_s < self.booster_burn_time_s - 1.0e-12:
                return "boost"
            if time_s < self.booster_release_time_s - 1.0e-12:
                return "coast"
            return "glide"
        return "boost" if time_s < self.burn_time_s - 1.0e-12 else "glide"

    @property
    def glide_lift_coefficient(self) -> float:
        return self.drag_coefficient * self.lift_to_drag
        ####


@dataclass(frozen=True, slots=True)
class LaunchCommand:
    """Open-loop launch and glide-bank choices for one envelope candidate."""

    azimuth_rad: float
    elevation_rad: float
    bank_rad: float = 0.0

    def __post_init__(self) -> None:
        for name in ("azimuth_rad", "elevation_rad", "bank_rad"):
            _validate_finite(name, float(getattr(self, name)))
        if not -0.5 * math.pi < self.elevation_rad < 0.5 * math.pi:
            raise ValueError("elevation_rad must be between -pi/2 and pi/2")
        ####

    @property
    def launch_direction(self) -> Vector3:
        cosine = math.cos(self.elevation_rad)
        return (
            cosine * math.cos(self.azimuth_rad),
            cosine * math.sin(self.azimuth_rad),
            math.sin(self.elevation_rad),
        )
        ####


@dataclass(frozen=True, slots=True)
class SearchAxis:
    """One explicitly realized axis of the outer candidate search."""

    name: str
    units: str
    values: tuple[float, ...]
    sampling: str = "explicit_grid"

    def __post_init__(self) -> None:
        if not self.name or not self.units or not self.values:
            raise ValueError("search axes require a name, units, and at least one value")
        if len(set(self.values)) != len(self.values):
            raise ValueError(f"search axis {self.name!r} contains duplicate values")
        for value in self.values:
            _validate_finite(f"search axis {self.name}", value)
        ####

    def as_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "units": self.units,
            "values": list(self.values),
            "sampling": self.sampling,
        }
        ####


@dataclass(frozen=True, slots=True)
class ReachabilitySearchSpace:
    """The realized outer search, including ordering and candidate count."""

    axes: tuple[SearchAxis, ...]
    composition: str
    candidate_count: int
    ordering: str = "axis_order_then_cartesian_product"

    def __post_init__(self) -> None:
        if not self.axes or self.candidate_count <= 0:
            raise ValueError("a search space requires axes and a positive candidate count")
        if self.candidate_count != math.prod(len(axis.values) for axis in self.axes) and self.composition == "cartesian_product":
            raise ValueError("cartesian search-space candidate count does not match its axes")
        ####

    def as_dict(self) -> dict[str, object]:
        return {
            "axes": [axis.as_dict() for axis in self.axes],
            "composition": self.composition,
            "candidate_count": self.candidate_count,
            "ordering": self.ordering,
        }
        ####


@dataclass(frozen=True, slots=True)
class TerminalCriteria:
    """Terminal and impact acceptance rules for an envelope study."""

    min_speed_m_s: float = 0.0
    max_speed_m_s: float = math.inf
    require_ground_contact: bool = False
    target_position_m: Vector3 = (0.0, 0.0, 0.0)
    max_impact_radius_m: float | None = None
    min_impact_speed_m_s: float | None = None
    max_impact_speed_m_s: float | None = None

    def __post_init__(self) -> None:
        if self.min_speed_m_s < 0.0 or self.max_speed_m_s < self.min_speed_m_s:
            raise ValueError("terminal speed bounds are invalid")
        _validate_finite("min_speed_m_s", self.min_speed_m_s)
        if not math.isinf(self.max_speed_m_s):
            _validate_finite("max_speed_m_s", self.max_speed_m_s)
        for index, value in enumerate(self.target_position_m):
            _validate_finite(f"target_position_m[{index}]", value)
        if self.max_impact_radius_m is not None:
            _validate_finite("max_impact_radius_m", self.max_impact_radius_m)
            if self.max_impact_radius_m < 0.0:
                raise ValueError("max_impact_radius_m must be non-negative")
        for name, value in (
            ("min_impact_speed_m_s", self.min_impact_speed_m_s),
            ("max_impact_speed_m_s", self.max_impact_speed_m_s),
        ):
            if value is not None:
                _validate_finite(name, value)
                if value < 0.0:
                    raise ValueError(f"{name} must be non-negative")
        if (
            self.min_impact_speed_m_s is not None
            and self.max_impact_speed_m_s is not None
            and self.max_impact_speed_m_s < self.min_impact_speed_m_s
        ):
            raise ValueError("impact speed bounds are invalid")
        ####

    def as_dict(self) -> dict[str, object]:
        return {
            "min_speed_m_s": self.min_speed_m_s,
            "max_speed_m_s": None if math.isinf(self.max_speed_m_s) else self.max_speed_m_s,
            "require_ground_contact": self.require_ground_contact,
            "target_position_m": list(self.target_position_m),
            "max_impact_radius_m": self.max_impact_radius_m,
            "min_impact_speed_m_s": self.min_impact_speed_m_s,
            "max_impact_speed_m_s": self.max_impact_speed_m_s,
        }
        ####

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> TerminalCriteria:
        """Reconstruct criteria from the serialized success specification."""

        target = payload.get("target_position_m", (0.0, 0.0, 0.0))
        if not isinstance(target, (list, tuple)) or len(target) != 3:
            raise ValueError("target_position_m must contain three values")
        max_speed = payload.get("max_speed_m_s")
        return cls(
            min_speed_m_s=float(payload.get("min_speed_m_s", 0.0)),
            max_speed_m_s=math.inf if max_speed is None else float(max_speed),
            require_ground_contact=bool(payload.get("require_ground_contact", False)),
            target_position_m=tuple(float(value) for value in target),  # type: ignore[arg-type]
            max_impact_radius_m=_optional_float(payload.get("max_impact_radius_m")),
            min_impact_speed_m_s=_optional_float(payload.get("min_impact_speed_m_s")),
            max_impact_speed_m_s=_optional_float(payload.get("max_impact_speed_m_s")),
        )


@dataclass(frozen=True, slots=True)
class PointMass3DofState:
    """Position, velocity, and mass state for the point-mass reduction."""

    time_s: float
    position_m: Vector3
    velocity_m_s: Vector3
    mass_kg: float
    phase: str

    @property
    def speed_m_s(self) -> float:
        return _norm(self.velocity_m_s)
        ####


@dataclass(frozen=True, slots=True)
class Pseudo6DofState(PointMass3DofState):
    """Point-mass translation with filtered roll, pitch, yaw, and rates."""

    attitude_rad: Vector3
    attitude_rate_rad_s: Vector3
    ####


@dataclass(frozen=True, slots=True)
class RigidBody6DofReachabilityState:
    """Reachability wrapper around the native rigid-body state contract."""

    native: RigidBody6DofState
    phase: str = "ballistic"

    @property
    def time_s(self) -> float:
        return self.native.time

    @property
    def position_m(self) -> Vector3:
        vector = self.native.position.vector
        return (vector.x, vector.y, vector.z)

    @property
    def velocity_m_s(self) -> Vector3:
        vector = self.native.velocity.vector
        return (vector.x, vector.y, vector.z)

    @property
    def mass_kg(self) -> float:
        return self.native.mass

    @property
    def speed_m_s(self) -> float:
        return _norm(self.velocity_m_s)


State = PointMass3DofState | Pseudo6DofState | RigidBody6DofReachabilityState


@dataclass(frozen=True, slots=True)
class DetachedBodyTrajectory:
    """Reduced witness trajectory for one body spawned at separation."""

    body_id: str
    shape: str
    parent_event_id: str
    deployment_time_s: float
    fidelity: ReachabilityFidelity
    states: tuple[State, ...]
    termination: EnvelopeTermination
    telemetry: tuple[dict[str, object], ...] = ()

    @property
    def classification(self) -> str:
        """Return the stable child terminal outcome used by envelope artifacts."""

        return {
            EnvelopeTermination.GROUND_CONTACT: "impact",
            EnvelopeTermination.HORIZON: "timeout",
            EnvelopeTermination.INVALID: "invalid",
        }[self.termination]

    @property
    def terminal(self) -> State:
        return self.states[-1]
    ####


@dataclass(frozen=True, slots=True)
class TrajectoryResult:
    """One deterministic witness trajectory."""

    fidelity: ReachabilityFidelity
    command: LaunchCommand
    states: tuple[State, ...]
    termination: EnvelopeTermination
    spawned_bodies: tuple[DetachedBodyTrajectory, ...] = ()
    deployment_events: tuple[dict[str, object], ...] = ()
    telemetry: tuple[dict[str, object], ...] = ()

    @property
    def terminal(self) -> State:
        return self.states[-1]
        ####

    @property
    def downrange_m(self) -> float:
        return self.terminal.position_m[0]
        ####

    @property
    def crossrange_m(self) -> float:
        return self.terminal.position_m[1]
        ####


@dataclass(frozen=True, slots=True)
class EnvelopeSample:
    """A candidate result retained in input order for reproducibility."""

    index: int
    command: LaunchCommand
    trajectory: TrajectoryResult
    feasible: bool
    limiting_factor: str | None
    classification: str = "feasible"
    failure_reasons: tuple[str, ...] = ()
    terminal_margins: tuple[tuple[str, float | None], ...] = ()
    path_metrics: tuple[tuple[str, float], ...] = ()

    @property
    def terminal(self) -> State:
        return self.trajectory.terminal
        ####

    @property
    def query_id(self) -> str:
        return f"query-{self.index:06d}"
        ####

    @property
    def timed_out(self) -> bool:
        return self.trajectory.termination is EnvelopeTermination.HORIZON


@dataclass(frozen=True, slots=True)
class _EnvelopeEvaluation:
    """Serializable evaluation data produced for every candidate."""

    feasible: bool
    classification: str
    failure_reasons: tuple[str, ...]
    limiting_factor: str | None
    terminal_margins: tuple[tuple[str, float | None], ...]
    path_metrics: tuple[tuple[str, float], ...]
    ####


@dataclass(frozen=True, slots=True)
class EnvelopeBounds:
    """Axis-aligned bounds of the terminal witness cloud."""

    downrange_min_m: float
    downrange_max_m: float
    crossrange_min_m: float
    crossrange_max_m: float
    altitude_min_m: float
    altitude_max_m: float
    ####


@dataclass(frozen=True, slots=True)
class ReachabilityEnvelope:
    """Deterministic terminal footprint and its replayable witnesses."""

    vehicle_id: str
    fidelity: ReachabilityFidelity
    samples: tuple[EnvelopeSample, ...]
    workers: int
    step_size_s: float
    horizon_s: float
    study_id: str = "alpha3_rocket_glide_terminal_footprint"
    search_space: ReachabilitySearchSpace | None = None
    terminal_criteria: TerminalCriteria = field(default_factory=TerminalCriteria)
    vehicle_parameters: tuple[tuple[str, object], ...] = ()
    integrator: str = "fixed_step_rk4"
    state_fields: tuple[str, ...] = ()
    provenance: tuple[tuple[str, object], ...] = ()
    spawn_children: bool = False

    @property
    def feasible_samples(self) -> tuple[EnvelopeSample, ...]:
        return tuple(sample for sample in self.samples if sample.feasible)
        ####

    @property
    def unsuccessful_samples(self) -> tuple[EnvelopeSample, ...]:
        return tuple(sample for sample in self.samples if not sample.feasible)
        ####

    @property
    def timed_out_samples(self) -> tuple[EnvelopeSample, ...]:
        return tuple(sample for sample in self.samples if sample.timed_out)

    @property
    def timed_out_query_ids(self) -> tuple[str, ...]:
        return tuple(sample.query_id for sample in self.timed_out_samples)

    @property
    def classification_counts(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for sample in self.samples:
            counts[sample.classification] = counts.get(sample.classification, 0) + 1
        return dict(sorted(counts.items()))
        ####

    @property
    def failure_reason_counts(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for sample in self.unsuccessful_samples:
            for reason in sample.failure_reasons:
                counts[reason] = counts.get(reason, 0) + 1
        return dict(sorted(counts.items()))
        ####

    @property
    def child_classification_counts(self) -> dict[str, int]:
        """Count terminal outcomes across all spawned child witnesses."""

        counts: dict[str, int] = {}
        for sample in self.samples:
            for child in sample.trajectory.spawned_bodies:
                counts[child.classification] = counts.get(child.classification, 0) + 1
        return dict(sorted(counts.items()))
        ####

    @property
    def bounds(self) -> EnvelopeBounds | None:
        samples = self.feasible_samples
        if not samples:
            return None
        positions = [sample.terminal.position_m for sample in samples]
        return EnvelopeBounds(
            downrange_min_m=min(position[0] for position in positions),
            downrange_max_m=max(position[0] for position in positions),
            crossrange_min_m=min(position[1] for position in positions),
            crossrange_max_m=max(position[1] for position in positions),
            altitude_min_m=min(position[2] for position in positions),
            altitude_max_m=max(position[2] for position in positions),
        )
        ####

    @property
    def evaluated_bounds(self) -> EnvelopeBounds | None:
        return _bounds_for_samples(self.samples)
        ####

    def as_dict(self, *, include_trajectories: bool = False) -> dict[str, object]:
        """Return a versioned, self-describing JSON-compatible result."""

        search_space = self.search_space or _infer_search_space(tuple(sample.command for sample in self.samples))
        state_fields = self.state_fields or _state_fields(self.fidelity)
        vehicle_parameters = dict(self.vehicle_parameters)
        bounds = self.bounds
        evaluated_bounds = self.evaluated_bounds
        samples = [_sample_dict(sample, state_fields, include_trajectories) for sample in self.samples]
        deployment_events = [event for sample in self.samples for event in sample.trajectory.deployment_events]
        spawned_children = [child for sample in self.samples for child in sample.trajectory.spawned_bodies]
        detached_body = vehicle_parameters.get("booster_detached_body")
        deployment_configuration = {
            "kind": "passive_aero_ballistic" if isinstance(detached_body, Mapping) else "none",
            "enabled": self.spawn_children,
            "configured": isinstance(detached_body, Mapping),
            "event_id": "booster-release" if isinstance(detached_body, Mapping) else None,
            "accepted_event_count": len(deployment_events),
            "spawned_child_count": len(spawned_children),
            "child_models": [detached_body] if isinstance(detached_body, Mapping) else [],
            "child_classification_counts": self.child_classification_counts,
        }
        payload: dict[str, object] = {
            "schema": "trajectory.reachability-envelope/v1alpha1",
        "study": {
                "study_id": self.study_id,
                "vehicle_id": self.vehicle_id,
                "vehicle_type": "staged_rocket_glide" if vehicle_parameters.get("booster_release_time_s", 0.0) else "single_stage_rocket_glide",
                "fidelity": self.fidelity.value,
                "model": {
                    "integrator": self.integrator,
                    "integrator_order": 4,
                    "phases": ["boost", "coast", "glide"] if vehicle_parameters.get("booster_release_time_s", 0.0) else ["boost", "glide"],
                    "state_fields": list(state_fields),
                    "force_model": ["thrust", "drag", "lift", "gravity"],
                    "atmosphere_model": "exponential_density",
                    "attitude_model": "filtered_commanded_attitude" if self.fidelity is ReachabilityFidelity.PSEUDO_6DOF else "launch_direction_and_bank_command",
                    "spawn_children": self.spawn_children,
                    "deployment_composition": "enabled" if self.spawn_children else "standalone",
                },
                "vehicle_parameters": vehicle_parameters,
            },
            "provenance": dict(self.provenance),
            "search_space": search_space.as_dict(),
            "success_spec": self.terminal_criteria.as_dict(),
            "deployment": deployment_configuration,
            "execution": {
                "step_size_s": self.step_size_s,
                "horizon_s": self.horizon_s,
                "workers_requested": self.workers,
                "worker_backend": "serial" if self.workers == 1 else "spawn_process_pool",
                "deterministic": True,
                "candidate_order_preserved": True,
                "spawn_children": self.spawn_children,
                "deployment_composition": "enabled" if self.spawn_children else "standalone",
            },
            "summary": {
                "candidate_count": len(self.samples),
                "feasible_count": len(self.feasible_samples),
                "unsuccessful_count": len(self.unsuccessful_samples),
                "classification_counts": self.classification_counts,
                "failure_reason_counts": self.failure_reason_counts,
                "child_classification_counts": self.child_classification_counts,
                "feasible_bounds": _bounds_dict(bounds),
                "evaluated_bounds": _bounds_dict(evaluated_bounds),
                "successful_query_ids": [sample.query_id for sample in self.feasible_samples],
                "unsuccessful_query_ids": [sample.query_id for sample in self.unsuccessful_samples],
                "timed_out_count": len(self.timed_out_samples),
                "timed_out_query_ids": list(self.timed_out_query_ids),
            },
            "vehicle_id": self.vehicle_id,
            "fidelity": self.fidelity.value,
            "workers": self.workers,
            "step_size_s": self.step_size_s,
            "horizon_s": self.horizon_s,
            "sample_count": len(self.samples),
            "feasible_count": len(self.feasible_samples),
            "bounds": _bounds_dict(bounds),
            "samples": samples,
        }
        return payload
        ####

    def write_json(self, path: str | Path, *, include_trajectories: bool = True) -> None:
        """Write the complete result artifact to a deterministic JSON file."""

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps(self.as_dict(include_trajectories=include_trajectories), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        ####


@dataclass(frozen=True, slots=True)
class _Derivative:
    position_m_s: Vector3
    velocity_m_s2: Vector3
    mass_kg_s: float
    attitude_rad_s: Vector3 = (0.0, 0.0, 0.0)
    attitude_rate_rad_s2: Vector3 = (0.0, 0.0, 0.0)
    ####


def _launch_state(vehicle: RocketGlideVehicle, command: LaunchCommand, fidelity: ReachabilityFidelity) -> State:
    direction = command.launch_direction
    velocity = _scale(direction, vehicle.initial_speed_m_s)
    common = {
        "time_s": 0.0,
        "position_m": (0.0, 0.0, vehicle.initial_altitude_m),
        "velocity_m_s": velocity,
        "mass_kg": vehicle.initial_mass_kg,
        "phase": "boost",
    }
    if fidelity in {ReachabilityFidelity.PSEUDO_6DOF, ReachabilityFidelity.RIGID_BODY_6DOF}:
        return Pseudo6DofState(
            **common,
            attitude_rad=(0.0, 0.0, 0.0),
            attitude_rate_rad_s=(0.0, 0.0, 0.0),
        )
    return PointMass3DofState(**common)
    ####


def _atmosphere(vehicle: RocketGlideVehicle, altitude_m: float) -> float:
    return vehicle.sea_level_density_kg_m3 * math.exp(-max(0.0, altitude_m) / vehicle.density_scale_height_m)
    ####


def _lift_direction(velocity: Vector3, bank_rad: float) -> Vector3:
    velocity_unit = _unit(velocity)
    up = (0.0, 0.0, 1.0)
    normal_up = _unit(_add(up, _scale(velocity_unit, -_dot(up, velocity_unit))), fallback=up)
    bank_axis = _unit(_cross(velocity_unit, normal_up), fallback=(0.0, 1.0, 0.0))
    return _unit(_add(_scale(normal_up, math.cos(bank_rad)), _scale(bank_axis, math.sin(bank_rad))), fallback=normal_up)
    ####


def _attitude_target(command: LaunchCommand) -> Vector3:
    return (command.bank_rad, command.elevation_rad, command.azimuth_rad)
    ####


def _attitude_derivative(vehicle: RocketGlideVehicle, state: Pseudo6DofState, command: LaunchCommand) -> tuple[Vector3, Vector3]:
    target = _attitude_target(command)
    errors = (
        _wrap_angle(target[0] - state.attitude_rad[0]),
        _wrap_angle(target[1] - state.attitude_rad[1]),
        _wrap_angle(target[2] - state.attitude_rad[2]),
    )
    desired_rates = tuple(
        _clamp(error / vehicle.attitude_time_constant_s, -vehicle.max_attitude_rate_rad_s, vehicle.max_attitude_rate_rad_s)
        for error in errors
    )
    rates = tuple(float(rate) for rate in desired_rates)
    rate_acceleration = tuple(
        (desired - actual) / vehicle.attitude_time_constant_s
        for desired, actual in zip(desired_rates, state.attitude_rate_rad_s, strict=True)
    )
    return rates, rate_acceleration  # type: ignore[return-value]
    ####


def _derivative(
    vehicle: RocketGlideVehicle,
    command: LaunchCommand,
    state: State,
    fidelity: ReachabilityFidelity,
) -> _Derivative:
    speed = _norm(state.velocity_m_s)
    velocity_unit = _unit(state.velocity_m_s, fallback=command.launch_direction)
    density = _atmosphere(vehicle, state.position_m[2])
    dynamic_pressure = 0.5 * density * speed * speed
    drag = _scale(velocity_unit, -dynamic_pressure * vehicle.reference_area_m2 * vehicle.drag_coefficient)
    gravity = (0.0, 0.0, -vehicle.gravity_m_s2 * state.mass_kg)
    phase = vehicle.phase_at(state.time_s)
    boost = phase == "boost"
    if fidelity is ReachabilityFidelity.PSEUDO_6DOF:
        assert isinstance(state, Pseudo6DofState)
        attitude_rates, rate_acceleration = _attitude_derivative(vehicle, state, command)
        pitch = state.attitude_rad[1]
        yaw = state.attitude_rad[2]
        thrust_direction = (
            math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            math.sin(pitch),
        )
        bank_rad = state.attitude_rad[0]
    else:
        attitude_rates = (0.0, 0.0, 0.0)
        rate_acceleration = (0.0, 0.0, 0.0)
        thrust_direction = command.launch_direction
        bank_rad = command.bank_rad

    thrust_n = vehicle.booster_thrust_n if vehicle.has_booster else vehicle.thrust_n
    thrust = _scale(thrust_direction, thrust_n if boost else 0.0)
    lift = _scale(
        _lift_direction(state.velocity_m_s, bank_rad),
        dynamic_pressure * vehicle.reference_area_m2 * vehicle.glide_lift_coefficient if not boost else 0.0,
    )
    acceleration = _scale(_add(_add(_add(thrust, drag), lift), gravity), 1.0 / state.mass_kg)
    return _Derivative(
        position_m_s=state.velocity_m_s,
        velocity_m_s2=acceleration,
        mass_kg_s=(
            -vehicle.booster_propellant_mass_kg / vehicle.booster_burn_time_s
            if vehicle.has_booster and boost
            else -vehicle.propellant_mass_kg / vehicle.burn_time_s
            if not vehicle.has_booster and boost
            else 0.0
        ),
        attitude_rad_s=attitude_rates,
        attitude_rate_rad_s2=rate_acceleration,
    )
    ####


def _state_with_delta(state: State, derivative: _Derivative, scale: float) -> State:
    position = _add(state.position_m, _scale(derivative.position_m_s, scale))
    velocity = _add(state.velocity_m_s, _scale(derivative.velocity_m_s2, scale))
    mass = state.mass_kg + scale * derivative.mass_kg_s
    if isinstance(state, Pseudo6DofState):
        attitude = _add(state.attitude_rad, _scale(derivative.attitude_rad_s, scale))
        rates = _add(state.attitude_rate_rad_s, _scale(derivative.attitude_rate_rad_s2, scale))
        return Pseudo6DofState(state.time_s + scale, position, velocity, mass, state.phase, attitude, rates)
    return PointMass3DofState(state.time_s + scale, position, velocity, mass, state.phase)
    ####


def _rk4_step(
    vehicle: RocketGlideVehicle,
    command: LaunchCommand,
    state: State,
    step_size_s: float,
    fidelity: ReachabilityFidelity,
) -> State:
    first = _derivative(vehicle, command, state, fidelity)
    second = _state_with_delta(state, first, 0.5 * step_size_s)
    second = _derivative(vehicle, command, second, fidelity)
    third = _state_with_delta(state, second, 0.5 * step_size_s)
    third = _derivative(vehicle, command, third, fidelity)
    fourth = _state_with_delta(state, third, step_size_s)
    fourth = _derivative(vehicle, command, fourth, fidelity)

    position_rate = tuple(
        (first.position_m_s[index] + 2.0 * second.position_m_s[index] + 2.0 * third.position_m_s[index] + fourth.position_m_s[index]) / 6.0
        for index in range(3)
    )
    velocity_rate = tuple(
        (first.velocity_m_s2[index] + 2.0 * second.velocity_m_s2[index] + 2.0 * third.velocity_m_s2[index] + fourth.velocity_m_s2[index]) / 6.0
        for index in range(3)
    )
    mass_rate = (first.mass_kg_s + 2.0 * second.mass_kg_s + 2.0 * third.mass_kg_s + fourth.mass_kg_s) / 6.0
    updated_position = _add(state.position_m, _scale(position_rate, step_size_s))
    updated_velocity = _add(state.velocity_m_s, _scale(velocity_rate, step_size_s))
    updated_mass = max(state.mass_kg + mass_rate * step_size_s, vehicle.release_mass_kg)
    if vehicle.has_booster:
        if state.time_s < vehicle.booster_burn_time_s <= state.time_s + step_size_s:
            updated_mass = vehicle.burnout_mass_kg
        if state.time_s < vehicle.booster_release_time_s <= state.time_s + step_size_s:
            updated_mass = vehicle.release_mass_kg
    elif state.time_s < vehicle.burn_time_s <= state.time_s + step_size_s:
        updated_mass = vehicle.release_mass_kg
    next_phase = vehicle.phase_at(state.time_s + step_size_s)
    if isinstance(state, Pseudo6DofState):
        attitude_rate = tuple(
            (first.attitude_rad_s[index] + 2.0 * second.attitude_rad_s[index] + 2.0 * third.attitude_rad_s[index] + fourth.attitude_rad_s[index]) / 6.0
            for index in range(3)
        )
        rate_acceleration = tuple(
            (first.attitude_rate_rad_s2[index] + 2.0 * second.attitude_rate_rad_s2[index] + 2.0 * third.attitude_rate_rad_s2[index] + fourth.attitude_rate_rad_s2[index]) / 6.0
            for index in range(3)
        )
        attitude = _add(state.attitude_rad, _scale(attitude_rate, step_size_s))
        rates = _add(state.attitude_rate_rad_s, _scale(rate_acceleration, step_size_s))
        return Pseudo6DofState(
            state.time_s + step_size_s,
            updated_position,
            updated_velocity,
            updated_mass,
            next_phase,
            attitude,
            rates,
        )
    return PointMass3DofState(
        state.time_s + step_size_s,
        updated_position,
        updated_velocity,
        updated_mass,
        next_phase,
    )
    ####


def _body_longitudinal_axis(state: State) -> Vector3:
    """Return the reduced body-x axis represented by yaw and pitch."""

    if isinstance(state, RigidBody6DofReachabilityState):
        axis = state.native.attitude.rotate(ContractVector3(1.0, 0.0, 0.0))
        return (axis.x, axis.y, axis.z)
    pitch = state.attitude_rad[1]
    yaw = state.attitude_rad[2]
    return (math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch))
    ####


def _projected_area(body: DetachedBodyDefinition, state: State) -> float:
    """Evaluate scalar projected area for the four Alpha 3 shape profiles."""

    if body.shape is DetachedBodyShape.SPHERE or not isinstance(state, (Pseudo6DofState, RigidBody6DofReachabilityState)):
        return body.reference_area_m2
    radius_or_axial, *remaining = body.dimensions_m
    axis = _body_longitudinal_axis(state)
    velocity_axis_cosine = abs(_dot(_unit(state.velocity_m_s), axis))
    angle = math.acos(_clamp(velocity_axis_cosine, 0.0, 1.0))
    sine = abs(math.sin(angle))
    cosine = abs(math.cos(angle))
    if body.shape is DetachedBodyShape.CYLINDER:
        length = remaining[0]
        return math.pi * radius_or_axial**2 * cosine + 2.0 * radius_or_axial * length * sine
    if body.shape is DetachedBodyShape.SPHEROID:
        transverse = remaining[0]
        return math.pi * transverse * math.sqrt(transverse**2 * cosine**2 + radius_or_axial**2 * sine**2)
    if body.shape is DetachedBodyShape.CONE:
        height = remaining[0]
        if cosine <= 1.0e-12:
            return radius_or_axial * height
        delta = height * sine / (radius_or_axial * cosine)
        if delta <= 1.0:
            return math.pi * radius_or_axial**2 * cosine
        return radius_or_axial**2 * cosine * (math.pi + math.sqrt(delta**2 - 1.0) - math.acos(1.0 / delta))
    return body.reference_area_m2
    ####


def _detached_inertia(body: DetachedBodyDefinition) -> ContractVector3:
    """Return declared inertia or a transparent geometric surrogate."""

    if body.inertia_kg_m2 is not None:
        return body.inertia_kg_m2
    radius_or_axial, *remaining = body.dimensions_m
    mass = body.mass_kg
    if body.shape is DetachedBodyShape.SPHERE:
        value = 0.4 * mass * radius_or_axial**2
        return ContractVector3(value, value, value)
    if body.shape is DetachedBodyShape.CYLINDER:
        length = remaining[0]
        return ContractVector3(
            0.5 * mass * radius_or_axial**2,
            mass * (3.0 * radius_or_axial**2 + length**2) / 12.0,
            mass * (3.0 * radius_or_axial**2 + length**2) / 12.0,
        )
    if body.shape is DetachedBodyShape.CONE:
        height = remaining[0]
        return ContractVector3(
            0.3 * mass * radius_or_axial**2,
            mass * (0.15 * radius_or_axial**2 + 0.6 * height**2),
            mass * (0.15 * radius_or_axial**2 + 0.6 * height**2),
        )
    transverse = remaining[0] if remaining else radius_or_axial
    return ContractVector3(
        0.2 * mass * transverse**2,
        mass * (radius_or_axial**2 + transverse**2) / 5.0,
        mass * (radius_or_axial**2 + transverse**2) / 5.0,
    )


def _rigid_body_model(vehicle: RocketGlideVehicle, body: DetachedBodyDefinition) -> RigidBody6DofModel:
    """Build the native rigid-body model for a spawned ballistic body."""

    inertia = _detached_inertia(body)

    def force_moment(native: RigidBody6DofState) -> RigidBodyForceMoment:
        velocity = native.attitude.conjugate().rotate(native.velocity.vector)
        speed = velocity.norm()
        density = _atmosphere(vehicle, native.position.vector.z)
        dynamic_pressure = 0.5 * density * speed**2
        area_state = RigidBody6DofReachabilityState(native)
        area = _projected_area(body, area_state)
        drag_body = velocity.scaled(-dynamic_pressure * area * vehicle.drag_coefficient / max(speed, 1.0e-12))
        if body.tumbling_policy in {TumblingPolicy.FIXED_ATTITUDE, TumblingPolicy.PRESCRIBED_SPIN}:
            moment = ContractVector3(0.0, 0.0, 0.0)
        else:
            aerodynamic_moment = (body.center_of_pressure_m - body.center_of_mass_m).cross(drag_body)
            # The surrogate represents unresolved aerodynamic damping explicitly
            # so coarse envelope steps cannot excite an unbounded tumble.
            moment = aerodynamic_moment - ContractVector3(
                inertia.x * native.body_rate.x / 5.0,
                inertia.y * native.body_rate.y / 5.0,
                inertia.z * native.body_rate.z / 5.0,
            )
        return RigidBodyForceMoment(
            drag_body,
            moment,
            aero_force_body=drag_body,
            aero_moment_body=moment,
        )

    return RigidBody6DofModel(
        inertia=inertia,
        force_moment=force_moment,
        gravity=lambda _state: ContractVector3(0.0, 0.0, -vehicle.gravity_m_s2),
        dry_mass=body.mass_kg,
    )


def _rigid_body_rk4_step(vehicle: RocketGlideVehicle, state: RigidBody6DofReachabilityState, step_size_s: float) -> RigidBody6DofReachabilityState:
    """Advance a spawned body through the native rigid-body derivative."""

    if step_size_s > 0.05:
        current = state
        remaining = step_size_s
        while remaining > 1.0e-12:
            substep = min(0.05, remaining)
            current = _rigid_body_rk4_step(vehicle, current, substep)
            remaining -= substep
        return current
    body = vehicle.booster_detached_body
    if body is None:
        raise ValueError("rigid-body propagation requires a detached body")
    model = _rigid_body_model(vehicle, body)
    values = state.native.to_values()

    def derivative(time: float, current: tuple[float, ...]) -> tuple[float, ...]:
        return model.derivative(RigidBody6DofState.from_values(time, current))

    first = derivative(state.time_s, values)
    second_values = tuple(value + 0.5 * step_size_s * slope for value, slope in zip(values, first, strict=True))
    second = derivative(state.time_s + 0.5 * step_size_s, second_values)
    third_values = tuple(value + 0.5 * step_size_s * slope for value, slope in zip(values, second, strict=True))
    third = derivative(state.time_s + 0.5 * step_size_s, third_values)
    fourth_values = tuple(value + step_size_s * slope for value, slope in zip(values, third, strict=True))
    fourth = derivative(state.time_s + step_size_s, fourth_values)
    integrated = tuple(
        value + step_size_s * (first_value + 2.0 * second_value + 2.0 * third_value + fourth_value) / 6.0
        for value, first_value, second_value, third_value, fourth_value in zip(values, first, second, third, fourth, strict=True)
    )
    return RigidBody6DofReachabilityState(
        RigidBody6DofState.from_values(state.time_s + step_size_s, integrated),
        state.phase,
    )


def _tumble_rate_acceleration(body: DetachedBodyDefinition, state: Pseudo6DofState) -> Vector3:
    """Return the bounded reduced attitude-rate model for a detached body."""

    policy = body.tumbling_policy
    if policy in {TumblingPolicy.FIXED_ATTITUDE, TumblingPolicy.PRESCRIBED_SPIN}:
        return (0.0, 0.0, 0.0)
    if body.inertia_kg_m2 is None:
        raise ValueError(f"{policy.value} requires detached-body inertia")
    inertia = body.inertia_kg_m2
    p, q, r = state.attitude_rate_rad_s
    torque_free = (
        (inertia.y - inertia.z) * q * r / inertia.x,
        (inertia.z - inertia.x) * r * p / inertia.y,
        (inertia.x - inertia.y) * p * q / inertia.z,
    )
    if policy is TumblingPolicy.STOCHASTIC_TUMBLE:
        modulation = 0.05 * math.sin(0.7 * state.time_s)
        return tuple(value + modulation * rate for value, rate in zip(torque_free, state.attitude_rate_rad_s, strict=True))  # type: ignore[return-value]
    return torque_free
    ####


def _ballistic_derivative(vehicle: RocketGlideVehicle, state: State) -> _Derivative:
    """Evaluate reduced drag, gravity, and detached-body attitude dynamics."""

    body = vehicle.booster_detached_body
    if body is None:
        raise ValueError("detached-body derivative requires a body definition")
    speed = _norm(state.velocity_m_s)
    velocity_unit = _unit(state.velocity_m_s)
    density = _atmosphere(vehicle, state.position_m[2])
    dynamic_pressure = 0.5 * density * speed * speed
    area = _projected_area(body, state)
    drag = _scale(velocity_unit, -dynamic_pressure * area * vehicle.drag_coefficient)
    gravity = (0.0, 0.0, -vehicle.gravity_m_s2 * state.mass_kg)
    acceleration = _scale(_add(drag, gravity), 1.0 / state.mass_kg)
    if isinstance(state, Pseudo6DofState):
        return _Derivative(
            state.velocity_m_s,
            acceleration,
            0.0,
            state.attitude_rate_rad_s,
            _tumble_rate_acceleration(body, state),
        )
    return _Derivative(state.velocity_m_s, acceleration, 0.0)
    ####


def _ballistic_rk4_step(vehicle: RocketGlideVehicle, state: State, step_size_s: float) -> State:
    """Advance one detached body with the same reduced RK4 convention."""

    if isinstance(state, RigidBody6DofReachabilityState):
        return _rigid_body_rk4_step(vehicle, state, step_size_s)
    first = _ballistic_derivative(vehicle, state)
    second = _ballistic_derivative(vehicle, _state_with_delta(state, first, 0.5 * step_size_s))
    third = _ballistic_derivative(vehicle, _state_with_delta(state, second, 0.5 * step_size_s))
    fourth = _ballistic_derivative(vehicle, _state_with_delta(state, third, step_size_s))
    position_rate = tuple(
        (first.position_m_s[index] + 2.0 * second.position_m_s[index] + 2.0 * third.position_m_s[index] + fourth.position_m_s[index]) / 6.0
        for index in range(3)
    )
    velocity_rate = tuple(
        (first.velocity_m_s2[index] + 2.0 * second.velocity_m_s2[index] + 2.0 * third.velocity_m_s2[index] + fourth.velocity_m_s2[index]) / 6.0
        for index in range(3)
    )
    position = _add(state.position_m, _scale(position_rate, step_size_s))
    velocity = _add(state.velocity_m_s, _scale(velocity_rate, step_size_s))
    if isinstance(state, Pseudo6DofState):
        attitude_rate = tuple(
            (first.attitude_rad_s[index] + 2.0 * second.attitude_rad_s[index] + 2.0 * third.attitude_rad_s[index] + fourth.attitude_rad_s[index]) / 6.0
            for index in range(3)
        )
        rate_acceleration = tuple(
            (first.attitude_rate_rad_s2[index] + 2.0 * second.attitude_rate_rad_s2[index] + 2.0 * third.attitude_rate_rad_s2[index] + fourth.attitude_rate_rad_s2[index]) / 6.0
            for index in range(3)
        )
        return Pseudo6DofState(
            state.time_s + step_size_s,
            position,
            velocity,
            state.mass_kg,
            "ballistic",
            _add(state.attitude_rad, _scale(attitude_rate, step_size_s)),
            _add(state.attitude_rate_rad_s, _scale(rate_acceleration, step_size_s)),
        )
    return PointMass3DofState(state.time_s + step_size_s, position, velocity, state.mass_kg, "ballistic")
    ####


def _parent_attitude(parent: State) -> Quaternion:
    if isinstance(parent, RigidBody6DofReachabilityState):
        return parent.native.attitude
    if isinstance(parent, Pseudo6DofState):
        return _euler_quaternion(parent.attitude_rad)
    return Quaternion.identity()


def _separation_impulse_inertial(vehicle: RocketGlideVehicle, parent: State) -> ContractVector3:
    impulse = vehicle.booster_separation_impulse_n_s
    if vehicle.booster_separation_impulse_frame is ImpulseFrame.BODY:
        return _parent_attitude(parent).rotate(impulse)
    return impulse


def _apply_parent_separation_delta_v(vehicle: RocketGlideVehicle, parent: State) -> State:
    """Apply the retained-stack impulse at the accepted release boundary."""

    if vehicle.booster_detached_body is None:
        return parent
    impulse = _separation_impulse_inertial(vehicle, parent)
    delta = impulse.scaled(1.0 / vehicle.release_mass_kg)
    velocity = _add(parent.velocity_m_s, (delta.x, delta.y, delta.z))
    if isinstance(parent, Pseudo6DofState):
        return Pseudo6DofState(parent.time_s, parent.position_m, velocity, parent.mass_kg, parent.phase, parent.attitude_rad, parent.attitude_rate_rad_s)
    if isinstance(parent, RigidBody6DofReachabilityState):
        native = RigidBody6DofState(
            parent.native.time,
            parent.native.position,
            FrameVector3(ContractVector3(*velocity), Frame.ECIC),
            parent.native.attitude,
            parent.native.body_rate,
            parent.native.mass,
            parent.native.propellant_mass,
            parent.native.heat_load,
            parent.native.peak_heat_rate,
        )
        return RigidBody6DofReachabilityState(native, parent.phase)
    return PointMass3DofState(parent.time_s, parent.position_m, velocity, parent.mass_kg, parent.phase)


def _detached_initial_state(body: DetachedBodyDefinition, parent: State, fidelity: ReachabilityFidelity, vehicle: RocketGlideVehicle) -> State:
    """Map a parent release state into a detached reduced-order model."""

    impulse = _separation_impulse_inertial(vehicle, parent)
    child_delta = impulse.scaled(-1.0 / body.mass_kg)
    child_velocity = _add(parent.velocity_m_s, (child_delta.x, child_delta.y, child_delta.z))
    if fidelity is ReachabilityFidelity.RIGID_BODY_6DOF:
        angular_rate = (
            ContractVector3(0.0, 0.0, 0.0)
            if body.tumbling_policy is TumblingPolicy.FIXED_ATTITUDE
            else body.initial_angular_rate_body_rad_s
        )
        native = RigidBody6DofState(
            parent.time_s,
            FrameVector3(ContractVector3(*parent.position_m), Frame.ECIC),
            FrameVector3(ContractVector3(*child_velocity), Frame.ECIC),
            _parent_attitude(parent),
            angular_rate,
            body.mass_kg,
            0.0,
        )
        return RigidBody6DofReachabilityState(native)
    if fidelity in {ReachabilityFidelity.PSEUDO_6DOF, ReachabilityFidelity.RIGID_BODY_6DOF}:
        attitude = parent.attitude_rad if isinstance(parent, Pseudo6DofState) else (0.0, 0.0, 0.0)
        angular_rate = (
            (0.0, 0.0, 0.0)
            if body.tumbling_policy is TumblingPolicy.FIXED_ATTITUDE
            else (
                body.initial_angular_rate_body_rad_s.x,
                body.initial_angular_rate_body_rad_s.y,
                body.initial_angular_rate_body_rad_s.z,
            )
        )
        return Pseudo6DofState(
            parent.time_s,
            parent.position_m,
            child_velocity,
            body.mass_kg,
            "ballistic",
            attitude,
            angular_rate,
        )
    return PointMass3DofState(parent.time_s, parent.position_m, child_velocity, body.mass_kg, "ballistic")
    ####


def _body_telemetry(vehicle: RocketGlideVehicle, body: DetachedBodyDefinition, state: State, termination: EnvelopeTermination | None = None) -> dict[str, object]:
    """Record accepted detached-body aero and attitude observables."""

    speed = _norm(state.velocity_m_s)
    density = _atmosphere(vehicle, state.position_m[2])
    area = _projected_area(body, state)
    drag_force_n = 0.5 * density * speed**2 * area * vehicle.drag_coefficient
    payload: dict[str, object] = {
        "time_s": state.time_s,
        "position_m": list(state.position_m),
        "velocity_m_s": list(state.velocity_m_s),
        "mass_kg": state.mass_kg,
        "projected_area_m2": area,
        "drag_force_n": drag_force_n,
        "event_id": "booster-release",
        "termination": None if termination is None else termination.value,
    }
    if isinstance(state, Pseudo6DofState):
        payload["attitude_rad"] = list(state.attitude_rad)
        payload["attitude_rate_rad_s"] = list(state.attitude_rate_rad_s)
    elif isinstance(state, RigidBody6DofReachabilityState):
        attitude = state.native.attitude
        payload["attitude_quaternion"] = [attitude.w, attitude.x, attitude.y, attitude.z]
        payload["attitude_rate_rad_s"] = [state.native.body_rate.x, state.native.body_rate.y, state.native.body_rate.z]
        payload.update(_rigid_body_model(vehicle, body).observables(state.native))
    return payload
    ####


def _parent_telemetry(vehicle: RocketGlideVehicle, state: State, termination: EnvelopeTermination | None = None) -> dict[str, object]:
    """Record accepted parent aero observables for the envelope artifact."""

    speed = _norm(state.velocity_m_s)
    density = _atmosphere(vehicle, state.position_m[2])
    drag_force_n = 0.5 * density * speed**2 * vehicle.reference_area_m2 * vehicle.drag_coefficient
    payload: dict[str, object] = {
        "time_s": state.time_s,
        "position_m": list(state.position_m),
        "velocity_m_s": list(state.velocity_m_s),
        "mass_kg": state.mass_kg,
        "projected_area_m2": vehicle.reference_area_m2,
        "drag_force_n": drag_force_n,
        "event_id": "booster-release" if vehicle.has_booster and state.time_s >= vehicle.booster_release_time_s else None,
        "termination": None if termination is None else termination.value,
    }
    if isinstance(state, Pseudo6DofState):
        payload["attitude_rad"] = list(state.attitude_rad)
        payload["attitude_rate_rad_s"] = list(state.attitude_rate_rad_s)
    return payload
    ####


def _simulate_detached_body(
    vehicle: RocketGlideVehicle,
    parent: State,
    *,
    fidelity: ReachabilityFidelity,
    step_size_s: float,
    horizon_s: float,
) -> DetachedBodyTrajectory:
    """Propagate one typed detached body from the accepted release state."""

    assert vehicle.booster_detached_body is not None
    body = vehicle.booster_detached_body
    state = _detached_initial_state(body, parent, fidelity, vehicle)
    states: list[State] = [state]
    termination = EnvelopeTermination.HORIZON
    while state.time_s < horizon_s - 1.0e-12:
        step = min(step_size_s, horizon_s - state.time_s)
        state = _ballistic_rk4_step(vehicle, state, step)
        states.append(state)
        if state.time_s > parent.time_s and state.position_m[2] <= 0.0:
            termination = EnvelopeTermination.GROUND_CONTACT
            break
        if not all(math.isfinite(value) for value in (*state.position_m, *state.velocity_m_s, state.mass_kg)):
            termination = EnvelopeTermination.INVALID
            break
    telemetry = [_body_telemetry(vehicle, body, accepted) for accepted in states]
    if telemetry:
        telemetry[-1]["termination"] = termination.value
    return DetachedBodyTrajectory(
        body.body_id,
        body.shape.value,
        "booster-release",
        parent.time_s,
        fidelity,
        tuple(states),
        termination,
        tuple(telemetry),
    )
    ####


def simulate_rocket_glide(
    vehicle: RocketGlideVehicle,
    command: LaunchCommand,
    *,
    fidelity: ReachabilityFidelity = ReachabilityFidelity.POINT_MASS_3DOF,
    step_size_s: float = 0.25,
    horizon_s: float = 120.0,
    spawn_children: bool = False,
) -> TrajectoryResult:
    """Propagate one rocket/glider witness with fixed-step RK4."""

    if step_size_s <= 0.0 or horizon_s <= 0.0:
        raise ValueError("step_size_s and horizon_s must be positive")
    state = _launch_state(vehicle, command, fidelity)
    states: list[State] = [state]
    spawned_bodies: list[DetachedBodyTrajectory] = []
    deployment_events: list[dict[str, object]] = []
    termination = EnvelopeTermination.HORIZON
    while state.time_s < horizon_s - 1.0e-12:
        step = min(step_size_s, horizon_s - state.time_s)
        boundaries = [
            boundary
            for boundary in (
                vehicle.booster_burn_time_s if vehicle.has_booster else vehicle.burn_time_s,
                vehicle.booster_release_time_s if vehicle.has_booster else None,
            )
            if boundary is not None and state.time_s < boundary < state.time_s + step
        ]
        if boundaries:
            step = min(boundaries) - state.time_s
        next_state = _rk4_step(vehicle, command, state, step, fidelity)
        states.append(next_state)
        state = next_state
        if (
            spawn_children
            and vehicle.booster_detached_body is not None
            and vehicle.has_booster
            and math.isclose(state.time_s, vehicle.booster_release_time_s, abs_tol=1.0e-9)
            and not spawned_bodies
        ):
            parent_before_separation = state
            state = _apply_parent_separation_delta_v(vehicle, state)
            states[-1] = state
            child = _simulate_detached_body(vehicle, state, fidelity=fidelity, step_size_s=step_size_s, horizon_s=horizon_s)
            spawned_bodies.append(child)
            parent_momentum_after = _scale(state.velocity_m_s, state.mass_kg)
            child_momentum_after = _scale(child.states[0].velocity_m_s, child.states[0].mass_kg)
            parent_momentum_before = _scale(parent_before_separation.velocity_m_s, state.mass_kg)
            child_momentum_before = _scale(parent_before_separation.velocity_m_s, child.states[0].mass_kg)
            momentum_residual = _norm(_add(_add(parent_momentum_after, child_momentum_after), _scale(_add(parent_momentum_before, child_momentum_before), -1.0)))
            deployment_events.append(
                {
                    "event_id": "booster-release",
                    "accepted_time_s": state.time_s,
                    "parent_model_id": vehicle.vehicle_id,
                    "child_model_id": child.body_id,
                    "parent_mass_before_kg": parent_before_separation.mass_kg,
                    "parent_mass_after_kg": state.mass_kg,
                    "child_mass_kg": child.terminal.mass_kg,
                    "parent_velocity_before_m_s": list(parent_before_separation.velocity_m_s),
                    "parent_velocity_after_m_s": list(state.velocity_m_s),
                    "child_initial_velocity_m_s": list(child.states[0].velocity_m_s),
                    "momentum_residual_kg_m_s": momentum_residual,
                    "shape": child.shape,
                    "impulse_frame": vehicle.booster_separation_impulse_frame.value,
                    "retained_impulse_n_s": [
                        vehicle.booster_separation_impulse_n_s.x,
                        vehicle.booster_separation_impulse_n_s.y,
                        vehicle.booster_separation_impulse_n_s.z,
                    ],
                    "status": "committed",
                }
            )
        if state.time_s > 0.0 and state.position_m[2] <= 0.0:
            termination = EnvelopeTermination.GROUND_CONTACT
            break
        if not all(math.isfinite(value) for value in (*state.position_m, *state.velocity_m_s, state.mass_kg)):
            termination = EnvelopeTermination.INVALID
            break
    parent_telemetry = [_parent_telemetry(vehicle, accepted) for accepted in states]
    if parent_telemetry:
        parent_telemetry[-1]["termination"] = termination.value
    return TrajectoryResult(
        fidelity,
        command,
        tuple(states),
        termination,
        tuple(spawned_bodies),
        tuple(deployment_events),
        tuple(parent_telemetry),
    )
    ####


def _state_fields(fidelity: ReachabilityFidelity) -> tuple[str, ...]:
    fields = (
        "time_s",
        "x_m",
        "y_m",
        "z_m",
        "vx_m_s",
        "vy_m_s",
        "vz_m_s",
        "speed_m_s",
        "mass_kg",
        "phase",
    )
    if fidelity is ReachabilityFidelity.PSEUDO_6DOF:
        return fields + (
            "roll_rad",
            "pitch_rad",
            "yaw_rad",
            "roll_rate_rad_s",
            "pitch_rate_rad_s",
            "yaw_rate_rad_s",
        )
    if fidelity is ReachabilityFidelity.RIGID_BODY_6DOF:
        return fields + (
            "qw",
            "qx",
            "qy",
            "qz",
            "wx_rad_s",
            "wy_rad_s",
            "wz_rad_s",
        )
    return fields
    ####


def _vehicle_parameters(vehicle: RocketGlideVehicle) -> dict[str, object]:
    """Return replayable primitive vehicle parameters for an envelope artifact."""

    parameters = {item.name: getattr(vehicle, item.name) for item in fields(RocketGlideVehicle)}
    if vehicle.booster_detached_body is not None:
        body = vehicle.booster_detached_body
        parameters["booster_detached_body"] = {
            "body_id": body.body_id,
            "mass_kg": body.mass_kg,
            "shape": body.shape.value,
            "dimensions_m": list(body.dimensions_m),
            "reference_area_m2": body.reference_area_m2,
            "tumbling_policy": body.tumbling_policy.value,
            "inertia_kg_m2": None if body.inertia_kg_m2 is None else [body.inertia_kg_m2.x, body.inertia_kg_m2.y, body.inertia_kg_m2.z],
            "initial_angular_rate_body_rad_s": [
                body.initial_angular_rate_body_rad_s.x,
                body.initial_angular_rate_body_rad_s.y,
                body.initial_angular_rate_body_rad_s.z,
            ],
            "center_of_mass_m": [body.center_of_mass_m.x, body.center_of_mass_m.y, body.center_of_mass_m.z],
            "center_of_pressure_m": [body.center_of_pressure_m.x, body.center_of_pressure_m.y, body.center_of_pressure_m.z],
        }
    impulse = parameters.get("booster_separation_impulse_n_s")
    if isinstance(impulse, ContractVector3):
        parameters["booster_separation_impulse_n_s"] = [impulse.x, impulse.y, impulse.z]
    if isinstance(parameters.get("booster_separation_impulse_frame"), ImpulseFrame):
        parameters["booster_separation_impulse_frame"] = parameters["booster_separation_impulse_frame"].value
    return parameters
    ####


def _detached_body_from_dict(payload: Mapping[str, object]) -> DetachedBodyDefinition:
    """Reconstruct a serialized detached-body contract for timeout reruns."""

    dimensions = payload.get("dimensions_m")
    if not isinstance(dimensions, list):
        raise ValueError("serialized detached body dimensions_m must be a list")
    inertia_payload = payload.get("inertia_kg_m2")
    inertia = None
    if isinstance(inertia_payload, list):
        if len(inertia_payload) != 3:
            raise ValueError("serialized detached body inertia must have three components")
        inertia = ContractVector3(*(float(value) for value in inertia_payload))
    angular_payload = payload.get("initial_angular_rate_body_rad_s", [0.0, 0.0, 0.0])
    if not isinstance(angular_payload, list) or len(angular_payload) != 3:
        raise ValueError("serialized detached body angular rate must have three components")
    center_of_mass_payload = payload.get("center_of_mass_m", [0.0, 0.0, 0.0])
    center_of_pressure_payload = payload.get("center_of_pressure_m", [0.0, 0.0, 0.0])
    if not isinstance(center_of_mass_payload, list) or len(center_of_mass_payload) != 3:
        raise ValueError("serialized detached body center_of_mass_m must have three components")
    if not isinstance(center_of_pressure_payload, list) or len(center_of_pressure_payload) != 3:
        raise ValueError("serialized detached body center_of_pressure_m must have three components")
    return DetachedBodyDefinition(
        str(payload["body_id"]),
        float(payload["mass_kg"]),
        DetachedBodyShape(str(payload["shape"])),
        tuple(float(value) for value in dimensions),
        float(payload["reference_area_m2"]),
        TumblingPolicy(str(payload.get("tumbling_policy", TumblingPolicy.FIXED_ATTITUDE.value))),
        inertia,
        ContractVector3(*(float(value) for value in angular_payload)),
        ContractVector3(*(float(value) for value in center_of_mass_payload)),
        ContractVector3(*(float(value) for value in center_of_pressure_payload)),
    )
    ####


def _path_metrics(trajectory: TrajectoryResult) -> tuple[tuple[str, float], ...]:
    states = trajectory.states
    phase_durations: dict[str, float] = {}
    for previous, current in zip(states[:-1], states[1:], strict=True):
        phase_durations[previous.phase] = phase_durations.get(previous.phase, 0.0) + current.time_s - previous.time_s
    specific_energies = [0.5 * state.speed_m_s**2 + 9.80665 * state.position_m[2] for state in states]
    metrics = {
        "duration_s": trajectory.terminal.time_s,
        "minimum_altitude_m": min(state.position_m[2] for state in states),
        "maximum_altitude_m": max(state.position_m[2] for state in states),
        "maximum_speed_m_s": max(state.speed_m_s for state in states),
        "minimum_mass_kg": min(state.mass_kg for state in states),
        "initial_specific_energy_m2_s2": specific_energies[0],
        "maximum_specific_energy_m2_s2": max(specific_energies),
        "terminal_specific_energy_m2_s2": specific_energies[-1],
        **{f"phase_duration_{phase}_s": duration for phase, duration in phase_durations.items()},
    }
    return tuple(sorted(metrics.items()))
    ####


def _evaluate(criteria: TerminalCriteria, trajectory: TrajectoryResult) -> _EnvelopeEvaluation:
    speed = trajectory.terminal.speed_m_s
    impact_speed = speed
    target_x, target_y, _ = criteria.target_position_m
    terminal_x, terminal_y, _ = trajectory.terminal.position_m
    impact_radius = math.hypot(terminal_x - target_x, terminal_y - target_y)
    impact_occurred = trajectory.termination is EnvelopeTermination.GROUND_CONTACT
    reasons: list[str] = []
    if trajectory.termination is EnvelopeTermination.INVALID:
        reasons.append("invalid_state")
    if criteria.require_ground_contact and trajectory.termination is not EnvelopeTermination.GROUND_CONTACT:
        reasons.append("terminal_event_not_reached")
    if speed < criteria.min_speed_m_s:
        reasons.append("terminal_speed_low")
    if speed > criteria.max_speed_m_s:
        reasons.append("terminal_speed_high")
    impact_constraints = (
        criteria.max_impact_radius_m is not None
        or criteria.min_impact_speed_m_s is not None
        or criteria.max_impact_speed_m_s is not None
    )
    if impact_constraints and not impact_occurred:
        reasons.append("impact_event_not_reached")
    if impact_occurred and criteria.max_impact_radius_m is not None and impact_radius > criteria.max_impact_radius_m:
        reasons.append("impact_radius_high")
    if impact_occurred and criteria.min_impact_speed_m_s is not None and impact_speed < criteria.min_impact_speed_m_s:
        reasons.append("impact_speed_low")
    if impact_occurred and criteria.max_impact_speed_m_s is not None and impact_speed > criteria.max_impact_speed_m_s:
        reasons.append("impact_speed_high")
    margins: tuple[tuple[str, float | None], ...] = (
        ("terminal_speed_min_margin_m_s", speed - criteria.min_speed_m_s),
        (
            "terminal_speed_max_margin_m_s",
            None if math.isinf(criteria.max_speed_m_s) else criteria.max_speed_m_s - speed,
        ),
        ("ground_contact_satisfied", 1.0 if impact_occurred else 0.0),
        ("impact_radius_margin_m", None if criteria.max_impact_radius_m is None else criteria.max_impact_radius_m - impact_radius),
        ("impact_speed_min_margin_m_s", None if criteria.min_impact_speed_m_s is None or not impact_occurred else impact_speed - criteria.min_impact_speed_m_s),
        ("impact_speed_max_margin_m_s", None if criteria.max_impact_speed_m_s is None or not impact_occurred else criteria.max_impact_speed_m_s - impact_speed),
    )
    classification = "invalid" if trajectory.termination is EnvelopeTermination.INVALID else "infeasible" if reasons else "feasible"
    return _EnvelopeEvaluation(
        feasible=not reasons,
        classification=classification,
        failure_reasons=tuple(reasons),
        limiting_factor=reasons[0] if reasons else None,
        terminal_margins=margins,
        path_metrics=_path_metrics(trajectory),
    )
    ####


@dataclass(frozen=True, slots=True)
class _EnvelopeJob:
    index: int
    vehicle: RocketGlideVehicle
    command: LaunchCommand
    fidelity: ReachabilityFidelity
    step_size_s: float
    horizon_s: float
    criteria: TerminalCriteria
    spawn_children: bool
    ####


def _run_envelope_job(job: _EnvelopeJob) -> EnvelopeSample:
    trajectory = simulate_rocket_glide(
        job.vehicle,
        job.command,
        fidelity=job.fidelity,
        step_size_s=job.step_size_s,
        horizon_s=job.horizon_s,
        spawn_children=job.spawn_children,
    )
    evaluation = _evaluate(job.criteria, trajectory)
    return EnvelopeSample(
        job.index,
        job.command,
        trajectory,
        evaluation.feasible,
        evaluation.limiting_factor,
        evaluation.classification,
        evaluation.failure_reasons,
        evaluation.terminal_margins,
        evaluation.path_metrics,
    )
    ####


def run_reachability_envelope(
    vehicle: RocketGlideVehicle,
    commands: Iterable[LaunchCommand],
    *,
    fidelity: ReachabilityFidelity = ReachabilityFidelity.POINT_MASS_3DOF,
    step_size_s: float = 0.25,
    horizon_s: float = 120.0,
    criteria: TerminalCriteria | None = None,
    workers: int = 1,
    search_space: ReachabilitySearchSpace | None = None,
    study_id: str = "alpha3_rocket_glide_terminal_footprint",
    provenance: Mapping[str, object] | None = None,
    spawn_children: bool = False,
) -> ReachabilityEnvelope:
    """Run a deterministic reachability batch with optional deployment composition."""

    if workers <= 0:
        raise ValueError("workers must be positive")
    command_list = tuple(commands)
    if not command_list:
        raise ValueError("commands must contain at least one launch command")
    selected_criteria = criteria or TerminalCriteria()
    jobs = tuple(
        _EnvelopeJob(index, vehicle, command, fidelity, step_size_s, horizon_s, selected_criteria, spawn_children)
        for index, command in enumerate(command_list)
    )
    if workers == 1:
        samples = tuple(_run_envelope_job(job) for job in jobs)
    else:
        context = multiprocessing.get_context("spawn")
        with ProcessPoolExecutor(max_workers=workers, mp_context=context) as executor:
            samples = tuple(executor.map(_run_envelope_job, jobs))
    return ReachabilityEnvelope(
        vehicle_id=vehicle.vehicle_id,
        fidelity=fidelity,
        samples=samples,
        workers=workers,
        step_size_s=step_size_s,
        horizon_s=horizon_s,
        study_id=study_id,
        search_space=search_space or _infer_search_space(command_list),
        terminal_criteria=selected_criteria,
        vehicle_parameters=tuple(_vehicle_parameters(vehicle).items()),
        state_fields=_state_fields(fidelity),
        provenance=tuple(sorted((provenance or {}).items())),
        spawn_children=spawn_children,
    )
    ####


def rerun_timed_out_envelope(
    vehicle: RocketGlideVehicle,
    previous: ReachabilityEnvelope,
    *,
    horizon_s: float,
    step_size_s: float | None = None,
    workers: int | None = None,
    criteria: TerminalCriteria | None = None,
) -> ReachabilityEnvelope:
    """Rerun only horizon-terminated candidates with a longer horizon."""

    timed_out = previous.timed_out_samples
    if not timed_out:
        raise ValueError("envelope contains no timed-out candidates")
    return run_reachability_envelope(
        vehicle,
        tuple(sample.command for sample in timed_out),
        fidelity=previous.fidelity,
        step_size_s=previous.step_size_s if step_size_s is None else step_size_s,
        horizon_s=horizon_s,
        criteria=previous.terminal_criteria if criteria is None else criteria,
        workers=previous.workers if workers is None else workers,
        study_id=f"{previous.study_id}_timeout_rerun",
        provenance={
            **dict(previous.provenance),
            "rerun_parent_study_id": previous.study_id,
            "rerun_reason": "timed_out_candidates",
            "rerun_parent_query_ids": list(previous.timed_out_query_ids),
            "rerun_horizon_s": horizon_s,
        },
    )


def timed_out_commands_from_artifact(payload: Mapping[str, object]) -> tuple[LaunchCommand, ...]:
    """Extract timed-out launch commands from a serialized envelope."""

    samples = payload.get("samples", [])
    if not isinstance(samples, list):
        raise ValueError("reachability artifact samples must be a list")
    commands: list[LaunchCommand] = []
    for sample in samples:
        if not isinstance(sample, Mapping) or not bool(sample.get("timed_out", sample.get("termination") == "horizon")):
            continue
        decision = sample.get("decision_vector", {})
        if not isinstance(decision, Mapping):
            raise ValueError("timed-out sample decision_vector must be an object")
        commands.append(
            LaunchCommand(
                float(decision["azimuth_rad"]),
                float(decision["elevation_rad"]),
                float(decision.get("bank_rad", 0.0)),
            )
        )
    if not commands:
        raise ValueError("reachability artifact contains no timed-out candidates")
    return tuple(commands)


def rerun_timed_out_artifact(
    payload: Mapping[str, object],
    *,
    horizon_s: float,
    step_size_s: float | None = None,
    workers: int | None = None,
) -> ReachabilityEnvelope:
    """Reconstruct a serialized study and rerun its timed-out candidates."""

    study = payload.get("study", {})
    if not isinstance(study, Mapping):
        raise ValueError("reachability artifact study must be an object")
    parameters = study.get("vehicle_parameters", {})
    if not isinstance(parameters, Mapping):
        raise ValueError("reachability artifact vehicle_parameters must be an object")
    vehicle_parameters = {str(key): value for key, value in parameters.items()}
    detached_payload = vehicle_parameters.get("booster_detached_body")
    if isinstance(detached_payload, Mapping):
        vehicle_parameters["booster_detached_body"] = _detached_body_from_dict(detached_payload)
    impulse_payload = vehicle_parameters.get("booster_separation_impulse_n_s")
    if isinstance(impulse_payload, list) and len(impulse_payload) == 3:
        vehicle_parameters["booster_separation_impulse_n_s"] = ContractVector3(*(float(value) for value in impulse_payload))
    frame_payload = vehicle_parameters.get("booster_separation_impulse_frame")
    if frame_payload is not None:
        vehicle_parameters["booster_separation_impulse_frame"] = ImpulseFrame(str(frame_payload))
    vehicle = RocketGlideVehicle(**vehicle_parameters)
    fidelity = ReachabilityFidelity(str(payload.get("fidelity", study.get("fidelity"))))
    execution = payload.get("execution", {})
    if not isinstance(execution, Mapping):
        execution = {}
    criteria_payload = payload.get("success_spec", {})
    criteria = TerminalCriteria.from_dict(criteria_payload) if isinstance(criteria_payload, Mapping) else TerminalCriteria()
    commands = timed_out_commands_from_artifact(payload)
    parent_study_id = str(study.get("study_id", "reachability_study"))
    return run_reachability_envelope(
        vehicle,
        commands,
        fidelity=fidelity,
        step_size_s=float(execution.get("step_size_s", 0.25)) if step_size_s is None else step_size_s,
        horizon_s=horizon_s,
        criteria=criteria,
        workers=int(execution.get("workers_requested", 1)) if workers is None else workers,
        study_id=f"{parent_study_id}_timeout_rerun",
        provenance={
            "rerun_parent_study_id": parent_study_id,
            "rerun_reason": "timed_out_candidates",
            "rerun_parent_query_count": len(commands),
            "rerun_horizon_s": horizon_s,
        },
        spawn_children=bool(execution.get("spawn_children", False)),
    )


def generate_launch_grid(
    azimuths_rad: Sequence[float],
    elevations_rad: Sequence[float],
    banks_rad: Sequence[float] = (0.0,),
) -> tuple[LaunchCommand, ...]:
    """Create a stable Cartesian product of launch and glide-bank choices."""

    return tuple(
        LaunchCommand(azimuth, elevation, bank)
        for azimuth in azimuths_rad
        for elevation in elevations_rad
        for bank in banks_rad
    )
    ####


def _infer_search_space(commands: tuple[LaunchCommand, ...]) -> ReachabilitySearchSpace:
    axes = (
        SearchAxis("launch.azimuth_rad", "rad", tuple(dict.fromkeys(command.azimuth_rad for command in commands))),
        SearchAxis("launch.elevation_rad", "rad", tuple(dict.fromkeys(command.elevation_rad for command in commands))),
        SearchAxis("glide.bank_rad", "rad", tuple(dict.fromkeys(command.bank_rad for command in commands))),
    )
    expected = tuple(
        (azimuth, elevation, bank)
        for azimuth in axes[0].values
        for elevation in axes[1].values
        for bank in axes[2].values
    )
    observed = tuple((command.azimuth_rad, command.elevation_rad, command.bank_rad) for command in commands)
    composition = "cartesian_product" if observed == expected else "explicit_candidate_set"
    ordering = "axis_order_then_cartesian_product" if composition == "cartesian_product" else "input_order"
    return ReachabilitySearchSpace(axes, composition, len(commands), ordering)
    ####


def _bounds_for_samples(samples: Sequence[EnvelopeSample]) -> EnvelopeBounds | None:
    if not samples:
        return None
    positions = [sample.terminal.position_m for sample in samples]
    return EnvelopeBounds(
        downrange_min_m=min(position[0] for position in positions),
        downrange_max_m=max(position[0] for position in positions),
        crossrange_min_m=min(position[1] for position in positions),
        crossrange_max_m=max(position[1] for position in positions),
        altitude_min_m=min(position[2] for position in positions),
        altitude_max_m=max(position[2] for position in positions),
    )
    ####


def _bounds_dict(bounds: EnvelopeBounds | None) -> dict[str, float] | None:
    if bounds is None:
        return None
    return {
        "downrange_min_m": bounds.downrange_min_m,
        "downrange_max_m": bounds.downrange_max_m,
        "crossrange_min_m": bounds.crossrange_min_m,
        "crossrange_max_m": bounds.crossrange_max_m,
        "altitude_min_m": bounds.altitude_min_m,
        "altitude_max_m": bounds.altitude_max_m,
    }
    ####


def _state_row(state: State, fidelity: ReachabilityFidelity | None = None) -> list[object]:
    row: list[object] = [
        state.time_s,
        *state.position_m,
        *state.velocity_m_s,
        state.speed_m_s,
        state.mass_kg,
        state.phase,
    ]
    if isinstance(state, Pseudo6DofState) and fidelity is ReachabilityFidelity.RIGID_BODY_6DOF:
        attitude = _euler_quaternion(state.attitude_rad)
        row.extend((attitude.w, attitude.x, attitude.y, attitude.z, *state.attitude_rate_rad_s))
    elif isinstance(state, Pseudo6DofState):
        row.extend((*state.attitude_rad, *state.attitude_rate_rad_s))
    elif isinstance(state, RigidBody6DofReachabilityState):
        attitude = state.native.attitude
        row.extend((attitude.w, attitude.x, attitude.y, attitude.z, state.native.body_rate.x, state.native.body_rate.y, state.native.body_rate.z))
    return row
    ####


def _terminal_state_dict(state: State) -> dict[str, object]:
    payload: dict[str, object] = {
        "time_s": state.time_s,
        "position_m": list(state.position_m),
        "velocity_m_s": list(state.velocity_m_s),
        "speed_m_s": state.speed_m_s,
        "mass_kg": state.mass_kg,
        "phase": state.phase,
    }
    if isinstance(state, Pseudo6DofState):
        payload["attitude_rad"] = list(state.attitude_rad)
        payload["attitude_rate_rad_s"] = list(state.attitude_rate_rad_s)
    elif isinstance(state, RigidBody6DofReachabilityState):
        attitude = state.native.attitude
        payload["attitude_quaternion"] = [attitude.w, attitude.x, attitude.y, attitude.z]
        payload["body_rate_rad_s"] = [state.native.body_rate.x, state.native.body_rate.y, state.native.body_rate.z]
    return payload
    ####


def _sample_dict(sample: EnvelopeSample, state_fields: tuple[str, ...], include_trajectory: bool) -> dict[str, object]:
    payload: dict[str, object] = {
        "query_id": sample.query_id,
        "index": sample.index,
        "search_coordinates": {
            "launch.azimuth_rad": sample.command.azimuth_rad,
            "launch.elevation_rad": sample.command.elevation_rad,
            "glide.bank_rad": sample.command.bank_rad,
        },
        "decision_vector": {
            "azimuth_rad": sample.command.azimuth_rad,
            "elevation_rad": sample.command.elevation_rad,
            "bank_rad": sample.command.bank_rad,
        },
        "success": sample.feasible,
        "feasible": sample.feasible,
        "classification": sample.classification,
        "timed_out": sample.timed_out,
        "failure_reasons": list(sample.failure_reasons),
        "limiting_factor": sample.limiting_factor,
        "terminal_margins": dict(sample.terminal_margins),
        "path_metrics": dict(sample.path_metrics),
        "termination": sample.trajectory.termination.value,
        "terminal_state": _terminal_state_dict(sample.terminal),
        "witness": {
            "available": sample.feasible,
            "trajectory_included": include_trajectory,
        },
        "terminal_position_m": list(sample.terminal.position_m),
        "terminal_velocity_m_s": list(sample.terminal.velocity_m_s),
        "terminal_speed_m_s": sample.terminal.speed_m_s,
        "deployment_events": [dict(event) for event in sample.trajectory.deployment_events],
        "spawned_body_ids": [body.body_id for body in sample.trajectory.spawned_bodies],
    }
    if include_trajectory:
        payload["trajectory"] = {
            "fields": list(state_fields),
            "rows": [_state_row(state, sample.trajectory.fidelity) for state in sample.trajectory.states],
        }
        payload["telemetry"] = [dict(row) for row in sample.trajectory.telemetry]
        payload["spawned_bodies"] = [
            {
                "body_id": body.body_id,
                "shape": body.shape,
                "parent_event_id": body.parent_event_id,
                "deployment_time_s": body.deployment_time_s,
                "fidelity": body.fidelity.value,
                "termination": body.termination.value,
                "classification": body.classification,
                "terminal_state": _terminal_state_dict(body.terminal),
                "telemetry": [dict(row) for row in body.telemetry],
                "trajectory": {
                    "fields": list(state_fields),
                    "rows": [_state_row(state, body.fidelity) for state in body.states],
                },
            }
            for body in sample.trajectory.spawned_bodies
        ]
    return payload
    ####


def _trajectory_dict(trajectory: TrajectoryResult) -> list[dict[str, object]]:
    return [
        {
            "time_s": state.time_s,
            "position_m": state.position_m,
            "velocity_m_s": state.velocity_m_s,
            "mass_kg": state.mass_kg,
            "phase": state.phase,
            **(
                {
                    "attitude_rad": state.attitude_rad,
                    "attitude_rate_rad_s": state.attitude_rate_rad_s,
                }
                if isinstance(state, Pseudo6DofState)
                else {}
            ),
        }
        for state in trajectory.states
    ]
    ####


__all__ = [
    "DetachedBodyTrajectory",
    "EnvelopeBounds",
    "EnvelopeSample",
    "EnvelopeTermination",
    "LaunchCommand",
    "PointMass3DofState",
    "Pseudo6DofState",
    "ReachabilityEnvelope",
    "ReachabilityFidelity",
    "ReachabilitySearchSpace",
    "RocketGlideVehicle",
    "RocketStageSpec",
    "SearchAxis",
    "StageSeparationSpec",
    "StagedRocketSpec",
    "TerminalCriteria",
    "TrajectoryResult",
    "generate_launch_grid",
    "rerun_timed_out_artifact",
    "rerun_timed_out_envelope",
    "run_reachability_envelope",
    "simulate_rocket_glide",
    "timed_out_commands_from_artifact",
]
####
