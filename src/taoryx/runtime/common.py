"""Shared state and callback contracts for the TAOS execution layer."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from copy import copy, deepcopy
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal, cast

from taoryx.contracts import Frame, Vector3
from taoryx.language.expressions import ExpressionType
from taoryx.modes import DynamicsMode, FidelitySetupError, Kinematic6DofState
from taoryx.outputs import VehicleKind
from taoryx.sensors import TruthPoint
from taoryx.state import PointMassRates, PointMassState

from .sensor_clock import SensorClockSpec

if TYPE_CHECKING:
    from .sensor_bus import SensorBus


@dataclass(frozen=True, slots=True)
class RuntimeState:
    """A named, time-tagged state with an optional frame-aware position."""

    time: float
    values: tuple[float, ...]
    frame: Frame | str = Frame.ECFC
    named: Mapping[str, float] = field(default_factory=dict)
    value_names: tuple[str, ...] = ()
    segment_endpoints: Mapping[int, RuntimeState] = field(default_factory=dict)

    def with_values(self, values: Sequence[float], *, time: float | None = None) -> RuntimeState:
        normalized = tuple(float(v) for v in values)
        named = dict(self.named)
        for name, value in zip(self.value_names, normalized, strict=False):
            named[name] = value
        resolved_time = self.time if time is None else time
        named["time"] = resolved_time
        return RuntimeState(resolved_time, normalized, self.frame, named, self.value_names, self.segment_endpoints)
    ####

    def interpolated(self, other: RuntimeState, time: float) -> RuntimeState:
        """Linearly interpolate this state and its numeric named observables."""

        if other.time == self.time:
            if time != self.time:
                raise ValueError("cannot interpolate distinct times from a zero-duration state")
            return self
        fraction = (time - self.time) / (other.time - self.time)
        if fraction < -1e-12 or fraction > 1.0 + 1e-12:
            raise ValueError("interpolation time lies outside the state interval")
        fraction = min(1.0, max(0.0, fraction))
        values = tuple(left + fraction * (right - left) for left, right in zip(self.values, other.values, strict=True))
        named = dict(self.named)
        for name in set(self.named) | set(other.named):
            left = self.named.get(name)
            right = other.named.get(name)
            if isinstance(left, (int, float)) and isinstance(right, (int, float)):
                named[name] = float(left) + fraction * (float(right) - float(left))
            elif right is not None:
                named[name] = right
        return RuntimeState(time, values, self.frame, named, self.value_names, self.segment_endpoints)
    ####

    def to_point_mass_state(self) -> PointMassState:
        """Convert the canonical nine-value runtime state to physical state.

        Runtime lowering may carry many named derived values, but only the
        canonical TAOS integration names cross into the point-mass kernel.
        """

        expected = PointMassState.STATE_NAMES
        if self.value_names and self.value_names != expected:
            raise ValueError("runtime state names do not match the canonical TAOS point-mass order")
        return PointMassState.from_values(self.time, list(self.values))
        ####

    @classmethod
    def from_point_mass_state(cls, state: PointMassState) -> RuntimeState:
        """Wrap a validated physical state for runtime scheduling and output."""

        values = state.to_values()
        named = dict(zip(PointMassState.STATE_NAMES, values, strict=True))
        named["time"] = state.time
        return cls(state.time, values, Frame.ECFC, named, PointMassState.STATE_NAMES)
        ####
####


Derivative = Callable[[RuntimeState], Sequence[float]]
PointMassDerivative = Callable[[PointMassState], PointMassRates]
BodyRateProvider = Callable[[RuntimeState], Vector3]
KinematicAttitudeTargetProvider = Callable[[RuntimeState], Vector3]
CommittedControlResolver = Callable[[RuntimeState], Mapping[str, float]]
StallDetector = Callable[[RuntimeState], bool]
SpawnProvider = Callable[[RuntimeState], Sequence["SpawnRequest"]]
TruthProvider = Callable[[RuntimeState], TruthPoint]
LoadEvaluationPhase = Literal[
    "solver_stage_environment",
    "solver_stage_rhs",
    "committed_truth_environment",
    "committed_truth_rhs",
]
ControlEvaluationPhase = Literal["solver_stage", "committed_truth"]


@dataclass(frozen=True, slots=True)
class LoadEvaluationRecord:
    """Time provenance for one runtime environment or RHS load evaluation.

    Solver-stage evaluations are retained solely as computational provenance;
    they never become sensor, controller, or telemetry truth. The separate
    achieved-control timestamp identifies the accepted boundary at which the
    currently applied command set became effective.
    """

    state_time_s: float
    achieved_control_time_s: float
    phase: LoadEvaluationPhase

    def __post_init__(self) -> None:
        if not math.isfinite(self.state_time_s) or not math.isfinite(self.achieved_control_time_s):
            raise ValueError("load-evaluation state and achieved-control times must be finite")
        if self.achieved_control_time_s > self.state_time_s + 1.0e-12:
            raise ValueError("load-evaluation achieved-control time cannot follow its evaluated state")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return an auditable, JSON-safe timing record."""

        return {
            "state_time_s": self.state_time_s,
            "achieved_control_time_s": self.achieved_control_time_s,
            "phase": self.phase,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class ControlEvaluationRecord:
    """Private control-vector provenance at one runtime evaluation point.

    This is deliberately distinct from a public semantic action trace. Solver
    stages may calculate a controller output while probing an uncommitted state;
    such values are computational provenance and cannot be presented as a
    command held over a truth interval.
    """

    state_time_s: float
    control_activation_time_s: float
    phase: ControlEvaluationPhase
    controls: tuple[tuple[str, float], ...]

    def __post_init__(self) -> None:
        if not math.isfinite(self.state_time_s) or not math.isfinite(self.control_activation_time_s):
            raise ValueError("control-evaluation times must be finite")
        if self.control_activation_time_s > self.state_time_s + 1.0e-12:
            raise ValueError("control activation time cannot follow its evaluated state")
        names = tuple(name for name, _ in self.controls)
        if names != tuple(sorted(names)) or len(set(names)) != len(names):
            raise ValueError("control-evaluation controls must be uniquely sorted")
        if any(not math.isfinite(value) for _, value in self.controls):
            raise ValueError("control-evaluation values must be finite")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return JSON-safe private provenance."""

        return {
            "state_time_s": self.state_time_s,
            "control_activation_time_s": self.control_activation_time_s,
            "phase": self.phase,
            "controls": dict(self.controls),
        }
        ####

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> ControlEvaluationRecord:
        """Restore one serialized private provenance sample."""

        controls = payload.get("controls")
        if not isinstance(controls, Mapping):
            raise ValueError("control-evaluation controls must be a mapping")
        return cls(
            float(cast(float | int | str, payload["state_time_s"])),
            float(cast(float | int | str, payload["control_activation_time_s"])),
            cast(ControlEvaluationPhase, str(payload["phase"])),
            tuple(sorted((str(name), float(cast(float | int | str, value))) for name, value in controls.items())),
        )
        ####
    ####


@dataclass(frozen=True, slots=True)
class ControlIntervalRecord:
    """Accepted interval control provenance, without a held-command claim.

    ``controls_at_interval_start`` is the exact control mapping visible when
    integration began. ``solver_stage_control_mutation_detected`` records that
    a private stage evaluation changed the mutable native control mapping. In
    that case the record is expressly ineligible for Mission Composition's held-command
    action-trace artifact.
    """

    interval_start_time_s: float
    committed_truth_time_s: float
    control_activation_time_s: float
    controls_at_interval_start: tuple[tuple[str, float], ...]
    solver_stage_control_mutation_detected: bool

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(value)
            for value in (
                self.interval_start_time_s,
                self.committed_truth_time_s,
                self.control_activation_time_s,
            )
        ):
            raise ValueError("control-interval times must be finite")
        if self.interval_start_time_s > self.committed_truth_time_s:
            raise ValueError("control interval starts after its committed truth boundary")
        if self.control_activation_time_s > self.interval_start_time_s + 1.0e-12:
            raise ValueError("control activation time cannot follow the interval start")
        names = tuple(name for name, _ in self.controls_at_interval_start)
        if names != tuple(sorted(names)) or len(set(names)) != len(names):
            raise ValueError("control-interval controls must be uniquely sorted")
        if any(not math.isfinite(value) for _, value in self.controls_at_interval_start):
            raise ValueError("control-interval values must be finite")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return JSON-safe accepted-interval provenance."""

        return {
            "interval_start_time_s": self.interval_start_time_s,
            "committed_truth_time_s": self.committed_truth_time_s,
            "control_activation_time_s": self.control_activation_time_s,
            "controls_at_interval_start": dict(self.controls_at_interval_start),
            "solver_stage_control_mutation_detected": self.solver_stage_control_mutation_detected,
            "claim_boundary": (
                "This is accepted-interval control provenance. It is not a held semantic action trace when "
                "solver-stage control mutation is detected."
            ),
        }
        ####

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> ControlIntervalRecord:
        """Restore one serialized accepted-interval provenance record."""

        controls = payload.get("controls_at_interval_start")
        if not isinstance(controls, Mapping):
            raise ValueError("control-interval controls must be a mapping")
        return cls(
            float(cast(float | int | str, payload["interval_start_time_s"])),
            float(cast(float | int | str, payload["committed_truth_time_s"])),
            float(cast(float | int | str, payload["control_activation_time_s"])),
            tuple(sorted((str(name), float(cast(float | int | str, value))) for name, value in controls.items())),
            bool(payload["solver_stage_control_mutation_detected"]),
        )
        ####
    ####


@dataclass(frozen=True, slots=True)
class TransitionTruthSnapshot:
    """Immutable truth and achieved-control state at a segment transition."""

    state: RuntimeState
    segment_number: int
    active: bool
    achieved_controls: tuple[tuple[str, float], ...] = ()

    def to_metadata(self) -> dict[str, object]:
        """Return a JSON-safe representation for reports and checkpoints."""

        return {
            "time": self.state.time,
            "values": list(self.state.values),
            "frame": getattr(self.state.frame, "value", str(self.state.frame)),
            "named": dict(self.state.named),
            "value_names": list(self.state.value_names),
            "segment": self.segment_number,
            "active": self.active,
            "achieved_controls": dict(self.achieved_controls),
        }
    ####
####


@dataclass(frozen=True, slots=True)
class TransitionTruthPair:
    """The committed pre/post truth pair for one event or segment transition."""

    event_name: str
    event_time: float
    action: str
    signal: str
    segment_from: int
    segment_to: int
    pre: TransitionTruthSnapshot
    post: TransitionTruthSnapshot
    residual: float
    source: str | None = None
    state_discontinuity: bool = False

    def to_metadata(self) -> dict[str, object]:
        """Return a report-friendly representation with both truth snapshots."""

        return {
            "name": self.event_name,
            "time": self.event_time,
            "action": self.action,
            "signal": self.signal,
            "segment_from": self.segment_from,
            "segment_to": self.segment_to,
            "residual": self.residual,
            "source": self.source,
            "state_discontinuity": self.state_discontinuity,
            "pre_truth": self.pre.to_metadata(),
            "post_truth": self.post.to_metadata(),
        }
    ####
####


@dataclass(slots=True)
class RuntimeVehicle:
    """Mutable integration record for one vehicle trajectory."""

    name: str
    state: RuntimeState
    derivative: Derivative | None = None
    step_size: float = 1.0
    dependencies: tuple[str, ...] = ()
    dependency_segments: Mapping[str, int] = field(default_factory=dict)
    segment_number: int = 1
    active: bool = True
    activation_pending: bool = True
    history: list[RuntimeState] = field(default_factory=list)
    stop_when: Callable[[RuntimeState], bool] | None = None
    events: tuple[EventCondition, ...] = ()
    integrator: str = "rk4"
    absolute_tolerance: float = 1e-8
    relative_tolerance: float = 1e-8
    max_step_size: float | None = None
    derived_definitions: Mapping[str, ExpressionType] = field(default_factory=dict)
    definition_evaluator: Callable[[Mapping[str, float]], Mapping[str, float]] | None = None
    definition_call_handler: Callable[[str, Sequence[float]], float] | None = None
    parameters: Mapping[str, float] = field(default_factory=dict)
    control_values: Mapping[str, float] = field(default_factory=dict)
    control_values_time_s: float | None = None
    committed_control_resolver: CommittedControlResolver | None = None
    committed_control_names: frozenset[str] = field(default_factory=frozenset)
    load_evaluation_history: list[LoadEvaluationRecord] = field(default_factory=list)
    control_evaluation_history: list[ControlEvaluationRecord] = field(default_factory=list)
    control_interval_history: list[ControlIntervalRecord] = field(default_factory=list)
    table_evaluators: Mapping[str, Callable[[Mapping[str, float]], float]] = field(default_factory=dict)
    environment_evaluator: Callable[[Mapping[str, float]], Mapping[str, float]] | None = None
    event_handlers: Mapping[str, Callable[[RuntimeState], RuntimeState]] = field(default_factory=dict)
    activation_handler: Callable[[RuntimeState], RuntimeState] | None = None
    point_mass_derivative: PointMassDerivative | None = None
    dynamics_mode: DynamicsMode = DynamicsMode.POINT_MASS
    publish_derived_rates: bool = True
    kinematic_state: Kinematic6DofState | None = None
    body_rate_provider: BodyRateProvider | None = None
    kinematic_attitude_target_provider: KinematicAttitudeTargetProvider | None = None
    kinematic_response_profile_id: str | None = None
    stall_detector: StallDetector | None = None
    vehicle_kind: VehicleKind = VehicleKind.GENERIC
    model_id: str | None = None
    parent_model_id: str | None = None
    spawn_provider: SpawnProvider | None = None
    truth_provider: TruthProvider | None = None
    controller_state: object | None = None
    fired_events: set[str] = field(default_factory=set)

    def __post_init__(self) -> None:
        if self.model_id is None:
            self.model_id = self.name
        if not self.model_id:
            raise ValueError("vehicle model_id must not be empty")
        if self.step_size <= 0.0:
            raise ValueError("vehicle step_size must be positive")
        if self.active:
            self.activation_pending = False
        if self.control_values:
            named = {**self.state.named, **self.control_values}
            self.state = RuntimeState(self.state.time, self.state.values, self.state.frame, named, self.state.value_names, self.state.segment_endpoints)
        if self.control_values_time_s is None:
            self.control_values_time_s = self.state.time
        if not math.isfinite(self.control_values_time_s) or self.control_values_time_s > self.state.time + 1.0e-12:
            raise ValueError("RuntimeVehicle control_values_time_s must be finite and no later than the committed state")
        if self.dynamics_mode is DynamicsMode.KINEMATIC_6DOF and self.kinematic_state is None:
            raise FidelitySetupError(
                "missing-kinematic-sidecar",
                "kinematic-6dof vehicles require a kinematic state sidecar",
                "construct Kinematic6DofState from the initial ECFC translation and attach it to RuntimeVehicle",
                field="kinematic_state",
            )
        if self.dynamics_mode is not DynamicsMode.KINEMATIC_6DOF and self.kinematic_state is not None:
            raise FidelitySetupError(
                "unexpected-kinematic-sidecar",
                "kinematic state sidecars require kinematic-6dof mode",
                "select kinematic-6dof or remove the sidecar for point-mass and rigid-body modes",
                field="dynamics_mode/kinematic_state",
            )
        if not self.history:
            self.history.append(self.state)
        ####

    def record_load_evaluation(self, state_time_s: float, phase: LoadEvaluationPhase) -> None:
        """Append private load-evaluation provenance without publishing a state."""

        control_time_s = self.control_values_time_s
        if control_time_s is None:
            raise RuntimeError("RuntimeVehicle control timestamp is unavailable")
        self.load_evaluation_history.append(LoadEvaluationRecord(state_time_s, control_time_s, phase))
        ####

    def effective_control_values(self, values: Mapping[str, float] | None = None) -> dict[str, float]:
        """Return declared control coordinates after an evaluator's local update."""

        effective = dict(self.control_values)
        if values is not None:
            for name in effective:
                if name in values and name not in self.committed_control_names:
                    effective[name] = float(values[name])
        return effective
        ####

    def resolve_committed_controls(self) -> None:
        """Resolve state-derived controls once at the current truth boundary.

        A resolver is intentionally evaluated only immediately before an
        accepted integration interval starts.  Its result becomes part of the
        vehicle's control mapping and is re-applied after every environment
        refresh, so an RK stage cannot silently replace the held command with
        a value calculated from speculative state.  Resolver callbacks are
        executable model code and are therefore reattached by lowering rather
        than serialized as checkpoint data.
        """

        if self.committed_control_resolver is None:
            return
        # A step request updates ``control_values`` immediately before this
        # resolver runs.  Present that held action at the committed truth
        # boundary as well as during derivative refresh.  Route resolvers can
        # then deliberately elect an externally requested guidance mode
        # without having to read mutable vehicle internals or wait one
        # integration interval for the command to appear in ``state.named``.
        resolver_state = RuntimeState(
            self.state.time,
            self.state.values,
            self.state.frame,
            {**self.state.named, **self.control_values},
            self.state.value_names,
            self.state.segment_endpoints,
        )
        resolved = self.committed_control_resolver(resolver_state)
        normalized: dict[str, float] = {}
        for name, value in resolved.items():
            if not isinstance(name, str) or not name:
                raise ValueError("committed control resolver returned an empty or non-string control name")
            numeric = float(value)
            if not math.isfinite(numeric):
                raise ValueError(f"committed control resolver returned a non-finite value for {name!r}")
            normalized[name] = numeric
        if normalized:
            self.control_values = {**self.control_values, **normalized}
            self.committed_control_names = frozenset((*self.committed_control_names, *normalized))
            self.control_values_time_s = self.state.time
        ####

    def record_control_evaluation(
        self,
        state_time_s: float,
        phase: ControlEvaluationPhase,
        values: Mapping[str, float] | None = None,
    ) -> None:
        """Retain the native control vector at one private evaluation point."""

        control_time_s = self.control_values_time_s
        if control_time_s is None:
            raise RuntimeError("RuntimeVehicle control timestamp is unavailable")
        controls = tuple(sorted((str(name), float(value)) for name, value in self.effective_control_values(values).items()))
        self.control_evaluation_history.append(
            ControlEvaluationRecord(state_time_s, control_time_s, phase, controls)
        )
        ####

    def record_control_interval(
        self,
        interval_start_time_s: float,
        committed_truth_time_s: float,
        controls_at_interval_start: Mapping[str, float],
        *,
        evaluation_history_start: int,
    ) -> None:
        """Record one accepted integration interval without asserting hold semantics."""

        control_time_s = self.control_values_time_s
        if control_time_s is None:
            raise RuntimeError("RuntimeVehicle control timestamp is unavailable")
        evaluations = self.control_evaluation_history[evaluation_history_start:]
        initial = tuple(sorted((str(name), float(value)) for name, value in controls_at_interval_start.items()))
        solver_stage_mutation = any(
            record.phase == "solver_stage" and record.controls != initial
            for record in evaluations
        )
        self.control_interval_history.append(
            ControlIntervalRecord(
                interval_start_time_s,
                committed_truth_time_s,
                control_time_s,
                initial,
                solver_stage_mutation,
            )
        )
        ####

    def discard_control_provenance_after(self, time_s: float) -> None:
        """Drop speculative control/load provenance after event refinement."""

        self.load_evaluation_history = [
            record for record in self.load_evaluation_history if record.state_time_s <= time_s + 1.0e-12
        ]
        self.control_evaluation_history = [
            record for record in self.control_evaluation_history if record.state_time_s <= time_s + 1.0e-12
        ]
        self.control_interval_history = [
            record for record in self.control_interval_history if record.committed_truth_time_s <= time_s + 1.0e-12
        ]
        ####
    ####


@dataclass(frozen=True, slots=True)
class SpawnRequest:
    """Deferred request to add an arbitrary vehicle at an accepted boundary."""

    event_id: str
    child: RuntimeVehicle
    parent_model_id: str | None = None
    source: str | None = None
    metadata: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.event_id:
            raise ValueError("spawn request event_id must not be empty")
        if self.parent_model_id is not None and not self.parent_model_id:
            raise ValueError("spawn request parent_model_id must not be empty")
        if not self.child.name or not self.child.model_id:
            raise ValueError("spawn request child must have stable name and model_id")
####


class DeploymentValidationError(ValueError):
    """Raised when an accepted-boundary deployment cannot be committed."""

    ####


@dataclass(slots=True)
class RuntimeProblem:
    """Resolved vehicle graph and problem-level execution settings."""

    vehicles: dict[str, RuntimeVehicle]
    print_times: tuple[float, ...] = ()
    table_knots: tuple[float, ...] = ()
    final_time: float | None = None
    metadata: dict[str, object] = field(default_factory=dict)
    event_history: list[dict[str, object]] = field(default_factory=list)
    required_truth_times: tuple[float, ...] = ()
    sensor_clocks: tuple[SensorClockSpec, ...] = ()
    transition_history: list[TransitionTruthPair] = field(default_factory=list)
    sensor_bus: "SensorBus | None" = None
    feedback_guard: Callable[[float], None] | None = None

    def active_vehicles(self) -> tuple[RuntimeVehicle, ...]:
        return tuple(vehicle for vehicle in self.vehicles.values() if vehicle.active)
    ####

    def add_spawned_vehicles(self, requests: Sequence[SpawnRequest], *, parent: RuntimeVehicle, accepted_time: float) -> None:
        """Validate and register children as one all-or-nothing collection update."""

        existing_names = set(self.vehicles)
        existing_model_ids = {item.model_id for item in self.vehicles.values()}
        pending_names: set[str] = set()
        pending_model_ids: set[str | None] = set()
        for request in requests:
            child = request.child
            parent_model_id = request.parent_model_id or parent.model_id
            if parent_model_id != parent.model_id:
                raise DeploymentValidationError(
                    f"spawn request {request.event_id!r} names parent {parent_model_id!r}, "
                    f"expected {parent.model_id!r}"
                )
            if child.name in existing_names or child.name in pending_names or child.model_id in existing_model_ids or child.model_id in pending_model_ids:
                raise DeploymentValidationError(f"spawned model {child.name!r} or model_id {child.model_id!r} already exists")
            if abs(child.state.time - accepted_time) > 1.0e-9:
                raise DeploymentValidationError(
                    f"spawned model {child.name!r} starts at {child.state.time:g}, expected accepted time {accepted_time:g}"
                )
            if not child.history or abs(child.history[-1].time - accepted_time) > 1.0e-9:
                raise DeploymentValidationError(f"spawned model {child.name!r} history is not initialized at the accepted time")
            if child.dependencies:
                raise DeploymentValidationError(f"spawned model {child.name!r} cannot defer activation through dependencies")
            pending_names.add(child.name)
            pending_model_ids.add(child.model_id)
        for request in requests:
            child = request.child
            child.parent_model_id = parent.model_id
            child.active = True
            child.activation_pending = False
            self.vehicles[child.name] = child
    ####

    def add_spawned_vehicle(self, request: SpawnRequest, *, parent: RuntimeVehicle, accepted_time: float) -> None:
        """Validate and register one child through the collection transaction."""

        self.add_spawned_vehicles((request,), parent=parent, accepted_time=accepted_time)
    ####

    def observe(self, vehicle: str | None = None, *, status_names: Sequence[str] = (), include_deep: bool = False) -> object:
        """Return a tiered observation for one vehicle or all vehicles."""

        from .observations import observe_vehicle

        if vehicle is not None:
            return observe_vehicle(self.vehicles[vehicle], status_names=status_names, include_deep=include_deep)
        return {name: observe_vehicle(item, status_names=status_names, include_deep=include_deep) for name, item in self.vehicles.items()}
    ####

    def clone_at(self, time: float, *, resume: bool = True) -> RuntimeProblem:
        """Clone the executable graph at a recorded or interpolated flight time.

        Vehicle callbacks remain shared because they are executable model code;
        mutable vehicle state, histories, metadata, and event sets are copied so
        the returned graph can be advanced independently.
        """

        if not self.vehicles:
            raise ValueError("cannot clone an empty runtime problem")
        histories: dict[str, list[RuntimeState]] = {}
        cloned_vehicles: dict[str, RuntimeVehicle] = {}
        for name, source in self.vehicles.items():
            if not source.history:
                raise ValueError(f"vehicle {name!r} has no history to clone")
            first = source.history[0].time
            last = source.history[-1].time
            if time < first - 1e-12 or time > last + 1e-12:
                raise ValueError(f"clone time {time} is outside vehicle {name!r} history [{first}, {last}]")
            bounded_time = min(last, max(first, time))
            retained = [state for state in source.history if state.time < bounded_time - 1e-12]
            exact = next((state for state in source.history if abs(state.time - bounded_time) <= 1e-12), None)
            if exact is None:
                upper_index = next(index for index, state in enumerate(source.history) if state.time > bounded_time)
                exact = source.history[upper_index - 1].interpolated(source.history[upper_index], bounded_time)
            retained.append(exact)
            histories[name] = retained
            clone = copy(source)
            clone.state = exact
            clone.history = retained
            clone.load_evaluation_history = [
                record for record in source.load_evaluation_history if record.state_time_s <= bounded_time + 1.0e-12
            ]
            clone.control_evaluation_history = [
                record for record in source.control_evaluation_history if record.state_time_s <= bounded_time + 1.0e-12
            ]
            clone.control_interval_history = [
                record
                for record in source.control_interval_history
                if record.committed_truth_time_s <= bounded_time + 1.0e-12
            ]
            if clone.control_values_time_s is None:
                raise RuntimeError(f"vehicle {name!r} has no control activation timestamp to clone")
            # A branch retains the effective command vector, but it becomes a
            # new executable boundary when the source command postdates the
            # requested rewind time.  Never carry a future control activation
            # timestamp into a historical branch.
            clone.control_values_time_s = min(clone.control_values_time_s, bounded_time)
            clone.fired_events = set(event for event in source.fired_events if event in {condition.name for condition in source.events if condition.action != "stop"})
            if resume:
                clone.active = True
                clone.activation_pending = False
            cloned_vehicles[name] = clone
        cloned = RuntimeProblem(
            cloned_vehicles,
            self.print_times,
            self.table_knots,
            self.final_time,
            deepcopy(self.metadata),
            deepcopy(self.event_history),
            self.required_truth_times,
            self.sensor_clocks,
            deepcopy(self.transition_history),
            deepcopy(self.sensor_bus),
            self.feedback_guard,
        )
        cloned.metadata["cloned_at_time"] = float(time)
        return cloned
    ####
####


@dataclass(frozen=True, slots=True)
class StepBoundary:
    """A future time at which a step must end."""

    time: float
    reason: str


@dataclass(frozen=True, slots=True)
class EventCondition:
    """A scalar event residual evaluated against a runtime state."""

    name: str
    function: Callable[[RuntimeState], float]
    action: str = "stop"
    predicate: Callable[[RuntimeState], bool] | None = None
    signal: str | None = None
    source: str | None = None


@dataclass(frozen=True, slots=True)
class SearchRestart:
    """A parameter update and earliest history time it invalidates."""

    parameter: str
    value: float
    restart_time: float


@dataclass(frozen=True, slots=True)
class DerivativePipeline:
    """Ordered derivative stages; each stage receives and returns named state."""

    stages: tuple[Callable[[Mapping[str, float]], Mapping[str, float]], ...]

    def evaluate(self, initial: Mapping[str, float]) -> dict[str, float]:
        current = dict(initial)
        for stage in self.stages:
            current = dict(stage(current))
        ####
        return current
    ####
####
