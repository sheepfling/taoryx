"""Physically accountable control-allocation and actuator contracts.

The controller layer is allowed to request a generalized wrench, but it is
not allowed to silently treat that request as a force or moment applied to a
vehicle.  This module closes that seam for every family by separating:

``controller demand -> constrained effector command -> actuator state -> wrench``.

Family adapters supply local effectiveness and nonlinear force/moment
evaluation.  The allocator only solves the bounded numerical problem and
reports what it could not achieve.  It is intentionally useful for elevons,
conventional surfaces, rotors, wheels, thrusters, gimbals, and magnetic
dipoles without encoding any of those families in the generic solver.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal, Protocol

import numpy as np
from scipy.optimize import lsq_linear

from .control import ControlCommand, ControlDemand, VehicleObservation
from .trim import DynamicsEvaluator, DynamicsLinearization, TrimResult, TrimSpec, finite_difference_dynamics_linearization

if TYPE_CHECKING:
    from .controller_realization import AllocationResult

AllocationStatus = Literal[
    "feasible",
    "feasible_near_limit",
    "partially_achievable",
    "infeasible",
    "numerically_singular",
    "solver_failure",
]


@dataclass(frozen=True, slots=True)
class EffectorLimits:
    """Physical position, rate, and response limits for one actuator.

    ``time_constant_s=0`` is an explicitly ideal actuator.  It does not
    eliminate travel or rate limits.  An unavailable effector is held at its
    previous actual position by the allocator instead of disappearing from a
    result without evidence.
    """

    name: str
    lower: float
    upper: float
    unit: str
    rate_limit_per_s: float | None = None
    time_constant_s: float = 0.0
    available: bool = True

    def __post_init__(self) -> None:
        if not self.name.strip() or not self.unit.strip():
            raise ValueError("effector limits require non-empty name and unit")
        if not all(math.isfinite(value) for value in (self.lower, self.upper)) or self.lower > self.upper:
            raise ValueError(f"effector {self.name!r} has invalid position limits")
        if self.rate_limit_per_s is not None and (
            not math.isfinite(self.rate_limit_per_s) or self.rate_limit_per_s <= 0.0
        ):
            raise ValueError(f"effector {self.name!r} rate limit must be finite and positive")
        if not math.isfinite(self.time_constant_s) or self.time_constant_s < 0.0:
            raise ValueError(f"effector {self.name!r} time constant must be finite and nonnegative")
        ####
    ####

    def clamp(self, value: float) -> float:
        """Return the physically reachable position bound for ``value``."""

        return min(self.upper, max(self.lower, float(value)))
        ####
    ####


@dataclass(frozen=True, slots=True)
class EffectorEffectiveness:
    """Local affine mapping from actual effectors to a requested wrench.

    The matrix maps incremental effector positions around the declared
    reference point:

    ``w = reference_wrench + G @ (u - reference_effectors)``.

    The mapping may be rectangular or rank deficient.  That is normal: a
    flying wing may have no independently controlled yaw moment, and a
    redundant vehicle may have more effectors than requested wrench axes.
    """

    wrench_names: tuple[str, ...]
    effector_names: tuple[str, ...]
    matrix: tuple[tuple[float, ...], ...]
    reference_wrench: Mapping[str, float] = field(default_factory=dict)
    reference_effectors: Mapping[str, float] = field(default_factory=dict)
    source: str = "unspecified"

    def __post_init__(self) -> None:
        if not self.wrench_names or not self.effector_names:
            raise ValueError("effectiveness requires at least one wrench and effector channel")
        if len(set(self.wrench_names)) != len(self.wrench_names):
            raise ValueError("effectiveness wrench names must be unique")
        if len(set(self.effector_names)) != len(self.effector_names):
            raise ValueError("effectiveness effector names must be unique")
        if len(self.matrix) != len(self.wrench_names) or any(len(row) != len(self.effector_names) for row in self.matrix):
            raise ValueError("effectiveness matrix dimensions do not match its channel names")
        if any(not math.isfinite(float(value)) for row in self.matrix for value in row):
            raise ValueError("effectiveness matrix must contain finite values")
        if set(self.reference_wrench) - set(self.wrench_names):
            raise ValueError("reference wrench names must be declared effectiveness axes")
        if set(self.reference_effectors) - set(self.effector_names):
            raise ValueError("reference effector names must be declared effectiveness effectors")
        if any(not math.isfinite(float(value)) for value in self.reference_wrench.values()):
            raise ValueError("reference wrench values must be finite")
        if any(not math.isfinite(float(value)) for value in self.reference_effectors.values()):
            raise ValueError("reference effector values must be finite")
        if not self.source.strip():
            raise ValueError("effectiveness source must not be empty")
        ####
    ####

    @property
    def array(self) -> np.ndarray:
        """Return the effectivity matrix as a local numeric array."""

        return np.asarray(self.matrix, dtype=float)
        ####
    ####

    def reference_wrench_vector(self) -> np.ndarray:
        """Return the declared affine wrench reference in ordered form."""

        return np.asarray([float(self.reference_wrench.get(name, 0.0)) for name in self.wrench_names], dtype=float)
        ####
    ####

    def reference_effector_vector(self) -> np.ndarray:
        """Return the declared affine effector reference in ordered form."""

        return np.asarray([float(self.reference_effectors.get(name, 0.0)) for name in self.effector_names], dtype=float)
        ####
    ####

    def wrench_for(self, effectors: Mapping[str, float]) -> dict[str, float]:
        """Evaluate the affine local wrench prediction at named effectors."""

        missing = set(self.effector_names) - set(effectors)
        if missing:
            raise KeyError(f"effector values are missing: {', '.join(sorted(missing))}")
        values = np.asarray([float(effectors[name]) for name in self.effector_names], dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError("effector values must be finite")
        wrench = self.reference_wrench_vector() + self.array @ (values - self.reference_effector_vector())
        return {name: float(value) for name, value in zip(self.wrench_names, wrench, strict=True)}
        ####
    ####


@dataclass(frozen=True, slots=True)
class WrenchAllocationResult:
    """One constrained allocation before actuator-response lag is applied."""

    status: AllocationStatus
    requested_wrench: Mapping[str, float]
    predicted_wrench: Mapping[str, float]
    residual_wrench: Mapping[str, float]
    wrench_weights: Mapping[str, float]
    controlled_wrench_axes: tuple[str, ...]
    uncontrolled_wrench_axes: tuple[str, ...]
    effectiveness_matrix: tuple[tuple[float, ...], ...]
    effector_commands: Mapping[str, float]
    active_lower_bounds: Mapping[str, float]
    active_upper_bounds: Mapping[str, float]
    position_saturated: tuple[str, ...] = ()
    rate_limited: tuple[str, ...] = ()
    unavailable_effectors: tuple[str, ...] = ()
    effectiveness_rank: int = 0
    iterations: int = 0
    solver_message: str = ""

    @property
    def residual_norm(self) -> float:
        """Return the Euclidean requested-versus-predicted wrench residual."""

        return math.sqrt(sum(float(value) * float(value) for value in self.residual_wrench.values()))
        ####
    ####

    @property
    def controlled_residual_norm(self) -> float:
        """Return residual norm over the explicitly controlled wrench axes."""

        return math.sqrt(
            sum(float(self.residual_wrench[name]) * float(self.residual_wrench[name]) for name in self.controlled_wrench_axes)
        )
        ####
    ####


@dataclass(frozen=True, slots=True)
class ActuatorAdvanceResult:
    """Actual actuator state following position, rate, and lag dynamics."""

    commanded_positions: Mapping[str, float]
    actual_positions: Mapping[str, float]
    rates_per_s: Mapping[str, float]
    position_saturated: tuple[str, ...] = ()
    rate_limited: tuple[str, ...] = ()
    lag_active: tuple[str, ...] = ()
    unavailable_effectors: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class PhysicalAllocationStep:
    """One complete demand-to-actual-wrench physical-control record."""

    allocation: WrenchAllocationResult
    actuator: ActuatorAdvanceResult
    achieved_wrench: Mapping[str, float]
    achieved_residual_wrench: Mapping[str, float]

    @property
    def achieved_residual_norm(self) -> float:
        """Return the realized, not merely predicted, wrench residual."""

        return math.sqrt(sum(float(value) * float(value) for value in self.achieved_residual_wrench.values()))
        ####
    ####

    @property
    def achieved_controlled_residual_norm(self) -> float:
        """Return realized residual over axes the allocator was asked to control."""

        return math.sqrt(
            sum(
                float(self.achieved_residual_wrench[name]) * float(self.achieved_residual_wrench[name])
                for name in self.allocation.controlled_wrench_axes
            )
        )
        ####
    ####

    def as_controller_result(self, allocator_id: str) -> AllocationResult:
        """Return the canonical controller-telemetry record for this step.

        Importing the Pydantic telemetry type lazily keeps the numerical
        allocator usable by stand-alone analysis tools while making every
        runtime integration able to emit identical requested/achieved data.
        """

        from .controller_realization import AllocationResult

        if not allocator_id.strip():
            raise ValueError("allocator telemetry requires a stable allocator id")
        saturated = tuple(
            sorted(
                set(self.allocation.position_saturated)
                | set(self.allocation.rate_limited)
                | set(self.actuator.position_saturated)
                | set(self.actuator.rate_limited)
                | set(self.actuator.unavailable_effectors)
            )
        )
        return AllocationResult(
            allocator_id=allocator_id,
            requested=dict(self.allocation.requested_wrench),
            allocated=dict(self.allocation.predicted_wrench),
            achieved=dict(self.achieved_wrench),
            residual=dict(self.allocation.residual_wrench),
            wrench_weights=dict(self.allocation.wrench_weights),
            controlled_wrench_axes=self.allocation.controlled_wrench_axes,
            uncontrolled_wrench_axes=self.allocation.uncontrolled_wrench_axes,
            saturated_channels=saturated,
            rate_limited_channels=tuple(sorted(set(self.allocation.rate_limited) | set(self.actuator.rate_limited))),
            allocation_status=self.allocation.status,
            commanded_effectors=dict(self.actuator.commanded_positions),
            actual_effectors=dict(self.actuator.actual_positions),
            effector_rates=dict(self.actuator.rates_per_s),
            actual_residual=dict(self.achieved_residual_wrench),
            effectiveness_matrix=self.allocation.effectiveness_matrix,
            effectiveness_rank=self.allocation.effectiveness_rank,
            allocator_iterations=self.allocation.iterations,
            solver_message=self.allocation.solver_message,
        )
        ####



@dataclass(frozen=True, slots=True)
class DerivativeProvenance:
    """Evidence that a local A/B matrix came from a nonlinear plant.

    The primary and comparison finite-difference steps are retained so a
    controller report can show whether a derivative was insensitive to the
    numerical perturbation size.  This is deliberately provenance, not a
    claim that the derivative is valid outside the stated operating point.
    """

    nonlinear_plant_id: str
    nonlinear_plant_revision: str
    state_names: tuple[str, ...]
    control_names: tuple[str, ...]
    method: str
    state_step: float
    control_step: float
    comparison_state_step: float
    comparison_control_step: float
    maximum_relative_difference: float
    maximum_absolute_difference: float
    comparison_absolute_floor: float
    derivative_consistent: bool
    state_units: Mapping[str, str] = field(default_factory=dict)
    control_units: Mapping[str, str] = field(default_factory=dict)
    frozen_states: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.nonlinear_plant_id.strip() or not self.nonlinear_plant_revision.strip() or not self.method.strip():
            raise ValueError("derivative provenance requires plant identity, revision, and method")
        if not self.state_names or not self.control_names:
            raise ValueError("derivative provenance requires ordered state and control names")
        if any(
            not math.isfinite(value) or value <= 0.0
            for value in (
                self.state_step,
                self.control_step,
                self.comparison_state_step,
                self.comparison_control_step,
            )
        ):
            raise ValueError("finite-difference steps must be finite and positive")
        if not math.isfinite(self.maximum_relative_difference) or self.maximum_relative_difference < 0.0:
            raise ValueError("maximum derivative difference must be finite and nonnegative")
        if not math.isfinite(self.maximum_absolute_difference) or self.maximum_absolute_difference < 0.0:
            raise ValueError("maximum absolute derivative difference must be finite and nonnegative")
        if not math.isfinite(self.comparison_absolute_floor) or self.comparison_absolute_floor < 0.0:
            raise ValueError("derivative comparison absolute floor must be finite and nonnegative")
        if set(self.state_units) - set(self.state_names):
            raise ValueError("state units must name declared states")
        if set(self.control_units) - set(self.control_names):
            raise ValueError("control units must name declared controls")
        if set(self.frozen_states) - set(self.state_names):
            raise ValueError("frozen states must name declared states")
        ####
    ####


@dataclass(frozen=True, slots=True)
class ProvenancedLinearization:
    """A primary linearization, a comparison derivative, and their evidence."""

    primary: DynamicsLinearization
    comparison: DynamicsLinearization
    provenance: DerivativeProvenance


class ControlPlantAdapter(Protocol):
    """Family adapter required for physically realizable controller evidence."""

    @property
    def state_names(self) -> Sequence[str]:
        """Return the stable local-state ordering used by the adapter."""
        ...

    @property
    def control_names(self) -> Sequence[str]:
        """Return the stable physical-effector ordering used by the adapter."""
        ...

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Evaluate the actual declared nonlinear plant state derivative."""
        ...

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Solve one physical-effector equilibrium for the declared plant."""
        ...

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        """Produce a provenance-bearing local linearization around ``trim``."""
        ...

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        """Return local physical effector effectiveness at the current state."""
        ...

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        """Map a wrench request into constrained, actual physical effectors."""
        ...


def finite_difference_linearization_with_provenance(
    trim_spec: TrimSpec,
    dynamics_evaluator: DynamicsEvaluator,
    trim: TrimResult,
    *,
    nonlinear_plant_id: str,
    nonlinear_plant_revision: str,
    state_step: float = 1.0e-5,
    control_step: float = 1.0e-5,
    comparison_factor: float = 0.5,
    maximum_relative_difference: float = 0.05,
    comparison_absolute_floor: float = 0.0,
    state_units: Mapping[str, str] | None = None,
    control_units: Mapping[str, str] | None = None,
    frozen_states: Sequence[str] = (),
    metadata: Mapping[str, str | float] | None = None,
) -> ProvenancedLinearization:
    """Derive a local plant matrix twice and record step-size consistency.

    Both matrices use centered finite differences through the exact supplied
    nonlinear evaluator.  The comparison remains part of the artifact rather
    than an invisible internal check so reviewers can inspect the numerical
    basis of any LQR design.
    """

    if not 0.0 < comparison_factor < 1.0:
        raise ValueError("comparison_factor must lie strictly between zero and one")
    if not math.isfinite(maximum_relative_difference) or maximum_relative_difference < 0.0:
        raise ValueError("maximum_relative_difference must be finite and nonnegative")
    if not math.isfinite(comparison_absolute_floor) or comparison_absolute_floor < 0.0:
        raise ValueError("comparison_absolute_floor must be finite and nonnegative")
    primary = finite_difference_dynamics_linearization(
        trim_spec,
        dynamics_evaluator,
        trim,
        state_step=state_step,
        control_step=control_step,
        metadata={"method": "centered-finite-difference", **(metadata or {})},
    )
    comparison_state_step = state_step * comparison_factor
    comparison_control_step = control_step * comparison_factor
    comparison = finite_difference_dynamics_linearization(
        trim_spec,
        dynamics_evaluator,
        trim,
        state_step=comparison_state_step,
        control_step=comparison_control_step,
        metadata={"method": "centered-finite-difference", **(metadata or {})},
    )
    differences = np.concatenate(
        (
            np.abs(primary.a_matrix - comparison.a_matrix).reshape(-1),
            np.abs(primary.b_matrix - comparison.b_matrix).reshape(-1),
        )
    )
    references = np.concatenate(
        (
            np.maximum(np.abs(primary.a_matrix), np.abs(comparison.a_matrix)).reshape(-1),
            np.maximum(np.abs(primary.b_matrix), np.abs(comparison.b_matrix)).reshape(-1),
        )
    )
    # A raw relative error is not meaningful for a derivative whose value is
    # indistinguishable from floating-point/table-query noise.  The declared
    # floor remains in the artifact so a caller cannot hide this choice.
    relative = differences / np.maximum(references, max(comparison_absolute_floor, 1.0e-12))
    observed_difference = float(relative.max()) if relative.size else 0.0
    observed_absolute_difference = float(differences.max()) if differences.size else 0.0
    provenance = DerivativeProvenance(
        nonlinear_plant_id=nonlinear_plant_id,
        nonlinear_plant_revision=nonlinear_plant_revision,
        state_names=primary.state_names,
        control_names=primary.control_names,
        method="centered-finite-difference",
        state_step=state_step,
        control_step=control_step,
        comparison_state_step=comparison_state_step,
        comparison_control_step=comparison_control_step,
        maximum_relative_difference=observed_difference,
        maximum_absolute_difference=observed_absolute_difference,
        comparison_absolute_floor=comparison_absolute_floor,
        derivative_consistent=observed_difference <= maximum_relative_difference,
        state_units=dict(state_units or {}),
        control_units=dict(control_units or {}),
        frozen_states=tuple(frozen_states),
    )
    return ProvenancedLinearization(primary, comparison, provenance)
    ####


def allocate_bounded_weighted_wrench(
    effectiveness: EffectorEffectiveness,
    effectors: Mapping[str, EffectorLimits],
    desired_wrench: Mapping[str, float],
    previous_effectors: Mapping[str, float],
    dt_s: float,
    *,
    preferred_effectors: Mapping[str, float] | None = None,
    wrench_weights: Mapping[str, float] | None = None,
    effector_weights: Mapping[str, float] | None = None,
    regularization: float = 0.0,
    feasibility_tolerance: float = 1.0e-7,
) -> WrenchAllocationResult:
    """Solve the bounded weighted least-squares physical allocation problem.

    The result is always explicit about residual error and active constraints.
    It never clips an unachievable wrench and calls it achieved.
    """

    if not math.isfinite(dt_s) or dt_s < 0.0:
        raise ValueError("allocation dt_s must be finite and nonnegative")
    if not math.isfinite(regularization) or regularization < 0.0:
        raise ValueError("allocation regularization must be finite and nonnegative")
    if not math.isfinite(feasibility_tolerance) or feasibility_tolerance <= 0.0:
        raise ValueError("allocation feasibility tolerance must be finite and positive")
    missing_limits = set(effectiveness.effector_names) - set(effectors)
    if missing_limits:
        raise KeyError(f"effector limits are missing: {', '.join(sorted(missing_limits))}")
    missing_wrench = set(effectiveness.wrench_names) - set(desired_wrench)
    if missing_wrench:
        raise KeyError(f"desired wrench is missing: {', '.join(sorted(missing_wrench))}")
    missing_previous = set(effectiveness.effector_names) - set(previous_effectors)
    if missing_previous:
        raise KeyError(f"previous effector state is missing: {', '.join(sorted(missing_previous))}")
    if any(not math.isfinite(float(desired_wrench[name])) for name in effectiveness.wrench_names):
        raise ValueError("desired wrench values must be finite")
    if any(not math.isfinite(float(previous_effectors[name])) for name in effectiveness.effector_names):
        raise ValueError("previous effector positions must be finite")

    names = effectiveness.effector_names
    wrench_names = effectiveness.wrench_names
    matrix = effectiveness.array
    reference = effectiveness.reference_effector_vector()
    desired = np.asarray([float(desired_wrench[name]) for name in wrench_names], dtype=float)
    prior = np.asarray([float(previous_effectors[name]) for name in names], dtype=float)
    preference = np.asarray(
        [
            float((preferred_effectors or {}).get(name, effectiveness.reference_effectors.get(name, prior[index])))
            for index, name in enumerate(names)
        ],
        dtype=float,
    )
    if not np.all(np.isfinite(preference)):
        raise ValueError("preferred effector positions must be finite")
    axis_weight = _weights_for(wrench_names, wrench_weights, "wrench")
    actuator_weight = _weights_for(names, effector_weights, "effector")
    if not np.any(axis_weight > 0.0):
        raise ValueError("at least one wrench weight must be positive")

    hard_lower, hard_upper, dynamic_lower, dynamic_upper, unavailable = _allocation_bounds(
        names,
        effectors,
        prior,
        dt_s,
    )
    hard_solution = _solve_weighted_problem(
        matrix,
        effectiveness.reference_wrench_vector(),
        reference,
        desired,
        preference,
        axis_weight,
        actuator_weight,
        regularization,
        hard_lower,
        hard_upper,
    )
    solution = _solve_weighted_problem(
        matrix,
        effectiveness.reference_wrench_vector(),
        reference,
        desired,
        preference,
        axis_weight,
        actuator_weight,
        regularization,
        dynamic_lower,
        dynamic_upper,
    )
    commands = solution.values
    predicted = effectiveness.wrench_for({name: float(value) for name, value in zip(names, commands, strict=True)})
    residual = {name: float(desired_wrench[name]) - predicted[name] for name in wrench_names}
    position_saturated = tuple(
        name
        for index, name in enumerate(names)
        if _at_limit(commands[index], hard_lower[index], hard_upper[index])
        and not math.isclose(commands[index], preference[index], rel_tol=0.0, abs_tol=1.0e-9)
    )
    rate_limited = tuple(
        name
        for index, name in enumerate(names)
        if (
            not math.isclose(dynamic_lower[index], hard_lower[index], rel_tol=0.0, abs_tol=1.0e-12)
            or not math.isclose(dynamic_upper[index], hard_upper[index], rel_tol=0.0, abs_tol=1.0e-12)
        )
        and not math.isclose(commands[index], hard_solution.values[index], rel_tol=0.0, abs_tol=1.0e-9)
    )
    weighted_matrix = axis_weight[:, None] * matrix
    active = np.asarray(
        [dynamic_upper[index] - dynamic_lower[index] > 1.0e-12 for index in range(len(names))],
        dtype=bool,
    )
    rank = int(np.linalg.matrix_rank(weighted_matrix[:, active])) if np.any(active) else 0
    controlled_axes = tuple(name for name, weight in zip(wrench_names, axis_weight, strict=True) if weight > 0.0)
    uncontrolled_axes = tuple(name for name, weight in zip(wrench_names, axis_weight, strict=True) if weight == 0.0)
    residual_vector = np.asarray([residual[name] for name in wrench_names], dtype=float)
    controlled_residual_norm = float(np.linalg.norm(axis_weight * residual_vector))
    controlled_target_norm = float(np.linalg.norm(axis_weight * desired))
    numerical_singularity = rank == 0 and controlled_residual_norm > feasibility_tolerance * max(1.0, controlled_target_norm)
    if solution.failure:
        status: AllocationStatus = "solver_failure"
    elif numerical_singularity:
        status = "numerically_singular"
    elif controlled_residual_norm <= feasibility_tolerance * max(1.0, controlled_target_norm):
        status = "feasible_near_limit" if position_saturated or rate_limited or unavailable else "feasible"
    elif position_saturated or rate_limited or unavailable or rank < int(np.count_nonzero(axis_weight > 0.0)):
        status = "partially_achievable"
    else:
        status = "infeasible"
    return WrenchAllocationResult(
        status=status,
        requested_wrench={name: float(desired_wrench[name]) for name in wrench_names},
        predicted_wrench=predicted,
        residual_wrench=residual,
        wrench_weights={name: float(weight) for name, weight in zip(wrench_names, axis_weight, strict=True)},
        controlled_wrench_axes=controlled_axes,
        uncontrolled_wrench_axes=uncontrolled_axes,
        effectiveness_matrix=tuple(tuple(float(value) for value in row) for row in matrix),
        effector_commands={name: float(value) for name, value in zip(names, commands, strict=True)},
        active_lower_bounds={name: float(value) for name, value in zip(names, dynamic_lower, strict=True)},
        active_upper_bounds={name: float(value) for name, value in zip(names, dynamic_upper, strict=True)},
        position_saturated=position_saturated,
        rate_limited=rate_limited,
        unavailable_effectors=unavailable,
        effectiveness_rank=rank,
        iterations=solution.iterations,
        solver_message=solution.message,
    )
    ####


def advance_actuators(
    effectors: Mapping[str, EffectorLimits],
    commanded_positions: Mapping[str, float],
    previous_positions: Mapping[str, float],
    dt_s: float,
) -> ActuatorAdvanceResult:
    """Advance actual effector positions through declared first-order dynamics."""

    if not math.isfinite(dt_s) or dt_s < 0.0:
        raise ValueError("actuator advance dt_s must be finite and nonnegative")
    names = tuple(effectors)
    if set(commanded_positions) != set(names) or set(previous_positions) != set(names):
        raise ValueError("actuator command and prior state names must match declared effectors")
    actual: dict[str, float] = {}
    rates: dict[str, float] = {}
    position_saturated: list[str] = []
    rate_limited: list[str] = []
    lag_active: list[str] = []
    unavailable: list[str] = []
    for name in names:
        limits = effectors[name]
        previous = limits.clamp(float(previous_positions[name]))
        raw_command = float(commanded_positions[name])
        command = limits.clamp(raw_command)
        if not math.isclose(raw_command, command, rel_tol=0.0, abs_tol=1.0e-12):
            position_saturated.append(name)
        if not limits.available:
            actual[name] = previous
            rates[name] = 0.0
            unavailable.append(name)
            continue
        if dt_s == 0.0:
            candidate = previous
        elif limits.time_constant_s > 0.0:
            fraction = 1.0 - math.exp(-dt_s / limits.time_constant_s)
            candidate = previous + fraction * (command - previous)
            if not math.isclose(candidate, command, rel_tol=0.0, abs_tol=1.0e-12):
                lag_active.append(name)
        else:
            candidate = command
        if limits.rate_limit_per_s is not None and dt_s > 0.0:
            permitted_delta = limits.rate_limit_per_s * dt_s
            bounded_candidate = min(previous + permitted_delta, max(previous - permitted_delta, candidate))
            if not math.isclose(candidate, bounded_candidate, rel_tol=0.0, abs_tol=1.0e-12):
                rate_limited.append(name)
            candidate = bounded_candidate
        value = limits.clamp(candidate)
        if not math.isclose(value, candidate, rel_tol=0.0, abs_tol=1.0e-12):
            position_saturated.append(name)
        actual[name] = value
        rates[name] = 0.0 if dt_s == 0.0 else (value - previous) / dt_s
    return ActuatorAdvanceResult(
        commanded_positions={name: float(commanded_positions[name]) for name in names},
        actual_positions=actual,
        rates_per_s=rates,
        position_saturated=tuple(sorted(set(position_saturated))),
        rate_limited=tuple(sorted(set(rate_limited))),
        lag_active=tuple(sorted(set(lag_active))),
        unavailable_effectors=tuple(sorted(set(unavailable))),
    )
    ####


def allocate_and_advance_wrench(
    effectiveness: EffectorEffectiveness,
    effectors: Mapping[str, EffectorLimits],
    desired_wrench: Mapping[str, float],
    previous_effectors: Mapping[str, float],
    dt_s: float,
    *,
    preferred_effectors: Mapping[str, float] | None = None,
    wrench_weights: Mapping[str, float] | None = None,
    effector_weights: Mapping[str, float] | None = None,
    regularization: float = 0.0,
    feasibility_tolerance: float = 1.0e-7,
) -> PhysicalAllocationStep:
    """Allocate a wrench and advance actual effectors in one evidence record."""

    allocation = allocate_bounded_weighted_wrench(
        effectiveness,
        effectors,
        desired_wrench,
        previous_effectors,
        dt_s,
        preferred_effectors=preferred_effectors,
        wrench_weights=wrench_weights,
        effector_weights=effector_weights,
        regularization=regularization,
        feasibility_tolerance=feasibility_tolerance,
    )
    actuator = advance_actuators(effectors, allocation.effector_commands, previous_effectors, dt_s)
    achieved = effectiveness.wrench_for(actuator.actual_positions)
    residual = {
        name: float(desired_wrench[name]) - achieved[name]
        for name in effectiveness.wrench_names
    }
    return PhysicalAllocationStep(allocation, actuator, achieved, residual)
    ####


@dataclass(slots=True)
class ConstrainedWrenchAllocator:
    """Stateful adapter from generic wrench demand to actual effectors.

    It implements the repository's existing :class:`ControlAllocator` shape
    while retaining the full allocation/actuator evidence in ``last_step``.
    The caller may write that record directly to a controller telemetry stream.
    """

    effectiveness: EffectorEffectiveness
    effectors: Mapping[str, EffectorLimits]
    preferred_effectors: Mapping[str, float] = field(default_factory=dict)
    wrench_weights: Mapping[str, float] = field(default_factory=dict)
    effector_weights: Mapping[str, float] = field(default_factory=dict)
    regularization: float = 0.0
    feasibility_tolerance: float = 1.0e-7
    _actual_positions: dict[str, float] = field(default_factory=dict, init=False)
    _last_time_s: float | None = field(default=None, init=False)
    last_step: PhysicalAllocationStep | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        if set(self.effectiveness.effector_names) != set(self.effectors):
            raise ValueError("allocator effectors must exactly match effectiveness channels")
        self._actual_positions = {
            name: self.effectors[name].clamp(float(self.effectiveness.reference_effectors.get(name, 0.0)))
            for name in self.effectiveness.effector_names
        }
        ####

    def reset(self, positions: Mapping[str, float] | None = None, *, time_s: float | None = None) -> None:
        """Reset the actuator state to named actual positions or trim values."""

        selected = positions or self.effectiveness.reference_effectors
        if set(selected) - set(self.effectiveness.effector_names):
            raise ValueError("allocator reset names must be declared effectors")
        self._actual_positions = {
            name: self.effectors[name].clamp(float(selected.get(name, self.effectiveness.reference_effectors.get(name, 0.0))))
            for name in self.effectiveness.effector_names
        }
        self._last_time_s = time_s
        self.last_step = None
        ####

    @property
    def actual_positions(self) -> Mapping[str, float]:
        """Expose a copy of the current physical actuator positions."""

        return dict(self._actual_positions)
        ####

    def allocate(self, demand: ControlDemand, observation: VehicleObservation) -> ControlCommand:
        """Apply demand using elapsed committed truth time, not interpolation."""

        if self._last_time_s is not None and observation.time_s < self._last_time_s:
            raise ValueError("allocator observation time must be monotonic")
        dt_s = 0.0 if self._last_time_s is None else observation.time_s - self._last_time_s
        step = allocate_and_advance_wrench(
            self.effectiveness,
            self.effectors,
            demand.values,
            self._actual_positions,
            dt_s,
            preferred_effectors=self.preferred_effectors,
            wrench_weights=self.wrench_weights,
            effector_weights=self.effector_weights,
            regularization=self.regularization,
            feasibility_tolerance=self.feasibility_tolerance,
        )
        self._actual_positions = dict(step.actuator.actual_positions)
        self._last_time_s = observation.time_s
        self.last_step = step
        saturated = tuple(
            sorted(
                set(step.allocation.position_saturated)
                | set(step.allocation.rate_limited)
                | set(step.actuator.position_saturated)
                | set(step.actuator.rate_limited)
                | set(step.actuator.unavailable_effectors)
            )
        )
        return ControlCommand(dict(step.actuator.actual_positions), saturated, demand.source)
        ####


@dataclass(frozen=True, slots=True)
class _LeastSquaresSolution:
    """Private numeric result used to preserve solver diagnostics."""

    values: np.ndarray
    iterations: int
    message: str
    failure: bool = False


def _weights_for(names: Sequence[str], provided: Mapping[str, float] | None, label: str) -> np.ndarray:
    """Resolve nonnegative wrench or positive actuator weights in order."""

    mapping = provided or {}
    unknown = set(mapping) - set(names)
    if unknown:
        raise ValueError(f"{label} weights name unknown channels: {', '.join(sorted(unknown))}")
    values = np.asarray([float(mapping.get(name, 1.0)) for name in names], dtype=float)
    if not np.all(np.isfinite(values)) or np.any(values < 0.0):
        raise ValueError(f"{label} weights must be finite and nonnegative")
    if label == "effector" and np.any(values <= 0.0):
        raise ValueError("effector weights must be positive")
    return values
    ####


def _allocation_bounds(
    names: Sequence[str],
    effectors: Mapping[str, EffectorLimits],
    prior: np.ndarray,
    dt_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, tuple[str, ...]]:
    """Return physical and rate-constrained position bounds."""

    hard_lower = np.empty(len(names), dtype=float)
    hard_upper = np.empty(len(names), dtype=float)
    dynamic_lower = np.empty(len(names), dtype=float)
    dynamic_upper = np.empty(len(names), dtype=float)
    unavailable: list[str] = []
    for index, name in enumerate(names):
        limits = effectors[name]
        hard_lower[index] = limits.lower
        hard_upper[index] = limits.upper
        if not limits.available:
            held = limits.clamp(float(prior[index]))
            dynamic_lower[index] = held
            dynamic_upper[index] = held
            unavailable.append(name)
            continue
        dynamic_lower[index] = limits.lower
        dynamic_upper[index] = limits.upper
        if limits.rate_limit_per_s is not None and dt_s > 0.0:
            permitted_delta = limits.rate_limit_per_s * dt_s
            dynamic_lower[index] = max(dynamic_lower[index], prior[index] - permitted_delta)
            dynamic_upper[index] = min(dynamic_upper[index], prior[index] + permitted_delta)
    return hard_lower, hard_upper, dynamic_lower, dynamic_upper, tuple(unavailable)
    ####


def _solve_weighted_problem(
    matrix: np.ndarray,
    reference_wrench: np.ndarray,
    reference_effectors: np.ndarray,
    desired_wrench: np.ndarray,
    preference: np.ndarray,
    wrench_weights: np.ndarray,
    effector_weights: np.ndarray,
    regularization: float,
    lower: np.ndarray,
    upper: np.ndarray,
) -> _LeastSquaresSolution:
    """Solve one affine constrained least-squares allocation deterministically."""

    fixed = np.isclose(lower, upper, rtol=0.0, atol=1.0e-12)
    values = np.empty(matrix.shape[1], dtype=float)
    values[fixed] = lower[fixed]
    free = ~fixed
    if not np.any(free):
        return _LeastSquaresSolution(values, 0, "all effectors fixed")
    target = desired_wrench - reference_wrench + matrix @ reference_effectors
    if np.any(fixed):
        target = target - matrix[:, fixed] @ values[fixed]
    free_matrix = matrix[:, free]
    rows: list[np.ndarray] = [wrench_weights[:, None] * free_matrix]
    rhs: list[np.ndarray] = [wrench_weights * target]
    if regularization > 0.0:
        square_root = math.sqrt(regularization)
        rows.append(square_root * np.diag(effector_weights[free]))
        rhs.append(square_root * effector_weights[free] * preference[free])
    coefficient = np.vstack(rows)
    target_vector = np.concatenate(rhs)
    try:
        result = lsq_linear(
            coefficient,
            target_vector,
            bounds=(lower[free], upper[free]),
            method="trf",
            lsq_solver="exact",
            tol=1.0e-12,
        )
    except (ArithmeticError, ValueError, np.linalg.LinAlgError) as error:
        values[free] = np.clip(preference[free], lower[free], upper[free])
        return _LeastSquaresSolution(values, 0, f"solver exception: {error}", True)
    values[free] = result.x
    if not result.success or not np.all(np.isfinite(values)):
        values[free] = np.clip(preference[free], lower[free], upper[free])
        return _LeastSquaresSolution(values, int(result.nit), str(result.message), True)
    return _LeastSquaresSolution(values, int(result.nit), str(result.message))
    ####


def _at_limit(value: float, lower: float, upper: float) -> bool:
    """Return whether ``value`` lies on a nondegenerate bound."""

    return upper - lower > 1.0e-12 and (
        math.isclose(value, lower, rel_tol=0.0, abs_tol=1.0e-9)
        or math.isclose(value, upper, rel_tol=0.0, abs_tol=1.0e-9)
    )
    ####


__all__ = [
    "ActuatorAdvanceResult",
    "AllocationStatus",
    "ConstrainedWrenchAllocator",
    "ControlPlantAdapter",
    "DerivativeProvenance",
    "EffectorEffectiveness",
    "EffectorLimits",
    "PhysicalAllocationStep",
    "ProvenancedLinearization",
    "WrenchAllocationResult",
    "advance_actuators",
    "allocate_and_advance_wrench",
    "allocate_bounded_weighted_wrench",
    "finite_difference_linearization_with_provenance",
]
