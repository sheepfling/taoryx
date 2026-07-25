"""Generic plant-bound trim and local-linearization utilities.

Vehicle adapters provide the residual calculation.  This module owns the
solver policy, bounds, scaling, diagnostics, and the finite-difference
linearization used by local LQR design.  It deliberately does not know the
meaning of ``alpha`` versus ``collective`` or any vehicle-specific frame.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field, replace
from typing import Any, Literal

import numpy as np
from scipy.optimize import least_squares

TrimEvaluator = Callable[[Mapping[str, float], Mapping[str, float]], Mapping[str, float]]
DynamicsEvaluator = Callable[[Mapping[str, float], Mapping[str, float]], Mapping[str, float]]
ProcedureEvaluator = Callable[
    [Mapping[str, float], Mapping[str, float], Mapping[str, float | str]],
    Mapping[str, float],
]


@dataclass(frozen=True, slots=True)
class TrimSpec:
    """Named bounded trim problem for one operating point.

    ``state_names`` and ``control_names`` are the plant adapter's canonical
    names.  ``residual_names`` should identify acceleration, force, or moment
    residuals in physically meaningful units.  Values are scaled before the
    least-squares solve, but the unscaled residuals are retained in the result.
    """

    state_names: tuple[str, ...]
    control_names: tuple[str, ...]
    residual_names: tuple[str, ...]
    state_initial: Mapping[str, float]
    control_initial: Mapping[str, float]
    state_lower: Mapping[str, float] | None = None
    state_upper: Mapping[str, float] | None = None
    control_lower: Mapping[str, float] | None = None
    control_upper: Mapping[str, float] | None = None
    residual_scales: Mapping[str, float] | None = None
    x_scale: Mapping[str, float] | None = None

    def __post_init__(self) -> None:
        names = (*self.state_names, *self.control_names)
        if len(set(names)) != len(names):
            raise ValueError("trim state and control names must be unique")
        if not self.residual_names:
            raise ValueError("trim requires at least one residual")
        for name in names:
            value = (self.state_initial if name in self.state_names else self.control_initial).get(name)
            if value is None or not np.isfinite(float(value)):
                raise ValueError(f"trim initial value missing or non-finite for {name!r}")
        for name, value in (self.residual_scales or {}).items():
            if name not in self.residual_names or not np.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"invalid trim residual scale for {name!r}")
        ####
    ####

    @property
    def variable_names(self) -> tuple[str, ...]:
        """Return the ordered state/control vector names."""

        return (*self.state_names, *self.control_names)
        ####

    def initial_vector(self) -> np.ndarray:
        """Return the initial state/control vector in contract order."""

        return np.array(
            [float((self.state_initial if name in self.state_names else self.control_initial)[name]) for name in self.variable_names],
            dtype=float,
        )
        ####

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """Return finite solver bounds, defaulting to unbounded variables."""

        lower = self.state_lower or {}
        upper = self.state_upper or {}
        control_lower = self.control_lower or {}
        control_upper = self.control_upper or {}
        lo: list[float] = []
        hi: list[float] = []
        for name in self.variable_names:
            is_state = name in self.state_names
            lo.append(float((lower if is_state else control_lower).get(name, -np.inf)))
            hi.append(float((upper if is_state else control_upper).get(name, np.inf)))
        result = (np.asarray(lo), np.asarray(hi))
        if np.any(result[0] > result[1]):
            raise ValueError("trim lower bound exceeds upper bound")
        return result
        ####


@dataclass(frozen=True, slots=True)
class TrimResult:
    """Auditable result of a bounded plant trim solve."""

    spec: TrimSpec
    state: Mapping[str, float]
    controls: Mapping[str, float]
    residuals: Mapping[str, float]
    scaled_residual_norm: float
    success: bool
    status: int
    message: str
    iterations: int
    cost: float

    @property
    def max_residual(self) -> float:
        """Return the largest absolute unscaled residual."""

        return max((abs(value) for value in self.residuals.values()), default=0.0)
        ####

    def vector(self) -> np.ndarray:
        """Return the solved state/control vector."""

        return np.array(
            [
                float(self.state[name]) if name in self.state else float(self.controls[name])
                for name in self.spec.variable_names
            ]
        )
        ####

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible, source-independent trim record."""

        return {
            "state": dict(self.state),
            "controls": dict(self.controls),
            "residuals": dict(self.residuals),
            "scaled_residual_norm": self.scaled_residual_norm,
            "max_residual": self.max_residual,
            "success": self.success,
            "status": self.status,
            "message": self.message,
            "iterations": self.iterations,
            "cost": self.cost,
        }
        ####


def solve_trim(
    spec: TrimSpec,
    evaluator: TrimEvaluator,
    *,
    max_nfev: int = 2000,
    residual_tolerance: float = 1e-8,
    acceptance_tolerance: float | None = None,
) -> TrimResult:
    """Solve a bounded trim problem against the supplied plant evaluator.

    The evaluator is called as ``evaluator(state, controls)`` and must return
    every name in ``spec.residual_names``.  The solver never invents vehicle
    equations or changes frames; that remains the adapter's responsibility.
    """

    if residual_tolerance <= 0.0 or not np.isfinite(residual_tolerance):
        raise ValueError("trim residual_tolerance must be positive and finite")
    accepted_norm = residual_tolerance if acceptance_tolerance is None else acceptance_tolerance
    if accepted_norm <= 0.0 or not np.isfinite(accepted_norm):
        raise ValueError("trim acceptance_tolerance must be positive and finite")

    def unpack(vector: np.ndarray) -> tuple[dict[str, float], dict[str, float]]:
        split = len(spec.state_names)
        return (
            dict(zip(spec.state_names, vector[:split], strict=True)),
            dict(zip(spec.control_names, vector[split:], strict=True)),
        )

    scales = {name: float((spec.residual_scales or {}).get(name, 1.0)) for name in spec.residual_names}

    def residual(vector: np.ndarray) -> np.ndarray:
        state, controls = unpack(vector)
        values = evaluator(state, controls)
        missing = set(spec.residual_names) - set(values)
        if missing:
            raise KeyError(f"trim evaluator omitted residuals: {', '.join(sorted(missing))}")
        return np.array([float(values[name]) / scales[name] for name in spec.residual_names], dtype=float)

    lower, upper = spec.bounds()
    initial = np.clip(spec.initial_vector(), lower, upper)
    result = least_squares(
        residual,
        initial,
        bounds=(lower, upper),
        x_scale=np.array([float((spec.x_scale or {}).get(name, 1.0)) for name in spec.variable_names]),
        max_nfev=max_nfev,
        ftol=residual_tolerance,
        xtol=residual_tolerance,
        gtol=residual_tolerance,
    )
    state, controls = unpack(result.x)
    raw = evaluator(state, controls)
    residuals = {name: float(raw[name]) for name in spec.residual_names}
    return TrimResult(
        spec=spec,
        state=state,
        controls=controls,
        residuals=residuals,
        scaled_residual_norm=float(np.linalg.norm(residual(result.x))),
        success=bool(result.success and np.linalg.norm(residual(result.x)) <= accepted_norm),
        status=int(result.status),
        message=str(result.message),
        iterations=int(result.nfev),
        cost=float(result.cost),
    )
    ####


TrimProcedureStatus = Literal[
    "accepted",
    "infeasible",
    "out_of_envelope",
    "numerically_unresolved",
    "adapter_invalid",
]
TrimGateComparison = Literal["minimum", "maximum", "equal"]


@dataclass(frozen=True, slots=True)
class TrimGate:
    """One optional post-solve gate evaluated against adapter diagnostics."""

    id: str
    metric: str
    comparison: TrimGateComparison
    limit: float
    unit: str

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.metric.strip() or not self.unit.strip():
            raise ValueError("trim gates require non-empty id, metric, and unit")
        if not np.isfinite(float(self.limit)):
            raise ValueError("trim gate limit must be finite")
        ####
    ####


@dataclass(frozen=True, slots=True)
class TrimProcedure:
    """Reusable, provenance-bearing policy for one operating-point solve.

    The plant adapter remains outside this object.  A procedure only owns the
    variable contract, operating-point metadata, deterministic search policy,
    and optional gates.  This keeps source equations and frame conversions in
    the family adapter while making trim generation uniform across families.
    """

    id: str
    vehicle: str
    fidelity: str
    spec: TrimSpec
    operating_point: Mapping[str, float | str] = field(default_factory=dict)
    provenance: Mapping[str, str] = field(default_factory=dict)
    multi_start: int = 1
    seed: int = 0
    perturbation_fraction: float = 0.10
    max_nfev: int = 2000
    residual_tolerance: float = 1.0e-8
    acceptance_tolerance: float | None = None
    continuation_axis: str | None = None
    continuation_values: tuple[float, ...] = ()
    gates: tuple[TrimGate, ...] = ()

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.vehicle.strip() or not self.fidelity.strip():
            raise ValueError("trim procedure identity fields must not be empty")
        if self.multi_start < 1:
            raise ValueError("trim procedure multi_start must be at least one")
        if not 0.0 <= self.perturbation_fraction <= 1.0:
            raise ValueError("trim procedure perturbation_fraction must be in [0, 1]")
        if self.max_nfev < 1:
            raise ValueError("trim procedure max_nfev must be positive")
        if self.residual_tolerance <= 0.0 or not np.isfinite(self.residual_tolerance):
            raise ValueError("trim procedure residual_tolerance must be positive and finite")
        if self.acceptance_tolerance is not None and (
            self.acceptance_tolerance <= 0.0 or not np.isfinite(self.acceptance_tolerance)
        ):
            raise ValueError("trim procedure acceptance_tolerance must be positive and finite")
        if self.continuation_values and self.continuation_axis is None:
            raise ValueError("continuation_values require continuation_axis")
        ####
    ####

    def as_dict(self) -> dict[str, Any]:
        """Return the immutable procedure contract as JSON-compatible data."""

        return {
            "id": self.id,
            "vehicle": self.vehicle,
            "fidelity": self.fidelity,
            "operating_point": dict(self.operating_point),
            "provenance": dict(self.provenance),
            "solver": {
                "multi_start": self.multi_start,
                "seed": self.seed,
                "perturbation_fraction": self.perturbation_fraction,
                "max_nfev": self.max_nfev,
                "residual_tolerance": self.residual_tolerance,
                "acceptance_tolerance": self.acceptance_tolerance,
                "continuation_axis": self.continuation_axis,
                "continuation_values": list(self.continuation_values),
            },
            "variables": {
                "state_names": list(self.spec.state_names),
                "control_names": list(self.spec.control_names),
                "residual_names": list(self.spec.residual_names),
                "state_initial": dict(self.spec.state_initial),
                "control_initial": dict(self.spec.control_initial),
                "state_lower": dict(self.spec.state_lower or {}),
                "state_upper": dict(self.spec.state_upper or {}),
                "control_lower": dict(self.spec.control_lower or {}),
                "control_upper": dict(self.spec.control_upper or {}),
                "residual_scales": dict(self.spec.residual_scales or {}),
                "x_scale": dict(self.spec.x_scale or {}),
            },
            "gates": [
                {
                    "id": gate.id,
                    "metric": gate.metric,
                    "comparison": gate.comparison,
                    "limit": gate.limit,
                    "unit": gate.unit,
                }
                for gate in self.gates
            ],
        }
        ####


@dataclass(frozen=True, slots=True)
class TrimGateResult:
    """Evaluation of one procedure gate."""

    id: str
    status: Literal["pass", "fail", "blocked"]
    metric: str
    actual: float | None
    limit: float
    unit: str
    message: str | None = None

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible gate result."""

        return {
            "id": self.id,
            "status": self.status,
            "metric": self.metric,
            "actual": self.actual,
            "limit": self.limit,
            "unit": self.unit,
            "message": self.message,
        }
        ####


@dataclass(frozen=True, slots=True)
class TrimProcedureResult:
    """Auditable result of a deterministic, possibly multi-start procedure."""

    procedure: TrimProcedure
    status: TrimProcedureStatus
    best: TrimResult | None
    attempts: tuple[TrimResult, ...]
    start_vectors: tuple[tuple[float, ...], ...]
    gate_results: tuple[TrimGateResult, ...] = ()
    failure_reason: str | None = None
    continuation: tuple[TrimProcedureResult, ...] = ()

    @property
    def converged(self) -> bool:
        """Return whether the procedure produced an accepted trim."""

        return self.status == "accepted" and self.best is not None
        ####

    def as_dict(self) -> dict[str, Any]:
        """Return a versionable procedure artifact."""

        return {
            "schema_version": 1,
            "procedure": self.procedure.as_dict(),
            "status": self.status,
            "failure_reason": self.failure_reason,
            "best": self.best.as_dict() if self.best is not None else None,
            "attempts": [attempt.as_dict() for attempt in self.attempts],
            "start_vectors": [list(vector) for vector in self.start_vectors],
            "gate_results": [result.as_dict() for result in self.gate_results],
            "continuation": [item.as_dict() for item in self.continuation],
        }
        ####


def _procedure_start_specs(procedure: TrimProcedure) -> tuple[TrimSpec, ...]:
    """Generate deterministic bounded initial guesses for a procedure."""

    base = procedure.spec
    initial = base.initial_vector()
    lower, upper = base.bounds()
    starts = [initial]
    if procedure.multi_start == 1:
        return (base,)
    rng = np.random.default_rng(procedure.seed)
    for _ in range(procedure.multi_start - 1):
        candidate = initial.copy()
        for index, value in enumerate(candidate):
            lo = lower[index]
            hi = upper[index]
            if np.isfinite(lo) and np.isfinite(hi):
                span = hi - lo
                candidate[index] = value + rng.uniform(-1.0, 1.0) * procedure.perturbation_fraction * span
            else:
                scale = max(1.0, abs(value))
                candidate[index] = value + rng.normal(0.0, procedure.perturbation_fraction * scale)
        starts.append(np.clip(candidate, lower, upper))
    specs: list[TrimSpec] = []
    for vector in starts:
        split = len(base.state_names)
        state = dict(zip(base.state_names, vector[:split], strict=True))
        controls = dict(zip(base.control_names, vector[split:], strict=True))
        specs.append(replace(base, state_initial=state, control_initial=controls))
    return tuple(specs)
    ####


def _evaluate_trim_gates(
    gates: Sequence[TrimGate], metrics: Mapping[str, float] | None
) -> tuple[TrimGateResult, ...]:
    """Evaluate declared gates without inventing unavailable metrics."""

    values = metrics or {}
    results: list[TrimGateResult] = []
    for gate in gates:
        if gate.metric not in values:
            results.append(TrimGateResult(gate.id, "blocked", gate.metric, None, gate.limit, gate.unit, "metric unavailable"))
            continue
        actual = float(values[gate.metric])
        if not np.isfinite(actual):
            results.append(TrimGateResult(gate.id, "blocked", gate.metric, actual, gate.limit, gate.unit, "metric is non-finite"))
            continue
        passed = {
            "minimum": actual >= gate.limit,
            "maximum": actual <= gate.limit,
            "equal": np.isclose(actual, gate.limit),
        }[gate.comparison]
        results.append(TrimGateResult(gate.id, "pass" if passed else "fail", gate.metric, actual, gate.limit, gate.unit))
    return tuple(results)
    ####


def solve_trim_procedure(
    procedure: TrimProcedure,
    evaluator: ProcedureEvaluator,
    *,
    metrics: Mapping[str, float] | None = None,
) -> TrimProcedureResult:
    """Solve one operating point using deterministic multi-start and gates."""

    attempts: list[TrimResult] = []
    starts: list[tuple[float, ...]] = []
    try:
        for spec in _procedure_start_specs(procedure):
            starts.append(tuple(float(value) for value in spec.initial_vector()))

            def residual_adapter(
                state: Mapping[str, float],
                controls: Mapping[str, float],
                op: Mapping[str, float | str] = procedure.operating_point,
            ) -> Mapping[str, float]:
                return evaluator(state, controls, op)

            attempts.append(
                solve_trim(
                    spec,
                    residual_adapter,
                    max_nfev=procedure.max_nfev,
                    residual_tolerance=procedure.residual_tolerance,
                    acceptance_tolerance=procedure.acceptance_tolerance,
                )
            )
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        return TrimProcedureResult(
            procedure,
            "adapter_invalid",
            None,
            tuple(attempts),
            tuple(starts),
            failure_reason=str(error),
        )
    successful = tuple(attempt for attempt in attempts if attempt.success)
    best = min(successful, key=lambda attempt: attempt.scaled_residual_norm) if successful else None
    gate_results = _evaluate_trim_gates(procedure.gates, metrics if best is not None else None)
    if best is None:
        status: TrimProcedureStatus = "infeasible" if attempts else "numerically_unresolved"
        reason = "no bounded start converged" if attempts else "no solver attempt completed"
    elif any(item.status == "blocked" for item in gate_results):
        status = "out_of_envelope"
        reason = "one or more declared trim gates were unavailable"
    elif any(item.status == "fail" for item in gate_results):
        status = "out_of_envelope"
        reason = "one or more declared trim gates failed"
    else:
        status = "accepted"
        reason = None
    return TrimProcedureResult(procedure, status, best, tuple(attempts), tuple(starts), gate_results, reason)
    ####


def solve_trim_continuation(
    procedure: TrimProcedure,
    evaluator: ProcedureEvaluator,
    *,
    metrics_by_point: Mapping[float, Mapping[str, float]] | None = None,
) -> TrimProcedureResult:
    """Solve declared operating points by warm-starting each next point.

    Continuation is deliberately explicit: no operating-point values are
    invented and a failed point is retained as the terminal diagnostic.
    """

    if procedure.continuation_axis is None or not procedure.continuation_values:
        return solve_trim_procedure(procedure, evaluator, metrics=(metrics_by_point or {}).get(0.0))
    current = procedure
    results: list[TrimProcedureResult] = []
    for value in procedure.continuation_values:
        operating_point = dict(current.operating_point)
        operating_point[procedure.continuation_axis] = float(value)
        current = replace(current, operating_point=operating_point, multi_start=1)
        result = solve_trim_procedure(
            current,
            evaluator,
            metrics=(metrics_by_point or {}).get(float(value)),
        )
        results.append(result)
        if not result.converged or result.best is None:
            return TrimProcedureResult(
                procedure,
                result.status,
                result.best,
                result.attempts,
                result.start_vectors,
                result.gate_results,
                f"continuation stopped at {procedure.continuation_axis}={value}: {result.failure_reason}",
                tuple(results),
            )
        current = replace(
            current,
            spec=replace(current.spec, state_initial=result.best.state, control_initial=result.best.controls),
        )
    return TrimProcedureResult(
        procedure,
        "accepted",
        results[-1].best if results else None,
        tuple(attempt for result in results for attempt in result.attempts),
        tuple(vector for result in results for vector in result.start_vectors),
        results[-1].gate_results if results else (),
        continuation=tuple(results),
    )
    ####


@dataclass(frozen=True, slots=True)
class DynamicsLinearization:
    """Named state-derivative Jacobian tied to one solved operating point."""

    state_names: tuple[str, ...]
    control_names: tuple[str, ...]
    a_matrix: np.ndarray
    b_matrix: np.ndarray
    trim_state: Mapping[str, float]
    trim_controls: Mapping[str, float]
    metadata: Mapping[str, str | float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.a_matrix.shape != (len(self.state_names), len(self.state_names)):
            raise ValueError("dynamics linearization A shape does not match state names")
        if self.b_matrix.shape != (len(self.state_names), len(self.control_names)):
            raise ValueError("dynamics linearization B shape does not match state/control names")
        if not np.isfinite(self.a_matrix).all() or not np.isfinite(self.b_matrix).all():
            raise ValueError("dynamics linearization matrices must be finite")
        ####
    ####

    @property
    def metadata_dict(self) -> dict[str, str | float]:
        """Return provenance metadata without exposing mutable mapping state."""

        return dict(self.metadata)
    ####


def finite_difference_linearization(
    spec: TrimSpec,
    evaluator: TrimEvaluator,
    result: TrimResult,
    *,
    state_step: float = 1e-6,
    control_step: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray]:
    """Return local ``A`` and ``B`` residual Jacobians around a trim result.

    This is a deliberately explicit local plant approximation.  Callers must
    map residual names to actual state derivatives before using the matrices
    for LQR; a force/moment residual Jacobian is not automatically a dynamics
    Jacobian.
    """

    x = result.vector()
    state_count = len(spec.state_names)
    control_count = len(spec.control_names)
    base_state = dict(result.state)
    base_controls = dict(result.controls)

    def evaluate(vector: np.ndarray) -> np.ndarray:
        split = state_count
        state = dict(zip(spec.state_names, vector[:split], strict=True))
        controls = dict(zip(spec.control_names, vector[split:], strict=True))
        values = evaluator(state, controls)
        return np.array([float(values[name]) for name in spec.residual_names], dtype=float)

    base = evaluate(x)
    a = np.zeros((len(spec.residual_names), state_count))
    b = np.zeros((len(spec.residual_names), control_count))
    for index in range(state_count):
        step = state_step * max(1.0, abs(x[index]))
        plus = x.copy(); plus[index] += step
        minus = x.copy(); minus[index] -= step
        a[:, index] = (evaluate(plus) - evaluate(minus)) / (2.0 * step)
    for index in range(control_count):
        position = state_count + index
        step = control_step * max(1.0, abs(x[position]))
        plus = x.copy(); plus[position] += step
        minus = x.copy(); minus[position] -= step
        b[:, index] = (evaluate(plus) - evaluate(minus)) / (2.0 * step)
    del base, base_state, base_controls
    return a, b
    ####


def finite_difference_dynamics_linearization(
    spec: TrimSpec,
    evaluator: DynamicsEvaluator,
    result: TrimResult,
    *,
    state_step: float = 1.0e-6,
    control_step: float = 1.0e-6,
    metadata: Mapping[str, str | float] | None = None,
) -> DynamicsLinearization:
    """Finite-difference true state derivatives for source-trim LQR design.

    Unlike :func:`finite_difference_linearization`, this route requires the
    evaluator to return derivatives named exactly like ``spec.state_names``.
    It therefore produces a dynamics ``A/B`` pair suitable for LQR without
    silently treating force or moment residuals as state derivatives.
    """

    if state_step <= 0.0 or control_step <= 0.0 or not np.isfinite(state_step + control_step):
        raise ValueError("dynamics linearization steps must be finite and positive")
    state_count = len(spec.state_names)
    control_count = len(spec.control_names)
    x = result.vector()

    def evaluate(vector: np.ndarray) -> np.ndarray:
        split = state_count
        state = dict(zip(spec.state_names, vector[:split], strict=True))
        controls = dict(zip(spec.control_names, vector[split:], strict=True))
        values = evaluator(state, controls)
        missing = set(spec.state_names) - set(values)
        if missing:
            raise KeyError("dynamics evaluator omitted derivatives: " + ", ".join(sorted(missing)))
        return np.array([float(values[name]) for name in spec.state_names], dtype=float)

    a = np.zeros((state_count, state_count), dtype=float)
    b = np.zeros((state_count, control_count), dtype=float)
    for index in range(state_count):
        step = state_step * max(1.0, abs(x[index]))
        plus = x.copy()
        minus = x.copy()
        plus[index] += step
        minus[index] -= step
        a[:, index] = (evaluate(plus) - evaluate(minus)) / (2.0 * step)
    for index in range(control_count):
        position = state_count + index
        step = control_step * max(1.0, abs(x[position]))
        plus = x.copy()
        minus = x.copy()
        plus[position] += step
        minus[position] -= step
        b[:, index] = (evaluate(plus) - evaluate(minus)) / (2.0 * step)
    return DynamicsLinearization(
        spec.state_names,
        spec.control_names,
        a,
        b,
        dict(result.state),
        dict(result.controls),
        dict(metadata or {}),
    )
    ####
