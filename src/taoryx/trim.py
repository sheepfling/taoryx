"""Generic plant-bound trim and local-linearization utilities.

Vehicle adapters provide the residual calculation.  This module owns the
solver policy, bounds, scaling, diagnostics, and the finite-difference
linearization used by local LQR design.  It deliberately does not know the
meaning of ``alpha`` versus ``collective`` or any vehicle-specific frame.
"""

from __future__ import annotations

import dataclasses
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
TrimDiagnosticSeverity = Literal["error", "warning", "info"]


@dataclass(frozen=True, slots=True)
class TrimDiagnostic:
    """Machine-readable trim setup or solve guidance."""

    code: str
    severity: TrimDiagnosticSeverity
    message: str
    action: str
    field: str | None = None
    details: Mapping[str, float | str] = dataclasses.field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.code.strip() or not self.message.strip() or not self.action.strip():
            raise ValueError("trim diagnostics require code, message, and action")
        if self.field is not None and not self.field.strip():
            raise ValueError("trim diagnostic field must not be blank")

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible diagnostic record."""

        return {
            "code": self.code,
            "severity": self.severity,
            "message": self.message,
            "action": self.action,
            "field": self.field,
            "details": dict(self.details),
        }


class TrimConfigurationError(ValueError):
    """Invalid static trim configuration with a repair-oriented diagnostic."""

    def __init__(self, code: str, message: str, action: str, *, field: str | None = None) -> None:
        self.diagnostic = TrimDiagnostic(code, "error", message, action, field)
        location = f" field={field!r}" if field is not None else ""
        super().__init__(f"[trim:{code}]{location} {message} Fix: {action}")


class TrimEvaluationError(ValueError):
    """Plant-evaluator failure with a repair-oriented diagnostic."""

    def __init__(self, code: str, message: str, action: str, *, field: str | None = None) -> None:
        self.diagnostic = TrimDiagnostic(code, "error", message, action, field)
        location = f" field={field!r}" if field is not None else ""
        super().__init__(f"[trim:{code}]{location} {message} Fix: {action}")


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
    operating_point: Mapping[str, float | str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        names = (*self.state_names, *self.control_names)
        if len(set(names)) != len(names):
            raise TrimConfigurationError(
                "duplicate-variable-name",
                "trim state and control names must be unique",
                "rename the repeated channel so it appears in exactly one ordered state/control list",
                field="state_names/control_names",
            )
        if not self.residual_names:
            raise TrimConfigurationError(
                "missing-residuals",
                "trim requires at least one residual",
                "declare the force, acceleration, or moment residuals that define equilibrium",
                field="residual_names",
            )
        if len(set(self.residual_names)) != len(self.residual_names):
            raise TrimConfigurationError(
                "duplicate-residual-name",
                "trim residual names must be unique",
                "keep one residual channel per physical equation and rename duplicates",
                field="residual_names",
            )
        for name in names:
            value = (self.state_initial if name in self.state_names else self.control_initial).get(name)
            if value is None:
                raise TrimConfigurationError(
                    "invalid-initial-value",
                    f"trim initial value for {name!r} is missing",
                    "provide one finite initial value in the same units and frame used by the plant adapter",
                    field=name,
                )
            try:
                numeric = float(value)
            except (TypeError, ValueError) as error:
                raise TrimConfigurationError(
                    "non-numeric-initial-value",
                    f"trim initial value for {name!r} is not numeric",
                    "provide a scalar number in the same units and frame used by the plant adapter",
                    field=name,
                ) from error
            if not np.isfinite(numeric):
                raise TrimConfigurationError(
                    "invalid-initial-value",
                    f"trim initial value for {name!r} is non-finite",
                    "provide one finite initial value in the same units and frame used by the plant adapter",
                    field=name,
                )
        variable_names = set(names)
        for label, values in (
            ("state_lower", self.state_lower),
            ("state_upper", self.state_upper),
            ("control_lower", self.control_lower),
            ("control_upper", self.control_upper),
        ):
            unknown = sorted(set(values or {}) - variable_names)
            if unknown:
                raise TrimConfigurationError(
                    "unknown-bound-variable",
                    f"{label} names variables that are not in the trim state/control contract: {', '.join(unknown)}",
                    "match every bound key to a declared state or control channel",
                    field=label,
                )
            for name, value in (values or {}).items():
                try:
                    numeric = float(value)
                except (TypeError, ValueError) as error:
                    raise TrimConfigurationError(
                        "non-numeric-bound",
                        f"{label}[{name!r}] is not numeric",
                        "replace the bound with a scalar number in the adapter's declared units",
                        field=f"{label}.{name}",
                    ) from error
                if np.isnan(numeric):
                    raise TrimConfigurationError(
                        "nonfinite-bound",
                        f"{label}[{name!r}] is NaN",
                        "replace NaN with a finite bound or the appropriate signed infinity",
                        field=f"{label}.{name}",
                    )
        lower = self.state_lower or {}
        upper = self.state_upper or {}
        control_lower = self.control_lower or {}
        control_upper = self.control_upper or {}
        for name in self.state_names:
            if float(lower.get(name, -np.inf)) > float(upper.get(name, np.inf)):
                raise TrimConfigurationError(
                    "inverted-state-bounds",
                    f"state bounds for {name!r} have lower greater than upper",
                    "swap the bounds or correct the units before solving",
                    field=name,
                )
        for name in self.control_names:
            if float(control_lower.get(name, -np.inf)) > float(control_upper.get(name, np.inf)):
                raise TrimConfigurationError(
                    "inverted-control-bounds",
                    f"control bounds for {name!r} have lower greater than upper",
                    "swap the bounds or correct the units before solving",
                    field=name,
                )
        for name, value in (self.residual_scales or {}).items():
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                numeric = float("nan")
            if name not in self.residual_names or not np.isfinite(numeric) or numeric <= 0.0:
                raise TrimConfigurationError(
                    "invalid-residual-scale",
                    f"trim residual scale for {name!r} is unknown, non-finite, or non-positive",
                    "declare one positive scale per residual in its physical units",
                    field=f"residual_scales.{name}",
                )
        for name, value in (self.x_scale or {}).items():
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                numeric = float("nan")
            if name not in variable_names or not np.isfinite(numeric) or numeric <= 0.0:
                raise TrimConfigurationError(
                    "invalid-variable-scale",
                    f"trim x scale for {name!r} is unknown, non-finite, or non-positive",
                    "declare a positive characteristic magnitude for each scaled solver variable",
                    field=f"x_scale.{name}",
                )
        ####
    ####

    def setup_diagnostics(self) -> tuple[TrimDiagnostic, ...]:
        """Return warnings that do not make the trim contract invalid."""

        lower, upper = self.bounds()
        initial = self.initial_vector()
        diagnostics: list[TrimDiagnostic] = []
        for name, value, lo, hi in zip(self.variable_names, initial, lower, upper, strict=True):
            if value < lo or value > hi:
                details: dict[str, float] = {"initial": float(value)}
                if np.isfinite(lo):
                    details["lower"] = float(lo)
                if np.isfinite(hi):
                    details["upper"] = float(hi)
                diagnostics.append(
                    TrimDiagnostic(
                        "initial-value-clipped",
                        "warning",
                        f"initial value for {name!r} lies outside its declared bounds and will be clipped",
                        "move the initial guess into the expected physical envelope; clipping can hide a poor starting condition",
                        name,
                        details,
                    )
                )
        return tuple(diagnostics)

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
    diagnostics: tuple[TrimDiagnostic, ...] = ()

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
            "operating_point": dict(self.spec.operating_point),
            "residuals": dict(self.residuals),
            "scaled_residual_norm": self.scaled_residual_norm,
            "max_residual": self.max_residual,
            "success": self.success,
            "status": self.status,
            "message": self.message,
            "iterations": self.iterations,
            "cost": self.cost,
            "diagnostics": [diagnostic.as_dict() for diagnostic in self.diagnostics],
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
        raise TrimConfigurationError(
            "invalid-residual-tolerance",
            "trim residual_tolerance must be positive and finite",
            "set residual_tolerance to a finite positive value in normalized residual units",
            field="residual_tolerance",
        )
    accepted_norm = residual_tolerance if acceptance_tolerance is None else acceptance_tolerance
    if accepted_norm <= 0.0 or not np.isfinite(accepted_norm):
        raise TrimConfigurationError(
            "invalid-acceptance-tolerance",
            "trim acceptance_tolerance must be positive and finite",
            "set acceptance_tolerance to a finite positive residual norm appropriate for the plant units",
            field="acceptance_tolerance",
        )

    def unpack(vector: np.ndarray) -> tuple[dict[str, float], dict[str, float]]:
        split = len(spec.state_names)
        return (
            dict(zip(spec.state_names, vector[:split], strict=True)),
            dict(zip(spec.control_names, vector[split:], strict=True)),
        )

    scales = {name: float((spec.residual_scales or {}).get(name, 1.0)) for name in spec.residual_names}

    def evaluate_values(state: Mapping[str, float], controls: Mapping[str, float]) -> Mapping[str, float]:
        try:
            values = evaluator(state, controls)
        except TrimEvaluationError:
            raise
        except Exception as error:
            detail = str(error).strip() or type(error).__name__
            raise TrimEvaluationError(
                "evaluator-failed",
                f"plant evaluator failed while evaluating the trim candidate: {detail}",
                "verify the fidelity adapter's frames, units, table coverage, and required mass/propulsion inputs",
            ) from error
        if not isinstance(values, Mapping):
            raise TrimEvaluationError(
                "invalid-evaluator-result",
                "plant evaluator did not return a mapping of named residuals",
                "return one finite numeric residual mapping keyed by TrimSpec.residual_names",
            )
        missing = set(spec.residual_names) - set(values)
        if missing:
            raise TrimEvaluationError(
                "missing-residual",
                f"plant evaluator omitted declared residuals: {', '.join(sorted(missing))}",
                "return every residual named by TrimSpec in the declared physical units",
                field="residual_names",
            )
        for name in spec.residual_names:
            try:
                numeric = float(values[name])
            except (TypeError, ValueError) as error:
                raise TrimEvaluationError(
                    "non-numeric-residual",
                    f"residual {name!r} is not numeric",
                    "convert the adapter output to a scalar float before returning it",
                    field=name,
                ) from error
            if not np.isfinite(numeric):
                raise TrimEvaluationError(
                    "nonfinite-residual",
                    f"residual {name!r} is non-finite",
                    "check table interpolation, atmosphere/propulsion inputs, and frame/unit conversions at this candidate",
                    field=name,
                )
        return values

    scales = {name: float((spec.residual_scales or {}).get(name, 1.0)) for name in spec.residual_names}

    def residual(vector: np.ndarray) -> np.ndarray:
        state, controls = unpack(vector)
        values = evaluate_values(state, controls)
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
    raw = evaluate_values(state, controls)
    residuals = {name: float(raw[name]) for name in spec.residual_names}
    scaled_residual_norm = float(np.linalg.norm(np.array([residuals[name] / scales[name] for name in spec.residual_names], dtype=float)))
    diagnostics = list(spec.setup_diagnostics())
    if not result.success or scaled_residual_norm > accepted_norm:
        if result.status == 0:
            diagnostics.append(
                TrimDiagnostic(
                    "solver-max-evaluations",
                    "error",
                    "trim solver reached max_nfev before satisfying the acceptance gate",
                    "improve the initial guess or scaling and then increase max_nfev; do not use more evaluations to hide an invalid model",
                    field="max_nfev",
                    details={"max_nfev": max_nfev},
                )
            )
        if scaled_residual_norm > accepted_norm:
            worst = max(spec.residual_names, key=lambda name: abs(residuals[name] / scales[name]))
            diagnostics.append(
                TrimDiagnostic(
                    "residual-above-tolerance",
                    "error",
                    f"best candidate residual norm {scaled_residual_norm:.6g} exceeds acceptance tolerance {accepted_norm:.6g}; worst residual is {worst!r}",
                    "check the equation balance, table envelope, operating-point assumptions, bounds, and residual scales before loosening tolerance",
                    field=worst,
                    details={"scaled_residual_norm": scaled_residual_norm, "acceptance_tolerance": accepted_norm, "residual": residuals[worst]},
                )
            )
        if not result.success and result.status != 0:
            diagnostics.append(
                TrimDiagnostic(
                    "solver-not-converged",
                    "error",
                    f"trim solver terminated without an accepted convergence status: {result.message}",
                    "inspect the residual trend and variable scales; use continuation or a better physical initial guess",
                    field="solver",
                )
            )
    for name, value, lo, hi in zip(spec.variable_names, result.x, lower, upper, strict=True):
        scale = max(1.0, abs(float(value)), abs(float(lo)) if np.isfinite(lo) else 0.0, abs(float(hi)) if np.isfinite(hi) else 0.0)
        if (np.isfinite(lo) and abs(float(value) - float(lo)) <= 1.0e-8 * scale) or (np.isfinite(hi) and abs(float(value) - float(hi)) <= 1.0e-8 * scale):
            details = {"value": float(value)}
            if np.isfinite(lo):
                details["lower"] = float(lo)
            if np.isfinite(hi):
                details["upper"] = float(hi)
            diagnostics.append(
                TrimDiagnostic(
                    "solution-at-bound",
                    "warning",
                    f"trim solution for {name!r} is at a declared bound",
                    "confirm the bound is physically intended; otherwise widen the envelope or revise the initial condition/model",
                    field=name,
                    details=details,
                )
            )
    return TrimResult(
        spec=spec,
        state=state,
        controls=controls,
        residuals=residuals,
        scaled_residual_norm=scaled_residual_norm,
        success=bool(result.success and scaled_residual_norm <= accepted_norm),
        status=int(result.status),
        message=str(result.message),
        iterations=int(result.nfev),
        cost=float(result.cost),
        diagnostics=tuple(diagnostics),
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
            raise TrimConfigurationError(
                "incomplete-trim-gate",
                "trim gates require non-empty id, metric, and unit",
                "declare a stable gate ID, the metric published by the plant adapter, and its physical unit",
                field="gates",
            )
        if not np.isfinite(float(self.limit)):
            raise TrimConfigurationError(
                "nonfinite-gate-limit",
                "trim gate limit must be finite",
                "provide a finite physical acceptance limit",
                field=f"gates.{self.id}.limit",
            )
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
            raise TrimConfigurationError(
                "missing-procedure-identity",
                "trim procedure id, vehicle, and fidelity must not be empty",
                "declare a stable procedure ID, vehicle ID, and one of point_mass_3dof, pseudo_6dof, or rigid_body_6dof",
                field="id/vehicle/fidelity",
            )
        if self.multi_start < 1:
            raise TrimConfigurationError(
                "invalid-multistart",
                "trim procedure multi_start must be at least one",
                "use one start for a warm-started continuation point or two or more for deterministic multi-start search",
                field="multi_start",
            )
        if not 0.0 <= self.perturbation_fraction <= 1.0:
            raise TrimConfigurationError(
                "invalid-perturbation-fraction",
                "trim procedure perturbation_fraction must be in [0, 1]",
                "set the perturbation fraction relative to bounded variable spans",
                field="perturbation_fraction",
            )
        if self.max_nfev < 1:
            raise TrimConfigurationError(
                "invalid-max-evaluations",
                "trim procedure max_nfev must be positive",
                "set a positive solver evaluation budget after correcting scaling and the initial guess",
                field="max_nfev",
            )
        if self.residual_tolerance <= 0.0 or not np.isfinite(self.residual_tolerance):
            raise TrimConfigurationError(
                "invalid-residual-tolerance",
                "trim procedure residual_tolerance must be positive and finite",
                "set a positive normalized residual tolerance appropriate for this fidelity",
                field="residual_tolerance",
            )
        if self.acceptance_tolerance is not None and (
            self.acceptance_tolerance <= 0.0 or not np.isfinite(self.acceptance_tolerance)
        ):
            raise TrimConfigurationError(
                "invalid-acceptance-tolerance",
                "trim procedure acceptance_tolerance must be positive and finite",
                "set a positive accepted residual norm or leave it equal to residual_tolerance",
                field="acceptance_tolerance",
            )
        if self.continuation_values and self.continuation_axis is None:
            raise TrimConfigurationError(
                "missing-continuation-axis",
                "continuation_values require continuation_axis",
                "name the operating-point field that the evaluator reads at each continuation value",
                field="continuation_axis",
            )
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
    diagnostics: tuple[TrimDiagnostic, ...] = ()

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
            "diagnostics": [diagnostic.as_dict() for diagnostic in self.diagnostics],
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
        diagnostic = error.diagnostic if isinstance(error, (TrimConfigurationError, TrimEvaluationError)) else TrimDiagnostic(
            "adapter-invalid",
            "error",
            f"trim adapter rejected the candidate: {error}",
            "inspect the fidelity adapter contract, especially named residuals, units, frames, and table coverage",
            field="evaluator",
        )
        return TrimProcedureResult(
            procedure,
            "adapter_invalid",
            None,
            tuple(attempts),
            tuple(starts),
            failure_reason=str(error),
            diagnostics=(diagnostic,),
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
    diagnostics: list[TrimDiagnostic] = []
    for attempt in attempts:
        diagnostics.extend(attempt.diagnostics)
    if status == "out_of_envelope":
        for gate in gate_results:
            if gate.status == "blocked":
                diagnostics.append(
                    TrimDiagnostic(
                        "gate-metric-unavailable",
                        "error",
                        f"declared trim gate {gate.id!r} could not evaluate metric {gate.metric!r}",
                        "publish the metric from the plant adapter or remove the gate from this fidelity profile",
                        field=f"gates.{gate.id}",
                    )
                )
            elif gate.status == "fail":
                diagnostics.append(
                    TrimDiagnostic(
                        "gate-failed",
                        "error",
                        f"declared trim gate {gate.id!r} failed: {gate.metric}={gate.actual} {gate.unit}, limit {gate.limit}",
                        "treat the operating point as outside the claimed envelope or correct the plant data/control authority",
                        field=f"gates.{gate.id}",
                    )
                )
    if best is None and attempts and not diagnostics:
        diagnostics.append(
            TrimDiagnostic(
                "no-accepted-attempt",
                "error",
                "no multi-start trim attempt satisfied the acceptance gate",
                "compare the attempt residuals and bounds, then revise the initial guesses, scaling, or operating point",
                field="multi_start",
            )
        )
    return TrimProcedureResult(procedure, status, best, tuple(attempts), tuple(starts), gate_results, reason, diagnostics=tuple(diagnostics))
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
                result.diagnostics,
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
        diagnostics=tuple(diagnostic for result in results for diagnostic in result.diagnostics),
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
            raise TrimConfigurationError(
                "linearization-a-shape-mismatch",
                "dynamics linearization A shape does not match state names",
                "build A with one row and column for every ordered state channel",
                field="a_matrix",
            )
        if self.b_matrix.shape != (len(self.state_names), len(self.control_names)):
            raise TrimConfigurationError(
                "linearization-b-shape-mismatch",
                "dynamics linearization B shape does not match state/control names",
                "build B with one row per state and one column per ordered control channel",
                field="b_matrix",
            )
        if not np.isfinite(self.a_matrix).all() or not np.isfinite(self.b_matrix).all():
            raise TrimConfigurationError(
                "nonfinite-linearization",
                "dynamics linearization matrices must be finite",
                "check derivative units, table perturbations, and the operating point before designing a controller",
                field="a_matrix/b_matrix",
            )
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
        try:
            values = evaluator(state, controls)
        except Exception as error:
            raise TrimEvaluationError(
                "linearization-evaluator-failed",
                f"plant evaluator failed during finite-difference perturbation: {error}",
                "reduce the perturbation only after verifying the trim state, table envelope, and source frame conventions",
                field="evaluator",
            ) from error
        if not isinstance(values, Mapping):
            raise TrimEvaluationError(
                "linearization-invalid-result",
                "linearization evaluator did not return a mapping of named residuals",
                "return one finite numeric residual mapping keyed by TrimSpec.residual_names",
                field="evaluator",
            )
        missing = set(spec.residual_names) - set(values)
        if missing:
            raise TrimEvaluationError(
                "linearization-missing-residual",
                "linearization evaluator omitted residuals: " + ", ".join(sorted(missing)),
                "return every residual named by TrimSpec; force/moment residuals still need a separate dynamics mapping for LQR",
                field="residual_names",
            )
        try:
            residual_values = [float(values[name]) for name in spec.residual_names]
        except (TypeError, ValueError) as error:
            raise TrimEvaluationError(
                "linearization-nonnumeric-residual",
                "linearization evaluator returned a non-numeric residual",
                "return finite scalar residuals keyed by the ordered TrimSpec.residual_names",
                field="residuals",
            ) from error
        if not np.isfinite(residual_values).all():
            raise TrimEvaluationError(
                "linearization-nonfinite-residual",
                "linearization evaluator returned a non-finite residual",
                "check table interpolation and perturbation points remain inside the declared plant envelope",
                field="residuals",
            )
        return np.array(residual_values, dtype=float)

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
        raise TrimConfigurationError(
            "invalid-linearization-step",
            "dynamics linearization steps must be finite and positive",
            "choose perturbations large enough to rise above numerical noise but small enough to remain local",
            field="state_step/control_step",
        )
    state_count = len(spec.state_names)
    control_count = len(spec.control_names)
    x = result.vector()

    def evaluate(vector: np.ndarray) -> np.ndarray:
        split = state_count
        state = dict(zip(spec.state_names, vector[:split], strict=True))
        controls = dict(zip(spec.control_names, vector[split:], strict=True))
        try:
            values = evaluator(state, controls)
        except Exception as error:
            raise TrimEvaluationError(
                "dynamics-evaluator-failed",
                f"dynamics evaluator failed during finite-difference perturbation: {error}",
                "return named state derivatives at both perturbation points and keep them in consistent SI units",
                field="evaluator",
            ) from error
        if not isinstance(values, Mapping):
            raise TrimEvaluationError(
                "invalid-linearization-result",
                "dynamics evaluator did not return a mapping of named state derivatives",
                "return one finite numeric derivative mapping keyed by TrimSpec.state_names",
                field="evaluator",
            )
        missing = set(spec.state_names) - set(values)
        if missing:
            raise TrimEvaluationError(
                "missing-state-derivative",
                "dynamics evaluator omitted derivatives: " + ", ".join(sorted(missing)),
                "return one derivative for every state channel; do not pass force/moment residuals as A/B dynamics",
                field="state_names",
            )
        try:
            derivatives = np.array([float(values[name]) for name in spec.state_names], dtype=float)
        except (TypeError, ValueError) as error:
            raise TrimEvaluationError(
                "non-numeric-state-derivative",
                "dynamics evaluator returned a non-numeric state derivative",
                "return finite scalar derivatives keyed by the ordered state names",
                field="state_derivatives",
            ) from error
        if not np.isfinite(derivatives).all():
            raise TrimEvaluationError(
                "nonfinite-state-derivative",
                "dynamics evaluator returned a non-finite state derivative",
                "check the perturbed state remains valid and that no table or frame conversion returns NaN/inf",
                field="state_derivatives",
            )
        return derivatives

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
