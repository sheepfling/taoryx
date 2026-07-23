"""Generic plant-bound trim and local-linearization utilities.

Vehicle adapters provide the residual calculation.  This module owns the
solver policy, bounds, scaling, diagnostics, and the finite-difference
linearization used by local LQR design.  It deliberately does not know the
meaning of ``alpha`` versus ``collective`` or any vehicle-specific frame.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

TrimEvaluator = Callable[[Mapping[str, float], Mapping[str, float]], Mapping[str, float]]
DynamicsEvaluator = Callable[[Mapping[str, float], Mapping[str, float]], Mapping[str, float]]


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
