"""Reusable local direct-wrench controller witness.

This is the deliberately narrow bridge between a source-load adapter and a
full vehicle mission executor.  It evaluates a *local* nonlinear velocity and
rate plant, derives an LQR from that same plant, projects each requested body
wrench through explicit limits, and records requested-versus-achieved
authority.  It does not add navigation, trim, propulsion scheduling, or
physical effector allocation that the source adapter does not provide.

The helper exists so a source-backed direct-wrench screen is not reimplemented
per vehicle in a ``tools/`` script.  A caller supplies all vehicle-dependent
data: state ordering, source derivative, equilibrium bridge bias, limits, and
LQR weights/scales.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass

import numpy as np

from .direct_wrench import DIRECT_WRENCH_NAMES, DirectWrenchLimits, DirectWrenchProjection
from .runtime.lqr import LqrResult, solve_scaled_continuous_lqr

LocalDerivative = Callable[[Mapping[str, float], Mapping[str, float]], Mapping[str, float]]
LocalWrenchBias = Callable[[Mapping[str, float]], Mapping[str, float]]


def _named_values(values: Mapping[str, float], names: tuple[str, ...], label: str) -> dict[str, float]:
    """Validate a complete finite named vector and preserve its ordering."""

    missing = set(names) - set(values)
    extra = set(values) - set(names)
    if missing or extra:
        raise ValueError(f"{label} axes mismatch; missing={sorted(missing)}, extra={sorted(extra)}")
    result = {name: float(values[name]) for name in names}
    if any(not math.isfinite(value) for value in result.values()):
        raise ValueError(f"{label} values must be finite")
    return result
    ####


@dataclass(frozen=True, slots=True)
class LocalDirectWrenchScreenConfig:
    """Vehicle-supplied contract for one local direct-wrench witness."""

    id: str
    plant_id: str
    state_names: tuple[str, ...]
    reference_state: Mapping[str, float]
    initial_state: Mapping[str, float]
    source_derivative: LocalDerivative
    balancing_wrench: LocalWrenchBias
    limits: DirectWrenchLimits
    state_scales: tuple[float, ...]
    control_scales: tuple[float, ...]
    state_cost_weights: tuple[float, ...]
    control_cost_weights: tuple[float, ...]
    dt_s: float
    duration_s: float
    state_derivative_step: float = 1.0e-5
    control_derivative_step: float = 1.0e-5
    final_error_fraction_limit: float = 0.25
    equilibrium_derivative_norm_limit: float = 1.0e-6

    def __post_init__(self) -> None:
        if not self.id or not self.plant_id:
            raise ValueError("local direct-wrench screen id and plant_id must be nonempty")
        if not self.state_names or len(set(self.state_names)) != len(self.state_names):
            raise ValueError("local direct-wrench screen state names must be nonempty and unique")
        count = len(self.state_names)
        for label, values in (
            ("state_scales", self.state_scales),
            ("state_cost_weights", self.state_cost_weights),
        ):
            if len(values) != count or any(not math.isfinite(value) or value <= 0.0 for value in values):
                raise ValueError(f"{label} must contain one positive finite value per state")
        for label, values in (
            ("control_scales", self.control_scales),
            ("control_cost_weights", self.control_cost_weights),
        ):
            if len(values) != len(DIRECT_WRENCH_NAMES) or any(not math.isfinite(value) or value <= 0.0 for value in values):
                raise ValueError(f"{label} must contain six positive finite wrench values")
        _named_values(self.reference_state, self.state_names, "reference state")
        _named_values(self.initial_state, self.state_names, "initial state")
        for label, value in (
            ("dt_s", self.dt_s),
            ("duration_s", self.duration_s),
            ("state_derivative_step", self.state_derivative_step),
            ("control_derivative_step", self.control_derivative_step),
            ("final_error_fraction_limit", self.final_error_fraction_limit),
            ("equilibrium_derivative_norm_limit", self.equilibrium_derivative_norm_limit),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{label} must be finite and positive")
        ####
    ####


@dataclass(frozen=True, slots=True)
class LocalDirectWrenchScreenExecution:
    """Auditable result of one source-backed local controller witness."""

    config: LocalDirectWrenchScreenConfig
    bias_wrench: Mapping[str, float]
    a_matrix: np.ndarray
    b_matrix: np.ndarray
    lqr: LqrResult
    rows: tuple[dict[str, object], ...]
    initial_error_norm: float
    final_error_norm: float
    equilibrium_projection: DirectWrenchProjection
    equilibrium_derivative: Mapping[str, float]
    equilibrium_derivative_norm: float
    equilibrium_pass: bool
    mission_pass: bool

    @property
    def observed_statuses(self) -> tuple[str, ...]:
        """Return direct-wrench projection statuses seen during the run."""

        return tuple(sorted({str(_mapping(row["wrench"], "wrench")["status"]) for row in self.rows}))
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a stable machine-readable execution record."""

        return {
            "schema": "taoryx.local-direct-wrench-screen/v1alpha1",
            "id": self.config.id,
            "plant_id": self.config.plant_id,
            "fidelity": "rigid_body_6dof_direct_wrench",
            "control_realization": "direct_wrench",
            "physical_effector_allocation": False,
            "state_names": list(self.config.state_names),
            "direct_wrench_names": list(DIRECT_WRENCH_NAMES),
            "reference_state": dict(self.config.reference_state),
            "initial_state": dict(self.config.initial_state),
            "bias_wrench": dict(self.bias_wrench),
            "limits": {
                "lower": dict(self.config.limits.lower),
                "upper": dict(self.config.limits.upper),
                "rate_limit_per_s": dict(self.config.limits.rate_limit_per_s),
            },
            "linearization": {
                "a_matrix": self.a_matrix.tolist(),
                "b_matrix": self.b_matrix.tolist(),
                "gain": np.asarray(self.lqr.gain).tolist(),
                "closed_loop_poles": [
                    {"real": float(value.real), "imaginary": float(value.imag)}
                    for value in self.lqr.closed_loop_eigenvalues
                ],
                "maximum_real_pole": self.lqr.maximum_real_pole,
                "controllable": self.lqr.controllable,
            },
            "evaluation": {
                "mission_pass": self.mission_pass,
                "equilibrium": {
                    "passed": self.equilibrium_pass,
                    "requested_balancing_wrench": dict(self.equilibrium_projection.requested),
                    "achieved_balancing_wrench": dict(self.equilibrium_projection.achieved),
                    "projection_status": self.equilibrium_projection.status,
                    "projection_residual_norm": self.equilibrium_projection.residual_norm,
                    "reference_derivative": dict(self.equilibrium_derivative),
                    "reference_derivative_normalized_norm": self.equilibrium_derivative_norm,
                    "reference_derivative_normalized_norm_limit": self.config.equilibrium_derivative_norm_limit,
                },
                "initial_error_norm": self.initial_error_norm,
                "final_error_norm": self.final_error_norm,
                "final_error_fraction": self.final_error_norm / max(self.initial_error_norm, 1.0e-12),
                "final_error_fraction_limit": self.config.final_error_fraction_limit,
                "sample_count": len(self.rows),
                "dt_s": self.config.dt_s,
                "duration_s": self.config.duration_s,
                "wrench_statuses_observed": list(self.observed_statuses),
            },
            "telemetry": list(self.rows),
            "claim_boundary": (
                "This is a local source-load direct-wrench controller witness. It does not establish physical "
                "effector allocation, a source physical trim, navigation, propulsion scheduling, an end-to-end "
                "mission, or family qualification."
            ),
        }
        ####
    ####


def run_local_direct_wrench_screen(config: LocalDirectWrenchScreenConfig) -> LocalDirectWrenchScreenExecution:
    """Run one bounded LQR recovery witness against a supplied local plant."""

    reference = _named_values(config.reference_state, config.state_names, "reference state")
    state = _named_values(config.initial_state, config.state_names, "initial state")
    bias = _named_values(config.balancing_wrench(reference), DIRECT_WRENCH_NAMES, "balancing wrench")
    zero_wrench = {name: 0.0 for name in DIRECT_WRENCH_NAMES}
    equilibrium_projection = config.limits.project(bias, zero_wrench, 1.0)
    equilibrium_derivative = _named_values(
        config.source_derivative(reference, equilibrium_projection.achieved),
        config.state_names,
        "reference source derivative",
    )
    equilibrium_derivative_norm = _error_norm(
        equilibrium_derivative,
        {name: 0.0 for name in config.state_names},
        config.state_names,
        config.state_scales,
    )
    equilibrium_pass = (
        equilibrium_projection.status == "feasible"
        and equilibrium_derivative_norm <= config.equilibrium_derivative_norm_limit
    )

    def derivative_with_feedback(
        candidate_state: Mapping[str, float],
        feedback_wrench: Mapping[str, float],
        *,
        dt_s: float,
    ) -> tuple[dict[str, float], dict[str, object]]:
        feedback = _named_values(feedback_wrench, DIRECT_WRENCH_NAMES, "feedback wrench")
        request = {name: bias[name] + feedback[name] for name in DIRECT_WRENCH_NAMES}
        projection = config.limits.project(request, zero_wrench, dt_s)
        derivative = _named_values(
            config.source_derivative(candidate_state, projection.achieved),
            config.state_names,
            "source derivative",
        )
        return derivative, projection.as_dict()
        ####

    a_matrix = np.zeros((len(config.state_names), len(config.state_names)))
    for column, name in enumerate(config.state_names):
        plus = dict(reference)
        minus = dict(reference)
        plus[name] += config.state_derivative_step
        minus[name] -= config.state_derivative_step
        plus_value, _ = derivative_with_feedback(plus, zero_wrench, dt_s=1.0)
        minus_value, _ = derivative_with_feedback(minus, zero_wrench, dt_s=1.0)
        a_matrix[:, column] = (
            _state_array(plus_value, config.state_names) - _state_array(minus_value, config.state_names)
        ) / (2.0 * config.state_derivative_step)
    b_matrix = np.zeros((len(config.state_names), len(DIRECT_WRENCH_NAMES)))
    for column, name in enumerate(DIRECT_WRENCH_NAMES):
        plus = dict(zero_wrench)
        minus = dict(zero_wrench)
        plus[name] += config.control_derivative_step
        minus[name] -= config.control_derivative_step
        plus_value, _ = derivative_with_feedback(reference, plus, dt_s=1.0)
        minus_value, _ = derivative_with_feedback(reference, minus, dt_s=1.0)
        b_matrix[:, column] = (
            _state_array(plus_value, config.state_names) - _state_array(minus_value, config.state_names)
        ) / (2.0 * config.control_derivative_step)
    lqr = solve_scaled_continuous_lqr(
        a_matrix.tolist(),
        b_matrix.tolist(),
        np.diag(config.state_cost_weights).tolist(),
        np.diag(config.control_cost_weights).tolist(),
        state_scales=config.state_scales,
        control_scales=config.control_scales,
        state_names=config.state_names,
        control_names=DIRECT_WRENCH_NAMES,
    )

    initial_error = _error_norm(state, reference, config.state_names, config.state_scales)
    previous = dict(zero_wrench)
    rows: list[dict[str, object]] = []
    steps = int(round(config.duration_s / config.dt_s))
    for index in range(steps + 1):
        time_s = index * config.dt_s
        error = _state_array(state, config.state_names) - _state_array(reference, config.state_names)
        feedback = {
            name: float(value)
            for name, value in zip(DIRECT_WRENCH_NAMES, -np.asarray(lqr.gain) @ error, strict=True)
        }
        requested = {name: bias[name] + feedback[name] for name in DIRECT_WRENCH_NAMES}
        projection = config.limits.project(requested, previous, config.dt_s)
        derivative = _named_values(
            config.source_derivative(state, projection.achieved),
            config.state_names,
            "source derivative",
        )
        row: dict[str, object] = {
            "time_s": time_s,
            "state": dict(state),
            "reference_state": dict(reference),
            "feedback_error": {name: state[name] - reference[name] for name in config.state_names},
            "feedback_norm": _error_norm(state, reference, config.state_names, config.state_scales),
            "bias_wrench": dict(bias),
            "feedback_wrench": feedback,
            "wrench": projection.as_dict(),
        }
        rows.append(row)
        if index == steps:
            break
        state = {name: state[name] + config.dt_s * derivative[name] for name in config.state_names}
        if any(not math.isfinite(value) for value in state.values()):
            raise RuntimeError(f"local direct-wrench screen {config.id!r} produced a nonfinite state")
        previous = dict(projection.achieved)
    final_error = _error_norm(state, reference, config.state_names, config.state_scales)
    statuses = {str(_mapping(row["wrench"], "wrench")["status"]) for row in rows}
    mission_pass = (
        math.isfinite(final_error)
        and equilibrium_pass
        and lqr.hurwitz
        and final_error < initial_error * config.final_error_fraction_limit
        and statuses == {"feasible"}
    )
    return LocalDirectWrenchScreenExecution(
        config=config,
        bias_wrench=bias,
        a_matrix=a_matrix,
        b_matrix=b_matrix,
        lqr=lqr,
        rows=tuple(rows),
        initial_error_norm=initial_error,
        final_error_norm=final_error,
        equilibrium_projection=equilibrium_projection,
        equilibrium_derivative=equilibrium_derivative,
        equilibrium_derivative_norm=equilibrium_derivative_norm,
        equilibrium_pass=equilibrium_pass,
        mission_pass=mission_pass,
    )
    ####


def _state_array(values: Mapping[str, float], names: tuple[str, ...]) -> np.ndarray:
    """Return one ordered local state vector."""

    return np.asarray([float(values[name]) for name in names], dtype=float)
    ####


def _error_norm(
    state: Mapping[str, float],
    reference: Mapping[str, float],
    names: tuple[str, ...],
    scales: tuple[float, ...],
) -> float:
    """Return the declared dimensionless local state error norm."""

    return float(
        math.sqrt(
            sum(((float(state[name]) - float(reference[name])) / scale) ** 2 for name, scale in zip(names, scales, strict=True))
        )
    )
    ####


def _mapping(value: object, label: str) -> Mapping[str, object]:
    """Check a nested telemetry value before reading it."""

    if not isinstance(value, Mapping):
        raise TypeError(f"{label} must be a mapping")
    return value
    ####


__all__ = [
    "LocalDirectWrenchScreenConfig",
    "LocalDirectWrenchScreenExecution",
    "run_local_direct_wrench_screen",
]
