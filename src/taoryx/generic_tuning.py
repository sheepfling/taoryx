"""Vehicle-independent trim, linearization, and LQR tuning utilities.

Family adapters own the plant equations, frames, units, and actuator
semantics.  This module owns the repeatable tuning workflow around those
adapters: solve a bounded trim, finite-difference the declared state
derivatives, synthesize named LQR profiles, and return auditable diagnostics.
It deliberately does not infer vehicle physics or silently convert residuals
into state derivatives.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from .controller_autotune import AutoTuneLimits, ManeuverEvaluator
from .runtime.lqr import LqrResult, LqrRobustnessReport, assess_lqr_robustness, solve_scaled_continuous_lqr
from .trim import DynamicsEvaluator, DynamicsLinearization, TrimEvaluator, TrimResult, TrimSpec, finite_difference_dynamics_linearization, solve_trim

Matrix = tuple[tuple[float, ...], ...]


@dataclass(frozen=True, slots=True)
class GenericLqrProfile:
    """One dimension-matched Q/R profile for any named plant."""

    id: str
    q_diagonal: tuple[float, ...]
    r_diagonal: tuple[float, ...]

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.q_diagonal or not self.r_diagonal:
            raise ValueError("generic LQR profiles require an id and non-empty weights")
        if any(not math.isfinite(value) or value <= 0.0 for value in (*self.q_diagonal, *self.r_diagonal)):
            raise ValueError("generic LQR profile weights must be finite and positive")
        ####
    ####


@dataclass(frozen=True, slots=True)
class GenericLqrCandidate:
    """One generic LQR candidate with stability and operational diagnostics."""

    vehicle_id: str
    profile_id: str
    state_names: tuple[str, ...]
    control_names: tuple[str, ...]
    state_scales: tuple[float, ...]
    control_scales: tuple[float, ...]
    weights: GenericLqrProfile
    lqr: LqrResult | None
    robustness: LqrRobustnessReport | None
    metrics: Mapping[str, float]
    violations: tuple[str, ...]
    score: float
    status: str

    @property
    def safe(self) -> bool:
        """Return whether every supplied gate passed."""

        return self.status == "safe"
        ####

    def as_dict(self) -> dict[str, Any]:
        """Return JSON-compatible candidate diagnostics."""

        poles: list[float | list[float]] = []
        if self.lqr is not None:
            for pole in self.lqr.closed_loop_eigenvalues:
                real = float(pole.real)
                imaginary = float(pole.imag)
                poles.append(real if abs(imaginary) <= 1.0e-14 else [real, imaginary])
        return {
            "vehicle_id": self.vehicle_id,
            "profile_id": self.profile_id,
            "state_names": list(self.state_names),
            "control_names": list(self.control_names),
            "state_scales": list(self.state_scales),
            "control_scales": list(self.control_scales),
            "weights": {
                "q_diagonal": list(self.weights.q_diagonal),
                "r_diagonal": list(self.weights.r_diagonal),
            },
            "metrics": dict(self.metrics),
            "violations": list(self.violations),
            "score": self.score if math.isfinite(self.score) else None,
            "status": self.status,
            "closed_loop_eigenvalues": poles,
            "condition_number": self.lqr.condition_number if self.lqr is not None else None,
            "matrix_sha256": {
                "a": self.lqr.a_sha256 if self.lqr is not None else None,
                "b": self.lqr.b_sha256 if self.lqr is not None else None,
                "q": self.lqr.q_sha256 if self.lqr is not None else None,
                "r": self.lqr.r_sha256 if self.lqr is not None else None,
                "k": self.lqr.k_sha256 if self.lqr is not None else None,
            },
            "robustness": {
                "nominal_max_real_pole": self.robustness.nominal_max_real_pole,
                "worst_max_real_pole": self.robustness.worst_max_real_pole,
                "margin_to_instability": self.robustness.margin_to_instability,
                "samples": self.robustness.samples,
                "stable": self.robustness.stable,
            } if self.robustness is not None else None,
        }
        ####


@dataclass(frozen=True, slots=True)
class GenericLqrReport:
    """Profile sweep result independent of a vehicle registry."""

    vehicle_id: str
    design_source: str
    candidates: tuple[GenericLqrCandidate, ...]

    @property
    def best(self) -> GenericLqrCandidate | None:
        """Return the lowest-scoring safe candidate, if one exists."""

        safe = tuple(candidate for candidate in self.candidates if candidate.safe)
        return min(safe, key=lambda candidate: candidate.score) if safe else None
        ####

    def as_dict(self) -> dict[str, Any]:
        """Return a reproducible profile report."""

        best = self.best
        return {
            "vehicle_id": self.vehicle_id,
            "design_source": self.design_source,
            "status": "safe" if best is not None else "no-safe-candidate",
            "best_profile_id": best.profile_id if best is not None else None,
            "candidates": [candidate.as_dict() for candidate in self.candidates],
        }
        ####


def _diagonal_matrix(values: Sequence[float], size: int, label: str) -> Matrix:
    """Validate and materialize a positive diagonal matrix."""

    if len(values) != size:
        raise ValueError(f"{label} length must be {size}, got {len(values)}")
    if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in values):
        raise ValueError(f"{label} values must be finite and positive")
    return tuple(tuple(float(values[row]) if row == column else 0.0 for column in range(size)) for row in range(size))
    ####


def _generic_candidate(
    vehicle_id: str,
    profile: GenericLqrProfile,
    a_matrix: Sequence[Sequence[float]],
    b_matrix: Sequence[Sequence[float]],
    state_names: tuple[str, ...],
    control_names: tuple[str, ...],
    state_scales: tuple[float, ...],
    control_scales: tuple[float, ...],
    limits: AutoTuneLimits,
    evaluator: ManeuverEvaluator | None,
) -> GenericLqrCandidate:
    """Synthesize and gate one dimension-matched generic candidate."""

    metrics: dict[str, float] = {}
    violations: list[str] = []
    robustness: LqrRobustnessReport | None = None
    try:
        q_matrix = _diagonal_matrix(profile.q_diagonal, len(state_names), "Q diagonal")
        r_matrix = _diagonal_matrix(profile.r_diagonal, len(control_names), "R diagonal")
        result = solve_scaled_continuous_lqr(
            a_matrix,
            b_matrix,
            q_matrix,
            r_matrix,
            state_scales=state_scales,
            control_scales=control_scales,
            state_names=state_names,
            control_names=control_names,
        )
        metrics["maximum_real_pole"] = result.maximum_real_pole
        metrics["condition_number"] = result.condition_number
        if result.maximum_real_pole > limits.maximum_real_pole:
            violations.append("closed-loop-pole-limit")
        if limits.derivative_uncertainty is not None:
            robustness = assess_lqr_robustness(a_matrix, b_matrix, result, limits.derivative_uncertainty)
            metrics["worst_uncertain_real_pole"] = robustness.worst_max_real_pole
            if limits.require_uncertainty_stability and not robustness.stable:
                violations.append("uncertainty-pole-limit")
        if evaluator is not None:
            for name, value in evaluator(result).items():
                if value is None:
                    continue
                numeric = float(value)
                if not np.isfinite(numeric):
                    violations.append(f"nonfinite-{name}")
                else:
                    metrics[str(name)] = numeric
            margin = metrics.get("table_margin_min_normalized")
            if margin is not None and margin < limits.minimum_table_margin:
                violations.append("table-margin-limit")
            saturation = metrics.get("control_saturation_fraction")
            if saturation is not None and saturation > limits.maximum_saturation_fraction:
                violations.append("control-saturation-limit")
            control_rate = metrics.get("control_rate_abs_max")
            if limits.maximum_control_rate is not None and control_rate is not None and control_rate > limits.maximum_control_rate:
                violations.append("control-rate-limit")
            tracking_error = metrics.get("tracking_error")
            if limits.maximum_tracking_error is not None and tracking_error is not None and tracking_error > limits.maximum_tracking_error:
                violations.append("tracking-error-limit")
        score = (
            max(0.0, metrics["maximum_real_pole"] - limits.maximum_real_pole) * 1.0e6
            + metrics["condition_number"] * 1.0e-6
            + max(0.0, limits.minimum_table_margin - metrics.get("table_margin_min_normalized", limits.minimum_table_margin)) * 1.0e3
            + max(0.0, metrics.get("control_saturation_fraction", 0.0) - limits.maximum_saturation_fraction) * 1.0e3
            + max(0.0, metrics.get("tracking_error", 0.0) - (limits.maximum_tracking_error or 0.0))
        )
        status = "safe" if not violations else "unsafe"
    except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
        result = None
        metrics["design_error"] = 1.0
        violations.append(f"design-error:{type(error).__name__}")
        score = float("inf")
        status = "failed"
    return GenericLqrCandidate(
        vehicle_id,
        profile.id,
        state_names,
        control_names,
        state_scales,
        control_scales,
        profile,
        result,
        robustness,
        metrics,
        tuple(violations),
        score,
        status,
    )
    ####


def tune_lqr_profiles(
    vehicle_id: str,
    a_matrix: Sequence[Sequence[float]],
    b_matrix: Sequence[Sequence[float]],
    *,
    state_names: Sequence[str],
    control_names: Sequence[str],
    state_scales: Sequence[float],
    control_scales: Sequence[float],
    profiles: Sequence[GenericLqrProfile],
    limits: AutoTuneLimits | None = None,
    evaluator: ManeuverEvaluator | None = None,
    design_source: str = "supplied-plant-linearization",
) -> GenericLqrReport:
    """Tune arbitrary named state/control dimensions from a supplied plant.

    The matrices must be true state-derivative Jacobians.  Family adapters
    remain responsible for producing them from source equations or a declared
    plant evaluator; this function never treats force or moment residuals as
    state derivatives.
    """

    states = tuple(state_names)
    controls = tuple(control_names)
    if not states or len(set(states)) != len(states):
        raise ValueError("generic LQR state names must be non-empty and unique")
    if not controls or len(set(controls)) != len(controls):
        raise ValueError("generic LQR control names must be non-empty and unique")
    scales_x = tuple(float(value) for value in state_scales)
    scales_u = tuple(float(value) for value in control_scales)
    if len(scales_x) != len(states) or any(not math.isfinite(value) or value <= 0.0 for value in scales_x):
        raise ValueError("state scales must match state names and be finite and positive")
    if len(scales_u) != len(controls) or any(not math.isfinite(value) or value <= 0.0 for value in scales_u):
        raise ValueError("control scales must match control names and be finite and positive")
    if not profiles:
        raise ValueError("generic LQR tuning requires at least one profile")
    resolved_limits = limits or AutoTuneLimits()
    candidates = tuple(
        _generic_candidate(vehicle_id, profile, a_matrix, b_matrix, states, controls, scales_x, scales_u, resolved_limits, evaluator)
        for profile in profiles
    )
    return GenericLqrReport(vehicle_id, design_source, candidates)
    ####


@dataclass(frozen=True, slots=True)
class TrimToTuneResult:
    """Result of the generic trim-to-linearization-to-LQR pipeline."""

    vehicle_id: str
    trim: TrimResult | None
    linearization: DynamicsLinearization | None
    lqr: GenericLqrReport | None
    status: str
    failure_reason: str | None = None

    def as_dict(self) -> dict[str, Any]:
        """Return stage results without hiding a failed prerequisite."""

        return {
            "vehicle_id": self.vehicle_id,
            "status": self.status,
            "failure_reason": self.failure_reason,
            "trim": self.trim.as_dict() if self.trim is not None else None,
            "linearization": {
                "state_names": list(self.linearization.state_names),
                "control_names": list(self.linearization.control_names),
                "a_matrix": self.linearization.a_matrix.tolist(),
                "b_matrix": self.linearization.b_matrix.tolist(),
                "trim_state": dict(self.linearization.trim_state),
                "trim_controls": dict(self.linearization.trim_controls),
                "metadata": dict(self.linearization.metadata),
            } if self.linearization is not None else None,
            "lqr": self.lqr.as_dict() if self.lqr is not None else None,
        }
        ####


def trim_linearize_and_tune(
    vehicle_id: str,
    trim_spec: TrimSpec,
    trim_evaluator: TrimEvaluator,
    dynamics_evaluator: DynamicsEvaluator,
    *,
    state_scales: Sequence[float],
    control_scales: Sequence[float],
    profiles: Sequence[GenericLqrProfile],
    limits: AutoTuneLimits | None = None,
    maneuver_evaluator: ManeuverEvaluator | None = None,
    state_step: float = 1.0e-6,
    control_step: float = 1.0e-6,
    metadata: Mapping[str, str | float] | None = None,
) -> TrimToTuneResult:
    """Run bounded trim, true derivative linearization, and generic LQR.

    This is the reusable adapter boundary for B747, X8, helicopters,
    spacecraft, and other families.  A failed trim stops the pipeline rather
    than allowing an LQR design around an arbitrary initial condition.
    """

    try:
        trim = solve_trim(trim_spec, trim_evaluator)
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        return TrimToTuneResult(vehicle_id, None, None, None, "trim_adapter_invalid", str(error))
    if not trim.success:
        return TrimToTuneResult(vehicle_id, trim, None, None, "trim_failed", trim.message)
    try:
        linearization = finite_difference_dynamics_linearization(
            trim_spec,
            dynamics_evaluator,
            trim,
            state_step=state_step,
            control_step=control_step,
            metadata=metadata,
        )
        report = tune_lqr_profiles(
            vehicle_id,
            tuple(tuple(float(value) for value in row) for row in linearization.a_matrix),
            tuple(tuple(float(value) for value in row) for row in linearization.b_matrix),
            state_names=linearization.state_names,
            control_names=linearization.control_names,
            state_scales=state_scales,
            control_scales=control_scales,
            profiles=profiles,
            limits=limits,
            evaluator=maneuver_evaluator,
            design_source="trimmed-plant-finite-difference-linearization",
        )
    except (KeyError, TypeError, ValueError, OverflowError, RuntimeError, np.linalg.LinAlgError) as error:
        return TrimToTuneResult(vehicle_id, trim, None, None, "linearization_or_tuning_failed", str(error))
    return TrimToTuneResult(vehicle_id, trim, linearization, report, "tuned")
    ####
