"""Generic plant-informed LQR profile synthesis and scoring.

This module provides the common controller-design loop for registered vehicles:
resolve the vehicle scaling contract, synthesize gentle/standard/aggressive
continuous-time LQR candidates, and apply optional maneuver metrics.  The
default linearization is deliberately only an attitude/moment bridge.  It is
useful for checking conditioning and profile ordering, but it is not source
validation.  A vehicle-specific source-trim Jacobian and maneuver evaluator
must be supplied before a candidate can support a flight claim.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

from .runtime.lqr import LqrResult, LqrRobustnessReport, LqrUncertaintySpec, assess_lqr_robustness, solve_scaled_continuous_lqr
from .trim import DynamicsLinearization
from .vehicle_registry import derive_lqr_scale_contract, lqr_profile_attributes, vehicle_definition


@dataclass(frozen=True, slots=True)
class AutoTuneLimits:
    """Optional gates used when ranking an automatically synthesized profile."""

    maximum_real_pole: float = -1.0e-8
    minimum_table_margin: float = 0.0
    maximum_saturation_fraction: float = 0.0
    maximum_control_rate: float | None = None
    maximum_tracking_error: float | None = None
    derivative_uncertainty: LqrUncertaintySpec | None = None
    require_uncertainty_stability: bool = True
    ####


@dataclass(frozen=True, slots=True)
class AutoTuneCandidate:
    """One profile candidate and its machine-readable diagnostics."""

    vehicle_id: str
    profile_id: str
    scales: Mapping[str, float]
    scale_provenance: Mapping[str, str]
    table_axis_limits: Mapping[str, tuple[float, float]]
    weights: Mapping[str, float]
    lqr: LqrResult | None
    robustness: LqrRobustnessReport | None
    metrics: Mapping[str, float]
    violations: tuple[str, ...]
    score: float
    status: str

    @property
    def safe(self) -> bool:
        """Whether this candidate passed every gate that was supplied."""

        return self.status == "safe"
    ####

    def as_dict(self) -> dict[str, Any]:
        """Serialize the candidate without depending on NumPy JSON support."""

        poles: list[float | list[float]] = []
        if self.lqr is not None:
            for value in self.lqr.closed_loop_eigenvalues:
                real = float(value.real)
                imaginary = float(value.imag)
                poles.append(real if abs(imaginary) <= 1.0e-14 else [real, imaginary])
        return {
            "vehicle_id": self.vehicle_id,
            "profile_id": self.profile_id,
            "scales": dict(self.scales),
            "scale_provenance": dict(self.scale_provenance),
            "table_axis_limits": {name: list(values) for name, values in self.table_axis_limits.items()},
            "weights": dict(self.weights),
            "metrics": dict(self.metrics),
            "violations": list(self.violations),
            "score": self.score if math.isfinite(self.score) else None,
            "status": self.status,
            "closed_loop_eigenvalues": poles,
            "condition_number": self.lqr.condition_number if self.lqr is not None else None,
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
class AutoTuneReport:
    """The complete profile sweep for one registered vehicle."""

    vehicle_id: str
    design_source: str
    candidates: tuple[AutoTuneCandidate, ...]

    @property
    def best(self) -> AutoTuneCandidate | None:
        """Return the lowest-scoring safe candidate, if one exists."""

        safe = tuple(candidate for candidate in self.candidates if candidate.safe)
        return min(safe, key=lambda candidate: candidate.score, default=None)
    ####

    def as_dict(self) -> dict[str, Any]:
        """Serialize the report for evidence packets and CLI output."""

        best = self.best
        return {
            "vehicle_id": self.vehicle_id,
            "design_source": self.design_source,
            "status": "safe" if best is not None else "no-safe-candidate",
            "best_profile_id": best.profile_id if best is not None else None,
            "candidates": [candidate.as_dict() for candidate in self.candidates],
        }
    ####


ManeuverEvaluator = Callable[[LqrResult], Mapping[str, float | None]]
Matrix = tuple[tuple[float, ...], ...]


def default_attitude_linearization(vehicle_id: str) -> tuple[Matrix, Matrix]:
    """Return the generic attitude-error/moment bridge for a registry vehicle.

    The bridge has attitude-error rates as its first three states and body
    rates as its last three states.  Moment commands affect angular
    acceleration through the registered diagonal inertia.  It is a numerical
    conditioning baseline, not a substitute for a source-trim Jacobian.
    """

    import numpy as np

    inertia = vehicle_definition(vehicle_id)["inertia_kg_m2"]
    a_matrix = np.zeros((6, 6), dtype=float)
    a_matrix[:3, 3:] = np.eye(3)
    b_matrix = np.zeros((6, 3), dtype=float)
    b_matrix[3:, :] = np.diag(
        [
            1.0 / float(inertia["x"]),
            1.0 / float(inertia["y"]),
            1.0 / float(inertia["z"]),
        ]
    )
    return tuple(tuple(float(value) for value in row) for row in a_matrix), tuple(
        tuple(float(value) for value in row) for row in b_matrix
    )
    ####


def _profile_weights(profile_id: str) -> dict[str, float]:
    """Read normalized Q/R weights through the canonical profile resolver."""

    attributes = lqr_profile_attributes(profile_id)
    return {
        "q_angle": float(attributes["q-angle"]),
        "q_rate": float(attributes["q-rate"]),
        "r_moment": float(attributes["r-moment"]),
    }
    ####


def _candidate(
    vehicle_id: str,
    profile_id: str,
    a_matrix: Sequence[Sequence[float]],
    b_matrix: Sequence[Sequence[float]],
    *,
    state_names: Sequence[str],
    control_names: Sequence[str],
    limits: AutoTuneLimits,
    evaluator: ManeuverEvaluator | None,
) -> AutoTuneCandidate:
    """Build and gate one LQR candidate, failing closed on design errors."""

    import numpy as np

    scales_contract = derive_lqr_scale_contract(vehicle_id)
    weights = _profile_weights(profile_id)
    state_scales = (
        float(scales_contract["state_angle_scale_rad"]),
        float(scales_contract["state_angle_scale_rad"]),
        float(scales_contract["state_angle_scale_rad"]),
        float(scales_contract["state_rate_scale_rad_s"]),
        float(scales_contract["state_rate_scale_rad_s"]),
        float(scales_contract["state_rate_scale_rad_s"]),
    )
    control_scales = (
        float(scales_contract["control_moment_scale_nm"]),
        float(scales_contract["control_moment_scale_nm"]),
        float(scales_contract["control_moment_scale_nm"]),
    )
    q_diagonal = (weights["q_angle"],) * 3 + (weights["q_rate"],) * 3
    q_matrix: Matrix = tuple(
        tuple(float(q_diagonal[row]) if row == column else 0.0 for column in range(6))
        for row in range(6)
    )
    r_diagonal = (weights["r_moment"],) * 3
    r_matrix: Matrix = tuple(
        tuple(float(r_diagonal[row]) if row == column else 0.0 for column in range(3))
        for row in range(3)
    )
    scales = {
        "state_angle_scale_rad": state_scales[0],
        "state_rate_scale_rad_s": state_scales[3],
        "control_moment_scale_nm": control_scales[0],
        "mass_scale_kg": float(scales_contract["mass_scale_kg"]),
        "inertia_scale_kg_m2": float(scales_contract["inertia_scale_kg_m2"]),
        "force_scale_n": float(scales_contract["force_scale_n"]),
        "weight_moment_scale_nm": float(scales_contract["weight_moment_scale_nm"]),
        "inertia_moment_scale_nm": float(scales_contract["inertia_moment_scale_nm"]),
    }
    metrics: dict[str, float] = {}
    violations: list[str] = []
    robustness: LqrRobustnessReport | None = None
    try:
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
            evaluated = evaluator(result)
            for name, value in evaluated.items():
                if value is None:
                    continue
                numeric = float(value)
                if not np.isfinite(numeric):
                    violations.append(f"nonfinite-{name}")
                    continue
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
        robustness = None
        metrics["design_error"] = 1.0
        violations.append(f"design-error:{type(error).__name__}")
        score = float("inf")
        status = "failed"
    table_axis_limits: dict[str, tuple[float, float]] = {}
    for name, values in scales_contract["table_axis_limits"].items():
        if len(values) != 2:
            raise ValueError(f"vehicle table axis {name!r} must have lower and upper limits")
        table_axis_limits[str(name)] = (float(values[0]), float(values[1]))
    return AutoTuneCandidate(
        vehicle_id=vehicle_id,
        profile_id=profile_id,
        scales=scales,
        scale_provenance={str(name): str(value) for name, value in scales_contract["provenance"].items()},
        table_axis_limits=table_axis_limits,
        weights=weights,
        lqr=result,
        robustness=robustness,
        metrics=metrics,
        violations=tuple(violations),
        score=score,
        status=status,
    )
    ####


def auto_tune_lqr_profiles(
    vehicle_id: str,
    *,
    a_matrix: Sequence[Sequence[float]] | None = None,
    b_matrix: Sequence[Sequence[float]] | None = None,
    linearization: DynamicsLinearization | None = None,
    state_names: Sequence[str] = ("attitude-error-x", "attitude-error-y", "attitude-error-z", "p", "q", "r"),
    control_names: Sequence[str] = ("moment-x", "moment-y", "moment-z"),
    profile_ids: Sequence[str] | None = None,
    limits: AutoTuneLimits | None = None,
    evaluator: ManeuverEvaluator | None = None,
) -> AutoTuneReport:
    """Sweep registered LQR styles for one vehicle.

    If matrices are omitted, the report is explicitly tagged as a
    ``runtime-inertia-attitude-bridge`` synthesis.  Passing one without the
    other is rejected.  A source-trim Jacobian and evaluator should be passed
    by vehicle-family adapters before controller evidence is promoted.
    """

    vehicle_definition(vehicle_id)
    resolved_a: Sequence[Sequence[float]]
    resolved_b: Sequence[Sequence[float]]
    if linearization is not None and (a_matrix is not None or b_matrix is not None):
        raise ValueError("provide either linearization or a_matrix/b_matrix, not both")
    if linearization is not None:
        resolved_a = tuple(tuple(float(value) for value in row) for row in linearization.a_matrix)
        resolved_b = tuple(tuple(float(value) for value in row) for row in linearization.b_matrix)
        state_names = linearization.state_names
        control_names = linearization.control_names
        design_source = "source-trim-dynamics-linearization"
    elif a_matrix is None:
        if b_matrix is not None:
            raise ValueError("a_matrix and b_matrix must be supplied together")
        resolved_a, resolved_b = default_attitude_linearization(vehicle_id)
        design_source = "runtime-inertia-attitude-bridge"
    else:
        if b_matrix is None:
            raise ValueError("a_matrix and b_matrix must be supplied together")
        resolved_a, resolved_b = a_matrix, b_matrix
        design_source = "supplied-source-trim-linearization"
    selected = tuple(profile_ids or (
        f"{vehicle_id.replace('_', '-')}-gentle",
        f"{vehicle_id.replace('_', '-')}-standard",
        f"{vehicle_id.replace('_', '-')}-aggressive",
    ))
    resolved_limits = limits or AutoTuneLimits()
    candidates = tuple(
        _candidate(
            vehicle_id,
            profile_id,
            resolved_a,
            resolved_b,
            state_names=state_names,
            control_names=control_names,
            limits=resolved_limits,
            evaluator=evaluator,
        )
        for profile_id in selected
    )
    return AutoTuneReport(vehicle_id, design_source, candidates)
    ####
