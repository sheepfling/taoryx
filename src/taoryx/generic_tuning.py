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
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Literal

import numpy as np

from .control_allocation import ControlPlantAdapter
from .controller_autotune import AutoTuneLimits, ManeuverEvaluator
from .runtime.lqr import (
    LqiController,
    LqiResult,
    LqrResult,
    LqrRobustnessReport,
    assess_lqr_robustness,
    solve_scaled_continuous_lqi,
    solve_scaled_continuous_lqr,
)
from .trim import DynamicsEvaluator, DynamicsLinearization, TrimEvaluator, TrimResult, TrimSpec, finite_difference_dynamics_linearization, solve_trim

Matrix = tuple[tuple[float, ...], ...]
AuthorityPreflightStatus = Literal["passed", "blocked"]


@dataclass(frozen=True, slots=True)
class AuthorityPreflightReport:
    """Family-provided authority gate placed before controller candidate search.

    The generic tuner cannot infer whether a two-elevon flying wing must
    control yaw, whether a rotor layout has independent yaw authority, or
    whether a gimbal is usable at the current flight phase.  The family
    adapter supplies that judgment here.  A blocked report prevents LQR
    synthesis, ensuring a structurally infeasible plant is not treated as a
    gain-tuning problem.
    """

    status: AuthorityPreflightStatus
    reason: str
    metrics: Mapping[str, float] = field(default_factory=dict)
    blockers: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not self.reason.strip():
            raise ValueError("authority preflight requires a non-empty reason")
        if any(not math.isfinite(float(value)) for value in self.metrics.values()):
            raise ValueError("authority preflight metrics must be finite")
        object.__setattr__(self, "metrics", dict(self.metrics))
        if self.status == "passed" and self.blockers:
            raise ValueError("a passed authority preflight cannot declare blockers")
        if self.status == "blocked" and not self.blockers:
            raise ValueError("a blocked authority preflight requires at least one blocker")
        ####

    def as_dict(self) -> dict[str, Any]:
        """Return the authority decision included in a tuning report."""

        return {
            "status": self.status,
            "reason": self.reason,
            "metrics": dict(self.metrics),
            "blockers": list(self.blockers),
        }
        ####
    ####


AuthorityPreflightEvaluator = Callable[[TrimResult, DynamicsLinearization], AuthorityPreflightReport]


@dataclass(frozen=True, slots=True)
class LinearAuthorityRequirement:
    """A declared pre-tuning authority requirement for one local plant.

    ``required_state_names`` are the state coordinates that the prospective
    controller must be able to influence through the *actual declared input
    space*.  They are intentionally supplied by the family strategy: a
    flying wing can require roll and pitch rate without pretending that two
    elevons supply independent yaw control, while a multirotor may require
    all three attitude-rate axes.

    The requirement does not manufacture an actuator model.  It evaluates the
    reachable subspace of the supplied local ``A``/``B`` pair and blocks a gain
    search when the requested objective is structurally unreachable.
    """

    id: str
    required_state_names: tuple[str, ...]
    minimum_controllability_rank: int | None = None
    maximum_uncontrolled_fraction: float = 1.0e-8
    maximum_controllability_condition: float | None = None

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("linear authority requirement requires an id")
        if not self.required_state_names or len(set(self.required_state_names)) != len(self.required_state_names):
            raise ValueError("linear authority requirement needs unique required state names")
        if self.minimum_controllability_rank is not None and self.minimum_controllability_rank <= 0:
            raise ValueError("minimum controllability rank must be positive when supplied")
        if not math.isfinite(self.maximum_uncontrolled_fraction) or not 0.0 <= self.maximum_uncontrolled_fraction < 1.0:
            raise ValueError("maximum uncontrolled fraction must be finite in [0, 1)")
        if self.maximum_controllability_condition is not None and (
            not math.isfinite(self.maximum_controllability_condition)
            or self.maximum_controllability_condition <= 1.0
        ):
            raise ValueError("maximum controllability condition must be finite and greater than one")
        ####
    ####


def linear_authority_preflight(
    requirement: LinearAuthorityRequirement,
    *,
    state_names: Sequence[str],
    a_matrix: Sequence[Sequence[float]],
    b_matrix: Sequence[Sequence[float]],
) -> AuthorityPreflightReport:
    """Evaluate controllability and named-state reachability before tuning.

    This is deliberately more specific than merely calling an LQR solver.  A
    failed Riccati solve tells a developer that something is wrong; this
    report identifies which declared mission/control state is unreachable and
    preserves the rank and conditioning evidence needed to choose between a
    new effector, a different topology, a schedule node, or a reduced mission.
    """

    names = tuple(state_names)
    if not names or len(set(names)) != len(names):
        raise ValueError("linear authority preflight requires unique state names")
    missing = tuple(name for name in requirement.required_state_names if name not in names)
    if missing:
        raise ValueError(f"authority requirement names unknown states: {', '.join(missing)}")

    a: Any = np.asarray(a_matrix, dtype=float)
    b: Any = np.asarray(b_matrix, dtype=float)
    state_count = len(names)
    if a.shape != (state_count, state_count):
        raise ValueError("authority preflight A matrix must be square in the declared state order")
    if b.ndim != 2 or b.shape[0] != state_count or b.shape[1] == 0:
        raise ValueError("authority preflight B matrix must have one or more declared controls")
    if not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError("authority preflight matrices must be finite")

    controllability = np.hstack(tuple(np.linalg.matrix_power(a, power) @ b for power in range(state_count)))
    singular_values = np.linalg.svd(controllability, compute_uv=False)
    largest = float(singular_values[0]) if len(singular_values) else 0.0
    rank_tolerance = max(largest * max(controllability.shape) * np.finfo(float).eps, 1.0e-14)
    rank = int(np.count_nonzero(singular_values > rank_tolerance))
    positive = singular_values[singular_values > rank_tolerance]
    condition = float(positive[0] / positive[-1]) if len(positive) else 0.0
    # A nested controller need not control every state in the parent plant.
    # For example, an inner multirotor attitude loop deliberately leaves
    # position and resource states to outer loops.  The caller still proves
    # that every *declared required* state is reachable below; the default rank
    # gate therefore reflects the requested local subproblem rather than
    # accidentally requiring a full-state controller.
    required_rank = requirement.minimum_controllability_rank or len(requirement.required_state_names)
    if required_rank > state_count:
        raise ValueError("authority minimum controllability rank exceeds the declared state dimension")

    if rank:
        left_vectors = np.linalg.svd(controllability, full_matrices=False)[0][:, :rank]
        reachable_projector = left_vectors @ left_vectors.T
    else:
        reachable_projector = np.zeros((state_count, state_count))

    metrics: dict[str, float] = {
        "state_dimension": float(state_count),
        "control_dimension": float(b.shape[1]),
        "controllability_rank": float(rank),
        "required_controllability_rank": float(required_rank),
        "controllability_condition_number": condition,
        "required_state_count": float(len(requirement.required_state_names)),
    }
    blockers: list[str] = []
    if rank < required_rank:
        blockers.append("controllability_rank_below_requirement")
    for state_name in requirement.required_state_names:
        index = names.index(state_name)
        basis = np.zeros(state_count)
        basis[index] = 1.0
        uncontrolled_fraction = float(np.linalg.norm(basis - reachable_projector @ basis))
        metrics[f"uncontrolled_fraction.{state_name}"] = uncontrolled_fraction
        if uncontrolled_fraction > requirement.maximum_uncontrolled_fraction:
            blockers.append(f"uncontrolled_required_state:{state_name}")
    if (
        requirement.maximum_controllability_condition is not None
        and (rank == 0 or condition > requirement.maximum_controllability_condition)
    ):
        blockers.append("controllability_ill_conditioned")

    if blockers:
        return AuthorityPreflightReport(
            "blocked",
            f"{requirement.id} found a structural authority limitation before gain synthesis",
            metrics,
            tuple(blockers),
        )
    return AuthorityPreflightReport(
        "passed",
        f"{requirement.id} found every declared required state controllable before gain synthesis",
        metrics,
    )
    ####


def linear_authority_preflight_evaluator(requirement: LinearAuthorityRequirement) -> AuthorityPreflightEvaluator:
    """Bind a reusable linear-authority requirement to trim/tuning workflow."""

    def evaluate(_: TrimResult, linearization: DynamicsLinearization) -> AuthorityPreflightReport:
        return linear_authority_preflight(
            requirement,
            state_names=linearization.state_names,
            a_matrix=tuple(tuple(float(value) for value in row) for row in linearization.a_matrix),
            b_matrix=tuple(tuple(float(value) for value in row) for row in linearization.b_matrix),
        )
        ####

    return evaluate
    ####


@dataclass(frozen=True, slots=True)
class GenericLqrProfile:
    """One dimension-matched Q/R profile for any named plant."""

    id: str
    q_diagonal: tuple[float, ...]
    r_diagonal: tuple[float, ...]
    integral_weight_multiplier: float = 1.0

    def __post_init__(self) -> None:
        if not self.id.strip() or not self.q_diagonal or not self.r_diagonal:
            raise ValueError("generic LQR profiles require an id and non-empty weights")
        if any(not math.isfinite(value) or value <= 0.0 for value in (*self.q_diagonal, *self.r_diagonal)):
            raise ValueError("generic LQR profile weights must be finite and positive")
        if not math.isfinite(self.integral_weight_multiplier) or self.integral_weight_multiplier <= 0.0:
            raise ValueError("generic LQR integral-weight multipliers must be finite and positive")
        ####
    ####


@dataclass(frozen=True, slots=True)
class NormalizedLqrProfileGrid:
    """Generate a bounded, scaled LQR profile lattice without hand tuning.

    State and control coordinates have already been normalized by the
    campaign's declared scales. The grid therefore searches only a small,
    interpretable set of relative tracking and effort priorities instead of
    asking every family to invent absolute Q/R magnitudes. It is a candidate
    generator, not a substitute for the family-specific nonlinear-response
    and actuator gates that determine promotion.
    """

    id_prefix: str
    state_weight_multipliers: tuple[float, ...] = (0.25, 1.0, 4.0)
    control_effort_multipliers: tuple[float, ...] = (2.0, 1.0, 0.25)
    integral_weight_multipliers: tuple[float, ...] = (1.0,)
    state_base_weights: tuple[float, ...] = ()
    control_base_weights: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        if not self.id_prefix.strip():
            raise ValueError("normalized LQR profile grids require an ID prefix")
        for label, values in (
            ("state weight multipliers", self.state_weight_multipliers),
            ("control effort multipliers", self.control_effort_multipliers),
            ("integral weight multipliers", self.integral_weight_multipliers),
        ):
            if not values or any(not math.isfinite(value) or value <= 0.0 for value in values):
                raise ValueError(f"normalized LQR {label} must be finite and positive")
            if len(values) != len(set(values)):
                raise ValueError(f"normalized LQR {label} must not contain duplicates")
        for label, values in (
            ("state base weights", self.state_base_weights),
            ("control base weights", self.control_base_weights),
        ):
            if values and any(not math.isfinite(value) or value <= 0.0 for value in values):
                raise ValueError(f"normalized LQR {label} must be finite and positive")
        ####

    def profiles(self, state_count: int, control_count: int) -> tuple[GenericLqrProfile, ...]:
        """Return deterministic candidates for one scaled local plant."""

        if state_count <= 0 or control_count <= 0:
            raise ValueError("normalized LQR profile grids require positive state/control dimensions")
        state_base = self.state_base_weights or (1.0,) * state_count
        control_base = self.control_base_weights or (1.0,) * control_count
        if len(state_base) != state_count:
            raise ValueError("normalized LQR state base weights must match the selected state dimension")
        if len(control_base) != control_count:
            raise ValueError("normalized LQR control base weights must match the selected control dimension")
        profiles: list[GenericLqrProfile] = []
        for tracking in self.state_weight_multipliers:
            for effort in self.control_effort_multipliers:
                for integral in self.integral_weight_multipliers:
                    identifier = f"{self.id_prefix}.tracking-{tracking:g}.effort-{effort:g}"
                    if self.integral_weight_multipliers != (1.0,):
                        identifier = f"{identifier}.integral-{integral:g}"
                    profiles.append(
                        GenericLqrProfile(
                            identifier,
                            tuple(value * tracking for value in state_base),
                            tuple(value * effort for value in control_base),
                            integral,
                        )
                    )
        return tuple(profiles)
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the portable candidate-generation policy."""

        return {
            "id_prefix": self.id_prefix,
            "state_weight_multipliers": list(self.state_weight_multipliers),
            "control_effort_multipliers": list(self.control_effort_multipliers),
            "integral_weight_multipliers": list(self.integral_weight_multipliers),
            "state_base_weights": list(self.state_base_weights),
            "control_base_weights": list(self.control_base_weights),
        }
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
    method: Literal["lqr", "lqi"] = "lqr"
    integral_output_names: tuple[str, ...] = ()
    integral_q_diagonal: tuple[float, ...] = ()
    lqi: LqiResult | None = None
    integral_weight_multiplier: float = 1.0

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
            "method": self.method,
            "integral_weight_multiplier": self.integral_weight_multiplier,
            "state_names": list(self.state_names),
            "control_names": list(self.control_names),
            "state_scales": list(self.state_scales),
            "control_scales": list(self.control_scales),
            "weights": {
                "q_diagonal": list(self.weights.q_diagonal),
                "r_diagonal": list(self.weights.r_diagonal),
                "integral_q_diagonal": list(self.integral_q_diagonal),
            },
            "integral_output_names": list(self.integral_output_names),
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
            "lqr_controller": {
                "state_gain": np.asarray(self.lqr.gain, dtype=float).tolist(),
            } if self.method == "lqr" and self.lqr is not None else None,
            "lqi_controller": {
                "output_matrix": np.asarray(self.lqi.output_matrix, dtype=float).tolist(),
                "state_gain": np.asarray(self.lqi.state_gain, dtype=float).tolist(),
                "integral_gain": np.asarray(self.lqi.integral_gain, dtype=float).tolist(),
            } if self.lqi is not None else None,
        }
        ####


@dataclass(frozen=True, slots=True)
class GenericLqrReport:
    """Profile sweep result independent of a vehicle registry."""

    vehicle_id: str
    design_source: str
    candidates: tuple[GenericLqrCandidate, ...]
    method: Literal["lqr", "lqi"] = "lqr"
    integral_output_names: tuple[str, ...] = ()

    @property
    def best(self) -> GenericLqrCandidate | None:
        """Return the best safe candidate with a deterministic nominal tie break.

        Linear score ties are common when a profile lattice changes only LQI
        integral priority: every candidate can meet the same pole and
        conditioning gates while their nonlinear offset behavior differs.
        Preserve the score as the primary decision, but when it is tied,
        prefer the nominal integral multiplier (``1.0``) before using the
        stable profile ID.  This prevents profile-list order from silently
        selecting a deliberately weak or aggressive integrator.
        """

        safe = tuple(candidate for candidate in self.candidates if candidate.safe)
        return (
            min(
                safe,
                key=lambda candidate: (
                    candidate.score,
                    abs(math.log(candidate.integral_weight_multiplier)) if candidate.method == "lqi" else 0.0,
                    candidate.profile_id,
                ),
            )
            if safe
            else None
        )
        ####

    def as_dict(self) -> dict[str, Any]:
        """Return a reproducible profile report."""

        best = self.best
        return {
            "vehicle_id": self.vehicle_id,
            "design_source": self.design_source,
            "method": self.method,
            "integral_output_names": list(self.integral_output_names),
            "status": "safe" if best is not None else "no-safe-candidate",
            "best_profile_id": best.profile_id if best is not None else None,
            "candidates": [candidate.as_dict() for candidate in self.candidates],
        }
    ####


@dataclass(frozen=True, slots=True)
class NativeCoordinateLqiSample:
    """One accepted native-control nonlinear LQI interval.

    ``applied_controls`` are the named coordinates that the family adapter
    itself accepts at its derivative boundary.  They are deliberately not
    called effectors: a response-law, guidance, or reduced-order plant may
    expose controls without claiming a physical allocator.
    """

    time_s: float
    state: Mapping[str, float]
    state_error: Mapping[str, float]
    lqi_integral_error: Mapping[str, float]
    requested_controls: Mapping[str, float]
    applied_controls: Mapping[str, float]
    control_saturated: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        """Return one JSON-safe committed nonlinear control sample."""

        return {
            "time_s": self.time_s,
            "state": dict(self.state),
            "state_error": dict(self.state_error),
            "lqi_integral_error": dict(self.lqi_integral_error),
            "requested_controls": dict(self.requested_controls),
            "applied_controls": dict(self.applied_controls),
            "control_saturated": list(self.control_saturated),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class NativeCoordinateLqiValidation:
    """Exact nonlinear evidence for LQI through native model controls.

    The record is appropriate when a ``ControlPlantAdapter`` owns an
    executable derivative and named controls but intentionally does not own a
    physical wrench allocator.  It preserves the distinction from
    ``PhysicalWrenchLqiValidation`` rather than fabricating actuator or wrench
    telemetry.
    """

    candidate: GenericLqrCandidate
    duration_s: float
    dt_s: float
    initial_state: Mapping[str, float]
    final_state: Mapping[str, float]
    state_reference: Mapping[str, float]
    environment: Mapping[str, float | str]
    assessment_state_names: tuple[str, ...]
    samples: tuple[NativeCoordinateLqiSample, ...]

    def __post_init__(self) -> None:
        if self.candidate.method != "lqi" or self.candidate.lqi is None:
            raise ValueError("native-coordinate LQI validation requires a retained LQI candidate")
        if not math.isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("native-coordinate LQI validation duration must be finite and positive")
        if not math.isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise ValueError("native-coordinate LQI validation time step must be finite and positive")
        if not self.assessment_state_names or set(self.assessment_state_names) - set(self.candidate.state_names):
            raise ValueError("native-coordinate LQI assessment states must be declared candidate states")
        if set(self.candidate.state_names) - set(self.initial_state) or set(self.candidate.state_names) - set(self.final_state):
            raise ValueError("native-coordinate LQI validation state is missing candidate coordinates")
        if set(self.candidate.state_names) - set(self.state_reference):
            raise ValueError("native-coordinate LQI reference is missing candidate coordinates")
        if any(
            not math.isfinite(float(value))
            for mapping in (self.initial_state, self.final_state, self.state_reference)
            for value in mapping.values()
        ):
            raise ValueError("native-coordinate LQI validation state and reference values must be finite")
        if any(
            isinstance(value, bool) or not isinstance(value, int | float | str)
            or (isinstance(value, int | float) and not math.isfinite(float(value)))
            for value in self.environment.values()
        ):
            raise ValueError("native-coordinate LQI environment values must be finite numeric or text")
        ####

    @property
    def initial_normalized_feedback_error_norm(self) -> float:
        """Return the declared assessment-state error at the initial sample."""

        return self._normalized_error_norm(self.initial_state)
        ####

    @property
    def final_normalized_feedback_error_norm(self) -> float:
        """Return the declared assessment-state error after the last interval."""

        return self._normalized_error_norm(self.final_state)
        ####

    @property
    def control_saturation_fraction(self) -> float:
        """Return the fraction of intervals constrained at the native boundary."""

        return sum(bool(sample.control_saturated) for sample in self.samples) / max(len(self.samples), 1)
        ####

    @property
    def maximum_continuous_control_saturation_duration_s(self) -> float:
        """Return the longest contiguous native-control bound interval."""

        longest = 0.0
        current = 0.0
        previous_time_s = 0.0
        for sample in self.samples:
            interval_s = sample.time_s - previous_time_s
            previous_time_s = sample.time_s
            if sample.control_saturated:
                current += interval_s
                longest = max(longest, current)
            else:
                current = 0.0
        return longest
        ####

    @property
    def saturated_controls(self) -> tuple[str, ...]:
        """Return every commanded native coordinate that reached its bound."""

        return tuple(sorted({name for sample in self.samples for name in sample.control_saturated}))
        ####

    @property
    def integrators_exercised(self) -> bool:
        """Return whether any retained LQI state became nonzero."""

        return any(
            abs(float(value)) > 1.0e-12
            for sample in self.samples
            for value in sample.lqi_integral_error.values()
        )
        ####

    def as_dict(self) -> dict[str, object]:
        """Serialize native nonlinear evidence without a physical-effector claim."""

        return {
            "schema": "taoryx.native-coordinate-lqi-validation/v1alpha1",
            "control_realization": "native_named_coordinates",
            "candidate": self.candidate.as_dict(),
            "duration_s": self.duration_s,
            "dt_s": self.dt_s,
            "initial_state": dict(self.initial_state),
            "final_state": dict(self.final_state),
            "state_reference": dict(self.state_reference),
            "environment": dict(self.environment),
            "assessment_state_names": list(self.assessment_state_names),
            "initial_normalized_feedback_error": self.initial_normalized_feedback_error_norm,
            "final_normalized_feedback_error": self.final_normalized_feedback_error_norm,
            "control_saturation_fraction": self.control_saturation_fraction,
            "maximum_continuous_control_saturation_duration_s": self.maximum_continuous_control_saturation_duration_s,
            "saturated_controls": list(self.saturated_controls),
            "integrators_exercised": self.integrators_exercised,
            "samples": [sample.as_dict() for sample in self.samples],
            "claim_boundary": (
                "Commands were applied through the adapter's declared native control coordinates. "
                "This does not establish physical-effector allocation, actuator dynamics, or hardware authority."
            ),
        }
        ####

    def _normalized_error_norm(self, state: Mapping[str, float]) -> float:
        scales = dict(zip(self.candidate.state_names, self.candidate.state_scales, strict=True))
        return math.sqrt(
            sum(
                ((float(state[name]) - float(self.state_reference[name])) / float(scales[name])) ** 2
                for name in self.assessment_state_names
            )
        )
        ####
    ####


def validate_nonlinear_native_coordinate_lqi(
    plant: ControlPlantAdapter,
    trim: TrimResult,
    candidate: GenericLqrCandidate,
    *,
    initial_state: Mapping[str, float],
    duration_s: float,
    dt_s: float,
    assessment_state_names: Sequence[str] | None = None,
    reference: Mapping[str, float] | None = None,
    control_lower: Mapping[str, float] | None = None,
    control_upper: Mapping[str, float] | None = None,
    integral_lower: Mapping[str, float] | None = None,
    integral_upper: Mapping[str, float] | None = None,
    environment: Mapping[str, float | str] | None = None,
) -> NativeCoordinateLqiValidation:
    """Run a retained generic LQI candidate through native plant controls.

    This is the non-allocator counterpart to physical wrench validation.  The
    function never converts a named model control into a wrench or actuator;
    it applies only the exact controls accepted by ``plant.state_derivative``.
    """

    if candidate.method != "lqi" or candidate.lqi is None:
        raise ValueError("native-coordinate nonlinear validation requires a retained LQI candidate")
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("native-coordinate LQI duration must be finite and positive")
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("native-coordinate LQI time step must be finite and positive")
    required_state_names = tuple(plant.state_names)
    required_control_names = tuple(plant.control_names)
    if tuple(trim.spec.state_names) != required_state_names or tuple(trim.spec.control_names) != required_control_names:
        raise ValueError("native-coordinate LQI trim does not match the adapter contract")
    if set(candidate.state_names) - set(required_state_names):
        raise ValueError("native-coordinate LQI candidate refers to states absent from the adapter")
    if set(candidate.control_names) - set(required_control_names):
        raise ValueError("native-coordinate LQI candidate refers to controls absent from the adapter")
    missing_state = set(required_state_names) - set(initial_state)
    if missing_state:
        raise KeyError(f"native-coordinate LQI initial state is missing: {', '.join(sorted(missing_state))}")
    state = {name: float(initial_state[name]) for name in required_state_names}
    if any(not math.isfinite(value) for value in state.values()):
        raise ValueError("native-coordinate LQI initial state must be finite")
    native_controls = {name: float(trim.controls[name]) for name in required_control_names}
    state_reference = {name: float(trim.state[name]) for name in candidate.state_names}
    tracked_reference = {name: float(trim.state[name]) for name in candidate.lqi.output_names}
    if reference is not None:
        missing_reference = set(candidate.lqi.output_names) - set(reference)
        unknown_reference = set(reference) - set(candidate.lqi.output_names)
        if missing_reference or unknown_reference:
            raise KeyError("native-coordinate LQI reference must identify exactly the tracked outputs")
        tracked_reference = {name: float(reference[name]) for name in candidate.lqi.output_names}
        state_reference.update(tracked_reference)
    if any(not math.isfinite(value) for value in state_reference.values()):
        raise ValueError("native-coordinate LQI reference must be finite")
    derivative_environment = dict(environment or {})
    if any(
        isinstance(value, bool) or not isinstance(value, int | float | str)
        or (isinstance(value, int | float) and not math.isfinite(float(value)))
        for value in derivative_environment.values()
    ):
        raise ValueError("native-coordinate LQI environment values must be finite numeric or text")
    controller = LqiController(
        candidate.lqi,
        state_trim={name: float(trim.state[name]) for name in candidate.state_names},
        control_trim={name: float(trim.controls[name]) for name in candidate.control_names},
        output_trim={name: float(trim.state[name]) for name in candidate.lqi.output_names},
        lower=control_lower or {},
        upper=control_upper or {},
        integral_lower=integral_lower or {},
        integral_upper=integral_upper or {},
    )
    assessment = tuple(assessment_state_names or candidate.lqi.output_names)
    if not assessment or set(assessment) - set(candidate.state_names):
        raise ValueError("native-coordinate LQI assessment states must be declared candidate states")
    samples: list[NativeCoordinateLqiSample] = []
    time_s = 0.0
    steps = int(math.ceil(duration_s / dt_s))
    for _ in range(steps):
        step_s = min(dt_s, duration_s - time_s)
        if step_s <= 0.0:
            break
        command = controller.command(state, tracked_reference, dt=step_s)
        native_controls.update({name: float(value) for name, value in command.controls.items()})
        state = _native_coordinate_rk4_state_step(plant, state, native_controls, step_s, derivative_environment)
        time_s += step_s
        state_error = {name: state[name] - state_reference[name] for name in candidate.state_names}
        samples.append(
            NativeCoordinateLqiSample(
                time_s,
                dict(state),
                state_error,
                dict(controller.integral_error),
                {name: float(value) for name, value in command.unsaturated.items()},
                {name: float(value) for name, value in command.controls.items()},
                tuple(command.saturated),
            )
        )
    if any(not math.isfinite(value) for value in state.values()):
        raise ValueError("native-coordinate LQI nonlinear integration produced a non-finite state")
    return NativeCoordinateLqiValidation(
        candidate,
        duration_s,
        dt_s,
        dict(initial_state),
        state,
        state_reference,
        derivative_environment,
        assessment,
        tuple(samples),
    )
    ####


def _native_coordinate_rk4_state_step(
    plant: ControlPlantAdapter,
    state: Mapping[str, float],
    controls: Mapping[str, float],
    dt_s: float,
    environment: Mapping[str, float | str],
) -> dict[str, float]:
    """Integrate one accepted native-control interval with RK4."""

    names = tuple(plant.state_names)

    def derivative(values: Mapping[str, float]) -> dict[str, float]:
        result = plant.state_derivative(values, controls, environment)
        return {name: float(result[name]) for name in names}

    k1 = derivative(state)
    k2 = derivative({name: float(state[name]) + 0.5 * dt_s * k1[name] for name in names})
    k3 = derivative({name: float(state[name]) + 0.5 * dt_s * k2[name] for name in names})
    k4 = derivative({name: float(state[name]) + dt_s * k3[name] for name in names})
    return {
        name: float(state[name]) + dt_s * (k1[name] + 2.0 * k2[name] + 2.0 * k3[name] + k4[name]) / 6.0
        for name in names
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


def tune_lqi_profiles(
    vehicle_id: str,
    a_matrix: Sequence[Sequence[float]],
    b_matrix: Sequence[Sequence[float]],
    *,
    state_names: Sequence[str],
    control_names: Sequence[str],
    state_scales: Sequence[float],
    control_scales: Sequence[float],
    profiles: Sequence[GenericLqrProfile],
    output_names: Sequence[str],
    integral_q_diagonal: Sequence[float],
    limits: AutoTuneLimits | None = None,
    design_source: str = "supplied-plant-linearization",
) -> GenericLqrReport:
    """Tune scaled LQI candidates with declared tracked outputs and weights.

    ``integral_q_diagonal`` is intentionally required input, not an automatic
    multiplier.  A plug-in must make the persistent-error priority explicit
    for each selected output; the host only runs the bounded candidate grid.
    """

    states = tuple(state_names)
    controls = tuple(control_names)
    outputs = tuple(output_names)
    if not states or len(set(states)) != len(states):
        raise ValueError("generic LQI state names must be non-empty and unique")
    if not controls or len(set(controls)) != len(controls):
        raise ValueError("generic LQI control names must be non-empty and unique")
    if not outputs or len(set(outputs)) != len(outputs) or set(outputs) - set(states):
        raise ValueError("generic LQI outputs must be unique declared states")
    integral_weights = tuple(float(value) for value in integral_q_diagonal)
    if len(integral_weights) != len(outputs) or any(not math.isfinite(value) or value <= 0.0 for value in integral_weights):
        raise ValueError("generic LQI integral weights must match outputs and be finite and positive")
    scales_x = tuple(float(value) for value in state_scales)
    scales_u = tuple(float(value) for value in control_scales)
    if len(scales_x) != len(states) or any(not math.isfinite(value) or value <= 0.0 for value in scales_x):
        raise ValueError("state scales must match LQI state names and be finite and positive")
    if len(scales_u) != len(controls) or any(not math.isfinite(value) or value <= 0.0 for value in scales_u):
        raise ValueError("control scales must match LQI control names and be finite and positive")
    if not profiles:
        raise ValueError("generic LQI tuning requires at least one profile")

    output = tuple(
        tuple(1.0 if state == output_name else 0.0 for state in states)
        for output_name in outputs
    )
    resolved_limits = limits or AutoTuneLimits()
    candidates = tuple(
        _generic_lqi_candidate(
            vehicle_id,
            profile,
            a_matrix,
            b_matrix,
            states,
            controls,
            scales_x,
            scales_u,
            output,
            outputs,
            tuple(value * profile.integral_weight_multiplier for value in integral_weights),
            resolved_limits,
        )
        for profile in profiles
    )
    return GenericLqrReport(
        vehicle_id,
        design_source,
        candidates,
        method="lqi",
        integral_output_names=outputs,
    )
    ####


def _generic_lqi_candidate(
    vehicle_id: str,
    profile: GenericLqrProfile,
    a_matrix: Sequence[Sequence[float]],
    b_matrix: Sequence[Sequence[float]],
    state_names: tuple[str, ...],
    control_names: tuple[str, ...],
    state_scales: tuple[float, ...],
    control_scales: tuple[float, ...],
    output_matrix: Sequence[Sequence[float]],
    output_names: tuple[str, ...],
    integral_q_diagonal: tuple[float, ...],
    limits: AutoTuneLimits,
) -> GenericLqrCandidate:
    """Synthesize one named LQI candidate through the common safety gates."""

    metrics: dict[str, float] = {}
    violations: list[str] = []
    lqi: LqiResult | None = None
    try:
        q_matrix = _diagonal_matrix(
            (*profile.q_diagonal, *integral_q_diagonal),
            len(state_names) + len(output_names),
            "augmented LQI Q diagonal",
        )
        r_matrix = _diagonal_matrix(profile.r_diagonal, len(control_names), "R diagonal")
        lqi = solve_scaled_continuous_lqi(
            a_matrix,
            b_matrix,
            q_matrix,
            r_matrix,
            output_matrix=output_matrix,
            output_names=output_names,
            state_scales=state_scales,
            control_scales=control_scales,
            state_names=state_names,
            control_names=control_names,
        )
        design = lqi.design
        metrics["maximum_real_pole"] = design.maximum_real_pole
        metrics["condition_number"] = design.condition_number
        if design.maximum_real_pole > limits.maximum_real_pole:
            violations.append("closed-loop-pole-limit")
        score = max(0.0, metrics["maximum_real_pole"] - limits.maximum_real_pole) * 1.0e6 + metrics["condition_number"] * 1.0e-6
        status = "safe" if not violations else "unsafe"
    except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
        design = None
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
        design,
        None,
        metrics,
        tuple(violations),
        score,
        status,
        method="lqi",
        integral_output_names=output_names,
        integral_q_diagonal=integral_q_diagonal,
        lqi=lqi,
        integral_weight_multiplier=profile.integral_weight_multiplier,
    )
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
    authority_preflight: AuthorityPreflightReport | None = None

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
            "authority_preflight": (
                self.authority_preflight.as_dict()
                if self.authority_preflight is not None
                else None
            ),
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
    authority_preflight: AuthorityPreflightEvaluator | None = None,
    maneuver_evaluator: ManeuverEvaluator | None = None,
    state_step: float = 1.0e-6,
    control_step: float = 1.0e-6,
    metadata: Mapping[str, str | float] | None = None,
) -> TrimToTuneResult:
    """Run bounded trim, true derivative linearization, and generic LQR.

    This is the reusable adapter boundary for B747, X8, helicopters,
    spacecraft, and other families.  A failed trim stops the pipeline rather
    than allowing an LQR design around an arbitrary initial condition.  When
    supplied, ``authority_preflight`` runs after derivative construction and
    before LQR synthesis, so a topology or effectivity blocker cannot be
    obscured by a gain sweep.
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
    except (KeyError, TypeError, ValueError, OverflowError, RuntimeError, np.linalg.LinAlgError) as error:
        return TrimToTuneResult(vehicle_id, trim, None, None, "linearization_or_tuning_failed", str(error))

    authority: AuthorityPreflightReport | None = None
    if authority_preflight is not None:
        try:
            authority = authority_preflight(trim, linearization)
        except (KeyError, TypeError, ValueError, OverflowError, RuntimeError, np.linalg.LinAlgError) as error:
            return TrimToTuneResult(vehicle_id, trim, linearization, None, "authority_preflight_invalid", str(error))
        if authority.status != "passed":
            return TrimToTuneResult(
                vehicle_id,
                trim,
                linearization,
                None,
                "authority_preflight_blocked",
                authority.reason,
                authority,
            )

    try:
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
        return TrimToTuneResult(vehicle_id, trim, linearization, None, "linearization_or_tuning_failed", str(error), authority)
    return TrimToTuneResult(vehicle_id, trim, linearization, report, "tuned", authority_preflight=authority)
    ####
