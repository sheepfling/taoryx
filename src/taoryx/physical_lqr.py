"""Plant-derived LQR designs with physically realized wrench commands.

This module is the narrow bridge between a local nonlinear plant derivative
and a nonlinear actuator-realization test.  It deliberately keeps three
spaces distinct:

``state error -> desired wrench increment -> bounded physical effectors``.

The first use is a local X8 table-coordinate proof.  The contracts are
generic so a conventional surface aircraft, multirotor, spacecraft, or
tiltrotor adapter can use the same validation path once it exposes a
``ControlPlantAdapter``.  It does not turn a source table coordinate into a
hardware claim: effector identity and sign provenance remain the adapter's
responsibility.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from .control_allocation import ControlPlantAdapter, EffectorEffectiveness, PhysicalAllocationStep, ProvenancedLinearization
from .runtime.lqr import LqrResult, solve_scaled_continuous_lqr
from .trim import TrimResult


@dataclass(frozen=True, slots=True)
class WrenchLinearizationProjection:
    """A plant-derived local mapping from desired wrench increments to state rates.

    ``effector_increment_per_wrench`` is the local, minimum-norm inverse of
    the selected actual-effector effectiveness matrix.  The nonlinear
    simulator does *not* use that inverse to apply a force or moment.  It is
    used only to derive the local LQR input matrix; every run still calls the
    bounded allocator and then the nonlinear plant.
    """

    state_names: tuple[str, ...]
    wrench_names: tuple[str, ...]
    effector_names: tuple[str, ...]
    a_matrix: tuple[tuple[float, ...], ...]
    b_matrix: tuple[tuple[float, ...], ...]
    effectiveness_matrix: tuple[tuple[float, ...], ...]
    effector_increment_per_wrench: tuple[tuple[float, ...], ...]
    nominal_wrench: Mapping[str, float]
    trim_state: Mapping[str, float]
    trim_effectors: Mapping[str, float]
    source_linearization: ProvenancedLinearization
    inverse_residual_norm: float

    @property
    def state_dimension(self) -> int:
        """Return the number of feedback state coordinates."""

        return len(self.state_names)
        ####
    ####

    @property
    def wrench_dimension(self) -> int:
        """Return the number of independently requested wrench axes."""

        return len(self.wrench_names)
        ####
    ####

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-safe account of the local input transformation."""

        return {
            "state_names": list(self.state_names),
            "wrench_names": list(self.wrench_names),
            "effector_names": list(self.effector_names),
            "a_matrix": [list(row) for row in self.a_matrix],
            "b_matrix": [list(row) for row in self.b_matrix],
            "effectiveness_matrix": [list(row) for row in self.effectiveness_matrix],
            "effector_increment_per_wrench": [list(row) for row in self.effector_increment_per_wrench],
            "nominal_wrench": dict(self.nominal_wrench),
            "trim_state": dict(self.trim_state),
            "trim_effectors": dict(self.trim_effectors),
            "inverse_residual_norm": self.inverse_residual_norm,
            "source_derivative_provenance": {
                "nonlinear_plant_id": self.source_linearization.provenance.nonlinear_plant_id,
                "nonlinear_plant_revision": self.source_linearization.provenance.nonlinear_plant_revision,
                "method": self.source_linearization.provenance.method,
                "derivative_consistent": self.source_linearization.provenance.derivative_consistent,
                "maximum_relative_difference": self.source_linearization.provenance.maximum_relative_difference,
                "maximum_absolute_difference": self.source_linearization.provenance.maximum_absolute_difference,
                "comparison_absolute_floor": self.source_linearization.provenance.comparison_absolute_floor,
            },
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class PhysicalWrenchLqrDesign:
    """One LQR gain whose inputs are declared local wrench increments."""

    id: str
    projection: WrenchLinearizationProjection
    result: LqrResult
    q_diagonal: tuple[float, ...]
    r_diagonal: tuple[float, ...]
    state_scales: tuple[float, ...]
    wrench_scales: tuple[float, ...]

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("physical LQR design requires a stable id")
        if len(self.q_diagonal) != self.projection.state_dimension:
            raise ValueError("physical LQR Q dimension does not match projected state")
        if len(self.r_diagonal) != self.projection.wrench_dimension:
            raise ValueError("physical LQR R dimension does not match projected wrench")
        if len(self.state_scales) != self.projection.state_dimension:
            raise ValueError("physical LQR state-scale dimension does not match projected state")
        if len(self.wrench_scales) != self.projection.wrench_dimension:
            raise ValueError("physical LQR wrench-scale dimension does not match projected wrench")
        if any(not math.isfinite(value) or value <= 0.0 for value in (*self.q_diagonal, *self.r_diagonal)):
            raise ValueError("physical LQR weights must be finite and positive")
        if any(not math.isfinite(value) or value <= 0.0 for value in (*self.state_scales, *self.wrench_scales)):
            raise ValueError("physical LQR scales must be finite and positive")
        if tuple(self.result.state_names) != self.projection.state_names:
            raise ValueError("physical LQR state names do not match projected plant")
        if tuple(self.result.control_names) != self.projection.wrench_names:
            raise ValueError("physical LQR wrench names do not match projected plant")
        ####
    ####

    def requested_wrench(self, state: Mapping[str, float]) -> tuple[dict[str, float], dict[str, float]]:
        """Return absolute requested wrench and local LQR increment.

        The returned wrench is an allocator request, not a force/moment
        directly applied to the plant.  Unselected axes retain their trim
        wrench so a rank-limited plant can make their status explicit.
        """

        missing = set(self.projection.state_names) - set(state)
        if missing:
            raise KeyError(f"physical LQR state is missing: {', '.join(sorted(missing))}")
        error = np.asarray(
            [float(state[name]) - float(self.projection.trim_state[name]) for name in self.projection.state_names],
            dtype=float,
        )
        increment = -np.asarray(self.result.gain, dtype=float) @ error
        requested = dict(self.projection.nominal_wrench)
        named_increment: dict[str, float] = {}
        for name, value in zip(self.projection.wrench_names, increment, strict=True):
            named_increment[name] = float(value)
            requested[name] = float(requested[name]) + float(value)
        return requested, named_increment
        ####
    ####

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-safe design artifact."""

        return {
            "id": self.id,
            "projection": self.projection.as_dict(),
            "q_diagonal": list(self.q_diagonal),
            "r_diagonal": list(self.r_diagonal),
            "state_scales": list(self.state_scales),
            "wrench_scales": list(self.wrench_scales),
            "gain": np.asarray(self.result.gain, dtype=float).tolist(),
            "closed_loop_poles": [
                {"real": float(value.real), "imaginary": float(value.imag)}
                for value in self.result.closed_loop_eigenvalues
            ],
            "maximum_real_pole": self.result.maximum_real_pole,
            "controllable": self.result.controllable,
            "condition_number": self.result.condition_number,
            "matrix_sha256": {
                "a": self.result.a_sha256,
                "b": self.result.b_sha256,
                "q": self.result.q_sha256,
                "r": self.result.r_sha256,
                "k": self.result.k_sha256,
            },
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class PhysicalWrenchLqrSample:
    """One committed nonlinear physical-controller sample."""

    time_s: float
    state: Mapping[str, float]
    state_error: Mapping[str, float]
    lqr_wrench_increment: Mapping[str, float]
    requested_wrench: Mapping[str, float]
    allocation: PhysicalAllocationStep

    def as_dict(self) -> dict[str, Any]:
        """Return telemetry with requested-versus-achieved physical evidence."""

        allocation = self.allocation
        return {
            "time_s": self.time_s,
            "state": dict(self.state),
            "state_error": dict(self.state_error),
            "lqr_wrench_increment": dict(self.lqr_wrench_increment),
            "requested_wrench": dict(self.requested_wrench),
            "predicted_wrench": dict(allocation.allocation.predicted_wrench),
            "achieved_wrench": dict(allocation.achieved_wrench),
            "allocation_residual": dict(allocation.allocation.residual_wrench),
            "achieved_residual": dict(allocation.achieved_residual_wrench),
            "allocation_status": allocation.allocation.status,
            "controlled_wrench_axes": list(allocation.allocation.controlled_wrench_axes),
            "uncontrolled_wrench_axes": list(allocation.allocation.uncontrolled_wrench_axes),
            "allocation_residual_norm": allocation.allocation.residual_norm,
            "allocation_controlled_residual_norm": allocation.allocation.controlled_residual_norm,
            "achieved_residual_norm": allocation.achieved_residual_norm,
            "achieved_controlled_residual_norm": allocation.achieved_controlled_residual_norm,
            "effectiveness_rank": allocation.allocation.effectiveness_rank,
            "effectiveness_matrix": [list(row) for row in allocation.allocation.effectiveness_matrix],
            "commanded_effectors": dict(allocation.actuator.commanded_positions),
            "actual_effectors": dict(allocation.actuator.actual_positions),
            "effector_rates": dict(allocation.actuator.rates_per_s),
            "position_saturated": list(allocation.actuator.position_saturated),
            "rate_limited": list(allocation.actuator.rate_limited),
            "lag_active": list(allocation.actuator.lag_active),
            "unavailable_effectors": list(allocation.actuator.unavailable_effectors),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class PhysicalWrenchLqrValidation:
    """Deterministic local nonlinear evidence for a plant-derived LQR."""

    design: PhysicalWrenchLqrDesign
    duration_s: float
    dt_s: float
    initial_state: Mapping[str, float]
    final_state: Mapping[str, float]
    samples: tuple[PhysicalWrenchLqrSample, ...]

    def __post_init__(self) -> None:
        if not math.isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("physical LQR validation duration must be finite and positive")
        if not math.isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise ValueError("physical LQR validation step must be finite and positive")
        if not self.samples:
            raise ValueError("physical LQR validation requires telemetry samples")
        ####
    ####

    @property
    def maximum_controlled_actual_residual(self) -> float:
        """Return the worst realized residual over actively controlled axes."""

        return max(sample.allocation.achieved_controlled_residual_norm for sample in self.samples)
        ####
    ####

    @property
    def saturation_fraction(self) -> float:
        """Return fraction of samples with an active physical limitation."""

        constrained = sum(_sample_is_constrained(sample) for sample in self.samples)
        return constrained / len(self.samples)
        ####
    ####

    @property
    def maximum_continuous_saturation_duration_s(self) -> float:
        """Return the longest committed run with an active physical limit."""

        longest = 0
        current = 0
        for sample in self.samples:
            if _sample_is_constrained(sample):
                current += 1
                longest = max(longest, current)
            else:
                current = 0
        return longest * self.dt_s
        ####
    ####

    @property
    def final_controlled_actual_residual(self) -> float:
        """Return the realized controlled-axis residual at the final sample."""

        return self.samples[-1].allocation.achieved_controlled_residual_norm
        ####
    ####

    @property
    def allocation_statuses(self) -> tuple[str, ...]:
        """Return all observed allocator statuses in deterministic order."""

        return tuple(sorted({sample.allocation.allocation.status for sample in self.samples}))
        ####
    ####

    @property
    def initial_feedback_error_norm(self) -> float:
        """Return the initial selected-state error norm."""

        return _feedback_error_norm(self.samples[0].state_error, self.design.projection.state_names)
        ####
    ####

    @property
    def final_feedback_error_norm(self) -> float:
        """Return the final selected-state error norm."""

        return _feedback_error_norm(self.samples[-1].state_error, self.design.projection.state_names)
        ####

    @property
    def initial_normalized_feedback_error_norm(self) -> float:
        """Return the initial error norm in declared LQR state units.

        The raw mixed-unit norms remain useful diagnostics, but a controller
        spanning radians, radians per second, and metres per second must be
        judged through the state scales used during synthesis.
        """

        return _normalized_feedback_error_norm(
            self.samples[0].state_error,
            self.design.projection.state_names,
            self.design.state_scales,
        )
        ####

    @property
    def final_normalized_feedback_error_norm(self) -> float:
        """Return the final error norm in declared LQR state units."""

        return _normalized_feedback_error_norm(
            self.samples[-1].state_error,
            self.design.projection.state_names,
            self.design.state_scales,
        )
        ####

    def as_dict(self) -> dict[str, Any]:
        """Return a self-contained local validation artifact."""

        return {
            "schema": "taoryx.physical-lqr-validation/v1alpha1",
            "design": self.design.as_dict(),
            "duration_s": self.duration_s,
            "dt_s": self.dt_s,
            "initial_state": dict(self.initial_state),
            "final_state": dict(self.final_state),
            "metrics": {
                "initial_feedback_error_norm": self.initial_feedback_error_norm,
                "final_feedback_error_norm": self.final_feedback_error_norm,
                "initial_normalized_feedback_error_norm": self.initial_normalized_feedback_error_norm,
                "final_normalized_feedback_error_norm": self.final_normalized_feedback_error_norm,
                "maximum_controlled_actual_residual": self.maximum_controlled_actual_residual,
                "final_controlled_actual_residual": self.final_controlled_actual_residual,
                "saturation_fraction": self.saturation_fraction,
                "maximum_continuous_saturation_duration_s": self.maximum_continuous_saturation_duration_s,
                "allocation_statuses": list(self.allocation_statuses),
            },
            "samples": [sample.as_dict() for sample in self.samples],
        }
        ####
    ####


def project_linearization_to_wrench(
    linearization: ProvenancedLinearization,
    effectiveness: EffectorEffectiveness,
    *,
    state_names: Sequence[str],
    wrench_names: Sequence[str],
    effector_names: Sequence[str],
    inverse_tolerance: float = 1.0e-8,
) -> WrenchLinearizationProjection:
    """Project actual-effector plant derivatives into locally allocatable wrench axes.

    This is a derivative transformation, not a force/moment injection.  The
    selected effectiveness matrix must have full row rank, otherwise the
    requested wrench axes are not independently controllable by the selected
    effectors at this operating point.
    """

    if not math.isfinite(inverse_tolerance) or inverse_tolerance <= 0.0:
        raise ValueError("wrench projection inverse tolerance must be finite and positive")
    state_names = tuple(state_names)
    wrench_names = tuple(wrench_names)
    effector_names = tuple(effector_names)
    if not state_names or not wrench_names or not effector_names:
        raise ValueError("wrench projection requires state, wrench, and effector channels")
    if len(set(state_names)) != len(state_names) or len(set(wrench_names)) != len(wrench_names) or len(set(effector_names)) != len(effector_names):
        raise ValueError("wrench projection channel names must be unique")
    source = linearization.primary
    missing_states = set(state_names) - set(source.state_names)
    missing_effectors = set(effector_names) - set(source.control_names)
    missing_wrenches = set(wrench_names) - set(effectiveness.wrench_names)
    missing_effectivity_effectors = set(effector_names) - set(effectiveness.effector_names)
    if missing_states:
        raise KeyError(f"linearization lacks selected states: {', '.join(sorted(missing_states))}")
    if missing_effectors:
        raise KeyError(f"linearization lacks selected effectors: {', '.join(sorted(missing_effectors))}")
    if missing_wrenches:
        raise KeyError(f"effectiveness lacks selected wrench axes: {', '.join(sorted(missing_wrenches))}")
    if missing_effectivity_effectors:
        raise KeyError(f"effectiveness lacks selected effectors: {', '.join(sorted(missing_effectivity_effectors))}")
    state_indices = tuple(source.state_names.index(name) for name in state_names)
    control_indices = tuple(source.control_names.index(name) for name in effector_names)
    wrench_indices = tuple(effectiveness.wrench_names.index(name) for name in wrench_names)
    effector_indices = tuple(effectiveness.effector_names.index(name) for name in effector_names)
    a_full = np.asarray(source.a_matrix, dtype=float)
    b_full = np.asarray(source.b_matrix, dtype=float)
    a = a_full[np.ix_(state_indices, state_indices)]
    b_effectors = b_full[np.ix_(state_indices, control_indices)]
    g = effectiveness.array[np.ix_(wrench_indices, effector_indices)]
    rank = int(np.linalg.matrix_rank(g))
    if rank < len(wrench_names):
        raise ValueError(
            "selected effector effectiveness cannot independently realize requested wrench axes: "
            f"rank {rank} for {len(wrench_names)} axes"
        )
    effector_increment_per_wrench = np.linalg.pinv(g)
    inverse_residual = float(np.linalg.norm(g @ effector_increment_per_wrench - np.eye(len(wrench_names))))
    if inverse_residual > inverse_tolerance:
        raise ValueError(
            "selected effector/wrench projection is not numerically consistent: "
            f"residual {inverse_residual:.6g} exceeds {inverse_tolerance:.6g}"
        )
    b_wrench = b_effectors @ effector_increment_per_wrench
    return WrenchLinearizationProjection(
        state_names=state_names,
        wrench_names=wrench_names,
        effector_names=effector_names,
        a_matrix=_matrix_tuple(a),
        b_matrix=_matrix_tuple(b_wrench),
        effectiveness_matrix=_matrix_tuple(g),
        effector_increment_per_wrench=_matrix_tuple(effector_increment_per_wrench),
        nominal_wrench={name: float(effectiveness.reference_wrench.get(name, 0.0)) for name in effectiveness.wrench_names},
        trim_state={name: float(source.trim_state[name]) for name in source.state_names},
        trim_effectors={name: float(source.trim_controls[name]) for name in source.control_names},
        source_linearization=linearization,
        inverse_residual_norm=inverse_residual,
    )
    ####


def design_physical_wrench_lqr(
    identifier: str,
    projection: WrenchLinearizationProjection,
    *,
    q_diagonal: Sequence[float],
    r_diagonal: Sequence[float],
    state_scales: Sequence[float],
    wrench_scales: Sequence[float],
) -> PhysicalWrenchLqrDesign:
    """Synthesize an LQR from actual nonlinear-plant derivatives.

    The returned control channels are *wrench increments*.  They are only
    realized later by a bounded physical-effector allocation through the same
    plant adapter.
    """

    q_diagonal = tuple(float(value) for value in q_diagonal)
    r_diagonal = tuple(float(value) for value in r_diagonal)
    state_scales = tuple(float(value) for value in state_scales)
    wrench_scales = tuple(float(value) for value in wrench_scales)
    if len(q_diagonal) != projection.state_dimension or len(state_scales) != projection.state_dimension:
        raise ValueError("physical wrench LQR state weights/scales must match projected state dimension")
    if len(r_diagonal) != projection.wrench_dimension or len(wrench_scales) != projection.wrench_dimension:
        raise ValueError("physical wrench LQR control weights/scales must match projected wrench dimension")
    q = tuple(
        tuple(value if row_index == column_index else 0.0 for column_index in range(projection.state_dimension))
        for row_index, value in enumerate(q_diagonal)
    )
    r = tuple(
        tuple(value if row_index == column_index else 0.0 for column_index in range(projection.wrench_dimension))
        for row_index, value in enumerate(r_diagonal)
    )
    result = solve_scaled_continuous_lqr(
        projection.a_matrix,
        projection.b_matrix,
        q,
        r,
        state_scales=state_scales,
        control_scales=wrench_scales,
        state_names=projection.state_names,
        control_names=projection.wrench_names,
    )
    if not result.hurwitz:
        raise ValueError(f"physical wrench LQR is not strictly stable: maximum real pole {result.maximum_real_pole:.6g}")
    return PhysicalWrenchLqrDesign(
        identifier,
        projection,
        result,
        q_diagonal,
        r_diagonal,
        state_scales,
        wrench_scales,
    )
    ####


def validate_nonlinear_wrench_lqr(
    plant: ControlPlantAdapter,
    trim: TrimResult,
    design: PhysicalWrenchLqrDesign,
    *,
    initial_state: Mapping[str, float],
    duration_s: float,
    dt_s: float,
) -> PhysicalWrenchLqrValidation:
    """Run deterministic local nonlinear recovery through real effectors.

    The controller is evaluated at every committed truth sample.  Actual
    effector positions are advanced first, then held over the accepted RK4
    interval.  This avoids fabricating a sensor/interpolated actuator state
    between accepted simulation samples and makes rate/lag consequences part
    of the nonlinear result.
    """

    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("nonlinear physical LQR duration must be finite and positive")
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("nonlinear physical LQR time step must be finite and positive")
    required_state_names = tuple(plant.state_names)
    missing_state = set(required_state_names) - set(initial_state)
    if missing_state:
        raise KeyError(f"initial nonlinear state is missing: {', '.join(sorted(missing_state))}")
    if tuple(trim.spec.state_names) != required_state_names:
        raise ValueError("nonlinear physical LQR trim does not match the adapter state contract")
    if tuple(trim.spec.control_names) != tuple(plant.control_names):
        raise ValueError("nonlinear physical LQR trim does not match the adapter effector contract")
    if set(design.projection.state_names) - set(required_state_names):
        raise ValueError("physical LQR design refers to states absent from adapter")
    state = {name: float(initial_state[name]) for name in required_state_names}
    if any(not math.isfinite(value) for value in state.values()):
        raise ValueError("nonlinear physical LQR initial state must be finite")
    actual_effectors = {name: float(trim.controls[name]) for name in plant.control_names}
    nominal_wrench = dict(design.projection.nominal_wrench)
    samples: list[PhysicalWrenchLqrSample] = []
    steps = int(math.ceil(duration_s / dt_s))
    time_s = 0.0
    for _ in range(steps):
        error = {name: state[name] - float(trim.state[name]) for name in required_state_names}
        requested, increment = design.requested_wrench(state)
        # The projection supplies the full nominal wrench, including declared
        # uncontrolled axes.  Guard against an adapter silently accepting an
        # incomplete wrench mapping.
        requested = {name: float(requested.get(name, nominal_wrench[name])) for name in nominal_wrench}
        allocation = plant.allocate(state, requested, actual_effectors, dt_s)
        if tuple(allocation.allocation.controlled_wrench_axes) != design.projection.wrench_names:
            raise ValueError(
                "plant allocation controlled axes do not match the physical LQR design: "
                f"{allocation.allocation.controlled_wrench_axes!r} != {design.projection.wrench_names!r}"
            )
        samples.append(
            PhysicalWrenchLqrSample(
                time_s,
                dict(state),
                error,
                increment,
                requested,
                allocation,
            )
        )
        actual_effectors = dict(allocation.actuator.actual_positions)
        step = min(dt_s, duration_s - time_s)
        if step <= 0.0:
            break
        state = _rk4_state_step(plant, state, actual_effectors, step)
        time_s += step
    final_state = dict(state)
    if any(not math.isfinite(value) for value in final_state.values()):
        raise ValueError("nonlinear physical LQR produced a non-finite final state")
    return PhysicalWrenchLqrValidation(design, duration_s, dt_s, dict(initial_state), final_state, tuple(samples))
    ####


def _rk4_state_step(
    plant: ControlPlantAdapter,
    state: Mapping[str, float],
    effectors: Mapping[str, float],
    dt_s: float,
) -> dict[str, float]:
    """Advance named local plant states with a held accepted actuator state."""

    names = tuple(plant.state_names)

    def derivative(values: Mapping[str, float]) -> np.ndarray:
        result = plant.state_derivative(values, effectors, {})
        missing = set(names) - set(result)
        if missing:
            raise KeyError(f"plant derivative is missing: {', '.join(sorted(missing))}")
        vector = np.asarray([float(result[name]) for name in names], dtype=float)
        if not np.all(np.isfinite(vector)):
            raise ValueError("physical LQR plant derivative contains a non-finite value")
        return vector
        ####

    base = np.asarray([float(state[name]) for name in names], dtype=float)
    k1 = derivative(state)
    k2 = derivative(_state_mapping(names, base + 0.5 * dt_s * k1))
    k3 = derivative(_state_mapping(names, base + 0.5 * dt_s * k2))
    k4 = derivative(_state_mapping(names, base + dt_s * k3))
    return _state_mapping(names, base + dt_s * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0)
    ####


def _matrix_tuple(values: np.ndarray) -> tuple[tuple[float, ...], ...]:
    """Return a stable immutable matrix representation."""

    return tuple(tuple(float(value) for value in row) for row in values)
    ####


def _state_mapping(names: Sequence[str], values: np.ndarray) -> dict[str, float]:
    """Pack a finite state vector under its canonical names."""

    if not np.all(np.isfinite(values)):
        raise ValueError("physical LQR state integration produced a non-finite value")
    return {name: float(value) for name, value in zip(names, values, strict=True)}
    ####


def _feedback_error_norm(errors: Mapping[str, float], state_names: Sequence[str]) -> float:
    """Return an unscaled norm for compact validation summaries."""

    return math.sqrt(sum(float(errors[name]) * float(errors[name]) for name in state_names))
    ####


def _normalized_feedback_error_norm(
    errors: Mapping[str, float],
    state_names: Sequence[str],
    state_scales: Sequence[float],
) -> float:
    """Return an error norm after applying declared LQR state scales."""

    if len(state_names) != len(state_scales):
        raise ValueError("normalized feedback norm requires one scale per selected state")
    return math.sqrt(
        sum(
            (float(errors[name]) / float(scale)) ** 2
            for name, scale in zip(state_names, state_scales, strict=True)
        )
    )
    ####


def _sample_is_constrained(sample: PhysicalWrenchLqrSample) -> bool:
    """Return whether an allocator or actuator limit affected one sample."""

    allocation = sample.allocation
    return bool(
        allocation.allocation.status != "feasible"
        or allocation.allocation.position_saturated
        or allocation.allocation.rate_limited
        or allocation.actuator.position_saturated
        or allocation.actuator.rate_limited
        or allocation.actuator.unavailable_effectors
    )
    ####
