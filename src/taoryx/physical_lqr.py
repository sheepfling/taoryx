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

import hashlib
import json
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
from typing import Any, Protocol

import numpy as np

from .control_allocation import ControlPlantAdapter, EffectorEffectiveness, PhysicalAllocationStep, ProvenancedLinearization
from .runtime.lqr import LqiController, LqiResult, LqrResult, solve_scaled_continuous_lqi, solve_scaled_continuous_lqr
from .trim import TrimResult
from .tuning_application import RuntimeTuningBindingReceipt, TuningApplicationContext


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

        return self.requested_wrench_for_reference(state, self.projection.trim_state)
        ####
    ####

    def requested_wrench_for_reference(
        self,
        state: Mapping[str, float],
        state_reference: Mapping[str, float],
    ) -> tuple[dict[str, float], dict[str, float]]:
        """Return a wrench request for an explicit local state reference.

        The reference must remain inside the declared linearization envelope.
        This method changes the feedback target only; it does not bypass the
        allocator or inject a wrench into the nonlinear plant.
        """

        missing = set(self.projection.state_names) - set(state)
        if missing:
            raise KeyError(f"physical LQR state is missing: {', '.join(sorted(missing))}")
        missing_reference = set(self.projection.state_names) - set(state_reference)
        if missing_reference:
            raise KeyError(f"physical LQR reference is missing: {', '.join(sorted(missing_reference))}")
        error = np.asarray(
            [float(state[name]) - float(state_reference[name]) for name in self.projection.state_names],
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
class PhysicalWrenchLqiDesign:
    """One offset-free LQI design whose demands remain allocator requests.

    The integrators accumulate only explicitly named local state outputs.  A
    caller still passes the returned wrench coordinates to the plant's bounded
    allocator; this class never converts a persistent error into a directly
    injected force or moment.
    """

    id: str
    projection: WrenchLinearizationProjection
    result: LqiResult
    q_diagonal: tuple[float, ...]
    r_diagonal: tuple[float, ...]
    integral_q_diagonal: tuple[float, ...]
    state_scales: tuple[float, ...]
    wrench_scales: tuple[float, ...]

    def __post_init__(self) -> None:
        if not self.id.strip():
            raise ValueError("physical LQI design requires a stable id")
        output_count = len(self.result.output_names)
        if not output_count or set(self.result.output_names) - set(self.projection.state_names):
            raise ValueError("physical LQI outputs must be unique projected state names")
        if len(self.q_diagonal) != self.projection.state_dimension:
            raise ValueError("physical LQI Q dimension does not match projected state")
        if len(self.r_diagonal) != self.projection.wrench_dimension:
            raise ValueError("physical LQI R dimension does not match projected wrench")
        if len(self.integral_q_diagonal) != output_count:
            raise ValueError("physical LQI integral-Q dimension does not match tracked outputs")
        if len(self.state_scales) != self.projection.state_dimension:
            raise ValueError("physical LQI state-scale dimension does not match projected state")
        if len(self.wrench_scales) != self.projection.wrench_dimension:
            raise ValueError("physical LQI wrench-scale dimension does not match projected wrench")
        if any(
            not math.isfinite(value) or value <= 0.0
            for value in (*self.q_diagonal, *self.r_diagonal, *self.integral_q_diagonal)
        ):
            raise ValueError("physical LQI weights must be finite and positive")
        if any(not math.isfinite(value) or value <= 0.0 for value in (*self.state_scales, *self.wrench_scales)):
            raise ValueError("physical LQI scales must be finite and positive")
        if tuple(self.result.state_names) != self.projection.state_names:
            raise ValueError("physical LQI state names do not match projected plant")
        if tuple(self.result.control_names) != self.projection.wrench_names:
            raise ValueError("physical LQI wrench names do not match projected plant")
        ####

    def build_controller(
        self,
        *,
        wrench_lower: Mapping[str, float] | None = None,
        wrench_upper: Mapping[str, float] | None = None,
        integral_lower: Mapping[str, float] | None = None,
        integral_upper: Mapping[str, float] | None = None,
    ) -> LqiController:
        """Build a named offset-free wrench controller around the source trim.

        Wrench bounds are optional and deliberately separate from actuator
        limits.  The downstream allocator remains authoritative for coupled
        rotor/surface saturation and logs every realized control value.
        """

        return LqiController(
            self.result,
            state_trim=self.projection.trim_state,
            control_trim=self.projection.nominal_wrench,
            output_trim={name: float(self.projection.trim_state[name]) for name in self.result.output_names},
            lower=wrench_lower or {},
            upper=wrench_upper or {},
            integral_lower=integral_lower or {},
            integral_upper=integral_upper or {},
        )
        ####

    def as_dict(self) -> dict[str, Any]:
        """Return the design and explicit output-integrator provenance."""

        return {
            "id": self.id,
            "method": "lqi",
            "projection": self.projection.as_dict(),
            "output_names": list(self.result.output_names),
            "q_diagonal": list(self.q_diagonal),
            "r_diagonal": list(self.r_diagonal),
            "integral_q_diagonal": list(self.integral_q_diagonal),
            "state_scales": list(self.state_scales),
            "wrench_scales": list(self.wrench_scales),
            "state_gain": np.asarray(self.result.state_gain, dtype=float).tolist(),
            "integral_gain": np.asarray(self.result.integral_gain, dtype=float).tolist(),
            "closed_loop_poles": [
                {"real": float(value.real), "imaginary": float(value.imag)}
                for value in self.result.design.closed_loop_eigenvalues
            ],
            "maximum_real_pole": self.result.maximum_real_pole,
            "controllable": self.result.design.controllable,
            "matrix_sha256": {
                "a": self.result.design.a_sha256,
                "b": self.result.design.b_sha256,
                "q": self.result.design.q_sha256,
                "r": self.result.design.r_sha256,
                "k": self.result.design.k_sha256,
            },
        }
        ####

    ####


def apply_tuning_context_to_physical_wrench_lqr_design(
    design: PhysicalWrenchLqrDesign,
    context: TuningApplicationContext,
) -> tuple[PhysicalWrenchLqrDesign, RuntimeTuningBindingReceipt]:
    """Instantiate one exact common LQR candidate in a physical-wrench runtime.

    The common campaign owns gain synthesis, while this seam verifies the
    selected candidate still has the exact source-derived state-to-wrench
    coordinates, scales, weights, and stable closed loop required by the
    nonlinear allocator-backed runtime.
    """

    context.require_runtime_compatibility(
        controller_method="lqr",
        state_names=design.projection.state_names,
        control_names=design.projection.wrench_names,
    )
    if tuple(context.state_scales) != design.state_scales:
        raise ValueError("tuning application context state scales do not match the physical-wrench runtime")
    if tuple(context.control_scales) != design.wrench_scales:
        raise ValueError("tuning application context control scales do not match the physical-wrench runtime")

    gain = np.asarray(context.resolved_gains["state_gain"], dtype=float)
    q_diagonal = _context_weight_diagonal(context, "q_diagonal", design.projection.state_dimension)
    r_diagonal = _context_weight_diagonal(context, "r_diagonal", design.projection.wrench_dimension)
    a_matrix = np.asarray(design.projection.a_matrix, dtype=float)
    b_matrix = np.asarray(design.projection.b_matrix, dtype=float)
    closed_loop_eigenvalues = np.linalg.eigvals(a_matrix - b_matrix @ gain)
    if not np.all(np.isfinite(closed_loop_eigenvalues)) or np.any(np.real(closed_loop_eigenvalues) >= 0.0):
        raise ValueError("tuning application context is not Hurwitz in the physical-wrench runtime")

    runtime_result = replace(
        design.result,
        gain=gain,
        closed_loop_eigenvalues=closed_loop_eigenvalues,
        q_sha256=_matrix_sha256(np.diag(q_diagonal)),
        r_sha256=_matrix_sha256(np.diag(r_diagonal)),
        k_sha256=_matrix_sha256(gain),
    )
    applied = replace(
        design,
        result=runtime_result,
        q_diagonal=q_diagonal,
        r_diagonal=r_diagonal,
    )
    receipt = context.runtime_binding_after_application(
        controller_method="lqr",
        state_names=applied.projection.state_names,
        control_names=applied.projection.wrench_names,
    )
    return applied, receipt
    ####


def apply_tuning_context_to_physical_wrench_lqi_design(
    design: PhysicalWrenchLqiDesign,
    context: TuningApplicationContext,
) -> tuple[PhysicalWrenchLqiDesign, RuntimeTuningBindingReceipt]:
    """Instantiate one exact common LQI candidate in a physical-wrench runtime.

    This is the reusable, fail-closed seam between a selected common campaign
    candidate and an allocator-backed nonlinear screen.  It deliberately
    rejects coordinate, scale, output-map, weight, and stability drift before
    returning a receipt that can be published as runtime evidence.
    """

    context.require_runtime_compatibility(
        controller_method="lqi",
        state_names=design.projection.state_names,
        control_names=design.projection.wrench_names,
        integral_output_names=design.result.output_names,
    )
    if tuple(context.state_scales) != design.state_scales:
        raise ValueError("tuning application context state scales do not match the physical-wrench runtime")
    if tuple(context.control_scales) != design.wrench_scales:
        raise ValueError("tuning application context control scales do not match the physical-wrench runtime")

    output_matrix = np.asarray(context.resolved_gains["output_matrix"], dtype=float)
    state_gain = np.asarray(context.resolved_gains["state_gain"], dtype=float)
    integral_gain = np.asarray(context.resolved_gains["integral_gain"], dtype=float)
    if not np.array_equal(output_matrix, np.asarray(design.result.output_matrix, dtype=float)):
        raise ValueError("tuning application context output matrix does not match the physical-wrench runtime")

    q_diagonal = _context_weight_diagonal(context, "q_diagonal", design.projection.state_dimension)
    r_diagonal = _context_weight_diagonal(context, "r_diagonal", design.projection.wrench_dimension)
    integral_q_diagonal = _context_weight_diagonal(
        context,
        "integral_q_diagonal",
        len(design.result.output_names),
    )
    augmented_gain = np.hstack((state_gain, integral_gain))
    a_matrix = np.asarray(design.projection.a_matrix, dtype=float)
    b_matrix = np.asarray(design.projection.b_matrix, dtype=float)
    integral_dimension = len(design.result.output_names)
    augmented_a = np.block(
        [
            [a_matrix, np.zeros((a_matrix.shape[0], integral_dimension))],
            [output_matrix, np.zeros((integral_dimension, integral_dimension))],
        ]
    )
    augmented_b = np.vstack((b_matrix, np.zeros((integral_dimension, b_matrix.shape[1]))))
    closed_loop_eigenvalues = np.linalg.eigvals(augmented_a - augmented_b @ augmented_gain)
    if not np.all(np.isfinite(closed_loop_eigenvalues)) or np.any(np.real(closed_loop_eigenvalues) >= 0.0):
        raise ValueError("tuning application context is not Hurwitz in the physical-wrench runtime")

    q_matrix = np.diag((*q_diagonal, *integral_q_diagonal))
    r_matrix = np.diag(r_diagonal)
    runtime_design = replace(
        design.result.design,
        gain=augmented_gain,
        closed_loop_eigenvalues=closed_loop_eigenvalues,
        q_sha256=_matrix_sha256(q_matrix),
        r_sha256=_matrix_sha256(r_matrix),
        k_sha256=_matrix_sha256(augmented_gain),
    )
    runtime_result = replace(
        design.result,
        design=runtime_design,
        output_matrix=output_matrix,
        state_gain=state_gain,
        integral_gain=integral_gain,
    )
    applied = replace(
        design,
        result=runtime_result,
        q_diagonal=q_diagonal,
        r_diagonal=r_diagonal,
        integral_q_diagonal=integral_q_diagonal,
    )
    receipt = context.runtime_binding_after_application(
        controller_method="lqi",
        state_names=applied.projection.state_names,
        control_names=applied.projection.wrench_names,
        integral_output_names=applied.result.output_names,
    )
    return applied, receipt
    ####


def _context_weight_diagonal(
    context: TuningApplicationContext,
    name: str,
    expected_dimension: int,
) -> tuple[float, ...]:
    """Read one finite positive diagonal from the exact selected candidate."""

    values = context.weights.get(name)
    if values is None or len(values) != expected_dimension:
        raise ValueError(f"tuning application context {name!r} does not match the physical-wrench runtime")
    resolved = tuple(float(value) for value in values)
    if any(not math.isfinite(value) or value <= 0.0 for value in resolved):
        raise ValueError(f"tuning application context {name!r} must contain finite positive weights")
    return resolved
    ####


def _matrix_sha256(matrix: np.ndarray) -> str:
    """Fingerprint one finite gain or weight matrix for runtime provenance."""

    array = np.ascontiguousarray(np.asarray(matrix, dtype=np.float64))
    if not np.isfinite(array).all():
        raise ValueError("physical-wrench tuning matrix fingerprint requires finite values")
    header = json.dumps({"shape": list(array.shape), "dtype": str(array.dtype)}, sort_keys=True).encode("utf-8")
    return hashlib.sha256(header + b"\0" + array.tobytes(order="C")).hexdigest()
    ####


@dataclass(frozen=True, slots=True)
class PhysicalWrenchLqrScheduleNode:
    """One physical-wrench LQR design at a scalar operating coordinate."""

    coordinate: float
    design: PhysicalWrenchLqrDesign

    def __post_init__(self) -> None:
        if not math.isfinite(self.coordinate):
            raise ValueError("physical-wrench schedule coordinates must be finite")
        ####
    ####


@dataclass(frozen=True, slots=True)
class ScheduledPhysicalWrenchCommand:
    """Interpolated wrench demand that still requires physical allocation."""

    coordinate: float
    lower_node: str
    upper_node: str
    interpolation_fraction: float
    requested_wrench: Mapping[str, float]
    wrench_increment: Mapping[str, float]

    def as_dict(self) -> dict[str, object]:
        """Return schedule provenance alongside the demand."""

        return {
            "coordinate": self.coordinate,
            "lower_node": self.lower_node,
            "upper_node": self.upper_node,
            "interpolation_fraction": self.interpolation_fraction,
            "requested_wrench": dict(self.requested_wrench),
            "wrench_increment": dict(self.wrench_increment),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class PhysicalWrenchLqrSchedule:
    """Continuously interpolate local wrench demands between validated nodes.

    The schedule interpolates gain, trim state, nominal wrench, and node
    provenance. It never applies a force or moment directly; callers must pass
    the returned demand through the plant's bounded allocator. This makes the
    schedule useful for fixed-wing, rotorcraft, spacecraft, and other adapters
    without changing their physical-effector boundary.
    """

    nodes: tuple[PhysicalWrenchLqrScheduleNode, ...]

    def __post_init__(self) -> None:
        if not self.nodes:
            raise ValueError("physical-wrench schedule requires at least one node")
        coordinates = tuple(node.coordinate for node in self.nodes)
        if any(right <= left for left, right in zip(coordinates, coordinates[1:], strict=False)):
            raise ValueError("physical-wrench schedule coordinates must be strictly increasing")
        first = self.nodes[0].design
        for node in self.nodes[1:]:
            design = node.design
            if design.projection.state_names != first.projection.state_names:
                raise ValueError("physical-wrench schedule state channels must match")
            if design.projection.wrench_names != first.projection.wrench_names:
                raise ValueError("physical-wrench schedule wrench channels must match")
            if design.result.gain.shape != first.result.gain.shape:
                raise ValueError("physical-wrench schedule gain dimensions must match")
        ####
    ####


    @property
    def state_names(self) -> tuple[str, ...]:
        """Return the scheduled state channels."""

        return self.nodes[0].design.projection.state_names
        ####
    ####

    @property
    def wrench_names(self) -> tuple[str, ...]:
        """Return the scheduled wrench channels."""

        return self.nodes[0].design.projection.wrench_names
        ####
    ####

    def _bracket(self, coordinate: float) -> tuple[PhysicalWrenchLqrScheduleNode, PhysicalWrenchLqrScheduleNode, float]:
        if not math.isfinite(coordinate):
            raise ValueError("physical-wrench schedule coordinate must be finite")
        if coordinate <= self.nodes[0].coordinate:
            return self.nodes[0], self.nodes[0], 0.0
        if coordinate >= self.nodes[-1].coordinate:
            return self.nodes[-1], self.nodes[-1], 0.0
        for lower, upper in zip(self.nodes[:-1], self.nodes[1:], strict=True):
            if lower.coordinate <= coordinate <= upper.coordinate:
                fraction = (coordinate - lower.coordinate) / (upper.coordinate - lower.coordinate)
                return lower, upper, fraction
        raise RuntimeError("physical-wrench schedule failed to bracket coordinate")
        ####

    def bracket(
        self,
        coordinate: float,
    ) -> tuple[PhysicalWrenchLqrScheduleNode, PhysicalWrenchLqrScheduleNode, float]:
        """Return the public schedule bracket used by transition replays.

        A transition runner needs the same endpoint provenance as
        :meth:`command`.  Exposing that lookup here prevents family tools from
        duplicating coordinate clamping and interpolation-boundary rules.
        """

        return self._bracket(coordinate)
        ####

    def state_reference(self, coordinate: float) -> dict[str, float]:
        """Return the interpolated trim-state reference at ``coordinate``."""

        lower, upper, fraction = self._bracket(coordinate)
        return self._interpolate_mapping(
            lower.design.projection.trim_state,
            upper.design.projection.trim_state,
            fraction,
            self.state_names,
        )
        ####

    @staticmethod
    def _interpolate_mapping(
        lower: Mapping[str, float],
        upper: Mapping[str, float],
        fraction: float,
        names: Sequence[str],
    ) -> dict[str, float]:
        return {
            name: (1.0 - fraction) * float(lower[name]) + fraction * float(upper[name])
            for name in names
        }
        ####

    def command(
        self,
        state: Mapping[str, float],
        coordinate: float,
        *,
        state_reference: Mapping[str, float] | None = None,
    ) -> ScheduledPhysicalWrenchCommand:
        """Return an interpolated demand for the current operating coordinate."""

        lower, upper, fraction = self._bracket(coordinate)
        lower_design = lower.design
        upper_design = upper.design
        state_names = self.state_names
        wrench_names = self.wrench_names
        missing = set(state_names) - set(state)
        if missing:
            raise KeyError(f"scheduled physical LQR state is missing: {', '.join(sorted(missing))}")
        reference = self._interpolate_mapping(
            lower_design.projection.trim_state,
            upper_design.projection.trim_state,
            fraction,
            state_names,
        )
        if state_reference is not None:
            missing_reference = set(state_names) - set(state_reference)
            if missing_reference:
                raise KeyError(f"scheduled physical LQR reference is missing: {', '.join(sorted(missing_reference))}")
            reference = {name: float(state_reference[name]) for name in state_names}
        error = np.asarray([float(state[name]) - reference[name] for name in state_names], dtype=float)
        lower_gain = np.asarray(lower_design.result.gain, dtype=float)
        upper_gain = np.asarray(upper_design.result.gain, dtype=float)
        gain = (1.0 - fraction) * lower_gain + fraction * upper_gain
        increment_values = -gain @ error
        lower_nominal = lower_design.projection.nominal_wrench
        upper_nominal = upper_design.projection.nominal_wrench
        nominal = self._interpolate_mapping(lower_nominal, upper_nominal, fraction, wrench_names)
        increment = {name: float(value) for name, value in zip(wrench_names, increment_values, strict=True)}
        requested = {name: nominal[name] + increment[name] for name in wrench_names}
        return ScheduledPhysicalWrenchCommand(
            coordinate=float(coordinate),
            lower_node=lower_design.id,
            upper_node=upper_design.id,
            interpolation_fraction=float(fraction),
            requested_wrench=requested,
            wrench_increment=increment,
        )
        ####
    ####


class ScheduledPhysicalPlant(Protocol):
    """Minimal plant seam required by a scheduled time-marching witness."""

    @property
    def state_names(self) -> Sequence[str]:
        """Return the state ordering used by the schedule."""
        ...

    @property
    def control_names(self) -> Sequence[str]:
        """Return the physical effectors used by the schedule."""
        ...

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Evaluate the declared plant derivative."""
        ...

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        """Allocate a demand through bounded physical effectors."""
        ...


def run_scheduled_physical_wrench_transition(
    schedule: PhysicalWrenchLqrSchedule,
    *,
    start_coordinate: float,
    end_coordinate: float,
    initial_state: Mapping[str, float],
    initial_effectors: Mapping[str, float],
    plant_for_coordinate: Callable[[float], ScheduledPhysicalPlant],
    state_scales: Sequence[float],
    duration_s: float,
    dt_s: float,
    perturbation: Mapping[str, float] | None = None,
    environment_for_coordinate: Callable[[float], Mapping[str, float | str]] | None = None,
    state_reference_for_coordinate: Callable[[float], Mapping[str, float]] | None = None,
    recovery_threshold: float = 0.25,
    minimum_final_norm: float = 0.05,
    sample_stride_steps: int = 60,
) -> dict[str, Any]:
    """Replay a scheduled physical controller through a bounded plant.

    This is the family-neutral time-marching contract for scheduled local
    evidence.  At each committed sample it computes a scheduled wrench
    demand, allocates that demand through the plant's actual effectors, and
    then integrates the state with the committed actuator positions held over
    the accepted RK4 interval.  The function never applies the requested
    wrench directly.

    ``plant_for_coordinate`` is intentionally supplied by the family adapter:
    it may select a source table node, a validated endpoint blend, a rotor
    model, or a spacecraft actuator model.  The generic runner owns only the
    control/evidence protocol and therefore cannot silently invent family
    dynamics or actuator mappings.
    """

    if not math.isfinite(start_coordinate) or not math.isfinite(end_coordinate):
        raise ValueError("scheduled transition coordinates must be finite")
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("scheduled transition duration must be finite and positive")
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("scheduled transition step must be finite and positive")
    if not math.isfinite(recovery_threshold) or recovery_threshold < 0.0:
        raise ValueError("scheduled transition recovery threshold must be finite and nonnegative")
    if not math.isfinite(minimum_final_norm) or minimum_final_norm < 0.0:
        raise ValueError("scheduled transition minimum final norm must be finite and nonnegative")
    if sample_stride_steps <= 0:
        raise ValueError("scheduled transition sample stride must be positive")
    state_names = schedule.state_names
    state_scales = tuple(float(value) for value in state_scales)
    if len(state_scales) != len(state_names) or any(not math.isfinite(value) or value <= 0.0 for value in state_scales):
        raise ValueError("scheduled transition state scales must be finite and positive")
    missing_state = set(state_names) - set(initial_state)
    if missing_state:
        raise KeyError(f"scheduled transition initial state is missing: {', '.join(sorted(missing_state))}")
    if not math.isfinite(start_coordinate) or not math.isfinite(end_coordinate):
        raise ValueError("scheduled transition coordinates must be finite")
    perturbation_values = {name: float(value) for name, value in (perturbation or {}).items()}
    unknown_perturbations = set(perturbation_values) - set(state_names)
    if unknown_perturbations:
        raise KeyError(f"scheduled transition perturbation is missing state channels: {', '.join(sorted(unknown_perturbations))}")
    state = {name: float(initial_state[name]) + perturbation_values.get(name, 0.0) for name in state_names}
    if any(not math.isfinite(value) for value in state.values()):
        raise ValueError("scheduled transition initial state must be finite")
    actual_effectors = {name: float(value) for name, value in initial_effectors.items()}
    plant0 = plant_for_coordinate(start_coordinate)
    if tuple(plant0.state_names) != state_names:
        raise ValueError("scheduled transition plant state channels do not match schedule")
    if set(actual_effectors) != set(plant0.control_names):
        raise ValueError("scheduled transition effectors do not match the starting plant")
    environment_for_coordinate = environment_for_coordinate or (lambda coordinate: {})
    state_reference_for_coordinate = state_reference_for_coordinate or schedule.state_reference

    def normalized_error(values: Mapping[str, float], coordinate: float) -> float:
        reference = state_reference_for_coordinate(coordinate)
        missing_reference = set(state_names) - set(reference)
        if missing_reference:
            raise KeyError(
                "scheduled transition reference is missing: " + ", ".join(sorted(missing_reference))
            )
        return math.sqrt(
            sum(
                ((float(values[name]) - float(reference[name])) / scale) ** 2
                for name, scale in zip(state_names, state_scales, strict=True)
            )
        )

    def rk4_step(
        plant: ScheduledPhysicalPlant,
        values: Mapping[str, float],
        coordinate: float,
        step_s: float,
    ) -> dict[str, float]:
        environment = environment_for_coordinate(coordinate)

        def derivative(candidate: Mapping[str, float]) -> np.ndarray:
            result = plant.state_derivative(candidate, actual_effectors, environment)
            vector = np.asarray([float(result[name]) for name in state_names], dtype=float)
            if not np.all(np.isfinite(vector)):
                raise ValueError("scheduled physical transition derivative is non-finite")
            return vector

        base = np.asarray([float(values[name]) for name in state_names], dtype=float)
        k1 = derivative(values)
        k2 = derivative({name: float(value) for name, value in zip(state_names, base + 0.5 * step_s * k1, strict=True)})
        k3 = derivative({name: float(value) for name, value in zip(state_names, base + 0.5 * step_s * k2, strict=True)})
        k4 = derivative({name: float(value) for name, value in zip(state_names, base + step_s * k3, strict=True)})
        result = base + step_s * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        if not np.all(np.isfinite(result)):
            raise ValueError("scheduled physical transition state is non-finite")
        return {name: float(value) for name, value in zip(state_names, result, strict=True)}

    initial_norm = normalized_error(state, start_coordinate)
    statuses: list[str] = []
    saturation_steps = 0
    max_residual = 0.0
    max_norm = initial_norm
    samples: list[dict[str, Any]] = []
    steps = int(math.ceil(duration_s / dt_s))
    time_s = 0.0
    for step_index in range(steps):
        elapsed = min(step_index * dt_s, duration_s)
        fraction = 0.0 if duration_s == 0.0 else elapsed / duration_s
        coordinate = start_coordinate + (end_coordinate - start_coordinate) * fraction
        plant = plant_for_coordinate(coordinate)
        command = schedule.command(state, coordinate)
        allocation = plant.allocate(state, dict(command.requested_wrench), actual_effectors, dt_s)
        statuses.append(allocation.allocation.status)
        constrained = (
            allocation.allocation.status != "feasible"
            or bool(allocation.actuator.position_saturated)
            or bool(allocation.actuator.rate_limited)
            or bool(allocation.actuator.unavailable_effectors)
        )
        if constrained:
            saturation_steps += 1
        max_residual = max(max_residual, allocation.achieved_controlled_residual_norm)
        norm = normalized_error(state, coordinate)
        max_norm = max(max_norm, norm)
        if step_index % sample_stride_steps == 0 or step_index == steps - 1:
            lower, upper, interpolation_fraction = schedule.bracket(coordinate)
            samples.append(
                {
                    "time_s": elapsed,
                    "coordinate": coordinate,
                    "lower_node": lower.design.id,
                    "upper_node": upper.design.id,
                    "interpolation_fraction": interpolation_fraction,
                    "normalized_error": norm,
                    "allocation_status": allocation.allocation.status,
                    "allocation_residual": allocation.achieved_controlled_residual_norm,
                    "state": dict(state),
                    "environment": dict(environment_for_coordinate(coordinate)),
                    "requested_wrench": dict(command.requested_wrench),
                    "achieved_wrench": dict(allocation.achieved_wrench),
                    "residual_wrench": dict(allocation.achieved_residual_wrench),
                    "actual_effectors": dict(allocation.actuator.actual_positions),
                    "position_saturated": list(allocation.actuator.position_saturated),
                    "rate_limited": list(allocation.actuator.rate_limited),
                }
            )
        actual_effectors = dict(allocation.actuator.actual_positions)
        step = min(dt_s, duration_s - time_s)
        if step <= 0.0:
            break
        state = rk4_step(plant, state, coordinate, step)
        time_s += step

    final_norm = normalized_error(state, end_coordinate)
    disallowed = {"infeasible", "partially_achievable", "numerically_singular", "solver_failure"}
    passed = (
        final_norm <= max(initial_norm * recovery_threshold, minimum_final_norm)
        and not disallowed.intersection(statuses)
        and saturation_steps == 0
        and math.isfinite(max_residual)
    )
    return {
        "passed": passed,
        "start_coordinate": start_coordinate,
        "end_coordinate": end_coordinate,
        "duration_s": duration_s,
        "dt_s": dt_s,
        "perturbation": perturbation_values,
        "initial_command": schedule.command(state, start_coordinate).as_dict(),
        "initial_normalized_error": initial_norm,
        "final_normalized_error": final_norm,
        "maximum_normalized_error": max_norm,
        "maximum_controlled_allocation_residual": max_residual,
        "allocation_statuses": sorted(set(statuses)),
        "saturation_steps": saturation_steps,
        "samples": samples,
        "direct_wrench_injection": False,
        "control_path": "scheduled wrench demand -> bounded physical effectors -> accepted plant derivative",
    }
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
        """Return the selected-state error norm before the first interval."""

        return _feedback_error_norm(
            {
                name: float(self.initial_state[name]) - float(self.design.projection.trim_state[name])
                for name in self.design.projection.state_names
            },
            self.design.projection.state_names,
        )
        ####
    ####

    @property
    def final_feedback_error_norm(self) -> float:
        """Return the final selected-state error norm."""

        return _feedback_error_norm(
            {
                name: float(self.final_state[name]) - float(self.design.projection.trim_state[name])
                for name in self.design.projection.state_names
            },
            self.design.projection.state_names,
        )
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


@dataclass(frozen=True, slots=True)
class PhysicalWrenchLqiSample:
    """One accepted end-of-interval nonlinear physical LQI sample.

    ``state`` is the accepted state at ``time_s`` after the held, allocator-
    realized effector command advanced one interval.  The sample preserves the
    output-integrator state alongside its exact requested and realized wrench.
    This makes offset-free control inspectable without treating the integral
    term as an injected load.
    """

    time_s: float
    state: Mapping[str, float]
    state_error: Mapping[str, float]
    lqi_integral_error: Mapping[str, float]
    lqi_wrench_command: Mapping[str, float]
    lqi_wrench_saturated: tuple[str, ...]
    requested_wrench: Mapping[str, float]
    allocation: PhysicalAllocationStep

    def as_dict(self) -> dict[str, Any]:
        """Return allocator-backed LQI telemetry for one truth sample."""

        payload = PhysicalWrenchLqrSample(
            self.time_s,
            self.state,
            self.state_error,
            self.lqi_wrench_command,
            self.requested_wrench,
            self.allocation,
        ).as_dict()
        payload["lqi_integral_error"] = dict(self.lqi_integral_error)
        payload["lqi_wrench_command"] = dict(self.lqi_wrench_command)
        payload["lqi_wrench_saturated"] = list(self.lqi_wrench_saturated)
        payload.pop("lqr_wrench_increment")
        return payload
        ####
    ####


@dataclass(frozen=True, slots=True)
class PhysicalWrenchLqiValidation:
    """Deterministic local nonlinear evidence for a plant-derived LQI.

    A completed record proves only this bounded local controller execution.
    It does not infer disturbance rejection, scheduling, navigation, or an
    operating envelope beyond the caller's declared test conditions.
    """

    design: PhysicalWrenchLqiDesign
    duration_s: float
    dt_s: float
    initial_state: Mapping[str, float]
    final_state: Mapping[str, float]
    reference: Mapping[str, float]
    environment: Mapping[str, float | str]
    samples: tuple[PhysicalWrenchLqiSample, ...]

    def __post_init__(self) -> None:
        if not math.isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("physical LQI validation duration must be finite and positive")
        if not math.isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise ValueError("physical LQI validation step must be finite and positive")
        if not self.samples:
            raise ValueError("physical LQI validation requires telemetry samples")
        missing_reference = set(self.design.result.output_names) - set(self.reference)
        if missing_reference:
            raise KeyError(f"physical LQI validation reference is missing: {', '.join(sorted(missing_reference))}")
        if any(not math.isfinite(float(value)) for value in self.reference.values()):
            raise ValueError("physical LQI validation reference must be finite")
        if any(
            isinstance(value, bool) or not isinstance(value, int | float | str)
            or (isinstance(value, int | float) and not math.isfinite(float(value)))
            for value in self.environment.values()
        ):
            raise ValueError("physical LQI validation environment values must be finite numeric or text")
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
        """Return fraction of samples affected by a physical limitation."""

        return sum(_sample_is_constrained(sample) for sample in self.samples) / len(self.samples)
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
    ####

    @property
    def initial_normalized_feedback_error_norm(self) -> float:
        """Return the initial feedback error in the declared LQI state units."""

        return _normalized_feedback_error_norm(
            {
                name: float(self.initial_state[name]) - float(self.design.projection.trim_state[name])
                for name in self.design.projection.state_names
            },
            self.design.projection.state_names,
            self.design.state_scales,
        )
        ####
    ####

    @property
    def final_normalized_feedback_error_norm(self) -> float:
        """Return the final feedback error in the declared LQI state units."""

        return _normalized_feedback_error_norm(
            {
                name: float(self.final_state[name]) - float(self.design.projection.trim_state[name])
                for name in self.design.projection.state_names
            },
            self.design.projection.state_names,
            self.design.state_scales,
        )
        ####
    ####

    @property
    def integrators_exercised(self) -> bool:
        """Return whether any explicitly declared output integral changed."""

        return any(
            abs(float(value)) > 1.0e-12
            for sample in self.samples
            for value in sample.lqi_integral_error.values()
        )
        ####
    ####

    def as_dict(self) -> dict[str, Any]:
        """Return a self-contained local physical-LQI validation artifact."""

        return {
            "schema": "taoryx.physical-lqi-validation/v1alpha1",
            "design": self.design.as_dict(),
            "duration_s": self.duration_s,
            "dt_s": self.dt_s,
            "initial_state": dict(self.initial_state),
            "final_state": dict(self.final_state),
            "reference": dict(self.reference),
            "environment": dict(self.environment),
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
                "integrators_exercised": self.integrators_exercised,
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


def design_physical_wrench_lqi(
    identifier: str,
    projection: WrenchLinearizationProjection,
    *,
    output_names: Sequence[str],
    q_diagonal: Sequence[float],
    r_diagonal: Sequence[float],
    integral_q_diagonal: Sequence[float],
    state_scales: Sequence[float],
    wrench_scales: Sequence[float],
) -> PhysicalWrenchLqiDesign:
    """Synthesize offset-free wrench feedback from real plant derivatives.

    ``output_names`` are local plant coordinates to be regulated against
    persistent matched bias.  The resulting command is a desired wrench in
    the projection's coordinates and must still traverse physical allocation
    before the nonlinear plant advances.
    """

    outputs = tuple(output_names)
    q_diagonal = tuple(float(value) for value in q_diagonal)
    r_diagonal = tuple(float(value) for value in r_diagonal)
    integral_q_diagonal = tuple(float(value) for value in integral_q_diagonal)
    state_scales = tuple(float(value) for value in state_scales)
    wrench_scales = tuple(float(value) for value in wrench_scales)
    if not outputs or len(set(outputs)) != len(outputs) or set(outputs) - set(projection.state_names):
        raise ValueError("physical wrench LQI outputs must be unique projected state names")
    if len(q_diagonal) != projection.state_dimension or len(state_scales) != projection.state_dimension:
        raise ValueError("physical wrench LQI state weights/scales must match projected state dimension")
    if len(r_diagonal) != projection.wrench_dimension or len(wrench_scales) != projection.wrench_dimension:
        raise ValueError("physical wrench LQI control weights/scales must match projected wrench dimension")
    if len(integral_q_diagonal) != len(outputs):
        raise ValueError("physical wrench LQI integral weights must match tracked outputs")
    if any(not math.isfinite(value) or value <= 0.0 for value in (*q_diagonal, *r_diagonal, *integral_q_diagonal)):
        raise ValueError("physical wrench LQI weights must be finite and positive")
    output_matrix = tuple(
        tuple(1.0 if state_name == output_name else 0.0 for state_name in projection.state_names)
        for output_name in outputs
    )
    q_values = (*q_diagonal, *integral_q_diagonal)
    q = tuple(
        tuple(value if row_index == column_index else 0.0 for column_index in range(len(q_values)))
        for row_index, value in enumerate(q_values)
    )
    r = tuple(
        tuple(value if row_index == column_index else 0.0 for column_index in range(projection.wrench_dimension))
        for row_index, value in enumerate(r_diagonal)
    )
    result = solve_scaled_continuous_lqi(
        projection.a_matrix,
        projection.b_matrix,
        q,
        r,
        output_matrix=output_matrix,
        output_names=outputs,
        state_scales=state_scales,
        control_scales=wrench_scales,
        state_names=projection.state_names,
        control_names=projection.wrench_names,
    )
    if not result.hurwitz:
        raise ValueError(f"physical wrench LQI is not strictly stable: maximum real pole {result.maximum_real_pole:.6g}")
    return PhysicalWrenchLqiDesign(
        identifier,
        projection,
        result,
        q_diagonal,
        r_diagonal,
        integral_q_diagonal,
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


def validate_nonlinear_wrench_lqi(
    plant: ControlPlantAdapter,
    trim: TrimResult,
    design: PhysicalWrenchLqiDesign,
    *,
    initial_state: Mapping[str, float],
    duration_s: float,
    dt_s: float,
    reference: Mapping[str, float] | None = None,
    wrench_lower: Mapping[str, float] | None = None,
    wrench_upper: Mapping[str, float] | None = None,
    integral_lower: Mapping[str, float] | None = None,
    integral_upper: Mapping[str, float] | None = None,
    environment: Mapping[str, float | str] | None = None,
) -> PhysicalWrenchLqiValidation:
    """Run a bounded nonlinear LQI screen through actual physical effectors.

    The controller produces only desired wrench coordinates.  At every
    committed sample those coordinates are allocated through the plant's
    declared effectors and the accepted actual positions are held over the
    RK4 interval.  This is the physical LQI counterpart to
    :func:`validate_nonlinear_wrench_lqr`; it never injects integral actions
    directly into the dynamics.
    """

    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("nonlinear physical LQI duration must be finite and positive")
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("nonlinear physical LQI time step must be finite and positive")
    required_state_names = tuple(plant.state_names)
    missing_state = set(required_state_names) - set(initial_state)
    if missing_state:
        raise KeyError(f"initial nonlinear state is missing: {', '.join(sorted(missing_state))}")
    if tuple(trim.spec.state_names) != required_state_names:
        raise ValueError("nonlinear physical LQI trim does not match the adapter state contract")
    if tuple(trim.spec.control_names) != tuple(plant.control_names):
        raise ValueError("nonlinear physical LQI trim does not match the adapter effector contract")
    if set(design.projection.state_names) - set(required_state_names):
        raise ValueError("physical LQI design refers to states absent from adapter")
    state = {name: float(initial_state[name]) for name in required_state_names}
    if any(not math.isfinite(value) for value in state.values()):
        raise ValueError("nonlinear physical LQI initial state must be finite")
    tracked_reference = {name: float(trim.state[name]) for name in design.result.output_names}
    if reference is not None:
        missing_reference = set(design.result.output_names) - set(reference)
        if missing_reference:
            raise KeyError(f"nonlinear physical LQI reference is missing: {', '.join(sorted(missing_reference))}")
        unknown_reference = set(reference) - set(design.result.output_names)
        if unknown_reference:
            raise KeyError(f"nonlinear physical LQI reference names unknown outputs: {', '.join(sorted(unknown_reference))}")
        tracked_reference = {name: float(reference[name]) for name in design.result.output_names}
    if any(not math.isfinite(value) for value in tracked_reference.values()):
        raise ValueError("nonlinear physical LQI reference must be finite")
    derivative_environment = dict(environment or {})
    if any(
        isinstance(value, bool) or not isinstance(value, int | float | str)
        or (isinstance(value, int | float) and not math.isfinite(float(value)))
        for value in derivative_environment.values()
    ):
        raise ValueError("nonlinear physical LQI environment values must be finite numeric or text")
    controller = design.build_controller(
        wrench_lower=wrench_lower,
        wrench_upper=wrench_upper,
        integral_lower=integral_lower,
        integral_upper=integral_upper,
    )
    actual_effectors = {name: float(trim.controls[name]) for name in plant.control_names}
    nominal_wrench = dict(design.projection.nominal_wrench)
    samples: list[PhysicalWrenchLqiSample] = []
    steps = int(math.ceil(duration_s / dt_s))
    time_s = 0.0
    for _ in range(steps):
        command = controller.command(state, tracked_reference, dt=dt_s)
        requested = dict(nominal_wrench)
        requested.update({name: float(value) for name, value in command.controls.items()})
        requested = {name: float(requested.get(name, nominal_wrench[name])) for name in nominal_wrench}
        allocation = plant.allocate(state, requested, actual_effectors, dt_s)
        if tuple(allocation.allocation.controlled_wrench_axes) != design.projection.wrench_names:
            raise ValueError(
                "plant allocation controlled axes do not match the physical LQI design: "
                f"{allocation.allocation.controlled_wrench_axes!r} != {design.projection.wrench_names!r}"
            )
        actual_effectors = dict(allocation.actuator.actual_positions)
        step = min(dt_s, duration_s - time_s)
        if step <= 0.0:
            break
        state = _rk4_state_step(plant, state, actual_effectors, step, derivative_environment)
        time_s += step
        state_error = {name: state[name] - float(trim.state[name]) for name in required_state_names}
        samples.append(
            PhysicalWrenchLqiSample(
                time_s,
                dict(state),
                state_error,
                dict(controller.integral_error),
                {name: float(value) for name, value in command.controls.items()},
                tuple(command.saturated),
                requested,
                allocation,
            )
        )
    final_state = dict(state)
    if any(not math.isfinite(value) for value in final_state.values()):
        raise ValueError("physical LQI plant integration produced a non-finite final state")
    return PhysicalWrenchLqiValidation(
        design,
        duration_s,
        dt_s,
        dict(initial_state),
        final_state,
        tracked_reference,
        derivative_environment,
        tuple(samples),
    )
    ####


def _rk4_state_step(
    plant: ControlPlantAdapter,
    state: Mapping[str, float],
    effectors: Mapping[str, float],
    dt_s: float,
    environment: Mapping[str, float | str] | None = None,
) -> dict[str, float]:
    """Advance named local plant states with a held accepted actuator state."""

    names = tuple(plant.state_names)

    def derivative(values: Mapping[str, float]) -> np.ndarray:
        result = plant.state_derivative(values, effectors, environment or {})
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


def _sample_is_constrained(sample: PhysicalWrenchLqrSample | PhysicalWrenchLqiSample) -> bool:
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
