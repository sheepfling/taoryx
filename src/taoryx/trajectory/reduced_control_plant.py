"""Common control-plant seam for declared lower-fidelity vehicle models.

The point-mass and named pseudo-6DOF tiers need the same disciplined
integration path as rigid-body models: a stable state/control schema, a trim
record, and a derivative provenance record.  They do *not* thereby acquire a
physical-effector allocator.  This module packages an already-declared
reduced nonlinear response or force law as a :class:`ControlPlantAdapter`
while keeping its lower evidence boundary explicit.

Families supply the actual derivative and trim functions.  The reusable
wrapper owns finite-difference settings, schema checks, and the guarantee
that ``effectiveness`` and ``allocate`` remain unavailable for reduced tiers.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping

from ..control_allocation import (
    EffectorEffectiveness,
    PhysicalAllocationStep,
    ProvenancedLinearization,
    finite_difference_linearization_with_provenance,
)
from ..trim import TrimResult, TrimSpec

ReducedDerivativeEvaluator = Callable[
    [Mapping[str, float], Mapping[str, float], Mapping[str, float | str]],
    Mapping[str, float],
]
ReducedTrimProvider = Callable[[Mapping[str, float], Mapping[str, float]], TrimResult]


def declared_equilibrium_trim(
    *,
    state_names: tuple[str, ...],
    control_names: tuple[str, ...],
    residuals: Mapping[str, float],
    state: Mapping[str, float],
    controls: Mapping[str, float],
    operating_point: Mapping[str, float | str] | None = None,
    message: str = "declared lower-tier equilibrium",
) -> TrimResult:
    """Create an auditable trim record after a family evaluates its residuals.

    This helper does not fabricate equilibrium.  A family must calculate the
    residuals through its own declared lower-tier dynamics before calling it.
    It is appropriate where the lower model has an analytic or inherited trim
    rather than an independently solvable physical-effector trim problem.
    """

    if not state_names or not control_names or not residuals:
        raise ValueError("declared lower-tier trim requires state, control, and residual channels")
    if set(state) != set(state_names):
        raise ValueError("declared lower-tier trim state must match the ordered state schema")
    if set(controls) != set(control_names):
        raise ValueError("declared lower-tier trim controls must match the ordered control schema")
    if not all(math.isfinite(float(value)) for value in (*state.values(), *controls.values(), *residuals.values())):
        raise ValueError("declared lower-tier trim values and residuals must be finite")
    spec = TrimSpec(
        state_names=state_names,
        control_names=control_names,
        residual_names=tuple(residuals),
        state_initial={name: float(state[name]) for name in state_names},
        control_initial={name: float(controls[name]) for name in control_names},
        operating_point=dict(operating_point or {}),
    )
    return TrimResult(
        spec=spec,
        state={name: float(state[name]) for name in state_names},
        controls={name: float(controls[name]) for name in control_names},
        residuals={name: float(value) for name, value in residuals.items()},
        scaled_residual_norm=math.sqrt(sum(float(value) ** 2 for value in residuals.values())),
        success=True,
        status=1,
        message=message,
        iterations=0,
        cost=0.0,
    )
    ####


class ReducedOrderControlPlant:
    """Adapt one named 3DOF or pseudo-6DOF nonlinear model to common tooling.

    The supplied derivative evaluator remains the sole source of model
    behavior.  ``trim_provider`` must return a record whose schema exactly
    matches this plant.  The adapter deliberately raises for physical
    effectiveness and allocation, preventing a semantic force or response
    input from being mistaken for a real actuator.
    """

    def __init__(
        self,
        *,
        state_names: tuple[str, ...],
        control_names: tuple[str, ...],
        derivative_evaluator: ReducedDerivativeEvaluator,
        trim_provider: ReducedTrimProvider,
        nonlinear_plant_id: str,
        nonlinear_plant_revision: str,
        state_units: Mapping[str, str],
        control_units: Mapping[str, str],
        claim_boundary: str,
        linearization_metadata: Mapping[str, str | float] | None = None,
    ) -> None:
        self.state_names = state_names
        self.control_names = control_names
        self.derivative_evaluator = derivative_evaluator
        self.trim_provider = trim_provider
        self.nonlinear_plant_id = nonlinear_plant_id
        self.nonlinear_plant_revision = nonlinear_plant_revision
        self.state_units = dict(state_units)
        self.control_units = dict(control_units)
        self.claim_boundary = claim_boundary
        self.linearization_metadata = dict(linearization_metadata or {})
        if not self.state_names or not self.control_names:
            raise ValueError("reduced-order control plant requires state and control channels")
        if len(self.state_names) != len(set(self.state_names)) or len(self.control_names) != len(set(self.control_names)):
            raise ValueError("reduced-order state and control channel names must be unique")
        if set(self.state_units) != set(self.state_names):
            raise ValueError("reduced-order state units must match the state schema")
        if set(self.control_units) != set(self.control_names):
            raise ValueError("reduced-order control units must match the control schema")
        if not self.nonlinear_plant_id.strip() or not self.nonlinear_plant_revision.strip() or not self.claim_boundary.strip():
            raise ValueError("reduced-order control plant requires identity, revision, and claim boundary")
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        """Evaluate the exact declared reduced dynamics."""

        derivative = self.derivative_evaluator(state, effectors, environment)
        if set(derivative) != set(self.state_names):
            raise ValueError("reduced-order derivative must match the state schema")
        if not all(math.isfinite(float(value)) for value in derivative.values()):
            raise ValueError("reduced-order derivative must be finite")
        return {name: float(derivative[name]) for name in self.state_names}
        ####

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Return the family-provided lower-tier equilibrium record."""

        result = self.trim_provider(target, initial_guess)
        if tuple(result.spec.state_names) != self.state_names or tuple(result.spec.control_names) != self.control_names:
            raise ValueError("reduced-order trim schema does not match the adapter")
        return result
        ####

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        """Finite-difference the same lower-tier nonlinear derivative twice."""

        if tuple(trim.spec.state_names) != self.state_names or tuple(trim.spec.control_names) != self.control_names:
            raise ValueError("reduced-order linearization trim does not match the adapter")

        def evaluate(state: Mapping[str, float], controls: Mapping[str, float]) -> Mapping[str, float]:
            return self.state_derivative(state, controls, {})
            ####

        def option(name: str, default: float) -> float:
            return float(options.get(name, default))
            ####

        return finite_difference_linearization_with_provenance(
            trim.spec,
            evaluate,
            trim,
            nonlinear_plant_id=self.nonlinear_plant_id,
            nonlinear_plant_revision=self.nonlinear_plant_revision,
            state_step=option("state_step", 1.0e-4),
            control_step=option("control_step", 1.0e-4),
            comparison_factor=option("comparison_factor", 0.5),
            maximum_relative_difference=option("maximum_relative_difference", 0.25),
            comparison_absolute_floor=option("comparison_absolute_floor", 1.0e-8),
            state_units=self.state_units,
            control_units=self.control_units,
            metadata={
                "claim_boundary": self.claim_boundary,
                "control_realization": "reduced_force_or_response_law",
                **self.linearization_metadata,
            },
        )
        ####

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        """Reject physical-effector claims at this intentionally reduced tier."""

        del state, effectors
        raise NotImplementedError(f"{self.claim_boundary}: reduced tiers have no physical effector effectiveness")
        ####

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        """Reject physical allocation at this intentionally reduced tier."""

        del state, desired_wrench, previous_effectors, dt_s
        raise NotImplementedError(f"{self.claim_boundary}: reduced tiers have no physical effector allocator")
        ####
    ####


__all__ = ["ReducedOrderControlPlant", "declared_equilibrium_trim"]
