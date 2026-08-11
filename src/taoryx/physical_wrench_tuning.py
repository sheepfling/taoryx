"""Common campaign adapter for an already-derived physical-wrench projection.

The local physical controller screens own their nonlinear plant, trim, and
allocator.  Their reusable auto-tuning path must use the *same* local
state-to-wrench coordinates, rather than separately tuning a broader plant
in effector coordinates and hoping a later runtime can reinterpret its gain.

This adapter is deliberately limited to a frozen, source-derived local
projection.  It is not a substitute for the nonlinear screen or the physical
allocator; it only exposes the exact trim/linearize contract needed by the
common campaign runner.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, replace

import numpy as np

from .control_allocation import DerivativeProvenance, EffectorEffectiveness, PhysicalAllocationStep, ProvenancedLinearization
from .family_adapter import AdapterCapability, AdapterOperation, StandardFamilyAdapter, descriptor_from_control_plant
from .physical_lqr import PhysicalWrenchLqiDesign, PhysicalWrenchLqrDesign, WrenchLinearizationProjection
from .trim import DynamicsLinearization, TrimResult, TrimSpec


@dataclass(frozen=True, slots=True)
class ProjectedPhysicalWrenchTuningPlant:
    """Expose one source-derived local wrench projection to the campaign host.

    The small-signal derivative is available only around the retained source
    trim.  It exists to keep automatic tuning coordinates identical to the
    allocator-backed controller runtime, not to promote a linear model to a
    stand-alone vehicle simulator.
    """

    projection: WrenchLinearizationProjection

    @property
    def state_names(self) -> tuple[str, ...]:
        """Return the ordered local feedback coordinates."""

        return self.projection.state_names
        ####

    @property
    def control_names(self) -> tuple[str, ...]:
        """Return the ordered desired-wrench coordinates."""

        return self.projection.wrench_names
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> dict[str, float]:
        """Evaluate the frozen local derivative without inventing disturbances."""

        if environment:
            raise ValueError("projected physical-wrench tuning derivative does not accept environment overrides")
        self._require_coordinates(state, controls)
        state_delta = np.asarray(
            [float(state[name]) - float(self.projection.trim_state[name]) for name in self.state_names],
            dtype=float,
        )
        control_delta = np.asarray(
            [float(controls[name]) - float(self.projection.nominal_wrench[name]) for name in self.control_names],
            dtype=float,
        )
        derivative = np.asarray(self.projection.a_matrix, dtype=float) @ state_delta + np.asarray(
            self.projection.b_matrix,
            dtype=float,
        ) @ control_delta
        return {name: float(value) for name, value in zip(self.state_names, derivative, strict=True)}
        ####

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Return the exact source trim that produced the retained projection."""

        if target or initial_guess:
            raise ValueError("projected physical-wrench tuning accepts no trim overrides at its frozen source point")
        spec = TrimSpec(
            state_names=self.state_names,
            control_names=self.control_names,
            residual_names=tuple(f"local_rate.{name}" for name in self.state_names),
            state_initial=dict(self.projection.trim_state),
            control_initial=dict(self.projection.nominal_wrench),
            residual_scales={f"local_rate.{name}": 1.0 for name in self.state_names},
            operating_point={"source": "retained_physical_wrench_projection"},
        )
        return TrimResult(
            spec=spec,
            state=dict(self.projection.trim_state),
            controls=dict(self.projection.nominal_wrench),
            residuals={name: 0.0 for name in spec.residual_names},
            scaled_residual_norm=0.0,
            success=True,
            status=1,
            message="retained source trim projected into the exact controller wrench coordinates",
            iterations=0,
            cost=0.0,
        )
        ####

    def linearize(
        self,
        trim: TrimResult,
        options: Mapping[str, float | str],
    ) -> ProvenancedLinearization:
        """Return the exact provenanced local state-to-wrench projection."""

        if not trim.success:
            raise ValueError("projected physical-wrench tuning requires a successful retained trim")
        if tuple(trim.spec.state_names) != self.state_names or tuple(trim.spec.control_names) != self.control_names:
            raise ValueError("projected physical-wrench tuning trim coordinates do not match its retained projection")
        if options:
            unknown = ", ".join(sorted(options))
            raise ValueError(f"projected physical-wrench tuning does not accept linearization options: {unknown}")
        source = self.projection.source_linearization.provenance
        metadata = {
            "method": "source-derived-physical-wrench-projection",
            "source_nonlinear_plant_id": source.nonlinear_plant_id,
            "source_nonlinear_plant_revision": source.nonlinear_plant_revision,
        }
        primary = DynamicsLinearization(
            self.state_names,
            self.control_names,
            np.asarray(self.projection.a_matrix, dtype=float),
            np.asarray(self.projection.b_matrix, dtype=float),
            dict(self.projection.trim_state),
            dict(self.projection.nominal_wrench),
            metadata,
        )
        provenance = DerivativeProvenance(
            nonlinear_plant_id=source.nonlinear_plant_id,
            nonlinear_plant_revision=source.nonlinear_plant_revision,
            state_names=self.state_names,
            control_names=self.control_names,
            method="source-derived-physical-wrench-projection",
            state_step=source.state_step,
            control_step=source.control_step,
            comparison_state_step=source.comparison_state_step,
            comparison_control_step=source.comparison_control_step,
            maximum_relative_difference=source.maximum_relative_difference,
            maximum_absolute_difference=source.maximum_absolute_difference,
            comparison_absolute_floor=source.comparison_absolute_floor,
            derivative_consistent=source.derivative_consistent,
        )
        return ProvenancedLinearization(primary=primary, comparison=primary, provenance=provenance)
        ####

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        """Reject allocation use: the nonlinear screen owns physical effectors."""

        raise RuntimeError("projected physical-wrench tuning does not expose physical-effector effectiveness")
        ####

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        """Reject allocation use: the nonlinear screen owns physical effectors."""

        raise RuntimeError("projected physical-wrench tuning does not expose physical-effector allocation")
        ####

    def _require_coordinates(self, state: Mapping[str, float], controls: Mapping[str, float]) -> None:
        missing_state = set(self.state_names) - set(state)
        missing_control = set(self.control_names) - set(controls)
        if missing_state or missing_control:
            details: list[str] = []
            if missing_state:
                details.append("state=" + ", ".join(sorted(missing_state)))
            if missing_control:
                details.append("controls=" + ", ".join(sorted(missing_control)))
            raise KeyError("projected physical-wrench tuning derivative is missing " + "; ".join(details))
        ####

    ####


@dataclass(frozen=True, slots=True)
class ScheduledProjectedPhysicalWrenchTuningPlant:
    """Expose several exact frozen projections through explicit trim targets.

    Each retained operating point remains a separate source-derived local
    model.  ``trim_target`` is solely an explicit node selector: it does not
    interpolate projections or manufacture a continuous schedule.  This
    lets the common campaign produce a candidate for every node that a
    discrete runtime will actually execute.
    """

    projections: Mapping[str, WrenchLinearizationProjection]
    node_trim_targets: Mapping[str, Mapping[str, float]]

    def __post_init__(self) -> None:
        if not self.projections:
            raise ValueError("scheduled physical-wrench tuning requires at least one projection")
        if set(self.projections) != set(self.node_trim_targets):
            raise ValueError("scheduled physical-wrench tuning projections and trim targets must name the same nodes")
        reference = next(iter(self.projections.values()))
        for node_id, projection in self.projections.items():
            if not node_id.strip():
                raise ValueError("scheduled physical-wrench tuning node IDs must not be blank")
            if projection.state_names != reference.state_names or projection.wrench_names != reference.wrench_names:
                raise ValueError("scheduled physical-wrench tuning nodes must share ordered state and wrench coordinates")
            target = self.node_trim_targets[node_id]
            if not target or any(not name.strip() for name in target):
                raise ValueError("scheduled physical-wrench tuning node selectors must be non-empty")
        ####

    @property
    def state_names(self) -> tuple[str, ...]:
        """Return the common ordered local feedback coordinates."""

        return next(iter(self.projections.values())).state_names
        ####

    @property
    def control_names(self) -> tuple[str, ...]:
        """Return the common ordered desired-wrench coordinates."""

        return next(iter(self.projections.values())).wrench_names
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> dict[str, float]:
        """Evaluate one explicitly named frozen node without interpolation."""

        node_id = environment.get("schedule_node_id")
        if not isinstance(node_id, str) or node_id not in self.projections or set(environment) != {"schedule_node_id"}:
            raise ValueError(
                "scheduled physical-wrench tuning derivatives require exactly one declared schedule_node_id environment"
            )
        return ProjectedPhysicalWrenchTuningPlant(self.projections[node_id]).state_derivative(state, controls, {})
        ####

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        """Select and return the retained trim for exactly one declared node."""

        if initial_guess:
            raise ValueError("scheduled physical-wrench tuning does not accept trim initial-guess overrides")
        node_id = self._node_id_for_target(target)
        projection = self.projections[node_id]
        retained = ProjectedPhysicalWrenchTuningPlant(projection).trim({}, {})
        return replace(
            retained,
            spec=replace(
                retained.spec,
                operating_point={
                    "source": "retained_scheduled_physical_wrench_projection",
                    "schedule_node_id": node_id,
                    **dict(target),
                },
            ),
        )
        ####

    def linearize(
        self,
        trim: TrimResult,
        options: Mapping[str, float | str],
    ) -> ProvenancedLinearization:
        """Return the source-derived projection selected by the retained trim."""

        node_id = trim.spec.operating_point.get("schedule_node_id")
        if not isinstance(node_id, str) or node_id not in self.projections:
            raise ValueError("scheduled physical-wrench tuning trim has no recognized schedule node identity")
        return ProjectedPhysicalWrenchTuningPlant(self.projections[node_id]).linearize(trim, options)
        ####

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        """Reject allocation use: the nonlinear screen owns physical effectors."""

        raise RuntimeError("scheduled physical-wrench tuning does not expose physical-effector effectiveness")
        ####

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        """Reject allocation use: the nonlinear screen owns physical effectors."""

        raise RuntimeError("scheduled physical-wrench tuning does not expose physical-effector allocation")
        ####

    def _node_id_for_target(self, target: Mapping[str, float]) -> str:
        """Resolve a campaign trim target only when it names one exact node."""

        normalized = {name: float(value) for name, value in target.items()}
        matches = [
            node_id
            for node_id, expected in self.node_trim_targets.items()
            if normalized == {name: float(value) for name, value in expected.items()}
        ]
        if len(matches) != 1:
            raise ValueError(
                "scheduled physical-wrench tuning trim target must select exactly one retained node; "
                f"got {dict(target)!r}"
            )
        return matches[0]
        ####

    ####


def build_projected_physical_wrench_tuning_adapter(
    design: PhysicalWrenchLqrDesign | PhysicalWrenchLqiDesign,
    *,
    family_id: str,
    adapter_id: str,
    physical_family: str,
    tier: str,
    state_units: Mapping[str, str],
    evidence_status: str = "development",
    validity_envelope: str,
    omitted_physics: tuple[str, ...],
) -> StandardFamilyAdapter:
    """Build the common campaign adapter for one exact physical runtime design.

    The campaign intentionally has no effectivity or allocation operation: it
    tunes in desired-wrench coordinates while the owning nonlinear runtime
    still sends each command through its actual bounded allocator.
    """

    plant = ProjectedPhysicalWrenchTuningPlant(design.projection)
    descriptor = descriptor_from_control_plant(
        plant,
        family_id=family_id,
        adapter_id=adapter_id,
        physical_family=physical_family,
        tier=tier,  # type: ignore[arg-type]
        state_units=state_units,
        control_units={name: ("N" if name.startswith("force_") else "N*m") for name in plant.control_names},
        evidence_status=evidence_status,
        validity_envelope=validity_envelope,
        omitted_physics=omitted_physics,
    )
    available = "source-derived local wrench projection retained from the physical controller screen"
    capabilities: dict[AdapterOperation, AdapterCapability] = {
        "state_derivative": AdapterCapability("state_derivative", "available", available),
        "trim": AdapterCapability("trim", "available", available),
        "linearize": AdapterCapability("linearize", "available", available),
        "effectiveness": AdapterCapability(
            "effectiveness",
            "not_applicable",
            "the campaign acts in desired-wrench coordinates; physical effectiveness remains owned by the runtime",
        ),
        "allocate": AdapterCapability(
            "allocate",
            "not_applicable",
            "the campaign acts in desired-wrench coordinates; physical allocation remains owned by the runtime",
        ),
    }
    return StandardFamilyAdapter(descriptor=descriptor, plant=plant, declared_capabilities=capabilities)
    ####


def build_scheduled_projected_physical_wrench_tuning_adapter(
    designs: Mapping[str, PhysicalWrenchLqrDesign | PhysicalWrenchLqiDesign],
    *,
    node_trim_targets: Mapping[str, Mapping[str, float]],
    family_id: str,
    adapter_id: str,
    physical_family: str,
    tier: str,
    state_units: Mapping[str, str],
    evidence_status: str = "development",
    validity_envelope: str,
    omitted_physics: tuple[str, ...],
) -> StandardFamilyAdapter:
    """Build a discrete multi-node adapter for an exact physical-wrench schedule.

    The adapter only lets the campaign select declared source nodes.  It does
    not interpolate gains, states, or derivatives; a family runtime remains
    responsible for applying every selected candidate to its actual bounded
    nonlinear allocator path.
    """

    if not designs:
        raise ValueError("scheduled physical-wrench tuning adapters require at least one design")
    plant = ScheduledProjectedPhysicalWrenchTuningPlant(
        {node_id: design.projection for node_id, design in designs.items()},
        node_trim_targets,
    )
    descriptor = descriptor_from_control_plant(
        plant,
        family_id=family_id,
        adapter_id=adapter_id,
        physical_family=physical_family,
        tier=tier,  # type: ignore[arg-type]
        state_units=state_units,
        control_units={name: ("N" if name.startswith("force_") else "N*m") for name in plant.control_names},
        evidence_status=evidence_status,
        validity_envelope=validity_envelope,
        omitted_physics=omitted_physics,
    )
    available = "explicit retained source-node physical-wrench projection for a discrete schedule campaign"
    capabilities: dict[AdapterOperation, AdapterCapability] = {
        "state_derivative": AdapterCapability("state_derivative", "available", available),
        "trim": AdapterCapability("trim", "available", available),
        "linearize": AdapterCapability("linearize", "available", available),
        "effectiveness": AdapterCapability(
            "effectiveness",
            "not_applicable",
            "the campaign acts in desired-wrench coordinates; physical effectiveness remains owned by the runtime",
        ),
        "allocate": AdapterCapability(
            "allocate",
            "not_applicable",
            "the campaign acts in desired-wrench coordinates; physical allocation remains owned by the runtime",
        ),
    }
    return StandardFamilyAdapter(descriptor=descriptor, plant=plant, declared_capabilities=capabilities)
    ####


__all__ = [
    "ProjectedPhysicalWrenchTuningPlant",
    "ScheduledProjectedPhysicalWrenchTuningPlant",
    "build_projected_physical_wrench_tuning_adapter",
    "build_scheduled_projected_physical_wrench_tuning_adapter",
]
