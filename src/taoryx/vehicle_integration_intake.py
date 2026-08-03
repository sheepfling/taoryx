"""Generate explicit intake blueprints for existing and new vehicle families.

The horizontal strategy catalog should make an ordinary vehicle addition a
data-and-adapter task, not an invitation to start a bespoke controller.  This
module turns that catalog into an actionable four-tier blueprint before any
family manifest or source mapping is written.

It deliberately does not infer a strategy when multiple topologies share a
physical-family label.  A rocket aircraft and an air-breathing fixed wing are
both powered fixed wing, but their phase boundaries and authority checks are
not interchangeable.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from .family_strategy import FamilyIntegrationStrategy, FamilyIntegrationStrategyCatalog, load_family_strategy_catalog
from .fidelity_contracts import CANONICAL_FIDELITY_TIERS, FidelityTier, control_realization_for

IntakeKind = Literal["existing_family", "new_topology"]


class StrategySelectionError(ValueError):
    """Raised when an intake cannot select one compatible strategy safely."""


@dataclass(frozen=True, slots=True)
class ExistingFamilyIntakeRequest:
    """The minimal identity needed to add a vehicle to an existing topology."""

    family_id: str
    physical_family: str
    mission_overlay: str
    adapter_id: str
    strategy_id: str | None = None

    def __post_init__(self) -> None:
        values = (self.family_id, self.physical_family, self.mission_overlay, self.adapter_id)
        if not all(value.strip() for value in values):
            raise ValueError("existing-family intake requires non-empty family, topology, mission, and adapter IDs")
        ####
    ####


@dataclass(frozen=True, slots=True)
class IntakeTierBlueprint:
    """One generated canonical fidelity slot and its required evidence path."""

    tier: FidelityTier
    control_realization: str
    data_requirements: tuple[str, ...]
    required_operations: tuple[str, ...]
    calibration_mode: str
    stages: tuple[str, ...]
    operating_point_campaign_required: bool
    initial_blockers: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a portable tier-level intake contract."""

        return {
            "tier": self.tier,
            "control_realization": self.control_realization,
            "data_requirements": list(self.data_requirements),
            "required_operations": list(self.required_operations),
            "calibration_mode": self.calibration_mode,
            "stages": list(self.stages),
            "operating_point_campaign_required": self.operating_point_campaign_required,
            "initial_blockers": list(self.initial_blockers),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class ExistingFamilyIntakeBlueprint:
    """Generated declaration and work package for an existing family strategy."""

    request: ExistingFamilyIntakeRequest
    strategy: FamilyIntegrationStrategy
    tiers: tuple[IntakeTierBlueprint, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a registry-entry template plus the unambiguous worklist."""

        registry_tiers = {
            tier.tier: {
                "profile_id": None,
                "promotion_status": "planned",
                "required_operations": list(tier.required_operations),
                "blockers": list(tier.initial_blockers),
            }
            for tier in self.tiers
        }
        return {
            "schema": "taoryx.vehicle-integration-intake/v1alpha1",
            "kind": "existing_family",
            "status": "ready_for_source_and_adapter_binding",
            "family_id": self.request.family_id,
            "physical_family": self.request.physical_family,
            "mission_overlay": self.request.mission_overlay,
            "strategy_id": self.strategy.id,
            "adapter_id": self.request.adapter_id,
            "family_inputs": list(self.strategy.family_inputs),
            "non_tunable_blockers": list(self.strategy.non_tunable_blockers),
            "mission_composition": _mission_composition_blueprint(self.strategy, self.request),
            "registry_entry_template": {
                "family_id": self.request.family_id,
                "physical_family": self.request.physical_family,
                "strategy_id": self.strategy.id,
                "mission_overlay": self.request.mission_overlay,
                "adapter_id": self.request.adapter_id,
                "automatic_lowering": True,
                "tiers": registry_tiers,
            },
            "tiers": [tier.as_dict() for tier in self.tiers],
            "claim_boundary": (
                "Intake blueprint only. Profile IDs, source evidence, plant operations, "
                "and qualification results remain undeclared until independently supplied."
            ),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class NewTopologyIntakeRequest:
    """The information known before a genuinely new physical family exists."""

    family_id: str
    physical_family: str
    proposed_strategy_id: str
    topology_summary: str

    def __post_init__(self) -> None:
        values = (self.family_id, self.physical_family, self.proposed_strategy_id, self.topology_summary)
        if not all(value.strip() for value in values):
            raise ValueError("new-topology intake requires non-empty identity and topology summary fields")
        ####
    ####


@dataclass(frozen=True, slots=True)
class NewTopologyIntakeScaffold:
    """The required decisions for a new topology without fabricated physics."""

    request: NewTopologyIntakeRequest

    def as_dict(self) -> dict[str, object]:
        """Return a strategy-authoring checklist and non-promotable template."""

        tiers = [
            {
                "tier": tier,
                "control_realization": control_realization_for(tier),
                "required_decisions": [
                    "applicability for this topology",
                    "minimum source/data contract",
                    "actual adapter operations and their evidence",
                    "calibration mode and ordered diagnostic stages",
                    "claim boundary and non-tunable topology blockers",
                ],
            }
            for tier in CANONICAL_FIDELITY_TIERS
        ]
        return {
            "schema": "taoryx.vehicle-integration-intake/v1alpha1",
            "kind": "new_topology",
            "status": "strategy_definition_required",
            "family_id": self.request.family_id,
            "physical_family": self.request.physical_family,
            "proposed_strategy_id": self.request.proposed_strategy_id,
            "topology_summary": self.request.topology_summary,
            "required_strategy_fields": [
                "summary",
                "family_inputs",
                "non_tunable_blockers",
                "exactly four canonical tier recipes",
            ],
            "tiers": tiers,
            "claim_boundary": (
                "A new topology scaffold is not a vehicle model. No data, operation, "
                "or control claim is inferred until the proposed strategy is authored and validated."
            ),
        }
        ####
    ####


def compatible_strategies(
    physical_family: str,
    *,
    catalog: FamilyIntegrationStrategyCatalog | None = None,
) -> tuple[FamilyIntegrationStrategy, ...]:
    """Return catalog strategies compatible with one physical-family label."""

    resolved_catalog = catalog or load_family_strategy_catalog()
    return tuple(
        strategy
        for strategy in resolved_catalog.strategies
        if physical_family in strategy.physical_families
    )
    ####


def select_existing_family_strategy(
    request: ExistingFamilyIntakeRequest,
    *,
    catalog: FamilyIntegrationStrategyCatalog | None = None,
) -> FamilyIntegrationStrategy:
    """Select a compatible strategy, refusing ambiguous topology inference."""

    resolved_catalog = catalog or load_family_strategy_catalog()
    compatible = compatible_strategies(request.physical_family, catalog=resolved_catalog)
    if request.strategy_id is not None:
        strategy = resolved_catalog.strategy(request.strategy_id)
        if request.physical_family not in strategy.physical_families:
            raise StrategySelectionError(
                f"strategy {strategy.id!r} does not support physical family {request.physical_family!r}"
            )
        return strategy
    if not compatible:
        raise StrategySelectionError(
            f"no registered strategy supports physical family {request.physical_family!r}; create a new topology strategy"
        )
    if len(compatible) != 1:
        candidates = ", ".join(strategy.id for strategy in compatible)
        raise StrategySelectionError(
            f"physical family {request.physical_family!r} is ambiguous across strategies: {candidates}; select strategy_id explicitly"
        )
    return compatible[0]
    ####


def build_existing_family_intake_blueprint(
    request: ExistingFamilyIntakeRequest,
    *,
    catalog: FamilyIntegrationStrategyCatalog | None = None,
) -> ExistingFamilyIntakeBlueprint:
    """Generate a complete four-tier intake work package for one new vehicle."""

    strategy = select_existing_family_strategy(request, catalog=catalog)
    tiers: list[IntakeTierBlueprint] = []
    for tier in CANONICAL_FIDELITY_TIERS:
        recipe = strategy.tier_recipes[tier]
        initial_blockers = (
            "source_lock_and_frame_contract_missing",
            "profile_and_adapter_binding_missing",
            *strategy.non_tunable_blockers,
        ) if recipe.applicable else ("not_applicable_by_declared_topology",)
        tiers.append(
            IntakeTierBlueprint(
                tier,
                control_realization_for(tier),
                recipe.data,
                tuple(recipe.operations),
                recipe.calibration_mode,
                recipe.stages,
                "operating_point_campaign" in recipe.stages,
                initial_blockers,
            )
        )
    return ExistingFamilyIntakeBlueprint(request, strategy, tuple(tiers))
    ####


def build_new_topology_intake_scaffold(request: NewTopologyIntakeRequest) -> NewTopologyIntakeScaffold:
    """Return the explicit authoring checklist for a new physical topology."""

    return NewTopologyIntakeScaffold(request)
    ####


def _mission_composition_blueprint(
    strategy: FamilyIntegrationStrategy,
    request: ExistingFamilyIntakeRequest,
) -> dict[str, object]:
    """Return mission-layer inputs without pretending they are vehicle data."""

    if strategy.id == "powered_fixed_wing.v1" and request.mission_overlay == "fixed_wing_racetrack":
        return {
            "template_id": "powered_fixed_wing_racetrack_v1",
            "capability_profile_required": [
                "operating_point_id",
                "minimum_speed_m_s",
                "nominal_speed_m_s",
                "maximum_speed_m_s",
                "maximum_bank_deg",
                "maximum_climb_rate_m_s",
                "maximum_descent_rate_m_s",
                "provenance",
                "evidence_class",
            ],
            "semantic_intent_required": [
                "low_altitude_m",
                "high_altitude_m",
                "requested_speed_m_s",
                "requested_bank_deg",
                "minimum_straight_length_m",
                "level_dwell_s",
            ],
            "compiler": "taoryx.powered_fixed_wing_mission_compiler.compile_powered_fixed_wing_racetrack",
            "expected_artifacts": [
                "capability_scaled_route_manifest",
                "baseline_comparison",
                "truth_objective_report",
            ],
            "claim_boundary": (
                "Compiled route geometry is a planning/preflight result. It does not prove "
                "controller tracking, nonlinear response, or physical effector allocation."
            ),
        }
    return {
        "template_id": request.mission_overlay,
        "status": "family_specific_mission_compiler_required",
        "claim_boundary": "No mission geometry or controller requirement is inferred for this overlay.",
    }
    ####


__all__ = [
    "ExistingFamilyIntakeBlueprint",
    "ExistingFamilyIntakeRequest",
    "IntakeKind",
    "IntakeTierBlueprint",
    "NewTopologyIntakeRequest",
    "NewTopologyIntakeScaffold",
    "StrategySelectionError",
    "build_existing_family_intake_blueprint",
    "build_new_topology_intake_scaffold",
    "compatible_strategies",
    "select_existing_family_strategy",
]
