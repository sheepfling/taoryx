"""Compile generic integration and tuning worklists from family topology.

The fidelity registry says which tiers a vehicle advertises; readiness says
whether its declared data and adapter operations are presently usable.  This
module adds the missing process layer.  A physical-family strategy declares
the prerequisite data and fixed diagnostic order for each canonical tier, so
an integration cannot jump from an incomplete source import directly into a
multi-day gain search.

Strategies are deliberately not controllers and do not qualify a vehicle.
They produce the next admissible diagnostic or calibration action while
preserving declared evidence blockers and non-applicable topologies.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .family_adapter import AdapterOperation
from .fidelity_contracts import CANONICAL_FIDELITY_TIERS, FidelityTier
from .horizontal_fidelity import HorizontalFidelityRegistry, load_horizontal_registry
from .horizontal_readiness import HorizontalReadinessReport, HorizontalTierReadiness, build_horizontal_readiness_report
from .vehicle_execution_bindings import (
    ExecutionMode,
    VehicleExecutionBindingCatalog,
    load_vehicle_execution_binding_catalog,
)
from .vehicle_registry import ROOT

FAMILY_STRATEGY_CATALOG = ROOT / "verification/family_integration_strategies.yaml"
StrategyWorkStatus = Literal[
    "not_applicable",
    "planned",
    "blocked",
    "waiting_for_adapter_operation",
    "strategy_development",
    "strategy_probe_ready",
]

# These modes establish only a bounded source/release execution witness.  They
# intentionally do not include a closed-loop controller or a local
# direct-wrench screen: those paths must still meet their adapter prerequisites
# before they can start a generic strategy campaign.
_REPLAY_OR_RELEASE_BATCH_EXECUTION_MODES: frozenset[ExecutionMode] = frozenset(
    {
        "source_history_replay",
        "source_scheduled_replay",
        "open_loop_witness",
        "passive_uncontrolled",
    }
)


class FamilyTierStrategy(BaseModel):
    """Reusable data and execution recipe for one canonical fidelity tier."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    applicable: bool = True
    data: tuple[str, ...] = ()
    operations: tuple[AdapterOperation, ...] = ()
    calibration_mode: str = Field(min_length=1)
    stages: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_contract(self) -> FamilyTierStrategy:
        if len(self.data) != len(set(self.data)):
            raise ValueError("tier strategy data requirements must be unique")
        if len(self.operations) != len(set(self.operations)):
            raise ValueError("tier strategy operations must be unique")
        if len(self.stages) != len(set(self.stages)):
            raise ValueError("tier strategy stages must be unique")
        if self.applicable and not self.stages:
            raise ValueError("an applicable tier strategy requires at least one stage")
        if not self.applicable and (self.data or self.operations or self.stages):
            raise ValueError("a non-applicable tier strategy cannot declare data, operations, or stages")
        if "linearize" in self.operations and "authority_preflight" not in self.stages:
            raise ValueError(
                "a tier strategy with plant linearization must run authority_preflight before controller work"
            )
        if "lqr" in self.calibration_mode and "operating_point_campaign" not in self.stages:
            raise ValueError(
                "an LQR calibration strategy must run an operating_point_campaign instead of a manual gain-search loop"
            )
        if "operating_point_campaign" in self.stages and "authority_preflight" in self.stages:
            if self.stages.index("authority_preflight") > self.stages.index("operating_point_campaign"):
                raise ValueError("authority_preflight must precede the operating_point_campaign")
        return self
        ####
    ####


class FamilyIntegrationStrategy(BaseModel):
    """A topology-specific process that remains reusable across vehicles."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    physical_families: tuple[str, ...] = Field(min_length=1)
    summary: str = Field(min_length=1)
    family_inputs: tuple[str, ...] = Field(min_length=1)
    non_tunable_blockers: tuple[str, ...] = ()
    tier_recipes: dict[FidelityTier, FamilyTierStrategy]

    @model_validator(mode="after")
    def validate_tier_recipes(self) -> FamilyIntegrationStrategy:
        if len(self.physical_families) != len(set(self.physical_families)):
            raise ValueError("strategy physical families must be unique")
        if len(self.family_inputs) != len(set(self.family_inputs)):
            raise ValueError("strategy family inputs must be unique")
        if len(self.non_tunable_blockers) != len(set(self.non_tunable_blockers)):
            raise ValueError("strategy non-tunable blockers must be unique")
        if set(self.tier_recipes) != set(CANONICAL_FIDELITY_TIERS):
            raise ValueError("strategy must declare exactly the canonical fidelity tiers")
        return self
        ####
    ####


class FamilyIntegrationStrategyCatalog(BaseModel):
    """Versioned catalog of generic family integration strategies."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(alias="schema", min_length=1)
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    strategies: tuple[FamilyIntegrationStrategy, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_ids(self) -> FamilyIntegrationStrategyCatalog:
        identifiers = tuple(item.id for item in self.strategies)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("family integration strategy IDs must be unique")
        return self
        ####

    def strategy(self, strategy_id: str) -> FamilyIntegrationStrategy:
        """Return one strategy by stable identifier."""

        for strategy in self.strategies:
            if strategy.id == strategy_id:
                return strategy
        raise KeyError(f"unknown family integration strategy {strategy_id!r}")
        ####
    ####


@dataclass(frozen=True, slots=True)
class FamilyStrategyFinding:
    """One fail-closed catalog-to-family mapping finding."""

    severity: Literal["error", "warning"]
    family_id: str
    code: str
    message: str

    def as_dict(self) -> dict[str, str]:
        """Return a stable machine-readable finding."""

        return {
            "severity": self.severity,
            "family_id": self.family_id,
            "code": self.code,
            "message": self.message,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class FamilyStrategyConformanceReport:
    """Result of binding every advertised family to one valid strategy."""

    strategy_catalog: str
    family_count: int
    findings: tuple[FamilyStrategyFinding, ...]

    @property
    def errors(self) -> tuple[FamilyStrategyFinding, ...]:
        """Return blocking strategy mapping errors."""

        return tuple(item for item in self.findings if item.severity == "error")
        ####

    @property
    def status(self) -> str:
        """Return pass only when every family has a compatible strategy."""

        return "pass" if not self.errors else "fail"
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the validated strategy mapping."""

        return {
            "schema": "taoryx.family-strategy-conformance/v1alpha1",
            "status": self.status,
            "strategy_catalog": self.strategy_catalog,
            "family_count": self.family_count,
            "findings": [item.as_dict() for item in self.findings],
            "error_count": len(self.errors),
            "claim_boundary": (
                "Strategy conformance only; a valid strategy mapping is not "
                "vehicle, controller, or mission qualification."
            ),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class FamilyStrategyWorkItem:
    """The next structured integration action for one family/tier slot."""

    family_id: str
    physical_family: str
    mission_overlay: str
    strategy_id: str
    tier: FidelityTier
    status: StrategyWorkStatus
    readiness_status: str
    calibration_mode: str
    family_inputs: tuple[str, ...]
    data_requirements: tuple[str, ...]
    required_operations: tuple[str, ...]
    pending_operations: tuple[str, ...]
    runnable_batch_execution_modes: tuple[ExecutionMode, ...]
    runnable_batch_missions: tuple[str, ...]
    runnable_batch_claim_boundaries: tuple[str, ...]
    stages: tuple[str, ...]
    next_action: str
    declared_blockers: tuple[str, ...]
    readiness_blockers: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        """Return a stable worklist row without promoting any evidence."""

        return {
            "family_id": self.family_id,
            "physical_family": self.physical_family,
            "mission_overlay": self.mission_overlay,
            "strategy_id": self.strategy_id,
            "tier": self.tier,
            "status": self.status,
            "readiness_status": self.readiness_status,
            "calibration_mode": self.calibration_mode,
            "family_inputs": list(self.family_inputs),
            "data_requirements": list(self.data_requirements),
            "required_operations": list(self.required_operations),
            "pending_operations": list(self.pending_operations),
            "runnable_batch_execution_modes": list(self.runnable_batch_execution_modes),
            "runnable_batch_missions": list(self.runnable_batch_missions),
            "runnable_batch_claim_boundaries": list(self.runnable_batch_claim_boundaries),
            "stages": list(self.stages),
            "next_action": self.next_action,
            "declared_blockers": list(self.declared_blockers),
            "readiness_blockers": list(self.readiness_blockers),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class FamilyStrategyWorklistReport:
    """All family/tier worklist rows plus strategy-mapping diagnostics."""

    conformance: FamilyStrategyConformanceReport
    readiness_status: str
    items: tuple[FamilyStrategyWorkItem, ...]

    def as_dict(self) -> dict[str, object]:
        """Return the action-oriented integration planning artifact."""

        counts: dict[str, int] = {}
        for item in self.items:
            counts[item.status] = counts.get(item.status, 0) + 1
        return {
            "schema": "taoryx.family-strategy-worklist/v1alpha1",
            "status": self.conformance.status,
            "strategy_conformance": self.conformance.as_dict(),
            "readiness_status": self.readiness_status,
            "family_count": self.conformance.family_count,
            "tier_count": len(self.items),
            "status_counts": counts,
            "items": [item.as_dict() for item in self.items],
            "claim_boundary": (
                "A worklist determines admissible diagnostic and calibration work. "
                "A declared replay/release batch witness may admit its first source "
                "or release audit while pending generic adapter operations remain "
                "visible; it is neither controller tuning evidence nor a promotion claim."
            ),
        }
        ####
    ####


def load_family_strategy_catalog(
    path: str | Path | None = None,
) -> FamilyIntegrationStrategyCatalog:
    """Load the versioned generic family-strategy catalog."""

    catalog_path = Path(path) if path is not None else FAMILY_STRATEGY_CATALOG
    payload = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{catalog_path} must contain a mapping")
    return FamilyIntegrationStrategyCatalog.model_validate(payload)
    ####


def validate_family_strategy_catalog(
    catalog: FamilyIntegrationStrategyCatalog | None = None,
    *,
    registry: HorizontalFidelityRegistry | None = None,
) -> FamilyStrategyConformanceReport:
    """Fail closed when a family lacks a compatible topology strategy."""

    resolved_catalog = catalog or load_family_strategy_catalog()
    resolved_registry = registry or load_horizontal_registry()
    findings: list[FamilyStrategyFinding] = []
    strategy_by_id = {item.id: item for item in resolved_catalog.strategies}
    for family in resolved_registry.families:
        strategy = strategy_by_id.get(family.strategy_id)
        if strategy is None:
            findings.append(
                FamilyStrategyFinding(
                    "error",
                    family.family_id,
                    "strategy-missing",
                    f"strategy {family.strategy_id!r} is not in {FAMILY_STRATEGY_CATALOG.relative_to(ROOT)}",
                )
            )
            continue
        if family.physical_family not in strategy.physical_families:
            findings.append(
                FamilyStrategyFinding(
                    "error",
                    family.family_id,
                    "physical-family-incompatible",
                    f"strategy {strategy.id!r} permits {strategy.physical_families!r}, not {family.physical_family!r}",
                )
            )
    return FamilyStrategyConformanceReport(
        str(FAMILY_STRATEGY_CATALOG.relative_to(ROOT)),
        len(resolved_registry.families),
        tuple(findings),
    )
    ####


def _ordered_union(*groups: tuple[str, ...]) -> tuple[str, ...]:
    """Return unique values in declaration order."""

    return tuple(dict.fromkeys(value for group in groups for value in group))
    ####


def _pending_operations(
    readiness: HorizontalTierReadiness,
    required_operations: tuple[str, ...],
) -> tuple[str, ...]:
    """Identify required adapter operations not verified by the probe artifact."""

    return tuple(
        operation
        for operation in required_operations
        if readiness.operation_status.get(operation) not in {"pass", "available"}
    )
    ####


def _work_status(
    recipe: FamilyTierStrategy,
    readiness: HorizontalTierReadiness,
    pending_operations: tuple[str, ...],
    *,
    runnable_replay_or_release_batch: bool,
) -> StrategyWorkStatus:
    """Classify whether the next strategy stage may be attempted."""

    if not recipe.applicable or readiness.status == "not_applicable":
        return "not_applicable"
    if readiness.declared_status == "planned" or readiness.status == "planned":
        return "planned"
    if readiness.status in {"blocked", "not_registered"}:
        return "blocked"
    if pending_operations and not runnable_replay_or_release_batch:
        return "waiting_for_adapter_operation"
    if readiness.status == "probe_ready":
        return "strategy_probe_ready"
    return "strategy_development"
    ####


def _runnable_replay_or_release_batch_evidence(
    family_id: str,
    tier: FidelityTier,
    catalog: VehicleExecutionBindingCatalog,
) -> tuple[tuple[ExecutionMode, ...], tuple[str, ...], tuple[str, ...]]:
    """Return declared batch witness details without treating them as an adapter probe.

    A source-history, source-scheduled, open-loop, or passive batch can start
    the first strategy audit even when a generic derivative/trim adapter has
    not been implemented.  It remains bounded execution evidence: callers
    must retain ``pending_operations`` and cannot infer a plant, trim, or
    controller result from this helper.
    """

    matches = tuple(
        binding
        for binding in catalog.bindings
        if (
            binding.family_id == family_id
            and binding.fidelity == tier
            and binding.operation == "batch"
            and binding.status == "runnable"
            and binding.execution_mode in _REPLAY_OR_RELEASE_BATCH_EXECUTION_MODES
        )
    )
    return (
        tuple(dict.fromkeys(binding.execution_mode for binding in matches)),
        tuple(dict.fromkeys(binding.mission for binding in matches)),
        tuple(dict.fromkeys(binding.claim_boundary for binding in matches)),
    )
    ####


def _next_action(
    status: StrategyWorkStatus,
    recipe: FamilyTierStrategy,
    readiness: HorizontalTierReadiness,
    pending_operations: tuple[str, ...],
) -> str:
    """Select an honest next action; never substitute gain tuning for a blocker."""

    if status == "not_applicable":
        return "no_control_tuning_required"
    if status == "waiting_for_adapter_operation":
        return f"validate_adapter_operation:{pending_operations[0]}"
    if status in {"planned", "blocked"}:
        blocker = readiness.blockers[0] if readiness.blockers else "declared_tier_prerequisite"
        return f"resolve_declared_blocker:{blocker}"
    if recipe.stages:
        return f"run_strategy_stage:{recipe.stages[0]}"
    return "inspect_strategy_contract"
    ####


def build_family_strategy_worklist(
    *,
    catalog: FamilyIntegrationStrategyCatalog | None = None,
    registry: HorizontalFidelityRegistry | None = None,
    readiness: HorizontalReadinessReport | None = None,
    execution_catalog: VehicleExecutionBindingCatalog | None = None,
) -> FamilyStrategyWorklistReport:
    """Join strategy, registry, and evidence status into the 36-slot worklist."""

    resolved_catalog = catalog or load_family_strategy_catalog()
    resolved_registry = registry or load_horizontal_registry()
    conformance = validate_family_strategy_catalog(resolved_catalog, registry=resolved_registry)
    readiness_report = readiness or build_horizontal_readiness_report()
    resolved_execution_catalog = execution_catalog or load_vehicle_execution_binding_catalog()
    readiness_by_key = {(item.family_id, item.tier): item for item in readiness_report.records}
    strategy_by_id = {item.id: item for item in resolved_catalog.strategies}
    items: list[FamilyStrategyWorkItem] = []
    for family in resolved_registry.families:
        strategy = strategy_by_id.get(family.strategy_id)
        if strategy is None:
            continue
        for tier in CANONICAL_FIDELITY_TIERS:
            readiness_item = readiness_by_key.get((family.family_id, tier))
            if readiness_item is None:
                continue
            recipe = strategy.tier_recipes[tier]
            required_operations = _ordered_union(
                tuple(recipe.operations),
                tuple(readiness_item.required_operations),
            )
            pending = _pending_operations(readiness_item, required_operations)
            execution_modes, execution_missions, execution_claim_boundaries = _runnable_replay_or_release_batch_evidence(
                family.family_id,
                tier,
                resolved_execution_catalog,
            )
            status = _work_status(
                recipe,
                readiness_item,
                pending,
                runnable_replay_or_release_batch=bool(execution_modes),
            )
            items.append(
                FamilyStrategyWorkItem(
                    family.family_id,
                    family.physical_family,
                    family.mission_overlay,
                    strategy.id,
                    tier,
                    status,
                    readiness_item.status,
                    recipe.calibration_mode,
                    tuple(strategy.family_inputs),
                    tuple(recipe.data),
                    required_operations,
                    pending,
                    execution_modes,
                    execution_missions,
                    execution_claim_boundaries,
                    tuple(recipe.stages),
                    _next_action(status, recipe, readiness_item, pending),
                    tuple(family.tiers[tier].blockers),
                    tuple(readiness_item.blockers),
                )
            )
    return FamilyStrategyWorklistReport(conformance, readiness_report.status, tuple(items))
    ####


__all__ = [
    "FAMILY_STRATEGY_CATALOG",
    "FamilyIntegrationStrategy",
    "FamilyIntegrationStrategyCatalog",
    "FamilyStrategyConformanceReport",
    "FamilyStrategyFinding",
    "FamilyStrategyWorkItem",
    "FamilyStrategyWorklistReport",
    "FamilyTierStrategy",
    "StrategyWorkStatus",
    "build_family_strategy_worklist",
    "load_family_strategy_catalog",
    "validate_family_strategy_catalog",
]
