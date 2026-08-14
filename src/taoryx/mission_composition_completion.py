"""Machine-readable Mission Composition completion matrix and physics backlog."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Literal, cast

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .mission_composition_inventory import (
    AssetDisposition,
    MissionCompositionInventory,
    _DiscoveryProvider,
    audit_mission_composition_inventory,
    load_mission_composition_inventory,
)
from .plugins import discover_plugins
from .trajectory.configuration_contract import (
    TrajectoryActuatorType,
    TrajectoryControlStatus,
    TrajectoryDynamicsFidelity,
    TrajectoryInputRealization,
    TrajectoryModelMetadata,
    TrajectoryRealizationMetadata,
)
from .trajectory.execution_contract import audit_provider_advertisement
from .trajectory.native_mission_composition import build_registry_mission_composition_runner
from .trajectory.registry_mission_composition import RegistryMissionCompositionProvider
from .vehicle_registry import ROOT

MISSION_COMPOSITION_PHYSICS_BACKLOG = ROOT / "verification/mission_composition_physics_backlog.yaml"
ExecutionDisposition = Literal["available", "blocked", "unsupported"]


class MissionCompositionBacklogItem(BaseModel):
    """One genuine post-integration model or physics dependency."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    families: tuple[str, ...] = Field(min_length=1)
    category: Literal[
        "native_model",
        "mission_physics",
        "control_allocation",
        "spawned_entity_physics",
        "segment_physics",
    ]
    description: str = Field(min_length=1)
    dependency: str = Field(min_length=1)


class MissionCompositionPhysicsBacklog(BaseModel):
    """Versioned boundary between integration completion and future physics."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-physics-backlog/v1"] = Field(
        alias="schema",
        serialization_alias="schema",
    )
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    items: tuple[MissionCompositionBacklogItem, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_items(self) -> MissionCompositionPhysicsBacklog:
        identifiers = tuple(item.id for item in self.items)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("Mission Composition physics backlog contains duplicate IDs")
        return self
        ####

    ####


class MissionCompositionRealizationCoverage(BaseModel):
    """One provider-neutral matrix row for an advertised realization."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    model_id: str
    family_id: str
    realization_id: str
    discovery_status: str
    inventory_status: AssetDisposition
    registry_status: Literal["available", "blocked", "unsupported"]
    supported_missions: tuple[str, ...]
    dynamics_fidelities: tuple[TrajectoryDynamicsFidelity, ...]
    control_realization: TrajectoryInputRealization
    actuator_types: tuple[TrajectoryActuatorType, ...]
    control_status: TrajectoryControlStatus
    control_channels: tuple[str, ...]
    control_authorities: tuple[str, ...]
    control_intents: tuple[str, ...]
    batch_status: ExecutionDisposition
    batch_tuples: tuple[str, ...]
    interactive_status: ExecutionDisposition
    interactive_tuples: tuple[str, ...]
    deployment_staging_capabilities: tuple[str, ...]
    optional_telemetry: tuple[str, ...]
    spawned_child_behavior: tuple[str, ...]
    blockers: tuple[str, ...]


class MissionCompositionFamilyCoverage(BaseModel):
    """Family-level identity and all selectable realization rows."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    model_id: str
    family_id: str
    name: str
    discovery_status: str
    inventory_status: AssetDisposition
    supported_missions: tuple[str, ...]
    capabilities: tuple[str, ...]
    optional_telemetry: tuple[str, ...]
    realizations: tuple[MissionCompositionRealizationCoverage, ...] = Field(min_length=1)


class MissionCompositionCompletionReport(BaseModel):
    """Serializable source-of-truth completion report for production discovery."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-completion/v1"] = Field(
        default="taoryx.mission-composition-completion/v1",
        alias="schema",
        serialization_alias="schema",
    )
    provider_id: str
    provider_version: str
    status: Literal["pass", "fail"]
    completion_rule: str
    inventory_status: Literal["pass", "fail"]
    advertisement_status: Literal["pass", "fail"]
    family_count: int = Field(ge=0)
    realization_count: int = Field(ge=0)
    registered_batch_tuple_count: int = Field(ge=0)
    registered_interactive_tuple_count: int = Field(ge=0)
    families: tuple[MissionCompositionFamilyCoverage, ...]
    future_physics_backlog: tuple[MissionCompositionBacklogItem, ...]
    diagnostics: tuple[str, ...] = ()


def load_mission_composition_physics_backlog(
    path: str | Path | None = None,
) -> MissionCompositionPhysicsBacklog:
    """Load the explicit post-integration physics/model-development boundary."""

    source = Path(path) if path is not None else MISSION_COMPOSITION_PHYSICS_BACKLOG
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source} must contain a mapping")
    return MissionCompositionPhysicsBacklog.model_validate(payload)
    ####


def build_mission_composition_completion_report(
    provider: RegistryMissionCompositionProvider | None = None,
    inventory: MissionCompositionInventory | None = None,
    backlog: MissionCompositionPhysicsBacklog | None = None,
) -> MissionCompositionCompletionReport:
    """Build the authoritative registry, execution, telemetry, and blocker matrix."""

    selected_provider = provider or RegistryMissionCompositionProvider()
    selected_inventory = inventory or load_mission_composition_inventory()
    selected_backlog = backlog or load_mission_composition_physics_backlog()
    debug_providers, debug_discovery_errors = _installed_debug_providers()
    inventory_audit = audit_mission_composition_inventory(
        selected_inventory,
        provider=selected_provider,
        debug_providers=debug_providers,
    )
    advertisement_audit = audit_provider_advertisement(
        selected_provider,
        build_registry_mission_composition_runner(selected_provider),
    )
    model_ids = {item.id for item in selected_provider.list_models()}
    unknown_backlog_families = sorted({family for item in selected_backlog.items for family in item.families if family not in model_ids})
    diagnostics = [*debug_discovery_errors, *inventory_audit.errors]
    diagnostics.extend(f"{item.code}: {item.message}" for item in advertisement_audit.diagnostics)
    if unknown_backlog_families:
        diagnostics.append(f"physics backlog references unknown production models: {unknown_backlog_families!r}")

    families = tuple(_family_coverage(model, selected_inventory) for model in selected_provider.list_models())
    registered_batch = sum(len(row.batch_tuples) for family in families for row in family.realizations if row.batch_status == "available")
    registered_interactive = sum(len(row.interactive_tuples) for family in families for row in family.realizations if row.interactive_status == "available")
    return MissionCompositionCompletionReport(
        provider_id=selected_provider.metadata.id,
        provider_version=selected_provider.metadata.version,
        status="pass" if not diagnostics else "fail",
        completion_rule=(
            "Every exact combination advertised as available executes through the common Mission Composition interface; "
            "every realization publishes explicit control channels, authority, and intent resolution; all other "
            "combinations are explicitly blocked or unsupported."
        ),
        inventory_status=inventory_audit.status,
        advertisement_status=advertisement_audit.status,
        family_count=len(families),
        realization_count=sum(len(item.realizations) for item in families),
        registered_batch_tuple_count=registered_batch,
        registered_interactive_tuple_count=registered_interactive,
        families=families,
        future_physics_backlog=selected_backlog.items,
        diagnostics=tuple(diagnostics),
    )
    ####


def _installed_debug_providers() -> tuple[tuple[_DiscoveryProvider, ...], tuple[str, ...]]:
    """Resolve optional debug providers through their own plug-in boundary.

    The production completion report may run with the reference compatibility
    package installed but the development-only debug package absent.  Direct
    imports would make that optional package a hidden runtime dependency.  A
    failed *installed* debug plug-in remains visible as an audit failure, while
    a genuinely absent debug package simply leaves the production audit scoped
    to production providers.
    """

    catalog = discover_plugins(strict=False)
    providers = tuple(
        cast(_DiscoveryProvider, contribution.value)
        for contribution in catalog.records("mission_composition_provider")
        if contribution.plugin.id == "taoryx.debug-models"
    )
    errors = tuple(
        f"debug-provider discovery {diagnostic.status}: {diagnostic.message}"
        for diagnostic in catalog.diagnostics
        if diagnostic.plugin_id == "taoryx.debug-models" and diagnostic.status not in {"disabled", "loaded"}
    )
    return providers, errors
    ####


def _family_coverage(
    model: TrajectoryModelMetadata,
    inventory: MissionCompositionInventory,
) -> MissionCompositionFamilyCoverage:
    family_id = model.family_id or model.id
    inventory_status = _inventory_status(inventory, model.id, None)
    telemetry = tuple(sorted(item.id for item in model.output_schema.telemetry_channels if item.operations))
    capability_flags = tuple(
        name
        for name, enabled in (
            ("custom_segments", model.capabilities.supports_custom_segments),
            ("deployment", model.capabilities.supports_deployment),
            ("staging", model.capabilities.supports_staging),
            ("dynamic_child_generation", model.capabilities.supports_dynamic_child_generation),
            ("multiple_stages", model.capabilities.supports_multiple_stages),
            ("submodels", model.capabilities.supports_submodels),
        )
        if enabled
    )
    return MissionCompositionFamilyCoverage(
        model_id=model.id,
        family_id=family_id,
        name=model.name,
        discovery_status=model.status,
        inventory_status=inventory_status,
        supported_missions=tuple(item.id for item in model.mission_templates) or ("provider_defined_custom_sequence",),
        capabilities=capability_flags,
        optional_telemetry=telemetry,
        realizations=tuple(_realization_coverage(model, realization, inventory_status, inventory) for realization in model.realizations),
    )
    ####


def _realization_coverage(
    model: TrajectoryModelMetadata,
    realization: TrajectoryRealizationMetadata,
    family_inventory_status: AssetDisposition,
    inventory: MissionCompositionInventory,
) -> MissionCompositionRealizationCoverage:
    operations = tuple(
        (mission.id, item)
        for mission in model.mission_templates
        for item in mission.operations
        if item.realization_id == realization.id and item.operation in {"batch", "step"}
    )
    batch_status, batch_tuples = _operation_coverage(realization, operations, "batch")
    interactive_status, interactive_tuples = _operation_coverage(realization, operations, "step")
    available_operations = {operation for operation, status in (("batch", batch_status), ("step", interactive_status)) if status == "available"}
    telemetry = tuple(
        sorted(
            channel.id
            for channel in model.output_schema.telemetry_channels
            if set(channel.operations) & available_operations
            and (not channel.compatible_realizations or realization.id in channel.compatible_realizations)
            and (not channel.compatible_fidelities or set(channel.compatible_fidelities) & set(realization.fidelity_aliases))
        )
    )
    deployments = tuple(
        item for item in model.deployments if not item.compatible_fidelities or set(item.compatible_fidelities) & set(realization.fidelity_aliases)
    )
    spawned = tuple(f"{item.status}:{item.id}:{item.lifecycle}:{item.child_model_id or item.child_model_kind}" for item in deployments) or ("none",)
    deployment_capabilities = tuple(
        item
        for item, enabled in (
            ("deployment", model.capabilities.supports_deployment),
            ("staging", model.capabilities.supports_staging),
            ("dynamic_child_generation", model.capabilities.supports_dynamic_child_generation),
            ("multiple_stages", model.capabilities.supports_multiple_stages),
            ("submodels", model.capabilities.supports_submodels),
        )
        if enabled
    ) or ("none",)
    operation_blockers = tuple(
        f"{mission_id}/{getattr(item, 'fidelity')}/{realization.id}/{getattr(item, 'operation')}: {blocker}"
        for mission_id, item in operations
        if item.status == "blocked"
        for blocker in item.blockers
    )
    deployment_blockers = tuple(f"deployment/{item.id}: {blocker}" for item in deployments for blocker in item.blockers)
    blockers = tuple(dict.fromkeys((*realization.blockers, *operation_blockers, *deployment_blockers)))
    return MissionCompositionRealizationCoverage(
        model_id=model.id,
        family_id=model.family_id or model.id,
        realization_id=realization.id,
        discovery_status=model.status,
        inventory_status=_inventory_status(inventory, model.id, realization.id, fallback=family_inventory_status),
        registry_status=realization.status,
        supported_missions=realization.mission_template_ids or ("provider_defined_custom_sequence",),
        dynamics_fidelities=realization.dynamics_fidelities,
        control_realization=realization.input_realization,
        actuator_types=realization.actuator_types,
        control_status=realization.controls.status,
        control_channels=tuple(
            f"{item.channel_kind}:{item.availability}:{item.id}->{item.native_channel_id or 'provider-internal'}" for item in realization.controls.channels
        )
        or ("none",),
        control_authorities=tuple(f"{item.availability}:{item.authority}:{item.id}" for item in realization.controls.authorities) or ("none",),
        control_intents=tuple(f"{item.resolution}:{item.id}" for item in realization.controls.intents) or ("none",),
        batch_status=batch_status,
        batch_tuples=batch_tuples,
        interactive_status=interactive_status,
        interactive_tuples=interactive_tuples,
        deployment_staging_capabilities=deployment_capabilities,
        optional_telemetry=telemetry,
        spawned_child_behavior=spawned,
        blockers=blockers,
    )
    ####


def _operation_coverage(
    realization: TrajectoryRealizationMetadata,
    operations: tuple[tuple[str, object], ...],
    operation: Literal["batch", "step"],
) -> tuple[ExecutionDisposition, tuple[str, ...]]:
    if realization.status == "unsupported":
        return "unsupported", ()
    exact = tuple((mission_id, item) for mission_id, item in operations if getattr(item, "operation") == operation)
    available = tuple(
        f"{mission_id}/{getattr(item, 'fidelity')}/{realization.id}/{operation}"
        for mission_id, item in exact
        if getattr(item, "status") == "available" and getattr(item, "common_runner_status") == "registered"
    )
    if available:
        return "available", available
    if not exact and not realization.mission_template_ids and operation in realization.operations:
        return "available", (f"provider_defined/{realization.id}/{operation}",)
    if realization.status == "blocked" or exact or operation in realization.operations:
        return "blocked", ()
    return "unsupported", ()
    ####


def _inventory_status(
    inventory: MissionCompositionInventory,
    model_id: str,
    realization_id: str | None,
    *,
    fallback: AssetDisposition | None = None,
) -> AssetDisposition:
    exact = next(
        (item.status for item in inventory.assets if item.published_model_id == model_id and item.realization_id == realization_id),
        None,
    )
    if exact is not None:
        return exact
    if fallback is not None:
        return fallback
    family = next(
        (item.status for item in inventory.assets if item.published_model_id == model_id and item.classification == "family"),
        None,
    )
    if family is None:
        raise ValueError(f"production model {model_id!r} has no inventory family disposition")
    return family
    ####


def render_mission_composition_completion_markdown(
    report: MissionCompositionCompletionReport,
) -> str:
    """Render the generated family/realization matrix and future-work boundary."""

    lines = [
        "# Mission Composition coverage matrix",
        "",
        "> Generated from the production provider advertisement, authoritative inventory, and common-executor audits. Do not hand-edit.",
        "",
        f"Status: **{report.status}**. Families: {report.family_count}. Realizations: {report.realization_count}. "
        f"Registered batch tuples: {report.registered_batch_tuple_count}. Registered interactive tuples: {report.registered_interactive_tuple_count}.",
        "",
        report.completion_rule,
        "",
        "| Family / realization | Registry | Missions | Dynamics | Input / actuators | Control advertisement | Batch | Interactive | Deployment / staging | Optional telemetry | Spawned children | Blockers |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for family in report.families:
        for row in family.realizations:
            lines.append(
                "| "
                + " | ".join(
                    (
                        _markdown_cell(f"{family.name} / `{row.realization_id}`"),
                        _markdown_cell(f"{row.inventory_status}; {row.registry_status}; {row.discovery_status}"),
                        _markdown_cell(", ".join(row.supported_missions)),
                        _markdown_cell(", ".join(row.dynamics_fidelities)),
                        _markdown_cell(f"{row.control_realization}; {', '.join(row.actuator_types)}"),
                        _markdown_cell(
                            f"{row.control_status}; channels={', '.join(row.control_channels)}; "
                            f"authorities={', '.join(row.control_authorities)}; intents={', '.join(row.control_intents)}"
                        ),
                        _markdown_cell(_operation_cell(row.batch_status, row.batch_tuples)),
                        _markdown_cell(_operation_cell(row.interactive_status, row.interactive_tuples)),
                        _markdown_cell(", ".join(row.deployment_staging_capabilities)),
                        _markdown_cell(", ".join(row.optional_telemetry) or "none"),
                        _markdown_cell(", ".join(row.spawned_child_behavior)),
                        _markdown_cell(", ".join(row.blockers) or "none"),
                    )
                )
                + " |"
            )
    lines.extend(("", "## Intentionally deferred physics and model development", ""))
    for item in report.future_physics_backlog:
        lines.append(f"- **{item.id}** ({', '.join(item.families)}): {item.description} Dependency: {item.dependency}")
    if report.diagnostics:
        lines.extend(("", "## Audit diagnostics", ""))
        lines.extend(f"- {item}" for item in report.diagnostics)
    lines.append("")
    return "\n".join(lines)
    ####


def _operation_cell(status: ExecutionDisposition, tuples: tuple[str, ...]) -> str:
    return status if not tuples else f"{status}: {', '.join(tuples)}"
    ####


def _markdown_cell(value: str) -> str:
    return value.replace("|", "\\|").replace("\n", " ")
    ####


__all__ = [
    "MISSION_COMPOSITION_PHYSICS_BACKLOG",
    "MissionCompositionBacklogItem",
    "MissionCompositionCompletionReport",
    "MissionCompositionFamilyCoverage",
    "MissionCompositionPhysicsBacklog",
    "MissionCompositionRealizationCoverage",
    "build_mission_composition_completion_report",
    "load_mission_composition_physics_backlog",
    "render_mission_composition_completion_markdown",
]
