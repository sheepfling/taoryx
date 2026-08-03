"""Common, backend-neutral contracts for family flagship showcases.

The showcase layer describes what a run must prove and how it is evidenced.
It does not own vehicle state, integration, or controller implementation.  A
provider turns a resolved showcase into a run artifact; the renderer consumes
that artifact without reopening a problem file or guessing units.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from taoryx.fidelity_contracts import (
    ControlRealization,
    LegacyFidelityTier,
    canonicalize_fidelity,
)

EvidenceGrade = Literal[
    "source",
    "identified",
    "derived",
    "estimated",
    "synthetic",
    "mixed",
    "unavailable",
]
ShowcaseOutcome = Literal[
    "completed",
    "completed_degraded",
    "partial",
    "resource_limited",
    "envelope_limited",
    "time_limited",
    "aborted",
    "numerical_failure",
]
ShowcaseArchetype = Literal[
    "mission_geometry",
    "mission_timeline",
    "dynamics_and_resources",
    "envelope_and_qualification",
    "object_lineage",
]
LineageEventType = Literal["spawn", "release", "separation", "terminal", "death"]
StartContractType = Literal[
    "grounded",
    "trimmed_airborne",
    "hover",
    "air_release",
    "separation",
    "orbit_state",
    "impact_state",
]


class FailureCode(StrEnum):
    """Stable failure vocabulary shared by showcase evaluators."""

    START_CONTRACT_FAILED = "START_CONTRACT_FAILED"
    DATA_DOMAIN_VIOLATION = "DATA_DOMAIN_VIOLATION"
    FRAME_OR_TIME_INVALID = "FRAME_OR_TIME_INVALID"
    NUMERICAL_DIVERGENCE = "NUMERICAL_DIVERGENCE"
    GUIDANCE_INFEASIBLE = "GUIDANCE_INFEASIBLE"
    CONTROL_AUTHORITY_EXCEEDED = "CONTROL_AUTHORITY_EXCEEDED"
    ACTUATOR_LIMIT_EXCEEDED = "ACTUATOR_LIMIT_EXCEEDED"
    RESOURCE_DEPLETED = "RESOURCE_DEPLETED"
    CONTACT_OR_COLLISION_FAILURE = "CONTACT_OR_COLLISION_FAILURE"
    SEGMENT_TRANSITION_FAILED = "SEGMENT_TRANSITION_FAILED"
    TERMINAL_CORRIDOR_MISSED = "TERMINAL_CORRIDOR_MISSED"
    TIMEOUT = "TIMEOUT"
    ARTIFACT_OR_REPLAY_MISMATCH = "ARTIFACT_OR_REPLAY_MISMATCH"
    UNSUPPORTED_CLAIM = "UNSUPPORTED_CLAIM"
####


class StartContract(BaseModel):
    """Family-appropriate initial condition and acceptance contract."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    type: StartContractType
    acceptance: dict[str, Any] = Field(default_factory=dict)
    settling_duration_s: float = Field(default=0.0, ge=0.0)
    hidden_settling_allowed: bool = False
####


class MissionSegmentSpec(BaseModel):
    """One named objective-bearing mission segment."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    objective: str = Field(min_length=1)
    entry_requirements: dict[str, Any] = Field(default_factory=dict)
    success_event: str = Field(min_length=1)
    failure_events: tuple[FailureCode, ...] = ()
    required_controls: tuple[str, ...] = ()
    required_observations: tuple[str, ...] = ()
####


class TerminalContract(BaseModel):
    """Explicit terminal set; timeout is never implicitly success."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    success_event: str = Field(min_length=1)
    corridor: dict[str, Any] = Field(default_factory=dict)
    dwell_s: float = Field(default=0.0, ge=0.0)
    timeout_is_success: Literal[False] = False
    failure_events: tuple[FailureCode, ...] = ()
####


class ShowcaseArchetypeSpec(BaseModel):
    """One reusable human-facing proof archetype."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: ShowcaseArchetype
    display_name: str = Field(min_length=1)
    purpose: str = Field(min_length=1)
    required_modules: tuple[str, ...] = Field(min_length=1)
    optional_modules: tuple[str, ...] = ()
####


class ShowcaseRecipe(BaseModel):
    """Family-specific realization of the common showcase archetypes."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    family: str = Field(min_length=1)
    display_name: str = Field(min_length=1)
    priority: int = Field(ge=1)
    archetypes: tuple[ShowcaseArchetype, ...] = Field(min_length=4)
    emphasis: tuple[str, ...] = Field(min_length=1)
    required_events: tuple[str, ...] = ()
    lineage_required: bool = False

    @model_validator(mode="after")
    def validate_archetypes(self) -> ShowcaseRecipe:
        if len(self.archetypes) != len(set(self.archetypes)):
            raise ValueError("showcase recipe archetypes must be unique")
        if self.lineage_required and "object_lineage" not in self.archetypes:
            raise ValueError("lineage-required showcase recipe must include object_lineage")
        return self
        ####
    ####


class ShowcaseArchetypeCatalog(BaseModel):
    """Versioned catalog of shared proof products and family recipes."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = 1
    id: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)
    archetypes: tuple[ShowcaseArchetypeSpec, ...] = Field(min_length=5)
    recipes: tuple[ShowcaseRecipe, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_catalog(self) -> ShowcaseArchetypeCatalog:
        expected = {"mission_geometry", "mission_timeline", "dynamics_and_resources", "envelope_and_qualification", "object_lineage"}
        actual = {item.id for item in self.archetypes}
        if actual != expected:
            raise ValueError(f"showcase archetype catalog must define exactly {sorted(expected)}")
        recipe_ids = [recipe.id for recipe in self.recipes]
        if len(recipe_ids) != len(set(recipe_ids)):
            raise ValueError("showcase recipe IDs must be unique")
        return self
        ####
    ####


class ObjectLineageEvent(BaseModel):
    """Accepted-boundary lifecycle event for one mission object."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    event_type: LineageEventType
    object_id: str = Field(min_length=1)
    parent_object_id: str | None = None
    time_s: float = Field(ge=0.0)
    reason: str = Field(min_length=1)
####


class ObjectLineageNode(BaseModel):
    """One active object and its parent/terminal disposition."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    role: str = Field(min_length=1)
    parent_id: str | None = None
    spawn_event_id: str | None = None
    active_from_s: float = Field(ge=0.0)
    active_to_s: float | None = Field(default=None, ge=0.0)
    death_event_id: str | None = None
    terminal_disposition: str | None = None

    @model_validator(mode="after")
    def validate_interval(self) -> ObjectLineageNode:
        if self.active_to_s is not None and self.active_to_s < self.active_from_s:
            raise ValueError("lineage active_to_s must not precede active_from_s")
        return self
        ####
    ####


class ObjectLineage(BaseModel):
    """A replayable parent/child graph for release, staging, and termination."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    nodes: tuple[ObjectLineageNode, ...] = Field(min_length=1)
    events: tuple[ObjectLineageEvent, ...] = ()

    @model_validator(mode="after")
    def validate_graph(self) -> ObjectLineage:
        node_ids = {node.id for node in self.nodes}
        if len(node_ids) != len(self.nodes):
            raise ValueError("lineage node IDs must be unique")
        event_ids = {event.id for event in self.events}
        if len(event_ids) != len(self.events):
            raise ValueError("lineage event IDs must be unique")
        node_by_id = {node.id: node for node in self.nodes}
        for node in self.nodes:
            if node.parent_id is not None and node.parent_id not in node_ids:
                raise ValueError(f"lineage parent is unknown: {node.parent_id}")
            if node.spawn_event_id is not None and node.spawn_event_id not in event_ids:
                raise ValueError(f"lineage spawn event is unknown: {node.spawn_event_id}")
            if node.death_event_id is not None and node.death_event_id not in event_ids:
                raise ValueError(f"lineage death event is unknown: {node.death_event_id}")
        for event in self.events:
            if event.object_id not in node_ids:
                raise ValueError(f"lineage event object is unknown: {event.object_id}")
            if event.parent_object_id is not None and event.parent_object_id not in node_ids:
                raise ValueError(f"lineage event parent is unknown: {event.parent_object_id}")
        for node in self.nodes:
            seen: set[str] = set()
            current = node
            while current.parent_id is not None:
                if current.id in seen:
                    raise ValueError("lineage parent graph must be acyclic")
                seen.add(current.id)
                current = node_by_id[current.parent_id]
        return self
        ####
    ####


class FamilyShowcaseTemplate(BaseModel):
    """Semantic mission template shared by vehicles in one physical family."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    version: str = Field(min_length=1)
    family: str = Field(min_length=1)
    display_name: str = Field(min_length=1)
    claim: str = Field(min_length=1)
    nonclaims: tuple[str, ...] = ()
    start_contract: StartContract
    segments: tuple[MissionSegmentSpec, ...] = Field(min_length=1)
    required_semantic_controls: tuple[str, ...] = ()
    required_events: tuple[str, ...] = ()
    terminal_contract: TerminalContract
    plot_modules: tuple[str, ...] = ()
    robustness_profile: str | None = None
    archetypes: tuple[ShowcaseArchetype, ...] = (
        "mission_geometry",
        "mission_timeline",
        "dynamics_and_resources",
        "envelope_and_qualification",
    )
    lineage_required: bool = False

    @model_validator(mode="after")
    def validate_segments(self) -> FamilyShowcaseTemplate:
        ids = [segment.id for segment in self.segments]
        if len(ids) != len(set(ids)):
            raise ValueError("showcase segment IDs must be unique")
        events = {segment.success_event for segment in self.segments}
        events.add(self.terminal_contract.success_event)
        missing = set(self.required_events).difference(events)
        if missing:
            raise ValueError(f"showcase required events are not produced: {sorted(missing)}")
        if self.lineage_required and "object_lineage" not in self.archetypes:
            raise ValueError("lineage-required showcase must include object_lineage")
        return self
        ####
    ####


class VehicleShowcaseBinding(BaseModel):
    """Vehicle/data-package binding for a family showcase."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    family: str = Field(min_length=1)
    version: str = Field(min_length=1)
    vehicle_package: str = Field(min_length=1)
    source_hashes: tuple[str, ...] = ()
    start_state_factory: str = Field(min_length=1)
    envelope: dict[str, Any] = Field(default_factory=dict)
    terminal_tolerances: dict[str, Any] = Field(default_factory=dict)
    supported_segments: tuple[str, ...] = ()
    evidence_grade: EvidenceGrade
    unsupported_behaviors: tuple[str, ...] = ()
####


class FidelityShowcaseRealization(BaseModel):
    """Named realization of a semantic showcase at one dynamics fidelity."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    fidelity: LegacyFidelityTier
    control_realization: ControlRealization = "unspecified"
    realization_id: str = Field(min_length=1)
    state_schema: tuple[str, ...] = Field(min_length=1)
    semantic_command_mapping: dict[str, str] = Field(default_factory=dict)
    physical_effectors: tuple[str, ...] = ()
    available_physics: tuple[str, ...] = ()
    claim: str = Field(min_length=1)
    nonclaims: tuple[str, ...] = ()
    evidence_grade: EvidenceGrade

    @model_validator(mode="after")
    def normalize_fidelity(self) -> FidelityShowcaseRealization:
        """Require explicit semantics for the legacy rigid-body label."""

        normalized = canonicalize_fidelity(
            self.fidelity,
            control_realization=self.control_realization,
        )
        if normalized != self.fidelity:
            object.__setattr__(self, "fidelity", normalized)
        return self

    @model_validator(mode="after")
    def validate_control_boundary(self) -> FidelityShowcaseRealization:
        if self.control_realization == "surface_allocated" and not self.physical_effectors:
            raise ValueError("surface_allocated showcase realizations must declare physical effectors")
        if self.control_realization == "direct_wrench" and self.physical_effectors:
            raise ValueError("direct_wrench showcase realizations must not imply active physical effectors")
        return self
        ####
    ####
####


class ArtifactFile(BaseModel):
    """One self-contained artifact entry with content identity."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    path: str = Field(min_length=1)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    media_type: str = Field(min_length=1)
    required: bool = True
####


class EvidenceBoardSpec(BaseModel):
    """Renderer declaration for the standard card and 4K evidence board."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    profile: str = Field(min_length=1)
    modules: tuple[str, ...] = Field(min_length=1)
    event_marker_policy: Literal["family_local"] = "family_local"
    missing_channel_policy: Literal["unavailable"] = "unavailable"
    units_required: bool = True
####


class VehicleInterfaceEvidence(BaseModel):
    """Exact caller-facing interface retained by a composition-backed board."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    interface_id: str = Field(min_length=1)
    fingerprint_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    fidelity: LegacyFidelityTier
    control_realization: ControlRealization
    evidence_status: str = Field(min_length=1)
    available_authority_profiles: tuple[str, ...] = ()
    available_observation_profiles: tuple[str, ...] = ()
    selected_observation_profile: str | None = None
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def normalize_fidelity(self) -> VehicleInterfaceEvidence:
        """Keep the recorded interface on one unambiguous fidelity tier."""

        normalized = canonicalize_fidelity(
            self.fidelity,
            control_realization=self.control_realization,
        )
        if normalized != self.fidelity:
            object.__setattr__(self, "fidelity", normalized)
        if (
            self.selected_observation_profile is not None
            and self.selected_observation_profile not in self.available_observation_profiles
        ):
            raise ValueError("showcase-selected observation profile is not available from the retained interface")
        return self
        ####
    ####
####


class ShowcaseRunArtifact(BaseModel):
    """Manifest for a replayable, evaluator-backed showcase run."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: str = "taoryx.showcase/v1alpha1"
    run_id: str = Field(min_length=1)
    showcase_id: str = Field(min_length=1)
    vehicle_binding_id: str = Field(min_length=1)
    fidelity: LegacyFidelityTier
    control_realization: ControlRealization = "unspecified"
    realization: FidelityShowcaseRealization | None = None
    scenario_contract_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    outcome: ShowcaseOutcome
    claim: str = Field(min_length=1)
    nonclaims: tuple[str, ...] = ()
    files: tuple[ArtifactFile, ...] = Field(min_length=1)
    board: EvidenceBoardSpec
    archetypes: tuple[ShowcaseArchetype, ...] = ()
    object_lineage: ObjectLineage | None = None
    vehicle_interface: VehicleInterfaceEvidence | None = None

    @model_validator(mode="after")
    def normalize_fidelity(self) -> ShowcaseRunArtifact:
        """Require explicit semantics for the legacy rigid-body label."""

        normalized = canonicalize_fidelity(
            self.fidelity,
            control_realization=self.control_realization,
        )
        if normalized != self.fidelity:
            object.__setattr__(self, "fidelity", normalized)
        return self

    @model_validator(mode="after")
    def validate_artifacts(self) -> ShowcaseRunArtifact:
        paths = [item.path for item in self.files]
        if len(paths) != len(set(paths)):
            raise ValueError("showcase artifact paths must be unique")
        required = {item.path for item in self.files if item.required}
        if "manifest.json" not in required:
            raise ValueError("showcase artifact must include a required manifest.json")
        if "object_lineage" in self.archetypes and self.object_lineage is None:
            raise ValueError("object_lineage archetype requires a lineage artifact")
        return self
        ####

    @model_validator(mode="after")
    def validate_realization_boundary(self) -> ShowcaseRunArtifact:
        """Ensure retained realization metadata matches legacy summary fields."""

        if self.realization is not None:
            if self.realization.fidelity != self.fidelity:
                raise ValueError("showcase artifact fidelity disagrees with its realization")
            if self.realization.control_realization != self.control_realization:
                raise ValueError("showcase artifact control realization disagrees with its realization")
            if self.realization.claim != self.claim:
                raise ValueError("showcase artifact claim disagrees with its realization")
            if self.realization.nonclaims != self.nonclaims:
                raise ValueError("showcase artifact nonclaims disagree with its realization")
        if self.vehicle_interface is not None:
            if self.vehicle_interface.fidelity != self.fidelity:
                raise ValueError("showcase artifact fidelity disagrees with its vehicle interface")
            if self.vehicle_interface.control_realization != self.control_realization:
                raise ValueError("showcase artifact control realization disagrees with its vehicle interface")
        return self
        ####
    ####


__all__ = [
    "ArtifactFile",
    "ControlRealization",
    "EvidenceBoardSpec",
    "FailureCode",
    "LineageEventType",
    "FidelityShowcaseRealization",
    "FamilyShowcaseTemplate",
    "MissionSegmentSpec",
    "ObjectLineage",
    "ObjectLineageEvent",
    "ObjectLineageNode",
    "ShowcaseArchetype",
    "ShowcaseArchetypeCatalog",
    "ShowcaseArchetypeSpec",
    "ShowcaseRecipe",
    "ShowcaseRunArtifact",
    "ShowcaseOutcome",
    "StartContract",
    "TerminalContract",
    "VehicleShowcaseBinding",
    "VehicleInterfaceEvidence",
]
####
