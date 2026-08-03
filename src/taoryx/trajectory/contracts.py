"""Provider-neutral Alpha 2 case and schema contracts.

This module must remain free of imports from :mod:`taoryx.runtime`.  A
provider may compile these models into a native runtime, but the case
contract is useful before any simulator implementation is loaded.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

from ..controller_realization import ControllerRealization
from ..fidelity_contracts import LegacyFidelityTier

CapabilityStatus = Literal["native", "emulated", "approximated", "unsupported"]
FidelityProfile = LegacyFidelityTier
ParameterKind = Literal["number", "integer", "boolean", "string"]
ParameterRole = Literal["independent", "derived", "developer"]
ModifierOperation = Literal["set", "add", "scale"]
VariantPolicy = Literal["reject", "project"]
VariantStatus = Literal["qualified", "extended", "projected"]
AuthorityMode = Literal["autopilot", "commanded", "overlay", "direct", "mixed"]
HoldBehavior = Literal["hold", "default", "failsafe"]
ControlInputMode = Literal["absolute", "rate"]
ObservationKind = Literal["state", "actuator_achieved", "resource", "event_prediction", "diagnostic"]
SemanticLevel = Literal["guidance", "attitude_rate", "body_motion", "effector", "state", "resource", "event", "diagnostic"]
ChannelAvailability = Literal["always", "conditional", "unavailable"]
EvidenceGrade = Literal["source", "identified", "derived", "estimated", "synthetic", "mixed", "unavailable", "unknown"]


class CaseValue(BaseModel):
    """A user-supplied value with an optional display/input unit."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    value: Any
    unit: str | None = None
####


class ParameterSchema(BaseModel):
    """Declaration for one configurable parameter."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    kind: ParameterKind = "number"
    canonical_unit: str | None = None
    default: Any = None
    required: bool = False
    minimum: float | None = None
    maximum: float | None = None
    role: ParameterRole = "independent"
    qualified_minimum: float | None = None
    qualified_maximum: float | None = None
    search_transform: Literal["linear", "log", "fraction", "categorical"] = "linear"
    coupling_group: str | None = None
    requires_retrim: bool = False
    requires_requalification: bool = False
    provenance: str = ""
    description: str = ""
####


class VariantModifier(BaseModel):
    """A bounded semantic change applied to a resolved family parameter."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    target: str = Field(min_length=1)
    operation: ModifierOperation
    canonical_unit: str | None = None
    minimum: float | None = None
    maximum: float | None = None
    qualified_minimum: float | None = None
    qualified_maximum: float | None = None
    coupling_group: str | None = None
    requires_retrim: bool = False
    requires_requalification: bool = False
    provenance: str = ""
    description: str = ""
####


class DerivedParameter(BaseModel):
    """A small declarative derivation over already-resolved parameters."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    operation: Literal["sum", "difference", "product", "ratio", "scale"]
    dependencies: tuple[str, ...] = Field(min_length=1)
    canonical_unit: str | None = None
    constant: float = 0.0
    provenance: str = ""
####


class VariantSpace(BaseModel):
    """Bounded candidate and derivation metadata for one family."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    policy: VariantPolicy = "reject"
    modifiers: tuple[VariantModifier, ...] = ()
    derived: tuple[DerivedParameter, ...] = ()
####


class VariantResolutionReport(BaseModel):
    """Machine-readable validity and qualification result for one candidate."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    status: VariantStatus
    candidate_original: dict[str, Any] = Field(default_factory=dict)
    candidate_applied: dict[str, Any] = Field(default_factory=dict)
    projection_distance: float = 0.0
    modifiers_applied: tuple[str, ...] = ()
    derived_values: dict[str, Any] = Field(default_factory=dict)
    invalidations: tuple[str, ...] = ()
    diagnostics: tuple[str, ...] = ()
    fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
####


class ControlSchema(BaseModel):
    """Provider-neutral declaration for an externally controllable channel."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    unit: str | None = None
    default: float = 0.0
    minimum: float | None = None
    maximum: float | None = None
    authority_modes: tuple[AuthorityMode, ...] = ("autopilot", "commanded", "overlay")
    default_authority: AuthorityMode = "commanded"
    command_modes: tuple[ControlInputMode, ...] = ("absolute",)
    default_command_mode: ControlInputMode = "absolute"
    rate_unit: str | None = None
    cadence_s: float = Field(default=0.0, ge=0.0)
    hold_behavior: HoldBehavior = "hold"
    failsafe_value: float | None = None
    overlay_minimum: float | None = None
    overlay_maximum: float | None = None
    rate_limit_per_s: float | None = Field(default=None, gt=0.0)
    frame: str | None = None
    semantic_level: SemanticLevel = "effector"
    availability: ChannelAvailability = "always"
    achieved_observation_id: str | None = None
    allocation_id: str | None = None
    evidence_grade: EvidenceGrade = "unknown"
    uncertainty: str | None = None
    description: str = ""
####


@dataclass(frozen=True, slots=True)
class ControlFrame:
    """Neutral boundary inputs for one control-arbitration transition.

    ``values`` is the legacy/common external-command mapping.  The explicit
    mappings let a caller provide autopilot and direct-effector candidates at
    the same boundary without either source writing around the arbiter.
    """

    values: Mapping[str, float] = field(default_factory=dict)
    rates: Mapping[str, float] = field(default_factory=dict)
    autopilot: Mapping[str, float] = field(default_factory=dict)
    autopilot_rates: Mapping[str, float] = field(default_factory=dict)
    direct: Mapping[str, float] = field(default_factory=dict)
    direct_rates: Mapping[str, float] = field(default_factory=dict)
    authority: Mapping[str, AuthorityMode] = field(default_factory=dict)
    input_modes: Mapping[str, ControlInputMode] = field(default_factory=dict)
    active: Mapping[str, bool] = field(default_factory=dict)
    valid: bool = True
####


class ObservationSchema(BaseModel):
    """Provider-neutral declaration for an observable channel."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    unit: str | None = None
    source: str | None = None
    kind: ObservationKind = "state"
    frame: str | None = None
    semantic_level: SemanticLevel = "state"
    availability: ChannelAvailability = "always"
    neutral_value: Any = None
    evidence_grade: EvidenceGrade = "unknown"
    uncertainty: str | None = None
    description: str = ""
####


class CapabilitySchema(BaseModel):
    """Family-level capabilities negotiated before a provider is loaded."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    fidelities: tuple[FidelityProfile, ...] = ()
    control_intents: tuple[str, ...] = ()
    observation_kinds: tuple[ObservationKind, ...] = ()
    modes: tuple[str, ...] = ()
    terminal_conditions: tuple[str, ...] = ()
    supports_resources: bool = False
    supports_allocation: bool = False
    supports_mode_transitions: bool = False
    supports_checkpoint: bool = True
    provenance: str = ""
    description: str = ""
####


class ComponentSlot(BaseModel):
    """Typed composition point for a family component or loadout."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    component_kind: str = Field(min_length=1)
    required: bool = False
    compatible_fidelities: tuple[FidelityProfile, ...] = ()
    selected_component: str | None = None
    provenance: str = ""
####


class ResourceSchema(BaseModel):
    """First-class consumable or capacity resource declaration."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    resource_kind: str = Field(min_length=1)
    canonical_unit: str | None = None
    default: float | None = None
    minimum: float | None = None
    reserve: float | None = None
    consumption_channels: tuple[str, ...] = ()
    observation_id: str | None = None
    evidence_grade: EvidenceGrade = "estimated"
    uncertainty: str | None = None
    provenance: str = ""
####


class AllocationSchema(BaseModel):
    """Requested-to-effector allocation contract."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    requested_channels: tuple[str, ...] = ()
    allocated_channels: tuple[str, ...] = ()
    achieved_observations: tuple[str, ...] = ()
    supports_rate_commands: bool = False
    frame: str | None = None
    provenance: str = ""
####


class ModeTransitionSchema(BaseModel):
    """Typed hybrid-mode transition with explicit safety and handoff data."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    from_mode: str = Field(min_length=1)
    to_mode: str = Field(min_length=1)
    entry_guards: tuple[str, ...] = ()
    exit_guards: tuple[str, ...] = ()
    abort_to: str | None = None
    hysteresis_s: float = Field(default=0.0, ge=0.0)
    schedules: dict[str, Any] = Field(default_factory=dict)
    controller_handoff: dict[str, Any] = Field(default_factory=dict)
    resource_continuity: bool = True
    state_continuity: bool = True
    provenance: str = ""
####


class FamilyPackage(BaseModel):
    """Versioned family metadata used by the Alpha 2 resolver."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    family_id: str = Field(min_length=1)
    version: str = Field(min_length=1)
    display_name: str = Field(min_length=1)
    fidelities: tuple[FidelityProfile, ...]
    parameters: tuple[ParameterSchema, ...]
    variants: dict[str, dict[str, Any]] = Field(default_factory=dict)
    variant_space: VariantSpace = Field(default_factory=VariantSpace)
    loadouts: dict[str, dict[str, Any]] = Field(default_factory=dict)
    missions: dict[str, dict[str, Any]] = Field(default_factory=dict)
    segment_plans: dict[str, dict[str, Any]] = Field(default_factory=dict)
    segment_graphs: dict[str, dict[str, Any]] = Field(default_factory=dict)
    controls: tuple[ControlSchema, ...] = ()
    observations: tuple[ObservationSchema, ...] = ()
    capabilities: CapabilitySchema = Field(default_factory=CapabilitySchema)
    component_slots: tuple[ComponentSlot, ...] = ()
    resources: tuple[ResourceSchema, ...] = ()
    allocations: tuple[AllocationSchema, ...] = ()
    mode_transitions: tuple[ModeTransitionSchema, ...] = ()
    evidence_grade: EvidenceGrade = "unknown"
    uncertainty: str | None = None
    provenance: str = ""

    def parameter_map(self) -> dict[str, ParameterSchema]:
        """Return schemas indexed by stable semantic identifier."""

        result = {item.id: item for item in self.parameters}
        if len(result) != len(self.parameters):
            raise ValueError(f"family {self.family_id!r} has duplicate parameter IDs")
        return result
        ####

    def control_map(self) -> dict[str, ControlSchema]:
        """Return controls indexed by stable semantic identifier."""

        result = {item.id: item for item in self.controls}
        if len(result) != len(self.controls):
            raise ValueError(f"family {self.family_id!r} has duplicate control IDs")
        return result
        ####

    def observation_map(self) -> dict[str, ObservationSchema]:
        """Return observations indexed by stable semantic identifier."""

        result = {item.id: item for item in self.observations}
        if len(result) != len(self.observations):
            raise ValueError(f"family {self.family_id!r} has duplicate observation IDs")
        return result
        ####
####


class FamilyCatalog(BaseModel):
    """Collection of versioned family packages."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(default=1, ge=1)
    families: tuple[FamilyPackage, ...]

    def family(self, family_id: str) -> FamilyPackage:
        """Return one family or raise a diagnostic-friendly error."""

        matches = tuple(item for item in self.families if item.family_id == family_id)
        if not matches:
            raise KeyError(f"unknown vehicle family {family_id!r}")
        if len(matches) != 1:
            raise ValueError(f"family catalog contains duplicate ID {family_id!r}")
        return matches[0]
        ####
####


class CaseIntent(BaseModel):
    """Human-authored, provider-neutral request before preset resolution."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(default=1, ge=1)
    case_id: str = Field(min_length=1)
    family: str = Field(min_length=1)
    fidelity: FidelityProfile
    variant: str = "baseline"
    loadout: str = "baseline"
    mission: str = "baseline"
    segment_plan: str = "baseline"
    controller: str = "none"
    variant_parameters: dict[str, CaseValue] = Field(default_factory=dict)
    overrides: dict[str, CaseValue] = Field(default_factory=dict)
    requested_controls: tuple[str, ...] = ()
    requested_observations: tuple[str, ...] = ()
    extensions: dict[str, Any] = Field(default_factory=dict)
####


class ProvenanceRecord(BaseModel):
    """Audit trail for one final resolved value."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    parameter_id: str = Field(min_length=1)
    source: str = Field(min_length=1)
    input_unit: str | None = None
    canonical_unit: str | None = None
    input_value: Any = None
    canonical_value: Any = None
    derivation: str | None = None
    dependencies: tuple[str, ...] = ()
####


class ResolvedValue(BaseModel):
    """A final canonical parameter value and its source classification."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    value: Any
    unit: str | None = None
    source: str
####


class ResolvedVariant(BaseModel):
    """Immutable vehicle binding produced before provider compilation."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    family: str = Field(min_length=1)
    family_version: str = Field(min_length=1)
    fidelity: FidelityProfile
    variant: str = Field(min_length=1)
    loadout: str = Field(min_length=1)
    parameters: dict[str, ResolvedValue]
    resolution: VariantResolutionReport
    fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
####


class ResolvedCase(BaseModel):
    """Immutable, canonical case representation shared by providers."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(default=1, ge=1)
    case_id: str = Field(min_length=1)
    family: str = Field(min_length=1)
    family_version: str = Field(min_length=1)
    fidelity: FidelityProfile
    variant: str
    loadout: str
    mission: str
    segment_plan: str
    controller: str
    controller_realization: ControllerRealization | None = None
    parameters: dict[str, ResolvedValue]
    controls: tuple[ControlSchema, ...]
    observations: tuple[ObservationSchema, ...]
    capabilities: CapabilitySchema = Field(default_factory=CapabilitySchema)
    component_slots: tuple[ComponentSlot, ...] = ()
    resources: tuple[ResourceSchema, ...] = ()
    allocations: tuple[AllocationSchema, ...] = ()
    mode_transitions: tuple[ModeTransitionSchema, ...] = ()
    evidence_grade: EvidenceGrade = "unknown"
    uncertainty: str | None = None
    provenance: tuple[ProvenanceRecord, ...]
    segment_graph: dict[str, Any] = Field(default_factory=dict)
    extensions: dict[str, Any] = Field(default_factory=dict)
    resolved_variant: ResolvedVariant | None = None
    identity_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")

    @property
    def variant_resolution(self) -> VariantResolutionReport | None:
        """Backward-compatible access to the nested resolution report."""

        return None if self.resolved_variant is None else self.resolved_variant.resolution
        ####

    def canonical_payload(self) -> dict[str, Any]:
        """Return the identity payload without the self-referential digest."""

        payload = self.model_dump(mode="json")
        payload.pop("identity_sha256", None)
        # Preserve the pre-realization Alpha 2 identity for cases that do not
        # yet select a controller.  Once a realization exists it is part of
        # the immutable case identity.
        if self.controller_realization is None:
            payload.pop("controller_realization", None)
        return payload
        ####

    def recompute_identity(self) -> str:
        """Compute the stable case identity from canonical JSON."""

        encoded = json.dumps(self.canonical_payload(), sort_keys=True, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()
        ####

    def write_json(self, path: str) -> None:
        """Write the resolved case as deterministic, human-readable JSON."""

        from pathlib import Path

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(self.model_dump(mode="json"), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        ####

    def explain(self, parameter_id: str | None = None) -> list[dict[str, Any]]:
        """Return provenance rows, optionally narrowed to one parameter."""

        rows = [item.model_dump(mode="json") for item in self.provenance]
        if parameter_id is None:
            return rows
        return [row for row in rows if row["parameter_id"] == parameter_id]
        ####
####


__all__ = [
    "AuthorityMode",
    "AllocationSchema",
    "CapabilitySchema",
    "ChannelAvailability",
    "CapabilityStatus",
    "CaseIntent",
    "CaseValue",
    "ControlSchema",
    "ControlInputMode",
    "ControlFrame",
    "EvidenceGrade",
    "ComponentSlot",
    "DerivedParameter",
    "FamilyCatalog",
    "FamilyPackage",
    "FidelityProfile",
    "HoldBehavior",
    "ModifierOperation",
    "ModeTransitionSchema",
    "ObservationSchema",
    "ObservationKind",
    "ParameterSchema",
    "ParameterRole",
    "ProvenanceRecord",
    "ResolvedCase",
    "ResolvedVariant",
    "ResolvedValue",
    "ResourceSchema",
    "SemanticLevel",
    "VariantModifier",
    "VariantPolicy",
    "VariantResolutionReport",
    "VariantSpace",
    "VariantStatus",
]
####
