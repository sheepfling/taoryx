"""Discoverable vehicle-to-trajectory composition registry.

This module is deliberately a *projection*, not a second vehicle model
database.  Physics, source provenance, and fidelity readiness continue to
belong to their existing authorities.  The composition registry adds the
missing user-facing answer: given a vehicle-family selection, which fidelity
tiers, initialization contracts, reusable segment kinds, and mission
templates may be requested?

Entries may describe planned work.  ``promotion_status`` on the resolved
fidelity record remains the authoritative distinction between declared,
development, and qualified availability.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .family_manifest import UnifiedFamilyManifest, UnifiedFamilyManifestCatalog, load_unified_family_manifest_catalog
from .fidelity_contracts import CANONICAL_FIDELITY_TIERS, ControlRealization, FidelityTier, control_realization_for, runtime_fidelity_for
from .vehicle_registry import ROOT

VEHICLE_COMPOSITION_REGISTRY = ROOT / "verification/vehicle_composition_registry.yaml"

CompositionStatus = Literal["runnable", "development", "planned"]


class CompositionParameter(BaseModel):
    """One user-visible field accepted by an initialization or segment."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    canonical_unit: str | None = None
    required: bool = False
    description: str = Field(min_length=1)
####


class InitializationContract(BaseModel):
    """Family-appropriate initial-state contract and its public inputs."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    status: CompositionStatus = "development"
    description: str = Field(min_length=1)
    compatible_fidelities: tuple[FidelityTier, ...] = Field(min_length=1)
    parameters: tuple[CompositionParameter, ...] = ()

    @model_validator(mode="after")
    def validate_parameters(self) -> InitializationContract:
        ids = [item.id for item in self.parameters]
        if len(ids) != len(set(ids)):
            raise ValueError(f"initialization {self.id!r} has duplicate parameters")
        return self
        ####
####


class SegmentContract(BaseModel):
    """One composable semantic segment, independent of a native controller."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    status: CompositionStatus = "development"
    description: str = Field(min_length=1)
    compatible_fidelities: tuple[FidelityTier, ...] = Field(min_length=1)
    parameters: tuple[CompositionParameter, ...] = ()
    required_control_intents: tuple[str, ...] = ()
    preserves_state: bool = True
    permitted_transition_events: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_parameters(self) -> SegmentContract:
        ids = [item.id for item in self.parameters]
        if len(ids) != len(set(ids)):
            raise ValueError(f"segment {self.id!r} has duplicate parameters")
        return self
        ####
####


class MissionTemplateContract(BaseModel):
    """One ordered composition recipe advertised by a vehicle family."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    status: CompositionStatus = "development"
    description: str = Field(min_length=1)
    initialization_contracts: tuple[str, ...] = Field(min_length=1)
    compatible_fidelities: tuple[FidelityTier, ...] = Field(min_length=1)
    segment_sequence: tuple[str, ...] = Field(min_length=1)
    backing_templates: tuple[str, ...] = ()
    qualification_missions: tuple[str, ...] = ()
####


class VehicleCompositionDeclaration(BaseModel):
    """Human-facing trajectory-composition overlay for one registered family."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    family_id: str = Field(min_length=1)
    initialization_contracts: tuple[InitializationContract, ...] = Field(min_length=1)
    segment_contracts: tuple[SegmentContract, ...] = Field(min_length=1)
    mission_templates: tuple[MissionTemplateContract, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_references(self) -> VehicleCompositionDeclaration:
        initialization_ids = {item.id for item in self.initialization_contracts}
        segment_ids = {item.id for item in self.segment_contracts}
        if len(initialization_ids) != len(self.initialization_contracts):
            raise ValueError(f"family {self.family_id!r} has duplicate initialization contracts")
        if len(segment_ids) != len(self.segment_contracts):
            raise ValueError(f"family {self.family_id!r} has duplicate segment contracts")
        mission_ids = [item.id for item in self.mission_templates]
        if len(mission_ids) != len(set(mission_ids)):
            raise ValueError(f"family {self.family_id!r} has duplicate mission templates")
        for mission in self.mission_templates:
            unknown_initialization = sorted(set(mission.initialization_contracts) - initialization_ids)
            unknown_segments = sorted(set(mission.segment_sequence) - segment_ids)
            if unknown_initialization:
                raise ValueError(f"mission {mission.id!r} references unknown initialization: {unknown_initialization}")
            if unknown_segments:
                raise ValueError(f"mission {mission.id!r} references unknown segments: {unknown_segments}")
        return self
        ####
####


class VehicleCompositionRegistry(BaseModel):
    """Source registry for discoverable family trajectory composition."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(alias="schema", min_length=1)
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    vehicles: tuple[VehicleCompositionDeclaration, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_families(self) -> VehicleCompositionRegistry:
        ids = [item.family_id for item in self.vehicles]
        if len(ids) != len(set(ids)):
            raise ValueError("composition registry contains duplicate family IDs")
        return self
        ####
####


@dataclass(frozen=True, slots=True)
class ResolvedVehicleComposition:
    """Composition entry joined to the canonical family/fidelity authority."""

    family: UnifiedFamilyManifest
    declaration: VehicleCompositionDeclaration

    def as_dict(self) -> dict[str, object]:
        """Return the user-facing inspect payload without copying plant data."""

        # Import here so the execution catalog can consume compiled
        # compositions without turning this discovery projection into a
        # registry-to-runtime import cycle.
        from .vehicle_execution_bindings import execution_binding_records
        from .vehicle_interface import interface_contract_for_composition, validate_vehicle_interface_contract

        tier_records: dict[str, object] = {}
        interface_records: dict[str, object] = {}
        for tier in CANONICAL_FIDELITY_TIERS:
            binding = self.family.family.tiers[tier]
            tier_records[tier] = {
                "profile_id": binding.profile_id,
                "declared": binding.profile_id is not None,
                "promotion_status": binding.promotion_status,
                "blockers": list(binding.blockers),
                "required_operations": list(binding.required_operations),
                "runtime_fidelity": runtime_fidelity_for(tier),
                "control_realization": resolved_control_realization_for(self.family.family_id, tier),
            }
            interface = interface_contract_for_composition(self, tier)
            findings = validate_vehicle_interface_contract(interface)
            interface_records[tier] = {
                "interface_id": interface.id,
                "fingerprint_sha256": interface.fingerprint,
                "validation_status": "pass" if not findings else "fail",
                "available_authority_profiles": [
                    item.id for item in interface.authority_profiles if item.availability == "available"
                ],
                "available_observation_profiles": [
                    item.id for item in interface.observation_profiles if item.availability == "available"
                ],
            }
        return {
            "schema": "taoryx.vehicle-composition/v1alpha1",
            "vehicle_id": self.family.family.vehicle_registry_id or self.family.family_id,
            "family_id": self.family.family_id,
            "display_name": (
                str(self.family.vehicle_definition.get("display_name"))
                if self.family.vehicle_definition is not None
                else self.family.source_manifest.display_name if self.family.source_manifest is not None else self.family.family_id
            ),
            "physical_family": self.family.family.physical_family,
            "mission_overlay": self.family.family.mission_overlay,
            "adapter_id": self.family.family.adapter_id,
            "automatic_lowering": self.family.family.automatic_lowering,
            "source_manifest": self.family.source_manifest_path,
            "fidelities": tier_records,
            "interfaces": interface_records,
            "initialization_contracts": [item.model_dump(mode="json") for item in self.declaration.initialization_contracts],
            "segment_contracts": [item.model_dump(mode="json") for item in self.declaration.segment_contracts],
            "mission_templates": [item.model_dump(mode="json") for item in self.declaration.mission_templates],
            "execution_bindings": execution_binding_records(self.family.family_id),
        }
        ####
####


@dataclass(frozen=True, slots=True)
class ResolvedVehicleCompositionCatalog:
    """Validated complete user-facing vehicle registry."""

    vehicles: tuple[ResolvedVehicleComposition, ...]

    def vehicle(self, identifier: str) -> ResolvedVehicleComposition:
        """Resolve a stable family or native vehicle identifier."""

        matches = tuple(
            item
            for item in self.vehicles
            if identifier in {item.family.family_id, item.family.family.vehicle_registry_id}
        )
        if not matches:
            raise KeyError(f"unknown vehicle or family identifier {identifier!r}")
        if len(matches) != 1:
            raise ValueError(f"ambiguous vehicle identifier {identifier!r}")
        return matches[0]
        ####

    def list_dict(self) -> list[dict[str, object]]:
        """Return compact records suitable for a command-line listing."""

        result: list[dict[str, object]] = []
        for item in self.vehicles:
            payload = item.as_dict()
            fidelities = payload["fidelities"]
            assert isinstance(fidelities, Mapping)
            execution_bindings = payload["execution_bindings"]
            assert isinstance(execution_bindings, list)
            result.append(
                {
                    "vehicle_id": payload["vehicle_id"],
                    "family_id": payload["family_id"],
                    "display_name": payload["display_name"],
                    "physical_family": payload["physical_family"],
                    "fidelities": {
                        name: record["promotion_status"]
                        for name, record in fidelities.items()
                        if isinstance(record, Mapping) and record["declared"]
                    },
                    "mission_templates": [template.id for template in item.declaration.mission_templates],
                    "runnable_operations": sorted(
                        {
                            str(binding["operation"])
                            for binding in execution_bindings
                            if isinstance(binding, Mapping) and binding.get("status") == "runnable"
                        }
                    ),
                }
            )
        return result
        ####
####

def load_vehicle_composition_registry(path: str | Path | None = None) -> VehicleCompositionRegistry:
    """Load the declarative user-facing composition overlay."""

    source = Path(path) if path is not None else VEHICLE_COMPOSITION_REGISTRY
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{source} must contain a mapping")
    return VehicleCompositionRegistry.model_validate(payload)
    ####


def resolved_control_realization_for(family_id: str, tier: FidelityTier) -> ControlRealization:
    """Return the profile-specific realization when a tier has an exception.

    Most pseudo-6DOF profiles use a named response law.  The passive tumbling
    profile is deliberately different: it reuses native rigid-body dynamics
    and declares ``uncontrolled``.  Looking through the profile catalog here
    keeps a composition or inspection record from relabeling that exception as
    a generic controller merely because it shares the pseudo tier.
    """

    if tier != "pseudo_6dof":
        return control_realization_for(tier)
    from .trajectory.pseudo6dof_profiles import load_pseudo6dof_catalog

    _, profile = load_pseudo6dof_catalog().for_family(family_id)
    return profile.control_realization
    ####


def load_resolved_vehicle_composition_catalog(
    *,
    registry: VehicleCompositionRegistry | None = None,
    manifests: UnifiedFamilyManifestCatalog | None = None,
) -> ResolvedVehicleCompositionCatalog:
    """Join declared composition to authoritative family and fidelity records."""

    resolved_registry = registry or load_vehicle_composition_registry()
    resolved_manifests = manifests or load_unified_family_manifest_catalog()
    if resolved_manifests.errors:
        manifest_details = "; ".join(f"{item.family_id}: {item.code}" for item in resolved_manifests.errors)
        raise ValueError(f"unified family manifest cannot support composition registry: {manifest_details}")
    manifest_by_id = {item.family_id: item for item in resolved_manifests.families}
    declaration_by_id = {item.family_id: item for item in resolved_registry.vehicles}
    missing_declarations = sorted(set(manifest_by_id) - set(declaration_by_id))
    unknown_declarations = sorted(set(declaration_by_id) - set(manifest_by_id))
    if missing_declarations or unknown_declarations:
        join_details: list[str] = []
        if missing_declarations:
            join_details.append(f"missing composition declarations: {', '.join(missing_declarations)}")
        if unknown_declarations:
            join_details.append(f"unknown composition declarations: {', '.join(unknown_declarations)}")
        raise ValueError("; ".join(join_details))

    for declaration in resolved_registry.vehicles:
        family = manifest_by_id[declaration.family_id]
        declared_tiers = {tier for tier, binding in family.family.tiers.items() if binding.profile_id is not None}
        for initialization in declaration.initialization_contracts:
            unsupported = sorted(set(initialization.compatible_fidelities) - declared_tiers)
            if unsupported:
                raise ValueError(f"{declaration.family_id}/{initialization.id} declares unavailable fidelities: {unsupported}")
        for segment in declaration.segment_contracts:
            unsupported = sorted(set(segment.compatible_fidelities) - declared_tiers)
            if unsupported:
                raise ValueError(f"{declaration.family_id}/{segment.id} declares unavailable fidelities: {unsupported}")
        for mission in declaration.mission_templates:
            unsupported = sorted(set(mission.compatible_fidelities) - declared_tiers)
            if unsupported:
                raise ValueError(f"{declaration.family_id}/{mission.id} declares unavailable fidelities: {unsupported}")

    return ResolvedVehicleCompositionCatalog(
        tuple(ResolvedVehicleComposition(manifest_by_id[item.family_id], item) for item in resolved_registry.vehicles)
    )
    ####


__all__ = [
    "CompositionParameter",
    "InitializationContract",
    "MissionTemplateContract",
    "ResolvedVehicleComposition",
    "ResolvedVehicleCompositionCatalog",
    "SegmentContract",
    "VEHICLE_COMPOSITION_REGISTRY",
    "VehicleCompositionDeclaration",
    "VehicleCompositionRegistry",
    "load_resolved_vehicle_composition_catalog",
    "load_vehicle_composition_registry",
    "resolved_control_realization_for",
]
