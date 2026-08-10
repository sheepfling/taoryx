"""Horizontal conformance for the common vehicle integration seam.

The profile catalog describes executable fidelity contracts.  This registry
adds the physical-family overlay and adapter identity so a new vehicle can be
integrated by declaring the same four tier slots rather than inventing a new
selection path.  Validation is deliberately fail-closed: profile IDs must
resolve to the canonical catalog, passive families may mark actuator tiers
``null``, and automatic lowering remains governed by checked evidence.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from .family_adapter import ADAPTER_OPERATIONS, AdapterOperation
from .fidelity_contracts import CANONICAL_FIDELITY_TIERS, FidelityTier
from .plugins.resources import packaged_resource_fallback
from .trajectory.pseudo6dof_profiles import (
    AutomaticLoweringReport,
    Pseudo6DOFCatalog,
    build_automatic_lowering_report,
    load_pseudo6dof_catalog,
    load_qualified_fidelity_evidence,
)
from .vehicle_registry import ROOT

HORIZONTAL_REGISTRY = packaged_resource_fallback(
    ROOT / "verification/horizontal_fidelity_registry.yaml",
    package="taoryx_reference_models",
    resource="data/verification/horizontal_fidelity_registry.yaml",
)

PromotionStatus = Literal["qualified", "development", "planned", "not_applicable"]
DataEvidenceStatus = Literal["ready", "source_declared"]
PROMOTION_OPERATIONS = frozenset(ADAPTER_OPERATIONS)


class HorizontalTierBinding(BaseModel):
    """One canonical fidelity slot in a family integration manifest."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    profile_id: str | None = None
    promotion_status: PromotionStatus = "development"
    required_operations: tuple[AdapterOperation, ...] = ()
    blockers: tuple[str, ...] = ()

    @field_validator("required_operations", mode="before")
    @classmethod
    def validate_operation_names(cls, value: object) -> object:
        """Keep the actionable unknown-operation diagnostic before Literal parsing."""

        if isinstance(value, (list, tuple)):
            unknown = sorted(set(str(item) for item in value) - PROMOTION_OPERATIONS)
            if unknown:
                raise ValueError(f"unknown promotion operations: {', '.join(unknown)}")
        return value

    @model_validator(mode="after")
    def validate_promotion_contract(self) -> HorizontalTierBinding:
        unknown = sorted(set(self.required_operations) - PROMOTION_OPERATIONS)
        if unknown:
            raise ValueError(f"unknown promotion operations: {', '.join(unknown)}")
        if self.profile_id is None and self.promotion_status not in {"planned", "not_applicable"}:
            raise ValueError("a null profile must be planned or not_applicable")
        if self.promotion_status == "qualified" and self.blockers:
            raise ValueError("a qualified tier cannot declare promotion blockers")
        return self
    ####


class HorizontalTierDataEvidence(BaseModel):
    """Checked-in model-data evidence for a tier without legacy vehicle metadata.

    Some providers are derived or composite models rather than source-family
    packages or entries in ``vehicle_models.yaml``.  They still need an exact,
    inspectable data basis before a public Composition endpoint is treated as
    ready for runtime probes.  This record deliberately stops at data
    availability; it cannot promote a vehicle, controller, or mission.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    status: DataEvidenceStatus
    paths: tuple[str, ...] = Field(min_length=1)
    sha256: dict[str, str]
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_paths(self) -> HorizontalTierDataEvidence:
        if len(self.paths) != len(set(self.paths)):
            raise ValueError("tier data-evidence paths must be unique")
        if any(not path or Path(path).is_absolute() for path in self.paths):
            raise ValueError("tier data-evidence paths must be non-empty repository-relative paths")
        if set(self.sha256) != set(self.paths):
            raise ValueError("tier data-evidence hashes must cover exactly the declared paths")
        if any(len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest) for digest in self.sha256.values()):
            raise ValueError("tier data-evidence hashes must be lowercase SHA-256 values")
        return self
        ####
    ####


class HorizontalFamilyManifest(BaseModel):
    """Family overlay and complete four-tier declaration."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    family_id: str = Field(min_length=1)
    physical_family: str = Field(min_length=1)
    strategy_id: str = Field(min_length=1)
    mission_overlay: str = Field(min_length=1)
    adapter_id: str = Field(min_length=1)
    automatic_lowering: bool
    vehicle_registry_id: str | None = None
    source_manifest: str | None = None
    source_family_id: str | None = None
    data_evidence: dict[FidelityTier, HorizontalTierDataEvidence] = Field(default_factory=dict)
    tiers: dict[FidelityTier, HorizontalTierBinding]

    @model_validator(mode="after")
    def validate_tier_slots(self) -> HorizontalFamilyManifest:
        expected = set(CANONICAL_FIDELITY_TIERS)
        actual = set(self.tiers)
        if actual != expected:
            raise ValueError(f"{self.family_id} must declare exactly the canonical fidelity tiers")
        return self
        ####
    ####


class HorizontalFidelityRegistry(BaseModel):
    """Versioned horizontal vehicle integration registry."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(alias="schema", min_length=1)
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    families: tuple[HorizontalFamilyManifest, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_families(self) -> HorizontalFidelityRegistry:
        ids = [item.family_id for item in self.families]
        if len(ids) != len(set(ids)):
            raise ValueError("horizontal family IDs must be unique")
        return self
        ####
    ####


@dataclass(frozen=True, slots=True)
class HorizontalConformanceFinding:
    """One actionable cross-family conformance finding."""

    severity: str
    code: str
    path: str
    message: str

    def as_dict(self) -> dict[str, str]:
        return {
            "severity": self.severity,
            "code": self.code,
            "path": self.path,
            "message": self.message,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class HorizontalFamilyConformance:
    """Validated result for one family and its automatic-lowering probe."""

    family_id: str
    physical_family: str
    adapter_id: str
    tiers: Mapping[str, str | None]
    promotion: Mapping[str, Mapping[str, object]]
    lowering: AutomaticLoweringReport

    def as_dict(self) -> dict[str, object]:
        return {
            "family_id": self.family_id,
            "physical_family": self.physical_family,
            "adapter_id": self.adapter_id,
            "tiers": dict(self.tiers),
            "promotion": {key: dict(value) for key, value in self.promotion.items()},
            "lowering": self.lowering.as_dict(),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class HorizontalConformanceReport:
    """Complete machine-readable horizontal integration result."""

    registry: str
    catalog: str
    status: str
    family_count: int
    families: tuple[HorizontalFamilyConformance, ...]
    findings: tuple[HorizontalConformanceFinding, ...]

    @property
    def errors(self) -> tuple[HorizontalConformanceFinding, ...]:
        return tuple(item for item in self.findings if item.severity == "error")
        ####

    def as_dict(self) -> dict[str, object]:
        return {
            "schema": "taoryx.horizontal-fidelity-conformance/v1alpha1",
            "registry": self.registry,
            "catalog": self.catalog,
            "status": self.status,
            "family_count": self.family_count,
            "families": [item.as_dict() for item in self.families],
            "findings": [item.as_dict() for item in self.findings],
            "error_count": len(self.errors),
        }
        ####
    ####


def load_horizontal_registry(path: str | Path | None = None) -> HorizontalFidelityRegistry:
    """Load and validate the canonical horizontal registry."""

    registry_path = Path(path) if path is not None else HORIZONTAL_REGISTRY
    payload = yaml.safe_load(registry_path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{registry_path} must contain a mapping")
    return HorizontalFidelityRegistry.model_validate(payload)
    ####


def _finding(
    findings: list[HorizontalConformanceFinding],
    severity: str,
    code: str,
    path: str,
    message: str,
) -> None:
    findings.append(HorizontalConformanceFinding(severity, code, path, message))
    ####


def validate_horizontal_fidelity(
    registry: HorizontalFidelityRegistry | None = None,
    *,
    catalog: Pseudo6DOFCatalog | None = None,
    evidence: Mapping[str, Mapping[str, object]] | None = None,
    adapter_operation_status: Mapping[str, Mapping[FidelityTier, Mapping[str, str]]] | None = None,
) -> HorizontalConformanceReport:
    """Validate all family tier slots against the canonical catalog.

    The surface tier is probed as a requested target so an unqualified surface
    path visibly lowers to the best checked parent.  This does not promote a
    development profile or imply a physical-effector result.
    """

    resolved_registry = registry or load_horizontal_registry()
    resolved_catalog = catalog or load_pseudo6dof_catalog()
    records = evidence if evidence is not None else load_qualified_fidelity_evidence()
    findings: list[HorizontalConformanceFinding] = []
    catalog_bindings = {item.family_id: item for item in resolved_catalog.bindings}
    registry_families = {item.family_id: item for item in resolved_registry.families}
    catalog_ids = set(catalog_bindings)
    registry_ids = set(registry_families)
    for family_id in sorted(catalog_ids - registry_ids):
        _finding(findings, "error", "family-missing", family_id, "catalog family has no horizontal registry entry")
    for family_id in sorted(registry_ids - catalog_ids):
        _finding(findings, "error", "family-unknown", family_id, "horizontal registry family is absent from the canonical catalog")

    results: list[HorizontalFamilyConformance] = []
    catalog_profiles: dict[str, Any] = {
        str(getattr(profile, "id")): profile
        for profile in (*resolved_catalog.profiles, *resolved_catalog.direct_wrench_profiles, *resolved_catalog.surface_allocation_profiles)
    }
    catalog_point_ids = {item.point_mass_profile_id: item.family_id for item in resolved_catalog.bindings}
    for family in resolved_registry.families:
        binding = catalog_bindings.get(family.family_id)
        if binding is None:
            continue
        expected = {
            "point_mass_3dof": binding.point_mass_profile_id,
            "pseudo_6dof": binding.pseudo_profile_id,
            "rigid_body_6dof_direct_wrench": binding.direct_wrench_profile_id,
            "rigid_body_6dof_surface_allocated": binding.surface_allocation_profile_id,
        }
        actual: dict[str, str | None] = {tier: family.tiers[tier].profile_id for tier in CANONICAL_FIDELITY_TIERS}
        for tier in CANONICAL_FIDELITY_TIERS:
            path = f"{family.family_id}.tiers.{tier}"
            if actual[tier] != expected[tier]:
                _finding(findings, "error", "profile-binding-mismatch", path, f"registry profile {actual[tier]!r} does not match catalog profile {expected[tier]!r}")
            profile_id = actual[tier]
            if profile_id is not None:
                profile = catalog_profiles.get(profile_id)
                if profile is None and tier == "point_mass_3dof" and catalog_point_ids.get(profile_id) == family.family_id:
                    profile = None
                elif profile is None:
                    _finding(findings, "error", "profile-unknown", path, f"profile {profile_id!r} is not present in the canonical catalog")
                elif getattr(profile, "family_id", None) != family.family_id:
                    _finding(findings, "error", "profile-family-mismatch", path, f"profile {profile_id!r} belongs to {getattr(profile, 'family_id', None)!r}")
            elif tier in {"point_mass_3dof", "pseudo_6dof"}:
                _finding(findings, "error", "required-tier-missing", path, "point-mass and pseudo-6DOF tiers may not be null")
        if family.automatic_lowering != binding.automatic_lowering:
            _finding(findings, "error", "lowering-policy-mismatch", family.family_id, "registry and catalog automatic-lowering policy disagree")
        lowering = build_automatic_lowering_report(
            family.family_id,
            "rigid_body_6dof_surface_allocated",
            records,
            catalog=resolved_catalog,
            required_operations={
                tier: family.tiers[tier].required_operations
                for tier in CANONICAL_FIDELITY_TIERS
            },
            operation_status=(adapter_operation_status or {}).get(family.family_id),
        )
        promotion: dict[str, Mapping[str, object]] = {
            tier: {
                "declared_status": family.tiers[tier].promotion_status,
                "required_operations": list(family.tiers[tier].required_operations),
                "blockers": list(family.tiers[tier].blockers),
            }
            for tier in CANONICAL_FIDELITY_TIERS
        }
        results.append(HorizontalFamilyConformance(family.family_id, family.physical_family, family.adapter_id, actual, promotion, lowering))
    status = "pass" if not findings else "fail"
    return HorizontalConformanceReport(
        str(HORIZONTAL_REGISTRY.relative_to(ROOT)),
        str((ROOT / "verification/pseudo6dof_profiles.yaml").relative_to(ROOT)),
        status,
        len(results),
        tuple(results),
        tuple(findings),
    )
    ####


__all__ = [
    "HORIZONTAL_REGISTRY",
    "HorizontalConformanceFinding",
    "HorizontalConformanceReport",
    "HorizontalFamilyConformance",
    "HorizontalFamilyManifest",
    "HorizontalFidelityRegistry",
    "HorizontalTierBinding",
    "PROMOTION_OPERATIONS",
    "PromotionStatus",
    "load_horizontal_registry",
    "validate_horizontal_fidelity",
]
