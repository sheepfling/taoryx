"""Unified family-integration manifest resolution.

The repository has several intentionally different authorities: the
horizontal tier overlay, the pseudo/direct/surface profile catalog, legacy
vehicle definitions, and source-grounded family manifests.  This module is
the typed join between them.  It does not copy physics data; it proves that a
family's advertised identity and four tier slots resolve consistently before
runtime or automatic lowering is invoked.
"""

from __future__ import annotations

import hashlib
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

from .fidelity_contracts import FidelityTier
from .horizontal_fidelity import HorizontalFamilyManifest, HorizontalFidelityRegistry, load_horizontal_registry
from .trajectory.pseudo6dof_profiles import (
    DirectWrenchProfile,
    FidelityBinding,
    Pseudo6DOFCatalog,
    Pseudo6DOFProfile,
    SurfaceAllocationProfile,
    load_pseudo6dof_catalog,
)
from .vehicle_registry import REGISTRY, ROOT

if TYPE_CHECKING:
    from .trajectory.reference_families import ReferenceFamilyManifest


@dataclass(frozen=True, slots=True)
class UnifiedFamilyManifest:
    """One resolved family record across integration authorities."""

    family: HorizontalFamilyManifest
    binding: FidelityBinding
    pseudo_profile: Pseudo6DOFProfile
    direct_profile: DirectWrenchProfile | None
    surface_profile: SurfaceAllocationProfile | None
    vehicle_definition: Mapping[str, Any] | None
    source_manifest: ReferenceFamilyManifest | None
    source_manifest_path: str | None

    @property
    def family_id(self) -> str:
        """Return the stable horizontal family identifier."""

        return self.family.family_id
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a compact machine-readable resolved manifest."""

        return {
            "family_id": self.family_id,
            "physical_family": self.family.physical_family,
            "strategy_id": self.family.strategy_id,
            "mission_overlay": self.family.mission_overlay,
            "adapter_id": self.family.adapter_id,
            "automatic_lowering": self.family.automatic_lowering,
            "vehicle_registry_id": self.family.vehicle_registry_id,
            "source_manifest": self.source_manifest_path,
            "source_family_id": self.family.source_family_id,
            "resolved_source_family_id": self.source_manifest.family_id if self.source_manifest is not None else None,
            "data_evidence": {
                tier: evidence.model_dump(mode="json")
                for tier, evidence in self.family.data_evidence.items()
            },
            "tiers": {
                "point_mass_3dof": self.binding.point_mass_profile_id,
                "pseudo_6dof": self.binding.pseudo_profile_id,
                "rigid_body_6dof_direct_wrench": self.binding.direct_wrench_profile_id,
                "rigid_body_6dof_surface_allocated": self.binding.surface_allocation_profile_id,
            },
            "source_manifest_profile_count": len(self.source_manifest.fidelity_profiles) if self.source_manifest is not None else 0,
            "vehicle_definition_present": self.vehicle_definition is not None,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class UnifiedFamilyManifestFinding:
    """One issue found while joining family authorities."""

    family_id: str
    code: str
    message: str


@dataclass(frozen=True, slots=True)
class UnifiedFamilyManifestCatalog:
    """Resolved nine-family integration catalog and join findings."""

    families: tuple[UnifiedFamilyManifest, ...]
    findings: tuple[UnifiedFamilyManifestFinding, ...]

    @property
    def errors(self) -> tuple[UnifiedFamilyManifestFinding, ...]:
        """Return join failures."""

        return self.findings
        ####

    def as_dict(self) -> dict[str, object]:
        """Return stable JSON for validation artifacts."""

        return {
            "schema": "taoryx.unified-family-manifest/v1alpha1",
            "status": "pass" if not self.errors else "fail",
            "family_count": len(self.families),
            "families": [family.as_dict() for family in self.families],
            "findings": [
                {"family_id": item.family_id, "code": item.code, "message": item.message}
                for item in self.findings
            ],
        }
        ####
    ####


def _load_vehicle_definitions(path: str | Path = REGISTRY) -> Mapping[str, Mapping[str, Any]]:
    import yaml

    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    vehicles = payload.get("vehicles") if isinstance(payload, Mapping) else None
    if not isinstance(vehicles, Mapping):
        raise ValueError(f"{path} must contain a vehicles mapping")
    return {
        str(key): value
        for key, value in vehicles.items()
        if isinstance(value, Mapping)
    }
    ####


def _profile_maps(catalog: Pseudo6DOFCatalog) -> tuple[dict[str, Pseudo6DOFProfile], dict[str, DirectWrenchProfile], dict[str, SurfaceAllocationProfile]]:
    return (
        {profile.id: profile for profile in catalog.profiles},
        {profile.id: profile for profile in catalog.direct_wrench_profiles},
        {profile.id: profile for profile in catalog.surface_allocation_profiles},
    )
    ####


def load_unified_family_manifest_catalog(
    *,
    horizontal: HorizontalFidelityRegistry | None = None,
    pseudo: Pseudo6DOFCatalog | None = None,
    root: Path = ROOT,
) -> UnifiedFamilyManifestCatalog:
    """Resolve all family/tier bindings through one typed join."""

    from .trajectory.reference_families import load_reference_family_manifest

    horizontal_registry = horizontal or load_horizontal_registry()
    pseudo_catalog = pseudo or load_pseudo6dof_catalog()
    pseudo_map, direct_map, surface_map = _profile_maps(pseudo_catalog)
    vehicle_definitions = _load_vehicle_definitions(root / "verification/vehicle_models.yaml")
    findings: list[UnifiedFamilyManifestFinding] = []
    resolved: list[UnifiedFamilyManifest] = []
    pseudo_bindings = {binding.family_id: binding for binding in pseudo_catalog.bindings}

    for family in horizontal_registry.families:
        binding = pseudo_bindings.get(family.family_id)
        if binding is None:
            findings.append(UnifiedFamilyManifestFinding(family.family_id, "pseudo-binding-missing", "family has no pseudo/direct/surface catalog binding"))
            continue
        pseudo_profile = pseudo_map.get(binding.pseudo_profile_id)
        if pseudo_profile is None:
            findings.append(UnifiedFamilyManifestFinding(family.family_id, "pseudo-profile-missing", f"profile {binding.pseudo_profile_id!r} is not present"))
            continue
        direct_profile = direct_map.get(binding.direct_wrench_profile_id) if binding.direct_wrench_profile_id is not None else None
        surface_profile = surface_map.get(binding.surface_allocation_profile_id) if binding.surface_allocation_profile_id is not None else None
        expected_profile_ids: dict[FidelityTier, str | None] = {
            "point_mass_3dof": binding.point_mass_profile_id,
            "pseudo_6dof": binding.pseudo_profile_id,
            "rigid_body_6dof_direct_wrench": binding.direct_wrench_profile_id,
            "rigid_body_6dof_surface_allocated": binding.surface_allocation_profile_id,
        }
        actual_profile_ids: dict[FidelityTier, str | None] = {tier: family.tiers[tier].profile_id for tier in expected_profile_ids}
        if actual_profile_ids != expected_profile_ids:
            findings.append(UnifiedFamilyManifestFinding(family.family_id, "tier-profile-mismatch", f"horizontal tier IDs {actual_profile_ids!r} do not match canonical catalog {expected_profile_ids!r}"))
        if binding.automatic_lowering != family.automatic_lowering:
            findings.append(UnifiedFamilyManifestFinding(family.family_id, "lowering-policy-mismatch", "horizontal and pseudo catalogs disagree on automatic lowering"))
        vehicle_definition = None
        if family.vehicle_registry_id is not None:
            vehicle_definition = vehicle_definitions.get(family.vehicle_registry_id)
            if vehicle_definition is None:
                findings.append(UnifiedFamilyManifestFinding(family.family_id, "vehicle-definition-missing", f"vehicle registry ID {family.vehicle_registry_id!r} is not present"))
        source_manifest = None
        source_manifest_path = family.source_manifest
        if source_manifest_path is not None:
            path = root / source_manifest_path
            if not path.is_file():
                findings.append(UnifiedFamilyManifestFinding(family.family_id, "source-manifest-missing", f"source manifest {source_manifest_path!r} does not exist"))
            else:
                try:
                    source_manifest = load_reference_family_manifest(path)
                    if family.source_family_id is not None and source_manifest.family_id != family.source_family_id:
                        findings.append(UnifiedFamilyManifestFinding(family.family_id, "source-family-id-mismatch", f"source manifest declares {source_manifest.family_id!r}, expected {family.source_family_id!r}"))
                except (OSError, ValueError) as error:
                    findings.append(UnifiedFamilyManifestFinding(family.family_id, "source-manifest-invalid", str(error)))
        for tier, evidence in family.data_evidence.items():
            for evidence_path in evidence.paths:
                path = root / evidence_path
                if not path.is_file():
                    findings.append(
                        UnifiedFamilyManifestFinding(
                            family.family_id,
                            "tier-data-evidence-missing",
                            f"{tier} data evidence {evidence_path!r} does not exist",
                        )
                    )
                    continue
                actual_sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
                if actual_sha256 != evidence.sha256[evidence_path]:
                    findings.append(
                        UnifiedFamilyManifestFinding(
                            family.family_id,
                            "tier-data-evidence-hash-mismatch",
                            f"{tier} data evidence {evidence_path!r} SHA-256 does not match its declaration",
                        )
                    )
        resolved.append(
            UnifiedFamilyManifest(
                family,
                binding,
                pseudo_profile,
                direct_profile,
                surface_profile,
                vehicle_definition,
                source_manifest,
                source_manifest_path,
            )
        )
    return UnifiedFamilyManifestCatalog(tuple(resolved), tuple(findings))
    ####


__all__ = [
    "UnifiedFamilyManifest",
    "UnifiedFamilyManifestCatalog",
    "UnifiedFamilyManifestFinding",
    "load_unified_family_manifest_catalog",
]
