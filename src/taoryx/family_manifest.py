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
from .vehicle_catalog_resources import vehicle_catalog_resources, vehicle_catalog_root

if TYPE_CHECKING:
    from .plugins.discovery import PluginCatalog
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
            "data_evidence": {tier: evidence.model_dump(mode="json") for tier, evidence in self.family.data_evidence.items()},
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
            "findings": [{"family_id": item.family_id, "code": item.code, "message": item.message} for item in self.findings],
        }
        ####

    ####


def _load_vehicle_definitions(
    path: str | Path | None = None,
    *,
    plugins: PluginCatalog | None = None,
) -> Mapping[str, Mapping[str, Any]]:
    import yaml

    sources = (
        (Path(path),)
        if path is not None
        else vehicle_catalog_resources("verification/vehicle_models.yaml", plugins=plugins)
    )
    result: dict[str, Mapping[str, Any]] = {}
    for source in sources:
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
        vehicles = payload.get("vehicles") if isinstance(payload, Mapping) else None
        if not isinstance(vehicles, Mapping):
            raise ValueError(f"{source} must contain a vehicles mapping")
        for key, value in vehicles.items():
            if not isinstance(value, Mapping):
                continue
            identifier = str(key)
            if identifier in result:
                raise ValueError(f"vehicle definition {identifier!r} is owned by more than one catalog fragment")
            result[identifier] = value
    return result
    ####


def _profile_maps(catalog: Pseudo6DOFCatalog) -> tuple[dict[str, Pseudo6DOFProfile], dict[str, DirectWrenchProfile], dict[str, SurfaceAllocationProfile]]:
    return (
        {profile.id: profile for profile in catalog.profiles},
        {profile.id: profile for profile in catalog.direct_wrench_profiles},
        {profile.id: profile for profile in catalog.surface_allocation_profiles},
    )
    ####


def _resource_root_for_relative_path(
    relative_path: str,
    *,
    fallback: Path,
    default_resources: bool,
    plugins: PluginCatalog | None = None,
) -> Path:
    """Return the fragment root owning one family-specific package asset.

    Aggregate catalog joins deliberately combine rows from independent vehicle
    wheels.  A source-manifest or data-evidence path is therefore resolved
    through the same fragment resolver rather than assumed to sit beside the
    first aggregate registry fragment.  An explicit ``root`` remains an
    isolated-provider boundary and bypasses that cross-package lookup.
    """

    if not default_resources:
        return fallback
    relative = Path(relative_path)
    for candidate in vehicle_catalog_resources(relative_path, plugins=plugins):
        if candidate.is_file():
            return candidate.parents[len(relative.parts) - 1]
    return fallback
    ####


def load_unified_family_manifest_catalog(
    *,
    horizontal: HorizontalFidelityRegistry | None = None,
    pseudo: Pseudo6DOFCatalog | None = None,
    root: Path | None = None,
    plugins: PluginCatalog | None = None,
    validate_source_imports: bool = True,
) -> UnifiedFamilyManifestCatalog:
    """Resolve all family/tier bindings through one typed join.

    Set ``validate_source_imports`` to ``False`` for metadata-only discovery.
    That still validates the typed source-family manifest and its declared
    identity, while deferring the expensive DAVE-ML import-sidecar cross-check
    to an explicit provenance or source-data operation.
    """

    default_resources = root is None
    resource_root = root or vehicle_catalog_root(plugins=plugins)
    horizontal_registry = horizontal or load_horizontal_registry(plugins=plugins)
    pseudo_catalog = pseudo or load_pseudo6dof_catalog(plugins=plugins)
    pseudo_map, direct_map, surface_map = _profile_maps(pseudo_catalog)
    vehicle_definitions = _load_vehicle_definitions(
        None if default_resources else resource_root / "verification/vehicle_models.yaml",
        plugins=plugins,
    )
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
            findings.append(
                UnifiedFamilyManifestFinding(
                    family.family_id,
                    "tier-profile-mismatch",
                    f"horizontal tier IDs {actual_profile_ids!r} do not match canonical catalog {expected_profile_ids!r}",
                )
            )
        if binding.automatic_lowering != family.automatic_lowering:
            findings.append(
                UnifiedFamilyManifestFinding(family.family_id, "lowering-policy-mismatch", "horizontal and pseudo catalogs disagree on automatic lowering")
            )
        vehicle_definition = None
        if family.vehicle_registry_id is not None:
            vehicle_definition = vehicle_definitions.get(family.vehicle_registry_id)
            if vehicle_definition is None:
                findings.append(
                    UnifiedFamilyManifestFinding(
                        family.family_id, "vehicle-definition-missing", f"vehicle registry ID {family.vehicle_registry_id!r} is not present"
                    )
                )
        source_manifest = None
        source_manifest_path = family.source_manifest
        family_resource_root = resource_root
        if source_manifest_path is not None:
            family_resource_root = _resource_root_for_relative_path(
                source_manifest_path,
                fallback=resource_root,
                default_resources=default_resources,
                plugins=plugins,
            )
            path = family_resource_root / source_manifest_path
            if not path.is_file():
                findings.append(
                    UnifiedFamilyManifestFinding(family.family_id, "source-manifest-missing", f"source manifest {source_manifest_path!r} does not exist")
                )
            else:
                try:
                    # Reference-family parsing is an optional model-package
                    # concern.  A standalone family such as Hummingbird has
                    # no source manifest and must not import that package.
                    from .trajectory.reference_families import load_reference_family_manifest

                    source_manifest = load_reference_family_manifest(
                        path,
                        validate_daveml_import=validate_source_imports,
                    )
                    if family.source_family_id is not None and source_manifest.family_id != family.source_family_id:
                        findings.append(
                            UnifiedFamilyManifestFinding(
                                family.family_id,
                                "source-family-id-mismatch",
                                f"source manifest declares {source_manifest.family_id!r}, expected {family.source_family_id!r}",
                            )
                        )
                except (OSError, ValueError) as error:
                    findings.append(UnifiedFamilyManifestFinding(family.family_id, "source-manifest-invalid", str(error)))
        for tier, evidence in family.data_evidence.items():
            for evidence_path in evidence.paths:
                evidence_root = _resource_root_for_relative_path(
                    evidence_path,
                    fallback=family_resource_root,
                    default_resources=default_resources,
                    plugins=plugins,
                )
                path = evidence_root / evidence_path
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
