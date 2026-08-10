"""Joined readiness for every family and canonical fidelity tier.

The repository intentionally keeps several authorities separate: vehicle
metadata, source-family manifests, the four-tier horizontal registry, and
executable adapter probes. This module is the typed join used before a
simulation or showcase is selected. It reports missing evidence explicitly;
it never turns a declared profile into a qualified runtime path.
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .family_manifest import UnifiedFamilyManifest, load_unified_family_manifest_catalog
from .fidelity_contracts import CANONICAL_FIDELITY_TIERS, FidelityTier
from .fidelity_readiness import validate_fidelity_readiness
from .vehicle_registry import ROOT

ADAPTER_ARTIFACT = ROOT / "verification/alpha3_horizontal_fidelity/adapter_registry.json"
READINESS_ARTIFACT = ROOT / "verification/alpha3_horizontal_fidelity/readiness.json"


@dataclass(frozen=True, slots=True)
class HorizontalTierReadiness:
    """One family/tier readiness record."""

    family_id: str
    tier: FidelityTier
    profile_id: str | None
    declared_status: str
    data_status: str
    data_source: str
    data_claim_boundary: str | None
    adapter_status: str
    required_operations: tuple[str, ...]
    operation_status: Mapping[str, str]
    blockers: tuple[str, ...]

    @property
    def status(self) -> str:
        """Return the strict joined status without promoting evidence."""

        if self.declared_status == "not_applicable":
            return "not_applicable"
        if self.declared_status == "planned":
            return "planned"
        if self.data_status in {"blocked", "not_registered"} or self.adapter_status == "blocked":
            return "blocked"
        if self.adapter_status == "not_checked":
            return "data_ready" if self.data_status in {"ready", "source_declared"} else self.data_status
        if self.required_operations and any(
            self.operation_status.get(operation) not in {"pass", "available"}
            for operation in self.required_operations
        ):
            return "blocked"
        if self.adapter_status == "pass" and self.data_status in {"ready", "source_declared"}:
            return "probe_ready"
        return "development"
        ####

    def as_dict(self) -> dict[str, object]:
        """Return stable JSON for the generated readiness artifact."""

        return {
            "family_id": self.family_id,
            "tier": self.tier,
            "profile_id": self.profile_id,
            "declared_status": self.declared_status,
            "status": self.status,
            "data_status": self.data_status,
            "data_source": self.data_source,
            "data_claim_boundary": self.data_claim_boundary,
            "adapter_status": self.adapter_status,
            "required_operations": list(self.required_operations),
            "operation_status": dict(self.operation_status),
            "blockers": list(self.blockers),
        }
        ####


@dataclass(frozen=True, slots=True)
class HorizontalReadinessReport:
    """Complete joined readiness matrix."""

    status: str
    family_count: int
    tier_count: int
    records: tuple[HorizontalTierReadiness, ...]
    findings: tuple[str, ...] = ()

    @property
    def blockers(self) -> tuple[HorizontalTierReadiness, ...]:
        """Return records that cannot currently enter their declared tier."""

        return tuple(item for item in self.records if item.status == "blocked")
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a machine-readable report with summary counts."""

        counts: dict[str, int] = {}
        for record in self.records:
            counts[record.status] = counts.get(record.status, 0) + 1
        return {
            "schema": "taoryx.horizontal-readiness/v1alpha1",
            "status": self.status,
            "family_count": self.family_count,
            "tier_count": self.tier_count,
            "status_counts": counts,
            "records": [item.as_dict() for item in self.records],
            "findings": list(self.findings),
            "claim_boundary": (
                "Readiness and adapter-probe evidence only; this artifact is not "
                "family mission qualification."
            ),
        }
        ####


def _profile_id(family: UnifiedFamilyManifest, tier: FidelityTier) -> str | None:
    binding = family.binding
    return {
        "point_mass_3dof": binding.point_mass_profile_id,
        "pseudo_6dof": binding.pseudo_profile_id,
        "rigid_body_6dof_direct_wrench": binding.direct_wrench_profile_id,
        "rigid_body_6dof_surface_allocated": binding.surface_allocation_profile_id,
    }[tier]
    ####


@dataclass(frozen=True, slots=True)
class HorizontalShowcasePreflight:
    """Preflight result for one family/tier showcase binding."""

    family_id: str
    tier: FidelityTier
    allowed: bool
    readiness_status: str
    blockers: tuple[str, ...]
    advisories: tuple[str, ...]
    required_operations: tuple[str, ...]
    operation_status: Mapping[str, str]

    def as_dict(self) -> dict[str, object]:
        """Return a stable preflight artifact."""

        return {
            "family_id": self.family_id,
            "tier": self.tier,
            "allowed": self.allowed,
            "readiness_status": self.readiness_status,
            "blockers": list(self.blockers),
            "advisories": list(self.advisories),
            "required_operations": list(self.required_operations),
            "operation_status": dict(self.operation_status),
            "claim_boundary": (
                "Showcase assembly preflight only; this does not promote "
                "the family, mission, or source evidence."
            ),
        }
        ####
    ####


def _source_profile(family: UnifiedFamilyManifest, tier: FidelityTier) -> Any | None:
    """Find a source-manifest profile matching the canonical tier."""

    if family.source_manifest is None:
        return None
    realization = {
        "point_mass_3dof": "force_model",
        "pseudo_6dof": "response_law",
        "rigid_body_6dof_direct_wrench": "direct_wrench",
        "rigid_body_6dof_surface_allocated": "surface_allocated",
    }[tier]
    runtime = "rigid_body_6dof" if tier.startswith("rigid_body") else tier
    matches = [
        item
        for item in family.source_manifest.fidelity_profiles
        if str(item.runtime_fidelity) == runtime and item.control_realization == realization
    ]
    if matches:
        return matches[0]
    if tier == "rigid_body_6dof_direct_wrench":
        return next(
            (item for item in family.source_manifest.fidelity_profiles if str(item.runtime_fidelity) == runtime),
            None,
        )
    return None
    ####


def _adapter_records(path: Path) -> dict[tuple[str, FidelityTier], Mapping[str, Any]]:
    if not path.is_file():
        return {}
    payload = json.loads(path.read_text(encoding="utf-8"))
    matrix = payload.get("tier_matrix", {}).get("checks", [])
    result: dict[tuple[str, FidelityTier], Mapping[str, Any]] = {}
    if not isinstance(matrix, list):
        return result
    for item in matrix:
        if not isinstance(item, Mapping):
            continue
        family_id = item.get("family_id")
        tier = item.get("tier")
        if isinstance(family_id, str) and tier in CANONICAL_FIDELITY_TIERS:
            result[(family_id, tier)] = item
    return result
    ####


def _data_status(family: UnifiedFamilyManifest, tier: FidelityTier) -> tuple[str, str, str | None, tuple[str, ...]]:
    """Join metadata, source-family, or declared tier evidence without guessing."""

    if family.family.vehicle_registry_id is not None and family.vehicle_definition is not None:
        report = validate_fidelity_readiness(family.family.vehicle_registry_id, tier)
        blockers = tuple(f"{item.requirement_id}: {item.message}" for item in report.errors)
        return (
            report.status,
            "verification/vehicle_models.yaml",
            "Vehicle-registry data completeness only; this does not establish runtime, control, or mission qualification.",
            blockers,
        )
    source_profile = _source_profile(family, tier)
    if source_profile is not None:
        blockers = tuple(f"omitted:{item}" for item in source_profile.omitted_physics)
        return (
            "source_declared",
            str(family.source_manifest_path),
            "Source-family profile declaration only; omitted physics and separate runtime/qualification gates remain in force.",
            blockers,
        )
    evidence = family.family.data_evidence.get(tier)
    if evidence is not None:
        missing = tuple(path for path in evidence.paths if not (ROOT / path).is_file())
        if missing:
            return (
                "blocked",
                ", ".join(evidence.paths),
                evidence.claim_boundary,
                tuple(f"tier-data-evidence-missing:{path}" for path in missing),
            )
        return evidence.status, ", ".join(evidence.paths), evidence.claim_boundary, ()
    if family.family_id == "tumbling_body" and tier in {"point_mass_3dof", "pseudo_6dof"}:
        return (
            "source_declared",
            "verification/pseudo6dof_profiles.yaml",
            "Declared passive-body profile only; this is not a controlled or actuator-backed path.",
            (),
        )
    return "not_registered", "", None, ("no vehicle metadata, source-family profile, or tier data evidence is bound",)
    ####


def build_horizontal_readiness_report(
    *,
    adapter_artifact: str | Path | None = None,
) -> HorizontalReadinessReport:
    """Build the joined nine-family, four-tier readiness matrix."""

    catalog = load_unified_family_manifest_catalog()
    adapter_path = Path(adapter_artifact) if adapter_artifact is not None else ADAPTER_ARTIFACT
    adapter_map = _adapter_records(adapter_path)
    findings = tuple(f"{item.family_id}: {item.code}: {item.message}" for item in catalog.findings)
    records: list[HorizontalTierReadiness] = []
    for family in catalog.families:
        for tier in CANONICAL_FIDELITY_TIERS:
            binding = family.family.tiers[tier]
            data_status, data_source, data_claim_boundary, data_blockers = _data_status(family, tier)
            adapter = adapter_map.get((family.family_id, tier))
            adapter_status = str(adapter.get("status", "not_checked")) if adapter is not None else "not_checked"
            probe = adapter.get("probe") if adapter is not None else None
            operation_status: dict[str, str] = {}
            if isinstance(probe, Mapping) and isinstance(probe.get("operations"), list):
                operation_status = {
                    str(item["operation"]): str(item["status"])
                    for item in probe["operations"]
                    if isinstance(item, Mapping) and "operation" in item and "status" in item
                }
            blockers = tuple(binding.blockers) + data_blockers
            if adapter is None and binding.promotion_status not in {"planned", "not_applicable"}:
                blockers += ("adapter_probe_artifact_not_found",)
            records.append(
                HorizontalTierReadiness(
                    family.family_id,
                    tier,
                    _profile_id(family, tier),
                    binding.promotion_status,
                    data_status,
                    data_source,
                    data_claim_boundary,
                    adapter_status,
                    tuple(binding.required_operations),
                    operation_status,
                    blockers,
                )
            )
    status = "blocked" if findings else "development"
    return HorizontalReadinessReport(status, len(catalog.families), len(records), tuple(records), findings)
    ####


def preflight_horizontal_showcase(
    family_id: str,
    tier: FidelityTier,
    *,
    report: HorizontalReadinessReport | None = None,
    required_operations: tuple[str, ...] = (),
) -> HorizontalShowcasePreflight:
    """Check whether one declared family/tier path may enter a showcase.

    Family-mission qualification is intentionally not a preflight blocker;
    the caller still receives that distinction through ``readiness_status``
    and the advisory list. Hard data, adapter, and requested-operation
    blockers are rejected before a run is started.
    """

    resolved = report if report is not None else build_horizontal_readiness_report()
    record = next(
        (item for item in resolved.records if item.family_id == family_id and item.tier == tier),
        None,
    )
    if record is None:
        return HorizontalShowcasePreflight(
            family_id,
            tier,
            False,
            "missing",
            ("family/tier is absent from the joined readiness matrix",),
            (),
            required_operations,
            {},
        )

    blockers: list[str] = []
    if record.status in {"blocked", "planned", "not_applicable", "not_registered"}:
        blockers.extend(record.blockers or (f"readiness status is {record.status}",))
    if record.data_status in {"blocked", "not_registered"}:
        blockers.append(f"data_status={record.data_status}")
    if record.adapter_status == "blocked":
        blockers.append("adapter_status=blocked")
    for operation in required_operations:
        status = record.operation_status.get(operation, "not_checked")
        if status not in {"pass", "available"}:
            blockers.append(f"operation:{operation}={status}")

    advisories = tuple(record.blockers)
    return HorizontalShowcasePreflight(
        family_id,
        tier,
        not blockers,
        record.status,
        tuple(dict.fromkeys(blockers)),
        advisories,
        required_operations,
        record.operation_status,
    )
    ####


__all__ = [
    "ADAPTER_ARTIFACT",
    "HorizontalReadinessReport",
    "HorizontalShowcasePreflight",
    "HorizontalTierReadiness",
    "READINESS_ARTIFACT",
    "build_horizontal_readiness_report",
    "preflight_horizontal_showcase",
]
