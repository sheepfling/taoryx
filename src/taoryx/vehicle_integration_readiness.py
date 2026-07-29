"""Provider-neutral readiness reports for onboarding source vehicle families.

This layer is intentionally separate from runtime qualification.  It answers
whether a family manifest declares the inputs needed to begin each fidelity
workflow and identifies the first missing declaration or artifact.  It does
not infer physical validity from a profile status or replace family-specific
trim, allocation, or mission evidence.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .vehicle_registry import ROOT

SUPPORTED_FAMILIES = ROOT / "verification/supported_reference_families.yaml"
FAMILY_ROOT = ROOT / "families"
STANDARD_PROFILES: tuple[tuple[str, str], ...] = (
    ("point_mass_3dof", "point_mass_3dof"),
    ("pseudo_6dof", "pseudo_6dof"),
    ("rigid_body_6dof_direct_wrench", "rigid_body_6dof"),
    ("rigid_body_6dof_surface_allocated", "rigid_body_6dof"),
)
QUALIFIED_STATUSES = frozenset(
    {"equivalence_passed", "multi_fidelity_qualified", "qualified", "runtime_replay_qualification_passed"}
)


@dataclass(frozen=True, slots=True)
class IntegrationReadinessFinding:
    """One actionable source-family readiness finding."""

    severity: str
    code: str
    path: str
    message: str
    hint: str

    def as_dict(self) -> dict[str, str]:
        """Return a stable JSON-compatible finding."""

        return {
            "severity": self.severity,
            "code": self.code,
            "path": self.path,
            "message": self.message,
            "hint": self.hint,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class FidelityProfileReadiness:
    """Readiness of one declared or expected fidelity profile."""

    profile_id: str
    runtime_fidelity: str
    declared: bool
    declared_status: str
    metadata_ready: bool
    automatically_lowerable: bool
    findings: tuple[IntegrationReadinessFinding, ...]

    @property
    def errors(self) -> tuple[IntegrationReadinessFinding, ...]:
        """Return profile-level blocking findings."""

        return tuple(item for item in self.findings if item.severity == "error")
        ####

    def as_dict(self) -> dict[str, Any]:
        """Return a stable JSON-compatible profile report."""

        return {
            "profile_id": self.profile_id,
            "runtime_fidelity": self.runtime_fidelity,
            "declared": self.declared,
            "declared_status": self.declared_status,
            "metadata_ready": self.metadata_ready,
            "automatically_lowerable": self.automatically_lowerable,
            "error_count": len(self.errors),
            "findings": [item.as_dict() for item in self.findings],
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class VehicleIntegrationReadinessReport:
    """Complete manifest-level readiness report for one source family."""

    family_id: str
    display_name: str
    physical_family: str
    manifest: str
    status: str
    findings: tuple[IntegrationReadinessFinding, ...]
    profiles: tuple[FidelityProfileReadiness, ...]

    @property
    def errors(self) -> tuple[IntegrationReadinessFinding, ...]:
        """Return all blocking findings, including profile gaps."""

        profile_errors = tuple(item for profile in self.profiles for item in profile.errors)
        return tuple(item for item in self.findings if item.severity == "error") + profile_errors
        ####

    def as_dict(self) -> dict[str, Any]:
        """Return a stable machine-readable readiness report."""

        return {
            "schema_version": "taoryx.vehicle-integration-readiness/v1",
            "family_id": self.family_id,
            "display_name": self.display_name,
            "physical_family": self.physical_family,
            "manifest": self.manifest,
            "status": self.status,
            "error_count": len(self.errors),
            "warning_count": sum(item.severity == "warning" for item in self.findings)
            + sum(sum(finding.severity == "warning" for finding in profile.findings) for profile in self.profiles),
            "findings": [item.as_dict() for item in self.findings],
            "profiles": [profile.as_dict() for profile in self.profiles],
        }
        ####
    ####


def _load_yaml(path: Path) -> Mapping[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{path} must contain a mapping")
    return payload
    ####


def _present(value: Any) -> bool:
    """Return whether a manifest value is materially declared."""

    return value is not None and value != "" and value != [] and value != {}
    ####


def _lookup(value: Mapping[str, Any], path: str) -> Any:
    current: Any = value
    for part in path.split("."):
        if not isinstance(current, Mapping) or part not in current:
            return None
        current = current[part]
    return current
    ####


def _finding(
    findings: list[IntegrationReadinessFinding],
    severity: str,
    code: str,
    path: str,
    message: str,
    hint: str,
) -> None:
    findings.append(IntegrationReadinessFinding(severity, code, path, message, hint))
    ####


def _check_manifest(manifest_path: Path, family_id: str, manifest: Mapping[str, Any]) -> list[IntegrationReadinessFinding]:
    findings: list[IntegrationReadinessFinding] = []
    required: tuple[tuple[str, str, str], ...] = (
        ("source.kind", "source-kind-missing", "Declare the source format and provider."),
        ("source.package_sha256", "source-hash-missing", "Pin the immutable source package hash."),
        ("source.availability", "source-availability-missing", "Declare whether the source package is locally verified."),
        ("plant.fidelity", "plant-fidelity-missing", "Declare the parent executable plant fidelity."),
        ("plant.body_frame", "body-frame-missing", "Declare the body-axis frame and handedness."),
        ("plant.navigation_frame", "navigation-frame-missing", "Declare the navigation frame."),
        ("plant.quaternion_order", "quaternion-order-missing", "Declare quaternion component ordering."),
        ("plant.validity_envelope", "validity-envelope-missing", "Declare the source/model validity envelope."),
        ("fidelity_profiles", "fidelity-profiles-missing", "Declare every fidelity profile the family intends to expose."),
    )
    for path, code, hint in required:
        if not _present(_lookup(manifest, path)):
            _finding(findings, "error", code, f"{manifest_path}:{path}", f"required manifest field {path!r} is missing", hint)
    bindings = manifest.get("bindings")
    if not isinstance(bindings, Sequence) or isinstance(bindings, (str, bytes)) or not bindings:
        _finding(findings, "error", "bindings-missing", f"{manifest_path}:bindings", "no family binding files are declared", "Declare canonical controls, observations, envelope, source lock, and qualification bindings.")
    else:
        for binding in bindings:
            binding_path = ROOT / manifest_path.parent.relative_to(ROOT) / str(binding)
            if not binding_path.is_file():
                _finding(findings, "error", "binding-artifact-missing", str(binding_path.relative_to(ROOT)), f"declared binding {binding!r} does not exist", "Create the binding artifact or correct its relative path.")
    layers = manifest.get("layers")
    if not isinstance(layers, Mapping) or not layers:
        _finding(findings, "warning", "layer-evidence-missing", f"{manifest_path}:layers", "no layer evidence map is declared", "Add trim, linearization, controls, reductions, and scenario evidence references as they become available.")
    source_status = str(_lookup(manifest, "source.availability"))
    if source_status not in {"local_corpus_input_verified", "source_package_verified", "available"}:
        _finding(findings, "warning", "source-availability-unverified", f"{manifest_path}:source.availability", f"source availability is declared as {source_status!r}", "Verify the source payload before runtime replay.")
    return findings
    ####


def _profile_report(
    manifest_path: Path,
    profile_id: str,
    runtime_fidelity: str,
    profile: Mapping[str, Any] | None,
) -> FidelityProfileReadiness:
    findings: list[IntegrationReadinessFinding] = []
    if profile is None:
        _finding(findings, "error", "profile-not-declared", f"{manifest_path}:fidelity_profiles", f"expected profile for {runtime_fidelity!r} is not declared", "Add a named profile or explicitly remove this tier from the family support contract.")
        return FidelityProfileReadiness(profile_id, runtime_fidelity, False, "not_declared", False, False, tuple(findings))
    status = str(profile.get("status", "undeclared_status"))
    if not _present(profile.get("equations")):
        _finding(findings, "error", "profile-equations-missing", f"{manifest_path}:fidelity_profiles.{profile_id}.equations", "profile has no equation declaration", "List the equations actually executed by this fidelity realization.")
    if not _present(profile.get("data_contract")):
        _finding(findings, "error", "profile-data-contract-missing", f"{manifest_path}:fidelity_profiles.{profile_id}.data_contract", "profile has no data contract", "Declare the minimum inputs and evidence expected by this fidelity.")
    if not _present(profile.get("omitted_physics")):
        _finding(findings, "warning", "profile-nonclaims-missing", f"{manifest_path}:fidelity_profiles.{profile_id}.omitted_physics", "omitted physics are not listed", "List omitted physics so automatic lowering and reports remain honest.")
    metadata_ready = not any(item.severity == "error" for item in findings)
    return FidelityProfileReadiness(
        profile_id,
        runtime_fidelity,
        True,
        status,
        metadata_ready,
        metadata_ready and status in QUALIFIED_STATUSES,
        tuple(findings),
    )
    ####


def validate_vehicle_integration_readiness(family_id: str) -> VehicleIntegrationReadinessReport:
    """Build a deterministic manifest and four-tier readiness report."""

    registry = _load_yaml(SUPPORTED_FAMILIES)
    entries = registry.get("families")
    entry = next(
        (item for item in entries if isinstance(item, Mapping) and str(item.get("id")) == family_id),
        None,
    ) if isinstance(entries, list) else None
    if not isinstance(entry, Mapping):
        raise KeyError(f"unknown supported reference family {family_id!r}")
    manifest_label = str(entry.get("manifest", ""))
    manifest_path = ROOT / manifest_label
    if not manifest_path.is_file():
        raise FileNotFoundError(f"family manifest does not exist: {manifest_label}")
    manifest = _load_yaml(manifest_path)
    findings = _check_manifest(manifest_path, family_id, manifest)
    declared_profiles = manifest.get("fidelity_profiles")

    def select_profile(runtime: str, expected_label: str) -> Mapping[str, Any] | None:
        candidates = [
            item
            for item in declared_profiles
            if isinstance(item, Mapping) and str(item.get("runtime_fidelity")) == runtime
        ] if isinstance(declared_profiles, list) else []
        if runtime == "rigid_body_6dof":
            preferred = [
                item
                for item in candidates
                if expected_label.removeprefix("rigid_body_6dof_") in str(item.get("profile_id", ""))
            ]
            if preferred:
                return preferred[0]
            if expected_label != "rigid_body_6dof":
                return None
        return candidates[0] if candidates else None

    profile_reports: list[FidelityProfileReadiness] = []
    for expected_label, runtime in STANDARD_PROFILES:
        profile = select_profile(runtime, expected_label)
        profile_id = (
            str(profile.get("profile_id", f"{family_id}.{expected_label}"))
            if profile is not None
            else f"{family_id}.{expected_label}"
        )
        profile_reports.append(_profile_report(manifest_path, profile_id, runtime, profile))
    profiles = tuple(profile_reports)
    all_errors = any(item.severity == "error" for item in findings) or any(profile.errors for profile in profiles)
    status = "blocked" if all_errors else "ready_for_runtime_probes"
    if not all_errors and any(item.findings for item in profiles) or any(item.severity == "warning" for item in findings):
        status = "ready_with_warnings"
    return VehicleIntegrationReadinessReport(
        family_id,
        str(manifest.get("display_name", entry.get("display_name", family_id))),
        str(manifest.get("physical_family", entry.get("physical_family", "unknown"))),
        manifest_label,
        status,
        tuple(findings),
        profiles,
    )
    ####


def validate_all_vehicle_integration_readiness() -> tuple[VehicleIntegrationReadinessReport, ...]:
    """Return readiness reports for every supported source family."""

    registry = _load_yaml(SUPPORTED_FAMILIES)
    entries = registry.get("families")
    family_ids = [str(item["id"]) for item in entries if isinstance(item, Mapping) and _present(item.get("id"))] if isinstance(entries, list) else []
    return tuple(validate_vehicle_integration_readiness(family_id) for family_id in family_ids)
    ####


__all__ = [
    "FidelityProfileReadiness",
    "IntegrationReadinessFinding",
    "VehicleIntegrationReadinessReport",
    "validate_all_vehicle_integration_readiness",
    "validate_vehicle_integration_readiness",
]
####
