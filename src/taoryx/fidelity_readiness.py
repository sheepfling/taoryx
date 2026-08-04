"""Data-readiness checks for the four Taoryx vehicle realization tiers.

This module answers whether a registered vehicle has enough declared data to
enter a fidelity tier. It does not run a trim, trajectory, controller, or
numerical qualification case.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .fidelity_contracts import CANONICAL_FIDELITY_TIERS, control_realization_for
from .vehicle_registry import ROOT

CATALOG = ROOT / "verification/fidelity_data_requirements.yaml"
REGISTRY = ROOT / "verification/vehicle_models.yaml"
SEVERITIES = {"required", "recommended"}
READINESS_TIER_ALIASES = {"pseudo_6dof_kinematic_bridge": "pseudo_6dof"}
CATALOG_TIER_NAMES = {canonical: legacy for legacy, canonical in READINESS_TIER_ALIASES.items()}


def canonical_readiness_tier(tier: str) -> str:
    """Normalize readiness names while rejecting ambiguous rigid-body legacy names."""

    if tier in CANONICAL_FIDELITY_TIERS:
        return tier
    if tier in READINESS_TIER_ALIASES:
        return READINESS_TIER_ALIASES[tier]
    if tier == "rigid_body_6dof":
        raise ValueError("legacy 'rigid_body_6dof' requires direct_wrench or surface_allocated")
    raise KeyError(f"unknown fidelity tier {tier!r}")
    ####


@dataclass(frozen=True, slots=True)
class ReadinessFinding:
    """One checklist result for a vehicle and fidelity tier."""

    requirement_id: str
    title: str
    severity: str
    status: str
    message: str
    paths: tuple[str, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        """Return a stable JSON-compatible finding."""

        return {
            "requirement_id": self.requirement_id,
            "title": self.title,
            "severity": self.severity,
            "status": self.status,
            "message": self.message,
            "paths": list(self.paths),
        }

    ####


@dataclass(frozen=True, slots=True)
class FidelityReadinessReport:
    """Machine-readable readiness result, separate from runtime qualification."""

    vehicle_id: str
    family_id: str
    tier: str
    findings: tuple[ReadinessFinding, ...]
    runtime_proof_status: str = "not_evaluated"

    @property
    def control_realization(self) -> str:
        """Return the canonical realization represented by this readiness tier."""

        return control_realization_for(self.tier) if self.tier in CANONICAL_FIDELITY_TIERS else "unspecified"
        ####

    @property
    def errors(self) -> tuple[ReadinessFinding, ...]:
        """Return missing required data."""

        return tuple(item for item in self.findings if item.status == "missing" and item.severity == "required")

    ####

    @property
    def warnings(self) -> tuple[ReadinessFinding, ...]:
        """Return missing recommended data or unresolved evidence checks."""

        return tuple(item for item in self.findings if item.status in {"missing", "not_checked"} and item.severity == "recommended")

    ####

    @property
    def status(self) -> str:
        """Return blocked, partial, or metadata-ready."""

        if self.errors:
            return "blocked"
        if self.warnings:
            return "partial"
        return "ready_for_runtime_probes"

    ####

    def as_dict(self) -> dict[str, Any]:
        """Return a stable report payload."""

        return {
            "vehicle_id": self.vehicle_id,
            "family_id": self.family_id,
            "tier": self.tier,
            "control_realization": self.control_realization,
            "status": self.status,
            "runtime_proof_status": self.runtime_proof_status,
            "error_count": len(self.errors),
            "warning_count": len(self.warnings),
            "findings": [item.as_dict() for item in self.findings],
        }

    ####


def _load_yaml(path: Path) -> Mapping[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"{path} must contain a mapping")
    return payload
    ####


def _lookup(value: Mapping[str, Any], path: str) -> Any:
    current: Any = value
    for part in path.split("."):
        if not isinstance(current, Mapping) or part not in current:
            return None
        current = current[part]
    return current
    ####


def _present(value: Any) -> bool:
    return value is not None and value != "" and value != [] and value != {}
    ####


def _requirements(catalog: Mapping[str, Any], tier: str) -> list[Mapping[str, Any]]:
    tier = CATALOG_TIER_NAMES.get(tier, tier)
    tiers = catalog.get("tiers", {})
    if not isinstance(tiers, Mapping) or tier not in tiers:
        raise KeyError(f"unknown fidelity tier: {tier}")
    definition = tiers[tier]
    if not isinstance(definition, Mapping):
        raise ValueError(f"tier {tier} must be a mapping")
    result: list[Mapping[str, Any]] = []
    for inherited in definition.get("inherits", ()):
        result.extend(_requirements(catalog, str(inherited)))
    result.extend(item for item in definition.get("requirements", ()) if isinstance(item, Mapping))
    return result
    ####


def _family_requirements(catalog: Mapping[str, Any], family_id: str, tier: str) -> list[Mapping[str, Any]]:
    tier = CATALOG_TIER_NAMES.get(tier, tier)
    families = catalog.get("family_overlays", {})
    family = families.get(family_id) if isinstance(families, Mapping) else None
    if not isinstance(family, Mapping):
        return []
    result = [item for item in family.get("requirements", ()) if isinstance(item, Mapping)]
    tier_overlays = family.get("tiers", {})
    tier_overlay = None
    if isinstance(tier_overlays, Mapping):
        tier_overlay = tier_overlays.get(tier)
        if tier_overlay is None:
            tier_overlay = tier_overlays.get(READINESS_TIER_ALIASES.get(tier, tier))
    if isinstance(tier_overlay, Mapping):
        result.extend(item for item in tier_overlay.get("requirements", ()) if isinstance(item, Mapping))
    return result
    ####


def _table_names(vehicle: Mapping[str, Any]) -> list[str]:
    bindings = _lookup(vehicle, "table_bindings")
    if not isinstance(bindings, Sequence) or isinstance(bindings, (str, bytes)):
        return []
    return [str(item).lower() for item in bindings]
    ####


def _check(requirement: Mapping[str, Any], vehicle: Mapping[str, Any]) -> tuple[str, str, tuple[str, ...]]:
    kind = str(requirement.get("kind", ""))
    paths = tuple(str(path) for path in requirement.get("paths", ()) if path is not None)
    if "path" in requirement:
        paths = (str(requirement["path"]),)
    values = [_lookup(vehicle, path) for path in paths]
    if kind == "all_fields":
        ok = all(_present(value) for value in values)
        return ("present" if ok else "missing", "all declared fields are present" if ok else "one or more declared fields are missing", paths)
    if kind == "any_fields":
        ok = any(_present(value) for value in values)
        return ("present" if ok else "missing", "at least one supported declaration is present" if ok else "no supported declaration is present", paths)
    if kind == "non_empty_list":
        result = values[0] if values else None
        ok = isinstance(result, list) and bool(result)
        return ("present" if ok else "missing", "list is populated" if ok else "list is empty or absent", paths)
    if kind == "non_empty_mapping":
        result = values[0] if values else None
        ok = isinstance(result, Mapping) and bool(result)
        return ("present" if ok else "missing", "mapping is populated" if ok else "mapping is empty or absent", paths)
    if kind == "value_in":
        ok = bool(values) and values[0] in requirement.get("values", ())
        return (
            "present" if ok else "missing",
            "value is in the allowed set" if ok else f"value {values[0] if values else None!r} is not in the allowed set",
            paths,
        )
    if kind == "value_not_in":
        ok = bool(values) and _present(values[0]) and values[0] not in requirement.get("values", ())
        return ("present" if ok else "missing", "value is explicitly non-empty" if ok else "value is absent or disallowed", paths)
    if kind == "string_contains":
        value = str(values[0]) if values else ""
        token = str(requirement.get("value", "")).lower()
        ok = token in value.lower()
        return ("present" if ok else "missing", f"value contains {token!r}" if ok else f"value does not contain {token!r}", paths)
    if kind == "controls_minimum":
        result = values[0] if values else None
        minimum = int(requirement.get("minimum", 1))
        actual = len(result) if isinstance(result, list) else 0
        ok = actual >= minimum
        return ("present" if ok else "missing", f"{actual} controls declared; {minimum} required", paths)
    if kind == "controls_have_limits":
        result = values[0] if values else None
        ok = isinstance(result, list) and bool(result) and all(isinstance(item, Mapping) and "lower" in item and "upper" in item for item in result)
        return ("present" if ok else "missing", "all controls have lower and upper limits" if ok else "one or more controls lacks lower/upper limits", paths)
    if kind == "controls_named_any":
        result = values[0] if values else None
        names = [str(item.get("name", "")).lower() for item in result if isinstance(item, Mapping)] if isinstance(result, list) else []
        requested = [str(item).lower() for item in requirement.get("names", ())]
        ok = any(any(token in name for token in requested) for name in names)
        return ("present" if ok else "missing", "family control role is represented" if ok else "required family control role is absent", paths)
    if kind == "all_artifacts":
        bindings = values[0] if values else None
        missing = [str(item) for item in bindings if not (ROOT / str(item)).is_file()] if isinstance(bindings, list) else ["table_bindings"]
        ok = isinstance(bindings, list) and bool(bindings) and not missing
        return ("present" if ok else "missing", "all bound artifacts exist" if ok else f"missing artifacts: {', '.join(missing)}", paths)
    if kind == "table_token_any":
        names = _table_names(vehicle)
        tokens = [str(token).lower() for token in requirement.get("tokens", ())]
        ok = any(token in name for name in names for token in tokens)
        return (
            "present" if ok else "missing",
            "a matching control or propulsion table is bound" if ok else "no bound table matches the required family data",
            paths,
        )
    if kind == "table_token_all_group":
        names = _table_names(vehicle)
        groups = requirement.get("token_groups", ())
        ok = all(any(str(token).lower() in name for name in names for token in group) for group in groups if isinstance(group, list))
        return ("present" if ok else "missing", "all required table groups are represented" if ok else "one or more required table groups is absent", paths)
    if kind == "declared_probe":
        return ("not_checked", f"runtime probe {requirement.get('probe', 'unspecified')} is not run by metadata readiness", paths)
    return ("not_checked", f"unsupported checklist check kind {kind!r}", paths)
    ####


def validate_fidelity_readiness(vehicle_id: str, tier: str) -> FidelityReadinessReport:
    """Validate one registered vehicle against one fidelity data contract."""

    canonical_tier = canonical_readiness_tier(tier)
    catalog = _load_yaml(CATALOG)
    registry = _load_yaml(REGISTRY)
    vehicles = registry.get("vehicles", {})
    if not isinstance(vehicles, Mapping) or vehicle_id not in vehicles or not isinstance(vehicles[vehicle_id], Mapping):
        raise KeyError(f"unknown vehicle registry id: {vehicle_id}")
    vehicle = vehicles[vehicle_id]
    family_id = str(vehicle.get("family_id", ""))
    findings: list[ReadinessFinding] = []
    requirements = _requirements(catalog, canonical_tier) + _family_requirements(catalog, family_id, canonical_tier)
    seen: set[str] = set()
    for requirement in requirements:
        requirement_id = str(requirement.get("id", "unknown"))
        if requirement_id in seen:
            continue
        seen.add(requirement_id)
        severity = str(requirement.get("severity", "required"))
        if severity not in SEVERITIES:
            raise ValueError(f"invalid severity {severity!r} for {requirement_id}")
        status, message, paths = _check(requirement, vehicle)
        findings.append(ReadinessFinding(requirement_id, str(requirement.get("title", requirement_id)), severity, status, message, paths))
    return FidelityReadinessReport(vehicle_id, family_id, canonical_tier, tuple(findings))
    ####


def validate_all_fidelity_readiness(vehicle_id: str, tiers: Sequence[str]) -> tuple[FidelityReadinessReport, ...]:
    """Validate all requested tiers for one vehicle."""

    return tuple(validate_fidelity_readiness(vehicle_id, tier) for tier in tiers)
    ####
