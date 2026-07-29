"""Validate the explicit source-grounded reference-family support boundary."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
REGISTRY = ROOT / "verification/supported_reference_families.yaml"
FAMILIES = ROOT / "verification/vehicle_families.yaml"


def _load(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected a mapping in {path}")
    return payload
####


def validate() -> None:
    """Validate support records against family and source manifests."""

    registry = _load(REGISTRY)
    family_registry = _load(FAMILIES)
    if registry.get("registry_id") != "taoryx_supported_reference_families_v1":
        raise ValueError("invalid supported reference-family registry header")
    records = registry.get("families")
    if not isinstance(records, list) or not records:
        raise ValueError("supported reference-family registry must contain families")
    ids: set[str] = set()
    for record in records:
        if not isinstance(record, dict):
            raise ValueError("reference-family records must be mappings")
        family_id = record.get("id")
        physical_family = record.get("physical_family")
        manifest_path = record.get("manifest")
        if not all(isinstance(value, str) and value for value in (family_id, physical_family, manifest_path)):
            raise ValueError("reference-family records require id, physical_family, and manifest")
        if family_id in ids:
            raise ValueError(f"duplicate supported reference family: {family_id}")
        ids.add(family_id)
        family = family_registry.get("families", {}).get(physical_family)
        if not isinstance(family, dict):
            raise ValueError(f"{family_id} maps to unknown physical family {physical_family}")
        if family_id not in family.get("reference_family_examples", []):
            raise ValueError(f"{family_id} is missing from {physical_family} reference examples")
        manifest = _load(ROOT / manifest_path)
        if manifest.get("family_id") != family_id:
            raise ValueError(f"manifest family mismatch for {family_id}")
        if manifest.get("manifest_type") != "taoryx.reference-family/v1alpha1":
            raise ValueError(f"invalid reference-family manifest type for {family_id}")
        plant = manifest.get("plant", {})
        if plant.get("qualification") != "runtime_replay_qualification_passed":
            raise ValueError(f"{family_id} lacks the required source replay qualification")
        profiles = record.get("supported_profiles")
        manifest_profiles = {profile.get("profile_id") for profile in manifest.get("fidelity_profiles", [])}
        if not isinstance(profiles, list) or not profiles:
            raise ValueError(f"{family_id} must declare supported profiles")
        for profile in profiles:
            if not isinstance(profile, dict) or profile.get("id") not in manifest_profiles:
                raise ValueError(f"{family_id} support profile is absent from its manifest")
    ####


def main() -> int:
    validate()
    print(f"validated {len(_load(REGISTRY)['families'])} supported reference families")
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
####
