"""Validate that composition-managed maturity claims retain runnable witnesses."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
MATURITY_REGISTRY = ROOT / "verification/vehicle_maturity_registry.yaml"
COMPOSITION_REGISTRY = ROOT / "verification/vehicle_composition_registry.yaml"
EXECUTION_WITNESSES = ROOT / "verification/vehicle_execution_witnesses.yaml"
PACKAGED_MIRROR = ROOT / "packages/taoryx-reference-models/src/taoryx_reference_models/data/verification/vehicle_maturity_registry.yaml"


def _load_mapping(path: Path) -> dict[str, Any]:
    """Load one YAML mapping with a useful failure message."""

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected a mapping in {path}")
    return payload
####


def _composition_family_ids(registry: dict[str, Any]) -> set[str]:
    """Return the exact composition family identifiers from the authoritative catalog."""

    records = registry.get("vehicles")
    if not isinstance(records, list):
        raise ValueError("vehicle composition registry must contain a vehicles list")
    identifiers = {
        record.get("family_id")
        for record in records
        if isinstance(record, dict) and isinstance(record.get("family_id"), str)
    }
    if not identifiers:
        raise ValueError("vehicle composition registry contains no family identifiers")
    return identifiers
####


def _batch_witness_families(witnesses: dict[str, Any]) -> set[str]:
    """Resolve vehicle identifiers from checked-in batch witness requests."""

    records = witnesses.get("witnesses")
    if not isinstance(records, list):
        raise ValueError("vehicle execution witnesses must contain a witnesses list")
    families: set[str] = set()
    for record in records:
        if not isinstance(record, dict) or record.get("operation") != "batch":
            continue
        composition_path = record.get("composition")
        if not isinstance(composition_path, str) or not composition_path:
            raise ValueError("batch witness requires a composition path")
        composition = _load_mapping(ROOT / composition_path)
        family_id = composition.get("vehicle")
        if not isinstance(family_id, str) or not family_id:
            raise ValueError(f"batch witness {record.get('id')!r} has no vehicle identifier")
        families.add(family_id)
    return families
####


def validate() -> None:
    """Check maturity metadata against concrete public Composition witnesses.

    A ``composition_family_id`` makes a maturity record an executable catalog
    claim.  It must name an installed composition family and retain at least one
    checked-in public batch witness.  The witness executor independently proves
    runtime lowering; this validator prevents the planning ledger from silently
    drifting away from those concrete entry points.
    """

    maturity = _load_mapping(MATURITY_REGISTRY)
    if not PACKAGED_MIRROR.is_file():
        raise ValueError("vehicle maturity registry packaged mirror is missing")
    if PACKAGED_MIRROR.read_text(encoding="utf-8") != MATURITY_REGISTRY.read_text(encoding="utf-8"):
        raise ValueError("vehicle maturity registry packaged mirror is stale")
    if maturity.get("registry_id") != "taoryx_vehicle_maturity_v1":
        raise ValueError("invalid vehicle maturity registry header")
    levels = maturity.get("levels")
    if not isinstance(levels, list) or not all(isinstance(level, str) for level in levels):
        raise ValueError("vehicle maturity registry must declare string levels")
    records = maturity.get("records")
    if not isinstance(records, list) or not records:
        raise ValueError("vehicle maturity registry must contain records")

    composition_ids = _composition_family_ids(_load_mapping(COMPOSITION_REGISTRY))
    witnessed_families = _batch_witness_families(_load_mapping(EXECUTION_WITNESSES))
    identifiers: set[str] = set()
    composition_managed_count = 0
    for record in records:
        if not isinstance(record, dict):
            raise ValueError("vehicle maturity records must be mappings")
        identifier = record.get("id")
        maturity_level = record.get("maturity")
        status = record.get("status")
        next_gate = record.get("next_gate")
        if not all(isinstance(value, str) and value for value in (identifier, maturity_level, status, next_gate)):
            raise ValueError("vehicle maturity records require id, maturity, status, and next_gate")
        if identifier in identifiers:
            raise ValueError(f"duplicate vehicle maturity record: {identifier}")
        identifiers.add(identifier)
        if maturity_level not in levels:
            raise ValueError(f"{identifier} declares unknown maturity level {maturity_level!r}")

        composition_family_id = record.get("composition_family_id")
        if composition_family_id is None:
            continue
        if not isinstance(composition_family_id, str) or not composition_family_id:
            raise ValueError(f"{identifier} has an invalid composition_family_id")
        composition_managed_count += 1
        if composition_family_id not in composition_ids:
            raise ValueError(f"{identifier} references unknown composition family {composition_family_id!r}")
        if composition_family_id not in witnessed_families:
            raise ValueError(
                f"{identifier} composition family {composition_family_id!r} lacks a checked-in batch execution witness"
            )
    if composition_managed_count == 0:
        raise ValueError("vehicle maturity registry declares no composition-managed records")
####


def main() -> int:
    """Validate the maturity registry as a command-line focused catalog check."""

    validate()
    print("validated vehicle maturity registry")
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
####
