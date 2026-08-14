"""Validate that composition-managed maturity claims retain runnable witnesses."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from taoryx.builtin_plugins import source_plugin_entry_points

ROOT = Path(__file__).resolve().parents[1]
MATURITY_REGISTRY = ROOT / "verification/vehicle_maturity_registry.yaml"
COMPOSITION_REGISTRY = ROOT / "verification/vehicle_composition_registry.yaml"
EXECUTION_WITNESSES = ROOT / "verification/vehicle_execution_witnesses.yaml"


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
    identifiers: set[str] = set()
    for record in records:
        if not isinstance(record, dict):
            continue
        family_id = record.get("family_id")
        if isinstance(family_id, str):
            identifiers.add(family_id)
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


def _packaged_fragment_paths() -> tuple[Path, ...]:
    """Find maturity fragments from their owning package entry-point declarations.

    The checkout ledger is deliberately reconstructed from every declared
    sibling plug-in package that carries this resource.  A static package list
    would drift whenever a family is split, merged, or newly added.
    """

    fragments: list[Path] = []
    for declaration in source_plugin_entry_points():
        module_name = declaration.value.partition(":")[0]
        package_name = module_name.split(".", maxsplit=1)[0]
        source = (
            declaration.project.parent
            / "src"
            / package_name
            / "data"
            / "verification"
            / "vehicle_maturity_registry.yaml"
        )
        if source.is_file():
            fragments.append(source)
    if not fragments:
        raise ValueError("no declared plug-in package owns a vehicle maturity registry fragment")
    return tuple(fragments)
####


def _validate_packaged_fragments(canonical: dict[str, Any]) -> None:
    """Prove that the package-owned maturity rows reconstruct the source ledger.

    The checkout-level registry is still the canonical editing surface.  A
    split family owns a non-overlapping fragment in its own wheel, so bytewise
    equality with the reference-model package would incorrectly require that
    package to keep a second family copy. Compare records by stable ID
    instead, preserving the source ledger's headers and exact record content.
    """

    canonical_records = canonical.get("records")
    if not isinstance(canonical_records, list):
        raise ValueError("vehicle maturity registry must contain records")
    fragments: list[dict[str, Any]] = []
    for source in _packaged_fragment_paths():
        fragment = _load_mapping(source)
        if fragment.get("registry_id") != canonical.get("registry_id"):
            raise ValueError(f"vehicle maturity registry packaged fragment has an incompatible header: {source}")
        if fragment.get("levels") != canonical.get("levels"):
            raise ValueError(f"vehicle maturity registry packaged fragment has incompatible levels: {source}")
        fragments.append(fragment)

    packaged_records = [
        record
        for fragment in fragments
        for record in fragment.get("records", [])
        if isinstance(record, dict)
    ]
    canonical_by_id = {
        record.get("id"): record
        for record in canonical_records
        if isinstance(record, dict) and isinstance(record.get("id"), str)
    }
    packaged_by_id = {
        record.get("id"): record
        for record in packaged_records
        if isinstance(record.get("id"), str)
    }
    if len(packaged_by_id) != len(packaged_records):
        raise ValueError("vehicle maturity registry packaged fragments contain duplicate or invalid record IDs")
    if packaged_by_id != canonical_by_id:
        raise ValueError("vehicle maturity registry packaged fragments do not reconstruct the canonical ledger")
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
    _validate_packaged_fragments(maturity)
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
        if not isinstance(identifier, str) or not identifier:
            raise ValueError("vehicle maturity records require id, maturity, status, and next_gate")
        if not isinstance(maturity_level, str) or not maturity_level:
            raise ValueError("vehicle maturity records require id, maturity, status, and next_gate")
        if not isinstance(status, str) or not status:
            raise ValueError("vehicle maturity records require id, maturity, status, and next_gate")
        if not isinstance(next_gate, str) or not next_gate:
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
