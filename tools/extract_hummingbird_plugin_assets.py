"""Mechanically materialize the Hummingbird plug-in's owned data fragment.

The canonical checkout registries remain authoritative.  This script extracts
only Hummingbird rows into the independently installable distribution so the
wheel never needs the reference-model aggregate merely to describe or run its
own family.  It is intentionally narrow and deterministic; do not hand-edit
the generated files under ``packages/taoryx-hummingbird/.../data``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import tempfile
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "packages/taoryx-hummingbird/src/taoryx_hummingbird/data"
FAMILY_ID = "hummingbird"


def _read(relative: str) -> dict[str, Any]:
    payload = yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{relative} must contain a mapping")
    return payload
    ####


def _write(target: Path, relative: str, payload: dict[str, Any]) -> None:
    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    ####


def _copy(target: Path, relative: str) -> None:
    """Copy one checked-in Hummingbird witness input under package data."""

    source = ROOT / relative
    destination = target / relative
    if not source.is_file():
        raise ValueError(f"missing Hummingbird package asset: {relative}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    ####


def _write_json(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write one package-owned provenance fragment deterministically."""

    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _family_rows(payload: dict[str, Any], key: str) -> dict[str, Any]:
    result = dict(payload)
    rows = payload.get(key)
    if not isinstance(rows, list):
        raise ValueError(f"{key} must contain a list")
    result[key] = [row for row in rows if isinstance(row, dict) and row.get("family_id") == FAMILY_ID]
    return result
    ####


def extract(target: Path = TARGET) -> None:
    """Write the family-owned catalog rows and source assets."""

    # Package data is wholly generated. Recreate its exact tree rather than
    # merging onto an earlier result, so deleted canonical assets cannot linger
    # in a later Hummingbird wheel.
    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)

    vehicle_models = _read("verification/vehicle_models.yaml")
    vehicles = vehicle_models.get("vehicles")
    if not isinstance(vehicles, dict) or FAMILY_ID not in vehicles:
        raise ValueError("canonical vehicle_models registry has no Hummingbird record")
    vehicle_models["vehicles"] = {FAMILY_ID: vehicles[FAMILY_ID]}
    _write(target, "verification/vehicle_models.yaml", vehicle_models)

    for relative, key in (
        ("verification/vehicle_composition_registry.yaml", "vehicles"),
        ("verification/horizontal_fidelity_registry.yaml", "families"),
        ("verification/vehicle_execution_bindings.yaml", "bindings"),
        ("verification/vehicle_execution_parity.yaml", "bindings"),
        ("verification/vehicle_endpoint_specs.yaml", "endpoints"),
    ):
        _write(target, relative, _family_rows(_read(relative), key))

    witness_catalog = _read("verification/vehicle_execution_witnesses.yaml")
    _write(target, "verification/vehicle_execution_witnesses.yaml", _hummingbird_witness_fragment(witness_catalog))
    _write(
        target,
        "verification/vehicle_execution_parity_witnesses.yaml",
        _hummingbird_parity_witness_fragment(_read("verification/vehicle_execution_parity_witnesses.yaml")),
    )
    for relative in _hummingbird_witness_paths(witness_catalog):
        _copy(target, relative)

    for relative in _hummingbird_source_asset_paths():
        _copy(target, relative)
    _write_source_table_catalog_fragments(target)

    pseudo = _read("verification/pseudo6dof_profiles.yaml")
    for key in ("profiles", "direct_wrench_profiles", "surface_allocation_profiles", "bindings"):
        rows = pseudo.get(key)
        if not isinstance(rows, list):
            raise ValueError(f"pseudo6dof_profiles.yaml {key} must contain a list")
        pseudo[key] = [row for row in rows if isinstance(row, dict) and row.get("family_id") == FAMILY_ID]
    _write(target, "verification/pseudo6dof_profiles.yaml", pseudo)

    maturity = _read("verification/vehicle_maturity_registry.yaml")
    records = maturity.get("records")
    if not isinstance(records, list):
        raise ValueError("vehicle_maturity_registry.yaml records must contain a list")
    maturity["records"] = [row for row in records if isinstance(row, dict) and row.get("composition_family_id") == FAMILY_ID]
    _write(target, "verification/vehicle_maturity_registry.yaml", maturity)

    lqr_path = ROOT / "verification/lqr_scaling_profiles.yaml"
    lqr = yaml.safe_load(lqr_path.read_text(encoding="utf-8"))
    if not isinstance(lqr, dict):
        raise ValueError("canonical lqr_scaling_profiles.yaml must contain a mapping")
    profiles = lqr.get("profiles")
    if not isinstance(profiles, dict):
        raise ValueError("lqr_scaling_profiles.yaml profiles must contain a mapping")
    lqr["profiles"] = {key: row for key, row in profiles.items() if isinstance(row, dict) and row.get("vehicle") == FAMILY_ID}
    _write(target, "verification/lqr_scaling_profiles.yaml", lqr)

    _write_json(
        target,
        "package_data_provenance.json",
        {
            "schema": "taoryx.hummingbird-package-data/v1",
            "family_id": FAMILY_ID,
            "source_catalog_root": "verification/",
            "claim_boundary": (
                "This package fragment mirrors only Hummingbird catalog rows, source tables, "
                "focused witnesses, and local controller-screen evidence; repository-root catalogs "
                "remain canonical. The reduced streaming interface does not promote the physical "
                "rotor screens into a flight-qualified controller."
            ),
        },
    )

    ####


def _hummingbird_witness_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Extract every Hummingbird endpoint, variant, and graph witness."""

    result = dict(payload)
    for key in ("witnesses", "variant_witnesses", "graph_extension_witnesses"):
        rows = payload.get(key, [])
        if not isinstance(rows, list):
            raise ValueError(f"vehicle_execution_witnesses.yaml {key} must contain a list")
        selected: list[dict[str, Any]] = []
        for row in rows:
            if not isinstance(row, dict):
                continue
            composition = row.get("composition")
            if not isinstance(composition, str):
                continue
            if composition.startswith("examples/vehicle_composition/hummingbird_"):
                selected.append(row)
        result[key] = selected
    return result
    ####


def _hummingbird_parity_witness_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Extract Hummingbird replay evidence without unrelated parity traces."""

    witnesses = payload.get("witnesses")
    if not isinstance(witnesses, list):
        raise ValueError("vehicle_execution_parity_witnesses.yaml witnesses must contain a list")
    result = dict(payload)
    result["witnesses"] = [
        row
        for row in witnesses
        if isinstance(row, dict)
        and isinstance(row.get("composition"), str)
        and row["composition"].startswith("examples/vehicle_composition/hummingbird_")
    ]
    return result
    ####


def _hummingbird_witness_paths(payload: dict[str, Any]) -> tuple[str, ...]:
    """Return unique package-relative inputs referenced by Hummingbird witnesses."""

    fragment = _hummingbird_witness_fragment(payload)
    paths: set[str] = set()
    for key in ("witnesses", "variant_witnesses", "graph_extension_witnesses"):
        rows = fragment.get(key, [])
        if not isinstance(rows, list):
            continue
        for row in rows:
            if isinstance(row, dict) and isinstance(row.get("composition"), str):
                paths.add(row["composition"])
    return tuple(sorted(paths))
    ####


def _write_source_table_catalog_fragments(target: Path) -> None:
    """Carry Hummingbird's table provenance without the fixed-wing tables."""

    relative = "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
    catalog_path = ROOT / relative / "catalog.yaml"
    catalog = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
    if not isinstance(catalog, dict) or not isinstance(catalog.get("tables"), list):
        raise ValueError("source table catalog must contain a tables list")
    catalog["tables"] = [row for row in catalog["tables"] if isinstance(row, dict) and row.get("vehicle") == FAMILY_ID]
    _write(target, f"{relative}/catalog.yaml", catalog)

    manifest_path = ROOT / relative / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise ValueError("source table manifest must contain an object")
    generated = manifest.get("generated")
    models = manifest.get("models")
    if not isinstance(generated, list) or not isinstance(models, dict):
        raise ValueError("source table manifest must contain generated files and models")
    manifest["generated"] = [name for name in generated if isinstance(name, str) and name.startswith("hummingbird_")]
    manifest["models"] = {FAMILY_ID: models[FAMILY_ID]} if FAMILY_ID in models else {}
    _write_json(target, f"{relative}/manifest.json", manifest)
    ####


def _hummingbird_source_asset_paths() -> tuple[str, ...]:
    """Return the physical source files that transfer with this family."""

    bundle = "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1"
    paths = {
        "examples/generated/vehicles/hummingbird_canonical_hover_6dof.prb",
        "examples/generated/vehicles/hummingbird_individual_rotor_hover_6dof.prb",
    }
    table_root = ROOT / bundle / "tables"
    paths.update(path.relative_to(ROOT).as_posix() for path in table_root.glob("hummingbird_*.tbl") if path.is_file())
    source_root = ROOT / bundle / "quadcopter_hummingbird"
    paths.update(
        path.relative_to(ROOT).as_posix() for path in source_root.rglob("*") if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc"
    )
    return tuple(sorted(paths))
    ####


def _file_digests(root: Path) -> dict[str, str]:
    """Return a content-addressed inventory for one generated data tree."""

    if not root.is_dir():
        return {}
    return {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(root.rglob("*")) if path.is_file()}
    ####


def check(target: Path = TARGET) -> tuple[str, ...]:
    """Report every missing, stale, or unexpected Hummingbird package-data file."""

    with tempfile.TemporaryDirectory(prefix="taoryx-hummingbird-plugin-assets-") as directory:
        expected_root = Path(directory) / "data"
        extract(expected_root)
        expected = _file_digests(expected_root)
    actual = _file_digests(target)
    missing = tuple(sorted(set(expected) - set(actual)))
    unexpected = tuple(sorted(set(actual) - set(expected)))
    changed = tuple(sorted(path for path in set(expected) & set(actual) if expected[path] != actual[path]))
    return (
        *(f"missing generated Hummingbird asset: {path}" for path in missing),
        *(f"unexpected generated Hummingbird asset: {path}" for path in unexpected),
        *(f"stale generated Hummingbird asset: {path}" for path in changed),
    )
    ####


def main(argv: list[str] | None = None) -> int:
    """Regenerate or verify the complete Hummingbird package-data boundary."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if package data differs from canonical Hummingbird inputs")
    args = parser.parse_args(argv)
    if args.check:
        differences = check()
        if differences:
            print("Hummingbird plug-in package data is stale:")
            print("\n".join(differences))
            return 1
        print("Hummingbird plug-in package data: current")
        return 0
    extract()
    print("Hummingbird plug-in package data: regenerated")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
