"""Mechanically materialize passive-body-owned package data.

The repository-root verification catalog remains canonical.  This extractor
copies the non-overlapping ``tumbling_body`` rows and direct-release examples
into the independently installable passive-bodies distribution.  Do not
hand-edit its generated package data.
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
TARGET = ROOT / "packages/taoryx-passive-bodies/src/taoryx_passive_bodies/data"
FAMILY_ID = "tumbling_body"


def _read(relative: str) -> dict[str, Any]:
    """Read one canonical mapping catalog."""

    payload = yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{relative} must contain a mapping")
    return payload
    ####


def _write(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write a deterministic package-owned YAML fragment."""

    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    ####


def _copy(target: Path, relative: str) -> None:
    """Copy one direct-release witness input into package data."""

    source = ROOT / relative
    destination = target / relative
    if not source.is_file():
        raise ValueError(f"missing passive-body asset: {relative}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    ####


def _family_rows(payload: dict[str, Any], key: str) -> dict[str, Any]:
    """Filter a conventional list catalog to the passive family."""

    rows = payload.get(key)
    if not isinstance(rows, list):
        raise ValueError(f"{key} must contain a list")
    result = dict(payload)
    result[key] = [row for row in rows if isinstance(row, dict) and row.get("family_id") == FAMILY_ID]
    return result
    ####


def _passive_witness_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Select direct-release execution witnesses and no parent-model rows."""

    result = dict(payload)
    for key in ("witnesses", "variant_witnesses", "graph_extension_witnesses"):
        rows = payload.get(key, [])
        if not isinstance(rows, list):
            raise ValueError(f"vehicle_execution_witnesses.yaml {key} must contain a list")
        result[key] = [
            row
            for row in rows
            if isinstance(row, dict)
            and isinstance(row.get("composition"), str)
            and row["composition"].startswith("examples/vehicle_composition/tumbling_body_")
        ]
    return result
    ####


def extract(target: Path = TARGET) -> None:
    """Write all portable catalog rows required by the passive direct witness."""

    # Package data is wholly generated. Recreate its exact tree rather than
    # merging onto an earlier result, so stale release assets cannot linger in
    # a later passive-bodies wheel.
    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)

    vehicle_models = _read("verification/vehicle_models.yaml")
    vehicles = vehicle_models.get("vehicles")
    if not isinstance(vehicles, dict):
        raise ValueError("canonical vehicle_models registry must contain a vehicles mapping")
    # The passive family deliberately has no legacy vehicle-model record:
    # ``vehicle_registry_id`` is null in its horizontal manifest.  Retain a
    # valid empty fragment instead of manufacturing propulsion-era metadata.
    vehicle_models["vehicles"] = {}
    _write(target, "verification/vehicle_models.yaml", vehicle_models)

    for relative, key in (
        ("verification/vehicle_composition_registry.yaml", "vehicles"),
        ("verification/horizontal_fidelity_registry.yaml", "families"),
        ("verification/vehicle_execution_bindings.yaml", "bindings"),
        ("verification/vehicle_execution_parity.yaml", "bindings"),
        ("verification/vehicle_endpoint_specs.yaml", "endpoints"),
    ):
        _write(target, relative, _family_rows(_read(relative), key))

    pseudo = _read("verification/pseudo6dof_profiles.yaml")
    for key in ("profiles", "direct_wrench_profiles", "surface_allocation_profiles", "bindings"):
        pseudo = _family_rows(pseudo, key)
    _write(target, "verification/pseudo6dof_profiles.yaml", pseudo)

    maturity = _read("verification/vehicle_maturity_registry.yaml")
    records = maturity.get("records")
    if not isinstance(records, list):
        raise ValueError("vehicle maturity registry must contain records")
    maturity["records"] = [row for row in records if isinstance(row, dict) and row.get("composition_family_id") == FAMILY_ID]
    _write(target, "verification/vehicle_maturity_registry.yaml", maturity)

    witnesses = _read("verification/vehicle_execution_witnesses.yaml")
    _write(target, "verification/vehicle_execution_witnesses.yaml", _passive_witness_fragment(witnesses))
    for relative in (
        "examples/vehicle_composition/tumbling_body_direct_release_3dof_compose.yaml",
        "examples/vehicle_composition/tumbling_body_direct_release_pseudo6dof_compose.yaml",
    ):
        _copy(target, relative)

    lqr_path = ROOT / "verification/lqr_scaling_profiles.yaml"
    lqr = yaml.safe_load(lqr_path.read_text(encoding="utf-8"))
    if not isinstance(lqr, dict):
        raise ValueError("canonical lqr_scaling_profiles.yaml must contain a mapping")
    profiles = lqr.get("profiles")
    if not isinstance(profiles, dict):
        raise ValueError("lqr_scaling_profiles.yaml profiles must contain a mapping")
    lqr["profiles"] = {identifier: row for identifier, row in profiles.items() if isinstance(row, dict) and row.get("vehicle") == FAMILY_ID}
    _write(target, "verification/lqr_scaling_profiles.yaml", lqr)

    provenance = {
        "schema": "taoryx.passive-bodies-package-data/v1",
        "family_id": FAMILY_ID,
        "source_catalog_root": "verification/",
        "claim_boundary": "This package fragment mirrors only passive-body-owned catalog rows; repository-root catalogs remain canonical.",
    }
    path = target / "package_data_provenance.json"
    path.write_text(json.dumps(provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _file_digests(root: Path) -> dict[str, str]:
    """Return a content-addressed inventory for one generated data tree."""

    if not root.is_dir():
        return {}
    return {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(root.rglob("*")) if path.is_file()}
    ####


def check(target: Path = TARGET) -> tuple[str, ...]:
    """Report every missing, stale, or unexpected passive-body package asset."""

    with tempfile.TemporaryDirectory(prefix="taoryx-passive-bodies-plugin-assets-") as directory:
        expected_root = Path(directory) / "data"
        extract(expected_root)
        expected = _file_digests(expected_root)
    actual = _file_digests(target)
    missing = tuple(sorted(set(expected) - set(actual)))
    unexpected = tuple(sorted(set(actual) - set(expected)))
    changed = tuple(sorted(path for path in set(expected) & set(actual) if expected[path] != actual[path]))
    return (
        *(f"missing generated passive-body asset: {path}" for path in missing),
        *(f"unexpected generated passive-body asset: {path}" for path in unexpected),
        *(f"stale generated passive-body asset: {path}" for path in changed),
    )
    ####


def main(argv: list[str] | None = None) -> int:
    """Regenerate or verify the complete passive-bodies package-data boundary."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if package data differs from canonical passive-body inputs")
    args = parser.parse_args(argv)
    if args.check:
        differences = check()
        if differences:
            print("Passive-bodies plug-in package data is stale:")
            print("\n".join(differences))
            return 1
        print("Passive-bodies plug-in package data: current")
        return 0
    extract()
    print("Passive-bodies plug-in package data: regenerated")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
