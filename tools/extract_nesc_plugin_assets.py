"""Mechanically materialize NESC-owned package data.

The repository-root verification catalog remains canonical.  This extractor
copies only the NASA/NESC two-stage source-replay family rows, its source
manifest/evidence, and its composition witnesses into the independently
installable ``taoryx-nesc`` distribution.  Do not hand-edit the generated
package data.
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
TARGET = ROOT / "packages/taoryx-nesc/src/taoryx_nesc/data"
FAMILY_ID = "reference_nesc_two_stage_rocket"
_NESC_WITNESS_PREFIX = "examples/vehicle_composition/nesc_"


def _read(relative: str) -> dict[str, Any]:
    """Read one canonical mapping catalog."""

    payload = yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{relative} must contain a mapping")
    return payload
    ####


def _write(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write one deterministic NESC-owned YAML fragment."""

    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    ####


def _copy(target: Path, relative: str) -> None:
    """Copy one NESC-owned source or witness asset into package data."""

    source = ROOT / relative
    destination = target / relative
    if not source.is_file():
        raise ValueError(f"missing NESC package asset: {relative}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    ####


def _family_rows(payload: dict[str, Any], key: str) -> dict[str, Any]:
    """Filter a conventional list catalog to the NESC composition family."""

    rows = payload.get(key)
    if not isinstance(rows, list):
        raise ValueError(f"{key} must contain a list")
    result = dict(payload)
    result[key] = [row for row in rows if isinstance(row, dict) and row.get("family_id") == FAMILY_ID]
    return result
    ####


def _nesc_witness_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Select NESC source-replay witnesses without unrelated endpoint rows."""

    result = dict(payload)
    for key in ("witnesses", "variant_witnesses", "graph_extension_witnesses"):
        rows = payload.get(key, [])
        if not isinstance(rows, list):
            raise ValueError(f"vehicle_execution_witnesses.yaml {key} must contain a list")
        result[key] = [
            row for row in rows if isinstance(row, dict) and isinstance(row.get("composition"), str) and row["composition"].startswith(_NESC_WITNESS_PREFIX)
        ]
    return result
    ####


def _copytree(target: Path, relative: str) -> None:
    """Copy one NESC-owned source family directory into package data."""

    source = ROOT / relative
    if not source.is_dir():
        raise ValueError(f"missing NESC package directory: {relative}")
    shutil.copytree(
        source,
        target / relative,
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".ruff_cache"),
    )
    ####


def extract(target: Path = TARGET) -> None:
    """Write the NESC-owned catalog, source evidence, and witness inputs."""

    # Package data is wholly generated. Recreate its exact tree rather than
    # merging onto an earlier result, so deleted source evidence or unrelated
    # child-deployment assets cannot linger in a later NESC wheel.
    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)

    vehicle_models = _read("verification/vehicle_models.yaml")
    vehicles = vehicle_models.get("vehicles")
    if not isinstance(vehicles, dict):
        raise ValueError("canonical vehicle_models registry must contain a vehicles mapping")
    # NESC is a source-replay family with no legacy vehicle-model record.  Do
    # not invent an unrelated force-model declaration just for a fragment.
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
    _write(target, "verification/vehicle_execution_witnesses.yaml", _nesc_witness_fragment(witnesses))
    for relative in (
        "examples/vehicle_composition/nesc_staged_source_replay_3dof_compose.yaml",
        "examples/vehicle_composition/nesc_staged_source_replay_pseudo6dof_compose.yaml",
        "examples/vehicle_composition/nesc_staged_source_replay_with_passive_child_pseudo6dof_compose.yaml",
    ):
        _copy(target, relative)

    lqr = _read("verification/lqr_scaling_profiles.yaml")
    profiles = lqr.get("profiles")
    if not isinstance(profiles, dict):
        raise ValueError("lqr_scaling_profiles.yaml profiles must contain a mapping")
    lqr["profiles"] = {identifier: row for identifier, row in profiles.items() if isinstance(row, dict) and row.get("vehicle") == FAMILY_ID}
    _write(target, "verification/lqr_scaling_profiles.yaml", lqr)

    _copytree(target, f"families/{FAMILY_ID}")
    _copy(target, "verification/daveml_nesc_reduction_qualification.json")

    provenance = {
        "schema": "taoryx.nesc-package-data/v1",
        "family_id": FAMILY_ID,
        "source_catalog_root": "verification/",
        "claim_boundary": "This package fragment mirrors only NESC-owned catalog rows and source replay evidence; repository-root catalogs remain canonical.",
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
    """Report every missing, stale, or unexpected NESC package-data file."""

    with tempfile.TemporaryDirectory(prefix="taoryx-nesc-plugin-assets-") as directory:
        expected_root = Path(directory) / "data"
        extract(expected_root)
        expected = _file_digests(expected_root)
    actual = _file_digests(target)
    missing = tuple(sorted(set(expected) - set(actual)))
    unexpected = tuple(sorted(set(actual) - set(expected)))
    changed = tuple(sorted(path for path in set(expected) & set(actual) if expected[path] != actual[path]))
    return (
        *(f"missing generated NESC asset: {path}" for path in missing),
        *(f"unexpected generated NESC asset: {path}" for path in unexpected),
        *(f"stale generated NESC asset: {path}" for path in changed),
    )
    ####


def main(argv: list[str] | None = None) -> int:
    """Regenerate or verify the complete NESC package-data boundary."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if package data differs from canonical NESC inputs")
    args = parser.parse_args(argv)
    if args.check:
        differences = check()
        if differences:
            print("NESC plug-in package data is stale:")
            print("\n".join(differences))
            return 1
        print("NESC plug-in package data: current")
        return 0
    extract()
    print("NESC plug-in package data: regenerated")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
