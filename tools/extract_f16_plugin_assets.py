"""Mechanically materialize the F-16 S.119 plug-in's owned package data.

The repository-root vehicle catalogs remain canonical.  This extractor writes
only the F-16 rows, source family, source package, local-screen witnesses, and
the one planning profile needed by ``taoryx-f16``.  Do not hand-edit the
generated package data.
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
TARGET = ROOT / "packages/taoryx-f16/src/taoryx_f16/data"
FAMILY_ID = "f16_s119"
_SOURCE_FAMILY_ID = "reference_f16_s119"
_WITNESS_PREFIX = "examples/vehicle_composition/f16_"


def _read(relative: str) -> dict[str, Any]:
    """Read one canonical mapping catalog."""

    payload = yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{relative} must contain a mapping")
    return payload
    ####


def _write(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write one deterministic F-16-owned YAML fragment."""

    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    ####


def _write_json(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write one deterministic F-16-owned JSON fragment."""

    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _copy(target: Path, relative: str) -> None:
    """Copy one F-16 source, evidence, or witness input into package data."""

    source = ROOT / relative
    destination = target / relative
    if not source.is_file():
        raise ValueError(f"missing F-16 package asset: {relative}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    ####


def _family_rows(payload: dict[str, Any], key: str) -> dict[str, Any]:
    """Filter a conventional list catalog to the F-16 composition family."""

    rows = payload.get(key)
    if not isinstance(rows, list):
        raise ValueError(f"{key} must contain a list")
    result = dict(payload)
    result[key] = [row for row in rows if isinstance(row, dict) and row.get("family_id") == FAMILY_ID]
    return result
    ####


def _witness_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Select F-16 composition witnesses without unrelated endpoint rows."""

    result = dict(payload)
    for key in ("witnesses", "variant_witnesses", "graph_extension_witnesses"):
        rows = payload.get(key, [])
        if not isinstance(rows, list):
            raise ValueError(f"vehicle_execution_witnesses.yaml {key} must contain a list")
        result[key] = [
            row for row in rows if isinstance(row, dict) and isinstance(row.get("composition"), str) and row["composition"].startswith(_WITNESS_PREFIX)
        ]
    return result
    ####


def _parity_witness_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Select F-16 replay evidence without borrowing another family trace."""

    witnesses = payload.get("witnesses")
    if not isinstance(witnesses, list):
        raise ValueError("vehicle_execution_parity_witnesses.yaml witnesses must contain a list")
    result = dict(payload)
    result["witnesses"] = [
        row
        for row in witnesses
        if isinstance(row, dict) and isinstance(row.get("composition"), str) and row["composition"].startswith(_WITNESS_PREFIX)
    ]
    return result
    ####


def _write_planning_profile(target: Path) -> None:
    """Carry the sole F-16 route-planning profile used by its adapter."""

    payload = _read("verification/powered_fixed_wing_mission_profiles.yaml")
    profiles = payload.get("profiles")
    baselines = payload.get("baselines")
    if not isinstance(profiles, dict) or not isinstance(baselines, dict):
        raise ValueError("powered_fixed_wing_mission_profiles.yaml must contain profiles and baselines mappings")
    profile_id = "f16-subsonic"
    if profile_id not in profiles or profile_id not in baselines:
        raise ValueError("canonical F-16 powered-fixed-wing planning profile is missing")
    payload["profiles"] = {profile_id: profiles[profile_id]}
    payload["baselines"] = {profile_id: baselines[profile_id]}
    _write(target, "verification/powered_fixed_wing_mission_profiles.yaml", payload)
    ####


def extract(target: Path = TARGET) -> None:
    """Write the F-16 catalog fragments, source package, evidence, and inputs."""

    # Package data is wholly generated. Recreate its exact tree rather than
    # merging onto an earlier result, so deleted canonical assets cannot linger
    # in a later F-16 wheel.
    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)

    # F-16 is a package-backed source family rather than the legacy flat
    # vehicle-model registry format.  Preserve the catalog envelope without
    # inventing a separate force-model record.
    vehicle_models = _read("verification/vehicle_models.yaml")
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
    fragment = _witness_fragment(witnesses)
    _write(target, "verification/vehicle_execution_witnesses.yaml", fragment)
    _write(
        target,
        "verification/vehicle_execution_parity_witnesses.yaml",
        _parity_witness_fragment(_read("verification/vehicle_execution_parity_witnesses.yaml")),
    )
    witness_paths = {
        row["composition"]
        for key in ("witnesses", "variant_witnesses", "graph_extension_witnesses")
        for row in fragment.get(key, [])
        if isinstance(row, dict) and isinstance(row.get("composition"), str)
    }
    for relative in sorted(witness_paths):
        _copy(target, relative)

    lqr = _read("verification/lqr_scaling_profiles.yaml")
    profiles = lqr.get("profiles")
    if not isinstance(profiles, dict):
        raise ValueError("lqr_scaling_profiles.yaml profiles must contain a mapping")
    lqr["profiles"] = {identifier: row for identifier, row in profiles.items() if isinstance(row, dict) and row.get("vehicle") == FAMILY_ID}
    _write(target, "verification/lqr_scaling_profiles.yaml", lqr)
    _write_planning_profile(target)

    source_family = ROOT / "families" / _SOURCE_FAMILY_ID
    if not source_family.is_dir():
        raise ValueError(f"missing F-16 source family directory: {source_family}")
    shutil.copytree(
        source_family,
        target / "families" / _SOURCE_FAMILY_ID,
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"),
    )
    _copy(target, "verification/daveml_f16_equilibrium_trim_evidence.json")
    _copy(target, "verification/f16_runtime_linearization_evidence.json")
    _copy(target, "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml")
    _copy(target, "resources/aerospace/daveml/nesc-model-catalog-v1.0/qualified/f16-s119/f16-s119-reference-v0.7.txair")

    _write_json(
        target,
        "package_data_provenance.json",
        {
            "schema": "taoryx.f16-package-data/v1",
            "family_id": FAMILY_ID,
            "source_family_id": _SOURCE_FAMILY_ID,
            "source_catalog_root": "verification/",
            "claim_boundary": "This package fragment mirrors only F-16-owned catalog rows, source assets, and local-screen evidence; repository-root catalogs remain canonical.",
        },
    )
    ####


def _file_digests(root: Path) -> dict[str, str]:
    """Return a content-addressed inventory for one generated data tree."""

    if not root.is_dir():
        return {}
    return {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(root.rglob("*")) if path.is_file()}
    ####


def check(target: Path = TARGET) -> tuple[str, ...]:
    """Report every missing, stale, or unexpected F-16 package-data file."""

    with tempfile.TemporaryDirectory(prefix="taoryx-f16-plugin-assets-") as directory:
        expected_root = Path(directory) / "data"
        extract(expected_root)
        expected = _file_digests(expected_root)
    actual = _file_digests(target)
    missing = tuple(sorted(set(expected) - set(actual)))
    unexpected = tuple(sorted(set(actual) - set(expected)))
    changed = tuple(sorted(path for path in set(expected) & set(actual) if expected[path] != actual[path]))
    return (
        *(f"missing generated F-16 asset: {path}" for path in missing),
        *(f"unexpected generated F-16 asset: {path}" for path in unexpected),
        *(f"stale generated F-16 asset: {path}" for path in changed),
    )
    ####


def main(argv: list[str] | None = None) -> int:
    """Regenerate or verify the complete F-16 package-data boundary."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if package data differs from canonical F-16 inputs")
    args = parser.parse_args(argv)
    if args.check:
        differences = check()
        if differences:
            print("F-16 plug-in package data is stale:")
            print("\n".join(differences))
            return 1
        print("F-16 plug-in package data: current")
        return 0
    extract()
    print("F-16 plug-in package data: regenerated")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
