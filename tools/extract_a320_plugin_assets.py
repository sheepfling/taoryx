"""Materialize the A320/OpenAP plug-in's owned package data.

The repository-root catalogs remain canonical.  This extractor carries just
the A320 composition family, its OpenAP/JSBSim surrogate source material,
route-planning profile, evidence, and focused witnesses into the independent
``taoryx-a320`` distribution.  The runtime retains its one integrity-pinned
source archive without duplicating the archive's unrelated exploded contents.
Do not hand-edit the generated package data.
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
TARGET = ROOT / "packages/taoryx-a320/src/taoryx_a320/data"
FAMILY_ID = "a320_openap_3dof"
_WITNESS_PREFIX = "examples/vehicle_composition/a320_"
_CORPUS_ROOT = "resources/aerospace/daveml/taoryx-corpus-v1.1"
_A320_CORPUS_INPUTS = (
    f"{_CORPUS_ROOT}/corpus.zip",
    f"{_CORPUS_ROOT}/NOTICE.md",
)


def _read(relative: str) -> dict[str, Any]:
    """Read one canonical YAML catalog."""

    payload = yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{relative} must contain a mapping")
    return payload
    ####


def _write(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write one deterministic A320-owned catalog fragment."""

    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    ####


def _write_json(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write one deterministic JSON package fragment."""

    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _copy(target: Path, relative: str) -> None:
    """Copy one checked-in source, evidence, or witness asset."""

    source = ROOT / relative
    destination = target / relative
    if not source.is_file():
        raise ValueError(f"missing A320 package asset: {relative}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    ####


def _copytree(target: Path, relative: str) -> None:
    """Copy one A320-owned metadata or source directory."""

    source = ROOT / relative
    if not source.is_dir():
        raise ValueError(f"missing A320 package directory: {relative}")
    shutil.copytree(
        source,
        target / relative,
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"),
    )
    ####


def _family_rows(payload: dict[str, Any], key: str) -> dict[str, Any]:
    """Filter a conventional list catalog to the A320 composition family."""

    rows = payload.get(key)
    if not isinstance(rows, list):
        raise ValueError(f"{key} must contain a list")
    result = dict(payload)
    result[key] = [row for row in rows if isinstance(row, dict) and row.get("family_id") == FAMILY_ID]
    return result
    ####


def _witness_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Select only A320 endpoint, variant, and graph witnesses."""

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
    """Select A320 replay evidence without borrowing another family trace."""

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
    """Carry the sole A320 racetrack-planning profile and baseline."""

    payload = _read("verification/powered_fixed_wing_mission_profiles.yaml")
    profiles = payload.get("profiles")
    baselines = payload.get("baselines")
    if not isinstance(profiles, dict) or not isinstance(baselines, dict):
        raise ValueError("powered_fixed_wing_mission_profiles.yaml must contain profiles and baselines mappings")
    profile_id = "a320-cruise"
    if profile_id not in profiles or profile_id not in baselines:
        raise ValueError("canonical A320 powered-fixed-wing planning profile is missing")
    payload["profiles"] = {profile_id: profiles[profile_id]}
    payload["baselines"] = {profile_id: baselines[profile_id]}
    _write(target, "verification/powered_fixed_wing_mission_profiles.yaml", payload)
    ####


def extract(target: Path = TARGET) -> None:
    """Write the A320-owned catalog, evidence, source assets, and witnesses."""

    # Package data is wholly generated. Recreate its exact tree rather than
    # merging onto an earlier result, so deleted canonical assets or unrelated
    # exploded corpus material cannot linger in a later A320 wheel.
    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)

    # A320 uses data-evidence manifests rather than the legacy flat model
    # registry.  Keep the envelope without inventing an unrelated model row.
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

    witnesses = _witness_fragment(_read("verification/vehicle_execution_witnesses.yaml"))
    _write(target, "verification/vehicle_execution_witnesses.yaml", witnesses)
    _write(
        target,
        "verification/vehicle_execution_parity_witnesses.yaml",
        _parity_witness_fragment(_read("verification/vehicle_execution_parity_witnesses.yaml")),
    )
    witness_paths = {
        row["composition"]
        for key in ("witnesses", "variant_witnesses", "graph_extension_witnesses")
        for row in witnesses.get(key, [])
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

    for relative in (
        "families/a320_openap_3dof",
        "families/a320_openap_jsbsim_pseudo6dof",
        "families/reference_a320",
    ):
        _copytree(target, relative)
    # The models hash-verify and read named members from the immutable outer
    # archive. Its exploded tree contains foreign F-16, HL-20, NESC, and other
    # JSBSim assets, so the A320 wheel owns only the required archive and its
    # redistribution notice. A320 collection manifests carry the member-level
    # source provenance and hashes.
    for relative in _A320_CORPUS_INPUTS:
        _copy(target, relative)
    for relative in (
        "verification/daveml_a320_openap_integration.json",
        "verification/daveml_a320_matched_comparison.json",
    ):
        _copy(target, relative)

    _write_json(
        target,
        "package_data_provenance.json",
        {
            "schema": "taoryx.a320-package-data/v1",
            "family_id": FAMILY_ID,
            "source_catalog_root": "verification/",
            "source_archive": {
                "path": f"{_CORPUS_ROOT}/corpus.zip",
                "sha256": "9dbb38f23924f2cd2775cb37b1b87476d9ddda672ecab9b84828b253931ebd31",
                "storage_boundary": "archive_and_attribution_notice_only",
            },
            "claim_boundary": "This package fragment mirrors only A320/OpenAP catalog rows, source assets, evidence, and focused witnesses; repository-root catalogs remain canonical and unrelated exploded corpus members are excluded.",
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
    """Report every missing, stale, or unexpected A320 package-data file."""

    with tempfile.TemporaryDirectory(prefix="taoryx-a320-plugin-assets-") as directory:
        expected_root = Path(directory) / "data"
        extract(expected_root)
        expected = _file_digests(expected_root)
    actual = _file_digests(target)
    missing = tuple(sorted(set(expected) - set(actual)))
    unexpected = tuple(sorted(set(actual) - set(expected)))
    changed = tuple(sorted(path for path in set(expected) & set(actual) if expected[path] != actual[path]))
    return (
        *(f"missing generated A320 asset: {path}" for path in missing),
        *(f"unexpected generated A320 asset: {path}" for path in unexpected),
        *(f"stale generated A320 asset: {path}" for path in changed),
    )
    ####


def main(argv: list[str] | None = None) -> int:
    """Regenerate or verify the complete A320 package-data boundary."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if package data differs from canonical A320 inputs")
    args = parser.parse_args(argv)
    if args.check:
        differences = check()
        if differences:
            print("A320 plug-in package data is stale:")
            print("\n".join(differences))
            return 1
        print("A320 plug-in package data: current")
        return 0
    extract()
    print("A320 plug-in package data: regenerated")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
