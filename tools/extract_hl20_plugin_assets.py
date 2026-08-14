"""Materialize the HL-20 plug-in's owned package data.

Repository-root catalogs remain canonical. This extractor transfers the HL-20
composition family, DAVE-ML source fixture, evidence, and local-screen
witnesses into ``taoryx-hl20`` so local controls do not depend on the
reference-model aggregate. The optional reachability package consumes the same
source assets but owns its distinct booster-release and glide-energy overlay
catalogs, bindings, requests, and witnesses.
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
TARGET = ROOT / "packages/taoryx-hl20/src/taoryx_hl20/data"
FAMILY_ID = "hl20_mod_k"
_SOURCE_FAMILY_ID = "reference_hl20_mod_k"
_WITNESS_PREFIX = "examples/vehicle_composition/hl20_"
_REACHABILITY_MISSION_IDS = frozenset(
    {"hl20_source_booster_release_replay_v1", "lifting_body_glide_energy_management_v1"}
)
_REACHABILITY_INITIALIZATION_IDS = frozenset({"source_booster_launch", "high_altitude_release"})
_REACHABILITY_SEGMENT_IDS = frozenset(
    {
        "source_booster_release",
        "source_scheduled_bank",
        "source_ground_contact",
        "trim_capture",
        "glide_bank_reversal",
        "energy_handoff",
    }
)
_REACHABILITY_COMPOSITIONS = frozenset(
    {
        "examples/vehicle_composition/hl20_source_booster_release_replay_3dof_compose.yaml",
        "examples/vehicle_composition/hl20_source_booster_release_replay_pseudo6dof_compose.yaml",
        "examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml",
        "examples/vehicle_composition/hl20_glide_energy_capability_pseudo6dof_compose.yaml",
    }
)


def _read(relative: str) -> dict[str, Any]:
    """Read one canonical YAML catalog as a mutable mapping."""

    payload = yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{relative} must contain a mapping")
    return payload
    ####


def _write(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write one deterministic HL-20-owned YAML catalog fragment."""

    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    ####


def _write_json(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write one deterministic HL-20-owned JSON fragment."""

    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _copy(target: Path, relative: str) -> None:
    """Copy one HL-20-owned source, evidence, or witness asset."""

    source = ROOT / relative
    destination = target / relative
    if not source.is_file():
        raise ValueError(f"missing HL-20 package asset: {relative}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    ####


def _copytree(target: Path, relative: str) -> None:
    """Copy one HL-20-owned source or evidence directory."""

    source = ROOT / relative
    if not source.is_dir():
        raise ValueError(f"missing HL-20 package directory: {relative}")
    shutil.copytree(
        source,
        target / relative,
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".ruff_cache"),
    )
    ####


def _family_rows(payload: dict[str, Any], key: str) -> dict[str, Any]:
    """Keep rows owned by the HL-20 composition family."""

    rows = payload.get(key)
    if not isinstance(rows, list):
        raise ValueError(f"{key} must contain a list")
    result = dict(payload)
    result[key] = [row for row in rows if isinstance(row, dict) and row.get("family_id") == FAMILY_ID]
    return result
    ####


def _hl20_base_composition_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Keep local HL-20 rows while leaving reachability endpoints to their owner."""

    result = _family_rows(payload, "vehicles")
    vehicles = result["vehicles"]
    if not isinstance(vehicles, list) or len(vehicles) != 1 or not isinstance(vehicles[0], dict):
        raise ValueError("HL-20 composition registry must contain one family declaration")
    vehicle = dict(vehicles[0])
    for key, excluded in (
        ("initialization_contracts", _REACHABILITY_INITIALIZATION_IDS),
        ("segment_contracts", _REACHABILITY_SEGMENT_IDS),
        ("mission_templates", _REACHABILITY_MISSION_IDS),
    ):
        rows = vehicle.get(key)
        if not isinstance(rows, list):
            raise ValueError(f"HL-20 composition registry {key} must contain a list")
        vehicle[key] = [row for row in rows if not isinstance(row, dict) or row.get("id") not in excluded]
    result["vehicles"] = [vehicle]
    return result
    ####


def _hl20_base_execution_binding_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Keep bindings implemented by HL-20 rather than its optional overlay."""

    result = _family_rows(payload, "bindings")
    bindings = result["bindings"]
    if not isinstance(bindings, list):
        raise ValueError("vehicle_execution_bindings.yaml bindings must contain a list")
    result["bindings"] = [
        row
        for row in bindings
        if not isinstance(row, dict) or row.get("mission") not in _REACHABILITY_MISSION_IDS
    ]
    return result
    ####


def _witness_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Select HL-20 endpoint, variant, and graph witnesses."""

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
            and row["composition"].startswith(_WITNESS_PREFIX)
            and row["composition"] not in _REACHABILITY_COMPOSITIONS
        ]
    return result
    ####


def _parity_witness_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Select HL-20 replay evidence without borrowing another family trace."""

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


def _write_lqr_scaling_profiles(target: Path) -> None:
    """Carry exactly the HL-20 controller-normalization profiles."""

    payload = _read("verification/lqr_scaling_profiles.yaml")
    profiles = payload.get("profiles")
    if not isinstance(profiles, dict):
        raise ValueError("lqr_scaling_profiles.yaml profiles must contain a mapping")
    payload["profiles"] = {identifier: row for identifier, row in profiles.items() if isinstance(row, dict) and row.get("vehicle") == FAMILY_ID}
    _write(target, "verification/lqr_scaling_profiles.yaml", payload)
    ####


def _write_vehicle_models(target: Path) -> None:
    """Write the required empty legacy-model fragment for source-manifest HL-20.

    HL-20 is represented by its source-family manifest rather than a legacy
    ``vehicle_models`` record, but every independently loadable catalog root
    still carries the standard mapping-shaped file.
    """

    payload = _read("verification/vehicle_models.yaml")
    vehicles = payload.get("vehicles")
    if not isinstance(vehicles, dict):
        raise ValueError("canonical vehicle_models registry must contain a vehicles mapping")
    payload["vehicles"] = {}
    _write(target, "verification/vehicle_models.yaml", payload)
    ####


def extract(target: Path = TARGET) -> None:
    """Write HL-20 catalog rows, source fixture, evidence, and witnesses."""

    # Package data is wholly generated. Recreate its exact tree rather than
    # merging onto an earlier result, so deleted source evidence or unrelated
    # assets cannot linger in a later HL-20 wheel.
    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)

    _write_vehicle_models(target)

    _write(
        target,
        "verification/vehicle_composition_registry.yaml",
        _hl20_base_composition_fragment(_read("verification/vehicle_composition_registry.yaml")),
    )
    _write(
        target,
        "verification/vehicle_execution_bindings.yaml",
        _hl20_base_execution_binding_fragment(_read("verification/vehicle_execution_bindings.yaml")),
    )

    for relative, key in (
        ("verification/horizontal_fidelity_registry.yaml", "families"),
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
    _write_lqr_scaling_profiles(target)

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

    _copytree(target, f"families/{_SOURCE_FAMILY_ID}")

    for relative in (
        "resources/aerospace/daveml/nesc-model-catalog-v1.0/qualified/hl20-mod-k/hl20-mod-k-unpowered-v0.10.txair",
        "verification/alpha3_cross_fidelity/hl20_mod_k.json",
        "verification/alpha3_hl20_development",
        "verification/alpha3_hl20_direct_wrench",
        "verification/alpha3_hl20_fidelity",
        "verification/alpha3_hl20_r1",
        "verification/alpha3_hl20_source_allocation",
        "verification/alpha3_hl20_source_control",
        "verification/alpha3_hl20_source_surface_replay",
        "verification/daveml_hl20_linearization_evidence.json",
        "verification/daveml_hl20_load_evidence.json",
        "verification/daveml_hl20_scenario_evidence.json",
        "verification/daveml_hl20_trim_evidence.json",
        "verification/hl20_ca_hi_qualification.json",
        "examples/showcases/hl20_california_to_hawaii",
    ):
        source = ROOT / relative
        if source.is_dir():
            _copytree(target, relative)
        else:
            _copy(target, relative)

    _write_json(
        target,
        "package_data_provenance.json",
        {
            "schema": "taoryx.hl20-package-data/v1",
            "family_id": FAMILY_ID,
            "source_family_id": _SOURCE_FAMILY_ID,
            "source_catalog_root": "verification/",
            "claim_boundary": (
                "This package fragment mirrors only HL-20 catalog rows, DAVE-ML source fixture, evidence, and focused "
                "local-screen witnesses; repository-root catalogs remain canonical. The reachability plug-in may consume "
                "the source assets but owns its booster-release and glide-energy overlay catalogs, bindings, requests, "
                "and witnesses."
            ),
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
    """Report every missing, stale, or unexpected HL-20 package-data file."""

    with tempfile.TemporaryDirectory(prefix="taoryx-hl20-plugin-assets-") as directory:
        expected_root = Path(directory) / "data"
        extract(expected_root)
        expected = _file_digests(expected_root)
    actual = _file_digests(target)
    missing = tuple(sorted(set(expected) - set(actual)))
    unexpected = tuple(sorted(set(actual) - set(expected)))
    changed = tuple(sorted(path for path in set(expected) & set(actual) if expected[path] != actual[path]))
    return (
        *(f"missing generated HL-20 asset: {path}" for path in missing),
        *(f"unexpected generated HL-20 asset: {path}" for path in unexpected),
        *(f"stale generated HL-20 asset: {path}" for path in changed),
    )
    ####


def main(argv: list[str] | None = None) -> int:
    """Regenerate or verify the complete HL-20 package-data boundary."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if package data differs from canonical HL-20 inputs")
    args = parser.parse_args(argv)
    if args.check:
        differences = check()
        if differences:
            print("HL-20 plug-in package data is stale:")
            print("\n".join(differences))
            return 1
        print("HL-20 plug-in package data: current")
        return 0
    extract()
    print("HL-20 plug-in package data: regenerated")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
