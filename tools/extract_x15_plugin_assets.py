"""Materialize the X-15 plug-in's owned package data.

The repository-root catalogs remain canonical.  This extractor transfers the
X-15 composition family, source fixtures, source tables, evidence, and local
screen witnesses into ``taoryx-x15`` so its local control screens never need
the reference-model aggregate. The optional reachability plug-in consumes the
same package-owned source assets but owns its distinct staged-mission catalog,
bindings, and witnesses.
Do not hand-edit generated files below ``packages/taoryx-x15/.../data``.
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
TARGET = ROOT / "packages/taoryx-x15/src/taoryx_x15/data"
FAMILY_ID = "x15"
_WITNESS_PREFIX = "examples/vehicle_composition/x15_"
_REACHABILITY_MISSION_ID = "x15_staged_booster_reachability_v1"
_REACHABILITY_INITIALIZATION_IDS = frozenset({"staged_booster_launch"})
_REACHABILITY_SEGMENT_IDS = frozenset(
    {"booster_powered", "booster_coast_release", "unpowered_glide_handoff", "impact_witness"}
)
_REACHABILITY_COMPOSITIONS = frozenset(
    {
        "examples/vehicle_composition/x15_staged_booster_reachability_3dof_compose.yaml",
        "examples/vehicle_composition/x15_staged_booster_reachability_pseudo6dof_compose.yaml",
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
    """Write one deterministic X-15-owned YAML catalog fragment."""

    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    ####


def _write_json(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write one deterministic X-15-owned JSON fragment."""

    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _copy(target: Path, relative: str) -> None:
    """Copy one X-15-owned source, evidence, or witness asset."""

    source = ROOT / relative
    destination = target / relative
    if not source.is_file():
        raise ValueError(f"missing X-15 package asset: {relative}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    ####


def _copytree(target: Path, relative: str) -> None:
    """Copy one X-15-owned source or evidence directory."""

    source = ROOT / relative
    if not source.is_dir():
        raise ValueError(f"missing X-15 package directory: {relative}")
    shutil.copytree(
        source,
        target / relative,
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".ruff_cache"),
    )
    ####


def _family_rows(payload: dict[str, Any], key: str) -> dict[str, Any]:
    """Keep rows owned by the X-15 composition family."""

    rows = payload.get(key)
    if not isinstance(rows, list):
        raise ValueError(f"{key} must contain a list")
    result = dict(payload)
    result[key] = [row for row in rows if isinstance(row, dict) and row.get("family_id") == FAMILY_ID]
    return result
    ####


def _x15_base_composition_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Keep local X-15 composition rows while leaving the overlay to reachability."""

    result = _family_rows(payload, "vehicles")
    vehicles = result["vehicles"]
    if not isinstance(vehicles, list) or len(vehicles) != 1 or not isinstance(vehicles[0], dict):
        raise ValueError("X-15 composition registry must contain one family declaration")
    vehicle = dict(vehicles[0])
    for key, excluded in (
        ("initialization_contracts", _REACHABILITY_INITIALIZATION_IDS),
        ("segment_contracts", _REACHABILITY_SEGMENT_IDS),
        ("mission_templates", {_REACHABILITY_MISSION_ID}),
    ):
        rows = vehicle.get(key)
        if not isinstance(rows, list):
            raise ValueError(f"X-15 composition registry {key} must contain a list")
        vehicle[key] = [row for row in rows if not isinstance(row, dict) or row.get("id") not in excluded]
    result["vehicles"] = [vehicle]
    return result
    ####


def _x15_base_execution_binding_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Keep bindings implemented by the X-15 package, not its optional overlay."""

    result = _family_rows(payload, "bindings")
    bindings = result["bindings"]
    if not isinstance(bindings, list):
        raise ValueError("vehicle_execution_bindings.yaml bindings must contain a list")
    result["bindings"] = [
        row
        for row in bindings
        if not isinstance(row, dict) or row.get("mission") != _REACHABILITY_MISSION_ID
    ]
    return result
    ####


def _witness_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Select only X-15 endpoint, variant, and graph witnesses."""

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
    """Select X-15 replay evidence without borrowing another family trace."""

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
    """Carry exactly the X-15 controller-normalization profiles."""

    payload = _read("verification/lqr_scaling_profiles.yaml")
    profiles = payload.get("profiles")
    if not isinstance(profiles, dict):
        raise ValueError("lqr_scaling_profiles.yaml profiles must contain a mapping")
    payload["profiles"] = {identifier: row for identifier, row in profiles.items() if isinstance(row, dict) and row.get("vehicle") == FAMILY_ID}
    _write(target, "verification/lqr_scaling_profiles.yaml", payload)
    ####


def _write_vehicle_models(target: Path) -> None:
    """Retain the one X-15 model record required by family manifests."""

    payload = _read("verification/vehicle_models.yaml")
    vehicles = payload.get("vehicles")
    if not isinstance(vehicles, dict) or FAMILY_ID not in vehicles:
        raise ValueError("canonical vehicle_models registry has no X-15 record")
    payload["vehicles"] = {FAMILY_ID: vehicles[FAMILY_ID]}
    _write(target, "verification/vehicle_models.yaml", payload)
    ####


def extract(target: Path = TARGET) -> None:
    """Write X-15 catalog rows, source assets, evidence, and witnesses."""

    # Package data is wholly generated. Recreate its exact tree rather than
    # merging onto an earlier result, so deleted source evidence or unrelated
    # assets cannot linger in a later X-15 wheel.
    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)

    _write_vehicle_models(target)

    _write(target, "verification/vehicle_composition_registry.yaml", _x15_base_composition_fragment(_read("verification/vehicle_composition_registry.yaml")))
    _write(target, "verification/vehicle_execution_bindings.yaml", _x15_base_execution_binding_fragment(_read("verification/vehicle_execution_bindings.yaml")))

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

    for relative in (
        "examples/generated/vehicles/x15_canonical_research_6dof.prb",
        "examples/showcases/x15_6dof_research",
        "examples/showcases/x15_rocket_to_hawaii",
        "tests/fixtures/x15_coherent_6dof_public_research_v1",
        "verification/alpha3_cross_fidelity/x15.json",
        "verification/alpha3_x15_direct_wrench",
        "verification/alpha3_x15_r1",
        "verification/generated/x15_physical_lqr_readiness.json",
        "verification/x15_fidelity_ladder",
        "verification/x15_maneuver_catalog.yaml",
        "verification/x15_trim_adjudication.yaml",
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
                "schema": "taoryx.x15-package-data/v1",
            "family_id": FAMILY_ID,
            "source_catalog_root": "verification/",
            "claim_boundary": (
                "This package fragment mirrors only X-15 catalog rows, source fixtures, evidence, and focused "
                "local-screen witnesses; repository-root catalogs remain canonical. The reachability plug-in "
                "may consume the staged X-15 source assets but owns its distinct mission overlay, bindings, "
                "and staged composition witnesses."
            ),
        },
    )
    ####


def _file_digests(root: Path) -> dict[str, str]:
    """Return a content-addressed inventory for one generated data tree."""

    if not root.is_dir():
        return {}
    return {
        path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(root.rglob("*"))
        if path.is_file() and ".ruff_cache" not in path.relative_to(root).parts
    }
    ####


def check(target: Path = TARGET) -> tuple[str, ...]:
    """Report every missing, stale, or unexpected X-15 package-data file."""

    with tempfile.TemporaryDirectory(prefix="taoryx-x15-plugin-assets-") as directory:
        expected_root = Path(directory) / "data"
        extract(expected_root)
        expected = _file_digests(expected_root)
    actual = _file_digests(target)
    missing = tuple(sorted(set(expected) - set(actual)))
    unexpected = tuple(sorted(set(actual) - set(expected)))
    changed = tuple(sorted(path for path in set(expected) & set(actual) if expected[path] != actual[path]))
    return (
        *(f"missing generated X-15 asset: {path}" for path in missing),
        *(f"unexpected generated X-15 asset: {path}" for path in unexpected),
        *(f"stale generated X-15 asset: {path}" for path in changed),
    )
    ####


def main(argv: list[str] | None = None) -> int:
    """Regenerate or verify the complete X-15 package-data boundary."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if package data differs from canonical X-15 inputs")
    args = parser.parse_args(argv)
    if args.check:
        differences = check()
        if differences:
            print("X-15 plug-in package data is stale:")
            print("\n".join(differences))
            return 1
        print("X-15 plug-in package data: current")
        return 0
    extract()
    print("X-15 plug-in package data: regenerated")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
