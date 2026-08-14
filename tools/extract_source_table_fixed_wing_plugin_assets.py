"""Materialize source-table X8/B747 package-owned assets.

The canonical repository registries remain the editing authority. This tool
copies only the two shared source-table fixed-wing families into their
installable plug-in so focused discovery, composition, and local control
screens do not import or read the compatibility aggregate.
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
TARGET = ROOT / "packages/taoryx-source-table-fixed-wing/src/taoryx_source_table_fixed_wing/data"
FAMILY_IDS = frozenset({"skywalker_x8", "b747"})
PROFILE_IDS = frozenset({"x8-cruise", "b747-cruise"})
_WITNESS_PREFIXES = (
    "examples/vehicle_composition/x8_",
    "examples/vehicle_composition/b747_",
)
_BUNDLE = "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1"


def _read(relative: str) -> dict[str, Any]:
    """Read one canonical YAML catalog as a mutable mapping."""

    payload = yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{relative} must contain a mapping")
    return payload
    ####


def _write(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write one deterministic source-table package fragment."""

    destination = target / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    ####


def _write_json(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write one deterministic JSON ownership record or manifest."""

    destination = target / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _copy(target: Path, relative: str) -> None:
    """Copy one package-owned source, witness, or table file."""

    source = ROOT / relative
    destination = target / relative
    if not source.is_file():
        raise ValueError(f"missing source-table fixed-wing package asset: {relative}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    ####


def _copytree(target: Path, relative: str) -> None:
    """Copy one package-owned source directory."""

    source = ROOT / relative
    if not source.is_dir():
        raise ValueError(f"missing source-table fixed-wing package directory: {relative}")
    shutil.copytree(
        source,
        target / relative,
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"),
    )
    ####


def _family_rows(payload: dict[str, Any], key: str) -> dict[str, Any]:
    """Keep conventional list-registry rows owned by X8 or B747."""

    rows = payload.get(key)
    if not isinstance(rows, list):
        raise ValueError(f"{key} must contain a list")
    result = dict(payload)
    result[key] = [row for row in rows if isinstance(row, dict) and row.get("family_id") in FAMILY_IDS]
    return result
    ####


def _witness_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Keep X8/B747 endpoint, variant, and graph witnesses only."""

    result = dict(payload)
    for key in ("witnesses", "variant_witnesses", "graph_extension_witnesses"):
        rows = payload.get(key, [])
        if not isinstance(rows, list):
            raise ValueError(f"vehicle_execution_witnesses.yaml {key} must contain a list")
        result[key] = [
            row for row in rows if isinstance(row, dict) and isinstance(row.get("composition"), str) and row["composition"].startswith(_WITNESS_PREFIXES)
        ]
    return result
    ####


def _parity_witness_fragment(payload: dict[str, Any]) -> dict[str, Any]:
    """Keep X8/B747 replay evidence without unrelated parity traces."""

    witnesses = payload.get("witnesses")
    if not isinstance(witnesses, list):
        raise ValueError("vehicle_execution_parity_witnesses.yaml witnesses must contain a list")
    result = dict(payload)
    result["witnesses"] = [
        row
        for row in witnesses
        if isinstance(row, dict) and isinstance(row.get("composition"), str) and row["composition"].startswith(_WITNESS_PREFIXES)
    ]
    return result
    ####


def _witness_paths(payload: dict[str, Any]) -> tuple[str, ...]:
    """Return every checked-in composition used by the selected witnesses."""

    fragment = _witness_fragment(payload)
    paths = {
        row["composition"]
        for key in ("witnesses", "variant_witnesses", "graph_extension_witnesses")
        for row in fragment.get(key, [])
        if isinstance(row, dict) and isinstance(row.get("composition"), str)
    }
    return tuple(sorted(paths))
    ####


def _write_vehicle_models(target: Path) -> None:
    """Carry the two physical source-table model rows without peer baggage."""

    payload = _read("verification/vehicle_models.yaml")
    vehicles = payload.get("vehicles")
    if not isinstance(vehicles, dict) or not FAMILY_IDS.issubset(vehicles):
        raise ValueError("canonical vehicle model registry is missing source-table fixed-wing records")
    payload["vehicles"] = {identifier: vehicles[identifier] for identifier in sorted(FAMILY_IDS)}
    _write(target, "verification/vehicle_models.yaml", payload)
    ####


def _write_pseudo_profiles(target: Path) -> None:
    """Carry profile rows and bindings for both source-table family IDs."""

    payload = _read("verification/pseudo6dof_profiles.yaml")
    for key in ("profiles", "direct_wrench_profiles", "surface_allocation_profiles", "bindings"):
        payload = _family_rows(payload, key)
    _write(target, "verification/pseudo6dof_profiles.yaml", payload)
    ####


def _write_maturity(target: Path) -> None:
    """Carry only endpoint maturity rows owned by X8 and B747."""

    payload = _read("verification/vehicle_maturity_registry.yaml")
    records = payload.get("records")
    if not isinstance(records, list):
        raise ValueError("vehicle maturity registry must contain records")
    payload["records"] = [row for row in records if isinstance(row, dict) and row.get("composition_family_id") in FAMILY_IDS]
    _write(target, "verification/vehicle_maturity_registry.yaml", payload)
    ####


def _write_lqr_profiles(target: Path) -> None:
    """Carry the normalization profiles for both local controller families."""

    payload = _read("verification/lqr_scaling_profiles.yaml")
    profiles = payload.get("profiles")
    if not isinstance(profiles, dict):
        raise ValueError("lqr scaling profile catalog must contain profiles")
    payload["profiles"] = {identifier: row for identifier, row in profiles.items() if isinstance(row, dict) and row.get("vehicle") in FAMILY_IDS}
    _write(target, "verification/lqr_scaling_profiles.yaml", payload)
    ####


def _write_powered_fixed_wing_profiles(target: Path) -> None:
    """Carry exactly the X8/B747 route-planning profiles and baselines."""

    payload = _read("verification/powered_fixed_wing_mission_profiles.yaml")
    profiles = payload.get("profiles")
    baselines = payload.get("baselines")
    if not isinstance(profiles, dict) or not isinstance(baselines, dict):
        raise ValueError("powered fixed-wing mission profiles must contain profiles and baselines mappings")
    if not PROFILE_IDS.issubset(profiles) or not PROFILE_IDS.issubset(baselines):
        raise ValueError("canonical source-table fixed-wing planning profiles are missing")
    payload["profiles"] = {identifier: profiles[identifier] for identifier in sorted(PROFILE_IDS)}
    payload["baselines"] = {identifier: baselines[identifier] for identifier in sorted(PROFILE_IDS)}
    _write(target, "verification/powered_fixed_wing_mission_profiles.yaml", payload)
    ####


def _write_language_backed_route_assets(target: Path) -> None:
    """Carry selected route contracts and every native problem they name."""

    missions = _read("verification/family_qualification_missions.yaml")
    rows = missions.get("missions")
    if not isinstance(rows, list):
        raise ValueError("family qualification mission catalog must contain a missions list")
    selected_missions = [row for row in rows if isinstance(row, dict) and row.get("family") in FAMILY_IDS]
    if not selected_missions:
        raise ValueError("canonical mission catalog has no source-table fixed-wing rows")
    missions["missions"] = selected_missions
    _write(target, "verification/family_qualification_missions.yaml", missions)

    routes = _read("verification/racetrack_templates.yaml")
    bindings = routes.get("bindings")
    if not isinstance(bindings, dict):
        raise ValueError("racetrack template catalog must contain bindings")
    routes["bindings"] = {
        identifier: binding for identifier, binding in bindings.items() if isinstance(binding, dict) and binding.get("vehicle_id") in FAMILY_IDS
    }
    _write(target, "verification/racetrack_templates.yaml", routes)

    source_paths = {str(path) for mission in selected_missions for path in (*mission.get("tables", ()), mission.get("problem")) if isinstance(path, str)}
    for relative in sorted(source_paths):
        _copy(target, relative)
    ####


def _write_source_table_catalog_fragments(target: Path) -> None:
    """Carry provenance indexes for only the X8/B747 table inventory."""

    relative = f"{_BUNDLE}/tables"
    catalog_path = ROOT / relative / "catalog.yaml"
    catalog = yaml.safe_load(catalog_path.read_text(encoding="utf-8"))
    if not isinstance(catalog, dict) or not isinstance(catalog.get("tables"), list):
        raise ValueError("source table catalog must contain a tables list")
    catalog["tables"] = [row for row in catalog["tables"] if isinstance(row, dict) and row.get("vehicle") in FAMILY_IDS]
    _write(target, f"{relative}/catalog.yaml", catalog)

    manifest_path = ROOT / relative / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise ValueError("source table manifest must contain an object")
    generated = manifest.get("generated")
    models = manifest.get("models")
    if not isinstance(generated, list) or not isinstance(models, dict):
        raise ValueError("source table manifest must contain generated files and models")
    manifest["generated"] = [name for name in generated if isinstance(name, str) and (name.startswith("b747_") or name.startswith("skywalker_x8_"))]
    manifest["models"] = {identifier: models[identifier] for identifier in sorted(FAMILY_IDS) if identifier in models}
    _write_json(target, f"{relative}/manifest.json", manifest)
    ####


def _copy_source_assets(target: Path) -> None:
    """Copy the exact tables, source grids, and local-program anchors used by runtime."""

    table_root = ROOT / _BUNDLE / "tables"
    for table in sorted((*table_root.glob("b747_*.tbl"), *table_root.glob("skywalker_x8_*.tbl"))):
        _copy(target, table.relative_to(ROOT).as_posix())
    for relative in (
        f"{_BUNDLE}/jet_b747",
        f"{_BUNDLE}/cruise_class_uav_skywalker_x8",
    ):
        _copytree(target, relative)
    generated = ROOT / "examples/generated/vehicles"
    for program in sorted((*generated.glob("b747_*.prb"), *generated.glob("skywalker_x8_*.prb"))):
        _copy(target, program.relative_to(ROOT).as_posix())
    ####


def extract(target: Path = TARGET) -> None:
    """Write the independent source-table fixed-wing data boundary."""

    # Package data is wholly generated. Recreate its exact shared X8/B747 tree
    # rather than merging onto an earlier result, so a source-table change
    # cannot leave deleted or third-family files in a later wheel.
    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)

    _write_vehicle_models(target)
    for relative, key in (
        ("verification/vehicle_composition_registry.yaml", "vehicles"),
        ("verification/horizontal_fidelity_registry.yaml", "families"),
        ("verification/vehicle_execution_bindings.yaml", "bindings"),
        ("verification/vehicle_execution_parity.yaml", "bindings"),
        ("verification/vehicle_endpoint_specs.yaml", "endpoints"),
    ):
        _write(target, relative, _family_rows(_read(relative), key))
    _write_pseudo_profiles(target)
    _write_maturity(target)
    _write_lqr_profiles(target)
    _write_powered_fixed_wing_profiles(target)
    _write_language_backed_route_assets(target)
    witnesses = _read("verification/vehicle_execution_witnesses.yaml")
    _write(target, "verification/vehicle_execution_witnesses.yaml", _witness_fragment(witnesses))
    _write(
        target,
        "verification/vehicle_execution_parity_witnesses.yaml",
        _parity_witness_fragment(_read("verification/vehicle_execution_parity_witnesses.yaml")),
    )
    for relative in _witness_paths(witnesses):
        _copy(target, relative)
    _write_source_table_catalog_fragments(target)
    _copy_source_assets(target)
    _write_json(
        target,
        "package_data_provenance.json",
        {
            "schema": "taoryx.source-table-fixed-wing-package-data/v1",
            "family_ids": sorted(FAMILY_IDS),
            "source_catalog_root": "verification/",
            "claim_boundary": (
                "This exact package fragment mirrors only Skywalker X8 and B747 source-table catalog rows, local programs, "
                "source grids, witnesses, and table provenance. Repository-root catalogs remain canonical; local "
                "controller screens remain development evidence rather than route-level qualification."
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
    """Report every missing, stale, or unexpected shared package-data file."""

    with tempfile.TemporaryDirectory(prefix="taoryx-source-table-fixed-wing-assets-") as directory:
        expected_root = Path(directory) / "data"
        extract(expected_root)
        expected = _file_digests(expected_root)
    actual = _file_digests(target)
    missing = tuple(sorted(set(expected) - set(actual)))
    unexpected = tuple(sorted(set(actual) - set(expected)))
    changed = tuple(sorted(path for path in set(expected) & set(actual) if expected[path] != actual[path]))
    return (
        *(f"missing generated source-table fixed-wing asset: {path}" for path in missing),
        *(f"unexpected generated source-table fixed-wing asset: {path}" for path in unexpected),
        *(f"stale generated source-table fixed-wing asset: {path}" for path in changed),
    )
    ####


def main(argv: list[str] | None = None) -> int:
    """Regenerate or verify the complete shared source-table package boundary."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if package data differs from canonical X8/B747 inputs")
    args = parser.parse_args(argv)
    if args.check:
        differences = check()
        if differences:
            print("Source-table fixed-wing plug-in package data is stale:")
            print("\n".join(differences))
            return 1
        print("Source-table fixed-wing plug-in package data: current")
        return 0
    extract()
    print("Source-table fixed-wing plug-in package data: regenerated")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
