"""Mechanically materialize reachability-owned package data.

The repository-root reachability profile catalog, X-15 and HL-20 overlay rows,
and HL-20 showcase remain the canonical editing surfaces. This extractor
copies only the optional reachability workbench's assets into its independently
installable wheel. Each vehicle overlay has a distinct resource root, so a
selected X-15 route never parses HL-20 overlay data (or conversely). Do not
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
TARGET = ROOT / "packages/taoryx-reachability/src/taoryx_reachability/data"
ASSET_FILES = ("verification/reachability_profile_catalog.yaml",)
ASSET_TREES = ("examples/showcases/hl20_california_to_hawaii",)
_X15_FAMILY_ID = "x15"
_HL20_FAMILY_ID = "hl20_mod_k"
_X15_OVERLAY_ROOT = "x15_overlay"
_HL20_OVERLAY_ROOT = "hl20_overlay"
_X15_STAGED_MISSION_ID = "x15_staged_booster_reachability_v1"
_X15_STAGED_INITIALIZATION_IDS = frozenset({"staged_booster_launch"})
_X15_STAGED_SEGMENT_IDS = frozenset(
    {"booster_powered", "booster_coast_release", "unpowered_glide_handoff", "impact_witness"}
)
_X15_STAGED_COMPOSITIONS = (
    "examples/vehicle_composition/x15_staged_booster_reachability_3dof_compose.yaml",
    "examples/vehicle_composition/x15_staged_booster_reachability_pseudo6dof_compose.yaml",
)
_HL20_OVERLAY_MISSION_IDS = frozenset(
    {"hl20_source_booster_release_replay_v1", "lifting_body_glide_energy_management_v1"}
)
_HL20_OVERLAY_INITIALIZATION_IDS = frozenset({"source_booster_launch", "high_altitude_release"})
_HL20_OVERLAY_SEGMENT_IDS = frozenset(
    {
        "source_booster_release",
        "source_scheduled_bank",
        "source_ground_contact",
        "trim_capture",
        "glide_bank_reversal",
        "energy_handoff",
    }
)
_HL20_OVERLAY_COMPOSITIONS = (
    "examples/vehicle_composition/hl20_source_booster_release_replay_3dof_compose.yaml",
    "examples/vehicle_composition/hl20_source_booster_release_replay_pseudo6dof_compose.yaml",
    "examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml",
    "examples/vehicle_composition/hl20_glide_energy_capability_pseudo6dof_compose.yaml",
)
_HL20_WITNESS_COMPOSITIONS = frozenset(_HL20_OVERLAY_COMPOSITIONS[:2])
_X15_OVERLAY_BINDINGS = frozenset(
    {
        (_X15_FAMILY_ID, _X15_STAGED_MISSION_ID, "point_mass_3dof"),
        (_X15_FAMILY_ID, _X15_STAGED_MISSION_ID, "pseudo_6dof"),
    }
)
_HL20_OVERLAY_BINDINGS = frozenset(
    {
        (_HL20_FAMILY_ID, "hl20_source_booster_release_replay_v1", "point_mass_3dof"),
        (_HL20_FAMILY_ID, "hl20_source_booster_release_replay_v1", "pseudo_6dof"),
        (_HL20_FAMILY_ID, "lifting_body_glide_energy_management_v1", "rigid_body_6dof_direct_wrench"),
        (_HL20_FAMILY_ID, "lifting_body_glide_energy_management_v1", "rigid_body_6dof_surface_allocated"),
    }
)


def _copy_file(target: Path, relative: str) -> None:
    """Copy one canonical reachability asset with its stable relative path."""

    source = ROOT / relative
    destination = target / relative
    if not source.is_file():
        raise ValueError(f"missing reachability asset: {relative}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    ####


def _copy_tree(target: Path, relative: str) -> None:
    """Copy one canonical showcase tree without interpreter-cache baggage."""

    source = ROOT / relative
    destination = target / relative
    if not source.is_dir():
        raise ValueError(f"missing reachability asset tree: {relative}")
    shutil.copytree(
        source,
        destination,
        dirs_exist_ok=True,
        ignore=shutil.ignore_patterns("__pycache__", "*.py[cod]"),
    )
    ####


def _read_yaml(relative: str) -> dict[str, Any]:
    """Read one canonical registry before extracting this overlay's rows."""

    payload = yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{relative} must contain a mapping")
    return payload
    ####


def _write_yaml(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write one deterministic reachability-owned YAML fragment."""

    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    ####


def _family_declaration(family_id: str) -> dict[str, Any]:
    """Return one canonical family declaration used to derive an overlay."""

    payload = _read_yaml("verification/vehicle_composition_registry.yaml")
    vehicles = payload.get("vehicles")
    if not isinstance(vehicles, list):
        raise ValueError("vehicle composition registry vehicles must contain a list")
    declaration = next(
        (row for row in vehicles if isinstance(row, dict) and row.get("family_id") == family_id),
        None,
    )
    if declaration is None:
        raise ValueError(f"canonical vehicle composition registry has no {family_id!r} declaration")
    return declaration
    ####


def _selected_rows(
    declaration: dict[str, Any],
    key: str,
    identifiers: frozenset[str],
) -> list[dict[str, Any]]:
    """Select one exact family row set by its stable public identifiers."""

    rows = declaration.get(key)
    if not isinstance(rows, list):
        raise ValueError(f"family declaration {key} must contain a list")
    selected = [row for row in rows if isinstance(row, dict) and row.get("id") in identifiers]
    selected_ids = {identifier for row in selected if isinstance((identifier := row.get("id")), str)}
    if selected_ids != identifiers:
        raise ValueError(f"family declaration {key} is missing a named reachability overlay row")
    return selected
    ####


def _catalog_overlay(
    family_id: str,
    initialization_ids: frozenset[str],
    segment_ids: frozenset[str],
    mission_ids: frozenset[str],
) -> dict[str, Any]:
    """Build append-only reachability additions for one selected family."""

    declaration = _family_declaration(family_id)
    return {
        "family_id": family_id,
        "initialization_contracts": _selected_rows(declaration, "initialization_contracts", initialization_ids),
        "segment_contracts": _selected_rows(declaration, "segment_contracts", segment_ids),
        "mission_templates": _selected_rows(declaration, "mission_templates", mission_ids),
    }
    ####


def _write_catalog_overlay(
    target: Path,
    family_id: str,
    initialization_ids: frozenset[str],
    segment_ids: frozenset[str],
    mission_ids: frozenset[str],
) -> None:
    """Write one family-specific optional composition overlay."""

    _write_yaml(
        target,
        "verification/vehicle_composition_registry.yaml",
        {
            "schema": "taoryx.vehicle-composition-overlay/v1alpha1",
            "version": 1,
            "description": f"Optional reachability-owned additions to the selected {family_id} family catalog.",
            "vehicles": [],
            "overlays": [
                _catalog_overlay(
                    family_id,
                    initialization_ids,
                    segment_ids,
                    mission_ids,
                ),
            ],
        },
    )
    ####


def _write_execution_bindings(target: Path, expected: frozenset[tuple[str, str, str]]) -> None:
    """Write the exact batch bindings owned by one optional overlay."""

    payload = _read_yaml("verification/vehicle_execution_bindings.yaml")
    rows = payload.get("bindings")
    if not isinstance(rows, list):
        raise ValueError("vehicle_execution_bindings.yaml bindings must contain a list")
    bindings = [
        row
        for row in rows
        if isinstance(row, dict)
        and isinstance(row.get("family_id"), str)
        and isinstance(row.get("mission"), str)
        and isinstance(row.get("fidelity"), str)
        and (row["family_id"], row["mission"], row["fidelity"]) in expected
    ]
    actual = frozenset((row["family_id"], row["mission"], row["fidelity"]) for row in bindings)
    if actual != expected:
        raise ValueError("canonical execution bindings are missing named reachability overlay rows")
    fragment = dict(payload)
    fragment["bindings"] = bindings
    _write_yaml(target, "verification/vehicle_execution_bindings.yaml", fragment)
    ####


def _write_execution_witnesses(target: Path, compositions: frozenset[str]) -> None:
    """Write one overlay's witnesses and no local-screen evidence."""

    payload = _read_yaml("verification/vehicle_execution_witnesses.yaml")
    rows = payload.get("witnesses")
    if not isinstance(rows, list):
        raise ValueError("vehicle_execution_witnesses.yaml witnesses must contain a list")
    witnesses = [row for row in rows if isinstance(row, dict) and row.get("composition") in compositions]
    actual_compositions = {row["composition"] for row in witnesses if isinstance(row.get("composition"), str)}
    if actual_compositions != compositions:
        raise ValueError("canonical execution witnesses are missing named reachability overlay rows")
    fragment = dict(payload)
    fragment["witnesses"] = witnesses
    fragment["variant_witnesses"] = []
    fragment["graph_extension_witnesses"] = []
    _write_yaml(target, "verification/vehicle_execution_witnesses.yaml", fragment)
    ####


def extract(target: Path = TARGET) -> None:
    """Rebuild the exact reachability-owned package-data tree."""

    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)
    for relative in ASSET_FILES:
        _copy_file(target, relative)
    for relative in ASSET_TREES:
        _copy_tree(target, relative)
    x15_overlay_target = target / _X15_OVERLAY_ROOT
    _write_catalog_overlay(
        x15_overlay_target,
        _X15_FAMILY_ID,
        _X15_STAGED_INITIALIZATION_IDS,
        _X15_STAGED_SEGMENT_IDS,
        frozenset({_X15_STAGED_MISSION_ID}),
    )
    _write_execution_bindings(x15_overlay_target, _X15_OVERLAY_BINDINGS)
    _write_execution_witnesses(x15_overlay_target, frozenset(_X15_STAGED_COMPOSITIONS))
    for relative in _X15_STAGED_COMPOSITIONS:
        _copy_file(x15_overlay_target, relative)

    hl20_overlay_target = target / _HL20_OVERLAY_ROOT
    _write_catalog_overlay(
        hl20_overlay_target,
        _HL20_FAMILY_ID,
        _HL20_OVERLAY_INITIALIZATION_IDS,
        _HL20_OVERLAY_SEGMENT_IDS,
        _HL20_OVERLAY_MISSION_IDS,
    )
    _write_execution_bindings(hl20_overlay_target, _HL20_OVERLAY_BINDINGS)
    _write_execution_witnesses(hl20_overlay_target, _HL20_WITNESS_COMPOSITIONS)
    for relative in _HL20_OVERLAY_COMPOSITIONS:
        _copy_file(hl20_overlay_target, relative)
    provenance = {
        "schema": "taoryx.reachability-package-data/v1",
        "source_catalog_root": "verification/",
        "asset_files": [
            *ASSET_FILES,
            f"{_X15_OVERLAY_ROOT}/verification/vehicle_composition_registry.yaml",
            f"{_X15_OVERLAY_ROOT}/verification/vehicle_execution_bindings.yaml",
            f"{_X15_OVERLAY_ROOT}/verification/vehicle_execution_witnesses.yaml",
            *(f"{_X15_OVERLAY_ROOT}/{relative}" for relative in _X15_STAGED_COMPOSITIONS),
            f"{_HL20_OVERLAY_ROOT}/verification/vehicle_composition_registry.yaml",
            f"{_HL20_OVERLAY_ROOT}/verification/vehicle_execution_bindings.yaml",
            f"{_HL20_OVERLAY_ROOT}/verification/vehicle_execution_witnesses.yaml",
            *(f"{_HL20_OVERLAY_ROOT}/{relative}" for relative in _HL20_OVERLAY_COMPOSITIONS),
        ],
        "asset_trees": list(ASSET_TREES),
        "claim_boundary": (
            "This package fragment owns reachability profile/showcase assets and additive X-15 staged plus HL-20 "
            "booster-release/glide-energy composition, execution, and witness rows. Vehicle source data remains "
            "with its family package."
        ),
    }
    (target / "package_data_provenance.json").write_text(
        json.dumps(provenance, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    ####


def _file_digests(root: Path) -> dict[str, str]:
    """Return one content-addressed inventory for a generated data tree."""

    if not root.is_dir():
        return {}
    return {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(root.rglob("*")) if path.is_file()}
    ####


def check(target: Path = TARGET) -> tuple[str, ...]:
    """Report every missing, stale, or unexpected reachability package asset."""

    with tempfile.TemporaryDirectory(prefix="taoryx-reachability-plugin-assets-") as directory:
        expected_root = Path(directory) / "data"
        extract(expected_root)
        expected = _file_digests(expected_root)
    actual = _file_digests(target)
    missing = tuple(sorted(set(expected) - set(actual)))
    unexpected = tuple(sorted(set(actual) - set(expected)))
    changed = tuple(sorted(path for path in set(expected) & set(actual) if expected[path] != actual[path]))
    return (
        *(f"missing generated reachability asset: {path}" for path in missing),
        *(f"unexpected generated reachability asset: {path}" for path in unexpected),
        *(f"stale generated reachability asset: {path}" for path in changed),
    )
    ####


def main(argv: list[str] | None = None) -> int:
    """Regenerate or verify the reachability package-data boundary."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if package data differs from canonical reachability inputs")
    args = parser.parse_args(argv)
    if args.check:
        differences = check()
        if differences:
            print("Reachability plug-in package data is stale:")
            print("\n".join(differences))
            return 1
        print("Reachability plug-in package data: current")
        return 0
    extract()
    print("Reachability plug-in package data: regenerated")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
