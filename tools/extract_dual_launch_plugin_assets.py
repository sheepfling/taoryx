"""Materialize package-owned dual-launch workflow data from canonical catalogs.

The repository-root verification files remain the editable source of truth.
This tool copies only the Dual Launch family row, maturity row, endpoint row,
and authored witness into the plug-in that owns the focused provider. Do not
hand-edit the generated package-data mirror.
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
TARGET = ROOT / "packages/taoryx-dual-launch/src/taoryx_dual_launch/data"
_FAMILY_ID = "dual_launch_glider"
_ENDPOINT_ID = "dual-launch-attached-booster-batch"
_WITNESS = "verification/workflow_endpoint_witnesses/dual_launch_attached_booster.yaml"


def _read(relative: str) -> dict[str, Any]:
    """Read one canonical YAML mapping."""

    payload = yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{relative} must contain a mapping")
    return payload
    ####


def _write(target: Path, relative: str, payload: dict[str, Any]) -> None:
    """Write one deterministic package-owned YAML fragment."""

    path = target / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
    ####


def _copy(target: Path, relative: str) -> None:
    """Copy one authored witness into the owning package."""

    source = ROOT / relative
    if not source.is_file():
        raise ValueError(f"missing Dual Launch workflow asset: {relative}")
    destination = target / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    ####


def _single_row(payload: dict[str, Any], key: str, identifier: str, *, id_key: str = "id") -> dict[str, Any]:
    """Return a catalog copy containing exactly one stable-identity row."""

    rows = payload.get(key)
    if not isinstance(rows, list):
        raise ValueError(f"{key} must contain a list")
    selected = [item for item in rows if isinstance(item, dict) and item.get(id_key) == identifier]
    if len(selected) != 1:
        raise ValueError(f"{key} must contain exactly one {identifier!r} row")
    result = dict(payload)
    result[key] = selected
    return result
    ####


def extract(target: Path = TARGET) -> None:
    """Write the non-overlapping family and endpoint fragments for Dual Launch."""

    # Package data is wholly generated. Recreate its exact tree rather than
    # merging onto an earlier result, so deleted dual-launch assets cannot
    # linger in a later independent wheel.
    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)

    _write(
        target,
        "verification/dual_launch_family_catalog.yaml",
        _single_row(_read("verification/alpha2_family_catalog.yaml"), "families", _FAMILY_ID, id_key="family_id"),
    )
    _write(
        target,
        "verification/vehicle_maturity_registry.yaml",
        _single_row(_read("verification/vehicle_maturity_registry.yaml"), "records", _FAMILY_ID),
    )
    _write(
        target,
        "verification/mission_workflow_endpoint_specs.yaml",
        _single_row(_read("verification/mission_workflow_endpoint_specs.yaml"), "endpoints", _ENDPOINT_ID),
    )
    _copy(target, _WITNESS)

    provenance = {
        "schema": "taoryx.dual-launch-package-data/v1",
        "family_id": _FAMILY_ID,
        "endpoint_id": _ENDPOINT_ID,
        "source_catalog_root": "verification/",
        "claim_boundary": (
            "This package fragment mirrors only the synthetic Dual Launch family metadata, maturity row, workflow "
            "endpoint, and authored witness; repository-root catalogs remain canonical."
        ),
    }
    (target / "package_data_provenance.json").write_text(
        json.dumps(provenance, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    ####


def _file_digests(root: Path) -> dict[str, str]:
    """Return a content-addressed inventory for one generated data tree."""

    if not root.is_dir():
        return {}
    return {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(root.rglob("*")) if path.is_file()}
    ####


def check(target: Path = TARGET) -> tuple[str, ...]:
    """Report every missing, stale, or unexpected Dual Launch package asset."""

    with tempfile.TemporaryDirectory(prefix="taoryx-dual-launch-plugin-assets-") as directory:
        expected_root = Path(directory) / "data"
        extract(expected_root)
        expected = _file_digests(expected_root)
    actual = _file_digests(target)
    missing = tuple(sorted(set(expected) - set(actual)))
    unexpected = tuple(sorted(set(actual) - set(expected)))
    changed = tuple(sorted(path for path in set(expected) & set(actual) if expected[path] != actual[path]))
    return (
        *(f"missing generated Dual Launch asset: {path}" for path in missing),
        *(f"unexpected generated Dual Launch asset: {path}" for path in unexpected),
        *(f"stale generated Dual Launch asset: {path}" for path in changed),
    )
    ####


def main(argv: list[str] | None = None) -> int:
    """Regenerate or verify the complete Dual Launch package-data boundary."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if package data differs from canonical Dual Launch inputs")
    args = parser.parse_args(argv)
    if args.check:
        differences = check()
        if differences:
            print("Dual Launch plug-in package data is stale:")
            print("\n".join(differences))
            return 1
        print("Dual Launch plug-in package data: current")
        return 0
    extract()
    print("Dual Launch plug-in package data: regenerated")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
