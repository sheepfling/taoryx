"""Materialize Simple Aero workflow-owned package data.

The repository-root workflow endpoint catalog remains canonical.  This tool
copies the Simple Aero endpoint row and its authored witness into the package
that owns the corresponding focused provider.  Do not hand-edit the generated
package-data mirror.
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
TARGET = ROOT / "packages/taoryx-simple-aero/src/taoryx_simple_aero/data"
_ENDPOINT_ID = "simple-aero-fixed-ld-batch"
_WITNESS = "verification/workflow_endpoint_witnesses/simple_aero_fixed_ld_baseline.yaml"
_FAMILY_ID = "simple_aero"
_STATIC_ASSET_DIRECTORIES = (
    "tests/fixtures/simple_aero_v1",
    "tests/fixtures/problem_file_dumps/simple_aero_segments",
    "tests/fixtures/problem_file_dumps/simple_aero_trajectories",
)


def _read(relative: str) -> dict[str, Any]:
    """Read one canonical YAML mapping."""

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
    """Copy one authored witness into the owning package."""

    source = ROOT / relative
    if not source.is_file():
        raise ValueError(f"missing Simple Aero workflow asset: {relative}")
    destination = target / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    ####


def _copytree(target: Path, relative: str) -> None:
    """Copy one complete Simple Aero fixture directory into package data."""

    source = ROOT / relative
    if not source.is_dir():
        raise ValueError(f"missing Simple Aero workflow directory: {relative}")
    shutil.copytree(
        source,
        target / relative,
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".ruff_cache"),
    )
    ####


def _write_family_catalog(target: Path) -> None:
    """Carry only the Simple Aero Alpha 2 family from the shared catalog."""

    catalog = _read("verification/alpha2_family_catalog.yaml")
    families = catalog.get("families")
    if not isinstance(families, list):
        raise ValueError("Alpha 2 family catalog must contain a families list")
    selected = [row for row in families if isinstance(row, dict) and row.get("family_id") == _FAMILY_ID]
    if len(selected) != 1:
        raise ValueError(f"Alpha 2 family catalog must contain exactly one {_FAMILY_ID!r} row")
    catalog["families"] = selected
    _write(target, "verification/alpha2_family_catalog.yaml", catalog)
    ####


def extract(target: Path = TARGET) -> None:
    """Write the non-overlapping Simple Aero endpoint fragment and witness."""

    # Package data is wholly generated. Recreate its exact tree rather than
    # merging onto an earlier result, so deleted fixtures or other workflow
    # family rows cannot linger in a later Simple Aero wheel.
    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)

    catalog = _read("verification/mission_workflow_endpoint_specs.yaml")
    endpoints = catalog.get("endpoints")
    if not isinstance(endpoints, list):
        raise ValueError("workflow endpoint catalog must contain an endpoints list")
    selected = [item for item in endpoints if isinstance(item, dict) and item.get("id") == _ENDPOINT_ID]
    if len(selected) != 1:
        raise ValueError(f"workflow endpoint catalog must contain exactly one {_ENDPOINT_ID!r} row")
    catalog["endpoints"] = selected
    _write(target, "verification/mission_workflow_endpoint_specs.yaml", catalog)
    _copy(target, _WITNESS)
    _write_family_catalog(target)
    _copy(target, "verification/simple_aero_segment_catalog.yaml")
    for relative in _STATIC_ASSET_DIRECTORIES:
        _copytree(target, relative)

    provenance = {
        "schema": "taoryx.simple-aero-package-data/v1",
        "endpoint_id": _ENDPOINT_ID,
        "source_catalog_root": "verification/",
        "claim_boundary": (
            "This package fragment mirrors only the Simple Aero workflow endpoint and its authored witness; repository-root catalogs remain canonical."
        ),
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
    """Report every missing, stale, or unexpected Simple Aero package asset."""

    with tempfile.TemporaryDirectory(prefix="taoryx-simple-aero-plugin-assets-") as directory:
        expected_root = Path(directory) / "data"
        extract(expected_root)
        expected = _file_digests(expected_root)
    actual = _file_digests(target)
    missing = tuple(sorted(set(expected) - set(actual)))
    unexpected = tuple(sorted(set(actual) - set(expected)))
    changed = tuple(sorted(path for path in set(expected) & set(actual) if expected[path] != actual[path]))
    return (
        *(f"missing generated Simple Aero asset: {path}" for path in missing),
        *(f"unexpected generated Simple Aero asset: {path}" for path in unexpected),
        *(f"stale generated Simple Aero asset: {path}" for path in changed),
    )
    ####


def main(argv: list[str] | None = None) -> int:
    """Regenerate or verify the complete Simple Aero package-data boundary."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if package data differs from canonical Simple Aero inputs")
    args = parser.parse_args(argv)
    if args.check:
        differences = check()
        if differences:
            print("Simple Aero plug-in package data is stale:")
            print("\n".join(differences))
            return 1
        print("Simple Aero plug-in package data: current")
        return 0
    extract()
    print("Simple Aero plug-in package data: regenerated")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
