"""Materialize package-owned endpoint data for the low-fidelity debug models.

The repository-root workflow catalog remains canonical. This tool writes only
the ballistic, waypoint, and contract-probe endpoint rows and their authored
witnesses to the independently installable debug-models package.
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
TARGET = ROOT / "packages/taoryx-debug-models/src/taoryx_debug_models/data"
_ENDPOINT_IDS = (
    "reference-ballistic-3dof-batch",
    "reference-waypoint-3dof-batch",
    "debug-contract-probe-batch",
)
_WITNESSES = (
    "verification/workflow_endpoint_witnesses/reference_ballistic_3dof.yaml",
    "verification/workflow_endpoint_witnesses/reference_waypoint_3dof.yaml",
    "verification/workflow_endpoint_witnesses/debug_contract_probe.yaml",
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
        raise ValueError(f"missing debug-model workflow asset: {relative}")
    destination = target / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    ####


def extract(target: Path = TARGET) -> None:
    """Write the non-overlapping debug-model endpoint fragment and witnesses."""

    # Package data is wholly generated. Recreate its exact tree rather than
    # merging onto an earlier result, so deleted endpoint assets cannot linger
    # in a later debug-models wheel.
    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)

    catalog = _read("verification/mission_workflow_endpoint_specs.yaml")
    endpoints = catalog.get("endpoints")
    if not isinstance(endpoints, list):
        raise ValueError("workflow endpoint catalog must contain an endpoints list")
    selected = [item for item in endpoints if isinstance(item, dict) and item.get("id") in _ENDPOINT_IDS]
    if tuple(item.get("id") for item in selected) != _ENDPOINT_IDS:
        raise ValueError(f"workflow endpoint catalog must contain exactly {_ENDPOINT_IDS!r} in order")
    catalog["endpoints"] = selected
    _write(target, "verification/mission_workflow_endpoint_specs.yaml", catalog)
    for witness in _WITNESSES:
        _copy(target, witness)

    provenance = {
        "schema": "taoryx.debug-models-package-data/v1",
        "endpoint_ids": list(_ENDPOINT_IDS),
        "source_catalog_root": "verification/",
        "claim_boundary": (
            "This package fragment mirrors only the analytical ballistic, waypoint, and contract-probe endpoint "
            "records and authored witnesses; repository-root catalogs remain canonical."
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
    """Report every missing, stale, or unexpected debug-models package asset."""

    with tempfile.TemporaryDirectory(prefix="taoryx-debug-models-plugin-assets-") as directory:
        expected_root = Path(directory) / "data"
        extract(expected_root)
        expected = _file_digests(expected_root)
    actual = _file_digests(target)
    missing = tuple(sorted(set(expected) - set(actual)))
    unexpected = tuple(sorted(set(actual) - set(expected)))
    changed = tuple(sorted(path for path in set(expected) & set(actual) if expected[path] != actual[path]))
    return (
        *(f"missing generated debug-models asset: {path}" for path in missing),
        *(f"unexpected generated debug-models asset: {path}" for path in unexpected),
        *(f"stale generated debug-models asset: {path}" for path in changed),
    )
    ####


def main(argv: list[str] | None = None) -> int:
    """Regenerate or verify the complete debug-models package-data boundary."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if package data differs from canonical debug-model inputs")
    args = parser.parse_args(argv)
    if args.check:
        differences = check()
        if differences:
            print("Debug-models plug-in package data is stale:")
            print("\n".join(differences))
            return 1
        print("Debug-models plug-in package data: current")
        return 0
    extract()
    print("Debug-models plug-in package data: regenerated")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
