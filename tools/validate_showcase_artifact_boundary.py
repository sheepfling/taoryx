#!/usr/bin/env python3
"""Validate the resolved fidelity/control boundary of showcase packets."""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
# Executing a tool by path makes ``tools/`` the import root.  Keep the
# repository root available too so --build-daveml can import its sibling
# builder exactly as documented by ``tools/dev.py``.
sys.path.insert(0, str(ROOT))

from taoryx.showcase.artifact_binding import validate_showcase_run_artifact_boundary


def _load(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"showcase manifest is not an object: {path}")
    return payload
    ####


def _manifest_paths(paths: tuple[Path, ...]) -> tuple[Path, ...]:
    found: list[Path] = []
    for path in paths:
        if path.is_file():
            found.append(path)
        elif path.is_dir():
            found.extend(sorted(path.rglob("manifest.json")))
        else:
            raise FileNotFoundError(path)
    return tuple(dict.fromkeys(found))
    ####


def validate_paths(paths: tuple[Path, ...]) -> dict[str, Any]:
    """Validate every resolved run artifact in the supplied manifests."""

    findings: list[dict[str, str]] = []
    artifact_count = 0
    manifest_count = 0
    for path in _manifest_paths(paths):
        manifest_count += 1
        payload = _load(path)
        run_payloads = payload.get("run_artifacts")
        if not isinstance(run_payloads, list):
            if "run_id" in payload:
                run_payloads = [payload]
            else:
                findings.append(
                    {
                        "path": str(path),
                        "code": "run_artifacts_missing",
                        "message": "manifest has no resolved run_artifacts list",
                    }
                )
                continue
        for index, run_payload in enumerate(run_payloads):
            artifact_count += 1
            if not isinstance(run_payload, dict):
                findings.append(
                    {
                        "path": str(path),
                        "code": "run_artifact_not_object",
                        "message": f"run_artifacts[{index}] is not an object",
                    }
                )
                continue
            try:
                validate_showcase_run_artifact_boundary(run_payload)
            except (TypeError, ValueError) as error:
                findings.append(
                    {
                        "path": str(path),
                        "code": "artifact_boundary_failed",
                        "message": f"run_artifacts[{index}]: {error}",
                    }
                )
    return {
        "schema": "taoryx.showcase-artifact-boundary/v1alpha1",
        "status": "pass" if not findings else "blocked",
        "manifest_count": manifest_count,
        "artifact_count": artifact_count,
        "findings": findings,
        "claim_boundary": "This validates metadata consistency; it does not promote mission or source evidence.",
    }
    ####


def _build_daveml_packet() -> tuple[tempfile.TemporaryDirectory[str], tuple[Path, ...]]:
    """Build the current common-builder packet outside the repository."""

    from tools.build_daveml_showcase_composites import DEFAULT_CATALOG, build

    temporary = tempfile.TemporaryDirectory(prefix="taoryx-showcase-boundary-")
    output = Path(temporary.name) / "daveml-families"
    build(DEFAULT_CATALOG, output)
    return temporary, (output,)
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", type=Path)
    parser.add_argument(
        "--build-daveml",
        action="store_true",
        help="build the current DAVE-ML showcase catalog in a temporary directory before checking it",
    )
    arguments = parser.parse_args()
    temporary: tempfile.TemporaryDirectory[str] | None = None
    try:
        paths = tuple(arguments.paths)
        if arguments.build_daveml:
            temporary, built_paths = _build_daveml_packet()
            paths += built_paths
        if not paths:
            paths = (ROOT / "artifacts/showcases/daveml-families",)
        report = validate_paths(paths)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report["status"] == "pass" else 1
    finally:
        if temporary is not None:
            temporary.cleanup()
    ####


if __name__ == "__main__":
    raise SystemExit(main())
