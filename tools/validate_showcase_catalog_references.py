#!/usr/bin/env python3
"""Validate that an aggregate showcase catalog points to canonical child runs.

This is deliberately separate from the run-artifact boundary validator. A
catalog may summarize several families, but it must resolve each referenced
packet to a manifest containing one or more canonical ``ShowcaseRunArtifact``
records. It may preserve child claims and outcomes; it may not invent a new
fidelity or control realization while aggregating them.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

from taoryx.showcase.artifact_binding import validate_showcase_run_artifact_boundary

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CATALOG = ROOT / "artifacts/showcases/alpha2/final-catalog-v1/catalog.json"
####


def _load(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected object in {path}")
    return payload
    ####


def _claim_boundary_sha256(artifact: Any) -> str:
    payload = {
        "fidelity": artifact.fidelity,
        "control_realization": artifact.control_realization,
        "claim": artifact.claim,
        "nonclaims": list(artifact.nonclaims),
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()
    ####


def validate_catalog(path: Path) -> dict[str, Any]:
    """Validate every family packet referenced by an aggregate catalog."""

    catalog = _load(path)
    records = catalog.get("families")
    findings: list[dict[str, str]] = []
    resolved: list[dict[str, Any]] = []
    if not isinstance(records, list):
        findings.append({"code": "catalog_records_missing", "message": "catalog has no families list"})
        records = []

    for index, record in enumerate(records):
        if not isinstance(record, dict):
            findings.append({"code": "catalog_record_invalid", "message": f"families[{index}] is not an object"})
            continue
        packet_name = record.get("packet")
        if not isinstance(packet_name, str) or not packet_name:
            findings.append({"code": "catalog_packet_missing", "message": f"families[{index}] has no packet path"})
            continue
        packet = (path.parent / packet_name).resolve()
        manifest_path = packet / "manifest.json"
        if not manifest_path.is_file():
            findings.append({"code": "child_manifest_missing", "message": f"{packet_name}/manifest.json does not exist"})
            continue
        try:
            manifest = _load(manifest_path)
            child_payloads = manifest.get("run_artifacts")
            if isinstance(child_payloads, dict):
                child_payloads = [child_payloads]
            if not isinstance(child_payloads, list) or not child_payloads:
                raise ValueError("child manifest has no canonical run_artifacts list")
            children = []
            for child_index, payload in enumerate(child_payloads):
                if not isinstance(payload, dict):
                    raise ValueError(f"run_artifacts[{child_index}] is not an object")
                artifact = validate_showcase_run_artifact_boundary(payload)
                children.append(
                    {
                        "run_id": artifact.run_id,
                        "fidelity": artifact.fidelity,
                        "control_realization": artifact.control_realization,
                        "outcome": artifact.outcome,
                        "claim_boundary_sha256": _claim_boundary_sha256(artifact),
                    }
                )
            resolved.append(
                {
                    "family_id": record.get("id"),
                    "packet": packet_name,
                    "manifest": str(manifest_path.relative_to(path.parent)),
                    "catalog_status": record.get("status"),
                    "catalog_mission_pass": record.get("mission_pass"),
                    "children": children,
                }
            )
        except (TypeError, ValueError, KeyError) as error:
            findings.append({"code": "child_manifest_invalid", "message": f"{packet_name}/manifest.json: {error}"})

    return {
        "schema": "taoryx.showcase-catalog-references/v1alpha1",
        "status": "pass" if not findings else "blocked",
        "catalog": str(path),
        "record_count": len(records),
        "resolved_record_count": len(resolved),
        "findings": findings,
        "resolved": resolved,
        "claim_boundary": "This validates aggregate references and preserves child claim metadata; it does not promote mission or source evidence.",
    }
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("catalog", type=Path, nargs="?", default=DEFAULT_CATALOG)
    arguments = parser.parse_args()
    report = validate_catalog(arguments.catalog.resolve())
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
    ####
