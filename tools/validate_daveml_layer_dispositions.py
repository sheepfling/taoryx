"""Validate explicit DAVE-ML family-library layer dispositions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
ALLOWED_STATUSES = {
    "verified",
    "not_applicable",
    "not_in_source_contract",
    "external_overlay_required",
    "not_promoted",
}


def validate_dispositions(registry_path: str | Path) -> dict[str, Any]:
    """Return a deterministic report for every declared downstream layer."""

    path = Path(registry_path)
    payload = _read_yaml(path)
    raw_families = payload.get("families")
    if not isinstance(raw_families, list) or not raw_families:
        raise ValueError(f"layer disposition registry needs a non-empty families list: {path}")

    reports: list[dict[str, Any]] = []
    failures: list[str] = []
    for entry in raw_families:
        if not isinstance(entry, dict):
            failures.append("family entry is not a mapping")
            continue
        family_id = str(entry.get("family_id", ""))
        integration_record = str(entry.get("integration_record", ""))
        layers = entry.get("layers")
        record_path = _resolve_path(integration_record)
        family_failures: list[str] = []
        if not family_id:
            family_failures.append("missing family_id")
        if not record_path.is_file():
            family_failures.append(f"missing integration record: {integration_record}")
        if not isinstance(layers, dict) or not layers:
            family_failures.append("layers must be a non-empty mapping")
            layers = {}
        layer_reports: dict[str, Any] = {}
        for layer_id, raw_layer in sorted(layers.items()):
            if not isinstance(raw_layer, dict):
                family_failures.append(f"{layer_id}: disposition is not a mapping")
                continue
            status = str(raw_layer.get("status", ""))
            evidence = str(raw_layer.get("evidence", ""))
            evidence_path = _resolve_path(evidence.split("#", 1)[0])
            layer_failures: list[str] = []
            if status not in ALLOWED_STATUSES:
                layer_failures.append(f"unsupported status {status!r}")
            if not evidence_path.is_file():
                layer_failures.append(f"missing evidence: {evidence}")
            if not str(raw_layer.get("rationale", "")).strip():
                layer_failures.append("missing rationale")
            if layer_failures:
                family_failures.extend(f"{layer_id}: {failure}" for failure in layer_failures)
            layer_reports[layer_id] = {
                "status": status,
                "evidence": evidence,
                "rationale": str(raw_layer.get("rationale", "")),
                "failures": layer_failures,
            }
        reports.append(
            {
                "family_id": family_id,
                "integration_record": integration_record,
                "layer_count": len(layer_reports),
                "layers": layer_reports,
                "failures": family_failures,
                "status": "verified" if not family_failures else "failed",
            }
        )
        failures.extend(f"{family_id}: {failure}" for failure in family_failures)

    return {
        "schema_version": "taoryx.daveml-family-layer-dispositions-report/v1",
        "status": "verified" if not failures else "failed",
        "registry": path.as_posix(),
        "claim_boundary": str(payload.get("claim_boundary", "")),
        "family_count": len(reports),
        "layer_count": sum(report["layer_count"] for report in reports),
        "families": reports,
        "failures": failures,
    }


def _read_yaml(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"layer disposition registry must be a mapping: {path}")
    return payload


def _resolve_path(label: str) -> Path:
    candidate = Path(label)
    if candidate.is_absolute():
        return candidate
    return (ROOT / candidate).resolve()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    report = validate_dispositions(arguments.registry)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "verified" else 1


if __name__ == "__main__":
    raise SystemExit(main())
