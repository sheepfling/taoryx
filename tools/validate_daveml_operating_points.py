"""Validate the DaveML family operating-point catalog and its evidence links."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "verification/daveml_operating_point_catalog.yaml"
CONTRACTS = ROOT / "verification/daveml_operational_contracts.json"
REPORT = ROOT / "verification/daveml_operating_point_catalog.json"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
####


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"evidence must be a mapping: {path}")
    return value
####


def _lookup(value: Any, path: str) -> Any:
    for part in path.split("."):
        if not isinstance(value, dict) or part not in value:
            raise ValueError(f"evidence field is missing: {path}")
        value = value[part]
    return value
####


def validate() -> dict[str, Any]:
    raw = yaml.safe_load(CATALOG.read_text(encoding="utf-8"))
    contracts = _load_json(CONTRACTS)
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ValueError("operating-point catalog schema_version must be 1")
    family_contracts = {str(item["family_id"]): item for item in contracts.get("families", [])}
    entries = raw.get("families")
    if not isinstance(entries, list) or not entries:
        raise ValueError("operating-point catalog requires families")
    seen: set[str] = set()
    report_entries: list[dict[str, Any]] = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("operating-point entry must be a mapping")
        family_id = str(entry.get("id", ""))
        if not family_id or family_id in seen:
            raise ValueError(f"duplicate or missing operating-point family: {family_id!r}")
        seen.add(family_id)
        contract = family_contracts.get(family_id)
        if contract is None:
            raise ValueError(f"{family_id}: missing operational contract")
        if entry.get("qualification_class") != contract["qualification_class"]:
            raise ValueError(f"{family_id}: qualification class does not match operational contract")
        applicability = str(entry.get("applicability", ""))
        if applicability not in {"required", "not_applicable"}:
            raise ValueError(f"{family_id}: invalid applicability")
        evidence_path = ROOT / str(entry.get("evidence", ""))
        if not evidence_path.is_file():
            raise ValueError(f"{family_id}: missing evidence {evidence_path}")
        evidence = _load_json(evidence_path)
        expected_status = str(entry.get("evidence_status", ""))
        actual_status = str(evidence.get("status", ""))
        if expected_status != actual_status:
            raise ValueError(f"{family_id}: evidence status {actual_status!r} != {expected_status!r}")
        if not str(entry.get("point_type", "")).strip() or not str(entry.get("point_id", "")).strip():
            raise ValueError(f"{family_id}: operating point requires type and id")
        point = entry.get("operating_point")
        if not isinstance(point, dict) or not point:
            raise ValueError(f"{family_id}: operating point values are required")
        residual = entry.get("residual")
        if not isinstance(residual, dict) or not str(residual.get("metric", "")).strip() or float(residual.get("tolerance", -1.0)) < 0.0:
            raise ValueError(f"{family_id}: residual contract is incomplete")
        if applicability == "not_applicable" and not str(entry.get("non_applicable_reason", "")).strip():
            raise ValueError(f"{family_id}: non-applicable point requires a reason")
        expansion_axes = entry.get("expansion_axes")
        if not isinstance(expansion_axes, list) or not expansion_axes:
            raise ValueError(f"{family_id}: expansion axes are required")
        nonclaims = entry.get("nonclaims")
        if not isinstance(nonclaims, list) or not nonclaims:
            raise ValueError(f"{family_id}: operating-point nonclaims are required")
        report_entries.append(
            {
                "family_id": family_id,
                "qualification_class": entry["qualification_class"],
                "applicability": applicability,
                "point_id": entry["point_id"],
                "point_type": entry["point_type"],
                "operating_point": point,
                "evidence": str(evidence_path.relative_to(ROOT)).replace("\\", "/"),
                "evidence_sha256": _sha256(evidence_path),
                "evidence_status": actual_status,
                "residual": residual,
                "expansion_axes": expansion_axes,
                "non_applicable_reason": entry.get("non_applicable_reason"),
                "nonclaims": nonclaims,
                "status": "verified",
            }
        )
    ####
    if set(family_contracts) != seen:
        raise ValueError("operating-point catalog must cover exactly the operational contract families")
    return {
        "schema_version": 1,
        "report_type": "taoryx.daveml-operating-point-catalog/v1",
        "catalog": str(CATALOG.relative_to(ROOT)).replace("\\", "/"),
        "catalog_sha256": _sha256(CATALOG),
        "operational_contracts_sha256": _sha256(CONTRACTS),
        "claim_boundary": raw["claim_boundary"],
        "family_count": len(report_entries),
        "families": report_entries,
        "status": "verified",
    }
####


def main() -> int:
    report = validate()
    REPORT.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
