"""Validate the executable Alpha 3 DAVE-ML completion registry."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
REGISTRY = ROOT / "verification/daveml_alpha3_completion.yaml"
REPORT = ROOT / "verification/daveml_alpha3_completion.json"
ALLOWED_STATUSES = {
    "verified_parent_regression_only",
    "contract_ready",
    "equivalence_pending",
    "verified_witness",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _path(label: str) -> Path:
    candidate = Path(label)
    return candidate if candidate.is_absolute() else ROOT / candidate
    ####


def _required_path(label: str, failures: list[str], context: str) -> dict[str, str]:
    path = _path(label)
    if not path.is_file():
        failures.append(f"{context}: missing artifact {label}")
        return {"path": label, "sha256": ""}
    if "INBOX" in path.as_posix().upper() or "/tmp/" in path.as_posix().lower():
        failures.append(f"{context}: temporary provenance path is not allowed: {label}")
    return {"path": path.relative_to(ROOT).as_posix(), "sha256": _sha256(path)}
    ####


def validate(registry_path: str | Path = REGISTRY) -> dict[str, Any]:
    """Validate all Alpha 3 contracts and emit a deterministic report."""

    path = Path(registry_path)
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise ValueError("Alpha 3 completion registry must have schema_version 1")
    failures: list[str] = []
    families_report: list[dict[str, Any]] = []
    families = payload.get("families")
    if not isinstance(families, list) or not families:
        raise ValueError("Alpha 3 completion registry requires families")
    seen: set[str] = set()
    for family in families:
        if not isinstance(family, dict):
            failures.append("family entry is not a mapping")
            continue
        family_id = str(family.get("id", ""))
        if not family_id or family_id in seen:
            failures.append(f"duplicate or missing family id: {family_id!r}")
        seen.add(family_id)
        qualification = str(family.get("qualification_class", ""))
        if qualification not in set(payload.get("qualification_classes", [])):
            failures.append(f"{family_id}: invalid qualification class {qualification!r}")
        family_failures: list[str] = []
        source = _required_path(str(family.get("source_parent", "")), family_failures, family_id)
        evidence_records = [
            _required_path(str(item), family_failures, f"{family_id} evidence")
            for item in family.get("evidence", [])
        ]
        if not evidence_records and family_id not in {"reference_nesc_two_stage_rocket"}:
            family_failures.append(f"{family_id}: no evidence records")
        overlay_records: list[dict[str, Any]] = []
        for overlay in family.get("overlays", []):
            if not isinstance(overlay, dict):
                family_failures.append(f"{family_id}: overlay is not a mapping")
                continue
            status = str(overlay.get("status", ""))
            if status not in ALLOWED_STATUSES:
                family_failures.append(f"{family_id}: invalid overlay status {status!r}")
            artifact = _required_path(str(overlay.get("artifact", "")), family_failures, f"{family_id} overlay")
            overlay_records.append({"id": overlay.get("id"), "kind": overlay.get("kind"), "status": status, **artifact})
        reduction_records: list[dict[str, Any]] = []
        for reduction in family.get("reductions", []):
            if not isinstance(reduction, dict):
                family_failures.append(f"{family_id}: reduction is not a mapping")
                continue
            status = str(reduction.get("status", ""))
            if status not in ALLOWED_STATUSES:
                family_failures.append(f"{family_id}: invalid reduction status {status!r}")
            artifact = _required_path(str(reduction.get("artifact", "")), family_failures, f"{family_id} reduction")
            reduction_records.append({"id": reduction.get("id"), "status": status, **artifact})
        deployment = None
        if isinstance(family.get("deployment"), dict):
            deployment = dict(family["deployment"])
            deployment_artifact = _required_path(str(deployment.get("artifact", "")), family_failures, f"{family_id} deployment")
            deployment.update(deployment_artifact)
        nonclaims = family.get("nonclaims")
        if not isinstance(nonclaims, list) or not nonclaims:
            family_failures.append(f"{family_id}: nonclaims are required")
        failures.extend(f"{family_id}: {item}" for item in family_failures)
        families_report.append(
            {
                "family_id": family_id,
                "qualification_class": qualification,
                "source_parent": source,
                "evidence": evidence_records,
                "overlays": overlay_records,
                "reductions": reduction_records,
                "deployment": deployment,
                "nonclaims": nonclaims,
                "failures": family_failures,
                "status": "verified" if not family_failures else "failed",
            }
        )
    ####
    pending = [
        {"family_id": family["family_id"], "id": reduction["id"], "status": reduction["status"]}
        for family in families_report
        for reduction in family["reductions"]
        if reduction["status"] == "equivalence_pending"
    ]
    return {
        "schema_version": "taoryx.daveml-alpha3-completion-report/v1",
        "registry": path.relative_to(ROOT).as_posix(),
        "claim_boundary": payload.get("claim_boundary", ""),
        "family_count": len(families_report),
        "families": families_report,
        "pending_reduction_equivalence": pending,
        "status": "verified_with_known_gaps" if not failures and pending else ("verified" if not failures else "failed"),
        "failures": failures,
    }
    ####


def main() -> int:
    report = validate()
    REPORT.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] != "failed" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
