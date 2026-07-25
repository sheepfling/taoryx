"""Audit a TAORYX fidelity packet or family rollup without rerunning dynamics."""

from __future__ import annotations

import argparse
import hashlib
import json
import zipfile
from pathlib import Path
from typing import Any

from taoryx.trajectory.evaluation import TrajectoryEvaluation


def _digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _audit_objective_composite(report: object, family_id: str) -> dict[str, object]:
    """Recompute a family score from its packaged raw objective records."""

    if not isinstance(report, dict):
        raise ValueError(f"{family_id} objective evaluation is not a mapping")
    records = report.get("objectives")
    if not isinstance(records, list) or not records:
        raise ValueError(f"{family_id} objective evaluation has no raw objectives")
    total_weight = 0.0
    weighted_score = 0.0
    required: list[dict[str, object]] = []
    advisory: list[dict[str, object]] = []
    for record in records:
        if not isinstance(record, dict):
            raise ValueError(f"{family_id} contains a malformed objective record")
        weight = float(record["weight"])
        status = str(record["status"])
        normalized = record.get("normalized_error")
        total_weight += weight
        if status == "pass" and normalized is not None:
            weighted_score += weight * max(0.0, 1.0 - float(normalized))
        (required if record.get("severity") == "required" else advisory).append(record)
    expected_score = 100.0 * weighted_score / total_weight if total_weight else 0.0
    actual_score = float(report["score"])
    if abs(expected_score - actual_score) > 1.0e-9:
        raise ValueError(f"{family_id} objective score is not reproducible: {actual_score} != {expected_score}")
    expected_status = (
        "blocked" if any(item["status"] == "blocked" for item in required)
        else "fail" if any(item["status"] == "fail" for item in required)
        else "diagnostic" if any(item["status"] != "pass" for item in advisory)
        else "pass"
    )
    if report.get("status") != expected_status:
        raise ValueError(f"{family_id} objective status is not reproducible")
    return {"score": expected_score, "status": expected_status, "objective_count": len(records)}


def _audit_neutral_evaluation(report: object, label: str) -> dict[str, object]:
    """Validate the provider-neutral evaluation envelope without rerunning a case."""

    if not isinstance(report, dict):
        raise ValueError(f"{label} neutral evaluation is missing")
    try:
        evaluation = TrajectoryEvaluation.model_validate(report)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} neutral evaluation is invalid: {error}") from error
    return {
        "scenario_id": evaluation.scenario_id,
        "validity": evaluation.validity,
        "qualification": evaluation.qualification,
        "feasibility": evaluation.feasibility,
        "outcome": evaluation.outcome,
        "required_gates_pass": evaluation.required_gates_pass,
    }


def audit(archive: Path) -> dict[str, Any]:
    """Return a machine-readable packet audit and raise on structural failure."""

    with zipfile.ZipFile(archive) as handle:
        names = set(handle.namelist())
        absolute_entries = sorted(
            name for name in names if name.startswith(("/", "\\")) or "/Users/" in name or "/private/" in name
        )
        if absolute_entries:
            raise ValueError(f"archive contains absolute-path entries: {absolute_entries[:3]}")
        if "manifest.json" not in names and "rollup.json" not in names:
            raise ValueError("archive has neither manifest.json nor rollup.json")
        manifest_name = "manifest.json" if "manifest.json" in names else "rollup.json"
        manifest = json.loads(handle.read(manifest_name))
        result: dict[str, Any] = {
            "archive": str(archive),
            "archive_sha256": _digest(archive.read_bytes()),
            "manifest": manifest_name,
            "file_count": len(names),
            "absolute_path_entries": absolute_entries,
        }
        if manifest_name == "manifest.json":
            declared = dict(manifest.get("files", {}))
            missing = sorted(name for name in declared if name not in names)
            mismatched = sorted(
                name for name, expected in declared.items() if name in names and _digest(handle.read(name)) != expected
            )
            if missing or mismatched:
                raise ValueError(f"manifest hash audit failed: missing={missing[:3]}, mismatched={mismatched[:3]}")
            families = manifest.get("families", [])
            if not families:
                raise ValueError("packet manifest contains no families")
            result["families"] = [str(family["id"]) for family in families]
            result["tiers"] = list(manifest.get("tiers", []))
            result["objective_statuses"] = {
                str(family["id"]): family.get("long_validation", {}).get("objective_evaluation", {}).get("status")
                for family in families
            }
            result["objective_composites"] = {
                str(family["id"]): _audit_objective_composite(
                    family.get("long_validation", {}).get("objective_evaluation"), str(family["id"])
                )
                for family in families
            }
            result["neutral_evaluations"] = {
                str(family["id"]): _audit_neutral_evaluation(
                    family.get("long_validation", {}).get("nominal", {}).get("evaluation"),
                    f"{family['id']} long-validation",
                )
                for family in families
            }
            result["controller_neutral_evaluations"] = {
                str(item["id"]): _audit_neutral_evaluation(
                    item.get("evaluation"),
                    f"controller mission {item['id']}",
                )
                for item in manifest.get("controller_missions", ())
            }
            result["hashes_checked"] = len(declared)
        else:
            packets = manifest.get("packets", [])
            if len(packets) != 4:
                raise ValueError(f"rollup must contain four family packets, found {len(packets)}")
            result["families"] = [str(item["family"]) for item in packets]
            result["hashes_checked"] = 0
        result["status"] = "pass"
        return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--json", action="store_true", help="emit JSON rather than a compact status line")
    args = parser.parse_args()
    report = audit(args.archive)
    print(json.dumps(report, indent=2, sort_keys=True) if args.json else f"PASS: {args.archive}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
