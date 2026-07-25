"""Audit milestone gates for a packaged TAORYX fidelity evidence packet."""

from __future__ import annotations

import argparse
import hashlib
import json
import zipfile
from pathlib import Path
from typing import Any

from taoryx.trajectory.evaluation import TrajectoryEvaluation

FAMILIES = {"b747", "skywalker_x8", "hummingbird", "x15"}
PASS = "pass"
DIAGNOSTIC = "diagnostic"
BLOCKED = "blocked"
NOT_RUN = "not_run"


def _digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()
####


def _status(
    name: str, passed: bool, *, diagnostic: bool = False, force_status: str | None = None, reason: str = ""
) -> dict[str, Any]:
    return {
        "id": name,
        "status": force_status or (PASS if passed else DIAGNOSTIC if diagnostic else BLOCKED),
        "reason": reason,
    }


def _neutral_evaluation(item: dict[str, Any], path: str) -> TrajectoryEvaluation | None:
    """Return a valid neutral evaluation, or ``None`` for an incomplete packet."""

    value: object = item
    for component in path.split("."):
        if not isinstance(value, dict):
            return None
        value = value.get(component)
    if not isinstance(value, dict):
        return None
    try:
        return TrajectoryEvaluation.model_validate(value)
    except (TypeError, ValueError):
        return None
    ####


def _snapshot_metadata_ok(handle: zipfile.ZipFile) -> bool:
    """Check that a packet can describe its complete dirty-worktree snapshot."""

    required = {
        "software_commit.txt",
        "working_tree.patch",
        "working_tree.status",
        "working_tree.untracked.json",
        "dependency.lock",
        "reproduce.sh",
    }
    names = set(handle.namelist())
    if not required <= names:
        return False
    try:
        manifest = json.loads(handle.read("working_tree.untracked.json"))
    except (KeyError, json.JSONDecodeError):
        return False
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        return False
    for item in manifest.get("files", ()):
        if not isinstance(item, dict):
            return False
        packet_path = item.get("packet_path")
        expected = item.get("sha256")
        if not isinstance(packet_path, str) or not isinstance(expected, str) or packet_path not in names:
            return False
        if _digest(handle.read(packet_path)) != expected:
            return False
    return True
    ####


def audit(archive: Path, reproduced_from: Path | None = None) -> dict[str, Any]:
    """Return milestone status and raise only for malformed packet structure."""

    with zipfile.ZipFile(archive) as handle:
        if "manifest.json" not in handle.namelist():
            raise ValueError("milestone audit requires an all-family packet manifest")
        manifest = json.loads(handle.read("manifest.json"))
        families = {str(item["id"]): item for item in manifest.get("families", [])}
        missing = sorted(FAMILIES - families.keys())
        if missing:
            raise ValueError(f"packet is missing required families: {missing}")

        m0 = _status(
            "M0-contract-freeze",
            bool(manifest.get("claim_boundary"))
            and all(item.get("scenario_identity") or item.get("long_validation", {}).get("catalog") for item in families.values()),
            reason="every family has a claim boundary and scenario/catalog identity",
        )
        m1 = _status(
            "M1-objective-scorer",
            all(
                item.get("long_validation", {}).get("objective_evaluation", {}).get("objectives")
                and (evaluation := _neutral_evaluation(item, "long_validation.nominal.evaluation")) is not None
                and evaluation.required_gates_pass
                for item in families.values()
            ),
            reason="every family packages raw objective records and a passing neutral evaluation",
        )
        m2 = _status(
            "M2-event-continuity",
            all(
                item.get("long_validation", {}).get("follow_on", {}).get("event_continuity_audit", {}).get("status") == PASS
                for item in families.values()
            ),
            reason="every family has a passing event continuity audit",
        )
        m3 = _status(
            "M3-parity-harness",
            bool(manifest.get("parity_report"))
            and set(manifest.get("tiers", []))
            >= {"point-mass-3dof", "kinematic-3-plus-3-dof", "rigid-body-6dof"},
            reason="packet declares all three fidelity tiers and embeds parity evidence",
        )

        closure_failures = [
            family_id
            for family_id, item in families.items()
            if item.get("long_validation", {}).get("follow_on", {}).get("closure_evaluation", {}).get("status") != PASS
        ]
        source_report = manifest.get("source_differential_report")
        m4 = _status(
            "M4-plant-evidence",
            bool(source_report)
            and not closure_failures
            and all(
                (evaluation := _neutral_evaluation(item, "long_validation.nominal.evaluation")) is not None
                and evaluation.required_gates_pass
                for item in families.values()
            ),
            reason=(
                "independent source report, closure, and neutral evaluation pass"
                if not closure_failures
                else f"closure failed: {closure_failures}"
            ),
        )

        mission_failures = [
            family_id
            for family_id, item in families.items()
            if item.get("long_validation", {}).get("objective_evaluation", {}).get("status") != PASS
            or item.get("long_validation", {}).get("follow_on", {}).get("exit_code") != 0
            or item.get("long_validation", {}).get("follow_on", {}).get("closure_evaluation", {}).get("status") != PASS
        ]
        m5 = _status(
            "M5-long-missions",
            not mission_failures
            and all(
                (evaluation := _neutral_evaluation(item, "long_validation.nominal.evaluation")) is not None
                and evaluation.required_gates_pass
                for item in families.values()
            ),
            reason=(
                "all required objectives, runtime, closure, convergence, and neutral evaluation gates pass"
                if not mission_failures
                else f"family gates failed: {mission_failures}"
            ),
        )

        controller_missions = tuple(dict(item) for item in manifest.get("controller_missions", ()))
        controller_families = {str(item.get("family")) for item in controller_missions}
        controller_failures = [
            str(item.get("id"))
            for item in controller_missions
            if item.get("objective_evaluation", {}).get("status") != PASS
            or item.get("closure_evaluation", {}).get("status") != PASS
            or item.get("event_continuity_audit", {}).get("status") != PASS
            or (evaluation := _neutral_evaluation(item, "evaluation")) is None
            or not evaluation.required_gates_pass
        ]
        missing_controller_families = FAMILIES - controller_families
        m5b = _status(
            "M5b-controller-mission-closure",
            controller_families == FAMILIES and not controller_failures,
            diagnostic=not controller_missions,
            reason=(
                "all declared controller missions have passing objectives, closure, continuity, convergence, and neutral evaluation"
                if controller_families == FAMILIES and not controller_failures
                else f"controller mission gates failed or missing families: {sorted(missing_controller_families | set(controller_failures))}"
            ),
        )

        absolute_entries = [
            name for name in handle.namelist() if name.startswith(("/", "\\")) or "/Users/" in name or "/private/" in name
        ]
        m6_status = DIAGNOSTIC
        m6_reason = "archive is hash-audited; clean-process reproduction must still be demonstrated"
        reproduction: dict[str, Any] | None = None
        if reproduced_from is not None:
            with zipfile.ZipFile(reproduced_from) as reproduced_handle:
                reproduced_manifest = json.loads(reproduced_handle.read("manifest.json"))
                reproduced_absolute = [
                    name
                    for name in reproduced_handle.namelist()
                    if name.startswith(("/", "\\")) or "/Users/" in name or "/private/" in name
                ]
                reproduced_snapshot_ok = _snapshot_metadata_ok(reproduced_handle)
            primary_scores = {
                str(item["id"]): item.get("long_validation", {}).get("objective_evaluation", {}).get("score")
                for item in families.values()
            }
            reproduced_scores = {
                str(item["id"]): item.get("long_validation", {}).get("objective_evaluation", {}).get("score")
                for item in reproduced_manifest.get("families", [])
            }
            primary_controller_scores = {
                str(item["id"]): item.get("objective_evaluation", {}).get("score")
                for item in controller_missions
            }
            reproduced_controller_scores = {
                str(item["id"]): item.get("objective_evaluation", {}).get("score")
                for item in reproduced_manifest.get("controller_missions", [])
            }
            reproduction_passed = (
                not absolute_entries
                and not reproduced_absolute
                and _snapshot_metadata_ok(handle)
                and reproduced_snapshot_ok
                and set(reproduced_scores) == set(primary_scores)
                and all(abs(float(reproduced_scores[key]) - float(primary_scores[key])) <= 1.0e-12 for key in primary_scores)
                and set(reproduced_controller_scores) == set(primary_controller_scores)
                and all(
                    abs(float(reproduced_controller_scores[key]) - float(primary_controller_scores[key])) <= 1.0e-12
                    for key in primary_controller_scores
                )
                and reproduced_manifest.get("tiers") == manifest.get("tiers")
                and bool(reproduced_manifest.get("parity_report"))
            )
            m6_status = PASS if reproduction_passed else BLOCKED
            m6_reason = "clean snapshot reproduced packet scores, tiers, and parity evidence" if reproduction_passed else "clean snapshot differs from the primary packet or lacks a complete source snapshot"
            reproduction = {
                "archive": str(reproduced_from),
                "archive_sha256": _digest(reproduced_from.read_bytes()),
                "passed": reproduction_passed,
                "primary_scores": primary_scores,
                "reproduced_scores": reproduced_scores,
                "primary_controller_scores": primary_controller_scores,
                "reproduced_controller_scores": reproduced_controller_scores,
            }
        m6 = _status(
            "M6-evidence-release",
            m6_status == PASS,
            force_status=m6_status,
            reason=m6_reason,
        )
        milestones = [m0, m1, m2, m3, m4, m5, m5b, m6]
        overall = PASS if all(item["status"] == PASS for item in milestones) else DIAGNOSTIC
        return {
            "archive": str(archive),
            "status": overall,
            "families": sorted(families),
            "milestones": milestones,
            "blocking_families": sorted(set(closure_failures + mission_failures + controller_failures)),
            "claim_boundary": manifest.get("claim_boundary"),
            "reproduction": reproduction,
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--reproduced-from", type=Path, help="clean-snapshot packet to compare for M6")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    result = audit(args.archive, args.reproduced_from)
    print(json.dumps(result, indent=2, sort_keys=True) if args.json else f"{result['status'].upper()}: {args.archive}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
