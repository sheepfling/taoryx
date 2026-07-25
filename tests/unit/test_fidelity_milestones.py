from __future__ import annotations

import importlib.util
import json
import zipfile
from pathlib import Path

ROOT = Path(__file__).parents[2]
SPEC = importlib.util.spec_from_file_location("audit_fidelity_milestones", ROOT / "tools/audit_fidelity_milestones.py")
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _packet(tmp_path: Path, *, x15_closure: str = "pass") -> Path:
    def evaluation(scenario_id: str) -> dict[str, object]:
        return {
            "schema_version": 1,
            "scenario_id": scenario_id,
            "validity": "valid",
            "qualification": "extended",
            "feasibility": "unknown",
            "outcome": "completed",
            "metrics": [],
            "gates": [{"id": "objective-report", "status": "pass", "message": "pass", "metric_ids": []}],
            "requested_controls": [],
            "achieved_controls": [],
            "resources": [],
            "events": [],
            "closure": [{
                "id": "closure-translation",
                "actual": 0.0,
                "target": 0.0,
                "tolerance": 1.0,
                "unit": "1",
                "status": "pass",
                "source": "test",
            }],
            "convergence": [{
                "id": "convergence-gate",
                "actual": 1.0,
                "target": 1.0,
                "tolerance": 0.5,
                "unit": "1",
                "status": "pass",
                "source": "test",
            }],
            "claim_boundary": "test evidence",
        }

    families = []
    for family in sorted(MODULE.FAMILIES):
        families.append(
            {
                "id": family,
                "scenario_identity": {"id": family},
                "long_validation": {
                    "catalog": {"id": family},
                    "nominal": {"evaluation": evaluation(f"{family}-long")},
                    "objective_evaluation": {"objectives": [{"id": "duration"}], "status": "pass", "score": 100.0},
                    "follow_on": {
                        "event_continuity_audit": {"status": "pass"},
                        "closure_evaluation": {"status": x15_closure if family == "x15" else "pass"},
                        "exit_code": 0,
                    },
                },
            }
        )
    manifest = {
        "claim_boundary": "research surrogate",
        "families": families,
        "tiers": ["point-mass-3dof", "kinematic-3-plus-3-dof", "rigid-body-6dof"],
        "parity_report": "evidence/parity.json",
        "source_differential_report": "evidence/source.json",
        "controller_missions": [
            {
                "id": mission_id,
                "family": family,
                "objective_evaluation": {"status": "pass", "score": 100.0},
                "closure_evaluation": {"status": "pass"},
                "event_continuity_audit": {"status": "pass"},
                "evaluation": evaluation(f"{mission_id}-mission"),
            }
            for family, mission_id in (
                ("b747", "b747-integrated-route-transition"),
                ("skywalker_x8", "skywalker-x8-powered-rectangle"),
                ("hummingbird", "hummingbird-elevated-rectangle"),
                ("x15", "x15-source-trim-propnav-transition"),
            )
        ],
        "files": {"manifest.json": "placeholder"},
    }
    archive = tmp_path / "packet.zip"
    with zipfile.ZipFile(archive, "w") as handle:
        handle.writestr("manifest.json", json.dumps(manifest))
        handle.writestr("software_commit.txt", "git_commit=test\n")
        handle.writestr("working_tree.patch", "")
        handle.writestr("working_tree.status", "")
        handle.writestr("dependency.lock", "python_version=3.12\n")
        handle.writestr("reproduce.sh", "#!/bin/sh\n")
        handle.writestr(
            "working_tree.untracked.json",
            json.dumps({"schema_version": 1, "base_commit": "test", "files": []}),
        )
    return archive


def test_milestone_audit_passes_structural_packet(tmp_path: Path) -> None:
    result = MODULE.audit(_packet(tmp_path))
    assert result["status"] == "diagnostic"  # M6 remains diagnostic until clean checkout is demonstrated.
    assert {item["id"] for item in result["milestones"][:5]} == {
        "M0-contract-freeze",
        "M1-objective-scorer",
        "M2-event-continuity",
        "M3-parity-harness",
        "M4-plant-evidence",
    }
    assert all(item["status"] == "pass" for item in result["milestones"][:5])


def test_milestone_audit_blocks_failed_closure(tmp_path: Path) -> None:
    result = MODULE.audit(_packet(tmp_path, x15_closure="fail"))
    statuses = {item["id"]: item["status"] for item in result["milestones"]}
    assert statuses["M4-plant-evidence"] == "blocked"
    assert statuses["M5-long-missions"] == "blocked"
    assert "x15" in result["blocking_families"]


def test_milestone_audit_accepts_reproduced_packet(tmp_path: Path) -> None:
    primary = _packet(tmp_path)
    reproduced_dir = tmp_path / "reproduced"
    reproduced_dir.mkdir()
    reproduced = _packet(reproduced_dir)
    result = MODULE.audit(primary, reproduced)
    assert result["milestones"][-1]["status"] == "pass"
    assert result["reproduction"]["passed"] is True


def test_milestone_audit_requires_all_controller_missions(tmp_path: Path) -> None:
    """M5b cannot pass from a weighted plant score alone."""

    archive = _packet(tmp_path)
    result = MODULE.audit(archive)
    statuses = {item["id"]: item["status"] for item in result["milestones"]}
    assert statuses["M5b-controller-mission-closure"] == "pass"
