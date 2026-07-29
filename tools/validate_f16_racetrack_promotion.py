"""Audit promotion evidence for the local F-16 physical-effector path."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
PACKET = ROOT / "artifacts/showcases/f16-s119-racetrack-fidelity-ladder/6dof-surfaces"
OUTPUT = ROOT / "verification/f16_t5_local_promotion_evidence.json"


def _load(path: Path) -> dict[str, Any]:
    """Load one JSON evidence artifact."""

    return json.loads(path.read_text(encoding="utf-8"))
    ####


def validate() -> dict[str, Any]:
    """Require agreement among the local nonlinear evidence layers."""

    evaluation = _load(PACKET / "evaluation.json")
    envelope = _load(PACKET / "envelope_report.json")
    convergence = _load(PACKET / "convergence_report.json")
    coverage = _load(PACKET / "control_coverage.json")
    closure = _load(PACKET / "equation_closure_report.json")
    robustness = _load(PACKET / "robustness_report.json")
    maneuvers = _load(ROOT / "verification/f16_local_maneuver_evidence.json")
    trim_hold = _load(ROOT / "verification/f16_physical_wrench_lqr_evidence.json")
    operating_points = _load(ROOT / "verification/f16_operating_points_evidence.json")
    sea_level = next(
        point for point in operating_points["points"] if point["id"] == "f16-sea-level-152mps"
    )

    checks: dict[str, bool] = {
        "surface_route_truth_objectives": bool(evaluation["mission_pass"])
        and evaluation["required_passed"] == evaluation["required_objectives"],
        "surface_route_hard_gates": bool(evaluation["hard_gates_passed"]),
        "source_envelope": envelope["status"] == "pass",
        "semantic_step_convergence": convergence["status"] == "semantic_gate_convergence_pass",
        "physical_path_no_nominal_saturation": coverage["saturation_fraction"] == 0.0,
        "source_load_closure": closure["status"] == "pass",
        "trim_hold": trim_hold["status"] == "development_screen_passed",
        "representative_maneuvers": maneuvers["status"] == "development_screen_passed"
        and all(bool(result["passed"]) for result in maneuvers["phases"].values()),
        "sea_level_operating_point": sea_level["max_residual"] <= operating_points["acceptance"]["maximum_absolute_trim_residual"]
        and sea_level["linearization"]["derivative_consistent"]
        and sea_level["physical_effectiveness"]["rank"] == 4
        and sea_level["local_lqr"]["hurwitz"]
        and sea_level["local_lqr"]["trim_hold_validation"]["saturation_fraction"] == 0.0,
    }
    passed = all(checks.values())
    return {
        "schema_version": "taoryx.f16-t5-local-promotion/v1",
        "status": "development_t5_local_pass" if passed else "development_t5_local_pending",
        "qualification_tier": "T5_local_nonlinear_validation",
        "family_id": "reference_f16_s119",
        "operating_point_id": "f16-sea-level-152mps",
        "checks": checks,
        "evidence": {
            "surface_packet": "artifacts/showcases/f16-s119-racetrack-fidelity-ladder/6dof-surfaces",
            "representative_maneuvers": "verification/f16_local_maneuver_evidence.json",
            "trim_hold": "verification/f16_physical_wrench_lqr_evidence.json",
            "operating_points": "verification/f16_operating_points_evidence.json",
        },
        "robustness": {
            "status": robustness["status"],
            "claim": "R1 boundary evidence is retained separately and is not converted into a release robustness pass.",
        },
        "claim": (
            "One source-grounded F-16 S-119 local operating point passes the nonlinear physical-effector "
            "racetrack and representative bank/pitch maneuver evidence through bounded elevator, aileron, "
            "rudder, and throttle allocation."
        ),
        "nonclaims": [
            "continuous gain scheduling between operating points",
            "full-envelope or flight qualification",
            "wind, sensor, or release robustness qualification",
            "manufacturer flight-control fidelity",
            "fuel-flow or endurance qualification",
            "TAOS 96 runtime compatibility",
        ],
    }
    ####


def main() -> int:
    """Write the promotion audit artifact."""

    report = validate()
    OUTPUT.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "development_t5_local_pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
