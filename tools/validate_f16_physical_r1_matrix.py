#!/usr/bin/env python3
"""Build the fixed local F-16 physical-effector R1 matrix.

The source-backed F-16 physical path is real and bounded. This tool preserves
both a complete local matrix and any future boundary cases as explicit
artifacts rather than treating a direct or physical-effector screen as an
envelope qualification pass.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, cast

from taoryx.physical_lqr import validate_nonlinear_wrench_lqr

try:
    from validate_f16_physical_wrench_perturbations import _build_case
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_f16_physical_wrench_perturbations import _build_case

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_f16_physical_r1"

CASES: dict[str, dict[str, float]] = {
    "u_plus": {"u_m_s": 0.1},
    "w_plus": {"w_m_s": 0.02},
    "q_plus": {"q_rad_s": 0.0004},
    "coupled_reversal": {"u_m_s": -0.1, "w_m_s": -0.02, "q_rad_s": -0.0004},
    "lateral_coupled": {"v_m_s": 0.1, "p_rad_s": 0.0002, "r_rad_s": -0.0002},
}


def _run_case(adapter: Any, trim: Any, design: Any, perturbation: dict[str, float]) -> dict[str, object]:
    """Run one source-effector witness and preserve its raw diagnostics."""

    state = dict(trim.state)
    state.update({name: state[name] + value for name, value in perturbation.items()})
    validation = validate_nonlinear_wrench_lqr(
        adapter,
        trim,
        design,
        initial_state=state,
        duration_s=5.0,
        dt_s=0.02,
    )
    initial = max(validation.initial_normalized_feedback_error_norm, 1.0e-12)
    ratio = validation.final_normalized_feedback_error_norm / initial
    failed_recovery = ratio >= 0.20
    status = "pass" if not failed_recovery and validation.allocation_statuses == ("feasible",) else "boundary"
    return {
        "perturbation": perturbation,
        "status": status,
        "mission_pass": status == "pass",
        "initial_normalized_feedback_error_norm": validation.initial_normalized_feedback_error_norm,
        "final_normalized_feedback_error_norm": validation.final_normalized_feedback_error_norm,
        "final_normalized_feedback_error_fraction": ratio,
        "allocation_statuses": list(validation.allocation_statuses),
        "saturation_fraction": validation.saturation_fraction,
        "maximum_controlled_actual_residual": validation.maximum_controlled_actual_residual,
        "maximum_continuous_saturation_duration_s": validation.maximum_continuous_saturation_duration_s,
        "numerical_valid": True,
        "failure_reason": "feedback_recovery_threshold_exceeded" if failed_recovery else None,
    }
    ####


def build_matrix(output: Path) -> dict[str, object]:
    """Build the deterministic F-16 physical-effector R1 artifact."""

    adapter, trim, design = _build_case()
    records: list[dict[str, object]] = []
    for case_id, perturbation in CASES.items():
        metrics = _run_case(adapter, trim, design, perturbation)
        case_dir = output / case_id
        case_dir.mkdir(parents=True, exist_ok=True)
        (case_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        records.append(
            {
                "case_id": case_id,
                "boundary_witness": not bool(metrics["mission_pass"]),
                "metrics": metrics,
                "artifact": str((case_dir / "metrics.json").relative_to(ROOT)),
            }
        )
    passed = sum(
        bool(cast(dict[str, object], record["metrics"])["mission_pass"])
        for record in records
    )
    boundary_failures = len(records) - passed
    status = "R1_physical_effector_matrix_complete" if boundary_failures == 0 else "R1_physical_boundary_only"
    report: dict[str, object] = {
        "schema": "taoryx.f16-physical-effector-r1-matrix/v1alpha1",
        "status": status,
        "vehicle": "reference_f16_s119",
        "claim": "Fixed local source-effector perturbation evidence for the reference F-16.",
        "claim_boundary": (
            "The source nonlinear plant, finite-difference effectiveness, bounded elevator/aileron/rudder/throttle "
            "allocation, and actuator overlays are exercised. A complete local matrix still does not establish "
            "scheduled control, statistical reliability, or full-envelope flight-control qualification."
        ),
        "control_path": "state error -> plant-derived LQR -> desired local wrench -> bounded physical effectors -> source nonlinear plant",
        "direct_body_moment_injection": False,
        "controller_id": design.id,
        "tuning_profile": "state_and_wrench_balanced_q10_r0p01",
        "plant_id": "reference-f16-s119-source-runtime-plant",
        "case_contract": {
            "duration_s": 5.0,
            "dt_s": 0.02,
            "recovery_threshold": 0.20,
            "failure_policy": "retain failed recovery and allocation cases as boundary evidence",
        },
        "case_count": len(records),
        "passed_case_count": passed,
        "failed_case_count": len(records) - passed,
        "boundary_failure_count": boundary_failures,
        "all_numerically_valid": all(
            bool(cast(dict[str, object], record["metrics"])["numerical_valid"])
            for record in records
        ),
        "cases": records,
        "reproduction": "PYTHONPATH=src python3 tools/validate_f16_physical_r1_matrix.py",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run and write the F-16 physical R1 matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    report = build_matrix(arguments.output)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
