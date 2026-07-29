"""Run the fixed R1 initial-condition perturbation matrix for the F-16 route."""

from __future__ import annotations

import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from validate_f16_racetrack import run_case

ROOT = Path(__file__).resolve().parents[1]
CASES: tuple[tuple[str, dict[str, float]], ...] = (
    ("north_offset_plus_100m", {"initial_north_offset_m": 100.0}),
    ("east_offset_minus_100m", {"initial_east_offset_m": -100.0}),
    ("speed_offset_plus_2mps", {"initial_speed_offset_m_s": 2.0}),
    ("bank_offset_plus_2deg", {"initial_bank_offset_rad": math.radians(2.0)}),
)


def _run_case(item: tuple[str, dict[str, float]]) -> dict[str, Any]:
    """Execute one fixed perturbation in a worker process."""

    case_id, perturbation = item
    evidence, rows = run_case("surface_allocated", None, 0.5, perturbation)
    results = evidence["evaluation"]["results"]
    failed = next((result for result in results if result["status"] != "pass"), None)
    return {
        "id": case_id,
        "perturbation": perturbation,
        "mission_pass": evidence["evaluation"]["mission_pass"],
        "required_passed": evidence["evaluation"]["required_passed"],
        "required_objectives": evidence["evaluation"]["required_objectives"],
        "numerical_valid": evidence["runtime"]["numerical_valid"],
        "first_failed_objective": None if failed is None else failed["id"],
        "first_failed_status": None if failed is None else failed["status"],
        "terminal_time_s": results[-1].get("truth_time_s") if results else None,
        "sample_count": len(rows),
    }
    ####


def main() -> int:
    """Generate a fixed, reproducible R1 robustness artifact."""

    try:
        with ProcessPoolExecutor(max_workers=len(CASES)) as executor:
            cases = list(executor.map(_run_case, CASES))
        execution_backend = "process_pool"
    except PermissionError:
        # Some constrained CI/macOS sandboxes deny POSIX semaphore creation.
        # Threads preserve the same deterministic case definitions and keep the
        # tool runnable without weakening the evidence contract.
        with ThreadPoolExecutor(max_workers=len(CASES)) as executor:
            cases = list(executor.map(_run_case, CASES))
        execution_backend = "thread_pool_fallback"
    passed = all(bool(case["mission_pass"]) and bool(case["numerical_valid"]) for case in cases)
    report = {
        "schema_version": "taoryx.f16-racetrack-robustness/v1",
        "status": "R1_fixed_matrix_nominal_pass" if passed else "R1_fixed_matrix_boundary_failure_recorded",
        "tier": "R1_fixed_initial_condition_matrix",
        "family_id": "reference_f16_s119",
        "vehicle_id": "reference_f16_s119",
        "fidelity": "rigid_body_6dof_surface_allocated",
        "control_path": "physical_effectors",
        "base_step_s": 0.5,
        "execution_backend": execution_backend,
        "cases": cases,
        "summary": {
            "case_count": len(cases),
            "passed_case_count": sum(bool(case["mission_pass"]) for case in cases),
            "failed_case_count": sum(not bool(case["mission_pass"]) for case in cases),
            "all_numerically_valid": all(bool(case["numerical_valid"]) for case in cases),
        },
        "claim": "Fixed initial-condition perturbation evidence for the locally surface-allocated F-16 racetrack.",
        "claim_boundary": (
            "This is a development R1 witness matrix, not a statistical reliability claim, wind robustness "
            "claim, scheduled-controller qualification, or full-envelope flight-control qualification."
        ),
    }
    output = ROOT / "verification/f16_racetrack_robustness_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
