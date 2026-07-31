#!/usr/bin/env python3
"""Run a fixed aggregate-thrust Hummingbird R1 matrix.

The pseudo run is the executable authority.  The 3DOF records are its
translation-only projection, matching the current Hummingbird fidelity
boundary; neither record claims individual motor allocation.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import replace
from math import radians
from pathlib import Path
from typing import Any

from taoryx.trajectory.hummingbird_pseudo6dof import HummingbirdPseudo6DOFModel

try:
    from validate_hummingbird_fidelity_ladder import _evaluate, _record, run_mission
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_hummingbird_fidelity_ladder import _evaluate, _record, run_mission

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_hummingbird_r1"
CASES: dict[str, str] = {
    "initial_position_offset": "initial position offset at grounded start",
    "initial_yaw_offset": "initial yaw offset before mission capture",
    "thrust_authority_minus_10pct": "aggregate maximum-thrust reduction",
    "battery_energy_minus_40pct": "aggregate battery-energy reduction",
}


def _case_model_and_state(case_id: str) -> tuple[HummingbirdPseudo6DOFModel, object | None]:
    """Resolve one declared model or initial-state perturbation."""

    model = HummingbirdPseudo6DOFModel()
    state = model.initial_state()
    if case_id == "initial_position_offset":
        state = replace(state, position_m=(0.15, -0.10, 0.0))
    elif case_id == "initial_yaw_offset":
        state = replace(state, attitude_rad=(0.0, 0.0, radians(10.0)))
    elif case_id == "thrust_authority_minus_10pct":
        model = replace(model, maximum_thrust_n=8.8)
        state = model.initial_state()
    elif case_id == "battery_energy_minus_40pct":
        model = replace(model, battery_energy_j=1_200.0)
        state = model.initial_state()
    else:
        raise ValueError(f"unknown Hummingbird R1 case: {case_id}")
    return model, state


def run_matrix(output: Path) -> dict[str, object]:
    """Execute the aggregate pseudo mission and its 3DOF projection."""

    output.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    for case_id in CASES:
        model, state = _case_model_and_state(case_id)
        rows, _ = run_mission(model=model, initial_state=state)
        evaluation = _evaluate(rows)
        for fidelity, profile_id in (
            ("pseudo_6dof", "hummingbird.attitude_response_p6dof.v1"),
            ("point_mass_3dof", "hummingbird.translation_projection_3dof.v1"),
        ):
            evidence = _record(rows, evaluation, fidelity, profile_id)
            case_dir = output / fidelity / case_id
            case_dir.mkdir(parents=True, exist_ok=True)
            evidence_path = case_dir / "evidence.json"
            evidence_path.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            artifact_name = (
                evidence_path.relative_to(ROOT).as_posix()
                if evidence_path.is_relative_to(ROOT)
                else evidence_path.as_posix()
            )
            records.append(
                {
                    "fidelity": fidelity,
                    "case_id": case_id,
                    "mission_pass": bool(evaluation["mission_pass"]),
                    "artifact": artifact_name,
                }
            )
    passed = sum(bool(record["mission_pass"]) for record in records)
    report: dict[str, object] = {
        "schema": "taoryx.hummingbird-r1-matrix/v1alpha1",
        "status": "R1_fixed_matrix_complete",
        "claim": "Fixed aggregate-thrust pseudo-6DOF and translation-projection R1 evidence for the Hummingbird mission.",
        "claim_boundary": "The pseudo tier uses an aggregate thrust-vector and bounded attitude-response surrogate. The 3DOF tier uses an independent translation-only force model. Neither tier establishes individual motor/rotor allocation, rotor inflow, reaction torque, physical moment balance, wind robustness, or statistical reliability.",
        "case_count": len(records),
        "passed_case_count": passed,
        "failed_case_count": len(records) - passed,
        "cases": CASES,
        "records": records,
        "perturbation_contract": {
            "dimensions": list(CASES),
            "authority_model": "aggregate_thrust_vector_surrogate",
            "failure_policy": "retain failed truth objectives as boundary evidence",
        },
        "reproduction": "PYTHONPATH=src python3 tools/validate_hummingbird_r1_matrix.py",
    }
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run and write the Hummingbird aggregate R1 matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(run_matrix(arguments.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
