#!/usr/bin/env python3
"""Run the fixed reduced-fidelity R1 matrix for the reference F-16.

The matrix exercises the same shared racetrack runner used by the nominal
point-mass and pseudo-6DOF cases.  It is deliberately separate from the
physical-surface witness: these cases prove reduced-tier perturbation
behavior and do not promote surface or actuator claims.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

try:
    from validate_f16_racetrack import run_case
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_f16_racetrack import run_case

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_f16_r1"
MODES = ("point_mass_3dof", "pseudo_6dof_kinematic_bridge")
CASES: dict[str, dict[str, float]] = {
    "initial_position_offset": {
        "initial_north_offset_m": 50.0,
        "initial_east_offset_m": -40.0,
    },
    "initial_speed_offset": {"initial_speed_offset_m_s": 5.0},
    "initial_bank_offset": {"initial_bank_offset_rad": 0.08},
    "initial_altitude_boundary": {"initial_altitude_offset_m": 25.0},
}


def _write_telemetry(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    """Write deterministic flat telemetry for one matrix case."""

    if not rows:
        path.write_text("\n", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    ####


def run_matrix(output: Path) -> dict[str, object]:
    """Execute both reduced tiers over the declared perturbation cases."""

    records: list[dict[str, Any]] = []
    output.mkdir(parents=True, exist_ok=True)
    for mode in MODES:
        for case_id, perturbation in CASES.items():
            packet, rows = run_case(mode, None, 2.0, perturbation)
            case_dir = output / mode / case_id
            case_dir.mkdir(parents=True, exist_ok=True)
            (case_dir / "evidence.json").write_text(
                json.dumps(packet, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            _write_telemetry(case_dir / "telemetry.csv", rows)
            evaluation = packet["evaluation"]
            assert isinstance(evaluation, dict)
            runtime = packet["runtime"]
            assert isinstance(runtime, dict)
            artifact_path = case_dir / "evidence.json"
            artifact_name = (
                artifact_path.relative_to(ROOT).as_posix()
                if artifact_path.is_relative_to(ROOT)
                else artifact_path.as_posix()
            )
            records.append(
                {
                    "mode": mode,
                    "case_id": case_id,
                    "perturbation": perturbation,
                    "mission_pass": bool(evaluation["mission_pass"]),
                    "numerical_valid": bool(runtime["numerical_valid"]),
                    "required_objectives": evaluation.get("required_objectives"),
                    "required_passed": evaluation.get("required_passed"),
                    "artifact": artifact_name,
                }
            )
    passed = sum(bool(record["mission_pass"]) for record in records)
    report: dict[str, object] = {
        "schema": "taoryx.f16-r1-matrix/v1alpha1",
        "status": "R1_fixed_matrix_complete",
        "claim": "Fixed reduced-tier perturbation evidence for the reference F-16 point-mass and pseudo-6DOF racetrack models.",
        "claim_boundary": "This is deterministic reduced-tier development evidence. It does not establish physical surface allocation, manufacturer-aircraft fidelity, wind robustness, statistical reliability, or full-envelope qualification. The altitude-boundary case is retained when its truth gates fail.",
        "case_count": len(records),
        "passed_case_count": passed,
        "failed_case_count": len(records) - passed,
        "modes": list(MODES),
        "cases": records,
        "perturbation_contract": {
            "dimensions": list(CASES),
            "initial_state_only": True,
            "failure_policy": "retain failed truth objectives and hard gates as boundary evidence",
        },
        "reproduction": "PYTHONPATH=src python3 tools/validate_f16_r1_matrix.py",
    }
    (output / "manifest.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run and write the F-16 reduced-tier R1 matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    report = run_matrix(arguments.output)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
