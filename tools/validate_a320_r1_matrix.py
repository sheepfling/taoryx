#!/usr/bin/env python3
"""Run a fixed calibrated operating-point R1 matrix for the A320 reductions.

Every case uses the existing A320 truth evaluator and shared racetrack
contract.  Only the OpenAP operating point changes; a failed case remains
visible as boundary evidence and never becomes a nominal promotion.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

try:
    from validate_a320_racetrack import run_case
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_a320_racetrack import run_case

from taoryx.trajectory import A320OpenAPOperatingPoint

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_a320_r1"
MODES = ("point_mass_3dof", "pseudo_6dof_kinematic_bridge")
CASES: dict[str, dict[str, float]] = {
    "altitude_minus_500m": {"altitude_m": 10000.0, "mach": 0.78, "mass_kg": 60000.0},
    "altitude_plus_500m": {"altitude_m": 11000.0, "mach": 0.78, "mass_kg": 60000.0},
    "mach_minus_020": {"altitude_m": 10500.0, "mach": 0.76, "mass_kg": 60000.0},
    "mach_plus_020": {"altitude_m": 10500.0, "mach": 0.80, "mass_kg": 60000.0},
    "mass_minus_2pct": {"altitude_m": 10500.0, "mach": 0.78, "mass_kg": 58800.0},
    "mass_plus_2pct": {"altitude_m": 10500.0, "mach": 0.78, "mass_kg": 61200.0},
}


def run_matrix(output: Path) -> dict[str, object]:
    """Execute both A320 reduced tiers over all declared operating points."""

    records: list[dict[str, Any]] = []
    output.mkdir(parents=True, exist_ok=True)
    for mode in MODES:
        for case_id, values in CASES.items():
            operating_point = A320OpenAPOperatingPoint(**values)
            packet, rows = run_case(mode, 0.2, operating_point)
            case_dir = output / mode / case_id
            case_dir.mkdir(parents=True, exist_ok=True)
            (case_dir / "evidence.json").write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            if rows:
                import csv

                with (case_dir / "telemetry.csv").open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
            records.append(
                {
                    "mode": mode,
                    "case_id": case_id,
                    "operating_point": values,
                    "mission_pass": bool(packet["evaluation"]["mission_pass"]),
                    "numerical_valid": bool(packet["runtime"]["numerical_valid"]),
                    "required_objectives": packet["evaluation"].get("required_objectives"),
                    "required_passed": packet["evaluation"].get("required_passed"),
                    "artifact": str((case_dir / "evidence.json").relative_to(ROOT)),
                }
            )
    passed = sum(bool(record["mission_pass"]) for record in records)
    report: dict[str, object] = {
        "schema": "taoryx.a320-r1-matrix/v1alpha1",
        "status": "R1_fixed_matrix_complete",
        "claim": "Fixed calibrated operating-point evidence for the A320 point-mass and pseudo-6DOF racetrack reductions.",
        "claim_boundary": "This is deterministic reduced-tier development evidence, not a physical-surface, manufacturer-aircraft, wind, statistical reliability, or full-envelope qualification.",
        "case_count": len(records),
        "passed_case_count": passed,
        "failed_case_count": len(records) - passed,
        "modes": list(MODES),
        "cases": records,
        "operating_point_contract": {
            "source_model": "OpenAP calibrated A320 performance model",
            "nominal": {"altitude_m": 10500.0, "mach": 0.78, "mass_kg": 60000.0},
            "dimensions": list(CASES),
            "failure_policy": "retain failed truth objectives and hard gates as boundary evidence",
        },
        "reproduction": "PYTHONPATH=src python3 tools/validate_a320_r1_matrix.py",
    }
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report


def main() -> int:
    """Run and write the A320 reduced-tier R1 matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    report = run_matrix(arguments.output)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
