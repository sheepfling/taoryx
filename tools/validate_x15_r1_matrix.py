#!/usr/bin/env python3
"""Run a fixed staged-surrogate R1 matrix for the X-15 reduced tiers."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import replace
from pathlib import Path
from typing import Any, cast

from taoryx.contracts import Vector3
from taoryx.reachability_envelope import LaunchCommand, ReachabilityFidelity, simulate_rocket_glide
from taoryx.x15_reachability import build_x15_fidelity_evidence, x15_surrogate_vehicle

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_x15_r1"
FIDELITIES = (ReachabilityFidelity.POINT_MASS_3DOF, ReachabilityFidelity.PSEUDO_6DOF)
CASES: dict[str, dict[str, object]] = {
    "release_angle_low": {"elevation_deg": 35.0},
    "release_angle_high": {"elevation_deg": 55.0},
    "initial_speed_plus": {"elevation_deg": 45.0, "initial_speed_delta_m_s": 50.0},
    "booster_thrust_minus_10pct": {"elevation_deg": 45.0, "booster_thrust_scale": 0.90},
    "wind_vector_plus": {"elevation_deg": 45.0, "wind_velocity_m_s": (5.0, 0.0, 0.0)},
}


def run_matrix(output: Path) -> dict[str, object]:
    """Execute reduced X-15 staged witnesses over the declared cases."""

    output.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    for fidelity in FIDELITIES:
        for case_id, values in CASES.items():
            vehicle = x15_surrogate_vehicle()
            if "initial_speed_delta_m_s" in values:
                vehicle = replace(
                    vehicle,
                    initial_speed_m_s=vehicle.initial_speed_m_s + float(cast(float, values["initial_speed_delta_m_s"])),
                )
            if "booster_thrust_scale" in values:
                vehicle = replace(
                    vehicle,
                    booster_thrust_n=vehicle.booster_thrust_n * float(cast(float, values["booster_thrust_scale"])),
                )
            if "wind_velocity_m_s" in values:
                wind = values["wind_velocity_m_s"]
                assert isinstance(wind, tuple) and len(wind) == 3
                vehicle = replace(vehicle, wind_velocity_m_s=Vector3(*(float(component) for component in wind)))
            command = LaunchCommand(0.0, math.radians(float(cast(float, values.get("elevation_deg", 45.0)))))
            trajectory = simulate_rocket_glide(
                vehicle,
                command,
                fidelity=fidelity,
                step_size_s=1.0,
                horizon_s=700.0,
                spawn_children=True,
            )
            evidence = build_x15_fidelity_evidence(
                trajectory,
                fidelity=fidelity,
                command=command,
                step_size_s=1.0,
                horizon_s=700.0,
            )
            case_dir = output / fidelity.value / case_id
            case_dir.mkdir(parents=True, exist_ok=True)
            evidence_path = case_dir / "evidence.json"
            evidence_path.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            artifact_name = (
                evidence_path.relative_to(ROOT).as_posix()
                if evidence_path.is_relative_to(ROOT)
                else evidence_path.as_posix()
            )
            evaluation = evidence["evaluation"]
            assert isinstance(evaluation, dict)
            records.append(
                {
                    "fidelity": fidelity.value,
                    "case_id": case_id,
                    "parameters": dict(values),
                    "mission_pass": bool(evaluation["mission_pass"]),
                    "artifact": artifact_name,
                }
            )
    passed = sum(bool(record["mission_pass"]) for record in records)
    report: dict[str, object] = {
        "schema": "taoryx.x15-r1-matrix/v1alpha1",
        "status": "R1_fixed_matrix_complete",
        "claim": "Fixed staged-surrogate event-chain evidence for the X-15 point-mass and pseudo-6DOF tiers.",
        "claim_boundary": "This is deterministic X-15-scaled reduced-order evidence with explicit deployment events. It does not establish a controlled terminal handoff, native X-15 batch dynamics, physical stabilator/rudder/throttle/RCS allocation, wind robustness, statistical reliability, or full-envelope qualification.",
        "case_count": len(records),
        "passed_case_count": passed,
        "failed_case_count": len(records) - passed,
        "fidelities": [fidelity.value for fidelity in FIDELITIES],
        "cases": records,
        "perturbation_contract": {
            "dimensions": list(CASES),
            "deployment": "explicit spent-booster child and release event retained",
            "failure_policy": "retain failed event-chain or terminal witnesses as boundary evidence",
        },
        "reproduction": "PYTHONPATH=src python3 tools/validate_x15_r1_matrix.py",
    }
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run and write the X-15 staged-surrogate R1 matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(run_matrix(arguments.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
