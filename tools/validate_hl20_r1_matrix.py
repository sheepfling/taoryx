#!/usr/bin/env python3
"""Run a fixed open-loop R1 matrix for the HL-20 reduced tiers.

This matrix exercises the existing synthetic-booster/source-geometry release
API.  It is intentionally not a controller or surface-allocation result:
the vehicle is passive/open-loop and the pseudo tier adds only its declared
attitude response law.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, cast

from taoryx.contracts import Vector3
from taoryx.hl20_reachability import run_hl20_release
from taoryx.reachability_envelope import LaunchCommand, ReachabilityFidelity

try:
    from validate_hl20_fidelity_ladder import _tier_evidence
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_hl20_fidelity_ladder import _tier_evidence

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_hl20_r1"
FIDELITIES = (ReachabilityFidelity.POINT_MASS_3DOF, ReachabilityFidelity.PSEUDO_6DOF)
CASES: dict[str, dict[str, object]] = {
    "release_elevation_low": {"elevation_deg": 60.0},
    "release_elevation_high": {"elevation_deg": 80.0},
    "release_speed_low": {"elevation_deg": 70.0, "initial_speed_m_s": 225.0},
    "wind_vector_plus": {"elevation_deg": 70.0, "wind_velocity_m_s": (5.0, 0.0, 0.0)},
}


def run_matrix(output: Path) -> dict[str, object]:
    """Execute both passive reduced tiers over the declared release cases."""

    output.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    for fidelity in FIDELITIES:
        for case_id, values in CASES.items():
            elevation_deg = float(cast(float, values.get("elevation_deg", 70.0)))
            wind = values.get("wind_velocity_m_s", (0.0, 0.0, 0.0))
            assert isinstance(wind, tuple) and len(wind) == 3
            envelope = run_hl20_release(
                commands=(LaunchCommand(azimuth_rad=0.0, elevation_rad=math.radians(elevation_deg)),),
                fidelity=fidelity,
                step_size_s=1.0,
                horizon_s=180.0,
                spawn_children=False,
                initial_speed_m_s=float(cast(float, values.get("initial_speed_m_s", 250.0))),
                wind_velocity_m_s=Vector3(*(float(component) for component in wind)),
            )
            evidence = _tier_evidence(envelope, fidelity)
            case_dir = output / fidelity.value / case_id
            case_dir.mkdir(parents=True, exist_ok=True)
            evidence_path = case_dir / "evidence.json"
            envelope.write_json(case_dir / "envelope.json", include_trajectories=True)
            evidence_path.write_text(
                json.dumps(evidence, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            evaluation = evidence["evaluation"]
            assert isinstance(evaluation, dict)
            artifact_name = (
                evidence_path.relative_to(ROOT).as_posix()
                if evidence_path.is_relative_to(ROOT)
                else evidence_path.as_posix()
            )
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
        "schema": "taoryx.hl20-r1-matrix/v1alpha1",
        "status": "R1_fixed_matrix_complete",
        "claim": "Fixed open-loop release and passive energy witnesses for the HL-20 point-mass and pseudo-6DOF tiers.",
        "claim_boundary": "The booster and reduced aerodynamics are declared engineering surrogates. This matrix does not establish source-exact attitude dynamics, closed-loop control, physical surface allocation, thermal protection, statistical reliability, or full-envelope qualification.",
        "case_count": len(records),
        "passed_case_count": passed,
        "failed_case_count": len(records) - passed,
        "fidelities": [fidelity.value for fidelity in FIDELITIES],
        "cases": records,
        "perturbation_contract": {
            "dimensions": list(CASES),
            "control_authority": "not_applicable_passive_open_loop",
            "failure_policy": "retain failed terminal or phase witnesses as boundary evidence",
        },
        "reproduction": "PYTHONPATH=src python3 tools/validate_hl20_r1_matrix.py",
    }
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run and write the HL-20 reduced-tier R1 matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(run_matrix(arguments.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
