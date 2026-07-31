#!/usr/bin/env python3
"""Run a bounded NESC response-law/source-replay R1 matrix.

The matrix varies only the declared pseudo attitude-response realization and
initial attitude.  Source translation, stage sequence, and mass history stay
fixed.  No gimbal effectiveness is invented.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, cast

from taoryx.trajectory import build_nesc_composite_pseudo6dof
from taoryx.trajectory.nesc_pseudo6dof import DEFAULT_REDUCTION

try:
    from validate_nesc_fidelity_ladder import _record
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_nesc_fidelity_ladder import _record

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_nesc_r1"
CASES: dict[str, dict[str, object]] = {
    "response_lag_fast": {"response_time_scale": 0.8, "initial_attitude_rad": (0.0, 0.0, 0.0)},
    "response_lag_slow": {"response_time_scale": 1.2, "initial_attitude_rad": (0.0, 0.0, 0.0)},
    "initial_attitude_offset": {"response_time_scale": 1.0, "initial_attitude_rad": (0.05, -0.04, 0.08)},
    "nominal_replay_integrity": {"response_time_scale": 1.0, "initial_attitude_rad": (0.0, 0.0, 0.0)},
}


def run_matrix(output: Path) -> dict[str, object]:
    """Execute source replay and response-law variants."""

    source = json.loads(DEFAULT_REDUCTION.read_text(encoding="utf-8"))
    source_rows = source["history"]
    assert isinstance(source_rows, list)
    output.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []
    for case_id, values in CASES.items():
        scale = float(cast(float, values["response_time_scale"]))
        attitude = values["initial_attitude_rad"]
        assert isinstance(attitude, tuple) and len(attitude) == 3
        result = build_nesc_composite_pseudo6dof(
            DEFAULT_REDUCTION,
            response_time_scale=scale,
            initial_attitude_rad=cast(tuple[float, float, float], tuple(float(value) for value in attitude)),
        )
        pseudo = _record(
            fidelity="pseudo_6dof",
            profile_id=result.profile_id,
            mission_pass=result.passed,
            rows=list(result.rows),
            claim="The retained NESC source translation history remains paired with a bounded response-law variant.",
        )
        pseudo["variant"] = dict(values)
        pseudo["response_checks"] = result.as_dict()["checks"]
        case_dir = output / case_id
        case_dir.mkdir(parents=True, exist_ok=True)
        pseudo_path = case_dir / "pseudo_6dof_evidence.json"
        pseudo_path.write_text(json.dumps(pseudo, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        records.append(
            {
                "case_id": case_id,
                "fidelity": "pseudo_6dof",
                "parameters": dict(values),
                "mission_pass": result.passed,
                "artifact": str(pseudo_path.relative_to(ROOT)) if pseudo_path.is_relative_to(ROOT) else pseudo_path.as_posix(),
            }
        )
        point = _record(
            fidelity="point_mass_3dof",
            profile_id="nesc_rocket.performance_3dof.v1",
            mission_pass=source.get("status") == "verified" and bool(source_rows),
            rows=[row for row in source_rows if isinstance(row, dict)],
            claim="The retained NESC source translation history preserves stage sequence and mass flow for the paired R1 case.",
        )
        point["variant"] = dict(values)
        point_path = case_dir / "point_mass_3dof_evidence.json"
        point_path.write_text(json.dumps(point, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        records.append(
            {
                "case_id": case_id,
                "fidelity": "point_mass_3dof",
                "parameters": dict(values),
                "mission_pass": bool(point["evaluation"]["mission_pass"]),
                "artifact": str(point_path.relative_to(ROOT)) if point_path.is_relative_to(ROOT) else point_path.as_posix(),
            }
        )
    passed = sum(bool(record["mission_pass"]) for record in records)
    report: dict[str, object] = {
        "schema": "taoryx.nesc-r1-matrix/v1alpha1",
        "status": "R1_fixed_matrix_complete",
        "claim": "Fixed source-translation and bounded attitude-response sensitivity evidence for the NESC reduced tiers.",
        "claim_boundary": "The source stage/mass/translation history remains fixed. The pseudo tier varies only its declared response-law lag and initial attitude. No source-exact gimbal effectiveness, plume interaction, staging-control law, statistical reliability, or full-envelope qualification is claimed.",
        "case_count": len(records),
        "passed_case_count": passed,
        "failed_case_count": len(records) - passed,
        "cases": records,
        "perturbation_contract": {
            "dimensions": list(CASES),
            "source_translation_fixed": True,
            "physical_gimbal_allocation": False,
        },
        "reproduction": "PYTHONPATH=src python3 tools/validate_nesc_r1_matrix.py",
    }
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run and write the NESC reduced-tier R1 matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(run_matrix(arguments.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
