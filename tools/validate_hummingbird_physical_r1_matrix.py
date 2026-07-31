#!/usr/bin/env python3
"""Run a bounded Hummingbird individual-rotor physical R1 matrix."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from taoryx.physical_lqr import design_physical_wrench_lqr, project_linearization_to_wrench, validate_nonlinear_wrench_lqr

try:
    from validate_hummingbird_physical_lqr import build_plant
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_hummingbird_physical_lqr import build_plant

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_hummingbird_physical_r1"
STATE_NAMES = ("roll_error_rad", "pitch_error_rad", "yaw_error_rad", "p_rad_s", "q_rad_s", "r_rad_s")
WRENCH_NAMES = ("moment_x_nm", "moment_y_nm", "moment_z_nm")
DISALLOWED_STATUSES = {"infeasible", "numerically_singular", "solver_failure"}
CASES: dict[str, dict[str, object]] = {
    "nominal_hover_attitude_rate": {
        "description": "declared local hover attitude/rate witness",
        "state_update": {
            "roll_error_rad": math.radians(5.0),
            "pitch_error_rad": math.radians(-3.0),
            "yaw_error_rad": math.radians(6.0),
            "p_rad_s": math.radians(8.0),
            "q_rad_s": math.radians(-6.0),
            "r_rad_s": math.radians(8.0),
        },
    },
    "yaw_reverse": {
        "description": "opposite-sign yaw and yaw-rate recovery witness",
        "state_update": {"yaw_error_rad": math.radians(-10.0), "r_rad_s": math.radians(-10.0)},
    },
    "coupled_rate_plus": {
        "description": "larger coupled attitude/rate interior witness",
        "state_update": {
            "roll_error_rad": math.radians(8.0),
            "pitch_error_rad": math.radians(-5.0),
            "yaw_error_rad": math.radians(10.0),
            "p_rad_s": math.radians(15.0),
            "q_rad_s": math.radians(-12.0),
            "r_rad_s": math.radians(15.0),
        },
    },
    "rotor_authority_boundary": {
        "description": "large coupled demand retained as an individual-rotor authority boundary",
        "state_update": {
            "roll_error_rad": math.radians(35.0),
            "pitch_error_rad": math.radians(-25.0),
            "yaw_error_rad": math.radians(40.0),
            "p_rad_s": math.radians(100.0),
            "q_rad_s": math.radians(-80.0),
            "r_rad_s": math.radians(100.0),
        },
        "expected_boundary": True,
    },
}


def _summary(validation: Any) -> dict[str, object]:
    """Return an independent physical-allocation pass calculation."""

    ratio = validation.final_feedback_error_norm / max(validation.initial_feedback_error_norm, 1.0e-12)
    statuses = set(validation.allocation_statuses)
    passed = (
        ratio <= 0.05
        and validation.saturation_fraction <= 0.10
        and validation.maximum_continuous_saturation_duration_s <= 0.10
        and not statuses.intersection(DISALLOWED_STATUSES)
    )
    return {
        "mission_pass": passed,
        "final_feedback_error_fraction": ratio,
        "final_controlled_actual_residual_nm": validation.final_controlled_actual_residual,
        "saturation_fraction": validation.saturation_fraction,
        "maximum_continuous_saturation_duration_s": validation.maximum_continuous_saturation_duration_s,
        "allocation_statuses": list(validation.allocation_statuses),
        "samples": len(validation.samples),
    }
    ####


def build_matrix(output: Path) -> dict[str, object]:
    """Build the fixed individual-rotor matrix around the source hover trim."""

    plant = build_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"Hummingbird physical R1 trim did not converge: {trim.as_dict()}")
    linearization = plant.linearize(
        trim,
        {
            "state_step": 1.0e-4,
            "control_step": 1.0,
            "comparison_factor": 0.5,
            "maximum_relative_difference": 0.10,
            "comparison_absolute_floor": 1.0e-7,
        },
    )
    effectiveness = plant.effectiveness(trim.state, trim.controls)
    projection = project_linearization_to_wrench(
        linearization,
        effectiveness,
        state_names=STATE_NAMES,
        wrench_names=WRENCH_NAMES,
        effector_names=tuple(plant.control_names),
    )
    design = design_physical_wrench_lqr(
        "hummingbird-source-hover-attitude-rate-wrench-lqr-v1",
        projection,
        q_diagonal=(20.0, 20.0, 10.0, 4.0, 4.0, 2.0),
        r_diagonal=(1.0, 1.0, 1.0),
        state_scales=(math.radians(10.0), math.radians(10.0), math.radians(15.0), math.radians(60.0), math.radians(60.0), math.radians(60.0)),
        wrench_scales=(0.10, 0.10, 0.05),
    )
    trim_state = dict(trim.state)
    records: list[dict[str, object]] = []
    for case_id, case in CASES.items():
        state = dict(trim_state)
        update = case.get("state_update")
        if isinstance(update, dict):
            state.update({str(name): float(value) for name, value in update.items()})
        validation = validate_nonlinear_wrench_lqr(
            plant,
            trim,
            design,
            initial_state=state,
            duration_s=4.0,
            dt_s=0.002,
        )
        metrics = _summary(validation)
        boundary = bool(case.get("expected_boundary", False))
        if boundary and bool(metrics["mission_pass"]):
            raise RuntimeError(f"expected Hummingbird authority boundary case {case_id!r} to fail")
        case_dir = output / case_id
        case_dir.mkdir(parents=True, exist_ok=True)
        metrics_path = case_dir / "metrics.json"
        metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        records.append(
            {
                "case_id": case_id,
                "description": case["description"],
                "mission_pass": bool(metrics["mission_pass"]),
                "boundary_witness": boundary,
                "metrics": metrics,
                "artifact": str(metrics_path.relative_to(ROOT)),
            }
        )
    return _write_report(output, trim, design, records)
    ####


def _write_report(output: Path, trim: Any, design: Any, records: list[dict[str, object]]) -> dict[str, object]:
    """Write the compact matrix manifest."""

    passed = sum(bool(record["mission_pass"]) for record in records)
    boundary_failures = sum(bool(record["boundary_witness"]) and not bool(record["mission_pass"]) for record in records)
    report: dict[str, object] = {
        "schema": "taoryx.hummingbird-physical-r1-matrix/v1alpha1",
        "status": "R1_physical_individual_rotor_matrix_complete",
        "vehicle": "hummingbird",
        "claim": "Fixed local Hummingbird individual-rotor allocation evidence around the RotorPy-derived hover trim.",
        "claim_boundary": "This is local hover physical-effector evidence. It does not establish translated flight, landing contact, battery discharge, blade-resolved aerodynamics, wind robustness, statistical reliability, or full-envelope multirotor qualification.",
        "control_path": "attitude/rate error -> plant-derived LQR -> desired body moments -> bounded four-rotor speed allocation -> motor lag -> nonlinear rigid-body plant",
        "direct_body_moment_injection": False,
        "effectors": [str(name) for name in design.projection.effector_names],
        "source_trim": trim.as_dict(),
        "selected_design": design.as_dict(),
        "case_count": len(records),
        "passed_case_count": passed,
        "failed_case_count": len(records) - passed,
        "boundary_failure_count": boundary_failures,
        "cases": records,
        "reproduction": "PYTHONPATH=src python3 tools/validate_hummingbird_physical_r1_matrix.py",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run and write the Hummingbird physical-surface R1 matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(build_matrix(arguments.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
