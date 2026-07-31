#!/usr/bin/env python3
"""Run the bounded X8 source-coordinate physical-effector R1 matrix.

The matrix is deliberately local.  It exercises the plant-derived roll/pitch
LQR and bounded source-coordinate elevon allocation at the resolved X8 trim,
while retaining a high-demand beta/table-domain witness.  The two source
elevon coordinates map to left/right commands in the runtime, but the pair
does not provide an independent yaw-moment effector.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from taoryx.airbreathing_control_mapping import x8_source_mapping
from taoryx.physical_lqr import design_physical_wrench_lqr, project_linearization_to_wrench, validate_nonlinear_wrench_lqr

try:
    from validate_x8_physical_lqr import build_plant
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_x8_physical_lqr import build_plant

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_x8_physical_r1"
STATE_NAMES = ("roll_error_rad", "pitch_error_rad", "p_rad_s", "q_rad_s")
WRENCH_NAMES = ("moment_x_nm", "moment_y_nm")
EFFECTOR_NAMES = ("differential-elevon-deg", "collective-elevon-deg")
DISALLOWED_STATUSES = {"infeasible", "numerically_singular", "solver_failure"}
CASES: dict[str, dict[str, object]] = {
    "nominal_coupled": {
        "description": "source-trim coupled roll/pitch recovery",
        "state_update": {
            "roll_error_rad": math.radians(5.0),
            "pitch_error_rad": math.radians(-3.0),
            "p_rad_s": math.radians(4.0),
            "q_rad_s": math.radians(-3.0),
        },
    },
    "roll_reversal": {
        "description": "opposite-sign roll recovery through differential elevon",
        "state_update": {"roll_error_rad": math.radians(-3.0), "p_rad_s": math.radians(-1.5)},
    },
    "pitch_reversal": {
        "description": "opposite-sign pitch recovery through collective elevon",
        "state_update": {"pitch_error_rad": math.radians(4.0), "q_rad_s": math.radians(-2.0)},
    },
    "coupled_reversal": {
        "description": "simultaneous roll/pitch reversal inside the local source envelope",
        "state_update": {
            "roll_error_rad": math.radians(3.0),
            "pitch_error_rad": math.radians(-3.0),
            "p_rad_s": math.radians(1.5),
            "q_rad_s": math.radians(-1.5),
        },
    },
    "beta_table_boundary": {
        "description": "high-demand case retained at the declared beta table boundary",
        "state_update": {
            "roll_error_rad": math.radians(30.0),
            "pitch_error_rad": math.radians(-20.0),
            "p_rad_s": math.radians(90.0),
            "q_rad_s": math.radians(-70.0),
        },
        "expected_boundary": True,
    },
}


def _design(plant: Any, trim: Any) -> Any:
    """Build the source-trim roll/pitch controller used by every case."""

    linearization = plant.linearize(
        trim,
        {
            "state_step": 1.0e-4,
            "control_step": 1.0e-3,
            "comparison_factor": 0.5,
            "maximum_relative_difference": 0.10,
            "comparison_absolute_floor": 1.0e-8,
        },
    )
    effectiveness = plant.effectiveness(trim.state, trim.controls)
    projection = project_linearization_to_wrench(
        linearization,
        effectiveness,
        state_names=STATE_NAMES,
        wrench_names=WRENCH_NAMES,
        effector_names=EFFECTOR_NAMES,
    )
    design = design_physical_wrench_lqr(
        "skywalker-x8-source-trim-roll-pitch-wrench-lqr-v1",
        projection,
        q_diagonal=(16.0, 16.0, 3.0, 3.0),
        r_diagonal=(1.0, 1.0),
        state_scales=(math.radians(10.0), math.radians(10.0), math.radians(45.0), math.radians(45.0)),
        wrench_scales=(0.20, 0.20),
    )
    return linearization, effectiveness, design
    ####


def _metrics(validation: Any) -> dict[str, object]:
    """Summarize controlled-axis recovery without hiding unallocated yaw."""

    ratio = validation.final_feedback_error_norm / max(validation.initial_feedback_error_norm, 1.0e-12)
    statuses = set(validation.allocation_statuses)
    return {
        "mission_pass": (
            ratio <= 0.05
            and validation.final_controlled_actual_residual <= 0.005
            and validation.saturation_fraction <= 0.10
            and validation.maximum_continuous_saturation_duration_s <= 0.25
            and not statuses.intersection(DISALLOWED_STATUSES)
        ),
        "final_feedback_error_fraction": ratio,
        "final_controlled_actual_residual_nm": validation.final_controlled_actual_residual,
        "saturation_fraction": validation.saturation_fraction,
        "maximum_continuous_saturation_duration_s": validation.maximum_continuous_saturation_duration_s,
        "allocation_statuses": list(validation.allocation_statuses),
        "uncontrolled_wrench_axes": ["moment_z_nm"],
        "samples": len(validation.samples),
    }
    ####


def build_matrix(output: Path) -> dict[str, object]:
    """Build the local source-coordinate physical-effector matrix."""

    plant = build_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"X8 source-coordinate trim did not converge: {trim.as_dict()}")
    linearization, effectiveness, design = _design(plant, trim)
    records: list[dict[str, Any]] = []
    for case_id, case in CASES.items():
        state = dict(trim.state)
        update = case.get("state_update")
        if isinstance(update, dict):
            state.update({str(name): float(value) for name, value in update.items()})
        boundary = bool(case.get("expected_boundary", False))
        try:
            validation = validate_nonlinear_wrench_lqr(
                plant,
                trim,
                design,
                initial_state=state,
                duration_s=4.0,
                dt_s=0.01,
            )
            metrics = _metrics(validation)
            if boundary and bool(metrics["mission_pass"]):
                raise RuntimeError(f"expected X8 boundary case {case_id!r} to remain a boundary")
            record_status = "pass" if bool(metrics["mission_pass"]) else "boundary"
            failure: dict[str, str] | None = None
        except (RuntimeError, ValueError) as error:
            if not boundary:
                raise
            metrics = {
                "mission_pass": False,
                "uncontrolled_wrench_axes": ["moment_z_nm"],
                "allocation_statuses": ["source_domain_boundary"],
                "samples": 0,
            }
            record_status = "boundary"
            failure = {"code": "DATA_DOMAIN_VIOLATION", "message": str(error)}
        records.append(
            {
                "case_id": case_id,
                "description": case["description"],
                "status": record_status,
                "boundary_witness": boundary,
                "metrics": metrics,
                "failure": failure,
            }
        )
    passed = sum(bool(record["metrics"]["mission_pass"]) for record in records)
    boundary_failures = sum(bool(record["boundary_witness"]) and not bool(record["metrics"]["mission_pass"]) for record in records)
    report: dict[str, object] = {
        "schema": "taoryx.x8-physical-r1-matrix/v1alpha1",
        "status": "R1_physical_source_coordinate_matrix_complete",
        "vehicle": "skywalker_x8",
        "claim": "Fixed local X8 source-coordinate physical-effector evidence around the source-trim operating point.",
        "claim_boundary": "This matrix proves only local roll/pitch allocation through bounded source-coordinate elevon commands. It does not establish independent yaw authority, mapped hardware servo wiring, end-to-end racetrack completion, scheduled control, wind robustness, or family qualification.",
        "control_path": "attitude/rate error -> plant-derived roll/pitch LQR -> desired roll/pitch moments -> bounded collective/differential elevon allocation -> actuator lag/rate limits -> nonlinear source-table plant",
        "direct_body_moment_injection": False,
        "physical_mapping": x8_source_mapping().as_dict(),
        "effectors": list(EFFECTOR_NAMES),
        "uncontrolled_wrench_axes": ["moment_z_nm"],
        "source_trim": trim.as_dict(),
        "linearization": {
            "derivative_consistent": linearization.provenance.derivative_consistent,
            "state_names": list(linearization.primary.state_names),
            "control_names": list(linearization.primary.control_names),
            "effectiveness": [list(row) for row in effectiveness.matrix],
        },
        "selected_design": design.as_dict(),
        "case_count": len(records),
        "passed_case_count": passed,
        "failed_case_count": len(records) - passed,
        "boundary_failure_count": boundary_failures,
        "cases": records,
        "reproduction": "PYTHONPATH=src python3 tools/validate_x8_physical_r1_matrix.py",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run and write the X8 local physical R1 matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(build_matrix(arguments.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
