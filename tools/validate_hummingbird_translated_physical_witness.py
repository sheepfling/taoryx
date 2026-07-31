#!/usr/bin/env python3
"""Validate Hummingbird physical rotor authority with translated body velocity.

The local source plant already contains a RotorPy-derived body-velocity load
grid. This witness exercises those loads while the individual-rotor attitude
controller allocates physical motor-speed commands. It is deliberately not a
waypoint or position-control qualification: no translational guidance loop is
introduced by this artifact.
"""

from __future__ import annotations

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
DEFAULT_OUTPUT = ROOT / "verification/alpha3_hummingbird_translated_physical"
STATE_NAMES = ("roll_error_rad", "pitch_error_rad", "yaw_error_rad", "p_rad_s", "q_rad_s", "r_rad_s")
WRENCH_NAMES = ("moment_x_nm", "moment_y_nm", "moment_z_nm")
CASES: dict[str, dict[str, float]] = {
    "forward_body_velocity_5mps": {"u_m_s": 5.0},
    "rearward_body_velocity_5mps": {"u_m_s": -5.0},
    "lateral_body_velocity_3mps": {"v_m_s": 3.0},
    "combined_body_velocity": {"u_m_s": 5.0, "v_m_s": 3.0},
    "vertical_body_velocity_2mps": {"w_m_s": -2.0},
}


def _build_design(plant: Any, trim: Any) -> Any:
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
        "hummingbird-source-hover-translated-body-wrench-lqr-v1",
        projection,
        q_diagonal=(20.0, 20.0, 10.0, 4.0, 4.0, 2.0),
        r_diagonal=(1.0, 1.0, 1.0),
        state_scales=(math.radians(10.0), math.radians(10.0), math.radians(15.0), math.radians(60.0), math.radians(60.0), math.radians(60.0)),
        wrench_scales=(0.10, 0.10, 0.05),
    )
    return design
    ####


def _run_case(plant: Any, trim: Any, design: Any, velocity_update: dict[str, float]) -> dict[str, object]:
    state = dict(trim.state)
    state.update(
        {
            "roll_error_rad": math.radians(5.0),
            "pitch_error_rad": math.radians(-3.0),
            "yaw_error_rad": math.radians(4.0),
            "p_rad_s": math.radians(3.0),
            "q_rad_s": math.radians(-2.0),
            "r_rad_s": math.radians(2.0),
            **velocity_update,
        }
    )
    validation = validate_nonlinear_wrench_lqr(
        plant,
        trim,
        design,
        initial_state=state,
        duration_s=2.0,
        dt_s=0.002,
    )
    ratio = validation.final_feedback_error_norm / max(validation.initial_feedback_error_norm, 1.0e-12)
    disallowed = {"infeasible", "numerically_singular", "solver_failure"}
    passed = ratio <= 0.05 and validation.saturation_fraction <= 0.10 and not disallowed.intersection(validation.allocation_statuses)
    return {
        "body_velocity_update_m_s": velocity_update,
        "mission_pass": passed,
        "final_feedback_error_fraction": ratio,
        "final_controlled_actual_residual_nm": validation.final_controlled_actual_residual,
        "maximum_controlled_actual_residual_nm": validation.maximum_controlled_actual_residual,
        "saturation_fraction": validation.saturation_fraction,
        "allocation_statuses": list(validation.allocation_statuses),
        "sample_count": len(validation.samples),
        "failure_reason": None if passed else "allocation_or_attitude_authority_boundary",
    }
    ####


def build_witness(output: Path = DEFAULT_OUTPUT) -> dict[str, object]:
    """Write translated-body physical rotor evidence."""

    output.mkdir(parents=True, exist_ok=True)
    plant = build_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"Hummingbird translated-body trim did not converge: {trim.as_dict()}")
    design = _build_design(plant, trim)
    records = [{"case_id": case_id, "metrics": _run_case(plant, trim, design, update)} for case_id, update in CASES.items()]
    passed_count = sum(bool(record["metrics"]["mission_pass"]) for record in records)  # type: ignore[index]
    report: dict[str, object] = {
        "schema": "taoryx.hummingbird-translated-physical-witness/v1alpha1",
        "status": "translated_body_physical_rotor_witness_pass" if passed_count == len(records) else "translated_body_physical_rotor_witness_boundary",
        "vehicle": "hummingbird",
        "fidelity": "rigid_body_6dof",
        "control_path": "body-velocity source loads -> attitude/rate error -> plant-derived LQR -> desired body moments -> bounded four-rotor speed allocation -> motor lag -> nonlinear rigid-body plant",
        "source_validity_basis": {
            "source": "RotorPy-derived deterministic body-velocity wrench grid",
            "body_velocity_witness_range_m_s": {"u": [-10.0, 10.0], "v": [-10.0, 10.0], "w": [-5.0, 5.0]},
            "fixture": "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/quadcopter_hummingbird/VALIDITY.md",
        },
        "source_trim": trim.as_dict(),
        "controller_id": design.id,
        "records": records,
        "summary": {
            "case_count": len(records),
            "passed_case_count": passed_count,
            "failed_case_count": len(records) - passed_count,
            "all_allocation_statuses_feasible": all(set(record["metrics"]["allocation_statuses"]) == {"feasible"} for record in records),  # type: ignore[index]
        },
        "claims": [
            "the source individual-rotor load model remains numerically evaluable under declared translated body velocities",
            "the local physical rotor allocator remains feasible while regulating attitude/rate perturbations in those translated-load cases",
        ],
        "nonclaims": [
            "position or velocity guidance and waypoint completion",
            "translated-flight family qualification",
            "battery electrical model or endurance",
            "wind variation, blade-resolved aerodynamics, or statistical robustness",
        ],
        "reproduction": "PYTHONPATH=src python3 tools/validate_hummingbird_translated_physical_witness.py",
        "next_gate": "Connect the native rotor allocator to a translated position/velocity mission runner and compare its physical telemetry with the aggregate pseudo tier; retain battery and wind as separate boundaries.",
    }
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run the translated-body physical rotor witness."""

    print(json.dumps(build_witness(), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
