#!/usr/bin/env python3
"""Run a bounded B747 physical-surface controller R1 matrix.

The matrix keeps the NASA CR-2144 condition-3 source trim and controller
design fixed, then exercises the same nonlinear plant through actual elevator,
aileron, rudder, and throttle allocation.  Interior speed witnesses and a
deliberately near-authority coupled-rate case are retained separately so a
boundary failure cannot be mistaken for a nominal pass.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, cast

from taoryx.physical_lqr import project_linearization_to_wrench, validate_nonlinear_wrench_lqr

try:
    from validate_b747_physical_lqr import (
        _STATE_NAMES,
        _SURFACE_NAMES,
        _WRENCH_NAMES,
        _design_candidates,
        _physical_wrench_scales,
        build_plant,
    )
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.validate_b747_physical_lqr import (
        _STATE_NAMES,
        _SURFACE_NAMES,
        _WRENCH_NAMES,
        _design_candidates,
        _physical_wrench_scales,
        build_plant,
    )

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_b747_physical_r1"
DISALLOWED_STATUSES = {"infeasible", "numerically_singular", "solver_failure"}
CASES: dict[str, dict[str, object]] = {
    "nominal_local": {
        "description": "declared condition-3 local coupled attitude/rate witness",
        "state_update": {
            "roll_error_rad": math.radians(1.0),
            "pitch_error_rad": math.radians(-0.5),
            "yaw_error_rad": math.radians(1.0),
            "p_rad_s": math.radians(0.5),
            "q_rad_s": math.radians(-0.5),
            "r_rad_s": math.radians(0.5),
        },
    },
    "speed_minus_5m_s": {
        "description": "five metre-per-second below the source-trim forward speed",
        "u_delta_m_s": -5.0,
    },
    "speed_plus_5m_s": {
        "description": "five metre-per-second above the source-trim forward speed",
        "u_delta_m_s": 5.0,
    },
    "coupled_rate_authority_boundary": {
        "description": "larger coupled attitude/rate demand retained as a surface-authority boundary",
        "state_update": {
            "roll_error_rad": math.radians(2.0),
            "pitch_error_rad": math.radians(-1.0),
            "yaw_error_rad": math.radians(2.0),
            "p_rad_s": math.radians(1.0),
            "q_rad_s": math.radians(-1.0),
            "r_rad_s": math.radians(1.0),
        },
        "expected_boundary": True,
    },
}


def _acceptance() -> dict[str, object]:
    """Return the fixed local physical-controller acceptance contract."""

    return {
        "final_normalized_feedback_error_fraction": 0.10,
        "maximum_saturation_fraction": 0.05,
        "maximum_continuous_saturation_duration_s": 0.50,
        "disallowed_allocation_statuses": sorted(DISALLOWED_STATUSES),
        "duration_s": 80.0,
        "dt_s": 0.05,
    }
    ####


def _summary(validation: Any, acceptance: dict[str, object]) -> dict[str, object]:
    """Return numeric metrics and the independent pass calculation."""

    ratio = validation.final_normalized_feedback_error_norm / max(
        validation.initial_normalized_feedback_error_norm,
        1.0e-12,
    )
    statuses = set(validation.allocation_statuses)
    passed = (
        ratio <= float(cast(float, acceptance["final_normalized_feedback_error_fraction"]))
        and validation.saturation_fraction <= float(cast(float, acceptance["maximum_saturation_fraction"]))
        and validation.maximum_continuous_saturation_duration_s
        <= float(cast(float, acceptance["maximum_continuous_saturation_duration_s"]))
        and not statuses.intersection(DISALLOWED_STATUSES)
    )
    return {
        "mission_pass": passed,
        "final_normalized_feedback_error_fraction": ratio,
        "final_controlled_actual_residual_nm": validation.final_controlled_actual_residual,
        "saturation_fraction": validation.saturation_fraction,
        "maximum_continuous_saturation_duration_s": validation.maximum_continuous_saturation_duration_s,
        "allocation_statuses": list(validation.allocation_statuses),
        "samples": len(validation.samples),
    }
    ####


def build_matrix(output: Path) -> dict[str, object]:
    """Build the fixed B747 local physical-effector matrix."""

    plant = build_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"B747 physical R1 trim did not converge: {trim.as_dict()}")
    linearization = plant.linearize(
        trim,
        {
            "state_step": 1.0e-4,
            "control_step": 1.0e-3,
            "comparison_factor": 0.5,
            "maximum_relative_difference": 0.10,
            "comparison_absolute_floor": 1.0e-7,
        },
    )
    effectiveness = plant.effectiveness(trim.state, trim.controls)
    projection = project_linearization_to_wrench(
        linearization,
        effectiveness,
        state_names=_STATE_NAMES,
        wrench_names=_WRENCH_NAMES,
        effector_names=_SURFACE_NAMES,
    )
    candidates = _design_candidates(projection, _physical_wrench_scales(effectiveness, plant))
    acceptance = _acceptance()
    nominal_state = dict(trim.state)
    nominal_state.update(
        {
            str(name): float(cast(float, value))
            for name, value in cast(dict[str, object], CASES["nominal_local"]["state_update"]).items()
        }
    )
    selected_design = None
    nominal_validation = None
    for design, tuning in candidates:
        validation = validate_nonlinear_wrench_lqr(
            plant,
            trim,
            design,
            initial_state=nominal_state,
            duration_s=float(cast(float, acceptance["duration_s"])),
            dt_s=float(cast(float, acceptance["dt_s"])),
        )
        metrics = _summary(validation, acceptance)
        if bool(metrics["mission_pass"]):
            selected_design = (design, tuning)
            nominal_validation = validation
            break
    if selected_design is None or nominal_validation is None:
        raise RuntimeError("no generic B747 physical-surface design passed the nominal local gate")

    design, tuning = selected_design
    records: list[dict[str, object]] = []
    for case_id, case in CASES.items():
        state = dict(trim.state)
        update = case.get("state_update")
        if isinstance(update, dict):
            state.update({str(name): float(cast(float, value)) for name, value in update.items()})
        if "u_delta_m_s" in case:
            state["u_m_s"] += float(cast(float, case["u_delta_m_s"]))
        validation = nominal_validation if case_id == "nominal_local" else validate_nonlinear_wrench_lqr(
            plant,
            trim,
            design,
            initial_state=state,
            duration_s=float(cast(float, acceptance["duration_s"])),
            dt_s=float(cast(float, acceptance["dt_s"])),
        )
        metrics = _summary(validation, acceptance)
        expected_boundary = bool(case.get("expected_boundary", False))
        if expected_boundary and bool(metrics["mission_pass"]):
            raise RuntimeError(f"expected B747 authority boundary case {case_id!r} to remain bounded evidence")
        case_dir = output / case_id
        case_dir.mkdir(parents=True, exist_ok=True)
        (case_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        records.append(
            {
                "case_id": case_id,
                "description": case["description"],
                "mission_pass": bool(metrics["mission_pass"]),
                "boundary_witness": expected_boundary,
                "metrics": metrics,
                "artifact": str((case_dir / "metrics.json").relative_to(ROOT)),
            }
        )
    passed = sum(bool(record["mission_pass"]) for record in records)
    boundary_failures = sum(bool(record["boundary_witness"]) and not bool(record["mission_pass"]) for record in records)
    report: dict[str, object] = {
        "schema": "taoryx.b747-physical-surface-r1-matrix/v1alpha1",
        "status": "R1_physical_surface_matrix_complete",
        "vehicle": "b747",
        "claim": "Fixed local B747 surface-allocation evidence around the NASA CR-2144 condition-3 trim.",
        "claim_boundary": "This is local physical-surface evidence at one source condition. The matrix does not establish gain scheduling, full-envelope transport control, source servo dynamics, fuel-flow/engine-spool dynamics, wind robustness, statistical reliability, or runway operations.",
        "control_path": "attitude/rate error -> generic plant-derived LQR -> desired body moments -> bounded elevator/aileron/rudder allocation -> nonlinear table-backed plant",
        "direct_body_moment_injection": False,
        "source_trim": trim.as_dict(),
        "selected_design": {"id": design.id, "tuning": tuning},
        "operating_condition": "NASA CR-2144 condition 3 / Mach 0.45 local neighborhood",
        "effectors": list(plant.control_names),
        "acceptance": acceptance,
        "case_count": len(records),
        "passed_case_count": passed,
        "failed_case_count": len(records) - passed,
        "boundary_failure_count": boundary_failures,
        "cases": records,
        "reproduction": "PYTHONPATH=src python3 tools/validate_b747_physical_r1_matrix.py",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run and write the B747 physical-surface R1 matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(build_matrix(arguments.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
