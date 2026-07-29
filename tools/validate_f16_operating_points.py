"""Resolve and validate the F-16 S-119 local operating-point catalog."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.control_allocation import EffectorLimits
from taoryx.physical_lqr import (
    design_physical_wrench_lqr,
    project_linearization_to_wrench,
    validate_nonlinear_wrench_lqr,
)
from taoryx.trajectory import (
    F16ReferencePhysicalPlant,
    load_f16_reference_plant,
    runtime_trim_result,
    solve_f16_source_trim,
)

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "families/reference_f16_s119/qualification/operating-points.yaml"
SIDECAR = ROOT / "families/reference_f16_s119/plant/daveml-import.json"
ATMOSPHERE = ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml"
ACTUATORS = ROOT / "families/reference_f16_s119/actuators/reference-first-order-v1.yaml"
CONTROLLER = ROOT / "families/reference_f16_s119/controllers/local-physical-wrench-lqr-v1.yaml"


def _limits(payload: dict[str, Any]) -> dict[str, EffectorLimits]:
    """Resolve the explicit development actuator contract."""

    names = {
        "elevator": "elevator_deg",
        "aileron": "aileron_deg",
        "rudder": "rudder_deg",
        "throttle": "throttle_fraction",
    }
    return {
        names[source_name]: EffectorLimits(
            names[source_name],
            float(values["lower"]),
            float(values["upper"]),
            str(values["unit"]),
            float(values["rate_limit_per_s"]),
            float(values["time_constant_s"]),
        )
        for source_name, values in payload["limits"].items()
    }
    ####


def _record(
    point: dict[str, Any],
    source: Any,
    actuator_limits: dict[str, EffectorLimits],
    controller_profile: dict[str, Any],
) -> dict[str, Any]:
    """Resolve one catalog entry and retain local derivative/authority evidence."""

    environment = point["environment"]
    state = point["state"]
    controls = point["controls"]
    resolved = solve_f16_source_trim(
        source,
        point_id=str(point["id"]),
        altitude_m=float(environment["geometric_altitude_m"]),
        true_airspeed_m_s=float(environment["true_airspeed_m_s"]),
        initial_alpha_deg=float(state["alpha_deg"]),
        initial_elevator_deg=float(controls["elevator_deg"]),
        initial_throttle_fraction=float(controls["throttle_fraction"]),
    )
    runtime_trim = runtime_trim_result(resolved)
    linearization = source.linearize_local(
        resolved.body_state,
        resolved.controls,
        trim_pitch_rad=resolved.trim_pitch_rad,
        altitude_m=resolved.altitude_m,
        state_step=1.0e-5,
        control_step=1.0e-5,
    )
    adapter = F16ReferencePhysicalPlant(
        source,
        runtime_trim,
        resolved.trim_pitch_rad,
        resolved.altitude_m,
        actuator_limits,
    )
    effectiveness = adapter.effectiveness(resolved.body_state, resolved.controls)
    allocation = adapter.allocate(resolved.body_state, effectiveness.reference_wrench, resolved.controls, 0.02)
    projection = project_linearization_to_wrench(
        linearization,
        effectiveness,
        state_names=tuple(controller_profile["state_names"]),
        wrench_names=tuple(controller_profile["wrench_names"]),
        effector_names=tuple(controller_profile["effector_names"]),
    )
    design = design_physical_wrench_lqr(
        f"{controller_profile['controller_id']}:{resolved.point_id}",
        projection,
        q_diagonal=controller_profile["q_diagonal"],
        r_diagonal=controller_profile["r_diagonal"],
        state_scales=controller_profile["state_scales"],
        wrench_scales=controller_profile["wrench_scales"],
    )
    lqr_validation = validate_nonlinear_wrench_lqr(
        adapter,
        runtime_trim,
        design,
        initial_state=resolved.body_state,
        duration_s=1.0,
        dt_s=0.02,
    )
    return {
        **resolved.as_dict(),
        "linearization": {
            "state_names": list(linearization.primary.state_names),
            "control_names": list(linearization.primary.control_names),
            "state_shape": list(linearization.primary.a_matrix.shape),
            "control_shape": list(linearization.primary.b_matrix.shape),
            "derivative_consistent": linearization.provenance.derivative_consistent,
            "maximum_relative_difference": linearization.provenance.maximum_relative_difference,
            "maximum_absolute_difference": linearization.provenance.maximum_absolute_difference,
            "state_step": linearization.provenance.state_step,
            "control_step": linearization.provenance.control_step,
            "comparison_state_step": linearization.provenance.comparison_state_step,
            "comparison_control_step": linearization.provenance.comparison_control_step,
            "plant_id": linearization.provenance.nonlinear_plant_id,
            "plant_revision": linearization.provenance.nonlinear_plant_revision,
        },
        "physical_effectiveness": {
            "wrench_names": list(effectiveness.wrench_names),
            "effector_names": list(effectiveness.effector_names),
            "rank": int(np.linalg.matrix_rank(effectiveness.array)),
            "shape": list(effectiveness.array.shape),
            "source": effectiveness.source,
            "trim_allocation_status": allocation.allocation.status,
            "trim_allocation_residual_norm": allocation.achieved_controlled_residual_norm,
            "trim_allocation_saturated": sorted(
                set(allocation.allocation.position_saturated)
                | set(allocation.allocation.rate_limited)
                | set(allocation.actuator.position_saturated)
                | set(allocation.actuator.rate_limited)
            ),
        },
        "local_lqr": {
            "controller_id": design.id,
            "controllable": design.result.controllable,
            "hurwitz": design.result.hurwitz,
            "maximum_real_pole": design.result.maximum_real_pole,
            "condition_number": design.result.condition_number,
            "closed_loop_poles": [
                {"real": float(value.real), "imaginary": float(value.imag)}
                for value in design.result.closed_loop_eigenvalues
            ],
            "trim_hold_validation": {
                "duration_s": lqr_validation.duration_s,
                "dt_s": lqr_validation.dt_s,
                "allocation_statuses": list(lqr_validation.allocation_statuses),
                "saturation_fraction": lqr_validation.saturation_fraction,
                "maximum_controlled_actual_residual": lqr_validation.maximum_controlled_actual_residual,
                "final_normalized_feedback_error_norm": lqr_validation.final_normalized_feedback_error_norm,
            },
        },
    }
    ####


def main() -> int:
    """Generate the reproducible multi-point local evidence artifact."""

    catalog = yaml.safe_load(CATALOG.read_text(encoding="utf-8"))
    actuator_profile = yaml.safe_load(ACTUATORS.read_text(encoding="utf-8"))
    controller_profile = yaml.safe_load(CONTROLLER.read_text(encoding="utf-8"))
    source = load_f16_reference_plant(SIDECAR, ATMOSPHERE)
    records = [_record(point, source, _limits(actuator_profile), controller_profile) for point in catalog["points"]]
    maximum_trim_residual = 3.0e-4
    passed = all(
        record["max_residual"] <= maximum_trim_residual
        and record["linearization"]["derivative_consistent"]
        and record["physical_effectiveness"]["rank"] == 4
        and record["physical_effectiveness"]["trim_allocation_status"] in {"feasible", "feasible_near_limit"}
        and record["local_lqr"]["controllable"]
        and record["local_lqr"]["hurwitz"]
        and record["local_lqr"]["trim_hold_validation"]["saturation_fraction"] == 0.0
        for record in records
    )
    report = {
        "schema_version": "taoryx.f16-operating-points-evidence/v1",
        "status": "development_schedule_witness_passed" if passed else "development_schedule_witness_failed",
        "family_id": "reference_f16_s119",
        "source_plant": "reference-f16-s119-source-runtime-plant",
        "point_count": len(records),
        "acceptance": {
            "maximum_absolute_trim_residual": maximum_trim_residual,
            "note": "The bound is in mixed force/moment residual units. The high-altitude source-table witnesses retain their raw residuals and scaled solver norms per point; the gate is not silently replaced by the scaled norm.",
        },
        "points": records,
        "claim": (
            "Seven source-retrimmed fixed-altitude, fixed-airspeed local operating points with plant-derived "
            "linearization and local physical-effector authority checks."
        ),
        "claim_boundary": (
            "This is local operating-point and schedule-readiness evidence. It does not claim a validated "
            "gain schedule, continuous transitions, wind robustness, or full-envelope F-16 flight control."
        ),
    }
    output = ROOT / "verification/f16_operating_points_evidence.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if passed else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
