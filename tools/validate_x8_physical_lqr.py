#!/usr/bin/env python3
"""Build the local X8 table-coordinate physical-LQR evidence artifact.

This is intentionally a single source-trim operating-point proof.  It does
not claim that the source collective/differential table coordinates have been
resolved into hardware left/right elevon signs; see the artifact nonclaim.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from taoryx.airbreathing_control_mapping import x8_mapping_hypotheses, x8_source_mapping
from taoryx.generic_tuning import LinearAuthorityRequirement, linear_authority_preflight
from taoryx.physical_lqr import validate_nonlinear_wrench_lqr
from taoryx.runtime_control_adapter import RuntimeRigidBodyLocalPlant
from taoryx.source_table_fixed_wing import build_x8_source_surface_physical_lqr_design, build_x8_source_table_plant

ROOT = Path(__file__).resolve().parents[1]
PROBLEM = ROOT / "examples/generated/vehicles/skywalker_x8_table_coordinate_trim_6dof.prb"
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
TABLES = tuple(
    TABLE_ROOT / name
    for name in (
        "skywalker_x8_static_6axis.tbl",
        "skywalker_x8_collective_elevon_6axis.tbl",
        "skywalker_x8_differential_elevon_6axis.tbl",
        "skywalker_x8_thrust.tbl",
    )
)
ACTUATOR_LIMITS = TABLE_ROOT.parent / "cruise_class_uav_skywalker_x8/controls/actuator_limits.csv"
CONTROL_MAPPING_STATUS = TABLE_ROOT.parent / "cruise_class_uav_skywalker_x8/controls/control_mapping_status.csv"


def build_plant() -> RuntimeRigidBodyLocalPlant:
    """Build the runtime-owned pinned source-table local plant."""

    return build_x8_source_table_plant()
    ####


def build_artifact() -> dict[str, Any]:
    """Build one deterministic trim, derivative, allocation, and recovery packet."""

    plant = build_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"source table-coordinate trim did not converge: {trim.as_dict()}")
    design = build_x8_source_surface_physical_lqr_design()
    projection = design.projection
    linearization = projection.source_linearization
    effectiveness = plant.effectiveness(trim.state, trim.controls)
    authority_preflight = linear_authority_preflight(
        LinearAuthorityRequirement(
            "x8-roll-pitch-source-coordinate-authority",
            ("roll_error_rad", "pitch_error_rad", "p_rad_s", "q_rad_s"),
        ),
        state_names=projection.state_names,
        a_matrix=projection.a_matrix,
        b_matrix=projection.b_matrix,
    )
    if authority_preflight.status != "passed":
        raise RuntimeError(
            "X8 source-coordinate authority preflight blocked roll/pitch LQR synthesis: "
            + json.dumps(authority_preflight.as_dict(), sort_keys=True)
        )
    initial_state = dict(trim.state)
    initial_state.update(
        {
            "roll_error_rad": math.radians(5.0),
            "pitch_error_rad": math.radians(-3.0),
            "p_rad_s": math.radians(4.0),
            "q_rad_s": math.radians(-3.0),
        }
    )
    validation = validate_nonlinear_wrench_lqr(
        plant,
        trim,
        design,
        initial_state=initial_state,
        duration_s=8.0,
        dt_s=0.01,
    )
    acceptance: dict[str, Any] = {
        "derivative_maximum_relative_difference": 0.10,
        "final_feedback_error_fraction_of_initial": 0.05,
        "final_controlled_actual_residual_nm": 0.005,
        "maximum_saturation_fraction": 0.05,
        "maximum_continuous_saturation_duration_s": 0.25,
        "disallowed_allocation_statuses": ["infeasible", "numerically_singular", "solver_failure"],
    }
    final_error_fraction = validation.final_feedback_error_norm / max(validation.initial_feedback_error_norm, 1.0e-12)
    disallowed_statuses = set(acceptance["disallowed_allocation_statuses"])
    completed = (
        linearization.provenance.derivative_consistent
        and final_error_fraction <= acceptance["final_feedback_error_fraction_of_initial"]
        and validation.final_controlled_actual_residual <= acceptance["final_controlled_actual_residual_nm"]
        and validation.saturation_fraction <= acceptance["maximum_saturation_fraction"]
        and validation.maximum_continuous_saturation_duration_s <= acceptance["maximum_continuous_saturation_duration_s"]
        and not (set(validation.allocation_statuses) & disallowed_statuses)
        and all(
            math.isfinite(value)
            for value in validation.final_state.values()
        )
    )
    return {
        "schema": "taoryx.x8-table-coordinate-physical-lqr/v1alpha1",
        "vehicle": "skywalker_x8",
        "operating_condition": {
            "source_problem": str(PROBLEM.relative_to(ROOT)),
            "tables": [str(path.relative_to(ROOT)) for path in TABLES],
            "fidelity": "rigid_body_6dof",
            "physical_control_coordinate": "source-table collective/differential elevon coordinates",
        },
        "provenance": {
            "assets": [
                {"path": str(path.relative_to(ROOT)), "sha256": _sha256(path)}
                for path in (PROBLEM, *TABLES, ACTUATOR_LIMITS, CONTROL_MAPPING_STATUS)
            ],
        },
        "actuator_contract": {
            name: {
                "lower": limits.lower,
                "upper": limits.upper,
                "unit": limits.unit,
                "rate_limit_per_s": limits.rate_limit_per_s,
                "time_constant_s": limits.time_constant_s,
                "available": limits.available,
            }
            for name, limits in plant.effector_limits.items()
        },
        "physical_mapping": {
            "status": "resolved_by_source_equation",
            "source_status": "verify_before_use_original_package_note",
            "selected_hypothesis": x8_source_mapping().as_dict(),
            "hypotheses": [mapping.as_dict() for mapping in x8_mapping_hypotheses()],
            "source_equation": "[delta_e, delta_a]^T = 1/2 [[1, 1], [-1, 1]] [delta_er, delta_el]^T",
            "source_url": "https://doi.org/10.1007/s13272-025-00816-3",
            "claim_boundary": "The source paper resolves the collective/differential-to-left/right sign mapping; servo wiring, hinge convention, individual actuator telemetry, and end-to-end mapped-surface validation remain separate gates.",
        },
        "claim": {
            "status": "local_nonlinear_table_coordinate_validation" if completed else "local_validation_failed",
            "proves": (
                "At the declared source-trim point, a nonlinear table-backed X8 local plant is trimmed, "
                "linearized, controlled through roll/pitch wrench demands, allocated through bounded source-table "
                "elevon coordinates with declared lag/rate limits, and evaluated again by the nonlinear plant."
            ),
            "nonclaims": [
                "The source paper resolves the mathematical left/right mapping, but this artifact does not prove servo wiring, hinge sign, or hardware actuator dynamics.",
                "This is one local operating point, not a gain-scheduled or envelope-wide controller qualification.",
                "Yaw is declared unallocated because the flying-wing table coordinate pair does not provide independent yaw-moment authority.",
                "The current short case has fixed mass and does not qualify battery discharge or endurance.",
            ],
            "earned_controller_evidence_tier": "T4_physically_allocated_source_coordinate",
            "nonlinear_table_coordinate_evidence": "passed" if completed else "failed",
            "promotion_blocker": "individual_left_right_actuator_and_end_to_end_racetrack_evidence_pending",
            "physical_allocation_evidence": "source_coordinate_allocation_with_resolved_left_right_mapping",
            "direct_body_moment_injection": False,
        },
        "acceptance": {
            **acceptance,
            "observed_final_feedback_error_fraction": final_error_fraction,
        },
        "authority_preflight": authority_preflight.as_dict(),
        "trim": trim.as_dict(),
        "linearization": {
            "state_names": list(linearization.primary.state_names),
            "control_names": list(linearization.primary.control_names),
            "a_matrix": linearization.primary.a_matrix.tolist(),
            "b_matrix": linearization.primary.b_matrix.tolist(),
            "comparison_a_matrix": linearization.comparison.a_matrix.tolist(),
            "comparison_b_matrix": linearization.comparison.b_matrix.tolist(),
            "provenance": {
                "nonlinear_plant_id": linearization.provenance.nonlinear_plant_id,
                "nonlinear_plant_revision": linearization.provenance.nonlinear_plant_revision,
                "method": linearization.provenance.method,
                "state_step": linearization.provenance.state_step,
                "control_step": linearization.provenance.control_step,
                "comparison_state_step": linearization.provenance.comparison_state_step,
                "comparison_control_step": linearization.provenance.comparison_control_step,
                "maximum_relative_difference": linearization.provenance.maximum_relative_difference,
                "maximum_absolute_difference": linearization.provenance.maximum_absolute_difference,
                "comparison_absolute_floor": linearization.provenance.comparison_absolute_floor,
                "derivative_consistent": linearization.provenance.derivative_consistent,
                "state_units": dict(linearization.provenance.state_units),
                "control_units": dict(linearization.provenance.control_units),
            },
        },
        "effectiveness": {
            "wrench_names": list(effectiveness.wrench_names),
            "effector_names": list(effectiveness.effector_names),
            "matrix": [list(row) for row in effectiveness.matrix],
            "reference_wrench": dict(effectiveness.reference_wrench),
            "reference_effectors": dict(effectiveness.reference_effectors),
            "source": effectiveness.source,
        },
        "nonlinear_validation": validation.as_dict(),
        "reproduction": "PYTHONPATH=src python3 tools/validate_x8_physical_lqr.py",
    }
    ####


def _sha256(path: Path) -> str:
    """Return the immutable SHA-256 of one declared source asset."""

    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def main() -> int:
    """Write the artifact, preserving a deterministic JSON representation."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "verification/generated/x8_table_coordinate_physical_lqr.json",
    )
    arguments = parser.parse_args()
    artifact = build_artifact()
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(arguments.output)
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
