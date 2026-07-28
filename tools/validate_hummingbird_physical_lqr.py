#!/usr/bin/env python3
"""Build the local Hummingbird individual-rotor physical-LQR evidence artifact.

The proof evaluates the extracted RotorPy individual-rotor equations rather
than a direct body-moment bridge.  Each source rotor command passes through
the documented 0--1500 rad/s range and 5 ms first-order motor response before
the nonlinear rigid-body plant evaluates force and moment.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from taoryx.contracts import Vector3
from taoryx.control_allocation import EffectorLimits
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.physical_lqr import (
    design_physical_wrench_lqr,
    project_linearization_to_wrench,
    validate_nonlinear_wrench_lqr,
)
from taoryx.runtime.program import LoadedProgram
from taoryx.runtime_control_adapter import RuntimeRigidBodyLocalPlant, local_rigid_body_plant_from_vehicle

ROOT = Path(__file__).resolve().parents[1]
PROBLEM = ROOT / "examples/generated/vehicles/hummingbird_individual_rotor_hover_6dof.prb"
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
TABLES = tuple(
    TABLE_ROOT / name
    for name in (
        "hummingbird_cx.tbl",
        "hummingbird_cy.tbl",
        "hummingbird_cz.tbl",
        "hummingbird_cmx.tbl",
        "hummingbird_cmy.tbl",
        "hummingbird_cmz.tbl",
    )
)
SOURCE_ROOT = TABLE_ROOT.parent / "quadcopter_hummingbird"
SOURCE_ASSETS = (
    SOURCE_ROOT / "VALIDITY.md",
    SOURCE_ROOT / "scripts/wrench_evaluator.py",
    SOURCE_ROOT / "controls/control_allocation_matrix.csv",
    SOURCE_ROOT / "controls/differential_speed_response.csv",
    SOURCE_ROOT / "geometry_mass/parameters.csv",
    SOURCE_ROOT / "geometry_mass/rotor_positions.csv",
    SOURCE_ROOT / "propulsion/rotor_static_map.csv",
)


def build_plant() -> RuntimeRigidBodyLocalPlant:
    """Build the source-hover local plant with four physical motor effectors."""

    program = LoadedProgram.load(PROBLEM, TABLES, profile=GrammarProfile.TAORYX)
    limits = {
        f"rotor-{index}-speed": EffectorLimits(
            f"rotor-{index}-speed",
            0.0,
            1500.0,
            "rad/s",
            # RotorPy supplies a first-order motor time constant but not a
            # separate hard slew-rate bound.  Do not fabricate one.
            time_constant_s=0.005,
        )
        for index in range(1, 5)
    }
    return local_rigid_body_plant_from_vehicle(
        "hummingbird-individual-rotor-source-plant",
        "rotorpy-hover-local-v1",
        program.case().vehicles["1"],
        inertia_kg_m2=Vector3(0.00365, 0.00368, 0.00703),
        reference_length_m=0.34,
        effector_limits=limits,
        effectiveness_steps={name: 1.0 for name in limits},
        allocation_wrench_weights={
            "moment_x_nm": 1.0,
            "moment_y_nm": 1.0,
            "moment_z_nm": 1.0,
        },
        # Rotor-speed effectiveness is on the order of 1e-4 N m/(rad/s).
        # Retain a trim preference only to resolve the one redundant rotor
        # direction; the regularizer must not turn an otherwise exact local
        # wrench request into a false infeasibility report.
        allocation_regularization=1.0e-16,
        allocation_feasibility_tolerance=1.0e-6,
    )
    ####


def build_artifact() -> dict[str, Any]:
    """Build one source-trim, derivative, allocation, and nonlinear packet."""

    plant = build_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"source individual-rotor trim did not converge: {trim.as_dict()}")
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
        state_names=(
            "roll_error_rad",
            "pitch_error_rad",
            "yaw_error_rad",
            "p_rad_s",
            "q_rad_s",
            "r_rad_s",
        ),
        wrench_names=("moment_x_nm", "moment_y_nm", "moment_z_nm"),
        effector_names=tuple(plant.control_names),
    )
    design = design_physical_wrench_lqr(
        "hummingbird-source-hover-attitude-rate-wrench-lqr-v1",
        projection,
        q_diagonal=(20.0, 20.0, 10.0, 4.0, 4.0, 2.0),
        r_diagonal=(1.0, 1.0, 1.0),
        state_scales=(
            math.radians(10.0),
            math.radians(10.0),
            math.radians(15.0),
            math.radians(60.0),
            math.radians(60.0),
            math.radians(60.0),
        ),
        wrench_scales=(0.10, 0.10, 0.05),
    )
    initial_state = dict(trim.state)
    initial_state.update(
        {
            "roll_error_rad": math.radians(5.0),
            "pitch_error_rad": math.radians(-3.0),
            "yaw_error_rad": math.radians(6.0),
            "p_rad_s": math.radians(8.0),
            "q_rad_s": math.radians(-6.0),
            "r_rad_s": math.radians(8.0),
        }
    )
    validation = validate_nonlinear_wrench_lqr(
        plant,
        trim,
        design,
        initial_state=initial_state,
        duration_s=4.0,
        dt_s=0.002,
    )
    disallowed_statuses = {"infeasible", "numerically_singular", "solver_failure"}
    acceptance: dict[str, Any] = {
        "derivative_maximum_relative_difference": 0.10,
        "final_feedback_error_fraction_of_initial": 0.05,
        "final_controlled_actual_residual_nm": 0.002,
        "maximum_saturation_fraction": 0.10,
        "maximum_continuous_saturation_duration_s": 0.10,
        "disallowed_allocation_statuses": sorted(disallowed_statuses),
    }
    final_error_fraction = validation.final_feedback_error_norm / max(validation.initial_feedback_error_norm, 1.0e-12)
    completed = (
        linearization.provenance.derivative_consistent
        and final_error_fraction <= acceptance["final_feedback_error_fraction_of_initial"]
        and validation.final_controlled_actual_residual <= acceptance["final_controlled_actual_residual_nm"]
        and validation.saturation_fraction <= acceptance["maximum_saturation_fraction"]
        and validation.maximum_continuous_saturation_duration_s <= acceptance["maximum_continuous_saturation_duration_s"]
        and not (set(validation.allocation_statuses) & disallowed_statuses)
        and all(math.isfinite(value) for value in validation.final_state.values())
    )
    return {
        "schema": "taoryx.hummingbird-individual-rotor-physical-lqr/v1alpha1",
        "vehicle": "hummingbird",
        "operating_condition": {
            "source_problem": str(PROBLEM.relative_to(ROOT)),
            "tables": [str(path.relative_to(ROOT)) for path in TABLES],
            "fidelity": "rigid_body_6dof",
            "physical_control_coordinate": "four source RotorPy motor-speed commands",
            "force_model": "individual-rotor-source-equations",
        },
        "provenance": {
            "assets": [
                {"path": str(path.relative_to(ROOT)), "sha256": _sha256(path)}
                for path in (PROBLEM, *TABLES, *SOURCE_ASSETS)
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
        "claim": {
            "status": "local_nonlinear_individual_rotor_validation" if completed else "local_validation_failed",
            "proves": (
                "At the declared RotorPy-derived hover trim, all four physical motor-speed commands are used to trim, "
                "linearize, allocate requested roll/pitch/yaw moments, advance through documented motor lag, and "
                "recover the nonlinear rigid-body attitude/rate plant without direct body-moment injection."
            ),
            "nonclaims": [
                "This is one local hover operating point, not a translated-flight, waypoint, landing-contact, or envelope-wide controller qualification.",
                "No battery voltage, current, discharge, temperature, or mass-change model is represented.",
                "The source equations include rigid rotor thrust/drag and motor lag but are not blade-resolved, dynamic-inflow, or drivetrain models.",
            ],
            "earned_controller_evidence_tier": "T5_nonlinearly_validated" if completed else "T3_linearly_controlled",
            "physical_allocation_evidence": "individual_rotor_source_equations_with_motor_lag",
            "direct_body_moment_injection": False,
        },
        "acceptance": {
            **acceptance,
            "observed_final_feedback_error_fraction": final_error_fraction,
        },
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
        "reproduction": "PYTHONPATH=src python3 tools/validate_hummingbird_physical_lqr.py",
    }
    ####


def _sha256(path: Path) -> str:
    """Return the immutable SHA-256 of one declared source asset."""

    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def main() -> int:
    """Write the deterministic local physical-control evidence packet."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "verification/generated/hummingbird_individual_rotor_physical_lqr.json",
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
