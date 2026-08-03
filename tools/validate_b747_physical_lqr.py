#!/usr/bin/env python3
"""Build a B747 condition-3 physical-surface LQR evidence artifact.

The proof begins at the NASA CR-2144 condition-3 source trim.  It solves the
same nonlinear table-backed plant for elevator, aileron, rudder, and throttle;
derives A/B from that plant; uses the generic profile tuner to choose a local
wrench LQR; and realizes every requested moment through bounded source-table
surface commands.  The source package does not supply servo-rate or lag data,
so the actuator contract is explicitly ideal rather than invented.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from taoryx.contracts import Vector3
from taoryx.control_allocation import EffectorEffectiveness, EffectorLimits
from taoryx.generic_tuning import (
    GenericLqrProfile,
    LinearAuthorityRequirement,
    linear_authority_preflight,
    tune_lqr_profiles,
)
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.physical_lqr import (
    PhysicalWrenchLqrDesign,
    WrenchLinearizationProjection,
    project_linearization_to_wrench,
    validate_nonlinear_wrench_lqr,
)
from taoryx.runtime.program import LoadedProgram
from taoryx.runtime_control_adapter import RuntimeRigidBodyLocalPlant, local_rigid_body_plant_from_vehicle

ROOT = Path(__file__).resolve().parents[1]
PROBLEM = ROOT / "examples/generated/vehicles/b747_condition3_surface_trim_6dof.prb"
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
TABLES = tuple(
    TABLE_ROOT / name
    for name in (
        "b747_nominal_static_6axis.tbl",
        "b747_nominal_elevator_6axis.tbl",
        "b747_nominal_aileron_6axis.tbl",
        "b747_nominal_rudder_6axis.tbl",
        "b747_jt9d_thrust.tbl",
    )
)
SOURCE_ROOT = TABLE_ROOT.parent / "jet_b747"
SOURCE_ASSETS = (
    SOURCE_ROOT / "aero/static_six_axis_grid.csv",
    SOURCE_ROOT / "aero/elevator_grid.csv",
    SOURCE_ROOT / "aero/aileron_grid.csv",
    SOURCE_ROOT / "aero/rudder_grid.csv",
    SOURCE_ROOT / "controls/actuator_limits.csv",
    SOURCE_ROOT / "propulsion/jt9d_installed_thrust_map.csv",
)

_STATE_NAMES = (
    "roll_error_rad",
    "pitch_error_rad",
    "yaw_error_rad",
    "u_m_s",
    "v_m_s",
    "w_m_s",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
)
_WRENCH_NAMES = ("moment_x_nm", "moment_y_nm", "moment_z_nm")
_SURFACE_NAMES = ("elevator-deg", "aileron-deg", "rudder-deg")


def build_plant() -> RuntimeRigidBodyLocalPlant:
    """Build the condition-3 source-table plant with actual controls."""

    program = LoadedProgram.load(PROBLEM, TABLES, profile=GrammarProfile.TAORYX)
    limits = {
        "elevator-deg": EffectorLimits("elevator-deg", -10.0, 10.0, "deg"),
        "aileron-deg": EffectorLimits("aileron-deg", -10.0, 10.0, "deg"),
        "rudder-deg": EffectorLimits("rudder-deg", -15.0, 15.0, "deg"),
        "throttle": EffectorLimits("throttle", 0.0, 1.0, "fraction"),
    }
    return local_rigid_body_plant_from_vehicle(
        "b747-condition3-source-table-plant",
        "nasa-cr-2144-condition3-local-v1",
        program.case().vehicles["1"],
        inertia_kg_m2=Vector3(24_675_886.7, 44_877_574.1, 67_384_152.0),
        reference_length_m=8.324088,
        effector_limits=limits,
        effectiveness_steps={
            "elevator-deg": 0.1,
            "aileron-deg": 0.1,
            "rudder-deg": 0.1,
            "throttle": 0.005,
        },
        allocation_wrench_weights={name: 1.0 for name in _WRENCH_NAMES},
        allocation_regularization=1.0e-14,
        allocation_feasibility_tolerance=1.0e-3,
    )
    ####


def _physical_wrench_scales(
    effectiveness: EffectorEffectiveness,
    plant: RuntimeRigidBodyLocalPlant,
) -> tuple[float, ...]:
    """Derive local controller scales from declared physical surface travel.

    The scale is deliberately a quarter of the available one-sided authority,
    leaving room for the allocation to expose a near-limit or infeasible
    request instead of normalizing the full surface range into routine use.
    """

    matrix = effectiveness.array
    result: list[float] = []
    for row, wrench_name in enumerate(_WRENCH_NAMES):
        authority = 0.0
        for column, effector_name in enumerate(effectiveness.effector_names):
            if effector_name not in _SURFACE_NAMES:
                continue
            limits = plant.effector_limits[effector_name]
            reference = float(effectiveness.reference_effectors[effector_name])
            positive = max(0.0, limits.upper - reference)
            negative = max(0.0, reference - limits.lower)
            authority += abs(float(matrix[row, column])) * min(positive, negative)
        if authority <= 0.0:
            raise ValueError(f"no declared local authority for {wrench_name!r}")
        result.append(max(authority * 0.25, 1.0))
    return tuple(result)
    ####


def _profiles() -> tuple[GenericLqrProfile, ...]:
    """Return reusable normalized slow-transport local tuning candidates."""

    return (
        GenericLqrProfile(
            "gentle",
            (0.25, 0.25, 0.25, 0.10, 0.25, 0.25, 1.0, 1.0, 1.0),
            (2.0, 2.0, 2.0),
        ),
        GenericLqrProfile(
            "standard",
            (1.0, 1.0, 1.0, 0.25, 1.0, 1.0, 1.0, 1.0, 1.0),
            (1.0, 1.0, 1.0),
        ),
        GenericLqrProfile(
            "aggressive",
            (4.0, 4.0, 4.0, 0.50, 4.0, 4.0, 1.0, 1.0, 1.0),
            (0.25, 0.25, 0.25),
        ),
    )
    ####


def _design_candidates(
    projection: WrenchLinearizationProjection,
    wrench_scales: tuple[float, ...],
) -> tuple[tuple[PhysicalWrenchLqrDesign, dict[str, Any]], ...]:
    """Synthesize profile candidates through the shared generic tuning seam."""

    state_scales = (
        math.radians(3.0),
        math.radians(3.0),
        math.radians(3.0),
        5.0,
        3.0,
        3.0,
        math.radians(9.0),
        math.radians(9.0),
        math.radians(9.0),
    )
    report = tune_lqr_profiles(
        "b747",
        projection.a_matrix,
        projection.b_matrix,
        state_names=_STATE_NAMES,
        control_names=_WRENCH_NAMES,
        state_scales=state_scales,
        control_scales=wrench_scales,
        profiles=_profiles(),
        design_source="b747-condition3-source-table-physical-wrench-projection",
    )
    candidates: list[tuple[PhysicalWrenchLqrDesign, dict[str, Any]]] = []
    for candidate in report.candidates:
        if candidate.lqr is None or candidate.lqr.maximum_real_pole >= 0.0:
            continue
        profile = candidate.weights
        design = PhysicalWrenchLqrDesign(
            f"b747-condition3-{candidate.profile_id}-surface-wrench-lqr-v1",
            projection,
            candidate.lqr,
            profile.q_diagonal,
            profile.r_diagonal,
            state_scales,
            wrench_scales,
        )
        candidates.append((design, candidate.as_dict()))
    if not candidates:
        raise RuntimeError("generic tuner did not produce a stable B747 physical-wrench candidate")
    return tuple(candidates)
    ####


def build_artifact() -> dict[str, Any]:
    """Build the source trim, tuning, allocation, and nonlinear evidence packet."""

    plant = build_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"B747 condition-3 physical trim did not converge: {trim.as_dict()}")
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
    authority_preflight = linear_authority_preflight(
        LinearAuthorityRequirement(
            "b747-condition3-three-axis-surface-authority",
            _STATE_NAMES,
        ),
        state_names=projection.state_names,
        a_matrix=projection.a_matrix,
        b_matrix=projection.b_matrix,
    )
    if authority_preflight.status != "passed":
        raise RuntimeError(
            "B747 physical-surface authority preflight blocked LQR synthesis: "
            + json.dumps(authority_preflight.as_dict(), sort_keys=True)
        )
    wrench_scales = _physical_wrench_scales(effectiveness, plant)
    initial_state = dict(trim.state)
    initial_state.update(
        {
            "roll_error_rad": math.radians(1.0),
            "pitch_error_rad": math.radians(-0.5),
            "yaw_error_rad": math.radians(1.0),
            "p_rad_s": math.radians(0.5),
            "q_rad_s": math.radians(-0.5),
            "r_rad_s": math.radians(0.5),
        }
    )
    acceptance: dict[str, Any] = {
        "derivative_maximum_relative_difference": 0.10,
        "final_normalized_feedback_error_fraction_of_initial": 0.10,
        "final_controlled_actual_residual_nm": 10.0,
        "maximum_saturation_fraction": 0.05,
        "maximum_continuous_saturation_duration_s": 0.50,
        "disallowed_allocation_statuses": ["infeasible", "numerically_singular", "solver_failure"],
    }
    trials: list[dict[str, Any]] = []
    selected_design: PhysicalWrenchLqrDesign | None = None
    selected_validation: Any | None = None
    for design, tuning_candidate in _design_candidates(projection, wrench_scales):
        try:
            validation = validate_nonlinear_wrench_lqr(
                plant,
                trim,
                design,
                initial_state=initial_state,
                duration_s=80.0,
                dt_s=0.05,
            )
        except (RuntimeError, ValueError) as error:
            trials.append(
                {
                    "tuning_candidate": tuning_candidate,
                    "design": design.as_dict(),
                    "failure": f"{type(error).__name__}: {error}",
                    "passed": False,
                }
            )
            continue
        final_error_fraction = validation.final_normalized_feedback_error_norm / max(
            validation.initial_normalized_feedback_error_norm,
            1.0e-12,
        )
        passed = (
            linearization.provenance.derivative_consistent
            and final_error_fraction <= acceptance["final_normalized_feedback_error_fraction_of_initial"]
            and validation.final_controlled_actual_residual <= acceptance["final_controlled_actual_residual_nm"]
            and validation.saturation_fraction <= acceptance["maximum_saturation_fraction"]
            and validation.maximum_continuous_saturation_duration_s <= acceptance["maximum_continuous_saturation_duration_s"]
            and not (set(validation.allocation_statuses) & set(acceptance["disallowed_allocation_statuses"]))
            and all(math.isfinite(value) for value in validation.final_state.values())
        )
        trials.append(
            {
                "tuning_candidate": tuning_candidate,
                "design": design.as_dict(),
                "metrics": {
                    **validation.as_dict()["metrics"],
                    "final_normalized_feedback_error_fraction": final_error_fraction,
                },
                "passed": passed,
            }
        )
        if passed and selected_design is None:
            selected_design = design
            selected_validation = validation
    if selected_design is None or selected_validation is None:
        summary = [
            {
                "id": str(trial["design"]["id"]),
                "metrics": dict(trial.get("metrics", {})),
                "failure": trial.get("failure"),
                "passed": bool(trial["passed"]),
            }
            for trial in trials
        ]
        raise RuntimeError(
            "no B747 physical-surface LQR profile passed the declared local acceptance gates: "
            + json.dumps(summary, sort_keys=True)
        )
    final_error_fraction = selected_validation.final_normalized_feedback_error_norm / max(
        selected_validation.initial_normalized_feedback_error_norm,
        1.0e-12,
    )
    return {
        "schema": "taoryx.b747-condition3-physical-surface-lqr/v1alpha1",
        "vehicle": "b747",
        "operating_condition": {
            "source_problem": str(PROBLEM.relative_to(ROOT)),
            "tables": [str(path.relative_to(ROOT)) for path in TABLES],
            "fidelity": "rigid_body_6dof",
            "source_condition": "NASA CR-2144 condition 3 / Mach 0.45 local neighborhood",
            "physical_control_coordinate": "elevator, aileron, rudder, and JT9D throttle",
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
            "status": "local_nonlinear_surface_validation",
            "proves": (
                "At the declared NASA CR-2144 condition-3 operating point, a B747 source-table nonlinear plant is "
                "trimmed through elevator, aileron, rudder, and installed-engine throttle; its local LQR requests "
                "moments that are allocated through the three physical aerodynamic surface coordinates and recovered "
                "by the nonlinear rigid-body plant without direct body-moment injection."
            ),
            "nonclaims": [
                "The source package does not provide servo rate or lag data; these surface actuators use an explicitly declared ideal response with only source-table deflection bounds.",
                "This is one clean condition-3 local operating point, not a gain-scheduled cruise, approach, takeoff, landing, or full-envelope controller qualification.",
                "The four-engine thrust deck supplies installed thrust but does not include fuel-flow, engine-spool, or engine-out dynamics.",
            ],
            "earned_controller_evidence_tier": "T5_nonlinearly_validated",
            "physical_allocation_evidence": "source_table_elevator_aileron_rudder_with_bounded_deflection",
            "direct_body_moment_injection": False,
        },
        "acceptance": {
            **acceptance,
            "observed_final_normalized_feedback_error_fraction": final_error_fraction,
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
            "controller_wrench_scales_nm": list(wrench_scales),
        },
        "tuning_trials": trials,
        "selected_design": selected_design.as_dict(),
        "nonlinear_validation": selected_validation.as_dict(),
        "reproduction": "PYTHONPATH=src python3 tools/validate_b747_physical_lqr.py",
    }
    ####


def _sha256(path: Path) -> str:
    """Return the immutable SHA-256 of one declared source asset."""

    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def main() -> int:
    """Write one deterministic B747 physical-control evidence packet."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "verification/generated/b747_condition3_physical_surface_lqr.json",
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
