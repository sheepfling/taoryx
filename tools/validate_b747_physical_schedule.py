#!/usr/bin/env python3
"""Validate B747 physical-surface schedule nodes from the source CSV grids.

The checked-in B747 runtime deck is intentionally a condition-3 artifact.  The
source bundle also contains condition-specific static and control grids.  This
tool materializes those grids in a temporary, content-derived deck, trims each
node through the same runtime plant adapter, and records which nodes can be
promoted to a physical-surface schedule.  It never treats an interpolated
coefficient or a failed trim as a qualified node.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import tempfile
from pathlib import Path
from typing import Any

import numpy as np

from taoryx.contracts import Vector3
from taoryx.control_allocation import EffectorLimits
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.modes import Quaternion
from taoryx.physical_lqr import project_linearization_to_wrench, validate_nonlinear_wrench_lqr
from taoryx.runtime.program import LoadedProgram
from taoryx.runtime_control_adapter import local_rigid_body_plant_from_vehicle

try:
    from import_slower_6dof_tables import _write_grid
    from validate_b747_physical_lqr import (
        _SURFACE_NAMES,
        _WRENCH_NAMES,
        PROBLEM,
        TABLE_ROOT,
        _design_candidates,
        _physical_wrench_scales,
    )
except ModuleNotFoundError:  # pragma: no cover - package execution path
    from tools.import_slower_6dof_tables import _write_grid
    from tools.validate_b747_physical_lqr import (
        _SURFACE_NAMES,
        _WRENCH_NAMES,
        PROBLEM,
        TABLE_ROOT,
        _design_candidates,
        _physical_wrench_scales,
    )

ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = TABLE_ROOT.parent / "jet_b747"
CONDITION_FILE = SOURCE_ROOT / "aero/flight_conditions.csv"
DEFAULT_OUTPUT = ROOT / "verification/alpha3_b747_physical_schedule"
NODE_IDS = ("3", "4", "5", "6", "7", "8", "9", "10")
INTERIOR_PERTURBATIONS: dict[str, dict[str, float]] = {
    "nominal_local": {
        "roll_error_rad": math.radians(1.0),
        "pitch_error_rad": math.radians(-0.5),
        "yaw_error_rad": math.radians(1.0),
        "p_rad_s": math.radians(0.5),
        "q_rad_s": math.radians(-0.5),
        "r_rad_s": math.radians(0.5),
    },
    "speed_minus_5m_s": {"u_m_s": -5.0},
    "speed_plus_5m_s": {"u_m_s": 5.0},
}
BOUNDARY_PERTURBATION = {
    "roll_error_rad": math.radians(2.0),
    "pitch_error_rad": math.radians(-1.0),
    "yaw_error_rad": math.radians(2.0),
    "p_rad_s": math.radians(1.0),
    "q_rad_s": math.radians(-1.0),
    "r_rad_s": math.radians(1.0),
}

_CONDITION3_ALPHA0_DEG = 3.1
_CONDITION3_QUATERNION = (
    0.5115355556970181,
    -0.5115355556970181,
    -0.4881919450971543,
    0.4881919450971543,
)


class B747NodeIntegrationFailure(RuntimeError):
    """Structured failure for one source-condition schedule node."""

    def __init__(self, code: str, message: str, hint: str, details: dict[str, Any]) -> None:
        super().__init__(message)
        self.code = code
        self.hint = hint
        self.details = details
        ####
    ####


def _read_conditions() -> dict[str, dict[str, str]]:
    """Load the source operating-point rows keyed by NASA condition ID."""

    with CONDITION_FILE.open(encoding="utf-8", newline="") as handle:
        rows = {str(row["fc_id"]): row for row in csv.DictReader(handle)}
    missing = [node_id for node_id in NODE_IDS if node_id not in rows]
    if missing:
        raise ValueError(f"B747 source condition catalog is missing nodes: {missing}")
    return rows
    ####


def _limits() -> dict[str, EffectorLimits]:
    """Return the declared B747 surface bounds used by the local adapter."""

    return {
        "elevator-deg": EffectorLimits("elevator-deg", -10.0, 10.0, "deg"),
        "aileron-deg": EffectorLimits("aileron-deg", -10.0, 10.0, "deg"),
        "rudder-deg": EffectorLimits("rudder-deg", -15.0, 15.0, "deg"),
        "throttle": EffectorLimits("throttle", 0.0, 1.0, "fraction"),
    }
    ####


def _filter(row: dict[str, str], point: dict[str, str]) -> bool:
    """Select one complete nominal source grid at a flight-condition node.

    The B747 source tables contain repeated Mach/condition rows at different
    altitudes.  Altitude is not a runtime interpolation axis for this local
    node deck, so it must be part of the materialization filter; otherwise a
    node silently mixes incompatible density/trim data before the solver sees
    it.
    """

    return (
        row["configuration"] == "nominal"
        and row["reference_fc_id"] == point["fc_id"]
        and row["mach"] == point["mach"]
        and row["altitude_ft"] == point["altitude_ft"]
    )
    ####


def _node_quaternion(point: dict[str, str]) -> Quaternion:
    """Return the source-node attitude while preserving the FC3 oracle.

    The checked-in FC3 problem encodes the source trim attitude in its
    quaternion.  Its actual body-velocity angle is a derived quantity and is
    not exactly the rounded ``alpha0_deg`` label in the source condition
    catalog.  Other CR-2144 rows publish a different ``alpha0_deg``.  A
    schedule intake must carry that operating-point change into the initial
    body attitude relative to the quaternion's actual FC3 body-velocity
    angle; offsetting from the rounded label leaves every non-FC3 node with a
    residual alpha mismatch that the elevator then incorrectly absorbs.
    The rotation is applied in the local body pitch axis, and FC3 is exactly
    unchanged by construction.
    """

    base = Quaternion(*_CONDITION3_QUATERNION).normalized()
    if point["fc_id"] == "3":
        return base
    body_velocity = base.conjugate().rotate(Vector3(0.0, 1.0, 0.0))
    condition3_body_alpha_deg = math.degrees(math.atan2(body_velocity.z, body_velocity.x))
    delta = math.radians(float(point["alpha0_deg"]) - condition3_body_alpha_deg)
    half_delta = 0.5 * delta
    return base.multiply(Quaternion(math.cos(half_delta), 0.0, math.sin(half_delta), 0.0)).normalized()
    ####


def _write_node_decks(directory: Path, point: dict[str, str]) -> tuple[Path, ...]:
    """Materialize source-preserving node decks into an isolated directory."""

    directory.mkdir(parents=True, exist_ok=True)
    coefficient_values = tuple((name, name.upper()) for name in ("cx", "cy", "cz", "cmx", "cmy", "cmz"))
    for kind in ("static", "elevator", "aileron", "rudder"):
        source = SOURCE_ROOT / "aero" / ("static_six_axis_grid.csv" if kind == "static" else f"{kind}_grid.csv")
        axes = (
            (("alpha", "alpha_offset_from_reference_deg"), ("beta", "beta_deg"))
            if kind == "static"
            else (
                ("alpha", "alpha_offset_from_reference_deg"),
                ("beta", "beta_deg"),
                (kind, f"{kind}_deg"),
            )
        )
        _write_grid(
            source,
            directory / f"b747_nominal_{kind}_6axis.tbl",
            axes=axes,
            filters=(lambda row, selected=point: _filter(row, selected),),
            value_columns=coefficient_values,
            comment=f"source NASA CR-2144 condition {point['fc_id']} node; angular axes are radians",
        )
    thrust = TABLE_ROOT / "b747_jt9d_thrust.tbl"
    thrust_target = directory / thrust.name
    thrust_target.write_text(thrust.read_text(encoding="utf-8"), encoding="utf-8")
    return tuple(directory / name for name in ("b747_nominal_static_6axis.tbl", "b747_nominal_elevator_6axis.tbl", "b747_nominal_aileron_6axis.tbl", "b747_nominal_rudder_6axis.tbl", thrust.name))
    ####


def _node_problem(point: dict[str, str]) -> str:
    """Return a temporary problem binding one source condition to the runtime."""

    # Keep the accepted runtime mass snapshot fixed while the source schedule
    # varies altitude, speed, and coefficient decks.  The source CSV weight
    # converts to a slightly different round-trip mass; silently changing it
    # would make the known condition-3 trim cease to be the regression oracle.
    mass = 288756.9
    altitude = 6378137.0 + float(point["altitude_m"]) + 100.0
    text = PROBLEM.read_text(encoding="utf-8")
    text = text.replace("b747-condition3-surface-trim-6dof", f"b747-condition{point['fc_id']}-physical-surface")
    text = text.replace("aero-alpha-reference-deg=3.1", f"aero-alpha-reference-deg={point['alpha0_deg']}")
    quaternion = _node_quaternion(point)
    text = re.sub(
        r"qw=[^ ]+ qx=[^ ]+ qy=[^ ]+ qz=[^ ]+",
        f"qw={quaternion.w:.12g} qx={quaternion.x:.12g} qy={quaternion.y:.12g} qz={quaternion.z:.12g}",
        text,
        count=1,
    )
    text = re.sub(r"x=6378237\.0 y=0\.0 z=0\.0 xdt=0\.0 ydt=153\.0096", f"x={altitude:.9f} y=0.0 z=0.0 xdt=0.0 ydt={point['velocity_m_s']}", text)
    text = text.replace("dry-mass-kg=288756.9", f"dry-mass-kg={mass:.9f}")
    text = text.replace("mass=288756.9", f"mass={mass:.9f}")
    return text
    ####


def _build_node(directory: Path, point: dict[str, str]) -> tuple[Any, Any, Any, Any]:
    """Build one source-derived runtime plant, trim, and physical LQR."""

    directory.mkdir(parents=True, exist_ok=True)
    problem = directory / "node.prb"
    problem.write_text(_node_problem(point), encoding="utf-8")
    tables = _write_node_decks(directory, point)
    program = LoadedProgram.load(problem, tables, profile=GrammarProfile.TAORYX)
    plant = local_rigid_body_plant_from_vehicle(
        f"b747-condition{point['fc_id']}-source-table-plant",
        f"nasa-cr-2144-condition{point['fc_id']}-local-v1",
        program.case().vehicles["1"],
        inertia_kg_m2=Vector3(24_675_886.7, 44_877_574.1, 67_384_152.0),
        reference_length_m=8.324088,
        effector_limits=_limits(),
        state_bounds={
            # The independent CR-2144 coefficient rows are not all force
            # balanced at their rounded alpha0 labels.  Permit only the local
            # body-velocity component that changes alpha, and keep it inside
            # the declared source table's +/-4 degree alpha-offset domain.
            "w_m_s": (
                float(point["velocity_m_s"])
                * math.tan(math.radians(float(point["alpha0_deg"]) - 3.9)),
                float(point["velocity_m_s"])
                * math.tan(math.radians(float(point["alpha0_deg"]) + 3.9)),
            )
        },
        effectiveness_steps={"elevator-deg": 0.1, "aileron-deg": 0.1, "rudder-deg": 0.1, "throttle": 0.005},
        allocation_wrench_weights={name: 1.0 for name in _WRENCH_NAMES},
        allocation_regularization=1.0e-14,
        allocation_feasibility_tolerance=1.0e-3,
    )
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        residuals = {name: float(value) for name, value in trim.residuals.items()}
        worst_name = max(residuals, key=lambda name: abs(residuals[name]), default=None)
        raise B747NodeIntegrationFailure(
            "source_trim_residual_above_tolerance",
            f"source condition {point['fc_id']} trim did not converge",
            "Resolve the source operating-point attitude, stabilizer/trim-bias convention, mass properties, and control-table closure before scheduling this node.",
            {
                "source_condition": point["fc_id"],
                "scaled_residual_norm": float(trim.scaled_residual_norm),
                "maximum_residual": float(trim.max_residual),
                "worst_residual_name": worst_name,
                "worst_residual": residuals.get(worst_name) if worst_name is not None else None,
                "residuals": residuals,
                "candidate_state": {name: float(value) for name, value in trim.state.items()},
                "candidate_controls": {name: float(value) for name, value in trim.controls.items()},
            },
        )
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
        state_names=plant.state_names,
        wrench_names=_WRENCH_NAMES,
        effector_names=_SURFACE_NAMES,
    )
    designs = _design_candidates(projection, _physical_wrench_scales(effectiveness, plant))
    return plant, trim, linearization, designs
    ####


def _run_case(plant: Any, trim: Any, design: Any, update: dict[str, float]) -> dict[str, Any]:
    """Run one bounded local perturbation and retain auditable metrics."""

    state = dict(trim.state)
    state.update({name: float(state[name]) + value for name, value in update.items()})
    validation = validate_nonlinear_wrench_lqr(plant, trim, design, initial_state=state, duration_s=80.0, dt_s=0.05)
    fraction = validation.final_normalized_feedback_error_norm / max(validation.initial_normalized_feedback_error_norm, 1.0e-12)
    passed = fraction <= 0.10 and validation.saturation_fraction <= 0.05 and validation.maximum_continuous_saturation_duration_s <= 0.50 and not set(validation.allocation_statuses).intersection({"infeasible", "numerically_singular", "solver_failure"})
    return {
        "passed": passed,
        "final_normalized_feedback_error_fraction": fraction,
        "allocation_statuses": list(validation.allocation_statuses),
        "saturation_fraction": validation.saturation_fraction,
        "maximum_continuous_saturation_duration_s": validation.maximum_continuous_saturation_duration_s,
        "final_controlled_actual_residual_nm": validation.final_controlled_actual_residual,
    }
    ####


def _evaluate_node(directory: Path, point: dict[str, str]) -> dict[str, Any]:
    """Evaluate one node, converting expected data/trim failures to evidence."""

    result: dict[str, Any] = {
        "point_id": point["fc_id"],
        "altitude_m": float(point["altitude_m"]),
        "mach": float(point["mach"]),
        "true_airspeed_m_s": float(point["velocity_m_s"]),
        "source_condition": dict(point),
        "initialization": {
            "alpha0_deg": float(point["alpha0_deg"]),
            "reference_condition3_alpha0_deg": _CONDITION3_ALPHA0_DEG,
            "body_pitch_offset_from_condition3_deg": float(point["alpha0_deg"]) - _CONDITION3_ALPHA0_DEG,
            "attitude_policy": "source_condition_alpha0_applied_as_local_body_pitch_offset",
        },
    }
    try:
        plant, trim, linearization, designs = _build_node(directory, point)
        trimmed_alpha_deg = math.degrees(math.atan2(float(trim.state["w_m_s"]), float(trim.state["u_m_s"])))
        result["initialization"].update(
            {
                "published_alpha0_deg": float(point["alpha0_deg"]),
                "retrimmed_body_alpha_deg": trimmed_alpha_deg,
                "retrim_alpha_delta_deg": trimmed_alpha_deg - float(point["alpha0_deg"]),
                "retrim_state_channel": "w_m_s",
                "retrim_domain": "published_alpha0_deg +/- 3.9 degrees",
            }
        )
        selected = None
        selected_cases: dict[str, Any] | None = None
        controller_trials: list[dict[str, Any]] = []
        for design, tuning in designs:
            cases = {case_id: _run_case(plant, trim, design, update) for case_id, update in INTERIOR_PERTURBATIONS.items()}
            controller_trials.append(
                {
                    "design_id": design.id,
                    "tuning": tuning,
                    "cases": cases,
                    "interior_passed": all(case["passed"] for case in cases.values()),
                }
            )
            if all(case["passed"] for case in cases.values()):
                selected = (design, tuning)
                selected_cases = cases
                break
        if selected is None or selected_cases is None:
            result.update(
                {
                    "status": "physical_surface_node_boundary",
                    "reason": "no generic local design passed all interior witnesses",
                    "trim": trim.as_dict(),
                    "derivative_consistent": linearization.provenance.derivative_consistent,
                    "controller_trials": controller_trials,
                    "boundary_contract": "source trim passed; controller/effector authority remains unqualified until one generic candidate passes every declared interior witness",
                }
            )
            return result
        design, tuning = selected
        boundary = _run_case(plant, trim, design, BOUNDARY_PERTURBATION)
        result.update({
            "status": "physical_surface_node_complete",
            "direct_body_moment_injection": False,
            "selected_design": {"id": design.id, "tuning": tuning},
            "trim": trim.as_dict(),
            "derivative_consistent": linearization.provenance.derivative_consistent,
            "maximum_real_pole": design.result.maximum_real_pole,
            "effectiveness_rank": int(np.linalg.matrix_rank(np.asarray(plant.effectiveness(trim.state, trim.controls).matrix))),
            "cases": selected_cases,
            "boundary_witness": {"case_id": "coupled_rate_authority_boundary", **boundary},
            "interior_passed": all(case["passed"] for case in selected_cases.values()),
        })
    except Exception as error:  # noqa: BLE001 - preserve node-specific integration diagnostics
        if isinstance(error, B747NodeIntegrationFailure):
            result.update(
                {
                    "status": "physical_surface_node_blocked",
                    "reason": str(error),
                    "blocker": {
                        "code": error.code,
                        "message": str(error),
                        "hint": error.hint,
                        "details": error.details,
                    },
                }
            )
        else:
            result.update(
                {
                    "status": "physical_surface_node_blocked",
                    "reason": f"{type(error).__name__}: {error}",
                    "blocker": {
                        "code": "source_node_integration_exception",
                        "message": str(error),
                        "hint": "Inspect the source-condition deck generation, table domains, and plant adapter contract before attempting promotion.",
                        "details": {"exception_type": type(error).__name__},
                    },
                }
            )
    return result
    ####


def build_schedule(output: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Build the source-derived B747 schedule-node evidence packet."""

    points = _read_conditions()
    with tempfile.TemporaryDirectory(prefix="taoryx-b747-schedule-") as temporary:
        temporary_root = Path(temporary)
        nodes = [_evaluate_node(temporary_root / point_id, points[point_id]) for point_id in NODE_IDS]

    complete_nodes = [node for node in nodes if node["status"] == "physical_surface_node_complete"]
    report: dict[str, Any] = {
        "schema": "taoryx.b747-physical-effector-schedule/v1alpha1",
        "status": "B747_physical_effector_schedule_nodes_complete" if len(complete_nodes) == len(nodes) else "B747_physical_effector_schedule_boundary_recorded",
        "family_id": "b747",
        "controller_profile": "generic_profile_sweep_per_source_condition",
        "control_path": "source_condition_deck -> physical trim -> source linearization -> desired wrench -> bounded elevator/aileron/rudder/throttle -> nonlinear plant",
        "direct_body_moment_injection": False,
        "schedule_contract": {
            "node_ids": list(NODE_IDS),
            "runtime_gain_interpolation": "not_run_until_all_source_nodes_complete",
            "transition_evidence": "blocked until every promoted node has a physical plant and trim",
        },
        "source_deck_generation": {
            "source_condition_catalog": str(CONDITION_FILE.relative_to(ROOT)),
            "filters": ["configuration=nominal", "reference_fc_id=node.fc_id", "mach=node.mach", "altitude_ft=node.altitude_ft"],
            "temporary_deck_policy": "materialized from source CSV for each run; no interpolated coefficient deck is persisted as qualified evidence",
            "mass_policy": "retain the accepted 288756.9 kg runtime mass snapshot while varying source condition decks; source CSV weight conversion is recorded but not silently substituted",
            "source_control_tables": ["static_six_axis_grid.csv", "elevator_grid.csv", "aileron_grid.csv", "rudder_grid.csv"],
            "attitude_initialization_policy": "preserve the checked-in condition-3 quaternion and apply each source row alpha0 delta about the local body pitch axis; condition 3 remains byte-for-byte equivalent in orientation",
            "bounded_retrim_policy": "allow only local body w velocity to move within source alpha0 +/- 3.9 degrees; published alpha0 remains the initial catalog label and re-trimmed alpha is reported from truth state",
        },
        "nodes": nodes,
        "summary": {
            "node_count": len(nodes),
            "passed_node_count": len(complete_nodes),
            "blocked_node_count": sum(node["status"] == "physical_surface_node_blocked" for node in nodes),
            "boundary_node_count": sum(node["status"] == "physical_surface_node_boundary" for node in nodes),
            "interior_case_count": len(nodes) * len(INTERIOR_PERTURBATIONS),
            "interior_passed_case_count": sum(sum(bool(case["passed"]) for case in node.get("cases", {}).values()) for node in nodes),
            "blockers_by_code": {
                code: sum(1 for node in nodes if node.get("blocker", {}).get("code") == code)
                for code in sorted({str(node.get("blocker", {}).get("code")) for node in nodes if node.get("blocker")})
            },
        },
        "claim_boundary": "Source-derived physical-surface node evidence only. No schedule interpolation, continuous transition replay, full-envelope qualification, servo dynamics, fuel-flow validation, runway operation, or reliability claim is made until every declared node completes. Blocked nodes carry machine-readable integration codes and remedies.",
        "reproduction": "PYTHONPATH=src python3 tools/validate_b747_physical_schedule.py",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Build the deterministic B747 source-effector schedule packet."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    report = build_schedule(arguments.output)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "B747_physical_effector_schedule_nodes_complete" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
