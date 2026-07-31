#!/usr/bin/env python3
"""Validate source-effector F-16 controller nodes across the local schedule.

This is the next promotion step after the single operating-point physical
effector matrix. Each node is retrimmed from the source catalog, linearized
from the same nonlinear plant used for runtime evaluation, and exercised with
the common physical-wrench perturbation cases. The artifact compares adjacent
gain/effectiveness nodes but deliberately does not claim that a continuous
runtime gain scheduler exists yet.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from taoryx.control_allocation import EffectorLimits
from taoryx.physical_lqr import (
    PhysicalWrenchLqrSchedule,
    PhysicalWrenchLqrScheduleNode,
    design_physical_wrench_lqr,
    project_linearization_to_wrench,
    validate_nonlinear_wrench_lqr,
)
from taoryx.trajectory import F16ReferencePhysicalPlant, load_f16_reference_plant
from taoryx.trajectory.f16_operating_points import runtime_trim_result, solve_f16_source_trim

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_f16_physical_schedule"
POINT_IDS = (
    "f16-sea-level-152mps",
    "f16-3km-152mps",
    "f16-6km-152mps",
    "f16-9km-152mps",
)
PERTURBATIONS: dict[str, dict[str, float]] = {
    "u_plus": {"u_m_s": 0.1},
    "w_plus": {"w_m_s": 0.02},
    "q_plus": {"q_rad_s": 0.0004},
    "coupled_reversal": {"u_m_s": -0.1, "w_m_s": -0.02, "q_rad_s": -0.0004},
    "lateral_coupled": {"v_m_s": 0.1, "p_rad_s": 0.0002, "r_rad_s": -0.0002},
}
RECOVERY_THRESHOLD = 0.20


def _load_limits() -> dict[str, EffectorLimits]:
    """Load the declared engineering actuator overlay."""

    payload = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/actuators/reference-first-order-v1.yaml").read_text(encoding="utf-8")
    )
    names = {"elevator": "elevator_deg", "aileron": "aileron_deg", "rudder": "rudder_deg", "throttle": "throttle_fraction"}
    dynamics = payload.get("dynamics", {})
    default_time_constant = float(dynamics.get("time_constant_s", 0.0))
    limits: dict[str, EffectorLimits] = {}
    for source_name, values in payload["limits"].items():
        position = values.get("position")
        if position is None:
            position = [values["lower"], values["upper"]]
        name = names[source_name]
        limits[name] = EffectorLimits(
            name,
            float(position[0]),
            float(position[1]),
            "fraction" if source_name == "throttle" else "deg",
            float(values["rate_per_s"]) if values.get("rate_per_s") is not None else None,
            float(values.get("time_constant_s", default_time_constant)),
        )
    return limits
    ####


def _build_node(source: Any, limits: dict[str, EffectorLimits], catalog_point: dict[str, Any]) -> tuple[Any, Any, Any, Any]:
    """Resolve one source trim and its plant-derived physical LQR."""

    environment = catalog_point["environment"]
    state = catalog_point["state"]
    controls = catalog_point["controls"]
    operating_point = solve_f16_source_trim(
        source,
        point_id=str(catalog_point["id"]),
        altitude_m=float(environment["geometric_altitude_m"]),
        true_airspeed_m_s=float(environment["true_airspeed_m_s"]),
        initial_alpha_deg=float(state["alpha_deg"]),
        initial_elevator_deg=float(controls["elevator_deg"]),
        initial_throttle_fraction=float(controls["throttle_fraction"]),
    )
    trim = runtime_trim_result(operating_point)
    adapter = F16ReferencePhysicalPlant(source, trim, operating_point.trim_pitch_rad, operating_point.altitude_m, limits)
    linearization = adapter.linearize(trim, {"state_step": 1.0e-5, "control_step": 1.0e-5})
    effectiveness = adapter.effectiveness(trim.state, trim.controls)
    projection = project_linearization_to_wrench(
        linearization,
        effectiveness,
        state_names=trim.spec.state_names,
        wrench_names=adapter.wrench_names,
        effector_names=trim.spec.control_names,
    )
    design = design_physical_wrench_lqr(
        f"f16.physical_schedule.{operating_point.point_id}",
        projection,
        q_diagonal=(10.0,) * 6,
        r_diagonal=(0.01,) * 4,
        state_scales=(50.0, 50.0, 50.0, 0.5, 0.5, 0.5),
        wrench_scales=(5000.0, 10000.0, 10000.0, 10000.0),
    )
    return operating_point, trim, adapter, design
    ####


def _validate_node(trim: Any, adapter: Any, design: Any) -> dict[str, Any]:
    """Run the shared local physical perturbation contract at one node."""

    cases: dict[str, Any] = {}
    for case_id, perturbation in PERTURBATIONS.items():
        initial_state = dict(trim.state)
        initial_state.update({name: initial_state[name] + value for name, value in perturbation.items()})
        validation = validate_nonlinear_wrench_lqr(
            adapter,
            trim,
            design,
            initial_state=initial_state,
            duration_s=5.0,
            dt_s=0.02,
        )
        ratio = validation.final_normalized_feedback_error_norm / max(validation.initial_normalized_feedback_error_norm, 1.0e-12)
        passed = ratio < RECOVERY_THRESHOLD and validation.allocation_statuses == ("feasible",) and validation.saturation_fraction == 0.0
        cases[case_id] = {
            "perturbation": perturbation,
            "passed": passed,
            "recovery_ratio": ratio,
            "allocation_statuses": list(validation.allocation_statuses),
            "saturation_fraction": validation.saturation_fraction,
            "maximum_controlled_actual_residual": validation.maximum_controlled_actual_residual,
            "maximum_continuous_saturation_duration_s": validation.maximum_continuous_saturation_duration_s,
        }
    return cases
    ####


def build_schedule(output: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    """Build the deterministic multi-node physical-effector schedule packet."""

    source = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    catalog = yaml.safe_load(
        (ROOT / "families/reference_f16_s119/qualification/operating-points.yaml").read_text(encoding="utf-8")
    )
    catalog_by_id = {str(point["id"]): point for point in catalog["points"]}
    missing = [point_id for point_id in POINT_IDS if point_id not in catalog_by_id]
    if missing:
        raise ValueError(f"F-16 schedule catalog is missing points: {missing}")
    limits = _load_limits()
    nodes: list[dict[str, Any]] = []
    gains: list[np.ndarray] = []
    schedule_nodes: list[PhysicalWrenchLqrScheduleNode] = []
    trim_states: list[dict[str, float]] = []
    trims: list[Any] = []
    adapters: list[Any] = []
    effectiveness_ranks: list[int] = []
    for point_id in POINT_IDS:
        operating_point, trim, adapter, design = _build_node(source, limits, catalog_by_id[point_id])
        cases = _validate_node(trim, adapter, design)
        gains.append(np.asarray(design.result.gain, dtype=float))
        schedule_nodes.append(PhysicalWrenchLqrScheduleNode(operating_point.altitude_m, design))
        trim_states.append({name: float(value) for name, value in trim.state.items()})
        trims.append(trim)
        adapters.append(adapter)
        effectiveness_ranks.append(int(np.linalg.matrix_rank(np.asarray(adapter.effectiveness(operating_point.body_state, operating_point.controls).matrix))))
        nodes.append(
            {
                "point_id": point_id,
                "altitude_m": operating_point.altitude_m,
                "true_airspeed_m_s": operating_point.true_airspeed_m_s,
                "mach": operating_point.mach,
                "trim_max_residual": operating_point.max_residual,
                "trim_scaled_residual_norm": operating_point.trim.scaled_residual_norm,
                "trim_controls": dict(operating_point.controls),
                "derivative_consistent": design.projection.source_linearization.provenance.derivative_consistent,
                "maximum_real_pole": design.result.maximum_real_pole,
                "controllable": design.result.controllable,
                "condition_number": design.result.condition_number,
                "effectiveness_rank": effectiveness_ranks[-1],
                "cases": cases,
                "passed_case_count": sum(bool(case["passed"]) for case in cases.values()),
                "failed_case_count": sum(not bool(case["passed"]) for case in cases.values()),
            }
        )
    schedule = PhysicalWrenchLqrSchedule(tuple(schedule_nodes))
    transitions: list[dict[str, Any]] = []
    for left, right, left_gain, right_gain in zip(POINT_IDS[:-1], POINT_IDS[1:], gains[:-1], gains[1:], strict=True):
        denominator = max(float(np.linalg.norm(left_gain)), float(np.linalg.norm(right_gain)), 1.0e-12)
        left_index = POINT_IDS.index(left)
        right_index = POINT_IDS.index(right)
        midpoint = 0.5 * (schedule_nodes[left_index].coordinate + schedule_nodes[right_index].coordinate)
        midpoint_state = {
            name: 0.5 * (trim_states[left_index][name] + trim_states[right_index][name])
            for name in schedule.state_names
        }
        midpoint_command = schedule.command(midpoint_state, midpoint)
        transitions.append(
            {
                "from_point": left,
                "to_point": right,
                "normalized_gain_delta_frobenius": float(np.linalg.norm(right_gain - left_gain) / denominator),
                "transition_evidence": "generic_interpolated_wrench_command",
                "midpoint_command_finite": all(math.isfinite(value) for value in midpoint_command.requested_wrench.values()),
                "midpoint_interpolation_fraction": midpoint_command.interpolation_fraction,
            }
        )
    transition_allocation_probes: list[dict[str, Any]] = []
    for left_index, (left, right) in enumerate(zip(POINT_IDS[:-1], POINT_IDS[1:], strict=True)):
        lower_node = schedule_nodes[left_index]
        upper_node = schedule_nodes[left_index + 1]
        coordinates = np.linspace(lower_node.coordinate, upper_node.coordinate, 5)
        probes: list[dict[str, Any]] = []
        for coordinate in coordinates:
            fraction = float((coordinate - lower_node.coordinate) / (upper_node.coordinate - lower_node.coordinate))
            state = {
                name: (1.0 - fraction) * trim_states[left_index][name] + fraction * trim_states[left_index + 1][name]
                for name in schedule.state_names
            }
            command = schedule.command(state, float(coordinate))
            endpoint_results: dict[str, Any] = {}
            for endpoint, adapter, trim in (
                ("lower", adapters[left_index], trims[left_index]),
                ("upper", adapters[left_index + 1], trims[left_index + 1]),
            ):
                previous_effectors = {name: float(value) for name, value in trim.controls.items()}
                allocation = adapter.allocate(state, command.requested_wrench, previous_effectors, 0.02)
                endpoint_results[endpoint] = {
                    "status": allocation.allocation.status,
                    "achieved_controlled_residual_norm": allocation.achieved_controlled_residual_norm,
                    "position_saturated": list(allocation.actuator.position_saturated),
                    "rate_limited": list(allocation.actuator.rate_limited),
                    "actual_effectors": dict(allocation.actuator.actual_positions),
                }
            passed = all(
                result["status"] == "feasible"
                and math.isfinite(float(result["achieved_controlled_residual_norm"]))
                and not result["position_saturated"]
                and not result["rate_limited"]
                for result in endpoint_results.values()
            )
            probes.append(
                {
                    "coordinate_m": float(coordinate),
                    "interpolation_fraction": fraction,
                    "command": command.as_dict(),
                    "endpoint_allocations": endpoint_results,
                    "passed": passed,
                }
            )
        transition_allocation_probes.append(
            {
                "from_point": left,
                "to_point": right,
                "probe_count": len(probes),
                "passed_probe_count": sum(bool(probe["passed"]) for probe in probes),
                "failed_probe_count": sum(not bool(probe["passed"]) for probe in probes),
                "probes": probes,
            }
        )
    all_passed = all(node["failed_case_count"] == 0 for node in nodes)
    report: dict[str, Any] = {
        "schema": "taoryx.f16-physical-effector-schedule/v1alpha1",
        "status": "F16_physical_effector_schedule_nodes_complete" if all_passed else "F16_physical_effector_schedule_boundary_recorded",
        "family_id": "reference_f16_s119",
        "controller_profile": "state_and_wrench_balanced_q10_r0p01",
        "control_path": "source_trim -> source_linearization -> desired_wrench -> bounded_elevator_aileron_rudder_throttle -> source_nonlinear_plant",
        "direct_body_moment_injection": False,
        "schedule_contract": {
            "node_ids": list(POINT_IDS),
            "same_local_perturbation_matrix_at_each_node": True,
            "runtime_gain_interpolation": "generic_command_contract_exercised",
            "transition_evidence": "adjacent gain comparison plus endpoint physical-allocation probes; time-marching nonlinear transition replay pending",
        },
        "nodes": nodes,
        "transitions": transitions,
        "transition_allocation_probes": transition_allocation_probes,
        "summary": {
            "node_count": len(nodes),
            "passed_node_count": sum(node["failed_case_count"] == 0 for node in nodes),
            "case_count": len(nodes) * len(PERTURBATIONS),
            "passed_case_count": sum(node["passed_case_count"] for node in nodes),
            "failed_case_count": sum(node["failed_case_count"] for node in nodes),
            "minimum_effectiveness_rank": min(effectiveness_ranks),
            "maximum_trim_residual": max(float(node["trim_max_residual"]) for node in nodes),
            "maximum_real_pole": max(float(node["maximum_real_pole"]) for node in nodes),
            "transition_probe_count": sum(len(item["probes"]) for item in transition_allocation_probes),
            "transition_probe_passed_count": sum(item["passed_probe_count"] for item in transition_allocation_probes),
            "transition_probe_failed_count": sum(item["failed_probe_count"] for item in transition_allocation_probes),
        },
        "claim_boundary": (
            "Four source-retrimmed local physical-effector nodes pass the declared perturbation matrix. "
            "The generic scheduled wrench-demand contract is exercised at adjacent midpoints and endpoint physical "
            "allocation probes, but no time-marching nonlinear "
            "continuous transition replay, statistical reliability, or full-envelope flight-control qualification is claimed. "
            "Actuator dynamics remain the declared engineering first-order overlay."
        ),
        "reproduction": "PYTHONPATH=src python3 tools/validate_f16_physical_schedule.py",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "manifest.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "reproduction.txt").write_text(str(report["reproduction"]) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Build the F-16 physical-effector schedule artifact."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    report = build_schedule(arguments.output)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["status"] == "F16_physical_effector_schedule_nodes_complete" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
