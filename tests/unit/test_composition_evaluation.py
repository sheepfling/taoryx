"""Tests for conservative normalized Product 3 run-evidence projection."""

from __future__ import annotations

from pathlib import Path

from taoryx.composition_control_trace import BatchControlSample, build_committed_control_trace
from taoryx.composition_evaluation import build_composition_trajectory_evaluation
from taoryx.composition_sensor_trace import BatchTruthSample
from taoryx.composition_status_trace import build_committed_status_trace
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from taoryx.vehicle_execution_preflight import preflight_vehicle_composition

ROOT = Path(__file__).resolve().parents[2]


def test_normalized_evaluation_projects_final_committed_resource_without_inventing_controls() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    )
    trace = build_committed_status_trace(
        composition,
        (
            BatchTruthSample(
                time_s=3.0,
                execution_status="completed",
                raw_values={
                    "position_ned_m": [0.0, 0.0, 0.0],
                    "velocity_ned_m_s": [0.0, 0.0, 0.0],
                    "attitude_rad": [0.0, 0.0, 0.0],
                    "body_rate_rad_s": [0.0, 0.0, 0.0],
                    "battery_fraction": 0.75,
                    "aggregate_thrust_n": 4.9,
                    "contact": True,
                    "response_profile_id": "hummingbird_attitude_response_pseudo6dof.v1",
                    "control_realization": "aggregate_thrust_vector_surrogate",
                    "physical_motor_allocation": False,
                    "mass_kg": 0.5,
                },
            ),
        ),
    )

    evaluation = build_composition_trajectory_evaluation(
        composition,
        preflight_vehicle_composition(composition),
        {"mission_pass": True, "results": []},
        runtime={"numerical_valid": True, "hard_gates_passed": True},
        envelope={"pass": True},
        claim_boundary="test fixture",
        status_trace=trace,
    )

    resources = {item.id: item for item in evaluation.resources}
    assert resources["resources.battery.fraction_remaining"].value == 0.75
    assert resources["resources.battery.fraction_remaining"].time_s == 3.0
    assert resources["resources.mass.total"].value == 0.5
    assert evaluation.feasibility == "likely_feasible"
    assert all(item.status == "unavailable" for item in evaluation.requested_controls)
    assert all(item.status == "unavailable" for item in evaluation.achieved_controls)
    graph_gate = next(gate for gate in evaluation.gates if gate.id == "mission-graph-execution")
    assert graph_gate.status == "advisory"
    assert "not asserted" in graph_gate.message
    ####


def test_normalized_evaluation_projects_a_validated_semantic_action_trace() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    )
    control_trace = build_committed_control_trace(
        composition,
        (
            BatchControlSample(
                interval_start_time_s=2.0,
                committed_truth_time_s=2.1,
                requested_actions={
                    "attitude.roll.command": 0.1,
                    "attitude.pitch.command": -0.2,
                    "attitude.yaw.command": 0.3,
                    "propulsion.command.fraction": 0.6,
                    "propulsion.enable": True,
                },
                achieved_effectors={},
            ),
        ),
    )

    evaluation = build_composition_trajectory_evaluation(
        composition,
        preflight_vehicle_composition(composition),
        {"mission_pass": True, "results": []},
        runtime={"numerical_valid": True, "hard_gates_passed": True},
        envelope={"pass": True},
        claim_boundary="test fixture",
        control_trace=control_trace,
    )

    requested = {item.id: item for item in evaluation.requested_controls}
    assert requested["requested.attitude.roll.command"].value == 0.1
    assert requested["requested.propulsion.enable"].value is True
    assert requested["requested.propulsion.command.fraction"].time_s == 2.1
    assert evaluation.achieved_controls == ()
    ####


def test_normalized_evaluation_reports_observed_graph_dispatches_as_evidence_not_mission_pass() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    )

    evaluation = build_composition_trajectory_evaluation(
        composition,
        preflight_vehicle_composition(composition),
        {"mission_pass": False, "results": []},
        runtime={
            "numerical_valid": True,
            "hard_gates_passed": True,
            "mission_graph_execution": {
                "observation_status": "observed",
                "completed_nominal_success_path": True,
            },
        },
        envelope={"pass": True},
        claim_boundary="test fixture",
    )

    graph_gate = next(gate for gate in evaluation.gates if gate.id == "mission-graph-execution")
    assert graph_gate.status == "advisory"
    assert "completion=True" in graph_gate.message
    assert evaluation.outcome == "partial"
    ####
