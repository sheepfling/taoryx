"""Tests for honest mission-graph execution evidence artifacts."""

from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.composition_graph_evidence import (
    GraphExecutionDispatch,
    graph_dispatch,
    observed_mission_graph_execution,
    unobserved_mission_graph_execution,
)
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request

ROOT = Path(__file__).resolve().parents[2]


def _composition(name: str):
    """Compile one checked-in composition fixture."""

    return compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / name))
    ####


def test_unobserved_graph_record_does_not_imply_controller_progress() -> None:
    composition = _composition("x8_racetrack_capability_3dof_compose.yaml")

    artifact = unobserved_mission_graph_execution(
        composition,
        "Batch telemetry does not contain committed controller graph dispatches.",
    ).as_dict()

    assert artifact["schema"] == "taoryx.mission-graph-execution/v1alpha1"
    assert artifact["observation_status"] == "unobserved"
    assert artifact["dispatches"] == []
    assert artifact["completed_nominal_success_path"] is None
    assert artifact["unobserved_reason"]
    ####


def test_observed_success_dispatches_cover_the_declared_nominal_path() -> None:
    composition = _composition("x8_racetrack_capability_3dof_compose.yaml")
    assert composition.mission_graph is not None

    dispatches = tuple(
        graph_dispatch(node, "success", committed_time_s=float(index))
        for index, node in enumerate(composition.mission_graph.nodes, start=1)
    )
    artifact = observed_mission_graph_execution(composition, dispatches).as_dict()

    assert artifact["observation_status"] == "observed"
    assert artifact["completed_nominal_success_path"] is True
    assert artifact["executed_instance_ids"] == artifact["expected_instance_ids"]
    ####


def test_observed_dispatch_rejects_an_edge_not_declared_by_the_graph() -> None:
    composition = _composition("x8_racetrack_capability_3dof_compose.yaml")
    assert composition.mission_graph is not None
    first = composition.mission_graph.nodes[0]
    invalid = GraphExecutionDispatch(
        instance_id=first.instance_id,
        segment_id=first.segment_id,
        outcome="success",
        committed_time_s=1.0,
        transition_status="transitioned",
        next_instance_id="not-a-graph-node",
        state_transfer="previous_terminal_truth_state",
    )

    with pytest.raises(ValueError, match="disagrees"):
        observed_mission_graph_execution(composition, (invalid,))
    ####


def test_observed_dispatch_rejects_reversed_committed_truth_time() -> None:
    composition = _composition("x8_racetrack_capability_3dof_compose.yaml")
    assert composition.mission_graph is not None
    first, second = composition.mission_graph.nodes[:2]
    dispatches = (
        graph_dispatch(first, "success", committed_time_s=2.0),
        graph_dispatch(second, "success", committed_time_s=1.0),
    )

    with pytest.raises(ValueError, match="reverses committed truth time"):
        observed_mission_graph_execution(composition, dispatches)
    ####
