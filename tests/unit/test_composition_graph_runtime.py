"""Tests for family-neutral, fail-closed compiled mission-graph execution."""

from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.composition_graph_runtime import (
    GraphSegmentExecution,
    MissionGraphRuntimeError,
    execute_compiled_mission_graph,
)
from taoryx.vehicle_composition import (
    MissionGraphNodeSelection,
    MissionGraphSelection,
    MissionTransitionSelection,
    compile_vehicle_composition,
    load_vehicle_composition_request,
)

ROOT = Path(__file__).resolve().parents[2]


def _composition(name: str):
    """Compile one checked-in composition fixture."""

    return compile_vehicle_composition(load_vehicle_composition_request(ROOT / "examples/vehicle_composition" / name))
    ####


def test_runtime_chains_only_committed_terminal_truth_state() -> None:
    composition = _composition("x8_racetrack_capability_3dof_compose.yaml")
    assert composition.mission_graph is not None

    def execute(node, state: int) -> GraphSegmentExecution[int, str]:
        return GraphSegmentExecution(node.segment_id, "success", float(state + 1), state + 1, node.instance_id)
        ####

    result = execute_compiled_mission_graph(composition, 0, execute)

    assert result.terminal_state == len(result.steps)
    assert result.evidence.observation_status == "observed"
    assert result.evidence.completed_nominal_success_path is True
    assert tuple(step.execution.payload for step in result.steps) == tuple(node.instance_id for node in composition.mission_graph.nodes)
    assert tuple(step.dispatch.committed_time_s for step in result.steps) == tuple(float(index) for index in range(1, len(result.steps) + 1))
    ####


def test_runtime_requires_a_family_callback_for_declared_physical_handoff() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    instance_ids = tuple(segment.instance_id or f"{index:02d}-{segment.id}" for index, segment in enumerate(request.segments, start=1))
    graph = MissionGraphSelection(
        entry_instance_id=instance_ids[0],
        nodes=tuple(
            MissionGraphNodeSelection(
                instance_id=instance_ids[index],
                success_transition=(
                    None if index + 1 == len(instance_ids) else MissionTransitionSelection(target_instance_id=instance_ids[index + 1])
                ),
                timeout_transition=(
                    MissionTransitionSelection(
                        target_instance_id=instance_ids[1],
                        state_transfer="declared_physical_transition",
                    )
                    if index == 0
                    else None
                ),
            )
            for index in range(len(instance_ids))
        ),
    )
    composition = compile_vehicle_composition(request.model_copy(update={"mission_graph": graph}))

    def timeout_first(node, state: int) -> GraphSegmentExecution[int, None]:
        outcome = "timeout" if node.instance_id == instance_ids[0] else "success"
        return GraphSegmentExecution(node.segment_id, outcome, float(state + 1), state + 1, None)
        ####

    with pytest.raises(MissionGraphRuntimeError, match="physical state-transfer callback"):
        execute_compiled_mission_graph(composition, 0, timeout_first)

    result = execute_compiled_mission_graph(
        composition,
        0,
        timeout_first,
        declared_physical_state_transfer=lambda transition, state: state + 100,
    )

    assert result.steps[0].dispatch.outcome == "timeout"
    assert result.steps[0].dispatch.state_transfer == "declared_physical_transition"
    assert result.terminal_state == len(result.steps) + 100
    assert result.evidence.completed_nominal_success_path is False
    ####


def test_runtime_rejects_an_invalid_committed_truth_time() -> None:
    composition = _composition("x8_racetrack_capability_3dof_compose.yaml")

    def invalid_time(node, state: int) -> GraphSegmentExecution[int, None]:
        return GraphSegmentExecution(node.segment_id, "success", -1.0, state, None)
        ####

    with pytest.raises(MissionGraphRuntimeError, match="invalid committed time"):
        execute_compiled_mission_graph(composition, 0, invalid_time)
    ####
