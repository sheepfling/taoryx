"""Family-neutral execution of a compiled semantic mission graph.

The composition compiler owns graph validity and each family translator owns
which outcomes and state-transfer rules it can support.  This module owns the
small shared runtime operation between them: execute the selected node, select
only its declared edge, and hand the committed terminal state to that edge.
It never creates a fallback, controller, plant, or physical transition.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Generic, TypeVar

from .composition_graph_evidence import (
    GraphExecutionDispatch,
    GraphOutcome,
    MissionGraphExecutionEvidence,
    graph_dispatch,
    observed_mission_graph_execution,
)
from .vehicle_composition import CompiledMissionGraphNode, CompiledMissionTransition, CompiledVehicleComposition

StateT = TypeVar("StateT")
PayloadT = TypeVar("PayloadT")


class MissionGraphRuntimeError(RuntimeError):
    """Fail-closed error from graph dispatch or state transfer."""


@dataclass(frozen=True, slots=True)
class GraphSegmentExecution(Generic[StateT, PayloadT]):
    """One family-owned segment execution at a committed truth boundary."""

    segment_id: str
    outcome: GraphOutcome
    committed_time_s: float
    terminal_state: StateT
    payload: PayloadT


@dataclass(frozen=True, slots=True)
class MissionGraphRuntimeStep(Generic[StateT, PayloadT]):
    """One executed node and its validated declared dispatch."""

    node: CompiledMissionGraphNode
    execution: GraphSegmentExecution[StateT, PayloadT]
    dispatch: GraphExecutionDispatch


@dataclass(frozen=True, slots=True)
class MissionGraphRuntimeResult(Generic[StateT, PayloadT]):
    """Terminal state and machine-verifiable dispatches from one graph run."""

    terminal_state: StateT
    steps: tuple[MissionGraphRuntimeStep[StateT, PayloadT], ...]
    evidence: MissionGraphExecutionEvidence


GraphSegmentExecutor = Callable[[CompiledMissionGraphNode, StateT], GraphSegmentExecution[StateT, PayloadT]]
DeclaredPhysicalStateTransfer = Callable[[CompiledMissionTransition, StateT], StateT]


def execute_compiled_mission_graph(
    composition: CompiledVehicleComposition,
    initial_state: StateT,
    execute_segment: GraphSegmentExecutor[StateT, PayloadT],
    *,
    declared_physical_state_transfer: DeclaredPhysicalStateTransfer[StateT] | None = None,
) -> MissionGraphRuntimeResult[StateT, PayloadT]:
    """Run exactly one compiled graph through family-owned segment callbacks.

    ``previous_terminal_truth_state`` is the only generic transfer because it
    preserves the committed truth boundary exactly.  A
    ``declared_physical_transition`` needs an explicit family callback; this
    prevents a generic runner from inventing a staging, contact, or separation
    transform merely because the graph names one.
    """

    graph = composition.mission_graph
    if graph is None:
        raise MissionGraphRuntimeError("compiled composition has no mission graph")
    nodes = {node.instance_id: node for node in graph.nodes}
    if len(nodes) != len(graph.nodes):
        raise MissionGraphRuntimeError("compiled mission graph contains duplicate instance IDs")
    state = initial_state
    current_instance_id: str | None = graph.entry_instance_id
    executed: set[str] = set()
    steps: list[MissionGraphRuntimeStep[StateT, PayloadT]] = []
    while current_instance_id is not None:
        if current_instance_id in executed:
            raise MissionGraphRuntimeError(f"mission graph attempted to revisit {current_instance_id!r}")
        node = nodes.get(current_instance_id)
        if node is None:
            raise MissionGraphRuntimeError(f"mission graph refers to unknown node {current_instance_id!r}")
        execution = execute_segment(node, state)
        if execution.segment_id != node.segment_id:
            raise MissionGraphRuntimeError(
                f"family segment executor returned {execution.segment_id!r} for graph node {node.instance_id!r}; "
                f"expected {node.segment_id!r}"
            )
        if not math.isfinite(execution.committed_time_s) or execution.committed_time_s < 0.0:
            raise MissionGraphRuntimeError(
                f"family segment executor returned invalid committed time for graph node {node.instance_id!r}"
            )
        dispatch = graph_dispatch(node, execution.outcome, committed_time_s=execution.committed_time_s)
        steps.append(MissionGraphRuntimeStep(node, execution, dispatch))
        executed.add(current_instance_id)
        state = execution.terminal_state
        if dispatch.next_instance_id is None:
            break
        state = _transfer_committed_state(
            dispatch.state_transfer,
            _transition_for_dispatch(node, execution.outcome),
            state,
            declared_physical_state_transfer,
        )
        current_instance_id = dispatch.next_instance_id
    evidence = observed_mission_graph_execution(composition, tuple(step.dispatch for step in steps))
    return MissionGraphRuntimeResult(state, tuple(steps), evidence)
    ####


def _transfer_committed_state(
    state_transfer: str | None,
    transition: CompiledMissionTransition | None,
    state: StateT,
    declared_physical_state_transfer: DeclaredPhysicalStateTransfer[StateT] | None,
) -> StateT:
    """Apply only the state transfer declared on the selected edge."""

    if state_transfer == "previous_terminal_truth_state":
        return state
    if state_transfer == "declared_physical_transition" and transition is not None and declared_physical_state_transfer is not None:
        return declared_physical_state_transfer(transition, state)
    if state_transfer == "declared_physical_transition":
        raise MissionGraphRuntimeError(
            "selected graph edge requires declared_physical_transition, but this family executor supplied no physical state-transfer callback"
        )
    raise MissionGraphRuntimeError(f"selected graph edge has unsupported state transfer {state_transfer!r}")
    ####


def _transition_for_dispatch(
    node: CompiledMissionGraphNode,
    outcome: GraphOutcome,
) -> CompiledMissionTransition | None:
    """Return the edge that produced a dispatch for explicit transfer checks."""

    return {
        "success": node.success_transition,
        "abort": node.abort_transition,
        "resource_limit": node.resource_limit_transition,
        "envelope_limit": node.envelope_limit_transition,
        "timeout": node.timeout_transition,
    }[outcome]
    ####


__all__ = [
    "DeclaredPhysicalStateTransfer",
    "GraphSegmentExecution",
    "GraphSegmentExecutor",
    "MissionGraphRuntimeError",
    "MissionGraphRuntimeResult",
    "MissionGraphRuntimeStep",
    "execute_compiled_mission_graph",
]
