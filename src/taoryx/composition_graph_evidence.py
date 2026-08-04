"""Canonical graph-execution evidence for composed vehicle runs.

Mission graph compilation and mission execution are separate claims.  This
module records the distinction in one portable artifact: an executor either
observed named graph dispatches at committed truth boundaries or explicitly
did not emit graph-transition evidence.  A successful truth-objective result
must never be reinterpreted as observed controller/graph progress.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from .vehicle_composition import (
    CompiledMissionGraph,
    CompiledMissionGraphNode,
    CompiledMissionTransition,
    CompiledVehicleComposition,
)

GraphOutcome = Literal["success", "abort", "resource_limit", "envelope_limit", "timeout"]
GraphObservationStatus = Literal["observed", "unobserved"]


@dataclass(frozen=True, slots=True)
class GraphExecutionDispatch:
    """One executor-observed outcome and selected graph transition."""

    instance_id: str
    segment_id: str
    outcome: GraphOutcome
    committed_time_s: float
    transition_status: Literal["transitioned", "declared_terminal", "no_declared_transition", "unhandled_outcome"]
    next_instance_id: str | None
    state_transfer: str | None

    def as_dict(self) -> dict[str, object]:
        """Return one machine-readable committed-boundary dispatch record."""

        return {
            "instance_id": self.instance_id,
            "segment_id": self.segment_id,
            "outcome": self.outcome,
            "committed_time_s": self.committed_time_s,
            "transition_status": self.transition_status,
            "next_instance_id": self.next_instance_id,
            "state_transfer": self.state_transfer,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class MissionGraphExecutionEvidence:
    """One honest execution-evidence envelope for a compiled mission graph."""

    composition_id: str
    composition_identity_sha256: str
    graph_status: str
    entry_instance_id: str
    observation_status: GraphObservationStatus
    expected_instance_ids: tuple[str, ...]
    dispatches: tuple[GraphExecutionDispatch, ...]
    completed_nominal_success_path: bool | None
    unobserved_reason: str | None

    def as_dict(self) -> dict[str, object]:
        """Return the stable graph-execution artifact format."""

        return {
            "schema": "taoryx.mission-graph-execution/v1alpha1",
            "composition_id": self.composition_id,
            "composition_identity_sha256": self.composition_identity_sha256,
            "graph_status": self.graph_status,
            "entry_instance_id": self.entry_instance_id,
            "observation_status": self.observation_status,
            "expected_instance_ids": list(self.expected_instance_ids),
            "executed_instance_ids": [item.instance_id for item in self.dispatches],
            "dispatches": [item.as_dict() for item in self.dispatches],
            "completed_nominal_success_path": self.completed_nominal_success_path,
            "unobserved_reason": self.unobserved_reason,
            "claim_boundary": (
                "Observed graph dispatches are executor evidence only and do not replace independent truth-objective "
                "evaluation. An unobserved graph record explicitly does not claim controller sequencing or transition "
                "execution merely because a trajectory or terminal objective passed."
            ),
        }
        ####
    ####


def graph_dispatch(
    node: CompiledMissionGraphNode,
    outcome: GraphOutcome,
    *,
    committed_time_s: float,
) -> GraphExecutionDispatch:
    """Resolve one declared graph edge and label an unhandled outcome honestly."""

    if not math.isfinite(committed_time_s) or committed_time_s < 0.0:
        raise ValueError("graph dispatch committed time must be finite and nonnegative")
    transition = _transition_for_outcome(node, outcome)
    if transition is None:
        return GraphExecutionDispatch(
            instance_id=node.instance_id,
            segment_id=node.segment_id,
            outcome=outcome,
            committed_time_s=committed_time_s,
            transition_status="declared_terminal" if outcome == "success" else "no_declared_transition",
            next_instance_id=None,
            state_transfer=None,
        )
    return GraphExecutionDispatch(
        instance_id=node.instance_id,
        segment_id=node.segment_id,
        outcome=outcome,
        committed_time_s=committed_time_s,
        transition_status="transitioned",
        next_instance_id=transition.target_instance_id,
        state_transfer=transition.state_transfer,
    )
    ####


def observed_mission_graph_execution(
    composition: CompiledVehicleComposition,
    dispatches: tuple[GraphExecutionDispatch, ...],
) -> MissionGraphExecutionEvidence:
    """Build a validated observed-dispatch artifact for one family executor."""

    graph = _graph(composition)
    nodes = {node.instance_id: node for node in graph.nodes}
    if len(nodes) != len(graph.nodes):
        raise ValueError("compiled graph has duplicate node IDs")
    seen: set[str] = set()
    previous: GraphExecutionDispatch | None = None
    for dispatch in dispatches:
        node = nodes.get(dispatch.instance_id)
        if node is None or node.segment_id != dispatch.segment_id:
            raise ValueError(f"graph dispatch references unknown or mismatched instance {dispatch.instance_id!r}")
        if dispatch.instance_id in seen:
            raise ValueError(f"graph dispatch revisits instance {dispatch.instance_id!r}")
        if not math.isfinite(dispatch.committed_time_s) or dispatch.committed_time_s < 0.0:
            raise ValueError(f"graph dispatch for {dispatch.instance_id!r} has invalid committed time")
        if previous is not None and dispatch.committed_time_s + 1.0e-12 < previous.committed_time_s:
            raise ValueError(f"graph dispatch for {dispatch.instance_id!r} reverses committed truth time")
        if previous is not None and previous.next_instance_id != dispatch.instance_id:
            raise ValueError(
                f"graph dispatch for {dispatch.instance_id!r} does not follow prior selected target {previous.next_instance_id!r}"
            )
        expected = graph_dispatch(node, dispatch.outcome, committed_time_s=dispatch.committed_time_s)
        if dispatch != expected:
            raise ValueError(f"graph dispatch for {dispatch.instance_id!r} disagrees with its declared {dispatch.outcome!r} edge")
        seen.add(dispatch.instance_id)
        previous = dispatch
    completed_success = bool(dispatches) and all(item.outcome == "success" for item in dispatches)
    if completed_success:
        expected_order = tuple(node.instance_id for node in graph.nodes)
        completed_success = tuple(item.instance_id for item in dispatches) == expected_order
    return MissionGraphExecutionEvidence(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        graph_status=graph.status,
        entry_instance_id=graph.entry_instance_id,
        observation_status="observed",
        expected_instance_ids=tuple(node.instance_id for node in graph.nodes),
        dispatches=dispatches,
        completed_nominal_success_path=completed_success,
        unobserved_reason=None,
    )
    ####


def unobserved_mission_graph_execution(
    composition: CompiledVehicleComposition,
    reason: str,
) -> MissionGraphExecutionEvidence:
    """Record that an executor supplied no graph-transition evidence."""

    if not reason.strip():
        raise ValueError("unobserved graph evidence requires a nonempty reason")
    graph = _graph(composition)
    return MissionGraphExecutionEvidence(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        graph_status=graph.status,
        entry_instance_id=graph.entry_instance_id,
        observation_status="unobserved",
        expected_instance_ids=tuple(node.instance_id for node in graph.nodes),
        dispatches=(),
        completed_nominal_success_path=None,
        unobserved_reason=reason,
    )
    ####


def _graph(composition: CompiledVehicleComposition) -> CompiledMissionGraph:
    """Return the mandatory compiled graph with one fail-closed diagnostic."""

    if composition.mission_graph is None:
        raise ValueError("compiled composition has no mission graph")
    return composition.mission_graph
    ####


def _transition_for_outcome(
    node: CompiledMissionGraphNode,
    outcome: GraphOutcome,
) -> CompiledMissionTransition | None:
    """Return the declared edge for one typed executor outcome."""

    return {
        "success": node.success_transition,
        "abort": node.abort_transition,
        "resource_limit": node.resource_limit_transition,
        "envelope_limit": node.envelope_limit_transition,
        "timeout": node.timeout_transition,
    }[outcome]
    ####


__all__ = [
    "GraphExecutionDispatch",
    "GraphObservationStatus",
    "GraphOutcome",
    "MissionGraphExecutionEvidence",
    "graph_dispatch",
    "observed_mission_graph_execution",
    "unobserved_mission_graph_execution",
]
