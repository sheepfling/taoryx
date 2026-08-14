"""Source-owned execution of composed NESC two-stage replay missions.

This adapter intentionally runs the retained NESC translation history instead
of inventing a participating rocket plant.  The composition must match the
pinned launch/staging/terminal witness, then the executor emits the common
run-artifact shape with independent source-event and terminal truth checks.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from .composition_control_trace import build_uncontrolled_committed_control_trace, control_trace_summary
from .composition_evaluation import build_composition_trajectory_evaluation
from .composition_graph_evidence import unobserved_mission_graph_execution
from .composition_resource_ledger import build_committed_resource_ledger, resource_ledger_summary
from .composition_sensor_trace import BatchTruthSample
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .deployment import (
    DeploymentBinding,
    DeploymentChildExecution,
    DeploymentReleaseRequest,
    DeploymentReleaseState,
    deployment_binding_for,
    execute_deployment_child,
)
from .mission_objectives import ControllerTransition, TruthObjectiveSpec, evaluate_truth_objectives
from .nesc_mission_translation import NescReplayMissionPlan, compile_nesc_source_replay_mission
from .trajectory.nesc_pseudo6dof import build_nesc_composite_pseudo6dof
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import VehicleExecutionPreflight, preflight_vehicle_composition

if TYPE_CHECKING:
    from .plugins import PluginCatalog

_EARTH_EQUATORIAL_RADIUS_M = 6_378_137.0
_NESC_PARENT_PLUGIN_ID = "taoryx.nesc"
_NESC_PARENT_OBJECT_ID = "nesc-primary"
_NESC_STAGE_DEPLOYMENT_ID = "stage_separation_lineage"
_NESC_STAGE_RELEASE_EVENT_ID = "stage_separation"
_NESC_PASSIVE_CHILD_PLUGIN_ID = "taoryx.passive-bodies"
_NESC_PASSIVE_CHILD_RUNTIME_ID = "taoryx.passive-bodies.local-atmosphere-release.v1"
_NESC_PASSIVE_CHILD_MODEL_ID = "nesc-synthetic-cylinder"
_NESC_PASSIVE_CHILD_FAMILY_ID = "tumbling_body"


@dataclass(frozen=True, slots=True)
class NescReplayCompositionExecution:
    """One immutable source-replay execution artifact."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: NescReplayMissionPlan
    output_dir: Path
    runtime: dict[str, object]
    envelope: dict[str, object]
    truth_evaluation: dict[str, object]
    controller_transitions: tuple[dict[str, object], ...]
    status_trace: dict[str, object]
    semantic_action_trace: dict[str, object]
    deployment_children: tuple[DeploymentChildExecution, ...]
    claim_boundary: str

    @property
    def parent_mission_pass(self) -> bool:
        """Return the parent replay verdict before optional child execution."""

        transitions_valid = all(item["reason"] == "EVENT_COMPLETE" for item in self.controller_transitions)
        return bool(self.runtime["hard_gates_passed"]) and bool(self.envelope["pass"]) and bool(self.truth_evaluation["mission_pass"]) and transitions_valid
        ####

    @property
    def deployment_pass(self) -> bool:
        """Return whether every explicitly selected child completed numerically."""

        return all(item.status == "completed" for item in self.deployment_children)
        ####

    @property
    def mission_pass(self) -> bool:
        """Return the selected composition verdict without promoting the parent claim."""

        return self.parent_mission_pass and self.deployment_pass
        ####

    def as_dict(self) -> dict[str, object]:
        """Return one compact, self-contained public execution report."""

        return {
            "schema": "taoryx.nesc-source-replay-composition-execution/v1alpha1",
            "status": (
                "nominal_source_replay_pass_with_independent_child"
                if self.mission_pass and self.deployment_children
                else "nominal_source_replay_pass"
                if self.mission_pass
                else "source_replay_or_selected_child_failed"
            ),
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "plan": self.plan.manifest(),
            "output_dir": str(self.output_dir),
            "runtime": self.runtime,
            "envelope": self.envelope,
            "truth_evaluation": self.truth_evaluation,
            "controller_transitions": list(self.controller_transitions),
            "status_trace": status_trace_summary(self.status_trace),
            "semantic_action_trace": control_trace_summary(self.semantic_action_trace),
            "deployment_children": [item.model_dump(mode="json", by_alias=True) for item in self.deployment_children],
            "entity_relationships": _deployment_relationships(self.deployment_children),
            "parent_mission_pass": self.parent_mission_pass,
            "deployment_pass": self.deployment_pass,
            "mission_pass": self.mission_pass,
            "claim_boundary": self.claim_boundary,
        }
        ####

    ####


def execute_nesc_source_replay_composition(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    *,
    plugins: PluginCatalog | None = None,
) -> NescReplayCompositionExecution:
    """Run the exact NESC replay declared by one source-compatible composition."""

    preflight = preflight_vehicle_composition(composition, plugins=plugins)
    if preflight.status != "translation_ready":
        details = "; ".join(preflight.diagnostics) or "no NESC source replay translator"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {details}")
    plan = compile_nesc_source_replay_mission(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    rows, source_checks, provenance = _replay_rows(plan)
    events = _truth_events(plan, rows)
    deployment_children = _execute_selected_deployments(composition, rows, events, plugins=plugins)
    transitions = _source_transitions(plan, events)
    finite = _finite_rows(rows)
    envelope = _envelope_report(rows)
    truth_evaluation = evaluate_truth_objectives(
        _objective_specs(plan, rows),
        rows,
        controller_transitions=tuple(
            ControllerTransition(
                objective_id=str(item["objective_id"]),
                time_s=_number(item, "time_s"),
                reason="EVENT_COMPLETE",
                source="source_replay_phase",
            )
            for item in transitions
        ),
        truth_events=frozenset(events),
        truth_event_times=events,
        hard_gates_passed=finite and bool(envelope["pass"]) and all(source_checks.values()),
    )
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.rocket.variable_mass_nesc.v1",
        "execution_mode": "pinned_source_history_replay",
        "control_realization": "uncontrolled_source_replay" if composition.fidelity == "point_mass_3dof" else "response_law",
        "physical_gimbal_allocation": False,
        "participating_nonlinear_plant": False,
        "source_checks": source_checks,
        "numerical_valid": finite,
        "duration_s": _number(rows[-1], "time_s") if rows else 0.0,
        "hard_gates_passed": finite and bool(envelope["pass"]) and all(source_checks.values()),
    }
    mission_graph_execution = unobserved_mission_graph_execution(
        composition,
        "The pinned source-history replay records source phase evidence but does not emit committed controller graph dispatches.",
    ).as_dict()
    runtime["mission_graph_execution"] = mission_graph_execution
    status_trace = build_committed_status_trace(composition, _status_samples(rows), plugins=plugins)
    semantic_action_trace = build_uncontrolled_committed_control_trace(
        composition,
        tuple(_number(row, "time_s") for row in rows),
        plugins=plugins,
    )
    resource_ledger = build_committed_resource_ledger(composition, status_trace, plugins=plugins)
    runtime["resource_ledger"] = resource_ledger_summary(resource_ledger)
    runtime["semantic_action_trace"] = control_trace_summary(semantic_action_trace)
    runtime["deployment_children"] = _deployment_runtime_summary(deployment_children)
    runtime["entity_relationships"] = _deployment_relationships(deployment_children)
    runtime["deployment_status"] = (
        "not_selected" if not deployment_children else "completed" if all(item.status == "completed" for item in deployment_children) else "failed"
    )
    result = NescReplayCompositionExecution(
        composition=composition,
        preflight=preflight,
        plan=plan,
        output_dir=destination,
        runtime=runtime,
        envelope=envelope,
        truth_evaluation=truth_evaluation,
        controller_transitions=tuple(transitions),
        status_trace=status_trace,
        semantic_action_trace=semantic_action_trace,
        deployment_children=deployment_children,
        claim_boundary=(
            "This is a source-owned NESC translation-history replay with optional named pseudo-6DOF attitude "
            "response. It proves the pinned source sequence, resource history, and endpoint only. It does not "
            "establish a participating nonlinear rocket plant, a pitch program, guidance, gimbal allocation, "
            "stage-separation dynamics, or physical control authority. A selected child is a separate synthetic "
            "local-atmosphere propagation with its own receipt and does not alter the parent source-replay claim."
        ),
    )
    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "mission_graph_execution.json", mission_graph_execution)
    _write_json(destination / "envelope_report.json", envelope)
    _write_json(destination / "controller_transitions.json", list(transitions))
    _write_json(destination / "objective_report.json", truth_evaluation)
    _write_json(destination / "source_provenance.json", provenance)
    _write_json(destination / "status_trace.json", status_trace)
    _write_json(destination / "semantic_action_trace.json", semantic_action_trace)
    _write_json(destination / "resource_ledger.json", resource_ledger)
    _write_json(
        destination / "deployment_children.json",
        [item.model_dump(mode="json", by_alias=True) for item in deployment_children],
    )
    _write_json(destination / "entity_relationships.json", _deployment_relationships(deployment_children))
    _write_deployment_children(destination, deployment_children)
    _write_json(
        destination / "evaluation.json",
        build_composition_trajectory_evaluation(
            composition,
            preflight,
            truth_evaluation,
            runtime=runtime,
            envelope=envelope,
            claim_boundary=result.claim_boundary,
            status_trace=status_trace,
            plugins=plugins,
        ).as_dict(),
    )
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _execute_selected_deployments(
    composition: CompiledVehicleComposition,
    rows: list[dict[str, object]],
    events: dict[str, float],
    *,
    plugins: PluginCatalog | None = None,
) -> tuple[DeploymentChildExecution, ...]:
    """Execute the one explicitly selected NESC stage-separation child, if any.

    The source replay remains a parent-only artifact unless a composition
    selects a child binding.  This avoids treating event lineage as permission
    to invent a released-body trajectory.
    """

    unsupported = tuple(item for item in composition.deployment_bindings if item.deployment_id != _NESC_STAGE_DEPLOYMENT_ID)
    if unsupported:
        raise ValueError(f"NESC source replay does not advertise composition deployment binding(s): {', '.join(item.deployment_id for item in unsupported)}")
    binding = deployment_binding_for(composition.deployment_bindings, _NESC_STAGE_DEPLOYMENT_ID)
    if binding is None:
        return ()
    _validate_nesc_passive_child_binding(binding)
    release_time_s = events[_NESC_STAGE_RELEASE_EVENT_ID]
    release_row = next(
        (row for row in rows if math.isclose(_number(row, "time_s"), release_time_s, rel_tol=0.0, abs_tol=1.0e-9) and str(row["phase"]) == "stack_coast"),
        None,
    )
    if release_row is None:
        raise ValueError("NESC source replay has no accepted stack-coast row for the declared separation event")
    release_state = _local_tangent_release_state(release_row)
    request = DeploymentReleaseRequest(
        binding=binding,
        parent_plugin_id=_NESC_PARENT_PLUGIN_ID,
        parent_model_id="reference_nesc_two_stage_rocket",
        parent_family_id="reference_nesc_two_stage_rocket",
        parent_composition_id=composition.id,
        parent_object_id=_NESC_PARENT_OBJECT_ID,
        release_event_id=_NESC_STAGE_RELEASE_EVENT_ID,
        release_state=release_state,
        parent_claim_boundary=(
            "The parent provides only the accepted source-replay kinematic state at stage separation. "
            "It does not provide an impulse, detached-stage geometry, or source-exact child aerodynamics."
        ),
    )
    return (execute_deployment_child(request, plugins=plugins),)
    ####


def _validate_nesc_passive_child_binding(binding: DeploymentBinding) -> None:
    """Reject a selection that disagrees with NESC's published child contract."""

    if binding.release_event_id != _NESC_STAGE_RELEASE_EVENT_ID:
        raise ValueError(f"NESC deployment {_NESC_STAGE_DEPLOYMENT_ID!r} must bind event {_NESC_STAGE_RELEASE_EVENT_ID!r}, not {binding.release_event_id!r}")
    if binding.state_transfer != "source_replay_kinematic_projection":
        raise ValueError("NESC source replay requires state_transfer='source_replay_kinematic_projection' for an independent child")
    expected_child = (
        _NESC_PASSIVE_CHILD_PLUGIN_ID,
        _NESC_PASSIVE_CHILD_RUNTIME_ID,
        _NESC_PASSIVE_CHILD_MODEL_ID,
        _NESC_PASSIVE_CHILD_FAMILY_ID,
    )
    selected_child = (
        binding.child.plugin_id,
        binding.child.runtime_id,
        binding.child.model_id,
        binding.child.family_id,
    )
    if selected_child != expected_child:
        raise ValueError(f"NESC source replay requires the advertised passive child {expected_child!r}, not {selected_child!r}")
    if binding.child.fidelity not in {"point_mass_3dof", "pseudo_6dof"}:
        raise ValueError(f"NESC's passive child supports only point_mass_3dof or pseudo_6dof, not {binding.child.fidelity!r}")
    ####


def _local_tangent_release_state(row: dict[str, object]) -> DeploymentReleaseState:
    """Project an accepted NESC ECI row into a declared local child-state seam.

    This is a deliberately limited kinematic projection.  It preserves the
    source radial altitude, radial velocity, and tangential speed at the event,
    while making no claim that subsequent local atmospheric propagation is an
    ECI or rotating-Earth continuation.
    """

    position_eci_m = _vector(row, "position_eci_m")
    velocity_eci_m_s = _vector(row, "velocity_eci_mps")
    radius_m = _norm(position_eci_m)
    if radius_m <= _EARTH_EQUATORIAL_RADIUS_M:
        raise ValueError("NESC stage-separation source row is not above the local atmosphere projection reference radius")
    radial_unit = (
        position_eci_m[0] / radius_m,
        position_eci_m[1] / radius_m,
        position_eci_m[2] / radius_m,
    )
    radial_velocity_m_s = sum(value * direction for value, direction in zip(velocity_eci_m_s, radial_unit, strict=True))
    tangential_velocity = (
        velocity_eci_m_s[0] - radial_velocity_m_s * radial_unit[0],
        velocity_eci_m_s[1] - radial_velocity_m_s * radial_unit[1],
        velocity_eci_m_s[2] - radial_velocity_m_s * radial_unit[2],
    )
    tangential_speed_m_s = _norm(tangential_velocity)
    if tangential_speed_m_s <= 0.0:
        raise ValueError("NESC stage-separation source row has no tangential velocity for local projection")
    return DeploymentReleaseState(
        frame="local_tangent",
        time_s=_number(row, "time_s"),
        position_m=(0.0, 0.0, radius_m - _EARTH_EQUATORIAL_RADIUS_M),
        velocity_m_s=(tangential_speed_m_s, 0.0, radial_velocity_m_s),
        provenance={
            "state_transfer": "source_replay_kinematic_projection",
            "projection_id": "nesc_eci_to_local_tangent_speed_projection_v1",
            "source_phase": str(row["phase"]),
            "source_position_eci_m": list(position_eci_m),
            "source_velocity_eci_mps": list(velocity_eci_m_s),
            "earth_reference_radius_m": _EARTH_EQUATORIAL_RADIUS_M,
            "radial_velocity_m_s": radial_velocity_m_s,
            "tangential_speed_m_s": tangential_speed_m_s,
            "claim_boundary": (
                "The local position uses an event-local tangent origin and radial altitude; local x is aligned to "
                "the instantaneous ECI tangential velocity. No ECI continuation, Earth rotation, child impulse, or "
                "source-exact detached-stage attitude is inferred."
            ),
        },
    )
    ####


def _deployment_runtime_summary(children: tuple[DeploymentChildExecution, ...]) -> list[dict[str, object]]:
    """Return compact child receipt facts alongside the parent runtime report."""

    return [
        {
            "binding_id": item.binding_id,
            "deployment_id": item.deployment_id,
            "state_transfer": item.state_transfer,
            "parent_plugin_id": item.parent_plugin_id,
            "parent_model_id": item.parent_model_id,
            "parent_family_id": item.parent_family_id,
            "parent_composition_id": item.parent_composition_id,
            "child_object_id": item.child_object_id,
            "runtime_plugin_id": item.runtime_plugin_id,
            "runtime_id": item.runtime_id,
            "status": item.status,
            "termination": item.termination,
            "telemetry_sample_count": len(item.telemetry),
        }
        for item in children
    ]
    ####


def _deployment_relationships(children: tuple[DeploymentChildExecution, ...]) -> list[dict[str, object]]:
    """Expose independent child lineage through the common relationship vocabulary."""

    return [
        {
            "kind": item.relationship_kind,
            "parent_object_id": item.parent_object_id,
            "child_object_id": item.child_object_id,
            "deployment_id": item.deployment_id,
            "event_id": item.parent_event_id,
            "state_transfer": item.state_transfer,
            "spawn_time_s": item.release_state.time_s,
        }
        for item in children
    ]
    ####


def _write_deployment_children(destination: Path, children: tuple[DeploymentChildExecution, ...]) -> None:
    """Write child receipts and telemetry beneath the parent execution output."""

    if not children:
        return
    child_root = destination / "deployment_children"
    child_root.mkdir(parents=True, exist_ok=True)
    for index, child in enumerate(children, start=1):
        safe_id = "".join(character if character.isalnum() or character in "-_" else "_" for character in child.child_object_id)
        child_directory = child_root / f"{index:02d}-{safe_id}"
        child_directory.mkdir()
        _write_json(child_directory / "execution.json", child.model_dump(mode="json", by_alias=True))
        _write_csv(child_directory / "truth_telemetry.csv", [dict(item) for item in child.telemetry])
    ####


def _replay_rows(
    plan: NescReplayMissionPlan,
) -> tuple[list[dict[str, object]], dict[str, bool], dict[str, object]]:
    """Load point-mass or pseudo response rows from the same pinned artifact."""

    if plan.fidelity == "pseudo_6dof":
        replay = build_nesc_composite_pseudo6dof(plan.source_artifact)
        raw_rows = [dict(row) for row in replay.rows]
        checks: dict[str, bool] = {name: passed for name, passed in replay.checks}
        provenance: dict[str, object] = {
            "source_artifact": str(plan.source_artifact),
            "response_profile_id": replay.profile_id,
            "source_checks": checks,
            "claim_boundary": "translation is retained source replay; attitude is a named response-law surrogate",
        }
    else:
        payload = json.loads(plan.source_artifact.read_text(encoding="utf-8"))
        history = payload.get("history") if isinstance(payload, dict) else None
        if not isinstance(history, list) or not history or not all(isinstance(item, dict) for item in history):
            raise ValueError("NESC source replay artifact has no valid history")
        raw_rows = [dict(item) for item in history]
        source = payload.get("checks", {})
        checks = {str(name): bool(value) for name, value in source.items()} if isinstance(source, dict) else {}
        provenance = {
            "source_artifact": str(plan.source_artifact),
            "response_profile_id": "not_applicable_point_mass",
            "source_checks": checks,
            "claim_boundary": "translation is retained source replay; no attitude, gimbal, or controller model is active",
        }
    rows = [_truth_row(row) for row in raw_rows]
    if not checks:
        checks = {"source_rows_retained": bool(rows)}
    return rows, checks, provenance
    ####


def _truth_row(row: dict[str, object]) -> dict[str, object]:
    """Normalize source/reduction rows without changing their reported truth."""

    position = _vector(row, "replay_position_eci_m", fallback="position_eci_m")
    velocity = _vector(row, "replay_velocity_eci_mps", fallback="velocity_eci_mps")
    source_position = _vector(row, "source_position_eci_m", fallback="position_eci_m")
    source_velocity = _vector(row, "source_velocity_eci_mps", fallback="velocity_eci_mps")
    result = dict(row)
    result["time_s"] = _number(row, "time_s")
    result["position_eci_m"] = list(position)
    result["velocity_eci_mps"] = list(velocity)
    result["position_eci_x_m"] = position[0]
    result["position_eci_y_m"] = position[1]
    result["position_eci_z_m"] = position[2]
    result["speed_m_s"] = _norm(velocity)
    result["mass_kg"] = _number(row, "mass_kg")
    result["position_error_m"] = _norm(_difference(position, source_position))
    result["velocity_error_mps"] = _norm(_difference(velocity, source_velocity))
    result["mass_error_kg"] = _optional_number(row.get("mass_error_kg"), default=0.0)
    result["phase"] = str(row["phase"])
    return result
    ####


def _status_samples(rows: list[dict[str, object]]) -> tuple[BatchTruthSample, ...]:
    """Expose retained replay rows through the same committed-status seam."""

    return tuple(
        BatchTruthSample(
            time_s=_number(row, "time_s"),
            raw_values=row,
            execution_status="completed" if index == len(rows) - 1 else "active",
        )
        for index, row in enumerate(rows)
    )
    ####


def _truth_events(plan: NescReplayMissionPlan, rows: list[dict[str, object]]) -> dict[str, float]:
    """Derive ordered physical phase events directly from replay truth rows."""

    starts = _phase_start_times(rows)
    required = {event for segment in plan.segments for event in segment.required_truth_events}
    events = {
        "liftoff_stage1_ignition": starts["stage1_burn"],
        "stage1_cutoff": starts["stack_coast"],
        "stage_separation": starts["stack_coast"],
        "upper_stage_ignition": starts["stage2_burn"],
        "stage2_cutoff": starts["orbit_coast"],
        "nominal_endpoint": _number(rows[-1], "time_s"),
    }
    missing = sorted(required - set(events))
    if missing:
        raise ValueError(f"NESC replay cannot derive declared truth event(s): {', '.join(missing)}")
    return events
    ####


def _source_transitions(
    plan: NescReplayMissionPlan,
    events: dict[str, float],
) -> list[dict[str, object]]:
    """Retain replay phase completion as diagnostic-only transition records."""

    transitions: list[dict[str, object]] = []
    for segment in plan.segments:
        completion = segment.required_truth_events[-1]
        transitions.append(
            {
                "objective_id": segment.instance_id,
                "segment_instance_id": segment.instance_id,
                "segment_id": segment.segment_id,
                "time_s": events[completion],
                "reason": "EVENT_COMPLETE",
                "source": "source_replay_phase",
                "event": completion,
            }
        )
    return transitions
    ####


def _objective_specs(
    plan: NescReplayMissionPlan,
    rows: list[dict[str, object]],
) -> tuple[TruthObjectiveSpec, ...]:
    """Build ordered replay-event and terminal objectives from source truth."""

    event_specs = tuple(
        TruthObjectiveSpec(
            id=segment.instance_id,
            objective_type="event",
            event_id=segment.required_truth_events[-1],
        )
        for segment in plan.segments[:-1]
    )
    final = TruthObjectiveSpec(
        id=plan.segments[-1].instance_id,
        objective_type="terminal_state_gate",
        target={
            "mass_error_kg": 0.0,
            "velocity_error_mps": 0.0,
            "position_error_m": 0.0,
        },
        tolerance={
            "mass_error_kg": 1.0e-6,
            "velocity_error_mps": 1.0e-6,
            "position_error_m": 0.2,
        },
        window_start_s=plan.segments[-1].start_time_s,
    )
    return (*event_specs, final)
    ####


def _envelope_report(rows: list[dict[str, object]]) -> dict[str, object]:
    """Check only source-replay finiteness and monotonic resource bounds."""

    mass_values = [_number(row, "mass_kg") for row in rows]
    phase_starts = _phase_start_times(rows)
    ordered_phases = tuple(phase_starts) == ("stage1_burn", "stack_coast", "stage2_burn", "orbit_coast")
    finite = _finite_rows(rows)
    mass_monotone = all(right <= left + 1.0e-9 for left, right in zip(mass_values, mass_values[1:]))
    maximum_position_error = max((_number(row, "position_error_m") for row in rows), default=math.inf)
    replay_position_margin = 0.2 - maximum_position_error
    return {
        "pass": (bool(rows) and finite and min(mass_values, default=-1.0) > 0.0 and mass_monotone and ordered_phases and replay_position_margin >= 0.0),
        "minimum_margins": {
            "mass_kg": min(mass_values, default=-1.0),
            "replay_position_error_margin_m": replay_position_margin,
        },
        "phase_order": list(phase_starts),
        "claim_boundary": (
            "This is a source-replay data-integrity envelope. The 0.2 m replay-position limit preserves the "
            "retained reduction witness resolution; it is not a propulsion, load, or gimbal authority envelope."
        ),
    }
    ####


def _phase_start_times(rows: list[dict[str, object]]) -> dict[str, float]:
    """Return source order and first truth timestamp for each phase."""

    starts: dict[str, float] = {}
    for row in rows:
        phase = str(row["phase"])
        starts.setdefault(phase, _number(row, "time_s"))
    return starts
    ####


def _finite_rows(rows: list[dict[str, object]]) -> bool:
    """Reject missing or non-finite replay truth before evaluation."""

    try:
        return bool(rows) and all(
            math.isfinite(value)
            for row in rows
            for value in (
                _number(row, "time_s"),
                _number(row, "mass_kg"),
                _number(row, "speed_m_s"),
                *_vector(row, "position_eci_m"),
                *_vector(row, "velocity_eci_mps"),
            )
        )
    except (KeyError, TypeError, ValueError):
        return False
    ####


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    """Write normalized truth telemetry with deterministic field order."""

    keys = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _csv_value(row.get(key)) for key in keys})
    ####


def _csv_value(value: object) -> object:
    return json.dumps(value, separators=(",", ":")) if isinstance(value, list | dict) else value
    ####


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _number(row: dict[str, object], name: str) -> float:
    return _optional_number(row.get(name), name=name)
    ####


def _optional_number(value: object, *, name: str = "value", default: float | None = None) -> float:
    """Return one finite telemetry number or a declared default."""

    if value is None and default is not None:
        return default
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError(f"NESC telemetry value {name!r} must be finite numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"NESC telemetry value {name!r} must be finite numeric")
    return result
    ####


def _vector(row: dict[str, object], name: str, *, fallback: str | None = None) -> tuple[float, float, float]:
    value = row.get(name, row.get(fallback) if fallback is not None else None)
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"NESC telemetry vector {name!r} must contain three values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"NESC telemetry vector {name!r} must be finite")
    return result  # type: ignore[return-value]
    ####


def _norm(value: tuple[float, float, float]) -> float:
    return math.sqrt(sum(component * component for component in value))
    ####


def _difference(
    value: tuple[float, float, float],
    reference: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return the componentwise difference between two three-vectors."""

    return (
        value[0] - reference[0],
        value[1] - reference[1],
        value[2] - reference[2],
    )
    ####


__all__ = ["NescReplayCompositionExecution", "execute_nesc_source_replay_composition"]
