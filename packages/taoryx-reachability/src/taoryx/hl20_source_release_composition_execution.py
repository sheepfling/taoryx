"""Source-owned batch execution for the HL-20 source-scheduled release witness.

The runner is intentionally narrow: it exposes the existing source-aerodynamic
plus synthetic-booster simulation under one exact composition.  The retained
bank schedule is reported as source configuration, not as an action trace or
as evidence of a guidance controller.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

from .composition_control_trace import build_uncontrolled_committed_control_trace, control_trace_summary
from .composition_evaluation import build_composition_trajectory_evaluation
from .composition_graph_evidence import unobserved_mission_graph_execution
from .composition_resource_ledger import build_committed_resource_ledger, resource_ledger_summary
from .composition_sensor_trace import BatchTruthSample
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .hl20_reachability import (
    HL20_AERODYNAMICS_SHA256,
    HL20_FIXED_MASS_KG,
    HL20_PACKAGE_SHA256,
    run_hl20_source_release,
)
from .hl20_source_release_mission_translation import (
    HL20SourceReleaseMissionPlan,
    compile_hl20_source_booster_release_mission,
)
from .mission_objectives import ControllerTransition, TruthObjectiveSpec, evaluate_truth_objectives
from .reachability_aerodynamics import HL20_SOURCE_MODEL_ID
from .reachability_envelope import ReachabilityFidelity, TrajectoryResult
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import VehicleExecutionPreflight, preflight_vehicle_composition


@dataclass(frozen=True, slots=True)
class HL20SourceReleaseCompositionExecution:
    """One composition-bound source-scheduled HL-20 release artifact."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: HL20SourceReleaseMissionPlan
    trajectory: TrajectoryResult
    output_dir: Path
    runtime: dict[str, object]
    envelope: dict[str, object]
    truth_evaluation: dict[str, object]
    controller_transitions: tuple[dict[str, object], ...]
    status_trace: dict[str, object]
    semantic_action_trace: dict[str, object]

    @property
    def mission_pass(self) -> bool:
        return (
            bool(self.runtime["hard_gates_passed"])
            and bool(self.envelope["pass"])
            and bool(self.truth_evaluation["mission_pass"])
            and all(item["reason"] == "EVENT_COMPLETE" for item in self.controller_transitions)
        )
        ####

    def as_dict(self) -> dict[str, object]:
        return {
            "schema": "taoryx.hl20-source-release-composition-execution/v1alpha1",
            "status": "nominal_source_scheduled_pass" if self.mission_pass else "source_scheduled_failed",
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
            "mission_pass": self.mission_pass,
            "claim_boundary": (
                "This executes the fixed source-aerodynamic HL-20 booster/release and scheduled-bank witness. "
                "It does not establish user-driven guidance, trim, physical control-surface allocation, a "
                "closed-loop controller, or the high-altitude glide-energy mission."
            ),
        }
        ####
    ####


def execute_hl20_source_booster_release_composition(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
) -> HL20SourceReleaseCompositionExecution:
    """Execute the exact retained HL-20 source-release composition."""

    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {'; '.join(preflight.diagnostics)}")
    plan = compile_hl20_source_booster_release_mission(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    fidelity = (
        ReachabilityFidelity.POINT_MASS_3DOF
        if composition.fidelity == "point_mass_3dof"
        else ReachabilityFidelity.PSEUDO_6DOF
    )
    envelope_result = run_hl20_source_release(
        fidelity=fidelity,
        step_size_s=0.5,
        horizon_s=120.0,
        spawn_children=True,
    )
    if not envelope_result.samples:
        raise ValueError("HL-20 source release runner produced no envelope samples")
    sample = envelope_result.samples[0]
    trajectory = sample.trajectory
    rows = [_truth_row(state) for state in trajectory.states]
    events = _truth_events(trajectory, rows)
    transitions = _source_transitions(plan, events)
    finite = _finite_rows(rows)
    ground_contact = str(getattr(trajectory.termination, "value", trajectory.termination)) == "ground_contact"
    envelope = {
        "pass": finite and ground_contact and bool(sample.feasible),
        "minimum_margins": {"terminal_altitude_margin_m": -min(_number(row, "altitude_m") for row in rows)},
        "termination": str(getattr(trajectory.termination, "value", trajectory.termination)),
        "claim_boundary": (
            "This checks finite committed source-runner truth and the declared ground-contact event; it is not a "
            "surface-authority, trim, or guidance envelope."
        ),
    }
    truth_evaluation = evaluate_truth_objectives(
        _objective_specs(plan),
        rows,
        controller_transitions=tuple(
            ControllerTransition(
                objective_id=str(item["objective_id"]),
                time_s=_number(item, "time_s"),
                reason="EVENT_COMPLETE",
                source="source_scheduled_profile",
            )
            for item in transitions
        ),
        truth_events=frozenset(events),
        truth_event_times=events,
        hard_gates_passed=bool(envelope["pass"]),
    )
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.hl20.source_aerodynamic_release_replay.v1",
        "execution_mode": "source_scheduled_aerodynamic_release_witness",
        "control_realization": (
            "source_scheduled_force_model" if composition.fidelity == "point_mass_3dof" else "source_scheduled_named_attitude_response"
        ),
        "source_aerodynamic_graph": True,
        "participating_guidance_controller": False,
        "physical_effector_allocation": False,
        "direct_wrench_injection": False,
        "step_size_s": 0.5,
        "horizon_s": 120.0,
        "numerical_valid": finite,
        "hard_gates_passed": bool(envelope["pass"]),
        "source_schedule": plan.manifest()["source_bank_schedule_deg"],
    }
    graph_execution = unobserved_mission_graph_execution(
        composition,
        "The fixed source bank schedule has no committed controller graph dispatches or public action channels.",
    ).as_dict()
    runtime["mission_graph_execution"] = graph_execution
    status_trace = build_committed_status_trace(composition, _status_samples(rows))
    action_trace = build_uncontrolled_committed_control_trace(composition, tuple(_number(row, "time_s") for row in rows))
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    runtime["resource_ledger"] = resource_ledger_summary(resource_ledger)
    runtime["semantic_action_trace"] = control_trace_summary(action_trace)
    result = HL20SourceReleaseCompositionExecution(
        composition,
        preflight,
        plan,
        trajectory,
        destination,
        runtime,
        envelope,
        truth_evaluation,
        tuple(transitions),
        status_trace,
        action_trace,
    )
    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_json(
        destination / "source_provenance.json",
        {
            "schema": "taoryx.hl20-source-release-provenance/v1alpha1",
            "source_model_id": HL20_SOURCE_MODEL_ID,
            "source_package_sha256": HL20_PACKAGE_SHA256,
            "source_aerodynamics_sha256": HL20_AERODYNAMICS_SHA256,
            "hl20_fixed_mass_kg": HL20_FIXED_MASS_KG,
            "runner": "taoryx.hl20_reachability.run_hl20_source_release",
            "fidelity": composition.fidelity,
            "source_schedule": plan.manifest()["source_bank_schedule_deg"],
            "claim_boundary": (
                "This binds the executed source-aerodynamic HL-20 release witness to its declared package and "
                "aerodynamic hashes. The synthetic booster and fixed bank schedule are configuration assumptions, "
                "not manufacturer flight-control data."
            ),
        },
    )
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "mission_graph_execution.json", graph_execution)
    _write_json(destination / "envelope_report.json", envelope)
    _write_json(destination / "controller_transitions.json", transitions)
    _write_json(destination / "objective_report.json", truth_evaluation)
    _write_json(destination / "status_trace.json", status_trace)
    _write_json(destination / "semantic_action_trace.json", action_trace)
    _write_json(destination / "resource_ledger.json", resource_ledger)
    _write_json(
        destination / "evaluation.json",
        build_composition_trajectory_evaluation(
            composition, preflight, truth_evaluation, runtime=runtime, envelope=envelope,
            claim_boundary=(
                "This executes the fixed source-aerodynamic HL-20 booster/release and scheduled-bank witness. "
                "It does not establish user-driven guidance, trim, physical control-surface allocation, a "
                "closed-loop controller, or the high-altitude glide-energy mission."
            ), status_trace=status_trace,
        ).as_dict(),
    )
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _truth_row(state: object) -> dict[str, object]:
    position = tuple(float(value) for value in getattr(state, "position_m"))
    velocity = tuple(float(value) for value in getattr(state, "velocity_m_s"))
    attitude = getattr(state, "attitude_rad", None)
    rates = getattr(state, "attitude_rate_rad_s", None)
    row: dict[str, object] = {
        "time_s": float(getattr(state, "time_s")),
        "position_m": list(position),
        "velocity_m_s": list(velocity),
        "north_m": position[0],
        "east_m": position[1],
        "altitude_m": position[2],
        "speed_m_s": math.sqrt(sum(value * value for value in velocity)),
        "mass_kg": float(getattr(state, "mass_kg")),
        "phase": str(getattr(state, "phase")),
    }
    if attitude is not None:
        row["attitude_rad"] = [float(value) for value in attitude]
    if rates is not None:
        row["attitude_rate_rad_s"] = [float(value) for value in rates]
    return row
    ####


def _truth_events(trajectory: object, rows: list[dict[str, object]]) -> dict[str, float]:
    events: dict[str, float] = {}
    for event in (*getattr(trajectory, "mission_events"), *getattr(trajectory, "deployment_events")):
        event_id = event.get("event_id")
        time_s = event.get("accepted_time_s")
        if isinstance(event_id, str) and isinstance(time_s, int | float):
            events[event_id] = float(time_s)
    events["ground-contact"] = _number(rows[-1], "time_s")
    return events
    ####


def _source_transitions(plan: HL20SourceReleaseMissionPlan, events: dict[str, float]) -> list[dict[str, object]]:
    transitions: list[dict[str, object]] = []
    for segment in plan.segments:
        event = segment.required_truth_events[-1]
        transitions.append({
            "objective_id": segment.instance_id,
            "segment_instance_id": segment.instance_id,
            "segment_id": segment.segment_id,
            "time_s": events[event],
            "reason": "EVENT_COMPLETE",
            "source": "source_scheduled_profile",
            "event": event,
        })
    return transitions
    ####


def _objective_specs(plan: HL20SourceReleaseMissionPlan) -> tuple[TruthObjectiveSpec, ...]:
    return tuple(
        TruthObjectiveSpec(id=segment.instance_id, objective_type="event", event_id=segment.required_truth_events[-1])
        for segment in plan.segments
    )
    ####


def _status_samples(rows: list[dict[str, object]]) -> tuple[BatchTruthSample, ...]:
    return tuple(BatchTruthSample(time_s=_number(row, "time_s"), raw_values=row, execution_status="completed" if index == len(rows) - 1 else "active") for index, row in enumerate(rows))
    ####


def _finite_rows(rows: list[dict[str, object]]) -> bool:
    return bool(rows) and all(
        math.isfinite(float(value))
        for row in rows
        for value in (_number(row, "time_s"), _number(row, "mass_kg"), _number(row, "speed_m_s"), *_vector(row, "position_m"), *_vector(row, "velocity_m_s"))
    )
    ####


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    keys = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(value, separators=(",", ":")) if isinstance(value, list | dict) else value for key, value in row.items()})
    ####


def _number(row: dict[str, object], name: str) -> float:
    value = row.get(name)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"HL-20 source-release telemetry {name!r} must be finite numeric")
    return float(value)
    ####


def _vector(row: dict[str, object], name: str) -> tuple[float, float, float]:
    value = row.get(name)
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"HL-20 source-release telemetry {name!r} must be a three-vector")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"HL-20 source-release telemetry {name!r} must be finite")
    return result  # type: ignore[return-value]
    ####


__all__ = ["HL20SourceReleaseCompositionExecution", "execute_hl20_source_booster_release_composition"]
