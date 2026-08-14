"""Source-owned nominal execution for the Hummingbird pseudo-6DOF composition.

The runner translates the declared hover/yaw/translation/contact mission to
the existing bounded aggregate-thrust model.  It deliberately does not import
the showcase script or create a second multirotor physics implementation.
Commands are derived from NED position/yaw intent, applied through the named
pseudo response law, and evaluated independently from the emitted truth rows.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, replace
from pathlib import Path

from .composition_control_trace import BatchControlSample, build_committed_control_trace, control_trace_summary
from .composition_evaluation import build_composition_trajectory_evaluation
from .composition_graph_evidence import GraphOutcome
from .composition_graph_runtime import GraphSegmentExecution, execute_compiled_mission_graph
from .composition_resource_ledger import build_committed_resource_ledger, resource_ledger_summary
from .composition_sensor_trace import BatchTruthSample, build_declared_sensor_trace
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .hummingbird_mission_translation import (
    HummingbirdMissionPlan,
    HummingbirdMissionSegment,
    compile_hummingbird_pseudo_mission,
)
from .trajectory import HummingbirdPseudo6DOFCommand, HummingbirdPseudo6DOFModel, HummingbirdPseudo6DOFState
from .variant_runtime_evidence import build_variant_runtime_evidence
from .vehicle_composition import CompiledMissionGraphNode, CompiledVehicleComposition
from .vehicle_execution_preflight import VehicleExecutionPreflight, preflight_vehicle_composition

_POSITION_TOLERANCE_M = 0.18
_GROUND_POSITION_TOLERANCE_M = 0.12
_SPEED_TOLERANCE_M_S = 0.25
_GROUND_SPEED_TOLERANCE_M_S = 0.05
_HEADING_TOLERANCE_RAD = math.radians(8.0)


@dataclass(frozen=True, slots=True)
class HummingbirdCompositionExecution:
    """Immutable Hummingbird pseudo-6DOF nominal-run result."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: HummingbirdMissionPlan
    output_dir: Path
    runtime: dict[str, object]
    envelope: dict[str, object]
    truth_evaluation: dict[str, object]
    controller_transitions: tuple[dict[str, object], ...]
    sensor_trace: dict[str, object] | None
    status_trace: dict[str, object]
    control_trace: dict[str, object]
    claim_boundary: str

    @property
    def mission_pass(self) -> bool:
        """Return the hard conjunction for this nominal source-owned run."""

        transitions_valid = all(item["reason"] in {"CAPTURED", "DWELL_COMPLETE", "EVENT_COMPLETE"} for item in self.controller_transitions)
        return bool(self.runtime["hard_gates_passed"]) and bool(self.envelope["pass"]) and bool(self.truth_evaluation["mission_pass"]) and transitions_valid
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a compact composition execution report."""

        return {
            "schema": "taoryx.hummingbird-pseudo-composition-execution/v1alpha1",
            "status": "nominal_case_pass" if self.mission_pass else "mission_failed",
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "plan": self.plan.manifest(),
            "output_dir": str(self.output_dir),
            "runtime": self.runtime,
            "envelope": self.envelope,
            "truth_evaluation": self.truth_evaluation,
            "controller_transitions": list(self.controller_transitions),
            "sensor_trace": _sensor_trace_summary(self.sensor_trace),
            "status_trace": status_trace_summary(self.status_trace),
            "control_trace": control_trace_summary(self.control_trace),
            "mission_pass": self.mission_pass,
            "claim_boundary": self.claim_boundary,
        }
        ####

    ####


def execute_hummingbird_pseudo_composition(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    *,
    dt_s: float = 0.02,
) -> HummingbirdCompositionExecution:
    """Execute one preflighted Hummingbird pseudo-6DOF composition.

    The force path is the existing aggregate thrust-vector response model.
    This is therefore a source-owned nominal mission execution path, but not
    a physical individual-rotor or motor-allocation validation.
    """

    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("Hummingbird pseudo execution dt_s must be positive and finite")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        details = "; ".join(preflight.diagnostics) or "no Hummingbird translator"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {details}")
    plan = compile_hummingbird_pseudo_mission(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    model = HummingbirdPseudo6DOFModel(mass_kg=plan.initial_mass_kg)
    state = _initial_state(model, plan)
    initial_sensor_sample = _hummingbird_sensor_sample(
        state,
        execution_status="ready",
        mass_kg=model.mass_kg,
        maximum_thrust_n=model.maximum_thrust_n,
    )
    rows: list[dict[str, object]] = []
    transitions: list[dict[str, object]] = []
    segments_by_instance = {segment.instance_id: segment for segment in plan.segments}

    def execute_segment(
        node: CompiledMissionGraphNode,
        current_state: HummingbirdPseudo6DOFState,
    ) -> GraphSegmentExecution[HummingbirdPseudo6DOFState, tuple[list[dict[str, object]], dict[str, object]]]:
        segment = segments_by_instance.get(node.instance_id)
        if segment is None:
            raise RuntimeError(f"Hummingbird graph refers to unknown segment {node.instance_id!r}")
        terminal_state, segment_rows, transition = _run_segment(model, current_state, segment, dt_s)
        return GraphSegmentExecution(
            segment_id=segment.segment_id,
            outcome=_outcome_for_transition_reason(str(transition["reason"])),
            committed_time_s=_number(transition["time_s"]),
            terminal_state=terminal_state,
            payload=(segment_rows, transition),
        )
        ####

    graph_run = execute_compiled_mission_graph(composition, state, execute_segment)
    for step in graph_run.steps:
        segment_rows, transition = step.execution.payload
        rows.extend(segment_rows)
        transition.update(
            {
                "graph_outcome": step.dispatch.outcome,
                "graph_transition_status": step.dispatch.transition_status,
                "next_instance_id": step.dispatch.next_instance_id,
                "state_transfer": step.dispatch.state_transfer,
            }
        )
        transitions.append(transition)
    successful = all(step.execution.outcome == "success" for step in graph_run.steps)

    finite = _finite_rows(rows)
    envelope = _envelope_report(rows, model, plan)
    evaluation = _evaluate_truth(
        plan.segments,
        rows,
        controller_transitions=tuple(transitions),
        envelope_pass=bool(envelope["pass"]),
        numerical_pass=finite,
    )
    sensor_samples = [initial_sensor_sample]
    for index, row in enumerate(rows):
        sensor_samples.append(
            _hummingbird_sensor_sample(
                _state_from_row(row),
                execution_status="completed" if index == len(rows) - 1 else "active",
                mass_kg=model.mass_kg,
                maximum_thrust_n=model.maximum_thrust_n,
                motors_enabled=bool(row["motors_enabled"]),
            )
        )
    sensor_trace = build_declared_sensor_trace(composition, tuple(sensor_samples))
    status_trace = build_committed_status_trace(composition, tuple(sensor_samples))
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    control_trace = build_committed_control_trace(composition, _control_samples(initial_sensor_sample.time_s, rows))
    variant_runtime_evidence = build_variant_runtime_evidence(
        composition,
        status_trace,
        consumed_native_inputs={"HummingbirdPseudo6DOFModel.mass_kg": model.mass_kg},
    )
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.multirotor.aggregate_thrust_pseudo6dof.v1",
        "response_profile_id": model.profile_id,
        "control_realization": "aggregate_thrust_vector_surrogate",
        "physical_motor_allocation": False,
        "initial_mass_kg": model.mass_kg,
        "dt_s": dt_s,
        "duration_s": _number(rows[-1]["time_s"]) if rows else 0.0,
        "numerical_valid": finite,
        "hard_gates_passed": successful and finite and bool(envelope["pass"]) and variant_runtime_evidence["status"] in {"pass", "not_applicable"},
        "mission_graph_execution": graph_run.evidence.as_dict(),
        "variant_runtime_evidence": variant_runtime_evidence,
        "resource_ledger": resource_ledger_summary(resource_ledger),
    }
    result = HummingbirdCompositionExecution(
        composition=composition,
        preflight=preflight,
        plan=plan,
        output_dir=destination,
        runtime=runtime,
        envelope=envelope,
        truth_evaluation=evaluation,
        controller_transitions=tuple(transitions),
        sensor_trace=sensor_trace,
        status_trace=status_trace,
        control_trace=control_trace,
        claim_boundary=(
            "This is a source-owned nominal Hummingbird pseudo-6DOF mission using a bounded aggregate "
            "thrust-vector and attitude-response model. It proves only the declared hover/yaw/translation/contact "
            "objectives. It does not establish individual motor allocation, rotor inflow, reaction torque, "
            "aerodynamic moment balance, electrical battery/SOC fidelity, robustness, or family qualification."
        ),
    )
    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "mission_graph_execution.json", runtime["mission_graph_execution"])
    _write_json(destination / "variant_runtime_evidence.json", runtime["variant_runtime_evidence"])
    _write_json(destination / "envelope_report.json", envelope)
    _write_json(destination / "controller_transitions.json", list(transitions))
    _write_json(destination / "objective_report.json", evaluation)
    if sensor_trace is not None:
        _write_json(destination / "sensor_observations.json", sensor_trace)
    _write_json(destination / "status_trace.json", status_trace)
    _write_json(destination / "resource_ledger.json", resource_ledger)
    _write_json(destination / "semantic_action_trace.json", control_trace)
    _write_json(
        destination / "evaluation.json",
        build_composition_trajectory_evaluation(
            composition,
            preflight,
            evaluation,
            runtime=runtime,
            envelope=envelope,
            claim_boundary=result.claim_boundary,
            status_trace=status_trace,
            control_trace=control_trace,
        ).as_dict(),
    )
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _outcome_for_transition_reason(reason: str) -> GraphOutcome:
    """Map controller diagnostics to the only graph outcomes this runner emits."""

    if reason in {"CAPTURED", "DWELL_COMPLETE", "EVENT_COMPLETE"}:
        return "success"
    if reason == "TIMEOUT_SKIP":
        return "timeout"
    return "abort"
    ####


def _initial_state(model: HummingbirdPseudo6DOFModel, plan: HummingbirdMissionPlan) -> HummingbirdPseudo6DOFState:
    north, east, down = plan.initialization_ned_m
    altitude = -down
    state = model.initial_state(altitude_m=altitude)
    return replace(
        state,
        position_m=(north, east, altitude),
        attitude_rad=(0.0, 0.0, plan.initial_heading_rad),
        contact=altitude == 0.0,
    )
    ####


def _run_segment(
    model: HummingbirdPseudo6DOFModel,
    state: HummingbirdPseudo6DOFState,
    segment: HummingbirdMissionSegment,
    dt_s: float,
) -> tuple[HummingbirdPseudo6DOFState, list[dict[str, object]], dict[str, object]]:
    """Run one controller-held semantic segment to capture or time budget."""

    rows: list[dict[str, object]] = []
    target_world = _world_position(segment.target_ned_m)
    dwell_start: float | None = None
    contact_time: float | None = None
    motors_enabled = True
    previous_contact = state.contact
    maximum_steps = math.ceil(segment.maximum_duration_s / dt_s)
    reason = "TIMEOUT_SKIP"
    for _ in range(maximum_steps):
        if segment.kind == "touchdown" and state.contact:
            if motors_enabled:
                motors_enabled = False
            command = HummingbirdPseudo6DOFCommand(
                yaw_rad=segment.target_heading_rad,
                thrust_ratio=0.0,
                motors_enabled=False,
                thrust_frame="body_euler",
            )
        else:
            command = _command_for_target(model, state, target_world, segment, motors_enabled)
        state, telemetry = model.step(state, command, dt_s)
        row = dict(telemetry)
        row.update(
            {
                "segment_instance_id": segment.instance_id,
                "segment_id": segment.segment_id,
                "segment_kind": segment.kind,
                "target_ned_m": list(segment.target_ned_m),
                "target_heading_rad": segment.target_heading_rad,
                "commanded_roll_rad": command.roll_rad,
                "commanded_pitch_rad": command.pitch_rad,
                "commanded_yaw_rad": command.yaw_rad,
                "commanded_thrust_ratio": command.thrust_ratio,
            }
        )
        if not previous_contact and state.contact:
            row["transition_event"] = "contact"
        elif not motors_enabled:
            row["transition_event"] = "motor_shutdown"
        else:
            row["transition_event"] = None
        previous_contact = state.contact
        rows.append(row)
        if segment.kind == "touchdown":
            if not motors_enabled and contact_time is None:
                # Start the required post-shutdown hold at the first accepted
                # truth sample with motors actually disabled, not at the
                # previous contact sample where the disable command was only
                # about to be applied.
                contact_time = state.time_s
            if contact_time is not None and state.time_s - contact_time + 1.0e-12 >= segment.required_dwell_s:
                if _ground_settled(state, target_world):
                    reason = "EVENT_COMPLETE"
                    break
            continue
        if _captured(state, target_world, segment.target_heading_rad, require_heading=segment.kind in {"hover", "translation", "yaw"}):
            if dwell_start is None:
                dwell_start = state.time_s
            if state.time_s - dwell_start + 1.0e-12 >= segment.required_dwell_s:
                reason = "DWELL_COMPLETE" if segment.required_dwell_s > 0.0 else "CAPTURED"
                break
        else:
            dwell_start = None
    transition = {
        "segment_instance_id": segment.instance_id,
        "segment_id": segment.segment_id,
        "time_s": state.time_s,
        "reason": reason,
        "controller_capture_diagnostic": reason in {"CAPTURED", "DWELL_COMPLETE", "EVENT_COMPLETE"},
        "maximum_duration_s": segment.maximum_duration_s,
    }
    if not rows:
        raise RuntimeError(f"Hummingbird segment {segment.instance_id!r} emitted no truth rows")
    return state, rows, transition
    ####


def _command_for_target(
    model: HummingbirdPseudo6DOFModel,
    state: HummingbirdPseudo6DOFState,
    target_world: tuple[float, float, float],
    segment: HummingbirdMissionSegment,
    motors_enabled: bool,
) -> HummingbirdPseudo6DOFCommand:
    """Map world-frame position intent to bounded pseudo response commands."""

    north_error = target_world[0] - state.position_m[0]
    east_error = target_world[1] - state.position_m[1]
    altitude_error = target_world[2] - state.position_m[2]
    if segment.maximum_speed_m_s is None:
        north_acceleration = _clamp(1.2 * north_error - 1.6 * state.velocity_m_s[0], -3.0, 3.0)
        east_acceleration = _clamp(1.2 * east_error - 1.6 * state.velocity_m_s[1], -3.0, 3.0)
    else:
        horizontal_error = math.hypot(north_error, east_error)
        desired_speed = min(segment.maximum_speed_m_s, 0.9 * horizontal_error)
        if horizontal_error <= 1.0e-9:
            target_north_velocity = 0.0
            target_east_velocity = 0.0
        else:
            target_north_velocity = desired_speed * north_error / horizontal_error
            target_east_velocity = desired_speed * east_error / horizontal_error
        # The first-order velocity loop has no overspeed forcing term: as the
        # achieved horizontal speed reaches the declared ceiling, acceleration
        # tends to zero, then becomes braking if it exceeds the target.
        north_acceleration = _clamp(target_north_velocity - state.velocity_m_s[0], -3.0, 3.0)
        east_acceleration = _clamp(target_east_velocity - state.velocity_m_s[1], -3.0, 3.0)
    if segment.kind == "touchdown":
        vertical_acceleration = _clamp(0.4 * altitude_error - 0.8 * state.velocity_m_s[2], -0.2, 0.2)
    else:
        vertical_acceleration = _clamp(1.8 * altitude_error - 1.8 * state.velocity_m_s[2], -1.0, 1.0)
    required_up_force = model.mass_kg * (model.gravity_m_s2 + vertical_acceleration)
    horizontal_force_north = model.mass_kg * north_acceleration
    horizontal_force_east = model.mass_kg * east_acceleration
    thrust = math.sqrt(horizontal_force_north**2 + horizontal_force_east**2 + required_up_force**2)
    yaw = state.attitude_rad[2]
    body_forward_force = math.cos(yaw) * horizontal_force_north + math.sin(yaw) * horizontal_force_east
    body_right_force = -math.sin(yaw) * horizontal_force_north + math.cos(yaw) * horizontal_force_east
    pitch = math.asin(_clamp(body_forward_force / max(thrust, 1.0e-9), -0.5, 0.5))
    roll = -math.asin(_clamp(body_right_force / max(thrust, 1.0e-9), -0.5, 0.5))
    available_thrust = model.maximum_thrust_n * max(state.battery_fraction, 1.0e-6)
    return HummingbirdPseudo6DOFCommand(
        roll_rad=roll,
        pitch_rad=pitch,
        yaw_rad=segment.target_heading_rad,
        thrust_ratio=_clamp(thrust / available_thrust, 0.0, 1.0),
        motors_enabled=motors_enabled,
        thrust_frame="body_euler",
    )
    ####


def _evaluate_truth(
    segments: tuple[HummingbirdMissionSegment, ...],
    rows: list[dict[str, object]],
    *,
    controller_transitions: tuple[dict[str, object], ...],
    envelope_pass: bool,
    numerical_pass: bool,
) -> dict[str, object]:
    """Evaluate every required objective solely from emitted truth telemetry."""

    objectives: list[dict[str, object]] = []
    transition_by_instance = {str(item["segment_instance_id"]): item for item in controller_transitions}
    for segment in segments:
        segment_rows = [row for row in rows if row["segment_instance_id"] == segment.instance_id]
        transition = transition_by_instance.get(segment.instance_id)
        if not segment_rows:
            objectives.append(_missing_objective(segment, transition))
            continue
        if segment.kind == "touchdown":
            objectives.append(_evaluate_touchdown(segment, segment_rows, transition))
        else:
            objectives.append(_evaluate_capture(segment, segment_rows, transition))
    terminal_pass = bool(objectives) and str(objectives[-1]["truth_result"]) == "PASS"
    required_pass = all(str(item["truth_result"]) == "PASS" for item in objectives)
    return {
        "independent_truth_evaluation": True,
        "required_objectives": objectives,
        "required_passed": sum(item["truth_result"] == "PASS" for item in objectives),
        "required_total": len(objectives),
        "terminal_pass": terminal_pass,
        "hard_envelope_violations": [] if envelope_pass else ["pseudo_response_envelope"],
        "numerical_pass": numerical_pass,
        "mission_pass": required_pass and terminal_pass and envelope_pass and numerical_pass,
        "claim_boundary": "Truth evaluation scans emitted pseudo-6DOF telemetry independently of controller transitions.",
    }
    ####


def _evaluate_capture(
    segment: HummingbirdMissionSegment,
    rows: list[dict[str, object]],
    transition: dict[str, object] | None,
) -> dict[str, object]:
    target = _world_position(segment.target_ned_m)
    require_heading = segment.kind in {"hover", "translation", "yaw"}
    dwell_start: float | None = None
    truth_time: float | None = None
    best_distance = math.inf
    best_heading = math.inf
    for row in rows:
        state = _state_from_row(row)
        distance = _distance(state.position_m, target)
        heading_error = abs(_wrapped_angle(segment.target_heading_rad - state.attitude_rad[2]))
        best_distance = min(best_distance, distance)
        best_heading = min(best_heading, heading_error)
        if _captured(state, target, segment.target_heading_rad, require_heading=require_heading):
            if dwell_start is None:
                dwell_start = state.time_s
            if state.time_s - dwell_start + 1.0e-12 >= segment.required_dwell_s:
                truth_time = state.time_s
                break
        else:
            dwell_start = None
    passed = truth_time is not None
    return {
        "id": segment.instance_id,
        "type": "yaw_dwell" if segment.kind == "yaw" else "position_velocity_heading_dwell",
        "required": True,
        "controller_transition": _transition_record(transition),
        "truth_result": "PASS" if passed else "FAIL",
        "truth_time_s": truth_time,
        "critical_metric": "position_velocity_heading_dwell" if require_heading else "position_velocity_dwell",
        "actual": {
            "closest_position_error_m": best_distance,
            "closest_heading_error_deg": math.degrees(best_heading) if require_heading else None,
        },
        "tolerance": {
            "position_radius_m": _POSITION_TOLERANCE_M,
            "speed_m_s": _SPEED_TOLERANCE_M_S,
            "heading_error_deg": math.degrees(_HEADING_TOLERANCE_RAD) if require_heading else None,
            "dwell_s": segment.required_dwell_s,
        },
    }
    ####


def _evaluate_touchdown(
    segment: HummingbirdMissionSegment,
    rows: list[dict[str, object]],
    transition: dict[str, object] | None,
) -> dict[str, object]:
    target = _world_position(segment.target_ned_m)
    contact_rows = [row for row in rows if bool(row["contact_state"])]
    first_contact = contact_rows[0] if contact_rows else None
    shutdown_rows = [row for row in contact_rows if bool(row["shutdown"])]
    settled = False
    truth_time: float | None = None
    if shutdown_rows:
        first_shutdown_time = _number(shutdown_rows[0]["time_s"])
        for row in shutdown_rows:
            state = _state_from_row(row)
            if state.time_s - first_shutdown_time + 1.0e-12 >= segment.required_dwell_s and _ground_settled(state, target):
                settled = True
                truth_time = state.time_s
                break
    touchdown = first_contact is not None and _distance(_state_from_row(first_contact).position_m, target) <= _GROUND_POSITION_TOLERANCE_M
    passed = touchdown and settled
    return {
        "id": segment.instance_id,
        "type": "touchdown",
        "required": True,
        "controller_transition": _transition_record(transition),
        "truth_result": "PASS" if passed else "FAIL",
        "truth_time_s": truth_time,
        "critical_metric": "contact_shutdown_settle",
        "actual": {
            "contact": first_contact is not None,
            "touchdown_position_error_m": _distance(_state_from_row(first_contact).position_m, target) if first_contact else math.inf,
            "post_contact_shutdown": bool(shutdown_rows),
            "post_contact_settled": settled,
        },
        "tolerance": {
            "pad_radius_m": _GROUND_POSITION_TOLERANCE_M,
            "post_contact_speed_m_s": _GROUND_SPEED_TOLERANCE_M_S,
            "post_contact_dwell_s": segment.required_dwell_s,
        },
    }
    ####


def _missing_objective(segment: HummingbirdMissionSegment, transition: dict[str, object] | None) -> dict[str, object]:
    return {
        "id": segment.instance_id,
        "type": segment.kind,
        "required": True,
        "controller_transition": _transition_record(transition),
        "truth_result": "FAIL",
        "truth_time_s": None,
        "critical_metric": "missing_truth_telemetry",
        "actual": None,
        "tolerance": None,
    }
    ####


def _transition_record(transition: dict[str, object] | None) -> dict[str, object]:
    if transition is None:
        return {"time_s": None, "reason": "NOT_EXECUTED"}
    return {
        "time_s": _number(transition["time_s"]),
        "reason": str(transition["reason"]),
        "controller_capture_diagnostic": bool(transition["controller_capture_diagnostic"]),
    }
    ####


def _captured(
    state: HummingbirdPseudo6DOFState,
    target_world: tuple[float, float, float],
    target_heading_rad: float,
    *,
    require_heading: bool,
) -> bool:
    return (
        _distance(state.position_m, target_world) <= _POSITION_TOLERANCE_M
        and _norm(state.velocity_m_s) <= _SPEED_TOLERANCE_M_S
        and (not require_heading or abs(_wrapped_angle(target_heading_rad - state.attitude_rad[2])) <= _HEADING_TOLERANCE_RAD)
    )
    ####


def _ground_settled(state: HummingbirdPseudo6DOFState, target_world: tuple[float, float, float]) -> bool:
    return (
        state.contact and _distance(state.position_m, target_world) <= _GROUND_POSITION_TOLERANCE_M and _norm(state.velocity_m_s) <= _GROUND_SPEED_TOLERANCE_M_S
    )
    ####


def _state_from_row(row: dict[str, object]) -> HummingbirdPseudo6DOFState:
    position = _three(row["position_m"])
    velocity = _three(row["velocity_m_s"])
    attitude = _three(row["achieved_attitude_rad"])
    rates = _three(row["body_rate_rad_s"])
    return HummingbirdPseudo6DOFState(
        _number(row["time_s"]),
        position,
        velocity,
        attitude,
        rates,
        _number(row["battery_fraction"]),
        _number(row["achieved_thrust_n"]),
        bool(row["contact_state"]),
    )
    ####


def _hummingbird_sensor_sample(
    state: HummingbirdPseudo6DOFState,
    *,
    execution_status: str,
    mass_kg: float,
    maximum_thrust_n: float,
    motors_enabled: bool = True,
) -> BatchTruthSample:
    """Return the same raw observation layout used by the Hummingbird episode."""

    position = state.position_m
    velocity = state.velocity_m_s
    return BatchTruthSample(
        time_s=state.time_s,
        raw_values={
            "position_ned_m": [position[0], position[1], -position[2]],
            "velocity_ned_m_s": [velocity[0], velocity[1], -velocity[2]],
            "attitude_rad": list(state.attitude_rad),
            "body_rate_rad_s": list(state.attitude_rate_rad_s),
            "battery_fraction": state.battery_fraction,
            "mass_kg": mass_kg,
            "aggregate_thrust_n": state.thrust_n,
            "aggregate_thrust_fraction": state.thrust_n / maximum_thrust_n,
            "motors_enabled": motors_enabled,
            "velocity_horizontal_speed_m_s": math.hypot(velocity[0], velocity[1]),
            "contact": state.contact,
            "guidance_waypoint_range_m": 0.0,
            "guidance_waypoint_captured": False,
            "guidance_waypoint_status": "inactive",
            "control_realization": "aggregate_thrust_vector_surrogate",
            "controller_method": "waypoint_velocity_cascade",
            "physical_motor_allocation": False,
        },
        execution_status=execution_status,
    )
    ####


def _control_samples(
    initial_time_s: float,
    rows: list[dict[str, object]],
) -> tuple[BatchControlSample, ...]:
    """Bind held aggregate-response commands to their committed truth rows.

    Hummingbird's pseudo-6DOF interface publishes no physical effector channel,
    so ``achieved_effectors`` remains empty. The aggregate thrust response is
    available in the status trace instead and must not be relabeled as motor
    allocation.
    """

    samples: list[BatchControlSample] = []
    interval_start_time_s = initial_time_s
    for row in rows:
        committed_truth_time_s = _number(row["time_s"])
        samples.append(
            BatchControlSample(
                interval_start_time_s=interval_start_time_s,
                committed_truth_time_s=committed_truth_time_s,
                requested_actions={
                    "attitude.roll.command": _number(row["commanded_roll_rad"]),
                    "attitude.pitch.command": _number(row["commanded_pitch_rad"]),
                    "attitude.yaw.command": _number(row["commanded_yaw_rad"]),
                    "propulsion.command.fraction": _number(row["commanded_thrust_ratio"]),
                    "propulsion.enable": bool(row["motors_enabled"]),
                },
                achieved_effectors={},
            )
        )
        interval_start_time_s = committed_truth_time_s
    return tuple(samples)
    ####


def _sensor_trace_summary(trace: dict[str, object] | None) -> dict[str, object] | None:
    """Keep the execution manifest concise while retaining the full trace separately."""

    if trace is None:
        return None
    samples = trace.get("samples")
    return {
        "schema": trace["schema"],
        "observation_profile_id": trace["observation_profile_id"],
        "sample_count": len(samples) if isinstance(samples, list) else 0,
        "artifact": "sensor_observations.json",
    }
    ####


def _envelope_report(
    rows: list[dict[str, object]],
    model: HummingbirdPseudo6DOFModel,
    plan: HummingbirdMissionPlan,
) -> dict[str, object]:
    profile = model.profile
    assert profile is not None
    rates = tuple(profile.response[axis].maximum_rate_rad_s for axis in ("roll", "pitch", "yaw"))
    rate_margin = math.inf
    tilt_margin = math.inf
    battery_margin = math.inf
    translation_speed_margin = math.inf
    speed_limit_by_instance = {segment.instance_id: segment.maximum_speed_m_s for segment in plan.segments if segment.maximum_speed_m_s is not None}
    for row in rows:
        body_rate = _three(row["body_rate_rad_s"])
        attitude = _three(row["achieved_attitude_rad"])
        rate_margin = min(rate_margin, *(limit - abs(value) for value, limit in zip(body_rate, rates, strict=True)))
        tilt_margin = min(tilt_margin, math.pi / 2.0 - abs(attitude[0]), math.pi / 2.0 - abs(attitude[1]))
        battery_margin = min(battery_margin, _number(row["battery_fraction"]))
        speed_limit = speed_limit_by_instance.get(str(row["segment_instance_id"]))
        if speed_limit is not None:
            translation_speed_margin = min(translation_speed_margin, speed_limit - _norm(_three(row["velocity_m_s"])))
    passed = bool(rows) and rate_margin >= -1.0e-9 and tilt_margin >= -1.0e-9 and battery_margin >= -1.0e-9 and translation_speed_margin >= -1.0e-9
    return {
        "pass": passed,
        "minimum_margins": {
            "body_rate_rad_s": rate_margin,
            "tilt_rad": tilt_margin,
            "battery_fraction": battery_margin,
            "translation_speed_m_s": translation_speed_margin,
        },
        "profile_id": model.profile_id,
        "claim_boundary": "Envelope limits belong to the declared pseudo response model, not a physical rotor/servo envelope.",
    }
    ####


def _finite_rows(rows: list[dict[str, object]]) -> bool:
    try:
        return bool(rows) and all(
            math.isfinite(_number(value))
            for row in rows
            for key in ("time_s", "battery_fraction", "achieved_thrust_n", "commanded_thrust_ratio")
            for value in (row[key],)
        )
    except (KeyError, TypeError, ValueError):
        return False
    ####


def _world_position(ned: tuple[float, float, float]) -> tuple[float, float, float]:
    return (ned[0], ned[1], -ned[2])
    ####


def _three(value: object) -> tuple[float, float, float]:
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError("Hummingbird telemetry vector must contain exactly three values")
    return (_number(value[0]), _number(value[1]), _number(value[2]))
    ####


def _number(value: object) -> float:
    if isinstance(value, bool):
        raise ValueError("Hummingbird telemetry numeric value cannot be boolean")
    if not isinstance(value, int | float | str):
        raise ValueError("Hummingbird telemetry numeric value must be numeric")
    try:
        result = float(value)
    except ValueError as error:
        raise ValueError("Hummingbird telemetry numeric value must be numeric") from error
    if not math.isfinite(result):
        raise ValueError("Hummingbird telemetry numeric value must be finite")
    return result
    ####


def _distance(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right, strict=True)))
    ####


def _norm(value: tuple[float, float, float]) -> float:
    return math.sqrt(sum(component * component for component in value))
    ####


def _wrapped_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi
    ####


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
    ####


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise ValueError("cannot write empty Hummingbird truth telemetry")
    columns = tuple(rows[0])
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(value, separators=(",", ":")) if isinstance(value, list | dict) else value for key, value in row.items()})
    ####


__all__ = ["HummingbirdCompositionExecution", "execute_hummingbird_pseudo_composition"]
