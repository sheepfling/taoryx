"""Source-owned batch execution for passive tumbling-body compositions.

The passive family uses no controller or allocation bridge.  Its point-mass
path is an explicit orientation-averaged drag reduction; its pseudo-6DOF path
reuses the native rigid-body passive-tumble equations.  The same immutable
composition, preflight, truth telemetry, and independent objective artifacts
are emitted for both paths.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from .composition_control_trace import build_uncontrolled_committed_control_trace, control_trace_summary
from .composition_evaluation import build_composition_trajectory_evaluation
from .composition_graph_evidence import unobserved_mission_graph_execution
from .composition_resource_ledger import build_committed_resource_ledger, resource_ledger_summary
from .composition_sensor_trace import BatchTruthSample
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .passive_tumbling_mission_translation import PassiveTumblingMissionPlan, compile_passive_tumbling_mission
from .reachability_envelope import DetachedBodyTrajectory, EnvelopeTermination, simulate_passive_body_release
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import VehicleExecutionPreflight, preflight_vehicle_composition


@dataclass(frozen=True, slots=True)
class PassiveTumblingCompositionExecution:
    """Immutable direct-release execution result for one passive-body witness."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: PassiveTumblingMissionPlan
    output_dir: Path
    runtime: dict[str, object]
    envelope: dict[str, object]
    truth_evaluation: dict[str, object]
    transitions: tuple[dict[str, object], ...]
    status_trace: dict[str, object]
    semantic_action_trace: dict[str, object]
    claim_boundary: str

    @property
    def mission_pass(self) -> bool:
        """Return the hard conjunction for the passive release-to-impact witness."""

        return (
            bool(self.runtime["hard_gates_passed"])
            and bool(self.envelope["pass"])
            and bool(self.truth_evaluation["mission_pass"])
            and all(item["reason"] == "EVENT_COMPLETE" for item in self.transitions)
        )
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the public, reproducible run summary."""

        return {
            "schema": "taoryx.passive-tumbling-composition-execution/v1alpha1",
            "status": "development_nominal_witness_pass" if self.mission_pass else "development_witness_failed",
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "plan": self.plan.manifest(),
            "output_dir": str(self.output_dir),
            "runtime": self.runtime,
            "envelope": self.envelope,
            "truth_evaluation": self.truth_evaluation,
            "controller_transitions": list(self.transitions),
            "status_trace": status_trace_summary(self.status_trace),
            "semantic_action_trace": control_trace_summary(self.semantic_action_trace),
            "mission_pass": self.mission_pass,
            "claim_boundary": self.claim_boundary,
        }
        ####

    ####


def execute_passive_tumbling_composition(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    *,
    step_size_s: float = 0.1,
) -> PassiveTumblingCompositionExecution:
    """Execute one declared passive cylinder release without a synthetic control path."""

    if not math.isfinite(step_size_s) or step_size_s <= 0.0:
        raise ValueError("passive tumbling step_size_s must be positive and finite")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        details = "; ".join(preflight.diagnostics) or "no passive tumbling translator"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {details}")
    plan = compile_passive_tumbling_mission(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    trajectory = simulate_passive_body_release(
        plan.vehicle,
        plan.body,
        plan.command,
        fidelity=plan.fidelity,
        step_size_s=step_size_s,
        horizon_s=plan.horizon_s,
    )
    truth_evaluation = _truth_evaluation(trajectory, plan)
    transitions = _transitions(plan, truth_evaluation)
    envelope = _envelope_report(trajectory, plan)
    finite = _finite_trajectory(trajectory)
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.passive_body.direct_release.v1",
        "execution_mode": "passive_direct_release",
        "control_realization": "uncontrolled",
        "participating_passive_body_plant": True,
        "direct_wrench_injection": False,
        "physical_effector_allocation": False,
        "step_size_s": step_size_s,
        "horizon_s": plan.horizon_s,
        "numerical_valid": finite,
        "hard_gates_passed": finite and bool(envelope["pass"]),
    }
    mission_graph_execution = unobserved_mission_graph_execution(
        composition,
        "The passive release simulation has no controller or graph-transition dispatcher to observe.",
    ).as_dict()
    runtime["mission_graph_execution"] = mission_graph_execution
    status_trace = build_committed_status_trace(composition, _status_samples(trajectory.telemetry))
    semantic_action_trace = build_uncontrolled_committed_control_trace(
        composition,
        tuple(_telemetry_number(row, "time_s") for row in trajectory.telemetry),
    )
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    runtime["resource_ledger"] = resource_ledger_summary(resource_ledger)
    runtime["semantic_action_trace"] = control_trace_summary(semantic_action_trace)
    execution = PassiveTumblingCompositionExecution(
        composition=composition,
        preflight=preflight,
        plan=plan,
        output_dir=destination,
        runtime=runtime,
        envelope=envelope,
        truth_evaluation=truth_evaluation,
        transitions=transitions,
        status_trace=status_trace,
        semantic_action_trace=semantic_action_trace,
        claim_boundary=(
            "The composition proves only the declared passive-cylinder release-to-impact witness. "
            "Its 3DOF result uses an orientation-averaged area policy; its pseudo-6DOF result reuses native "
            "rigid-body passive rotation. Neither exposes a controller, wrench command, actuator allocation, "
            "source-specific spent-stage identity, or family-wide envelope qualification."
        ),
    )
    _write_csv(destination / "truth_telemetry.csv", trajectory.telemetry)
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "mission_graph_execution.json", mission_graph_execution)
    _write_json(destination / "envelope_report.json", envelope)
    _write_json(destination / "controller_transitions.json", list(transitions))
    _write_json(destination / "objective_report.json", truth_evaluation)
    _write_json(destination / "status_trace.json", status_trace)
    _write_json(destination / "semantic_action_trace.json", semantic_action_trace)
    _write_json(destination / "resource_ledger.json", resource_ledger)
    _write_json(
        destination / "evaluation.json",
        build_composition_trajectory_evaluation(
            composition,
            preflight,
            truth_evaluation,
            runtime=runtime,
            envelope=envelope,
            claim_boundary=execution.claim_boundary,
            status_trace=status_trace,
        ).as_dict(),
    )
    _write_json(destination / "execution.json", execution.as_dict())
    return execution
    ####


def _truth_evaluation(
    trajectory: DetachedBodyTrajectory,
    plan: PassiveTumblingMissionPlan,
) -> dict[str, object]:
    """Independently evaluate the release, area/rotation, and impact truth state."""

    states = trajectory.states
    telemetry = trajectory.telemetry
    first = states[0]
    terminal = trajectory.terminal
    finite = _finite_trajectory(trajectory)
    release_pass = math.isclose(first.position_m[2], plan.vehicle.initial_altitude_m, abs_tol=1.0e-9) and math.isclose(
        first.speed_m_s, plan.vehicle.initial_speed_m_s, abs_tol=1.0e-9
    )
    observed_area_policies = sorted({str(row.get("projected_area_policy")) for row in telemetry})
    area_policy_pass = observed_area_policies == [plan.area_policy]
    angular_rate_max = max((_telemetry_number(row, "angular_rate_norm_rad_s") for row in telemetry), default=0.0)
    if plan.fidelity.value == "pseudo_6dof":
        rotation_pass = angular_rate_max > 0.0
        rotation_requirement = "native_passive_rotation_observed"
    else:
        rotation_pass = area_policy_pass
        rotation_requirement = "orientation_averaged_area_policy_declared"
    impact_pass = trajectory.termination is EnvelopeTermination.GROUND_CONTACT and terminal.position_m[2] <= plan.impact_plane_altitude_m
    objectives = (
        _objective("release_state", release_pass, first.time_s, "altitude_and_speed_at_declared_release"),
        _objective("passive_descent", area_policy_pass and rotation_pass, terminal.time_s, rotation_requirement),
        _objective("terminal_impact", impact_pass, terminal.time_s, "ground_contact_at_or_below_impact_plane"),
    )
    return {
        "independent_truth_evaluation": True,
        "required_objectives": list(objectives),
        "results": [
            {
                "id": item["id"],
                "required": True,
                "status": "pass" if item["truth_result"] == "PASS" else "fail",
                "actual": item["truth_result"],
                "critical_requirement": item["critical_requirement"],
            }
            for item in objectives
        ],
        "required_passed": sum(item["truth_result"] == "PASS" for item in objectives),
        "required_total": len(objectives),
        "terminal_pass": impact_pass,
        "numerical_pass": finite,
        "projected_area_policies": observed_area_policies,
        "maximum_angular_rate_rad_s": angular_rate_max,
        "mission_pass": finite and all(item["truth_result"] == "PASS" for item in objectives),
        "claim_boundary": (
            "The evaluator scans accepted passive-body truth telemetry. At 3DOF, a passing area-policy check "
            "does not establish physical tumble; at pseudo-6DOF the native rigid-body reuse must expose nonzero rotation."
        ),
    }
    ####


def _objective(identifier: str, passed: bool, time_s: float, requirement: str) -> dict[str, object]:
    """Return one uniform independent objective result."""

    return {
        "id": identifier,
        "required": True,
        "truth_time_s": time_s,
        "truth_result": "PASS" if passed else "FAIL",
        "critical_requirement": requirement,
    }
    ####


def _telemetry_number(row: Mapping[str, object], identifier: str) -> float:
    """Read one numeric telemetry value without accepting a stringly typed artifact."""

    value = row.get(identifier, 0.0)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"passive tumbling telemetry field {identifier!r} must be finite numeric")
    return float(value)
    ####


def _status_samples(rows: tuple[dict[str, object], ...]) -> tuple[BatchTruthSample, ...]:
    """Adapt passive truth telemetry to the portable committed-status contract."""

    return tuple(
        BatchTruthSample(
            time_s=_telemetry_number(row, "time_s"),
            raw_values=row,
            execution_status="completed" if index == len(rows) - 1 else "active",
        )
        for index, row in enumerate(rows)
    )
    ####


def _transitions(
    plan: PassiveTumblingMissionPlan,
    evaluation: Mapping[str, object],
) -> tuple[dict[str, object], ...]:
    """Record segment completion from evaluator truth rather than controller state."""

    objectives = evaluation.get("required_objectives")
    if not isinstance(objectives, list):
        raise ValueError("passive tumbling evaluation has no required-objective list")
    result_by_id = {str(item.get("id")): str(item.get("truth_result")) for item in objectives if isinstance(item, Mapping)}
    objective_map = {
        "passive_coast": ("release_state", "passive_descent"),
        "atmospheric_descent": ("terminal_impact",),
    }
    return tuple(
        {
            "segment_instance_id": segment.instance_id,
            "segment_id": segment.segment_id,
            "reason": "EVENT_COMPLETE" if all(result_by_id.get(identifier) == "PASS" for identifier in objective_map[segment.segment_id]) else "TIMEOUT_SKIP",
            "controller_capture_diagnostic": False,
            "source": "independent_passive_truth_evaluation",
            "required_truth_objectives": list(objective_map[segment.segment_id]),
        }
        for segment in plan.segments
    )
    ####


def _envelope_report(trajectory: DetachedBodyTrajectory, plan: PassiveTumblingMissionPlan) -> dict[str, object]:
    """Report the passive representation, impact, and numerical boundaries."""

    finite = _finite_trajectory(trajectory)
    realized = trajectory.fidelity.value
    expected_realized = "rigid_body_6dof" if plan.fidelity.value == "pseudo_6dof" else "point_mass_3dof"
    impact = trajectory.termination is EnvelopeTermination.GROUND_CONTACT
    return {
        "pass": finite and impact and realized == expected_realized,
        "termination": trajectory.termination.value,
        "requested_fidelity": plan.fidelity.value,
        "realized_fidelity": realized,
        "area_policy": plan.area_policy,
        "minimum_margins": {
            "numerical_integrity": 1.0 if finite else -1.0,
            "impact_event": 1.0 if impact else -1.0,
            "declared_fidelity_reuse": 1.0 if realized == expected_realized else -1.0,
        },
        "claim_boundary": (
            "This report checks passive numerical, terminal, and representation integrity only. "
            "It is not a thermal, material, source-vehicle, or actuator envelope report."
        ),
    }
    ####


def _finite_trajectory(trajectory: DetachedBodyTrajectory) -> bool:
    """Check all accepted passive-body truth states before promoting a result."""

    return all(all(math.isfinite(value) for value in (*state.position_m, *state.velocity_m_s, state.mass_kg)) for state in trajectory.states)
    ####


def _write_json(path: Path, payload: object) -> None:
    """Write one deterministic JSON artifact."""

    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _write_csv(path: Path, rows: tuple[dict[str, object], ...]) -> None:
    """Write accepted passive-body telemetry without dropping vector fields."""

    if not rows:
        raise ValueError("passive tumbling execution emitted no truth telemetry")
    fieldnames = sorted({key for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(value, sort_keys=True) if isinstance(value, dict | list | tuple) else value for key, value in row.items()})
    ####


__all__ = ["PassiveTumblingCompositionExecution", "execute_passive_tumbling_composition"]
