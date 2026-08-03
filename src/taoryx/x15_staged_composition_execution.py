"""Source-owned execution for X-15-scaled staged reachability compositions.

This executor is intentionally a narrow bridge from the composition product to
the retained reduced-order staging witness.  It does not wrap the synthetic
California-to-Hawaii example and does not promote the local X-15 direct-wrench
load bridge into an end-to-end mission controller.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from .composition_sensor_trace import BatchTruthSample
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .reachability_envelope import TrajectoryResult, simulate_rocket_glide
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import VehicleExecutionPreflight, preflight_vehicle_composition
from .x15_reachability import build_x15_fidelity_evidence, x15_integration_preflight, x15_surrogate_vehicle
from .x15_staged_mission_translation import X15StagedReachabilityMissionPlan, compile_x15_staged_reachability_mission


@dataclass(frozen=True, slots=True)
class X15StagedReachabilityCompositionExecution:
    """One immutable local-frame X-15-scaled staged execution result."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: X15StagedReachabilityMissionPlan
    output_dir: Path
    runtime: dict[str, object]
    envelope: dict[str, object]
    truth_evaluation: dict[str, object]
    controller_transitions: tuple[dict[str, object], ...]
    status_trace: dict[str, object]
    claim_boundary: str

    @property
    def mission_pass(self) -> bool:
        """Return the hard conjunction appropriate to this open-loop witness."""

        transitions_valid = all(item["reason"] == "EVENT_COMPLETE" for item in self.controller_transitions)
        return (
            bool(self.runtime["hard_gates_passed"])
            and bool(self.envelope["pass"])
            and bool(self.truth_evaluation["mission_pass"])
            and transitions_valid
        )
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the public, reproducible composition execution report."""

        return {
            "schema": "taoryx.x15-staged-reachability-composition-execution/v1alpha1",
            "status": "development_nominal_witness_pass" if self.mission_pass else "development_witness_failed",
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "plan": self.plan.manifest(),
            "output_dir": str(self.output_dir),
            "runtime": self.runtime,
            "envelope": self.envelope,
            "truth_evaluation": self.truth_evaluation,
            "controller_transitions": list(self.controller_transitions),
            "status_trace": status_trace_summary(self.status_trace),
            "mission_pass": self.mission_pass,
            "claim_boundary": self.claim_boundary,
        }
        ####
    ####


def execute_x15_staged_reachability_composition(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    *,
    step_size_s: float = 0.5,
) -> X15StagedReachabilityCompositionExecution:
    """Execute one exact X-15-scaled source-staging composition.

    The plant remains a local, fixed-L/D reachability surrogate.  The emitted
    evidence independently scans its accepted truth trajectory for the
    boost/coast/release/glide/handoff/impact objectives rather than treating
    an open-loop phase transition as mission success by itself.
    """

    if not math.isfinite(step_size_s) or step_size_s <= 0.0:
        raise ValueError("X-15 staged reachability step_size_s must be positive and finite")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        details = "; ".join(preflight.diagnostics) or "no X-15 staged translator"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {details}")
    plan = compile_x15_staged_reachability_mission(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    vehicle = x15_surrogate_vehicle()
    source_preflight = x15_integration_preflight(vehicle)
    trajectory = simulate_rocket_glide(
        vehicle,
        plan.command,
        fidelity=plan.fidelity,
        step_size_s=step_size_s,
        horizon_s=plan.horizon_s,
        spawn_children=True,
    )
    evidence = build_x15_fidelity_evidence(
        trajectory,
        fidelity=plan.fidelity,
        command=plan.command,
        step_size_s=step_size_s,
        horizon_s=plan.horizon_s,
    )
    truth_evaluation = _truth_evaluation(evidence)
    transitions = _phase_transitions(plan, truth_evaluation)
    finite = _finite_trajectory(trajectory)
    envelope = _envelope_report(trajectory, source_preflight.passed, finite)
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.x15.staged_reachability_surrogate.v1",
        "execution_mode": "source_pinned_local_reduced_staging_witness",
        "control_realization": "response_law" if plan.fidelity.value == "pseudo_6dof" else "open_loop",
        "participating_native_x15_plant": False,
        "direct_wrench_injection": False,
        "physical_effector_allocation": False,
        "step_size_s": step_size_s,
        "horizon_s": plan.horizon_s,
        "source_integration_preflight": source_preflight.as_dict(),
        "numerical_valid": finite,
        "hard_gates_passed": source_preflight.passed and finite and bool(envelope["pass"]),
    }
    status_trace = build_committed_status_trace(composition, _status_samples(trajectory.telemetry))
    result = X15StagedReachabilityCompositionExecution(
        composition=composition,
        preflight=preflight,
        plan=plan,
        output_dir=destination,
        runtime=runtime,
        envelope=envelope,
        truth_evaluation=truth_evaluation,
        controller_transitions=transitions,
        status_trace=status_trace,
        claim_boundary=(
            "The composition executes a local-frame X-15-scaled reduced staging witness using retained source "
            "mass, speed, cutoff, and release anchors. It proves the declared open-loop event and corridor "
            "witness only. It does not prove a California-to-Hawaii trajectory, native X-15 rigid-body control, "
            "direct-wrench control, physical effectors, controlled terminal handoff, or vehicle-family qualification."
        ),
    )
    _write_csv(destination / "truth_telemetry.csv", trajectory.telemetry)
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_json(destination / "source_provenance.json", evidence["source"])
    _write_json(destination / "fidelity_evidence.json", evidence)
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "envelope_report.json", envelope)
    _write_json(destination / "controller_transitions.json", list(transitions))
    _write_json(destination / "objective_report.json", truth_evaluation)
    _write_json(destination / "status_trace.json", status_trace)
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _truth_evaluation(evidence: Mapping[str, object]) -> dict[str, object]:
    """Project the reachability module's independent truth scan into common fields."""

    mission = evidence.get("mission")
    evaluation = evidence.get("evaluation")
    if not isinstance(mission, Mapping) or not isinstance(evaluation, Mapping):
        raise ValueError("X-15 reachability evidence is missing mission or evaluation records")
    objectives = mission.get("required_objectives")
    if not isinstance(objectives, list) or not all(isinstance(item, Mapping) for item in objectives):
        raise ValueError("X-15 reachability evidence has invalid required objectives")
    copied = [dict(item) for item in objectives]
    required_passed = sum(item.get("truth_result") == "PASS" for item in copied)
    return {
        "independent_truth_evaluation": mission.get("independent_truth_evaluation") is True,
        "required_objectives": copied,
        "required_passed": required_passed,
        "required_total": len(copied),
        "terminal_pass": copied[-1].get("truth_result") == "PASS" if copied else False,
        "hard_envelope_violations": list(evaluation.get("hard_envelope_violations", [])),
        "numerical_pass": evaluation.get("numerical_pass") is True,
        "mission_pass": evaluation.get("mission_pass") is True,
        "claim_boundary": (
            "The objective scan is reconstructed from the reduced accepted truth trajectory. "
            "Open-loop phase changes are diagnostic evidence, not independent objective success."
        ),
    }
    ####


def _phase_transitions(
    plan: X15StagedReachabilityMissionPlan,
    evaluation: Mapping[str, object],
) -> tuple[dict[str, object], ...]:
    """Record open-loop phase evidence without mislabeling it as controller capture."""

    objectives = evaluation.get("required_objectives")
    if not isinstance(objectives, list):
        raise ValueError("X-15 truth evaluation has no required-objective list")
    results = {
        str(item.get("id")): str(item.get("truth_result"))
        for item in objectives
        if isinstance(item, Mapping)
    }
    objective_map = {
        "booster_powered": ("booster_burn_and_cutoff",),
        "booster_coast_release": ("booster_release",),
        "unpowered_glide_handoff": ("unpowered_glide", "high_energy_terminal_corridor", "atmospheric_terminal_handoff"),
        "impact_witness": ("terminal_impact_witness",),
    }
    transitions: list[dict[str, object]] = []
    for segment in plan.segments:
        required = objective_map[segment.segment_id]
        passed = all(results.get(identifier) == "PASS" for identifier in required)
        transitions.append(
            {
                "segment_instance_id": segment.instance_id,
                "segment_id": segment.segment_id,
                "time_s": segment.end_time_s,
                "reason": "EVENT_COMPLETE" if passed else "TIMEOUT_SKIP",
                "controller_capture_diagnostic": False,
                "source": "open_loop_staged_profile",
                "required_truth_objectives": list(required),
            }
        )
    return tuple(transitions)
    ####


def _finite_trajectory(trajectory: TrajectoryResult) -> bool:
    """Check all accepted parent truth states for numerical integrity."""

    return all(
        all(math.isfinite(value) for value in (*state.position_m, *state.velocity_m_s, state.mass_kg))
        for state in trajectory.states
    )
    ####


def _status_samples(rows: tuple[dict[str, object], ...]) -> tuple[BatchTruthSample, ...]:
    """Adapt reduced staging telemetry to the committed batch-status projector."""

    return tuple(
        BatchTruthSample(
            time_s=_status_time(row),
            raw_values=row,
            execution_status="completed" if index == len(rows) - 1 else "active",
        )
        for index, row in enumerate(rows)
    )
    ####


def _status_time(row: Mapping[str, object]) -> float:
    """Read a finite telemetry timestamp without accepting a boolean value."""

    value = row.get("time_s")
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError("X-15 status telemetry time_s must be finite numeric")
    return float(value)
    ####


def _envelope_report(
    trajectory: TrajectoryResult,
    source_preflight_passed: bool,
    numerical_pass: bool,
) -> dict[str, object]:
    """Return declared phase and deployment integrity for this witness."""

    phases: list[str] = []
    for state in trajectory.states:
        if state.phase not in phases:
            phases.append(state.phase)
    expected_phase_order = ["boost", "coast", "glide"]
    phase_order_pass = phases == expected_phase_order
    deployment_pass = len(trajectory.deployment_events) == 1 and len(trajectory.spawned_bodies) == 1
    return {
        "pass": source_preflight_passed and numerical_pass and phase_order_pass and deployment_pass,
        "phase_order": phases,
        "expected_phase_order": expected_phase_order,
        "deployment_event_count": len(trajectory.deployment_events),
        "spawned_child_count": len(trajectory.spawned_bodies),
        "termination": trajectory.termination.value,
        "minimum_margins": {
            "source_staging_preflight": 1.0 if source_preflight_passed else -1.0,
            "phase_order": 1.0 if phase_order_pass else -1.0,
            "booster_deployment": 1.0 if deployment_pass else -1.0,
            "numerical_integrity": 1.0 if numerical_pass else -1.0,
        },
        "claim_boundary": (
            "This envelope report checks source staging, reduced phase order, deployment, and numerical integrity. "
            "It is not a native X-15 aerodynamic-table domain or physical-actuator envelope report."
        ),
    }
    ####


def _write_json(path: Path, payload: object) -> None:
    """Write one deterministic JSON artifact."""

    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _write_csv(path: Path, rows: tuple[dict[str, object], ...]) -> None:
    """Write accepted truth telemetry without losing structured field values."""

    if not rows:
        raise ValueError("X-15 staged reachability emitted no parent telemetry")
    fieldnames = sorted({key for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    key: json.dumps(value, sort_keys=True) if isinstance(value, dict | list | tuple) else value
                    for key, value in row.items()
                }
            )
    ####


__all__ = [
    "X15StagedReachabilityCompositionExecution",
    "execute_x15_staged_reachability_composition",
]
