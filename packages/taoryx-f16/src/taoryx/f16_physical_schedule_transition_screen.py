"""Composition endpoint for the bounded source-backed F-16 LQR schedule transition.

This endpoint is deliberately separate from the F-16 held-node schedule
interior screens.  It time-marches four retained altitude-coordinate
transitions using the common scheduled physical-wrench runner, the source
family's declared derivative/effectiveness blend, and actual bounded
elevator, aileron, rudder, and throttle allocation at every step.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from .claim_bound_evidence import bind_release_evidence
from .composition_control_trace import BatchControlSample, build_committed_control_trace, control_trace_summary
from .composition_evaluation import build_composition_trajectory_evaluation
from .composition_graph_evidence import unobserved_mission_graph_execution
from .composition_resource_ledger import build_committed_resource_ledger, resource_ledger_summary
from .composition_sensor_trace import BatchTruthSample
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .mission_capability import MissionCapabilityEstimate, estimate_mission_capability
from .source_f16 import (
    F16_SOURCE_PHYSICAL_SCHEDULE_PITCH_WRENCH_BIAS_FRACTION,
    F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS,
    F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_CASES,
    F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_DT_S,
    F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_DURATION_S,
    F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_RECOVERY_THRESHOLD,
    run_f16_source_physical_lqr_schedule_transition_cases,
)
from .tuning_application import TuningApplicationContextSet
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    VehicleExecutionPreflight,
    build_concrete_capability_preflight_evidence,
    preflight_vehicle_composition,
)

_MISSION_ID = "f16_local_physical_surface_lqr_schedule_transition_screen_v1"
_INITIALIZATION_ID = "source_physical_schedule_local_nodes"
_SEGMENT_ID = "local_physical_surface_lqr_schedule_transition_screen"
_CAPABILITY_ADAPTER_ID = "taoryx.f16_local_physical_surface_lqr_schedule_transition_screen.capability.v1"
_CONTROLLER_CAMPAIGN_ID = "f16-source-surface-schedule-lqr-v1"


@dataclass(frozen=True, slots=True)
class F16PhysicalScheduleTransitionScreenPlan:
    """Exact parameter-free lowering for the retained source LQR cases."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_instance_id: str

    def manifest(self) -> dict[str, object]:
        """Return the executable schedule policy and narrow claim boundary."""

        return {
            "schema": "taoryx.f16-local-physical-surface-lqr-schedule-transition-screen-plan/v1alpha1",
            "family_id": self.family_id,
            "mission_id": self.mission_id,
            "fidelity": self.fidelity,
            "initialization_id": self.initialization_id,
            "segments": [
                {
                    "instance_id": self.segment_instance_id,
                    "id": _SEGMENT_ID,
                    "transition_semantics": "screen_completion_only",
                }
            ],
            "controller_method": "lqr",
            "controller_campaign_id": _CONTROLLER_CAMPAIGN_ID,
            "control_realization": "surface_allocated",
            "controller_selection": "linearly_interpolated_source_lqr_schedule_by_altitude_coordinate",
            "derivative_effectiveness_policy": "linear_blend_of_validated_source_endpoint_nodes",
            "node_ids": list(F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS),
            "transition_cases": {
                identifier: {
                    "start_node_id": start,
                    "end_node_id": end,
                    "perturbation": dict(perturbation),
                }
                for identifier, (start, end, perturbation) in F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_CASES.items()
            },
            "case_duration_s": F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_DURATION_S,
            "dt_s": F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_DT_S,
            "persistent_disturbance_screen": {
                "id": "f16-schedule-matched-pitch-wrench-offset",
                "kind": "constant_offset",
                "input": "external_pitch_moment_bias_nm",
                "body_moment_axis": "total_moment_y_nm",
                "fraction_of_declared_pitch_wrench_scale": F16_SOURCE_PHYSICAL_SCHEDULE_PITCH_WRENCH_BIAS_FRACTION,
                "artifact_filename": "robustness_report.json",
            },
            "claim_boundary": (
                "This executes four bounded altitude-coordinate transitions through a source-node LQR interpolation, "
                "an explicit endpoint derivative/effectiveness blend, and actual bounded effectors. It is not an "
                "integrated navigation path, a new aerodynamic table, wind or mass rejection, a full envelope, or "
                "flight qualification. The paired three-case screen applies only a constant matched external pitch "
                "moment through the source plant's full-inertia dynamics seam."
            ),
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class F16PhysicalScheduleTransitionScreenExecution:
    """Public result for one four-case scheduled physical allocation campaign."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: F16PhysicalScheduleTransitionScreenPlan
    output_dir: Path
    runtime: dict[str, object]
    transition_report: dict[str, object]
    robustness_report: dict[str, object]
    evaluation: dict[str, object]
    status_trace: dict[str, object]
    control_trace: dict[str, object]
    claim_boundary: str

    @property
    def screen_pass(self) -> bool:
        """Return only the retained transition-screen disposition."""

        return bool(self.evaluation["mission_pass"])
        ####

    def as_dict(self) -> dict[str, object]:
        """Serialize the exact Composition execution result."""

        return {
            "schema": "taoryx.f16-local-physical-surface-lqr-schedule-transition-screen-execution/v1alpha1",
            "status": "development_schedule_transition_screen_pass" if self.screen_pass else "development_schedule_transition_screen_failed",
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "plan": self.plan.manifest(),
            "output_dir": str(self.output_dir),
            "runtime": self.runtime,
            "schedule_transition": _transition_report_summary(self.transition_report),
            "robustness_screen": _robustness_report_summary(self.robustness_report),
            "control_screen": self.evaluation,
            "screen_pass": self.screen_pass,
            "status_trace": status_trace_summary(self.status_trace),
            "control_trace": control_trace_summary(self.control_trace),
            "claim_boundary": self.claim_boundary,
        }
        ####

    ####


def compile_f16_physical_schedule_transition_screen(
    composition: CompiledVehicleComposition,
) -> F16PhysicalScheduleTransitionScreenPlan:
    """Reject any composition other than the exact source schedule screen."""

    if composition.family_id != "f16_s119":
        raise ValueError("F-16 schedule-transition screen requires the f16_s119 family")
    if composition.mission != _MISSION_ID:
        raise ValueError(f"F-16 schedule-transition screen requires mission {_MISSION_ID!r}")
    if composition.fidelity != "rigid_body_6dof_surface_allocated":
        raise ValueError("F-16 schedule-transition screen requires surface-allocated rigid-body fidelity")
    if composition.initialization.id != _INITIALIZATION_ID or composition.initialization.inputs:
        raise ValueError("F-16 schedule-transition screen requires the parameter-free source schedule initialization")
    if len(composition.segments) != 1 or composition.segments[0].id != _SEGMENT_ID:
        raise ValueError(f"F-16 schedule-transition screen requires exactly one {_SEGMENT_ID!r} segment")
    if composition.segments[0].inputs:
        raise ValueError("F-16 schedule-transition screen does not accept segment overrides")
    return F16PhysicalScheduleTransitionScreenPlan(
        family_id=composition.family_id,
        mission_id=composition.mission,
        fidelity=composition.fidelity,
        initialization_id=composition.initialization.id,
        segment_instance_id=composition.segments[0].instance_id,
    )
    ####


class F16PhysicalScheduleTransitionScreenCapabilityAdapter:
    """Advertise the bounded source-derived physical LQR transition endpoint."""

    id = _CAPABILITY_ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether this adapter owns the exact transition screen."""

        return composition.family_id == "f16_s119" and composition.mission == _MISSION_ID and composition.fidelity == "rigid_body_6dof_surface_allocated"
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Expose source nodes, blend policy, control path, and exact cases."""

        plan = compile_f16_physical_schedule_transition_screen(composition)
        manifest = plan.manifest()
        persistent_disturbance_screen = manifest.get("persistent_disturbance_screen")
        if not isinstance(persistent_disturbance_screen, Mapping):
            raise ValueError("F-16 schedule-transition plan has no typed persistent-disturbance screen")
        manifest["capability"] = {
            "control_realization": "surface_allocated",
            "participating_nonlinear_plant": True,
            "source_physical_trim": True,
            "physical_effector_allocation": True,
            "controller_method": "lqr",
            "controller_campaign_id": _CONTROLLER_CAMPAIGN_ID,
            "controller_selection": "linearly_interpolated_source_lqr_schedule_by_altitude_coordinate",
            "schedule_nodes": list(F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS),
            "transition_execution": {
                "status": "executed_by_this_schedule_transition_screen",
                "mission_id": plan.mission_id,
                "capability_adapter_id": self.id,
                "operations": ["validate", "batch"],
                "derivative_effectiveness_policy": "linear_blend_of_validated_source_endpoint_nodes",
                "control_path": "scheduled_wrench_to_bounded_elevator_aileron_rudder_throttle_allocation",
                "direct_body_moment_injection": False,
            },
            "persistent_disturbance_status": "available_as_declared_matched_external_pitch_moment_screen",
            "persistent_disturbance_screen": dict(persistent_disturbance_screen),
            "claim_boundary": plan.manifest()["claim_boundary"],
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                "four retained source-node LQR altitude-coordinate transitions execute through bounded physical surface/throttle allocation",
                "a three-case matched external pitch-moment robustness screen is emitted with each batch; wind, mass, navigation, envelope, and qualification gates remain separate",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####


def preflight_f16_physical_schedule_transition_screen(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Prove that Composition selects the executable transition evidence path."""

    plan = compile_f16_physical_schedule_transition_screen(composition)
    estimate = estimate_mission_capability(composition)
    if estimate is None or estimate.adapter_id != _CAPABILITY_ADAPTER_ID:
        raise ValueError("F-16 schedule-transition screen has no matching installed capability adapter")
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status="translation_ready",
        translator_id=_CAPABILITY_ADAPTER_ID,
        checks=(
            ExecutionPreflightCheck(
                "f16.semantic_local_physical_surface_lqr_schedule_transition_screen",
                [_INITIALIZATION_ID, _SEGMENT_ID],
                [plan.initialization_id, plan.segment_instance_id],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                "f16.schedule_transition_has_physical_allocator",
                True,
                True,
                None,
                True,
            ),
            ExecutionPreflightCheck(
                "f16.schedule_transition_has_explicit_source_blend_policy",
                "linear_blend_of_validated_source_endpoint_nodes",
                "linear_blend_of_validated_source_endpoint_nodes",
                None,
                True,
            ),
        ),
        diagnostics=(
            "composition lowers to four retained time-marching source schedule transitions; it is not a route translator or qualification runner",
            *estimate.diagnostics,
        ),
        derived_mission=estimate.manifest,
        capability_estimate=build_concrete_capability_preflight_evidence(composition, estimate),
    )
    ####


def execute_f16_physical_schedule_transition_screen(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None = None,
    *,
    tuning_context_set: TuningApplicationContextSet | None = None,
) -> F16PhysicalScheduleTransitionScreenExecution:
    """Run retained source schedule transitions through actual F-16 effectors."""

    if max_steps is not None:
        raise ValueError("--max-steps is unavailable for the fixed-cadence F-16 schedule-transition screen")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}")
    plan = compile_f16_physical_schedule_transition_screen(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    if tuning_context_set is not None:
        tuning_context_set.require_exact_nodes(F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS)
    transition_report = run_f16_source_physical_lqr_schedule_transition_cases(
        tuning_context_set=tuning_context_set,
    )
    robustness_report = _matched_pitch_wrench_offset_report(
        transition_report,
        tuning_context_set=tuning_context_set,
    )
    rows = _rows(transition_report)
    evaluation = _evaluation(transition_report)
    status_trace = build_committed_status_trace(composition, _status_samples(rows))
    control_trace = build_committed_control_trace(composition, _control_samples(rows))
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.fixed_wing.daveml.v1",
        "controller_method": "lqr",
        "controller_campaign_id": _CONTROLLER_CAMPAIGN_ID,
        "controller_selection": "linearly_interpolated_source_lqr_schedule_by_altitude_coordinate",
        "control_realization": "surface_allocated",
        "physical_effector_allocation": True,
        "derivative_effectiveness_policy": "linear_blend_of_validated_source_endpoint_nodes",
        "transition_case_count": len(F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_CASES),
        "source_node_count": len(F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS),
        "dt_s": F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_DT_S,
        "case_duration_s": F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_DURATION_S,
        "hard_gates_passed": evaluation["mission_pass"],
        "persistent_disturbance_screen": {
            "status": "applied",
            "id": robustness_report["id"],
            "artifact_filename": "robustness_report.json",
            "pass": robustness_report["pass"],
            "input": "external_pitch_moment_bias_nm",
            "body_moment_axis": "total_moment_y_nm",
            "fraction_of_declared_pitch_wrench_scale": F16_SOURCE_PHYSICAL_SCHEDULE_PITCH_WRENCH_BIAS_FRACTION,
        },
        "navigation_state": "no_integrated_navigation_or_node_state_transfer_claim",
        "resource_ledger": resource_ledger_summary(resource_ledger),
        "mission_graph_execution": unobserved_mission_graph_execution(
            composition,
            "The F-16 schedule-transition screen has one bounded controller evidence segment and makes no route-graph claim.",
        ).as_dict(),
    }
    tuning_bindings = transition_report.get("tuning_bindings")
    if tuning_bindings is not None:
        if not isinstance(tuning_bindings, list):
            raise TypeError("F-16 schedule transition tuning bindings must be a list")
        runtime["tuning_bindings"] = tuning_bindings
    result = F16PhysicalScheduleTransitionScreenExecution(
        composition=composition,
        preflight=preflight,
        plan=plan,
        output_dir=destination,
        runtime=runtime,
        transition_report=transition_report,
        robustness_report=robustness_report,
        evaluation=evaluation,
        status_trace=status_trace,
        control_trace=control_trace,
        claim_boundary=(
            "This is four retained time-marching F-16 source-node LQR schedule transitions with explicit source-node "
            "derivative/effectiveness blending and actual bounded surface/throttle allocation. It is not a navigation "
            "path, a wind or mass robustness result, full envelope, or flight qualification. The paired robustness "
            "artifact covers only a constant matched external pitch moment through the source full-inertia dynamics seam."
        ),
    )
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "mission_graph_execution.json", runtime["mission_graph_execution"])
    _write_json(destination / "schedule_transition_report.json", transition_report)
    _write_json(
        destination / "robustness_report.json",
        bind_release_evidence(robustness_report, kind="robustness", composition=composition),
    )
    _write_json(destination / "objective_report.json", evaluation)
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
            envelope=transition_report,
            claim_boundary=result.claim_boundary,
            status_trace=status_trace,
            control_trace=control_trace,
        ).as_dict(),
    )
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _evaluation(transition_report: Mapping[str, object]) -> dict[str, object]:
    """Expose the retained transition gates as public evaluation rows."""

    summary = _mapping(transition_report.get("summary"), "transition summary")
    passed = transition_report.get("status") == "pass"
    physical = transition_report.get("direct_body_moment_injection") is False
    return {
        "schema": "taoryx.f16-local-physical-surface-lqr-schedule-transition-screen-evaluation/v1alpha1",
        "kind": "local_physical_schedule_transition_screen",
        "mission_pass": passed and physical,
        "results": [
            {"id": "all_retained_transitions_pass", "status": "pass" if passed else "fail", "required": True},
            {"id": "bounded_physical_allocation", "status": "pass" if physical else "fail", "required": True},
        ],
        "metrics": dict(summary),
        "control_realization": "surface_allocated",
        "controller_method": "lqr",
        "claim_boundary": transition_report["claim_boundary"],
    }
    ####


def _transition_report_summary(report: Mapping[str, object]) -> dict[str, object]:
    """Return a compact reference to the full time-marching evidence artifact."""

    return {
        "schema": report["schema"],
        "status": report["status"],
        "controller_selection": report["controller_selection"],
        "node_ids": report["node_ids"],
        "summary": report["summary"],
        "artifact": "schedule_transition_report.json",
        "claim_boundary": report["claim_boundary"],
    }
    ####


def _matched_pitch_wrench_offset_report(
    nominal_transition_report: Mapping[str, object],
    *,
    tuning_context_set: TuningApplicationContextSet | None = None,
) -> dict[str, object]:
    """Run the declared matched external pitch-moment cases through source dynamics.

    Each case replays the complete four-transition campaign. The offset enters
    only through ``external_pitch_moment_bias_nm`` at the F-16 source
    full-inertia derivative seam, after the controller wrench is allocated to
    the bounded elevator, aileron, rudder, and throttle path.
    """

    cases: list[dict[str, object]] = []
    for identifier, fraction, retained in (
        ("nominal", 0.0, nominal_transition_report),
        ("positive-pitch-offset", F16_SOURCE_PHYSICAL_SCHEDULE_PITCH_WRENCH_BIAS_FRACTION, None),
        ("negative-pitch-offset", -F16_SOURCE_PHYSICAL_SCHEDULE_PITCH_WRENCH_BIAS_FRACTION, None),
    ):
        report = (
            dict(retained)
            if retained is not None
            else run_f16_source_physical_lqr_schedule_transition_cases(
                external_pitch_wrench_bias_fraction=fraction,
                tuning_context_set=tuning_context_set,
            )
        )
        transition_cases = _mapping(report.get("cases"), "F-16 robustness transition cases")
        if not transition_cases:
            raise ValueError("F-16 robustness transition report has no cases")
        normalized_cases = [_mapping(value, f"F-16 robustness transition case {case_id}") for case_id, value in transition_cases.items()]
        maximum_final_normalized_error = max(_number(case, "final_normalized_error") for case in normalized_cases)
        saturated_step_count = sum(int(_number(case, "saturation_steps")) for case in normalized_cases)
        committed_step_count = sum(int(math.ceil(_number(case, "duration_s") / _number(case, "dt_s"))) for case in normalized_cases)
        if committed_step_count <= 0:
            raise ValueError("F-16 robustness transition report has no committed steps")
        saturation_fraction = saturated_step_count / committed_step_count
        external_dynamics = _mapping(report.get("external_dynamics"), "F-16 external dynamics")
        reported_fraction = _number(external_dynamics, "pitch_wrench_bias_fraction")
        if reported_fraction != fraction:
            raise ValueError("F-16 robustness report does not retain its requested pitch bias fraction")
        case_pass = (
            report.get("status") == "pass"
            and maximum_final_normalized_error <= F16_SOURCE_PHYSICAL_SCHEDULE_TRANSITION_RECOVERY_THRESHOLD
            and saturation_fraction == 0.0
        )
        cases.append(
            {
                "id": identifier,
                "parameters": {"pitch_wrench_bias_fraction": fraction},
                "status": "pass" if case_pass else "fail",
                "pass": case_pass,
                "metrics": {
                    "maximum_final_normalized_error": maximum_final_normalized_error,
                    "saturation_fraction": saturation_fraction,
                },
                "external_pitch_moment_bias_nm": _number(external_dynamics, "external_pitch_moment_bias_nm"),
                "pitch_wrench_scale_nm": _number(external_dynamics, "pitch_wrench_scale_nm"),
                "committed_step_count": committed_step_count,
                "saturated_step_count": saturated_step_count,
                "transition_case_count": len(normalized_cases),
                "all_transition_cases_pass": report.get("status") == "pass",
            }
        )
    passed = all(case["pass"] is True for case in cases)
    return {
        "schema": "taoryx.endpoint-robustness-screen/v1alpha1",
        "id": "f16-schedule-matched-pitch-wrench-offset",
        "kind": "constant_offset",
        "status": "pass" if passed else "fail",
        "pass": passed,
        "controller": {
            "method": "lqr",
            "selection": "linearly_interpolated_source_lqr_schedule_by_altitude_coordinate",
            "pitch_wrench_bias_fraction": F16_SOURCE_PHYSICAL_SCHEDULE_PITCH_WRENCH_BIAS_FRACTION,
        },
        "cases": cases,
        "claim_boundary": (
            "This covers only the three declared constant external pitch-moment cases, each replayed across the four "
            "retained F-16 source-node schedule transitions. The moment is converted through the blended full source "
            "inertia after source derivative evaluation; it is not added to a controller request or allocator output. "
            "It does not establish wind or mass robustness, navigation, an arbitrary disturbance envelope, or flight "
            "qualification."
        ),
    }
    ####


def _robustness_report_summary(report: Mapping[str, object]) -> dict[str, object]:
    """Retain the public result pointer without duplicating all robustness detail."""

    return {
        "id": report["id"],
        "kind": report["kind"],
        "status": report["status"],
        "pass": report["pass"],
        "artifact": "robustness_report.json",
        "claim_boundary": report["claim_boundary"],
    }
    ####


def _rows(report: Mapping[str, object]) -> list[dict[str, float | int | str]]:
    """Flatten source-owned scheduled transition samples for interface projection."""

    cases = _mapping(report.get("cases"), "transition cases")
    mass_kg = _number(report, "source_mass_kg")
    rows: list[dict[str, float | int | str]] = []
    time_offset_s = 0.0
    for case_id, case_value in cases.items():
        case = _mapping(case_value, f"transition case {case_id}")
        samples = case.get("samples")
        if not isinstance(samples, list) or not samples:
            raise ValueError(f"transition case {case_id!r} has no committed samples")
        for sample_value in samples:
            sample = _mapping(sample_value, f"transition sample {case_id}")
            state = _mapping(sample.get("state"), "transition state")
            environment = _mapping(sample.get("environment"), "transition environment")
            requested = _mapping(sample.get("requested_wrench"), "transition requested wrench")
            achieved = _mapping(sample.get("achieved_wrench"), "transition achieved wrench")
            position_saturated = _strings(sample.get("position_saturated"), "transition position saturation")
            rate_limited = _strings(sample.get("rate_limited"), "transition rate limiting")
            u, v, w = (_number(state, name) for name in ("u_m_s", "v_m_s", "w_m_s"))
            lower_node = _text(sample, "lower_node")
            upper_node = _text(sample, "upper_node")
            rows.append(
                {
                    "time_s": time_offset_s + _number(sample, "time_s"),
                    "schedule_case_id": str(case_id),
                    "schedule_node_id": lower_node if lower_node == upper_node else f"{lower_node}->{upper_node}",
                    "u_m_s": u,
                    "v_m_s": v,
                    "w_m_s": w,
                    "p_rad_s": _number(state, "p_rad_s"),
                    "q_rad_s": _number(state, "q_rad_s"),
                    "r_rad_s": _number(state, "r_rad_s"),
                    "north_m": 0.0,
                    "east_m": 0.0,
                    "altitude_m": _number(environment, "altitude_m"),
                    "speed_m_s": math.sqrt(u * u + v * v + w * w),
                    "local_roll_deg": 0.0,
                    "local_pitch_deg": math.degrees(_number(environment, "trim_pitch_rad")),
                    "local_heading_deg": 0.0,
                    "requested_force_x_n": _number(requested, "total_force_x_n"),
                    "requested_moment_x_nm": _number(requested, "total_moment_x_nm"),
                    "requested_moment_y_nm": _number(requested, "total_moment_y_nm"),
                    "requested_moment_z_nm": _number(requested, "total_moment_z_nm"),
                    "achieved_force_x_n": _number(achieved, "total_force_x_n"),
                    "achieved_moment_x_nm": _number(achieved, "total_moment_x_nm"),
                    "achieved_moment_y_nm": _number(achieved, "total_moment_y_nm"),
                    "achieved_moment_z_nm": _number(achieved, "total_moment_z_nm"),
                    "residual_force_x_n": _number(requested, "total_force_x_n") - _number(achieved, "total_force_x_n"),
                    "residual_moment_x_nm": _number(requested, "total_moment_x_nm") - _number(achieved, "total_moment_x_nm"),
                    "residual_moment_y_nm": _number(requested, "total_moment_y_nm") - _number(achieved, "total_moment_y_nm"),
                    "residual_moment_z_nm": _number(requested, "total_moment_z_nm") - _number(achieved, "total_moment_z_nm"),
                    "elevator_deg": _number(_mapping(sample.get("actual_effectors"), "transition actual effectors"), "elevator_deg"),
                    "aileron_deg": _number(_mapping(sample.get("actual_effectors"), "transition actual effectors"), "aileron_deg"),
                    "rudder_deg": _number(_mapping(sample.get("actual_effectors"), "transition actual effectors"), "rudder_deg"),
                    "throttle_fraction": _number(_mapping(sample.get("actual_effectors"), "transition actual effectors"), "throttle_fraction"),
                    "allocation_status": _text(sample, "allocation_status"),
                    "saturation_count": len(position_saturated) + len(rate_limited),
                    "allocation_controlled_residual_norm": _number(sample, "allocation_residual"),
                    "mass_kg": mass_kg,
                }
            )
        time_offset_s += _number(case, "duration_s")
    return rows
    ####


def _status_samples(rows: list[dict[str, float | int | str]]) -> tuple[BatchTruthSample, ...]:
    """Project sampled source truth through the F-16 common interface."""

    samples: list[BatchTruthSample] = []
    for index, row in enumerate(rows):
        raw: dict[str, object] = dict(row)
        raw.update(
            {
                "body_velocity_m_s": [_number(row, name) for name in ("u_m_s", "v_m_s", "w_m_s")],
                "body_rate_rad_s": [_number(row, name) for name in ("p_rad_s", "q_rad_s", "r_rad_s")],
                "attitude_euler_deg": [_number(row, name) for name in ("local_roll_deg", "local_pitch_deg", "local_heading_deg")],
                "requested_force_body_n": [_number(row, "requested_force_x_n"), 0.0, 0.0],
                "requested_moment_body_nm": [_number(row, name) for name in ("requested_moment_x_nm", "requested_moment_y_nm", "requested_moment_z_nm")],
                "achieved_force_body_n": [_number(row, "achieved_force_x_n"), 0.0, 0.0],
                "achieved_moment_body_nm": [_number(row, name) for name in ("achieved_moment_x_nm", "achieved_moment_y_nm", "achieved_moment_z_nm")],
                "residual_force_body_n": [_number(row, "residual_force_x_n"), 0.0, 0.0],
                "residual_moment_body_nm": [_number(row, name) for name in ("residual_moment_x_nm", "residual_moment_y_nm", "residual_moment_z_nm")],
                "allocation_residual_norm": _number(row, "allocation_controlled_residual_norm"),
                "wrench_status": _text(row, "allocation_status"),
                "wrench_saturated": int(row["saturation_count"]) > 0,
                "physical_effector_allocation": True,
                "control_realization": "surface_allocated",
                "controller_method": "lqr",
                "controller_selection": "linearly_interpolated_source_lqr_schedule_by_altitude_coordinate",
            }
        )
        samples.append(
            BatchTruthSample(
                time_s=_number(row, "time_s"),
                execution_status="completed" if index == len(rows) - 1 else "active",
                raw_values=raw,
            )
        )
    return tuple(samples)
    ####


def _control_samples(rows: list[dict[str, float | int | str]]) -> tuple[BatchControlSample, ...]:
    """Retain actual engineering-overlay effectors at committed boundaries."""

    samples: list[BatchControlSample] = []
    previous_time: float | None = None
    for row in rows:
        time_s = _number(row, "time_s")
        samples.append(
            BatchControlSample(
                interval_start_time_s=time_s if previous_time is None else previous_time,
                committed_truth_time_s=time_s,
                requested_actions={},
                achieved_effectors={
                    "effector.elevator.position": _number(row, "elevator_deg"),
                    "effector.aileron.position": _number(row, "aileron_deg"),
                    "effector.rudder.position": _number(row, "rudder_deg"),
                    "effector.throttle.position": _number(row, "throttle_fraction"),
                },
            )
        )
        previous_time = time_s
    return tuple(samples)
    ####


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value
    ####


def _strings(value: object, label: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ValueError(f"{label} must be a list of strings")
    return tuple(value)
    ####


def _number(values: Mapping[str, object], name: str) -> float:
    value = values.get(name)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"F-16 schedule-transition value {name!r} must be finite numeric")
    return float(value)
    ####


def _text(values: Mapping[str, object], name: str) -> str:
    value = values.get(name)
    if not isinstance(value, str) or not value:
        raise ValueError(f"F-16 schedule-transition value {name!r} must be nonempty text")
    return value
    ####


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    if not rows:
        raise ValueError("F-16 schedule-transition screen produced no telemetry rows")
    fields = sorted({field for row in rows for field in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    ####


__all__ = [
    "F16PhysicalScheduleTransitionScreenCapabilityAdapter",
    "F16PhysicalScheduleTransitionScreenExecution",
    "F16PhysicalScheduleTransitionScreenPlan",
    "compile_f16_physical_schedule_transition_screen",
    "execute_f16_physical_schedule_transition_screen",
    "preflight_f16_physical_schedule_transition_screen",
]
