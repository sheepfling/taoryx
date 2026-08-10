"""Composition endpoints for retained F-16 physical schedule interiors.

This screen turns the already declared four-node source schedule evidence into
an exact, batch-runnable Composition path.  Each recovery explicitly selects
one source-retrimmed node and holds its locally derived controller; it does *not*
interpolate gains while the nonlinear plant is running.  The two retained
body-``w`` perturbations are the intersection that passes at every node.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from .composition_control_trace import BatchControlSample, build_committed_control_trace, control_trace_summary
from .composition_evaluation import build_composition_trajectory_evaluation
from .composition_graph_evidence import unobserved_mission_graph_execution
from .composition_resource_ledger import build_committed_resource_ledger, resource_ledger_summary
from .composition_sensor_trace import BatchTruthSample
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .mission_capability import MissionCapabilityEstimate, estimate_mission_capability
from .physical_lqr import (
    PhysicalWrenchLqiValidation,
    PhysicalWrenchLqrValidation,
    validate_nonlinear_wrench_lqi,
    validate_nonlinear_wrench_lqr,
)
from .source_f16 import (
    F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS,
    F16SourcePhysicalScheduleLqiNode,
    F16SourcePhysicalScheduleNode,
    build_f16_source_physical_schedule_lqi_nodes,
    build_f16_source_physical_schedule_nodes,
)
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    VehicleExecutionPreflight,
    build_concrete_capability_preflight_evidence,
    preflight_vehicle_composition,
)

_MISSION_ID = "f16_local_physical_surface_lqr_schedule_interior_screen_v1"
_INITIALIZATION_ID = "source_physical_schedule_local_nodes"
_SEGMENT_ID = "local_physical_surface_lqr_schedule_interior_screen"
_CAPABILITY_ADAPTER_ID = "taoryx.f16_local_physical_surface_lqr_schedule_interior_screen.capability.v1"
_LQI_MISSION_ID = "f16_local_physical_surface_lqi_schedule_interior_screen_v1"
_LQI_SEGMENT_ID = "local_physical_surface_lqi_schedule_interior_screen"
_LQI_CAPABILITY_ADAPTER_ID = "taoryx.f16_local_physical_surface_lqi_schedule_interior_screen.capability.v1"
_DURATION_S = 2.0
_DT_S = 0.02
_RECOVERY_THRESHOLD = 0.25
_INTERIOR_CASES: dict[str, dict[str, float]] = {
    "alpha_plus_1mps": {"w_m_s": 1.0},
    "alpha_minus_1mps": {"w_m_s": -1.0},
}
_LQI_INTERIOR_CASES: dict[str, dict[str, float]] = {
    "alpha_plus_0_25mps": {"w_m_s": 0.25},
    "alpha_minus_0_25mps": {"w_m_s": -0.25},
}


@dataclass(frozen=True, slots=True)
class _ScheduleScreenConfiguration:
    """Static, explicit bounds for one public held-node schedule screen."""

    mission_id: str
    segment_id: str
    capability_adapter_id: str
    controller_method: str
    interior_cases: Mapping[str, Mapping[str, float]]
    campaign_id: str

    ####


_SCHEDULE_CONFIGURATIONS: dict[str, _ScheduleScreenConfiguration] = {
    _MISSION_ID: _ScheduleScreenConfiguration(
        mission_id=_MISSION_ID,
        segment_id=_SEGMENT_ID,
        capability_adapter_id=_CAPABILITY_ADAPTER_ID,
        controller_method="lqr",
        interior_cases=_INTERIOR_CASES,
        campaign_id="f16-source-surface-local-lqr-v1",
    ),
    _LQI_MISSION_ID: _ScheduleScreenConfiguration(
        mission_id=_LQI_MISSION_ID,
        segment_id=_LQI_SEGMENT_ID,
        capability_adapter_id=_LQI_CAPABILITY_ADAPTER_ID,
        controller_method="lqi",
        interior_cases=_LQI_INTERIOR_CASES,
        campaign_id="f16-source-surface-local-lqi-v1",
    ),
}


def _configuration_for_mission(mission_id: str) -> _ScheduleScreenConfiguration:
    """Return the exact campaign configuration without accepting aliases."""

    try:
        return _SCHEDULE_CONFIGURATIONS[mission_id]
    except KeyError as error:
        expected = ", ".join(repr(identifier) for identifier in _SCHEDULE_CONFIGURATIONS)
        raise ValueError(f"F-16 schedule-interior screen requires one of {expected}") from error
    ####


def _nodes_for_plan(
    plan: "F16PhysicalScheduleInteriorScreenPlan",
) -> tuple[F16SourcePhysicalScheduleNode | F16SourcePhysicalScheduleLqiNode, ...]:
    """Build only the source-node controller family selected by Composition."""

    if plan.controller_method == "lqr":
        return build_f16_source_physical_schedule_nodes()
    if plan.controller_method == "lqi":
        return build_f16_source_physical_schedule_lqi_nodes()
    raise ValueError(f"unsupported F-16 schedule controller method {plan.controller_method!r}")
    ####


@dataclass(frozen=True, slots=True)
class F16PhysicalScheduleInteriorScreenPlan:
    """Exact lowering for one discrete four-node F-16 schedule campaign."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_instance_id: str
    duration_s: float
    dt_s: float
    controller_method: str

    def manifest(self) -> dict[str, object]:
        """Return the schedule's real node/case scope and nonclaims."""

        configuration = _configuration_for_mission(self.mission_id)
        return {
            "schema": f"taoryx.f16-local-physical-surface-{self.controller_method}-schedule-interior-screen-plan/v1alpha1",
            "family_id": self.family_id,
            "mission_id": self.mission_id,
            "fidelity": self.fidelity,
            "initialization_id": self.initialization_id,
            "segments": [
                {
                    "instance_id": self.segment_instance_id,
                    "id": configuration.segment_id,
                    "transition_semantics": "screen_completion_only",
                }
            ],
            "controller_method": self.controller_method,
            "control_realization": "surface_allocated",
            "controller_selection": "discrete_source_node_held_for_each_recovery",
            "node_ids": list(F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS),
            "interior_cases": {identifier: dict(perturbation) for identifier, perturbation in configuration.interior_cases.items()},
            "case_duration_s": self.duration_s,
            "dt_s": self.dt_s,
            "total_recovery_count": len(F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS) * len(configuration.interior_cases),
            "claim_boundary": (
                "This selects two retained body-w recovery cases at each of four source-retrimmed F-16 nodes. "
                f"Each local {self.controller_method.upper()} controller is held at its selected node. It does not execute a time-marching node transition, "
                "continuous gain interpolation, navigation, wind or mass rejection, a full envelope, or flight qualification."
            ),
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class F16PhysicalScheduleInteriorScreenExecution:
    """Public result for the bounded F-16 schedule-interior batch campaign."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: F16PhysicalScheduleInteriorScreenPlan
    output_dir: Path
    runtime: dict[str, object]
    schedule_report: dict[str, object]
    evaluation: dict[str, object]
    status_trace: dict[str, object]
    control_trace: dict[str, object]
    claim_boundary: str

    @property
    def screen_pass(self) -> bool:
        """Return only the bounded schedule-interior disposition."""

        return bool(self.evaluation["mission_pass"])
        ####

    def as_dict(self) -> dict[str, object]:
        """Serialize the exact source-node campaign result."""

        return {
            "schema": f"taoryx.f16-local-physical-surface-{self.plan.controller_method}-schedule-interior-screen-execution/v1alpha1",
            "status": "development_schedule_interior_screen_pass" if self.screen_pass else "development_schedule_interior_screen_failed",
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "plan": self.plan.manifest(),
            "output_dir": str(self.output_dir),
            "runtime": self.runtime,
            "schedule_interior": _schedule_report_summary(self.schedule_report),
            "control_screen": self.evaluation,
            "screen_pass": self.screen_pass,
            "status_trace": status_trace_summary(self.status_trace),
            "control_trace": control_trace_summary(self.control_trace),
            "claim_boundary": self.claim_boundary,
        }
        ####

    ####


def compile_f16_physical_schedule_interior_screen(
    composition: CompiledVehicleComposition,
) -> F16PhysicalScheduleInteriorScreenPlan:
    """Reject compositions that are not the exact bounded F-16 schedule screen."""

    if composition.family_id != "f16_s119":
        raise ValueError("F-16 schedule-interior screen requires the f16_s119 family")
    configuration = _configuration_for_mission(composition.mission)
    if composition.fidelity != "rigid_body_6dof_surface_allocated":
        raise ValueError("F-16 schedule-interior screen requires surface-allocated rigid-body fidelity")
    if composition.initialization.id != _INITIALIZATION_ID:
        raise ValueError(f"F-16 schedule-interior screen requires initialization {_INITIALIZATION_ID!r}")
    if composition.initialization.inputs:
        raise ValueError("F-16 schedule-interior screen does not accept initialization overrides")
    if len(composition.segments) != 1 or composition.segments[0].id != configuration.segment_id:
        raise ValueError(f"F-16 schedule-interior screen requires exactly one {configuration.segment_id!r} segment")
    segment = composition.segments[0]
    if segment.inputs:
        raise ValueError("F-16 schedule-interior screen does not accept segment overrides")
    return F16PhysicalScheduleInteriorScreenPlan(
        family_id=composition.family_id,
        mission_id=composition.mission,
        fidelity=composition.fidelity,
        initialization_id=composition.initialization.id,
        segment_instance_id=segment.instance_id,
        duration_s=_DURATION_S,
        dt_s=_DT_S,
        controller_method=configuration.controller_method,
    )
    ####


class F16PhysicalScheduleInteriorScreenCapabilityAdapter:
    """Advertise the narrow four-node F-16 physical LQR schedule interior."""

    id = _CAPABILITY_ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether this adapter owns the exact schedule-interior screen."""

        return (
            composition.family_id == "f16_s119"
            and composition.mission == _MISSION_ID
            and composition.fidelity == "rigid_body_6dof_surface_allocated"
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Expose node selection, real effectors, and retained boundary cases."""

        plan = compile_f16_physical_schedule_interior_screen(composition)
        nodes = _nodes_for_plan(plan)
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": "surface_allocated",
            "participating_nonlinear_plant": True,
            "source_physical_trim": True,
            "physical_effector_allocation": True,
            "controller_method": plan.controller_method,
            "controller_selection": "discrete_source_node_held_for_each_recovery",
            "schedule_nodes": [
                {
                    "point_id": node.point_id,
                    "altitude_m": node.altitude_m,
                    "true_airspeed_m_s": node.true_airspeed_m_s,
                    "controller_id": node.design.id,
                    "controller_hurwitz": node.design.result.hurwitz,
                    "derivative_consistent": node.design.projection.source_linearization.provenance.derivative_consistent,
                }
                for node in nodes
            ],
            "controlled_state_names": list(nodes[0].design.projection.state_names),
            "controlled_wrench_axes": list(nodes[0].design.projection.wrench_names),
            "effector_names": list(nodes[0].plant.control_names),
            "effector_dynamics": "declared engineering surface/throttle overlay with bounded position, rate, and lag",
            "schedule_interior_cases": {
                identifier: dict(perturbation)
                for identifier, perturbation in _configuration_for_mission(plan.mission_id).interior_cases.items()
            },
            "boundary_cases_retained_in_artifact": [
                "beta_plus_2mps",
                "beta_minus_2mps",
                "high_roll_rate_plus",
                "high_pitch_rate_plus",
                "high_yaw_rate_plus",
                "coupled_high_rate",
            ],
            "physical_screen_execution": {
                "status": "executed_by_this_schedule_interior_screen",
                "mission_id": plan.mission_id,
                "capability_adapter_id": self.id,
                "operations": ["validate", "batch"],
                "control_realization": "surface_allocated",
            },
            "persistent_disturbance_status": "not_executable_without_a_declared_source_wind_or_mass_derivative_environment",
            "claim_boundary": (
                "This batch endpoint executes the retained schedule-wide interior at four source-retrimmed nodes through "
                "actual bounded elevator, aileron, rudder, and throttle allocation. Node selection is discrete and held. "
                "It does not establish continuous gain scheduling, time-marching transition replay, wind/mass rejection, "
                "a full envelope, or flight qualification."
            ),
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                f"four source-retrimmed F-16 physical {plan.controller_method.upper()} nodes and their two retained body-w recovery cases are executable through bounded surface/throttle allocation",
                "node transition replay, continuous gain interpolation, wind, mass variation, and route-level evidence remain separate gates",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####


class F16PhysicalScheduleLqiInteriorScreenCapabilityAdapter(F16PhysicalScheduleInteriorScreenCapabilityAdapter):
    """Advertise only the separately bounded four-node F-16 physical LQI interior."""

    id = _LQI_CAPABILITY_ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether this adapter owns the exact LQI schedule screen."""

        return (
            composition.family_id == "f16_s119"
            and composition.mission == _LQI_MISSION_ID
            and composition.fidelity == "rigid_body_6dof_surface_allocated"
        )
        ####


def preflight_f16_physical_schedule_interior_screen(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Prove that Composition selects the concrete F-16 schedule interior."""

    plan = compile_f16_physical_schedule_interior_screen(composition)
    estimate = estimate_mission_capability(composition)
    if estimate is None or estimate.adapter_id != _configuration_for_mission(plan.mission_id).capability_adapter_id:
        raise ValueError("F-16 schedule-interior screen has no matching installed capability adapter")
    nodes = _nodes_for_plan(plan)
    controller_hurwitz = all(node.design.result.hurwitz for node in nodes)
    derivative_consistent = all(
        node.design.projection.source_linearization.provenance.derivative_consistent for node in nodes
    )
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status="translation_ready" if controller_hurwitz and derivative_consistent else "blocked",
        translator_id=_configuration_for_mission(plan.mission_id).capability_adapter_id,
        checks=(
            ExecutionPreflightCheck(
                f"f16.semantic_local_physical_surface_{plan.controller_method}_schedule_interior_screen",
                [_INITIALIZATION_ID, _configuration_for_mission(plan.mission_id).segment_id],
                [plan.initialization_id, plan.segment_instance_id],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                f"f16.schedule_node_{plan.controller_method}_controllers_hurwitz",
                True,
                controller_hurwitz,
                None,
                controller_hurwitz,
            ),
            ExecutionPreflightCheck(
                "f16.schedule_node_source_derivatives_consistent",
                True,
                derivative_consistent,
                None,
                derivative_consistent,
            ),
        ),
        diagnostics=(
            f"composition lowers exactly to four independent source-node physical {plan.controller_method.upper()} recoveries; it is not a continuous scheduler or route translator",
            *estimate.diagnostics,
        ),
        derived_mission=estimate.manifest,
        capability_estimate=build_concrete_capability_preflight_evidence(composition, estimate),
    )
    ####


def execute_f16_physical_schedule_interior_screen(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None = None,
) -> F16PhysicalScheduleInteriorScreenExecution:
    """Execute every retained interior case through actual F-16 effectors."""

    if max_steps is not None:
        raise ValueError("--max-steps is unavailable for the fixed-cadence F-16 schedule-interior screen")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        details = "; ".join(preflight.diagnostics) or "no translation-ready F-16 schedule-interior screen"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {details}")
    plan = compile_f16_physical_schedule_interior_screen(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    reports: list[dict[str, object]] = []
    rows: list[dict[str, float | int | str]] = []
    time_offset_s = 0.0
    configuration = _configuration_for_mission(plan.mission_id)
    nodes = _nodes_for_plan(plan)
    for node in nodes:
        node_cases: list[dict[str, object]] = []
        for case_id, perturbation in configuration.interior_cases.items():
            initial_state = dict(node.trim.state)
            initial_state.update(
                {name: float(initial_state[name]) + value for name, value in perturbation.items()}
            )
            validation: PhysicalWrenchLqrValidation | PhysicalWrenchLqiValidation
            if plan.controller_method == "lqr":
                if not isinstance(node, F16SourcePhysicalScheduleNode):
                    raise TypeError("F-16 LQR schedule selected an LQI node")
                validation = validate_nonlinear_wrench_lqr(
                    node.plant,
                    node.trim,
                    node.design,
                    initial_state=initial_state,
                    duration_s=plan.duration_s,
                    dt_s=plan.dt_s,
                )
            else:
                if not isinstance(node, F16SourcePhysicalScheduleLqiNode):
                    raise TypeError("F-16 LQI schedule selected an LQR node")
                validation = validate_nonlinear_wrench_lqi(
                    node.plant,
                    node.trim,
                    node.design,
                    initial_state=initial_state,
                    duration_s=plan.duration_s,
                    dt_s=plan.dt_s,
                    integral_lower={name: -1.0 for name in node.design.result.output_names},
                    integral_upper={name: 1.0 for name in node.design.result.output_names},
                )
            assessment = _assess_case(validation)
            node_cases.append(
                {
                    "case_id": case_id,
                    "perturbation": dict(perturbation),
                    "result": assessment,
                    "validation": validation.as_dict(),
                }
            )
            rows.extend(_rows(validation, node, case_id, time_offset_s))
            time_offset_s += plan.duration_s
        reports.append(
            {
                "point_id": node.point_id,
                "altitude_m": node.altitude_m,
                "true_airspeed_m_s": node.true_airspeed_m_s,
                "controller_id": node.design.id,
                "controller_hurwitz": node.design.result.hurwitz,
                "derivative_consistent": node.design.projection.source_linearization.provenance.derivative_consistent,
                "cases": node_cases,
            }
        )
    schedule_report = _schedule_report(reports, plan)
    evaluation = _evaluation(schedule_report, plan)
    status_trace = build_committed_status_trace(composition, _status_samples(rows, plan.controller_method))
    control_trace = build_committed_control_trace(composition, _control_samples(rows))
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.fixed_wing.daveml.v1",
        "controller_method": plan.controller_method,
        "integral_output_names": (
            list(nodes[0].design.result.output_names)
            if nodes and isinstance(nodes[0], F16SourcePhysicalScheduleLqiNode)
            else []
        ),
        "controller_selection": "discrete_source_node_held_for_each_recovery",
        "control_realization": "surface_allocated",
        "physical_effector_allocation": True,
        "schedule_node_count": len(reports),
        "schedule_case_count": len(reports) * len(configuration.interior_cases),
        "dt_s": plan.dt_s,
        "case_duration_s": plan.duration_s,
        "total_executed_duration_s": time_offset_s,
        "hard_gates_passed": evaluation["mission_pass"],
        "navigation_state": "independent_fixed_source_trim_local_recoveries",
        "resource_ledger": resource_ledger_summary(resource_ledger),
        "mission_graph_execution": unobserved_mission_graph_execution(
            composition,
            "The F-16 schedule-interior screen executes independent local recoveries and makes no route or node-transition claim.",
        ).as_dict(),
    }
    result = F16PhysicalScheduleInteriorScreenExecution(
        composition=composition,
        preflight=preflight,
        plan=plan,
        output_dir=destination,
        runtime=runtime,
        schedule_report=schedule_report,
        evaluation=evaluation,
        status_trace=status_trace,
        control_trace=control_trace,
        claim_boundary=(
            f"This is a batch campaign of eight independent F-16 source-node physical {plan.controller_method.upper()} recoveries: two retained "
            "body-w cases at each of four altitudes. It does not execute between-node state transfer, gain interpolation, "
            "navigation, wind/mass rejection, a full envelope, or flight qualification."
        ),
    )
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "mission_graph_execution.json", runtime["mission_graph_execution"])
    _write_json(destination / "schedule_interior_report.json", schedule_report)
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
            envelope=schedule_report,
            claim_boundary=result.claim_boundary,
            status_trace=status_trace,
            control_trace=control_trace,
        ).as_dict(),
    )
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _assess_case(validation: PhysicalWrenchLqrValidation | PhysicalWrenchLqiValidation) -> dict[str, object]:
    """Apply the source envelope's retained interior rule without clipping limits."""

    recovery_ratio = validation.final_normalized_feedback_error_norm / max(
        validation.initial_normalized_feedback_error_norm,
        1.0e-12,
    )
    disallowed = {"partially_achievable", "infeasible", "numerically_singular", "solver_failure"}
    allocation_statuses = set(validation.allocation_statuses)
    checks = {
        "feedback_recovery": recovery_ratio <= _RECOVERY_THRESHOLD,
        "allocation_status": not bool(allocation_statuses & disallowed),
        "no_saturation": validation.saturation_fraction == 0.0,
        "finite_final_state": all(math.isfinite(value) for value in validation.final_state.values()),
    }
    return {
        "mission_pass": all(checks.values()),
        "checks": checks,
        "recovery_ratio": recovery_ratio,
        "allocation_statuses": list(validation.allocation_statuses),
        "saturation_fraction": validation.saturation_fraction,
        "maximum_controlled_actual_residual": validation.maximum_controlled_actual_residual,
        "maximum_continuous_saturation_duration_s": validation.maximum_continuous_saturation_duration_s,
    }
    ####


def _schedule_report(reports: list[dict[str, object]], plan: F16PhysicalScheduleInteriorScreenPlan) -> dict[str, object]:
    """Build the complete node-by-node, case-by-case physical evidence record."""

    cases: list[dict[str, object]] = []
    for node in reports:
        node_cases = node.get("cases")
        if not isinstance(node_cases, list):
            continue
        cases.extend(case for case in node_cases if isinstance(case, dict))
    passed_count = sum(bool(cast_mapping(case["result"], "schedule case result")["mission_pass"]) for case in cases)
    return {
        "schema": f"taoryx.f16-physical-{plan.controller_method}-schedule-interior-screen/v1alpha1",
        "status": "pass" if passed_count == len(cases) else "failed",
        "controller_method": plan.controller_method,
        "controller_selection": "discrete_source_node_held_for_each_recovery",
        "node_ids": list(F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS),
        "interior_cases": {
            identifier: dict(perturbation)
            for identifier, perturbation in _configuration_for_mission(plan.mission_id).interior_cases.items()
        },
        "case_contract": {
            "duration_s": plan.duration_s,
            "dt_s": plan.dt_s,
            "recovery_threshold": _RECOVERY_THRESHOLD,
            "failure_policy": "a constrained or poor-recovery retained-interior case fails; no request clipping converts it to a pass",
        },
        "nodes": reports,
        "summary": {
            "node_count": len(reports),
            "case_count": len(cases),
            "passed_case_count": passed_count,
            "failed_case_count": len(cases) - passed_count,
        },
        "claim_boundary": (
            "The retained cases are a schedule-wide local source-node interior. This is not a continuous gain-schedule, "
            "a node-transition replay, a Mach/dynamic-pressure envelope, wind robustness, servo certification, statistical "
            "reliability, or operational-flight result."
        ),
    }
    ####


def _evaluation(schedule_report: Mapping[str, object], plan: F16PhysicalScheduleInteriorScreenPlan) -> dict[str, object]:
    """Expose the exact local schedule-screen gates as public evaluation rows."""

    summary = cast_mapping(schedule_report.get("summary"), "schedule summary")
    passed = schedule_report.get("status") == "pass"
    return {
        "schema": f"taoryx.f16-local-physical-surface-{plan.controller_method}-schedule-interior-screen-evaluation/v1alpha1",
        "kind": "local_physical_schedule_interior_screen",
        "mission_pass": passed,
        "results": [
            {"id": "all_retained_cases_pass", "status": "pass" if passed else "fail", "required": True},
            {
                "id": "four_source_nodes_exercised",
                "status": "pass" if summary["node_count"] == len(F16_SOURCE_PHYSICAL_SCHEDULE_POINT_IDS) else "fail",
                "required": True,
            },
        ],
        "metrics": dict(summary),
        "case_duration_s": plan.duration_s,
        "controller_method": plan.controller_method,
        "control_realization": "surface_allocated",
        "claim_boundary": (
            f"These gates apply only to independent, held-node physical {plan.controller_method.upper()} recoveries for the two retained schedule-wide "
            "interior perturbations. They are not gain-schedule, transition, route, robustness, or flight-qualification objectives."
        ),
    }
    ####


def _schedule_report_summary(schedule_report: Mapping[str, object]) -> dict[str, object]:
    """Return a compact public reference to the complete node/case artifact."""

    return {
        "schema": schedule_report["schema"],
        "status": schedule_report["status"],
        "controller_selection": schedule_report["controller_selection"],
        "node_ids": schedule_report["node_ids"],
        "interior_cases": schedule_report["interior_cases"],
        "summary": schedule_report["summary"],
        "artifact": "schedule_interior_report.json",
        "claim_boundary": schedule_report["claim_boundary"],
    }
    ####


def _rows(
    validation: PhysicalWrenchLqrValidation | PhysicalWrenchLqiValidation,
    node: F16SourcePhysicalScheduleNode | F16SourcePhysicalScheduleLqiNode,
    case_id: str,
    time_offset_s: float,
) -> list[dict[str, float | int | str]]:
    """Flatten committed physical allocation telemetry for one node/case recovery."""

    rows: list[dict[str, float | int | str]] = []
    for sample in validation.samples:
        payload = sample.as_dict()
        state = cast_mapping(payload.get("state"), "physical schedule sample state")
        requested = cast_mapping(payload.get("requested_wrench"), "physical schedule requested wrench")
        achieved = cast_mapping(payload.get("achieved_wrench"), "physical schedule achieved wrench")
        actual = cast_mapping(payload.get("actual_effectors"), "physical schedule actual effectors")
        position_saturated = cast_strings(payload.get("position_saturated"), "physical schedule position saturation")
        rate_limited = cast_strings(payload.get("rate_limited"), "physical schedule rate limitation")
        u, v, w = (number(state, name) for name in ("u_m_s", "v_m_s", "w_m_s"))
        rows.append(
            {
                "time_s": time_offset_s + number(payload, "time_s"),
                "schedule_node_id": node.point_id,
                "schedule_case_id": case_id,
                "u_m_s": u,
                "v_m_s": v,
                "w_m_s": w,
                "p_rad_s": number(state, "p_rad_s"),
                "q_rad_s": number(state, "q_rad_s"),
                "r_rad_s": number(state, "r_rad_s"),
                "north_m": 0.0,
                "east_m": 0.0,
                "altitude_m": node.altitude_m,
                "speed_m_s": math.sqrt(u * u + v * v + w * w),
                "local_roll_deg": 0.0,
                "local_pitch_deg": math.degrees(node.trim_pitch_rad),
                "local_heading_deg": 0.0,
                "requested_force_x_n": number(requested, "total_force_x_n"),
                "requested_moment_x_nm": number(requested, "total_moment_x_nm"),
                "requested_moment_y_nm": number(requested, "total_moment_y_nm"),
                "requested_moment_z_nm": number(requested, "total_moment_z_nm"),
                "achieved_force_x_n": number(achieved, "total_force_x_n"),
                "achieved_moment_x_nm": number(achieved, "total_moment_x_nm"),
                "achieved_moment_y_nm": number(achieved, "total_moment_y_nm"),
                "achieved_moment_z_nm": number(achieved, "total_moment_z_nm"),
                "residual_force_x_n": number(requested, "total_force_x_n") - number(achieved, "total_force_x_n"),
                "residual_moment_x_nm": number(requested, "total_moment_x_nm") - number(achieved, "total_moment_x_nm"),
                "residual_moment_y_nm": number(requested, "total_moment_y_nm") - number(achieved, "total_moment_y_nm"),
                "residual_moment_z_nm": number(requested, "total_moment_z_nm") - number(achieved, "total_moment_z_nm"),
                "elevator_deg": number(actual, "elevator_deg"),
                "aileron_deg": number(actual, "aileron_deg"),
                "rudder_deg": number(actual, "rudder_deg"),
                "throttle_fraction": number(actual, "throttle_fraction"),
                "allocation_status": text(payload, "allocation_status"),
                "saturation_count": len(position_saturated) + len(rate_limited),
                "allocation_controlled_residual_norm": number(payload, "achieved_controlled_residual_norm"),
                "mass_kg": float(node.plant.source.mass_kg),
            }
        )
    return rows
    ####


def _status_samples(
    rows: list[dict[str, float | int | str]],
    controller_method: str,
) -> tuple[BatchTruthSample, ...]:
    """Project every multi-node physical row through the common F-16 contract."""

    samples: list[BatchTruthSample] = []
    for index, row in enumerate(rows):
        raw: dict[str, object] = dict(row)
        raw.update(
            {
                "body_velocity_m_s": [number(row, name) for name in ("u_m_s", "v_m_s", "w_m_s")],
                "body_rate_rad_s": [number(row, name) for name in ("p_rad_s", "q_rad_s", "r_rad_s")],
                "attitude_euler_deg": [number(row, name) for name in ("local_roll_deg", "local_pitch_deg", "local_heading_deg")],
                "requested_force_body_n": [number(row, "requested_force_x_n"), 0.0, 0.0],
                "requested_moment_body_nm": [number(row, name) for name in ("requested_moment_x_nm", "requested_moment_y_nm", "requested_moment_z_nm")],
                "achieved_force_body_n": [number(row, "achieved_force_x_n"), 0.0, 0.0],
                "achieved_moment_body_nm": [number(row, name) for name in ("achieved_moment_x_nm", "achieved_moment_y_nm", "achieved_moment_z_nm")],
                "residual_force_body_n": [number(row, "residual_force_x_n"), 0.0, 0.0],
                "residual_moment_body_nm": [number(row, name) for name in ("residual_moment_x_nm", "residual_moment_y_nm", "residual_moment_z_nm")],
                "allocation_residual_norm": number(row, "allocation_controlled_residual_norm"),
                "wrench_status": text(row, "allocation_status"),
                "wrench_saturated": int(row["saturation_count"]) > 0,
                "physical_effector_allocation": True,
                "control_realization": "surface_allocated",
                "controller_method": controller_method,
                "controller_selection": "discrete_source_node_held_for_each_recovery",
            }
        )
        samples.append(
            BatchTruthSample(
                time_s=number(row, "time_s"),
                execution_status="completed" if index == len(rows) - 1 else "active",
                raw_values=raw,
            )
        )
    return tuple(samples)
    ####


def _control_samples(rows: list[dict[str, float | int | str]]) -> tuple[BatchControlSample, ...]:
    """Retain actual F-16 overlay positions at every committed boundary."""

    samples: list[BatchControlSample] = []
    previous_time: float | None = None
    for row in rows:
        time_s = number(row, "time_s")
        samples.append(
            BatchControlSample(
                interval_start_time_s=time_s if previous_time is None else previous_time,
                committed_truth_time_s=time_s,
                requested_actions={},
                achieved_effectors={
                    "effector.elevator.position": number(row, "elevator_deg"),
                    "effector.aileron.position": number(row, "aileron_deg"),
                    "effector.rudder.position": number(row, "rudder_deg"),
                    "effector.throttle.position": number(row, "throttle_fraction"),
                },
            )
        )
        previous_time = time_s
    return tuple(samples)
    ####


def cast_mapping(value: object, label: str) -> Mapping[str, object]:
    """Read a required telemetry mapping."""

    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value
    ####


def cast_strings(value: object, label: str) -> tuple[str, ...]:
    """Read a required saturated-channel list."""

    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ValueError(f"{label} must be a list of strings")
    return tuple(value)
    ####


def number(values: Mapping[str, object], name: str) -> float:
    """Read a required finite scalar from a source-owned row."""

    value = values.get(name)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"F-16 physical schedule telemetry {name!r} must be finite numeric")
    return float(value)
    ####


def text(values: Mapping[str, object], name: str) -> str:
    """Read a required nonempty source-owned text value."""

    value = values.get(name)
    if not isinstance(value, str) or not value:
        raise ValueError(f"F-16 physical schedule telemetry {name!r} must be nonempty text")
    return value
    ####


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    if not rows:
        raise ValueError("F-16 physical schedule screen produced no telemetry rows")
    fields = sorted({field for row in rows for field in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    ####


__all__ = [
    "F16PhysicalScheduleInteriorScreenCapabilityAdapter",
    "F16PhysicalScheduleInteriorScreenExecution",
    "F16PhysicalScheduleInteriorScreenPlan",
    "compile_f16_physical_schedule_interior_screen",
    "execute_f16_physical_schedule_interior_screen",
    "preflight_f16_physical_schedule_interior_screen",
]
