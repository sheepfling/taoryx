"""Composition endpoint for the B747 condition-3 physical surface LQR screen.

This endpoint lowers exactly one NASA CR-2144 condition-3 local recovery.  It
uses the source-table elevator, aileron, rudder, and throttle coordinates and
the bounded nonlinear allocator.  It deliberately does not translate that
screen into a gain schedule, transport racetrack, servo model, or flight
qualification claim.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from .claim_bound_evidence import bind_release_evidence
from .composition_control_trace import BatchControlSample, build_committed_control_trace, control_trace_summary
from .composition_evaluation import build_composition_trajectory_evaluation
from .composition_graph_evidence import unobserved_mission_graph_execution
from .composition_resource_ledger import build_committed_resource_ledger, resource_ledger_summary
from .composition_sensor_trace import BatchTruthSample
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .mission_capability import MissionCapabilityEstimate, estimate_mission_capability
from .physical_lqr import (
    PhysicalWrenchLqiDesign,
    PhysicalWrenchLqiValidation,
    PhysicalWrenchLqrDesign,
    PhysicalWrenchLqrValidation,
    apply_tuning_context_to_physical_wrench_lqi_design,
    validate_nonlinear_wrench_lqi,
    validate_nonlinear_wrench_lqr,
)
from .source_table_fixed_wing import (
    build_b747_condition3_source_surface_physical_lqi_design,
    build_b747_condition3_source_surface_physical_lqr_design,
    build_b747_condition3_source_table_plant,
)
from .trim import TrimResult
from .tuning_application import TuningApplicationContext
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    VehicleExecutionPreflight,
    build_concrete_capability_preflight_evidence,
    preflight_vehicle_composition,
)

_INITIALIZATION_ID = "condition3_source_table_trim_local_point"
_STANDARD_LQI_SCREEN_MISSION_ID = "b747_condition3_local_physical_surface_lqi_screen_v1"
_DURATION_S = 80.0
_DT_S = 0.05
_MATCHED_PITCH_WRENCH_BIAS_FRACTION = 0.05
LocalControllerMethod = Literal["lqr", "lqi"]
_SCREEN_BY_MISSION: dict[str, tuple[LocalControllerMethod, str, str]] = {
    "b747_condition3_local_physical_surface_lqr_screen_v1": (
        "lqr",
        "condition3_source_table_physical_lqr_screen",
        "taoryx.b747_condition3_local_physical_surface_lqr_screen.capability.v1",
    ),
    _STANDARD_LQI_SCREEN_MISSION_ID: (
        "lqi",
        "condition3_source_table_physical_lqi_screen",
        "taoryx.b747_condition3_local_physical_surface_lqi_screen.capability.v1",
    ),
}
_STATE_NAMES = (
    "roll_error_rad",
    "pitch_error_rad",
    "yaw_error_rad",
    "u_m_s",
    "v_m_s",
    "w_m_s",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
)


@dataclass(frozen=True, slots=True)
class B747LocalPhysicalControlScreenPlan:
    """Fixed lowering record for one B747 source-table recovery screen."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_instance_id: str
    controller_method: LocalControllerMethod
    duration_s: float
    dt_s: float

    def manifest(self) -> dict[str, object]:
        """Return the concrete local screen selected by this composition."""

        return {
            "schema": "taoryx.b747-condition3-local-physical-surface-control-screen-plan/v1alpha1",
            "family_id": self.family_id,
            "mission_id": self.mission_id,
            "fidelity": self.fidelity,
            "initialization_id": self.initialization_id,
            "segments": [
                {
                    "instance_id": self.segment_instance_id,
                    "id": _screen_definition(self.mission_id)[1],
                    "transition_semantics": "screen_completion_only",
                }
            ],
            "duration_s": self.duration_s,
            "dt_s": self.dt_s,
            "controller_method": self.controller_method,
            "control_realization": _control_realization(self.controller_method),
            "claim_boundary": (
                "This selects one B747 NASA CR-2144 condition-3 local attitude recovery. It does not execute a "
                "racetrack, gain schedule, servo-dynamics model, or transport flight-controller validation."
            ),
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class B747LocalPhysicalControlScreenExecution:
    """Public result of one B747 nonlinear physical-control screen."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: B747LocalPhysicalControlScreenPlan
    output_dir: Path
    runtime: dict[str, object]
    evaluation: dict[str, object]
    status_trace: dict[str, object]
    control_trace: dict[str, object]
    claim_boundary: str

    @property
    def screen_pass(self) -> bool:
        """Return the narrow local-screen disposition only."""

        return bool(self.evaluation["mission_pass"])
        ####

    def as_dict(self) -> dict[str, object]:
        """Serialize the compact public execution result."""

        return {
            "schema": "taoryx.b747-condition3-local-physical-surface-control-screen-execution/v1alpha1",
            "status": "development_local_screen_pass" if self.screen_pass else "development_local_screen_failed",
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "plan": self.plan.manifest(),
            "output_dir": str(self.output_dir),
            "runtime": self.runtime,
            "control_screen": self.evaluation,
            "screen_pass": self.screen_pass,
            "status_trace": status_trace_summary(self.status_trace),
            "control_trace": control_trace_summary(self.control_trace),
            "claim_boundary": self.claim_boundary,
        }
        ####

    ####


def compile_b747_condition3_local_physical_control_screen(
    composition: CompiledVehicleComposition,
) -> B747LocalPhysicalControlScreenPlan:
    """Fail closed unless Composition selected the exact condition-3 screen."""

    if composition.family_id != "b747":
        raise ValueError("B747 physical control screen requires the b747 family")
    if composition.mission not in _SCREEN_BY_MISSION:
        choices = ", ".join(sorted(_SCREEN_BY_MISSION))
        raise ValueError(f"B747 physical control screen requires one of: {choices}")
    if composition.fidelity != "rigid_body_6dof_surface_allocated":
        raise ValueError("B747 physical control screen requires surface-allocated rigid-body fidelity")
    if composition.initialization.id != _INITIALIZATION_ID:
        raise ValueError(f"B747 physical control screen requires initialization {_INITIALIZATION_ID!r}")
    if composition.initialization.inputs:
        raise ValueError("B747 physical control screen does not accept initialization overrides")
    controller_method, segment_id, _ = _screen_definition(composition.mission)
    if len(composition.segments) != 1 or composition.segments[0].id != segment_id:
        raise ValueError(f"B747 physical control screen requires exactly one {segment_id!r} segment")
    segment = composition.segments[0]
    if segment.inputs:
        raise ValueError("B747 physical control screen does not accept segment overrides")
    return B747LocalPhysicalControlScreenPlan(
        family_id=composition.family_id,
        mission_id=composition.mission,
        fidelity=composition.fidelity,
        initialization_id=composition.initialization.id,
        segment_instance_id=segment.instance_id,
        controller_method=controller_method,
        duration_s=_DURATION_S,
        dt_s=_DT_S,
    )
    ####


class B747LocalPhysicalControlScreenCapabilityAdapter:
    """Advertise one exact B747 source-table controller/allocator route."""

    id = "taoryx.b747_condition3_local_physical_surface_lqr_screen.capability.v1"
    controller_method: LocalControllerMethod = "lqr"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether this adapter owns the selected composition."""

        return (
            composition.family_id == "b747"
            and composition.mission == _mission_id_for(self.controller_method)
            and composition.fidelity == "rigid_body_6dof_surface_allocated"
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Expose controlled axes, effectors, and its physical boundaries."""

        plan = compile_b747_condition3_local_physical_control_screen(composition)
        plant = build_b747_condition3_source_table_plant()
        design = _design_for(self.controller_method)
        lqi_design = build_b747_condition3_source_surface_physical_lqi_design()
        manifest = plan.manifest()
        matched_offset_status = (
            "executed_by_this_screen"
            if plan.mission_id == _STANDARD_LQI_SCREEN_MISSION_ID
            else "executed_by_paired_standard_lqi_screen"
        )
        manifest["capability"] = {
            "control_realization": _control_realization(self.controller_method),
            "participating_nonlinear_plant": True,
            "source_physical_trim": True,
            "navigation_guidance": False,
            "screen_duration_s": plan.duration_s,
            "integration_dt_s": plan.dt_s,
            "controller_id": design.id,
            "controller_method": self.controller_method,
            "controller_hurwitz": design.result.hurwitz,
            "controlled_state_names": list(design.projection.state_names),
            "controlled_wrench_axes": list(design.projection.wrench_names),
            "effector_names": list(plant.control_names),
            "physical_effector_allocation": True,
            "effector_dynamics": "ideal bounded source-table coordinates; no source servo rate or lag data",
            "offset_free_tuning_candidate": {
                "campaign_id": "b747-source-surface-local-lqi-v1",
                "method": "lqi",
                "availability": "available_through_model_tune",
                "controller_id": lqi_design.id,
                "controller_hurwitz": lqi_design.result.hurwitz,
                "controlled_state_names": list(lqi_design.projection.state_names),
                "controlled_wrench_axes": list(lqi_design.projection.wrench_names),
                "integral_output_names": list(lqi_design.result.output_names),
                "integral_q_diagonal": list(lqi_design.integral_q_diagonal),
                "controller_automation": {
                    "integral_priority_grid": {
                        "base_integral_q_diagonal": list(lqi_design.integral_q_diagonal),
                        "multipliers": [0.1, 1.0, 10.0, 100.0],
                        "claim_boundary": (
                            "The common campaign sweeps source-coordinate LQI candidates as a design aid. Its "
                            "candidate coordinates are not represented as the physical-wrench allocator runtime binding."
                        ),
                    },
                    "physical_wrench_profile": {
                        "integral_q_diagonal": list(lqi_design.integral_q_diagonal),
                        "selection_evidence": (
                            "the standard LQI batch emits a three-case matched external pitch-moment screen"
                        ),
                        "claim_boundary": (
                            "This is one discrete physical-wrench LQI profile at the pinned condition-3 trim, not a "
                            "B747 gain schedule or adaptive controller."
                        ),
                    },
                },
                "physical_allocation_baseline": "focused_bounded_source_local_recovery_passed",
                "physical_screen_status": (
                    "executed_by_this_lqi_screen"
                    if self.controller_method == "lqi"
                    else "not_executed_by_this_lqr_screen"
                ),
                "physical_screen_execution": {
                    "status": (
                        "executed_by_this_lqi_screen"
                        if self.controller_method == "lqi"
                        else "executed_by_paired_lqi_screen"
                    ),
                    "mission_id": _mission_id_for("lqi"),
                    "capability_adapter_id": "taoryx.b747_condition3_local_physical_surface_lqi_screen.capability.v1",
                    "operations": ["validate", "batch"],
                    "control_realization": "source_table_physical_wrench_lqi_allocation",
                },
                "persistent_disturbance_status": matched_offset_status,
                "persistent_disturbance_screen": {
                    "id": "b747-local-lqi-matched-pitch-wrench-offset",
                    "status": matched_offset_status,
                    "environment_input": "external_pitch_moment_bias_nm",
                    "body_moment_axis": "moment_y_nm",
                    "fraction_of_declared_pitch_wrench_scale": _MATCHED_PITCH_WRENCH_BIAS_FRACTION,
                    "execution_mission_id": _STANDARD_LQI_SCREEN_MISSION_ID,
                    "artifact_filename": "robustness_report.json",
                },
                "claim_boundary": (
                    "The public condition-3 batch screen executes its declared controller through bounded "
                    "source-table allocation. The named LQI design has a focused local recovery baseline and the "
                    "standard LQI screen exercises a constant matched external pitch moment through an explicit plant-"
                    "dynamics seam. It does not establish wind or mass robustness, a B747 gain schedule, or transport "
                    "controller qualification."
                ),
            },
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                "pinned B747 condition-3 source trim, plant-derived three-axis controller, bounded elevator/aileron/rudder "
                "allocation, nonlinear eighty-second recovery, and a matched pitch-offset screen are available; schedule, route, and servo dynamics remain separate gates",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####


class B747LocalPhysicalLqiControlScreenCapabilityAdapter(B747LocalPhysicalControlScreenCapabilityAdapter):
    """Advertise the explicit offset-free B747 condition-3 source-table screen."""

    id = "taoryx.b747_condition3_local_physical_surface_lqi_screen.capability.v1"
    controller_method: LocalControllerMethod = "lqi"

    ####


def preflight_b747_condition3_local_physical_control_screen(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Prove that the selected composition lowers to the concrete B747 path."""

    plan = compile_b747_condition3_local_physical_control_screen(composition)
    estimate = estimate_mission_capability(composition)
    _, segment_id, capability_adapter_id = _screen_definition(composition.mission)
    if estimate is None or estimate.adapter_id != capability_adapter_id:
        raise ValueError("B747 physical control screen has no matching installed capability adapter")
    capability = estimate.manifest.get("capability")
    if not isinstance(capability, Mapping):
        raise ValueError("B747 physical control capability record is missing")
    controller_hurwitz = capability.get("controller_hurwitz") is True
    design = _design_for(plan.controller_method)
    derivative_consistent = design.projection.source_linearization.provenance.derivative_consistent
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status="translation_ready" if controller_hurwitz and derivative_consistent else "blocked",
        translator_id=capability_adapter_id,
        checks=(
            ExecutionPreflightCheck(
                f"b747.semantic_condition3_local_physical_surface_{plan.controller_method}_screen",
                [_INITIALIZATION_ID, segment_id],
                [plan.initialization_id, plan.segment_instance_id],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                "b747.local_physical_wrench_controller_hurwitz",
                True,
                controller_hurwitz,
                None,
                controller_hurwitz,
            ),
            ExecutionPreflightCheck(
                "b747.source_table_derivative_consistency",
                True,
                derivative_consistent,
                None,
                derivative_consistent,
            ),
        ),
        diagnostics=(
            "composition lowers exactly to the B747 condition-3 local physical surface-allocation screen; it is not a route or scheduled-envelope translator",
            *estimate.diagnostics,
        ),
        derived_mission=estimate.manifest,
        capability_estimate=build_concrete_capability_preflight_evidence(composition, estimate),
    )
    ####


def execute_b747_condition3_local_physical_control_screen(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None = None,
    *,
    tuning_context: TuningApplicationContext | None = None,
) -> B747LocalPhysicalControlScreenExecution:
    """Run the B747 nonlinear source-table plant through bounded allocation."""

    if max_steps is not None:
        raise ValueError("--max-steps is unavailable for the fixed-cadence B747 physical-control screen")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        details = "; ".join(preflight.diagnostics) or "no translation-ready B747 local physical screen"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {details}")
    plan = compile_b747_condition3_local_physical_control_screen(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    plant = build_b747_condition3_source_table_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"B747 condition-3 physical-control screen trim failed: {trim.as_dict()}")
    design = _design_for(plan.controller_method)
    tuning_binding = None
    if tuning_context is not None:
        if not isinstance(design, PhysicalWrenchLqiDesign):
            raise ValueError("the B747 LQI tuning context can only be applied to an LQI screen")
        design, tuning_binding = apply_tuning_context_to_physical_wrench_lqi_design(design, tuning_context)
    initial_state = _initial_state(trim)
    validation: PhysicalWrenchLqrValidation | PhysicalWrenchLqiValidation
    if plan.controller_method == "lqr":
        lqr_design = design if isinstance(design, PhysicalWrenchLqrDesign) else build_b747_condition3_source_surface_physical_lqr_design()
        validation = validate_nonlinear_wrench_lqr(
            plant,
            trim,
            lqr_design,
            initial_state=initial_state,
            duration_s=plan.duration_s,
            dt_s=plan.dt_s,
        )
    else:
        if not isinstance(design, PhysicalWrenchLqiDesign):
            raise RuntimeError("the B747 LQI screen requires an LQI physical-wrench design")
        lqi_design = design
        validation = validate_nonlinear_wrench_lqi(
            plant,
            trim,
            lqi_design,
            initial_state=initial_state,
            duration_s=plan.duration_s,
            dt_s=plan.dt_s,
            integral_lower={name: -0.5 for name in lqi_design.result.output_names},
            integral_upper={name: 0.5 for name in lqi_design.result.output_names},
        )
    assessment = _assess(validation)
    robustness_report: dict[str, object] | None = None
    if plan.mission_id == _STANDARD_LQI_SCREEN_MISSION_ID:
        if not isinstance(validation, PhysicalWrenchLqiValidation) or not isinstance(design, PhysicalWrenchLqiDesign):
            raise RuntimeError("the B747 matched pitch-offset screen requires the LQI validation path")
        robustness_report = _matched_pitch_wrench_offset_report(
            plant,
            trim,
            design,
            plan,
            nominal_validation=validation,
            nominal_assessment=assessment,
        )
    rows = _rows(validation)
    envelope = _source_table_envelope(rows)
    mass_kg = float(plant.source_state.mass)
    rows = [{**row, "mass_kg": mass_kg} for row in rows]
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.fixed_wing.source_table.v1",
        "controller_id": design.id,
        "controller_method": plan.controller_method,
        "integral_output_names": list(design.result.output_names) if isinstance(design, PhysicalWrenchLqiDesign) else [],
        "integral_q_diagonal": list(design.integral_q_diagonal) if isinstance(design, PhysicalWrenchLqiDesign) else [],
        "control_realization": _control_realization(plan.controller_method),
        "physical_effector_allocation": True,
        "controlled_state_names": list(design.projection.state_names),
        "controlled_wrench_axes": list(design.projection.wrench_names),
        "effector_names": list(plant.control_names),
        "effector_dynamics": "ideal bounded source-table coordinates; no source servo rate or lag data",
        "dt_s": plan.dt_s,
        "duration_s": plan.duration_s,
        "mass_kg": mass_kg,
        "source_table_envelope": envelope,
        "hard_gates_passed": assessment["screen_pass"],
        **({"tuning_binding": tuning_binding.as_dict()} if tuning_binding is not None else {}),
        "matched_pitch_offset_screen": (
            {
                "status": "applied",
                "id": "b747-local-lqi-matched-pitch-wrench-offset",
                "artifact_filename": "robustness_report.json",
                "pass": robustness_report["pass"],
                "claim_boundary": "Only the standard LQI endpoint runs this fixed three-case condition-3 screen.",
            }
            if robustness_report is not None
            else {"status": "not_selected"}
        ),
        "mission_graph_execution": unobserved_mission_graph_execution(
            composition,
            "The B747 condition-3 physical control screen has no route graph dispatcher and makes no mission-transition claim.",
        ).as_dict(),
    }
    evaluation = _evaluation(assessment, plan)
    status_trace = build_committed_status_trace(composition, _status_samples(rows, mass_kg, plan.controller_method))
    control_trace = build_committed_control_trace(composition, _control_samples(rows))
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    runtime["resource_ledger"] = resource_ledger_summary(resource_ledger)
    result = B747LocalPhysicalControlScreenExecution(
        composition=composition,
        preflight=preflight,
        plan=plan,
        output_dir=destination,
        runtime=runtime,
        evaluation=evaluation,
        status_trace=status_trace,
        control_trace=control_trace,
        claim_boundary=(
            "This is an eighty-second B747 NASA CR-2144 condition-3 local attitude recovery. Requested moments are "
            "allocated through bounded elevator, aileron, and rudder source-table coordinates; throttle stays at the "
            "trim coordinate. The standard LQI endpoint also screens a fixed matched external pitch-moment offset through "
            "the plant dynamics. It does not prove a racetrack, gain schedule, wind or mass robustness, servo dynamics, "
            "engine dynamics, or flight qualification."
        ),
    )
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "mission_graph_execution.json", runtime["mission_graph_execution"])
    _write_json(destination / "nonlinear_validation.json", validation.as_dict())
    if robustness_report is not None:
        _write_json(
            destination / "robustness_report.json",
            bind_release_evidence(robustness_report, kind="robustness", composition=composition),
        )
    _write_json(destination / "envelope_report.json", envelope)
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
            envelope=envelope,
            claim_boundary=result.claim_boundary,
            status_trace=status_trace,
            control_trace=control_trace,
        ).as_dict(),
    )
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _initial_state(trim: TrimResult) -> dict[str, float]:
    """Return the pinned condition-3 local perturbation for every LQI case."""

    state = dict(trim.state)
    state.update(
        {
            "roll_error_rad": math.radians(1.0),
            "pitch_error_rad": math.radians(-0.5),
            "yaw_error_rad": math.radians(1.0),
            "p_rad_s": math.radians(0.5),
            "q_rad_s": math.radians(-0.5),
            "r_rad_s": math.radians(0.5),
        }
    )
    return state
    ####


def _matched_pitch_wrench_offset_report(
    plant: object,
    trim: TrimResult,
    design: PhysicalWrenchLqiDesign,
    plan: B747LocalPhysicalControlScreenPlan,
    *,
    nominal_validation: PhysicalWrenchLqiValidation,
    nominal_assessment: Mapping[str, object],
) -> dict[str, object]:
    """Screen a matched pitch load through B747's bounded physical allocator.

    The external moment enters only the explicit local body-dynamics seam
    after the source-table runtime evaluates its loads. It is not injected
    into the controller request or allocation result, so recovery must use
    the actual bounded elevator coordinate and coupled source-table plant.
    """

    try:
        pitch_index = design.projection.wrench_names.index("moment_y_nm")
    except ValueError as error:
        raise ValueError("B747 source-surface LQI design does not control pitch moment") from error
    pitch_wrench_scale_nm = float(design.wrench_scales[pitch_index])
    cases: list[dict[str, object]] = []
    for identifier, fraction, retained in (
        ("nominal", 0.0, (nominal_validation, nominal_assessment)),
        ("positive-pitch-offset", _MATCHED_PITCH_WRENCH_BIAS_FRACTION, None),
        ("negative-pitch-offset", -_MATCHED_PITCH_WRENCH_BIAS_FRACTION, None),
    ):
        bias_nm = fraction * pitch_wrench_scale_nm
        assessment: Mapping[str, object]
        if retained is None:
            validation = validate_nonlinear_wrench_lqi(
                plant,  # type: ignore[arg-type]
                trim,
                design,
                initial_state=_initial_state(trim),
                duration_s=plan.duration_s,
                dt_s=plan.dt_s,
                integral_lower={name: -0.5 for name in design.result.output_names},
                integral_upper={name: 0.5 for name in design.result.output_names},
                environment={"external_pitch_moment_bias_nm": bias_nm},
            )
            assessment = _assess(validation)
        else:
            validation, assessment = retained
        final_fraction = _number(assessment, "final_feedback_error_fraction")
        saturation_fraction = _number(assessment, "saturation_fraction")
        case_pass = assessment.get("screen_pass") is True
        cases.append(
            {
                "id": identifier,
                "parameters": {"pitch_wrench_bias_fraction": fraction},
                "status": "pass" if case_pass else "fail",
                "metrics": {
                    "final_feedback_error_fraction": final_fraction,
                    "saturation_fraction": saturation_fraction,
                },
                "external_pitch_moment_bias_nm": bias_nm,
                "allocation_statuses": list(validation.allocation_statuses),
                "integrators_exercised": validation.integrators_exercised,
            }
        )
    passed = all(case["status"] == "pass" for case in cases)
    return {
        "schema": "taoryx.endpoint-robustness-screen/v1alpha1",
        "id": "b747-local-lqi-matched-pitch-wrench-offset",
        "kind": "constant_offset",
        "status": "pass" if passed else "fail",
        "pass": passed,
        "controller": {
            "id": design.id,
            "method": "lqi",
            "integral_q_diagonal": list(design.integral_q_diagonal),
            "pitch_wrench_scale_nm": pitch_wrench_scale_nm,
        },
        "cases": cases,
        "claim_boundary": (
            "This screen covers only constant matched external pitch moments of plus or minus five percent of the "
            "declared condition-3 pitch-wrench scale. It does not model wind, mass variation, a gain schedule, a "
            "transport route, engine/servo dynamics, or qualification."
        ),
    }
    ####


def _assess(validation: PhysicalWrenchLqrValidation | PhysicalWrenchLqiValidation) -> dict[str, object]:
    """Apply the accepted condition-3 nonlinear-screen gates."""

    final_error_fraction = validation.final_normalized_feedback_error_norm / max(
        validation.initial_normalized_feedback_error_norm,
        1.0e-12,
    )
    disallowed = {"infeasible", "numerically_singular", "solver_failure"}
    statuses = set(validation.allocation_statuses)
    checks = {
        "derivative_consistent": validation.design.projection.source_linearization.provenance.derivative_consistent,
        "feedback_recovery": final_error_fraction <= 0.10,
        "final_controlled_residual": validation.final_controlled_actual_residual <= 10.0,
        "saturation_fraction": validation.saturation_fraction <= 0.05,
        "continuous_saturation": validation.maximum_continuous_saturation_duration_s <= 0.50,
        "allocation_statuses": not bool(statuses & disallowed),
        "finite_final_state": all(math.isfinite(value) for value in validation.final_state.values()),
    }
    return {
        "screen_pass": all(checks.values()),
        "checks": checks,
        "allocation_statuses": list(validation.allocation_statuses),
        "final_feedback_error_fraction": final_error_fraction,
        "final_controlled_actual_residual_nm": validation.final_controlled_actual_residual,
        "saturation_fraction": validation.saturation_fraction,
        "maximum_continuous_saturation_duration_s": validation.maximum_continuous_saturation_duration_s,
        "initial_normalized_feedback_error": validation.initial_normalized_feedback_error_norm,
        "final_normalized_feedback_error": validation.final_normalized_feedback_error_norm,
    }
    ####


def _evaluation(assessment: Mapping[str, object], plan: B747LocalPhysicalControlScreenPlan) -> dict[str, object]:
    """Publish local evidence gates without recasting them as a route mission."""

    checks = assessment["checks"]
    if not isinstance(checks, Mapping):
        raise ValueError("B747 assessment did not retain screen checks")
    return {
        "schema": "taoryx.b747-condition3-local-physical-surface-control-screen-evaluation/v1alpha1",
        "kind": "local_physical_control_screen",
        "mission_pass": assessment["screen_pass"],
        "results": [
            {"id": identifier, "status": "pass" if value is True else "fail", "required": True}
            for identifier, value in checks.items()
        ],
        "metrics": {key: value for key, value in assessment.items() if key not in {"screen_pass", "checks"}},
        "screen_duration_s": plan.duration_s,
        "controller_method": plan.controller_method,
        "control_realization": _control_realization(plan.controller_method),
        "claim_boundary": (
            "All gates apply only to the pinned B747 NASA CR-2144 condition-3 local screen. They are not a racetrack "
            "truth-objective, gain-schedule, robustness, servo/engine-dynamics, or flight-qualification evaluation."
        ),
    }
    ####


def _source_table_envelope(rows: list[dict[str, float | int | str]]) -> dict[str, object]:
    """Report the exact run's no-extrapolation condition, not an aircraft envelope."""

    finite = bool(rows) and all(
        math.isfinite(float(value))
        for row in rows
        for value in row.values()
        if isinstance(value, int | float) and not isinstance(value, bool)
    )
    return {
        "schema": "taoryx.b747-condition3-source-table-local-envelope/v1alpha1",
        "pass": finite,
        "checks": [
            {
                "id": "source_table_no_extrapolation",
                "status": "pass" if finite else "fail",
                "required": True,
                "claim_boundary": (
                    "The source-table runtime rejects extrapolation before a committed row is emitted. This reports only "
                    "the pinned condition-3 screen's retained table-domain execution, not a broader B747 envelope."
                ),
            }
        ],
    }
    ####


def _rows(validation: PhysicalWrenchLqrValidation | PhysicalWrenchLqiValidation) -> list[dict[str, float | int | str]]:
    """Flatten source truth and allocation evidence into committed telemetry rows."""

    rows: list[dict[str, float | int | str]] = []
    for sample in validation.samples:
        payload = sample.as_dict()
        state = _mapping(payload.get("state"), "physical LQR sample state")
        requested = _mapping(payload.get("requested_wrench"), "physical LQR requested wrench")
        achieved = _mapping(payload.get("achieved_wrench"), "physical LQR achieved wrench")
        residual = _mapping(payload.get("achieved_residual"), "physical LQR achieved residual")
        actual = _mapping(payload.get("actual_effectors"), "physical LQR actual effectors")
        position_saturated = _strings(payload.get("position_saturated"), "physical LQR position saturation")
        rate_limited = _strings(payload.get("rate_limited"), "physical LQR rate limitation")
        rows.append(
            {
                "time_s": _number(payload, "time_s"),
                **{name: _number(state, name) for name in _STATE_NAMES},
                **{f"requested_moment_{axis}_nm": _number(requested, f"moment_{axis}_nm") for axis in ("x", "y", "z")},
                **{f"achieved_moment_{axis}_nm": _number(achieved, f"moment_{axis}_nm") for axis in ("x", "y", "z")},
                **{f"residual_moment_{axis}_nm": _number(residual, f"moment_{axis}_nm") for axis in ("x", "y", "z")},
                "elevator_deg": _number(actual, "elevator-deg"),
                "aileron_deg": _number(actual, "aileron-deg"),
                "rudder_deg": _number(actual, "rudder-deg"),
                "throttle_fraction": _number(actual, "throttle"),
                "allocation_status": _text(payload, "allocation_status"),
                "saturation_count": len(position_saturated) + len(rate_limited),
                "allocation_controlled_residual_norm": _number(payload, "achieved_controlled_residual_norm"),
            }
        )
    return rows
    ####


def _status_samples(
    rows: list[dict[str, float | int | str]],
    mass_kg: float,
    controller_method: LocalControllerMethod,
) -> tuple[BatchTruthSample, ...]:
    """Project exact B747 condition-3 truth into public status channels."""

    samples: list[BatchTruthSample] = []
    for index, row in enumerate(rows):
        raw: dict[str, object] = dict(row)
        raw.update(
            {
                "local_attitude_rad": [_number(row, name) for name in ("roll_error_rad", "pitch_error_rad", "yaw_error_rad")],
                "body_rate_rad_s": [_number(row, name) for name in ("p_rad_s", "q_rad_s", "r_rad_s")],
                "body_velocity_m_s": [_number(row, name) for name in ("u_m_s", "v_m_s", "w_m_s")],
                "requested_moment_body_nm": [_number(row, f"requested_moment_{axis}_nm") for axis in ("x", "y", "z")],
                "achieved_moment_body_nm": [_number(row, f"achieved_moment_{axis}_nm") for axis in ("x", "y", "z")],
                "residual_moment_body_nm": [_number(row, f"residual_moment_{axis}_nm") for axis in ("x", "y", "z")],
                "wrench_status": _text(row, "allocation_status"),
                "wrench_saturated": int(row["saturation_count"]) > 0,
                "physical_effector_allocation": True,
                "control_realization": _control_realization(controller_method),
                "controller_method": controller_method,
                "mass_kg": mass_kg,
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
    """Retain the actual bounded B747 source-table coordinates per interval."""

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
                    "effector.throttle.position": _number(row, "throttle_fraction"),
                    "effector.elevator.position": _number(row, "elevator_deg"),
                    "effector.aileron.position": _number(row, "aileron_deg"),
                    "effector.rudder.position": _number(row, "rudder_deg"),
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
        raise ValueError(f"B747 physical LQR telemetry {name!r} must be finite numeric")
    return float(value)
    ####


def _text(values: Mapping[str, object], name: str) -> str:
    value = values.get(name)
    if not isinstance(value, str) or not value:
        raise ValueError(f"B747 physical LQR telemetry {name!r} must be nonempty text")
    return value
    ####


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    if not rows:
        raise ValueError("B747 physical LQR screen produced no telemetry rows")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    ####


def _screen_definition(mission_id: str) -> tuple[LocalControllerMethod, str, str]:
    """Return the exact controller, segment, and capability adapter for a mission."""

    try:
        return _SCREEN_BY_MISSION[mission_id]
    except KeyError as error:
        raise ValueError(f"unknown B747 local physical-control mission: {mission_id!r}") from error
    ####


def _mission_id_for(controller_method: LocalControllerMethod) -> str:
    """Return the one exact local-screen mission owned by this controller method."""

    for mission_id, (method, _, _) in _SCREEN_BY_MISSION.items():
        if method == controller_method:
            return mission_id
    raise ValueError(f"unknown B747 local controller method: {controller_method!r}")
    ####


def _design_for(controller_method: LocalControllerMethod) -> PhysicalWrenchLqrDesign | PhysicalWrenchLqiDesign:
    """Build only the controller design declared by the exact screen."""

    if controller_method == "lqr":
        return build_b747_condition3_source_surface_physical_lqr_design()
    return build_b747_condition3_source_surface_physical_lqi_design()
    ####


def _control_realization(controller_method: LocalControllerMethod) -> str:
    """Name the allocator-backed source coordinate realization without aliasing methods."""

    return f"source_table_physical_wrench_{controller_method}_allocation"
    ####


__all__ = [
    "B747LocalPhysicalControlScreenCapabilityAdapter",
    "B747LocalPhysicalLqiControlScreenCapabilityAdapter",
    "B747LocalPhysicalControlScreenExecution",
    "B747LocalPhysicalControlScreenPlan",
    "compile_b747_condition3_local_physical_control_screen",
    "execute_b747_condition3_local_physical_control_screen",
    "preflight_b747_condition3_local_physical_control_screen",
]
