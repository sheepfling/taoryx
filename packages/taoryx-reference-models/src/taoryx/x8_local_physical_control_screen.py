"""Composition endpoint for the X8 source-table physical LQR screen.

The screen is deliberately small: it proves one source-trim roll/pitch
recovery through bounded collective/differential elevon coordinates.  It does
not relabel those virtual source coordinates as measured left/right hardware
servo positions, and it does not stand in for the separate racetrack mission.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

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
from .source_table_fixed_wing import (
    assess_x8_source_coupled_lateral_authority,
    build_x8_source_surface_physical_lqi_design,
    build_x8_source_surface_physical_lqr_design,
    build_x8_source_table_plant,
)
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    VehicleExecutionPreflight,
    build_concrete_capability_preflight_evidence,
    preflight_vehicle_composition,
)

_INITIALIZATION_ID = "source_table_trim_local_point"
LocalControllerMethod = Literal["lqr", "lqi"]


@dataclass(frozen=True, slots=True)
class _X8ScreenDefinition:
    """One pinned public source-table recovery scope."""

    controller_method: LocalControllerMethod
    segment_id: str
    capability_adapter_id: str
    duration_s: float
    dt_s: float
    powered_trim_and_extended_recovery: bool = False

    ####


_SCREEN_BY_MISSION: dict[str, _X8ScreenDefinition] = {
    "x8_local_physical_surface_lqr_screen_v1": _X8ScreenDefinition(
        "lqr",
        "source_table_physical_lqr_screen",
        "taoryx.x8_local_physical_surface_lqr_screen.capability.v1",
        8.0,
        0.01,
    ),
    "x8_local_physical_surface_lqi_screen_v1": _X8ScreenDefinition(
        "lqi",
        "source_table_physical_lqi_screen",
        "taoryx.x8_local_physical_surface_lqi_screen.capability.v1",
        8.0,
        0.01,
    ),
    "x8_local_physical_surface_lqi_long_recovery_screen_v1": _X8ScreenDefinition(
        "lqi",
        "source_table_physical_lqi_long_recovery_screen",
        "taoryx.x8_local_physical_surface_lqi_long_recovery_screen.capability.v1",
        20.0,
        0.02,
        powered_trim_and_extended_recovery=True,
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
class X8LocalPhysicalControlScreenPlan:
    """Fixed lowering record for the one source-table recovery screen."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_instance_id: str
    controller_method: LocalControllerMethod
    duration_s: float
    dt_s: float
    powered_trim_and_extended_recovery: bool

    def manifest(self) -> dict[str, object]:
        """Return the public lowering scope for this exact screen."""

        return {
            "schema": "taoryx.x8-local-physical-surface-control-screen-plan/v1alpha1",
            "family_id": self.family_id,
            "mission_id": self.mission_id,
            "fidelity": self.fidelity,
            "initialization_id": self.initialization_id,
            "segments": [
                {
                    "instance_id": self.segment_instance_id,
                    "id": _screen_definition(self.mission_id).segment_id,
                    "transition_semantics": "screen_completion_only",
                }
            ],
            "duration_s": self.duration_s,
            "dt_s": self.dt_s,
            "controller_method": self.controller_method,
            "control_realization": _control_realization(self.controller_method),
            "source_powered_trim": self.powered_trim_and_extended_recovery,
            "extended_recovery": {
                "status": "executed_by_this_screen" if self.powered_trim_and_extended_recovery else "not_selected",
                "duration_s": self.duration_s,
            },
            "claim_boundary": (
                "This selects one Skywalker X8 source-trim local roll/pitch recovery. It does not execute a "
                "racetrack, gain schedule, flight controller, or left/right servo-wiring validation."
            ),
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class X8LocalPhysicalControlScreenExecution:
    """Public result of one X8 nonlinear source-coordinate physical screen."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: X8LocalPhysicalControlScreenPlan
    output_dir: Path
    runtime: dict[str, object]
    evaluation: dict[str, object]
    status_trace: dict[str, object]
    control_trace: dict[str, object]
    claim_boundary: str

    @property
    def screen_pass(self) -> bool:
        """Return the narrow screen disposition only."""

        return bool(self.evaluation["mission_pass"])
        ####

    def as_dict(self) -> dict[str, object]:
        """Serialize the compact public execution result."""

        return {
            "schema": "taoryx.x8-local-physical-surface-control-screen-execution/v1alpha1",
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


def compile_x8_local_physical_control_screen(
    composition: CompiledVehicleComposition,
) -> X8LocalPhysicalControlScreenPlan:
    """Reject every composition that is not the exact pinned X8 screen."""

    if composition.family_id != "skywalker_x8":
        raise ValueError("X8 physical control screen requires the skywalker_x8 family")
    if composition.mission not in _SCREEN_BY_MISSION:
        choices = ", ".join(sorted(_SCREEN_BY_MISSION))
        raise ValueError(f"X8 physical control screen requires one of: {choices}")
    if composition.fidelity != "rigid_body_6dof_surface_allocated":
        raise ValueError("X8 physical control screen requires surface-allocated rigid-body fidelity")
    if composition.initialization.id != _INITIALIZATION_ID:
        raise ValueError(f"X8 physical control screen requires initialization {_INITIALIZATION_ID!r}")
    if composition.initialization.inputs:
        raise ValueError("X8 physical control screen does not accept initialization overrides")
    definition = _screen_definition(composition.mission)
    if len(composition.segments) != 1 or composition.segments[0].id != definition.segment_id:
        raise ValueError(f"X8 physical control screen requires exactly one {definition.segment_id!r} segment")
    segment = composition.segments[0]
    if segment.inputs:
        raise ValueError("X8 physical control screen does not accept segment overrides")
    return X8LocalPhysicalControlScreenPlan(
        family_id=composition.family_id,
        mission_id=composition.mission,
        fidelity=composition.fidelity,
        initialization_id=composition.initialization.id,
        segment_instance_id=segment.instance_id,
        controller_method=definition.controller_method,
        duration_s=definition.duration_s,
        dt_s=definition.dt_s,
        powered_trim_and_extended_recovery=definition.powered_trim_and_extended_recovery,
    )
    ####


class X8LocalPhysicalControlScreenCapabilityAdapter:
    """Advertise one exact source-table controller/allocator path before execution."""

    id = "taoryx.x8_local_physical_surface_lqr_screen.capability.v1"
    controller_method: LocalControllerMethod = "lqr"
    mission_id: str | None = None

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether this endpoint owns the selected composition."""

        return (
            composition.family_id == "skywalker_x8"
            and composition.mission == (self.mission_id or _mission_id_for(self.controller_method))
            and composition.fidelity == "rigid_body_6dof_surface_allocated"
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Expose actual controlled axes, effectors, and nonclaims to callers."""

        plan = compile_x8_local_physical_control_screen(composition)
        plant = build_x8_source_table_plant()
        trim = plant.trim(plant.source_local_state, plant.source_effectors)
        design = _design_for(self.controller_method)
        lqi_design = build_x8_source_surface_physical_lqi_design()
        coupled_lateral_authority = assess_x8_source_coupled_lateral_authority()
        manifest = plan.manifest()
        trim_controls = {
            name: float(trim.controls[name])
            for name in plant.control_names
            if name in trim.controls
        }
        source_powered_trim = bool(
            trim.success
            and set(trim_controls) == set(plant.control_names)
            and 0.0 <= trim_controls.get("throttle", -1.0) <= 1.0
        )
        manifest["capability"] = {
            "control_realization": _control_realization(self.controller_method),
            "participating_nonlinear_plant": True,
            "source_physical_trim": source_powered_trim,
            "source_powered_trim": {
                "status": "freshly_solved_for_this_estimate" if source_powered_trim else "unavailable",
                "success": trim.success,
                "controls": trim_controls,
                "solver_message": trim.message,
            },
            "navigation_guidance": False,
            "screen_duration_s": plan.duration_s,
            "integration_dt_s": plan.dt_s,
            "controller_id": design.id,
            "controller_method": self.controller_method,
            "controller_hurwitz": design.result.hurwitz,
            "controlled_state_names": list(design.projection.state_names),
            "controlled_wrench_axes": list(design.projection.wrench_names),
            "unallocated_wrench_axes": ["moment_z_nm"],
            "effector_names": list(plant.control_names),
            "physical_effector_allocation": True,
            "source_coordinate_mapping": "collective/differential elevon coordinates; not individual servo telemetry",
            "offset_free_tuning_candidate": {
                "campaign_id": "x8-source-surface-local-lqi-v1",
                "method": "lqi",
                "availability": "available_through_model_tune",
                "controller_id": lqi_design.id,
                "controller_hurwitz": lqi_design.result.hurwitz,
                "controlled_state_names": list(lqi_design.projection.state_names),
                "controlled_wrench_axes": list(lqi_design.projection.wrench_names),
                "integral_output_names": list(lqi_design.result.output_names),
                "integral_q_diagonal": list(lqi_design.integral_q_diagonal),
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
                    "mission_id": plan.mission_id if self.controller_method == "lqi" else _mission_id_for("lqi"),
                    "capability_adapter_id": self.id if self.controller_method == "lqi" else "taoryx.x8_local_physical_surface_lqi_screen.capability.v1",
                    "operations": ["validate", "batch"],
                    "control_realization": "source_table_coordinate_physical_wrench_lqi_allocation",
                },
                "persistent_disturbance_status": "not_executable_without_a_declared_source_derivative_environment",
                "claim_boundary": (
                    "The public batch screen executes its declared controller through bounded "
                    "collective/differential-elevon allocation. The named roll/pitch LQI design has a focused allocation "
                    "baseline, including the declared source rate and lag model. The source adapter exposes "
                    "no wind, bias, or mass-variation derivative input with which to establish persistent-"
                    "disturbance rejection. It neither adds independent yaw authority nor qualifies a gain "
                    "schedule, racetrack, or individual-servo controller."
                ),
            },
            "coupled_lateral_authority": {
                "status": coupled_lateral_authority.status,
                "control_coordinate": "differential-elevon-deg",
                "allocator_request_axis": "moment_x_nm",
                "feedback_state_names": [
                    "roll_error_rad",
                    "yaw_error_rad",
                    "v_m_s",
                    "p_rad_s",
                    "r_rad_s",
                ],
                "independent_yaw_wrench_axis": False,
                "nonlinear_recovery_status": "not_qualified",
                "linear_preflight": coupled_lateral_authority.as_dict(),
            },
            "extended_recovery": {
                "status": "executed_by_this_screen" if plan.powered_trim_and_extended_recovery else "not_selected",
                "duration_s": plan.duration_s,
                "fixed_cadence_s": plan.dt_s,
                "source_powered_trim": source_powered_trim,
                "claim_boundary": (
                    "This is a pinned 20-second local roll/pitch recovery after a fresh source-table powered trim. "
                    "It is not a lateral/yaw recovery, racetrack, disturbance-rejection, gain-schedule, or qualification claim."
                ) if plan.powered_trim_and_extended_recovery else None,
            },
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                "pinned X8 source trim, plant-derived roll/pitch controller, bounded source-coordinate allocation, and nonlinear "
                f"{plan.duration_s:g}-second recovery is available; physical racetrack, gain scheduling, and servo-wiring evidence remain separate gates",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####


class X8LocalPhysicalLqiControlScreenCapabilityAdapter(X8LocalPhysicalControlScreenCapabilityAdapter):
    """Advertise the explicit offset-free X8 source-table physical screen."""

    id = "taoryx.x8_local_physical_surface_lqi_screen.capability.v1"
    controller_method: LocalControllerMethod = "lqi"

    ####


class X8LocalPhysicalLongRecoveryLqiScreenCapabilityAdapter(X8LocalPhysicalControlScreenCapabilityAdapter):
    """Advertise the extended source-powered LQI recovery without promoting it to a route."""

    id = "taoryx.x8_local_physical_surface_lqi_long_recovery_screen.capability.v1"
    controller_method: LocalControllerMethod = "lqi"
    mission_id = "x8_local_physical_surface_lqi_long_recovery_screen_v1"

    ####


def preflight_x8_local_physical_control_screen(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Prove that Composition selected the concrete source-table path."""

    plan = compile_x8_local_physical_control_screen(composition)
    estimate = estimate_mission_capability(composition)
    definition = _screen_definition(composition.mission)
    segment_id = definition.segment_id
    capability_adapter_id = definition.capability_adapter_id
    if estimate is None or estimate.adapter_id != capability_adapter_id:
        raise ValueError("X8 physical control screen has no matching installed capability adapter")
    capability = estimate.manifest.get("capability")
    if not isinstance(capability, Mapping):
        raise ValueError("X8 physical control capability record is missing")
    controller_hurwitz = capability.get("controller_hurwitz") is True
    source_powered_trim = capability.get("source_powered_trim")
    source_powered_trim_ready = (
        isinstance(source_powered_trim, Mapping)
        and source_powered_trim.get("status") == "freshly_solved_for_this_estimate"
        and source_powered_trim.get("success") is True
    )
    coupled_lateral = capability.get("coupled_lateral_authority")
    if not isinstance(coupled_lateral, Mapping):
        raise ValueError("X8 physical control capability has no coupled-lateral authority diagnostic")
    coupled_lateral_reported = (
        coupled_lateral.get("status") == "passed"
        and coupled_lateral.get("control_coordinate") == "differential-elevon-deg"
        and coupled_lateral.get("independent_yaw_wrench_axis") is False
        and coupled_lateral.get("nonlinear_recovery_status") == "not_qualified"
    )
    design = _design_for(plan.controller_method)
    derivative_consistent = design.projection.source_linearization.provenance.derivative_consistent
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status=(
            "translation_ready"
            if controller_hurwitz
            and derivative_consistent
            and (not plan.powered_trim_and_extended_recovery or source_powered_trim_ready)
            else "blocked"
        ),
        translator_id=capability_adapter_id,
        checks=(
            ExecutionPreflightCheck(
                f"x8.semantic_local_physical_surface_{plan.controller_method}_screen",
                [_INITIALIZATION_ID, segment_id],
                [plan.initialization_id, plan.segment_instance_id],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                "x8.local_physical_wrench_controller_hurwitz",
                True,
                controller_hurwitz,
                None,
                controller_hurwitz,
            ),
            ExecutionPreflightCheck(
                "x8.source_table_derivative_consistency",
                True,
                derivative_consistent,
                None,
                derivative_consistent,
            ),
            ExecutionPreflightCheck(
                "x8.coupled_lateral_authority_diagnosed",
                True,
                coupled_lateral_reported,
                None,
                coupled_lateral_reported,
            ),
            *(
                (
                    ExecutionPreflightCheck(
                        "x8.source_powered_trim",
                        True,
                        source_powered_trim_ready,
                        None,
                        source_powered_trim_ready,
                    ),
                    ExecutionPreflightCheck(
                        "x8.extended_physical_lqi_recovery_duration_s",
                        20.0,
                        plan.duration_s,
                        20.0,
                        plan.duration_s >= 20.0,
                    ),
                )
                if plan.powered_trim_and_extended_recovery
                else ()
            ),
        ),
        diagnostics=(
            "composition lowers exactly to the X8 source-trim local physical surface-allocation screen; it is not a full route or scheduled-envelope translator",
            "differential-elevon has a source-linearized coupled lateral authority path, but nonlinear yaw/sideslip recovery and route use remain unqualified",
            *estimate.diagnostics,
        ),
        derived_mission=estimate.manifest,
        capability_estimate=build_concrete_capability_preflight_evidence(composition, estimate),
    )
    ####


def execute_x8_local_physical_control_screen(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None = None,
) -> X8LocalPhysicalControlScreenExecution:
    """Run source nonlinear dynamics through the bounded physical allocator."""

    if max_steps is not None:
        raise ValueError("--max-steps is unavailable for the fixed-cadence X8 physical-control screen")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        details = "; ".join(preflight.diagnostics) or "no translation-ready X8 local physical screen"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {details}")
    plan = compile_x8_local_physical_control_screen(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    plant = build_x8_source_table_plant()
    trim = plant.trim(plant.source_local_state, plant.source_effectors)
    if not trim.success:
        raise RuntimeError(f"X8 physical-control screen trim failed: {trim.as_dict()}")
    design = _design_for(plan.controller_method)
    initial_state = dict(trim.state)
    initial_state.update(
        {
            "roll_error_rad": math.radians(5.0),
            "pitch_error_rad": math.radians(-3.0),
            "p_rad_s": math.radians(4.0),
            "q_rad_s": math.radians(-3.0),
        }
    )
    if plan.controller_method == "lqr":
        validation = validate_nonlinear_wrench_lqr(
            plant,
            trim,
            design,
            initial_state=initial_state,
            duration_s=plan.duration_s,
            dt_s=plan.dt_s,
        )
    else:
        lqi_design = build_x8_source_surface_physical_lqi_design()
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
    if plan.powered_trim_and_extended_recovery:
        checks = assessment["checks"]
        if not isinstance(checks, dict):
            raise ValueError("X8 assessment did not retain mutable screen checks")
        checks["source_powered_trim"] = trim.success
        checks["extended_recovery_duration"] = plan.duration_s >= 20.0
        assessment["screen_pass"] = all(checks.values())
    rows = _rows(validation)
    envelope = _source_table_envelope(rows)
    mass_kg = float(plant.source_state.mass)
    rows = [{**row, "mass_kg": mass_kg} for row in rows]
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.fixed_wing.source_table.v1",
        "controller_id": design.id,
        "controller_method": plan.controller_method,
        "integral_output_names": list(design.result.output_names) if plan.controller_method == "lqi" else [],
        "control_realization": _control_realization(plan.controller_method),
        "physical_effector_allocation": True,
        "controlled_state_names": list(design.projection.state_names),
        "controlled_wrench_axes": list(design.projection.wrench_names),
        "unallocated_wrench_axes": ["moment_z_nm"],
        "effector_names": list(plant.control_names),
        "dt_s": plan.dt_s,
        "duration_s": plan.duration_s,
        "source_powered_trim": {
            "status": "freshly_solved_for_this_execution",
            "success": trim.success,
            "controls": {name: float(trim.controls[name]) for name in plant.control_names},
            "solver_message": trim.message,
        },
        "extended_recovery": {
            "status": "executed_by_this_screen" if plan.powered_trim_and_extended_recovery else "not_selected",
            "duration_s": plan.duration_s,
            "fixed_cadence_s": plan.dt_s,
        },
        "mass_kg": mass_kg,
        "source_table_envelope": envelope,
        "hard_gates_passed": assessment["screen_pass"],
        "mission_graph_execution": unobserved_mission_graph_execution(
            composition,
            "The local X8 physical control screen has no route graph dispatcher and makes no mission-transition claim.",
        ).as_dict(),
    }
    evaluation = _evaluation(assessment, plan)
    status_trace = build_committed_status_trace(composition, _status_samples(rows, mass_kg, plan.controller_method))
    control_trace = build_committed_control_trace(composition, _control_samples(rows))
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    runtime["resource_ledger"] = resource_ledger_summary(resource_ledger)
    result = X8LocalPhysicalControlScreenExecution(
        composition=composition,
        preflight=preflight,
        plan=plan,
        output_dir=destination,
        runtime=runtime,
        evaluation=evaluation,
        status_trace=status_trace,
        control_trace=control_trace,
        claim_boundary=(
            f"This is a {plan.duration_s:g}-second X8 source-trim roll/pitch recovery. Requested moments are allocated through bounded "
            "collective/differential source-table coordinates with declared lag and rate limits. It does not prove a racetrack, "
            "gain schedule, mass robustness, wind rejection, individual servo wiring, or flight qualification."
        ),
    )
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "mission_graph_execution.json", runtime["mission_graph_execution"])
    _write_json(destination / "nonlinear_validation.json", validation.as_dict())
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


def _assess(validation: PhysicalWrenchLqrValidation | PhysicalWrenchLqiValidation) -> dict[str, object]:
    """Apply the acceptance thresholds from the existing X8 evidence packet."""

    final_error_fraction = validation.final_feedback_error_norm / max(validation.initial_feedback_error_norm, 1.0e-12)
    disallowed = {"infeasible", "numerically_singular", "solver_failure"}
    statuses = set(validation.allocation_statuses)
    finite = all(math.isfinite(value) for value in validation.final_state.values())
    derivative_consistent = validation.design.projection.source_linearization.provenance.derivative_consistent
    checks = {
        "derivative_consistent": derivative_consistent,
        "feedback_recovery": final_error_fraction <= 0.05,
        "final_controlled_residual": validation.final_controlled_actual_residual <= 0.005,
        "saturation_fraction": validation.saturation_fraction <= 0.05,
        "continuous_saturation": validation.maximum_continuous_saturation_duration_s <= 0.25,
        "allocation_statuses": not bool(statuses & disallowed),
        "finite_final_state": finite,
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


def _evaluation(assessment: Mapping[str, object], plan: X8LocalPhysicalControlScreenPlan) -> dict[str, object]:
    """Publish each local gate without translating it into a route objective."""

    checks = assessment["checks"]
    if not isinstance(checks, Mapping):
        raise ValueError("X8 assessment did not retain screen checks")
    return {
        "schema": "taoryx.x8-local-physical-surface-control-screen-evaluation/v1alpha1",
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
            "All gates apply only to the pinned X8 source-trim local screen. They are not a racetrack truth-objective, "
            "gain-schedule, robustness, servo-wiring, or flight-qualification evaluation."
        ),
    }
    ####


def _source_table_envelope(rows: list[dict[str, float | int | str]]) -> dict[str, object]:
    """Record the no-extrapolation domain condition enforced by the source plant.

    ``RuntimeRigidBodyLocalPlant`` raises before emitting a sample when one of
    the selected source tables is evaluated outside its declared domain.  A
    completed finite row sequence is therefore evidence that this exact local
    run stayed in the retained table domain, not an inferred flight envelope.
    """

    finite = bool(rows) and all(
        math.isfinite(float(value))
        for row in rows
        for value in row.values()
        if isinstance(value, int | float) and not isinstance(value, bool)
    )
    return {
        "schema": "taoryx.x8-source-table-local-envelope/v1alpha1",
        "pass": finite,
        "checks": [
            {
                "id": "source_table_no_extrapolation",
                "status": "pass" if finite else "fail",
                "required": True,
                "claim_boundary": (
                    "The source-table runtime rejects extrapolation before a committed row is emitted. This reports only "
                    "the pinned screen's retained table-domain execution, not a broader aircraft envelope."
                ),
            }
        ],
    }
    ####


def _rows(validation: PhysicalWrenchLqrValidation | PhysicalWrenchLqiValidation) -> list[dict[str, float | int | str]]:
    """Flatten committed source truth while retaining requested/achieved allocation evidence."""

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
        row: dict[str, float | int | str] = {
            "time_s": _number(payload, "time_s"),
            **{name: _number(state, name) for name in _STATE_NAMES},
            "requested_moment_x_nm": _number(requested, "moment_x_nm"),
            "requested_moment_y_nm": _number(requested, "moment_y_nm"),
            "requested_moment_z_nm": _number(requested, "moment_z_nm"),
            "achieved_moment_x_nm": _number(achieved, "moment_x_nm"),
            "achieved_moment_y_nm": _number(achieved, "moment_y_nm"),
            "achieved_moment_z_nm": _number(achieved, "moment_z_nm"),
            "residual_moment_x_nm": _number(residual, "moment_x_nm"),
            "residual_moment_y_nm": _number(residual, "moment_y_nm"),
            "residual_moment_z_nm": _number(residual, "moment_z_nm"),
            "collective_elevon_deg": _number(actual, "collective-elevon-deg"),
            "differential_elevon_deg": _number(actual, "differential-elevon-deg"),
            "throttle_fraction": _number(actual, "throttle"),
            "allocation_status": _text(payload, "allocation_status"),
            "saturation_count": len(position_saturated) + len(rate_limited),
            "allocation_controlled_residual_norm": _number(payload, "achieved_controlled_residual_norm"),
        }
        rows.append(row)
    return rows
    ####


def _status_samples(
    rows: list[dict[str, float | int | str]],
    mass_kg: float,
    controller_method: LocalControllerMethod,
) -> tuple[BatchTruthSample, ...]:
    """Project exact source-coordinate plant and allocation truth into the interface."""

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
    """Retain the actual bounded source-table coordinates for every interval."""

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
                    "effector.elevon.collective.position": _number(row, "collective_elevon_deg"),
                    "effector.elevon.differential.position": _number(row, "differential_elevon_deg"),
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
        raise ValueError(f"X8 physical LQR telemetry {name!r} must be finite numeric")
    return float(value)
    ####


def _text(values: Mapping[str, object], name: str) -> str:
    value = values.get(name)
    if not isinstance(value, str) or not value:
        raise ValueError(f"X8 physical LQR telemetry {name!r} must be nonempty text")
    return value
    ####


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    if not rows:
        raise ValueError("X8 physical LQR screen produced no telemetry rows")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    ####


def _screen_definition(mission_id: str) -> _X8ScreenDefinition:
    """Return the exact controller, segment, and capability adapter for a mission."""

    try:
        return _SCREEN_BY_MISSION[mission_id]
    except KeyError as error:
        raise ValueError(f"unknown X8 local physical-control mission: {mission_id!r}") from error
    ####


def _mission_id_for(controller_method: LocalControllerMethod) -> str:
    """Return the one exact local-screen mission owned by this controller method."""

    for mission_id, definition in _SCREEN_BY_MISSION.items():
        if definition.controller_method == controller_method:
            return mission_id
    raise ValueError(f"unknown X8 local controller method: {controller_method!r}")
    ####


def _design_for(controller_method: LocalControllerMethod):
    """Build only the controller design declared by the exact screen."""

    if controller_method == "lqr":
        return build_x8_source_surface_physical_lqr_design()
    return build_x8_source_surface_physical_lqi_design()
    ####


def _control_realization(controller_method: LocalControllerMethod) -> str:
    """Name the allocator-backed source coordinate realization without aliasing methods."""

    return f"source_table_coordinate_physical_wrench_{controller_method}_allocation"
    ####


__all__ = [
    "X8LocalPhysicalControlScreenCapabilityAdapter",
    "X8LocalPhysicalLqiControlScreenCapabilityAdapter",
    "X8LocalPhysicalLongRecoveryLqiScreenCapabilityAdapter",
    "X8LocalPhysicalControlScreenExecution",
    "X8LocalPhysicalControlScreenPlan",
    "compile_x8_local_physical_control_screen",
    "execute_x8_local_physical_control_screen",
    "preflight_x8_local_physical_control_screen",
]
