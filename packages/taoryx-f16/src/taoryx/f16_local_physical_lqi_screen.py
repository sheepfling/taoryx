"""Exact Composition endpoint for the F-16 source-trim physical LQI recovery.

This is intentionally distinct from the existing one-second F-16 route-entry
LQR screen.  It proves a fixed-altitude, five-second body-velocity recovery
through the declared bounded surface/throttle overlay and physical allocator.
It does not synthesize navigation motion, a gain schedule, wind rejection, or
flight qualification.
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
    apply_tuning_context_to_physical_wrench_lqi_design,
    validate_nonlinear_wrench_lqi,
)
from .source_f16 import build_f16_local_physical_wrench_lqi_design, build_f16_source_physical_plant
from .tuning_application import TuningApplicationContext
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    VehicleExecutionPreflight,
    build_concrete_capability_preflight_evidence,
    preflight_vehicle_composition,
)

_MISSION_ID = "f16_local_physical_surface_lqi_screen_v1"
_INITIALIZATION_ID = "source_physical_control_local_point"
_SEGMENT_ID = "local_physical_surface_lqi_screen"
_CAPABILITY_ADAPTER_ID = "taoryx.f16_local_physical_surface_lqi_screen.capability.v1"
_DURATION_S = 5.0
_DT_S = 0.02
_STATE_NAMES = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")


@dataclass(frozen=True, slots=True)
class F16LocalPhysicalLqiScreenPlan:
    """Pinned lowering record for one F-16 fixed-altitude velocity recovery."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_instance_id: str
    duration_s: float
    dt_s: float

    def manifest(self) -> dict[str, object]:
        """Return the exact public scope without translating it into a route."""

        return {
            "schema": "taoryx.f16-local-physical-surface-lqi-screen-plan/v1alpha1",
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
            "duration_s": self.duration_s,
            "dt_s": self.dt_s,
            "controller_method": "lqi",
            "control_realization": "surface_allocated",
            "environment": "fixed_source_trim_altitude_and_pitch",
            "claim_boundary": (
                "This selects one F-16 source-trim fixed-altitude body-velocity recovery. It does not execute a "
                "racetrack, integrate navigation, establish a gain schedule, or validate wind, mass, or flight robustness."
            ),
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class F16LocalPhysicalLqiScreenExecution:
    """Public result of one allocator-backed F-16 physical LQI screen."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: F16LocalPhysicalLqiScreenPlan
    output_dir: Path
    runtime: dict[str, object]
    environment: dict[str, object]
    evaluation: dict[str, object]
    status_trace: dict[str, object]
    control_trace: dict[str, object]
    claim_boundary: str

    @property
    def screen_pass(self) -> bool:
        """Return only the fixed local recovery disposition."""

        return bool(self.evaluation["mission_pass"])
        ####

    def as_dict(self) -> dict[str, object]:
        """Serialize the compact public execution result."""

        return {
            "schema": "taoryx.f16-local-physical-surface-lqi-screen-execution/v1alpha1",
            "status": "development_local_screen_pass" if self.screen_pass else "development_local_screen_failed",
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "plan": self.plan.manifest(),
            "output_dir": str(self.output_dir),
            "runtime": self.runtime,
            "environment": self.environment,
            "control_screen": self.evaluation,
            "screen_pass": self.screen_pass,
            "status_trace": status_trace_summary(self.status_trace),
            "control_trace": control_trace_summary(self.control_trace),
            "claim_boundary": self.claim_boundary,
        }
        ####

    ####


def compile_f16_local_physical_lqi_screen(
    composition: CompiledVehicleComposition,
) -> F16LocalPhysicalLqiScreenPlan:
    """Reject every composition that is not the exact F-16 physical LQI screen."""

    if composition.family_id != "f16_s119":
        raise ValueError("F-16 physical LQI screen requires the f16_s119 family")
    if composition.mission != _MISSION_ID:
        raise ValueError(f"F-16 physical LQI screen requires mission {_MISSION_ID!r}")
    if composition.fidelity != "rigid_body_6dof_surface_allocated":
        raise ValueError("F-16 physical LQI screen requires surface-allocated rigid-body fidelity")
    if composition.initialization.id != _INITIALIZATION_ID:
        raise ValueError(f"F-16 physical LQI screen requires initialization {_INITIALIZATION_ID!r}")
    if composition.initialization.inputs:
        raise ValueError("F-16 physical LQI screen does not accept initialization overrides")
    if len(composition.segments) != 1 or composition.segments[0].id != _SEGMENT_ID:
        raise ValueError(f"F-16 physical LQI screen requires exactly one {_SEGMENT_ID!r} segment")
    segment = composition.segments[0]
    if segment.inputs:
        raise ValueError("F-16 physical LQI screen does not accept segment overrides")
    return F16LocalPhysicalLqiScreenPlan(
        family_id=composition.family_id,
        mission_id=composition.mission,
        fidelity=composition.fidelity,
        initialization_id=composition.initialization.id,
        segment_instance_id=segment.instance_id,
        duration_s=_DURATION_S,
        dt_s=_DT_S,
    )
    ####


class F16LocalPhysicalLqiScreenCapabilityAdapter:
    """Advertise the exact F-16 physical LQI/allocator path before execution."""

    id = _CAPABILITY_ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether this adapter owns the selected LQI composition."""

        return (
            composition.family_id == "f16_s119"
            and composition.mission == _MISSION_ID
            and composition.fidelity == "rigid_body_6dof_surface_allocated"
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Expose actual source coordinates, integrators, and explicit nonclaims."""

        plan = compile_f16_local_physical_lqi_screen(composition)
        plant = build_f16_source_physical_plant()
        design = build_f16_local_physical_wrench_lqi_design()
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": "surface_allocated",
            "participating_nonlinear_plant": True,
            "source_physical_trim": True,
            "navigation_guidance": False,
            "screen_duration_s": plan.duration_s,
            "integration_dt_s": plan.dt_s,
            "controller_id": design.id,
            "controller_method": "lqi",
            "controller_hurwitz": design.result.hurwitz,
            "controlled_state_names": list(design.projection.state_names),
            "controlled_wrench_axes": list(design.projection.wrench_names),
            "integral_output_names": list(design.result.output_names),
            "integral_q_diagonal": list(design.integral_q_diagonal),
            "effector_names": list(plant.control_names),
            "physical_effector_allocation": True,
            "effector_dynamics": "declared engineering surface/throttle overlay with bounded position, rate, and lag",
            "physical_screen_status": "executed_by_this_lqi_screen",
            "physical_screen_execution": {
                "status": "executed_by_this_lqi_screen",
                "mission_id": _MISSION_ID,
                "capability_adapter_id": _CAPABILITY_ADAPTER_ID,
                "operations": ["validate", "batch"],
                "control_realization": "surface_allocated",
            },
            "persistent_disturbance_status": "not_executable_without_a_declared_source_wind_or_mass_derivative_environment",
            "claim_boundary": (
                "This public batch screen executes the source-local velocity LQI controller through the declared "
                "surface/throttle overlay and allocator. The source derivative seam exposes no wind, bias, or mass "
                "variation input, so no persistent-disturbance result is claimed. It is not a scheduled controller, "
                "F-16 racetrack, or flight qualification."
            ),
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                "pinned F-16 source trim, velocity LQI, bounded surface/throttle allocation, and five-second nonlinear "
                "recovery are available; navigation, scheduling, wind, and mass-robustness evidence remain separate gates",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####


def preflight_f16_local_physical_lqi_screen(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Prove that Composition selected the concrete F-16 physical LQI path."""

    plan = compile_f16_local_physical_lqi_screen(composition)
    estimate = estimate_mission_capability(composition)
    if estimate is None or estimate.adapter_id != _CAPABILITY_ADAPTER_ID:
        raise ValueError("F-16 physical LQI screen has no matching installed capability adapter")
    capability = estimate.manifest.get("capability")
    if not isinstance(capability, Mapping):
        raise ValueError("F-16 physical LQI capability record is missing")
    design = build_f16_local_physical_wrench_lqi_design()
    controller_hurwitz = capability.get("controller_hurwitz") is True
    derivative_consistent = design.projection.source_linearization.provenance.derivative_consistent
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status="translation_ready" if controller_hurwitz and derivative_consistent else "blocked",
        translator_id=_CAPABILITY_ADAPTER_ID,
        checks=(
            ExecutionPreflightCheck(
                "f16.semantic_local_physical_surface_lqi_screen",
                [_INITIALIZATION_ID, _SEGMENT_ID],
                [plan.initialization_id, plan.segment_instance_id],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                "f16.local_physical_lqi_controller_hurwitz",
                True,
                controller_hurwitz,
                None,
                controller_hurwitz,
            ),
            ExecutionPreflightCheck(
                "f16.source_derivative_consistency",
                True,
                derivative_consistent,
                None,
                derivative_consistent,
            ),
        ),
        diagnostics=(
            "composition lowers exactly to the F-16 fixed-altitude local physical LQI screen; it is not a route or scheduled-envelope translator",
            *estimate.diagnostics,
        ),
        derived_mission=estimate.manifest,
        capability_estimate=build_concrete_capability_preflight_evidence(composition, estimate),
    )
    ####


def execute_f16_local_physical_lqi_screen(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None = None,
    *,
    tuning_context: TuningApplicationContext | None = None,
) -> F16LocalPhysicalLqiScreenExecution:
    """Run the exact source-trim velocity LQI through actual F-16 effectors."""

    if max_steps is not None:
        raise ValueError("--max-steps is unavailable for the fixed-cadence F-16 physical LQI screen")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        details = "; ".join(preflight.diagnostics) or "no translation-ready F-16 physical LQI screen"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {details}")
    plan = compile_f16_local_physical_lqi_screen(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    plant = build_f16_source_physical_plant()
    trim = plant.trim_result
    design = build_f16_local_physical_wrench_lqi_design()
    tuning_binding = None
    if tuning_context is not None:
        design, tuning_binding = apply_tuning_context_to_physical_wrench_lqi_design(design, tuning_context)
    initial_state = dict(trim.state)
    initial_state.update(
        {
            "u_m_s": initial_state["u_m_s"] + 0.5,
            "v_m_s": 0.05,
            "w_m_s": initial_state["w_m_s"] - 0.05,
            "p_rad_s": 0.001,
            "q_rad_s": -0.001,
            "r_rad_s": 0.001,
        }
    )
    validation = validate_nonlinear_wrench_lqi(
        plant,
        trim,
        design,
        initial_state=initial_state,
        duration_s=plan.duration_s,
        dt_s=plan.dt_s,
        integral_lower={name: -1.0 for name in design.result.output_names},
        integral_upper={name: 1.0 for name in design.result.output_names},
    )
    assessment = _assess(validation)
    rows = _rows(validation, altitude_m=plant.altitude_m, trim_pitch_rad=plant.trim_pitch_rad)
    for row in rows:
        row["mass_kg"] = float(plant.source.mass_kg)
        row["schedule_node_id"] = "f16-sea-level-152mps"
        row["controller_selection"] = "fixed_source_trim_node"
    environment = _fixed_environment(rows, plant.altitude_m, plant.trim_pitch_rad)
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.fixed_wing.daveml.v1",
        "controller_id": design.id,
        "controller_method": "lqi",
        "integral_output_names": list(design.result.output_names),
        "control_realization": "surface_allocated",
        "physical_effector_allocation": True,
        "controlled_state_names": list(design.projection.state_names),
        "controlled_wrench_axes": list(design.projection.wrench_names),
        "effector_names": list(plant.control_names),
        "dt_s": plan.dt_s,
        "duration_s": plan.duration_s,
        "mass_kg": plant.source.mass_kg,
        "controller_selection": "fixed_source_trim_node",
        "fixed_altitude_m": plant.altitude_m,
        "fixed_trim_pitch_rad": plant.trim_pitch_rad,
        "navigation_state": "fixed_local_origin_and_trim_attitude",
        "hard_gates_passed": assessment["screen_pass"],
        **({"tuning_binding": tuning_binding.as_dict()} if tuning_binding is not None else {}),
        "mission_graph_execution": unobserved_mission_graph_execution(
            composition,
            "The fixed-altitude F-16 physical LQI screen has no route graph dispatcher and makes no mission-transition claim.",
        ).as_dict(),
    }
    evaluation = _evaluation(assessment, plan)
    status_trace = build_committed_status_trace(composition, _status_samples(rows, plant.source.mass_kg))
    control_trace = build_committed_control_trace(composition, _control_samples(rows))
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    runtime["resource_ledger"] = resource_ledger_summary(resource_ledger)
    result = F16LocalPhysicalLqiScreenExecution(
        composition=composition,
        preflight=preflight,
        plan=plan,
        output_dir=destination,
        runtime=runtime,
        environment=environment,
        evaluation=evaluation,
        status_trace=status_trace,
        control_trace=control_trace,
        claim_boundary=(
            "This is a five-second F-16 fixed-altitude source-trim body-velocity recovery. Requested wrench commands "
            "are allocated through bounded engineering-overlay elevator, aileron, rudder, and throttle coordinates. "
            "It does not prove integrated navigation, a racetrack, gain schedule, wind or mass rejection, or flight qualification."
        ),
    )
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "mission_graph_execution.json", runtime["mission_graph_execution"])
    _write_json(destination / "nonlinear_validation.json", validation.as_dict())
    _write_json(destination / "fixed_environment.json", environment)
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
            envelope=environment,
            claim_boundary=result.claim_boundary,
            status_trace=status_trace,
            control_trace=control_trace,
        ).as_dict(),
    )
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _assess(validation: PhysicalWrenchLqiValidation) -> dict[str, object]:
    """Apply the existing bounded F-16 physical-LQI acceptance gates."""

    final_error_fraction = validation.final_normalized_feedback_error_norm / max(
        validation.initial_normalized_feedback_error_norm,
        1.0e-12,
    )
    disallowed = {"infeasible", "numerically_singular", "solver_failure"}
    statuses = set(validation.allocation_statuses)
    checks = {
        "derivative_consistent": validation.design.projection.source_linearization.provenance.derivative_consistent,
        "integrators_exercised": validation.integrators_exercised,
        "feedback_recovery": final_error_fraction <= 0.05,
        "final_controlled_residual": validation.final_controlled_actual_residual <= 20.0,
        "saturation_fraction": validation.saturation_fraction <= 0.05,
        "continuous_saturation": validation.maximum_continuous_saturation_duration_s <= 0.25,
        "allocation_statuses": not bool(statuses & disallowed),
        "finite_final_state": all(math.isfinite(value) for value in validation.final_state.values()),
    }
    return {
        "screen_pass": all(checks.values()),
        "checks": checks,
        "allocation_statuses": list(validation.allocation_statuses),
        "final_normalized_feedback_error_fraction": final_error_fraction,
        "final_controlled_actual_residual_n_nm": validation.final_controlled_actual_residual,
        "saturation_fraction": validation.saturation_fraction,
        "maximum_continuous_saturation_duration_s": validation.maximum_continuous_saturation_duration_s,
        "initial_normalized_feedback_error": validation.initial_normalized_feedback_error_norm,
        "final_normalized_feedback_error": validation.final_normalized_feedback_error_norm,
    }
    ####


def _evaluation(assessment: Mapping[str, object], plan: F16LocalPhysicalLqiScreenPlan) -> dict[str, object]:
    """Publish the local LQI gates without recasting them as route objectives."""

    checks = assessment["checks"]
    if not isinstance(checks, Mapping):
        raise ValueError("F-16 LQI assessment did not retain screen checks")
    return {
        "schema": "taoryx.f16-local-physical-surface-lqi-screen-evaluation/v1alpha1",
        "kind": "local_physical_control_screen",
        "mission_pass": assessment["screen_pass"],
        "results": [
            {"id": identifier, "status": "pass" if value is True else "fail", "required": True}
            for identifier, value in checks.items()
        ],
        "metrics": {key: value for key, value in assessment.items() if key not in {"screen_pass", "checks"}},
        "screen_duration_s": plan.duration_s,
        "controller_method": "lqi",
        "control_realization": "surface_allocated",
        "claim_boundary": (
            "All gates apply only to the fixed F-16 source-trim velocity-recovery screen. They are not navigation, "
            "racetrack, robustness, gain-scheduling, or flight-qualification objectives."
        ),
    }
    ####


def _rows(
    validation: PhysicalWrenchLqiValidation,
    *,
    altitude_m: float,
    trim_pitch_rad: float,
) -> list[dict[str, float | int | str]]:
    """Flatten actual allocator telemetry while preserving fixed local context."""

    rows: list[dict[str, float | int | str]] = []
    for sample in validation.samples:
        payload = sample.as_dict()
        state = _mapping(payload.get("state"), "physical LQI sample state")
        requested = _mapping(payload.get("requested_wrench"), "physical LQI requested wrench")
        achieved = _mapping(payload.get("achieved_wrench"), "physical LQI achieved wrench")
        actual = _mapping(payload.get("actual_effectors"), "physical LQI actual effectors")
        position_saturated = _strings(payload.get("position_saturated"), "physical LQI position saturation")
        rate_limited = _strings(payload.get("rate_limited"), "physical LQI rate limitation")
        u, v, w = (_number(state, name) for name in ("u_m_s", "v_m_s", "w_m_s"))
        rows.append(
            {
                "time_s": _number(payload, "time_s"),
                **{name: _number(state, name) for name in _STATE_NAMES},
                "north_m": 0.0,
                "east_m": 0.0,
                "altitude_m": altitude_m,
                "speed_m_s": math.sqrt(u * u + v * v + w * w),
                "local_roll_deg": 0.0,
                "local_pitch_deg": math.degrees(trim_pitch_rad),
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
                "elevator_deg": _number(actual, "elevator_deg"),
                "aileron_deg": _number(actual, "aileron_deg"),
                "rudder_deg": _number(actual, "rudder_deg"),
                "throttle_fraction": _number(actual, "throttle_fraction"),
                "allocation_status": _text(payload, "allocation_status"),
                "saturation_count": len(position_saturated) + len(rate_limited),
                "allocation_controlled_residual_norm": _number(payload, "achieved_controlled_residual_norm"),
            }
        )
    return rows
    ####


def _fixed_environment(rows: list[dict[str, float | int | str]], altitude_m: float, trim_pitch_rad: float) -> dict[str, object]:
    """Report the fixed source derivative context without calling it an envelope."""

    finite = bool(rows) and all(
        math.isfinite(float(value))
        for row in rows
        for value in row.values()
        if isinstance(value, int | float) and not isinstance(value, bool)
    )
    return {
        "schema": "taoryx.f16-fixed-source-trim-environment/v1alpha1",
        "pass": finite,
        "altitude_m": altitude_m,
        "trim_pitch_rad": trim_pitch_rad,
        "navigation_state": "fixed_local_origin_and_trim_attitude",
        "claim_boundary": (
            "The source derivative uses a fixed altitude and trim pitch. North/east position and attitude reported "
            "by this local screen are declared fixed context, not integrated navigation or attitude dynamics."
        ),
    }
    ####


def _status_samples(rows: list[dict[str, float | int | str]], mass_kg: float) -> tuple[BatchTruthSample, ...]:
    """Project the exact source local truth into the generic batch interface."""

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
                "controller_method": "lqi",
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
    """Retain each actual engineering-overlay coordinate at its committed boundary."""

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
        raise ValueError(f"F-16 physical LQI telemetry {name!r} must be finite numeric")
    return float(value)
    ####


def _text(values: Mapping[str, object], name: str) -> str:
    value = values.get(name)
    if not isinstance(value, str) or not value:
        raise ValueError(f"F-16 physical LQI telemetry {name!r} must be nonempty text")
    return value
    ####


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    if not rows:
        raise ValueError("F-16 physical LQI screen produced no telemetry rows")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    ####


__all__ = [
    "F16LocalPhysicalLqiScreenCapabilityAdapter",
    "F16LocalPhysicalLqiScreenExecution",
    "F16LocalPhysicalLqiScreenPlan",
    "compile_f16_local_physical_lqi_screen",
    "execute_f16_local_physical_lqi_screen",
    "preflight_f16_local_physical_lqi_screen",
]
