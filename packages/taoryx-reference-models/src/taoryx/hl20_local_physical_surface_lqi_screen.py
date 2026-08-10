"""Composition endpoint for local physical HL-20 source-surface LQI.

This screen retains the DAVE-ML Mach-1 scalar pitch-trim fragment's frozen
velocity/altitude fixture and controls only local attitude error/body rate.
It evaluates and allocates all seven named source surfaces at every committed
interval.  It deliberately does not promote the source fragment to a glide
equilibrium, navigation law, or flight qualification result.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .composition_control_trace import BatchControlSample, build_committed_control_trace, control_trace_summary
from .composition_evaluation import build_composition_trajectory_evaluation
from .composition_graph_evidence import unobserved_mission_graph_execution
from .composition_resource_ledger import build_committed_resource_ledger, resource_ledger_summary
from .composition_sensor_trace import BatchTruthSample
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .hl20_adapter import (
    HL20_SOURCE_PITCH_TRIM_ALPHA_DEG,
    HL20_SURFACE_LOCAL_STATE_NAMES,
    HL20SourceSurfaceLocalPlant,
    build_hl20_source_surface_local_plant,
    build_hl20_source_surface_physical_lqi_design,
)
from .hl20_controls import HL20_SURFACE_NAMES
from .mission_capability import MissionCapabilityEstimate, estimate_mission_capability
from .physical_lqr import PhysicalWrenchLqiValidation, validate_nonlinear_wrench_lqi
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    VehicleExecutionPreflight,
    build_concrete_capability_preflight_evidence,
    preflight_vehicle_composition,
)

_INITIALIZATION_ID = "source_mach1_pitch_trim_anchor"
_MISSION_ID = "hl20_source_surface_attitude_rate_lqi_screen_v1"
_SEGMENT_ID = "source_surface_attitude_rate_lqi_recovery_screen"
_ADAPTER_ID = "taoryx.hl20_source_surface_attitude_rate_lqi_screen.capability.v1"
_DT_S = 0.01
_DURATION_S = 2.0
_RECOVERY_LIMIT = 0.20
_STATE_NAMES = HL20_SURFACE_LOCAL_STATE_NAMES


@dataclass(frozen=True, slots=True)
class HL20LocalPhysicalSurfaceLqiScreenPlan:
    """Exact lowering record for the frozen-source local feedback screen."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_instance_id: str
    duration_s: float = _DURATION_S
    dt_s: float = _DT_S

    def manifest(self) -> dict[str, object]:
        """Return scope and explicit source-fragment/full-trim boundary."""

        return {
            "schema": "taoryx.hl20-local-physical-surface-lqi-screen-plan/v1alpha1",
            "family_id": self.family_id,
            "mission_id": self.mission_id,
            "fidelity": self.fidelity,
            "initialization_id": self.initialization_id,
            "segments": [{"instance_id": self.segment_instance_id, "id": _SEGMENT_ID, "transition_semantics": "screen_completion_only"}],
            "fixture": {
                "source_mach": 1.0,
                "source_pitch_trim_alpha_deg": HL20_SOURCE_PITCH_TRIM_ALPHA_DEG,
                "translation": "frozen_mach1_source_pitch_trim_body_velocity",
                "state_propagation": "local_attitude_error_and_body_rate_only",
            },
            "duration_s": self.duration_s,
            "dt_s": self.dt_s,
            "controller_method": "lqi",
            "control_realization": "source_surface_physical_wrench_lqi_allocation",
            "local_moment_balance_trim": {
                "status": "verified_by_execution",
                "residuals": ["p_rad_s", "q_rad_s", "r_rad_s"],
                "state_scope": list(_STATE_NAMES),
            },
            "source_pitch_trim_fragment": {"status": "verified", "scope": "scalar_pitch_coefficient_only"},
            "full_state_trim": {
                "status": "not_available",
                "reason": "the retained source binding fixes translation and has only a scalar pitch-trim fragment, not a gravity/force/attitude/full-glide equilibrium",
            },
            "claim_boundary": (
                "This is a bounded local HL-20 source-surface attitude/rate LQI recovery at a frozen Mach-1 source "
                "fixture. It does not establish full-state trim, translational propagation, glide guidance, navigation, "
                "arrival behavior, robustness, or flight qualification."
            ),
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class HL20LocalPhysicalSurfaceLqiScreenExecution:
    """Persisted nonlinear physical source-surface LQI result."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: HL20LocalPhysicalSurfaceLqiScreenPlan
    output_dir: Path
    runtime: dict[str, object]
    evaluation: dict[str, object]
    status_trace: dict[str, object]
    control_trace: dict[str, object]
    claim_boundary: str

    @property
    def screen_pass(self) -> bool:
        return self.evaluation["mission_pass"] is True
        ####

    def as_dict(self) -> dict[str, object]:
        """Serialize the public execution result."""

        return {
            "schema": "taoryx.hl20-local-physical-surface-lqi-screen-execution/v1alpha1",
            "status": "development_local_physical_lqi_screen_pass" if self.screen_pass else "development_local_physical_lqi_screen_failed",
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


def compile_hl20_local_physical_surface_lqi_screen(
    composition: CompiledVehicleComposition,
) -> HL20LocalPhysicalSurfaceLqiScreenPlan:
    """Fail closed unless Composition selected this exact narrow screen."""

    if composition.family_id != "hl20_mod_k":
        raise ValueError("HL-20 local physical surface LQI screen requires the hl20_mod_k family")
    if composition.mission != _MISSION_ID:
        raise ValueError(f"HL-20 local physical surface LQI screen requires mission {_MISSION_ID!r}")
    if composition.fidelity != "rigid_body_6dof_surface_allocated":
        raise ValueError("HL-20 local physical surface LQI screen requires surface-allocated rigid-body fidelity")
    if composition.initialization.id != _INITIALIZATION_ID or composition.initialization.inputs:
        raise ValueError("HL-20 local physical surface LQI screen requires its exact Mach-1 source initialization")
    if len(composition.segments) != 1 or composition.segments[0].id != _SEGMENT_ID or composition.segments[0].inputs:
        raise ValueError(f"HL-20 local physical surface LQI screen requires exactly one unmodified {_SEGMENT_ID!r} segment")
    return HL20LocalPhysicalSurfaceLqiScreenPlan(
        composition.family_id,
        composition.mission,
        composition.fidelity,
        composition.initialization.id,
        composition.segments[0].instance_id,
    )
    ####


class HL20LocalPhysicalSurfaceLqiScreenCapabilityAdapter:
    """Advertise the exact local physical source-surface path before execution."""

    id = _ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return composition.family_id == "hl20_mod_k" and composition.mission == _MISSION_ID and composition.fidelity == "rigid_body_6dof_surface_allocated"
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        plan = compile_hl20_local_physical_surface_lqi_screen(composition)
        plant = build_hl20_source_surface_local_plant()
        trim = plant.trim({}, {})
        design = build_hl20_source_surface_physical_lqi_design()
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": "source_surface_physical_wrench_lqi_allocation",
            "participating_nonlinear_source_load_evaluation": True,
            "physical_effector_allocation": True,
            "effector_names": list(HL20_SURFACE_NAMES),
            "effector_dynamics": "declared bounded first-order source-surface realization with 60 deg/s rate limit and 0.15 s lag",
            "controller_id": design.id,
            "controller_method": "lqi",
            "controller_hurwitz": design.result.hurwitz,
            "integral_output_names": list(design.result.output_names),
            "controlled_state_names": list(design.projection.state_names),
            "controlled_wrench_axes": list(design.projection.wrench_names),
            "local_moment_balance_trim": {"status": "verified" if trim.success else "failed", "max_residual_rad_s2": trim.max_residual, "controls_deg": dict(trim.controls)},
            "full_state_trim": plan.manifest()["full_state_trim"],
            "controller_automation": {"campaign_id": "hl20-source-surface-local-lqi-v1", "availability": "available_through_model_tune", "method": "lqi", "state_scope": list(design.projection.state_names)},
            "navigation_guidance": False,
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                "a Mach-1 source-fragment moment balance, derivative-consistent LQI design, bounded seven-surface allocation, and nonlinear fixed-fixture recovery are available",
                "full-state trim, translation, glide guidance, navigation, arrival behavior, robustness, and flight qualification remain separate gates",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####

    ####


def preflight_hl20_local_physical_surface_lqi_screen(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Prove exact Composition lowering and the source-derived feedback prerequisites."""

    plan = compile_hl20_local_physical_surface_lqi_screen(composition)
    estimate = estimate_mission_capability(composition)
    if estimate is None or estimate.adapter_id != _ADAPTER_ID:
        raise ValueError("HL-20 local physical surface LQI screen has no matching capability adapter")
    capability = estimate.manifest.get("capability")
    if not isinstance(capability, Mapping):
        raise ValueError("HL-20 local physical surface LQI screen has no matching capability record")
    design = build_hl20_source_surface_physical_lqi_design()
    trim = capability.get("local_moment_balance_trim")
    trim_ready = isinstance(trim, Mapping) and trim.get("status") == "verified"
    derivative_ready = design.projection.source_linearization.provenance.derivative_consistent
    controller_ready = capability.get("controller_hurwitz") is True
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status="translation_ready" if trim_ready and derivative_ready and controller_ready else "blocked",
        translator_id=_ADAPTER_ID,
        checks=(
            ExecutionPreflightCheck("hl20.semantic_local_physical_surface_lqi_screen", [_INITIALIZATION_ID, _SEGMENT_ID], [plan.initialization_id, plan.segment_instance_id], None, True),
            ExecutionPreflightCheck("hl20.local_moment_balance_trim", True, trim_ready, None, trim_ready),
            ExecutionPreflightCheck("hl20.source_surface_derivative_consistency", True, derivative_ready, None, derivative_ready),
            ExecutionPreflightCheck("hl20.source_surface_lqi_hurwitz", True, controller_ready, None, controller_ready),
        ),
        diagnostics=("composition lowers to the pinned HL-20 frozen-translation local attitude/rate LQI surface screen", *estimate.diagnostics),
        derived_mission=estimate.manifest,
        capability_estimate=build_concrete_capability_preflight_evidence(composition, estimate),
    )
    ####


def execute_hl20_local_physical_surface_lqi_screen(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None = None,
) -> HL20LocalPhysicalSurfaceLqiScreenExecution:
    """Run the local LQI demand through actual bounded seven-surface allocation."""

    if max_steps is not None:
        raise ValueError("--max-steps is unavailable for the fixed-cadence HL-20 physical surface LQI screen")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        raise ValueError(f"cannot execute HL-20 local physical surface LQI screen: {preflight.diagnostics}")
    plan = compile_hl20_local_physical_surface_lqi_screen(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)
    plant = build_hl20_source_surface_local_plant()
    trim = plant.trim({}, {})
    if not trim.success:
        raise RuntimeError(f"HL-20 source-surface moment-balance trim failed: {trim.as_dict()}")
    design = build_hl20_source_surface_physical_lqi_design()
    initial_state = dict(trim.state)
    initial_state.update({"roll_error_rad": 0.005, "pitch_error_rad": -0.002, "yaw_error_rad": 0.004})
    validation = validate_nonlinear_wrench_lqi(
        plant,
        trim,
        design,
        initial_state=initial_state,
        duration_s=plan.duration_s,
        dt_s=plan.dt_s,
        integral_lower={name: -0.5 for name in design.result.output_names},
        integral_upper={name: 0.5 for name in design.result.output_names},
    )
    assessment = _assess(validation)
    rows = _rows(validation, plant)
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.lifting_body.daveml.v1",
        "controller_id": design.id,
        "controller_method": "lqi",
        "integral_output_names": list(design.result.output_names),
        "control_realization": "source_surface_physical_wrench_lqi_allocation",
        "physical_effector_allocation": True,
        "effector_names": list(HL20_SURFACE_NAMES),
        "controlled_state_names": list(design.projection.state_names),
        "controlled_wrench_axes": list(design.projection.wrench_names),
        "effector_dynamics": "declared bounded first-order source-surface realization with 60 deg/s rate limit and 0.15 s lag",
        "integrators_exercised": validation.integrators_exercised,
        "control_saturation_fraction": validation.saturation_fraction,
        "local_moment_balance_trim": {"status": "verified", "max_residual_rad_s2": trim.max_residual},
        "full_state_trim": plan.manifest()["full_state_trim"],
        "dt_s": plan.dt_s,
        "duration_s": plan.duration_s,
        "mass_kg": plant.source_plant.mass_kg,
        "source_effectiveness_rank": int(np.linalg.matrix_rank(plant.effectiveness(trim.state, trim.controls).array)),
        "hard_gates_passed": assessment["screen_pass"],
        "mission_graph_execution": unobserved_mission_graph_execution(composition, "The HL-20 local source-surface LQI screen has no glide route or transition execution.").as_dict(),
    }
    evaluation = _evaluation(assessment, plan)
    status_trace = build_committed_status_trace(composition, _status_samples(rows, plant))
    control_trace = build_committed_control_trace(composition, _control_samples(rows))
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    runtime["resource_ledger"] = resource_ledger_summary(resource_ledger)
    result = HL20LocalPhysicalSurfaceLqiScreenExecution(
        composition, preflight, plan, destination, runtime, evaluation, status_trace, control_trace,
        "This is one two-second nonlinear fixed-translation HL-20 attitude/rate recovery. The LQI controller requests three body moments, which are allocated to all seven named source surfaces at every step. It does not establish full trim, translation, glide guidance, navigation, robustness, or flight qualification.",
    )
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "mission_graph_execution.json", runtime["mission_graph_execution"])
    _write_json(destination / "nonlinear_validation.json", validation.as_dict())
    _write_json(destination / "local_surface_lqi_screen.json", {"assessment": assessment, "rows": rows})
    _write_json(destination / "objective_report.json", evaluation)
    _write_json(destination / "status_trace.json", status_trace)
    _write_json(destination / "resource_ledger.json", resource_ledger)
    _write_json(destination / "semantic_action_trace.json", control_trace)
    _write_json(destination / "evaluation.json", build_composition_trajectory_evaluation(composition, preflight, evaluation, runtime=runtime, envelope={"local_moment_balance_trim": runtime["local_moment_balance_trim"]}, claim_boundary=result.claim_boundary, status_trace=status_trace, control_trace=control_trace).as_dict())
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _assess(validation: PhysicalWrenchLqiValidation) -> dict[str, object]:
    fraction = validation.final_normalized_feedback_error_norm / max(validation.initial_normalized_feedback_error_norm, 1.0e-12)
    statuses = set(validation.allocation_statuses)
    checks = {
        "derivative_consistent": validation.design.projection.source_linearization.provenance.derivative_consistent,
        "attitude_rate_recovery": fraction <= _RECOVERY_LIMIT,
        "integrators_exercised": validation.integrators_exercised,
        "no_allocation_saturation": validation.saturation_fraction == 0.0,
        "allocation_status": not bool(statuses & {"partially_achievable", "infeasible", "numerically_singular", "solver_failure"}),
        "finite_final_state": all(math.isfinite(value) for value in validation.final_state.values()),
    }
    return {
        "screen_pass": all(checks.values()), "checks": checks, "allocation_statuses": list(validation.allocation_statuses),
        "initial_normalized_feedback_error": validation.initial_normalized_feedback_error_norm,
        "final_normalized_feedback_error": validation.final_normalized_feedback_error_norm,
        "final_feedback_error_fraction": fraction, "saturation_fraction": validation.saturation_fraction,
        "maximum_controlled_actual_residual_nm": validation.maximum_controlled_actual_residual,
    }
    ####


def _evaluation(assessment: Mapping[str, object], plan: HL20LocalPhysicalSurfaceLqiScreenPlan) -> dict[str, object]:
    checks = assessment.get("checks")
    if not isinstance(checks, Mapping):
        raise ValueError("HL-20 local physical LQI assessment is missing checks")
    return {
        "schema": "taoryx.hl20-local-physical-surface-lqi-screen-evaluation/v1alpha1",
        "kind": "local_physical_control_screen", "mission_pass": assessment["screen_pass"],
        "results": [{"id": identifier, "status": "pass" if value is True else "fail", "required": True} for identifier, value in checks.items()],
        "metrics": {key: value for key, value in assessment.items() if key not in {"screen_pass", "checks"}},
        "screen_duration_s": plan.duration_s, "controller_method": "lqi",
        "control_realization": "source_surface_physical_wrench_lqi_allocation", "claim_boundary": plan.manifest()["claim_boundary"],
    }
    ####


def _rows(validation: PhysicalWrenchLqiValidation, plant: object) -> list[dict[str, float | int | str]]:
    if not hasattr(plant, "source_plant") or not hasattr(plant, "reference_source_state"):
        raise ValueError("HL-20 local source-surface plant lacks source-fixture access")
    source_plant = plant.source_plant
    source_reference = plant.reference_source_state
    rows: list[dict[str, float | int | str]] = []
    for sample in validation.samples:
        payload = sample.as_dict()
        state = _mapping(payload.get("state"), "sample state")
        requested = _mapping(payload.get("requested_wrench"), "requested wrench")
        achieved = _mapping(payload.get("achieved_wrench"), "achieved wrench")
        residual = _mapping(payload.get("achieved_residual"), "achieved residual")
        effectors = _mapping(payload.get("actual_effectors"), "actual effectors")
        actual = {name: _number(effectors, name) for name in HL20_SURFACE_NAMES}
        local_state = {**{name: float(source_reference[name]) for name in ("u_m_s", "v_m_s", "w_m_s", "altitude_m")}, **{name: _number(state, name) for name in ("p_rad_s", "q_rad_s", "r_rad_s")}}
        loads = source_plant.source.evaluate(
            (local_state["u_m_s"], local_state["v_m_s"], local_state["w_m_s"]), local_state["altitude_m"],
            (local_state["p_rad_s"], local_state["q_rad_s"], local_state["r_rad_s"]), controls=actual,
        )
        rows.append({
            "time_s": _number(payload, "time_s"),
            "u_m_s": local_state["u_m_s"],
            "v_m_s": local_state["v_m_s"],
            "w_m_s": local_state["w_m_s"],
            **{name: _number(state, name) for name in _STATE_NAMES},
            "source_pitch_coefficient": float(dict(loads.coefficients)["cm"]),
            "requested_pitch_moment_nm": _number(requested, "moment_y_nm"),
            "achieved_pitch_moment_nm": _number(achieved, "moment_y_nm"),
            "pitch_moment_residual_nm": _number(residual, "moment_y_nm"),
            "allocation_controlled_residual_norm": _number(payload, "achieved_controlled_residual_norm"),
            "allocation_status": _text(payload, "allocation_status"),
            "saturation_count": len(_strings(payload.get("position_saturated"), "position saturation")) + len(_strings(payload.get("rate_limited"), "rate limitation")),
            "source_effectiveness_rank": 3, "mass_kg": source_plant.mass_kg,
            **{f"surface_{name}_deg": actual[name] for name in HL20_SURFACE_NAMES},
        })
    return rows
    ####


def _status_samples(rows: list[dict[str, float | int | str]], plant: HL20SourceSurfaceLocalPlant) -> tuple[BatchTruthSample, ...]:
    source = plant.reference_source_state
    samples: list[BatchTruthSample] = []
    for index, row in enumerate(rows):
        raw: dict[str, object] = dict(row)
        raw.update({
            "local_attitude_rad": [_number(row, name) for name in ("roll_error_rad", "pitch_error_rad", "yaw_error_rad")],
            "body_velocity_m_s": [float(source[name]) for name in ("u_m_s", "v_m_s", "w_m_s")],
            "body_rate_rad_s": [_number(row, name) for name in ("p_rad_s", "q_rad_s", "r_rad_s")],
            "wrench_status": _text(row, "allocation_status"), "wrench_saturated": int(row["saturation_count"]) > 0,
            "physical_effector_allocation": True, "control_realization": "source_surface_physical_wrench_lqi_allocation",
            "controller_method": "lqi", "full_state_trim_status": "not_available", "local_moment_balance_trim_status": "verified",
        })
        samples.append(BatchTruthSample(time_s=_number(row, "time_s"), execution_status="completed" if index == len(rows) - 1 else "active", raw_values=raw))
    return tuple(samples)
    ####


def _control_samples(rows: list[dict[str, float | int | str]]) -> tuple[BatchControlSample, ...]:
    samples: list[BatchControlSample] = []
    previous: float | None = None
    for row in rows:
        time_s = _number(row, "time_s")
        samples.append(BatchControlSample(time_s if previous is None else previous, time_s, {}, {f"effector.surface.{name}.position": _number(row, f"surface_{name}_deg") for name in HL20_SURFACE_NAMES}))
        previous = time_s
    return tuple(samples)
    ####


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"HL-20 local physical LQI {label} must be a mapping")
    return value
    ####


def _strings(value: object, label: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ValueError(f"HL-20 local physical LQI {label} must be a list of strings")
    return tuple(value)
    ####


def _number(values: Mapping[str, object], name: str) -> float:
    value = values.get(name)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"HL-20 local physical LQI value {name!r} must be finite numeric")
    return float(value)
    ####


def _text(values: Mapping[str, object], name: str) -> str:
    value = values.get(name)
    if not isinstance(value, str) or not value:
        raise ValueError(f"HL-20 local physical LQI value {name!r} must be nonempty text")
    return value
    ####


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    if not rows:
        raise ValueError("HL-20 local physical LQI screen produced no telemetry rows")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    ####


__all__ = [
    "HL20LocalPhysicalSurfaceLqiScreenCapabilityAdapter",
    "HL20LocalPhysicalSurfaceLqiScreenExecution",
    "HL20LocalPhysicalSurfaceLqiScreenPlan",
    "compile_hl20_local_physical_surface_lqi_screen",
    "execute_hl20_local_physical_surface_lqi_screen",
    "preflight_hl20_local_physical_surface_lqi_screen",
]
