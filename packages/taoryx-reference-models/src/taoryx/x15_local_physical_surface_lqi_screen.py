"""Composition endpoint for bounded X-15 source-surface attitude/rate LQI.

The retained X-15 deck supplies three named aerodynamic source coordinates and
six-axis load evaluation at a pinned release/glide fixture.  This endpoint
uses the common physical-wrench LQI path to close only the locally observable
attitude-error/body-rate loop.  It retains the source translational fixture,
so it is not a full X-15 trim, a propagated flight state, or a high-energy
guidance result.
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
from .mission_capability import MissionCapabilityEstimate, estimate_mission_capability
from .physical_lqr import PhysicalWrenchLqiValidation, validate_nonlinear_wrench_lqi
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    VehicleExecutionPreflight,
    build_concrete_capability_preflight_evidence,
    preflight_vehicle_composition,
)
from .x15_adapter import (
    X15_MASS_KG,
    X15_SOURCE_SURFACE_NAMES,
    build_x15_source_surface_local_plant,
    build_x15_source_surface_physical_lqi_design,
)

_INITIALIZATION_ID = "source_release_glide_surface_authority_anchor"
_MISSION_ID = "x15_source_surface_attitude_rate_lqi_screen_v1"
_SEGMENT_ID = "source_surface_attitude_rate_lqi_recovery_screen"
_ADAPTER_ID = "taoryx.x15_source_surface_attitude_rate_lqi_screen.capability.v1"
_DT_S = 0.01
_DURATION_S = 2.0
_RECOVERY_LIMIT = 0.20
_STATE_NAMES = (
    "roll_error_rad",
    "pitch_error_rad",
    "yaw_error_rad",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
)
_WRENCH_NAMES = ("moment_x_nm", "moment_y_nm", "moment_z_nm")


@dataclass(frozen=True, slots=True)
class X15LocalPhysicalSurfaceLqiScreenPlan:
    """Exact lowering record for the pinned local source-surface recovery."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_instance_id: str
    duration_s: float = _DURATION_S
    dt_s: float = _DT_S

    def manifest(self) -> dict[str, object]:
        """Return the semantic scope and retained full-trim nonclaim."""

        return {
            "schema": "taoryx.x15-local-physical-surface-lqi-screen-plan/v1alpha1",
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
            "fixture": {
                "source": "x15_coherent_6dof_public_research_v1",
                "translation": "frozen_source_release_glide_body_velocity",
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
            "full_state_trim": {
                "status": "not_available",
                "reason": "the retained source binding fixes translation and does not close attitude, gravity, propulsion, reaction-control, or full vehicle equilibrium",
            },
            "claim_boundary": (
                "This is a bounded local X-15 source-surface attitude/rate LQI recovery at a frozen release fixture. "
                "It does not establish full-state trim, translational propagation, propulsion/RCS allocation, wind or "
                "mass robustness, navigation, high-energy guidance, or flight qualification."
            ),
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class X15LocalPhysicalSurfaceLqiScreenExecution:
    """Persisted result of one nonlinear X-15 physical LQI recovery."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: X15LocalPhysicalSurfaceLqiScreenPlan
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
            "schema": "taoryx.x15-local-physical-surface-lqi-screen-execution/v1alpha1",
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


def compile_x15_local_physical_surface_lqi_screen(
    composition: CompiledVehicleComposition,
) -> X15LocalPhysicalSurfaceLqiScreenPlan:
    """Fail closed unless Composition selected the exact source-local screen."""

    if composition.family_id != "x15":
        raise ValueError("X-15 local physical surface LQI screen requires the x15 family")
    if composition.mission != _MISSION_ID:
        raise ValueError(f"X-15 local physical surface LQI screen requires mission {_MISSION_ID!r}")
    if composition.fidelity != "rigid_body_6dof_surface_allocated":
        raise ValueError("X-15 local physical surface LQI screen requires surface-allocated rigid-body fidelity")
    if composition.initialization.id != _INITIALIZATION_ID or composition.initialization.inputs:
        raise ValueError("X-15 local physical surface LQI screen requires its exact frozen source-release initialization")
    if len(composition.segments) != 1 or composition.segments[0].id != _SEGMENT_ID:
        raise ValueError(f"X-15 local physical surface LQI screen requires exactly one {_SEGMENT_ID!r} segment")
    if composition.segments[0].inputs:
        raise ValueError("X-15 local physical surface LQI screen does not accept segment overrides")
    return X15LocalPhysicalSurfaceLqiScreenPlan(
        composition.family_id,
        composition.mission,
        composition.fidelity,
        composition.initialization.id,
        composition.segments[0].instance_id,
    )
    ####


class X15LocalPhysicalSurfaceLqiScreenCapabilityAdapter:
    """Advertise the exact source-surface LQI recovery before execution."""

    id = _ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "x15"
            and composition.mission == _MISSION_ID
            and composition.fidelity == "rigid_body_6dof_surface_allocated"
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        plan = compile_x15_local_physical_surface_lqi_screen(composition)
        plant = build_x15_source_surface_local_plant()
        trim = plant.trim({}, {})
        design = build_x15_source_surface_physical_lqi_design()
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": "source_surface_physical_wrench_lqi_allocation",
            "participating_nonlinear_source_load_evaluation": True,
            "physical_effector_allocation": True,
            "effector_names": list(X15_SOURCE_SURFACE_NAMES),
            "effector_dynamics": "source position bounds only; no source actuator rate or lag data",
            "controller_id": design.id,
            "controller_method": "lqi",
            "controller_hurwitz": design.result.hurwitz,
            "integral_output_names": list(design.result.output_names),
            "controlled_state_names": list(design.projection.state_names),
            "controlled_wrench_axes": list(design.projection.wrench_names),
            "local_moment_balance_trim": {
                "status": "verified" if trim.success else "failed",
                "max_residual_rad_s2": trim.max_residual,
                "controls_deg": dict(trim.controls),
            },
            "full_state_trim": plan.manifest()["full_state_trim"],
            "controller_automation": {
                "campaign_id": "x15-source-surface-local-lqi-v1",
                "availability": "available_through_model_tune",
                "method": "lqi",
                "state_scope": list(design.projection.state_names),
            },
            "navigation_guidance": False,
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                "a source-moment-balanced local attitude/rate reference, derivative-consistent LQI design, bounded three-surface allocation, and nonlinear fixed-fixture recovery are available",
                "full-state trim, translation, propulsion/RCS, wind/mass robustness, navigation, high-energy guidance, and flight qualification remain separate gates",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####

    ####


def preflight_x15_local_physical_surface_lqi_screen(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Prove that the exact Composition selection lowers to this local screen."""

    plan = compile_x15_local_physical_surface_lqi_screen(composition)
    estimate = estimate_mission_capability(composition)
    if estimate is None or estimate.adapter_id != _ADAPTER_ID:
        raise ValueError("X-15 local physical surface LQI screen has no matching capability adapter")
    capability = estimate.manifest.get("capability")
    if not isinstance(capability, Mapping):
        raise ValueError("X-15 local physical surface LQI capability is missing")
    design = build_x15_source_surface_physical_lqi_design()
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
            ExecutionPreflightCheck(
                "x15.semantic_local_physical_surface_lqi_screen",
                [_INITIALIZATION_ID, _SEGMENT_ID],
                [plan.initialization_id, plan.segment_instance_id],
                None,
                True,
            ),
            ExecutionPreflightCheck("x15.local_moment_balance_trim", True, trim_ready, None, trim_ready),
            ExecutionPreflightCheck("x15.source_surface_derivative_consistency", True, derivative_ready, None, derivative_ready),
            ExecutionPreflightCheck("x15.source_surface_lqi_hurwitz", True, controller_ready, None, controller_ready),
        ),
        diagnostics=(
            "composition lowers to the pinned X-15 frozen-translation local attitude/rate LQI surface screen",
            *estimate.diagnostics,
        ),
        derived_mission=estimate.manifest,
        capability_estimate=build_concrete_capability_preflight_evidence(composition, estimate),
    )
    ####


def execute_x15_local_physical_surface_lqi_screen(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None = None,
) -> X15LocalPhysicalSurfaceLqiScreenExecution:
    """Run local LQI through the actual bounded source-surface allocator."""

    if max_steps is not None:
        raise ValueError("--max-steps is unavailable for the fixed-cadence X-15 physical surface LQI screen")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        raise ValueError(f"cannot execute X-15 local physical surface LQI screen: {preflight.diagnostics}")
    plan = compile_x15_local_physical_surface_lqi_screen(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)
    plant = build_x15_source_surface_local_plant()
    trim = plant.trim({}, {})
    if not trim.success:
        raise RuntimeError(f"X-15 source-surface moment-balance trim failed: {trim.as_dict()}")
    design = build_x15_source_surface_physical_lqi_design()
    initial_state = dict(trim.state)
    initial_state.update(
        {
            "roll_error_rad": 0.005,
            "pitch_error_rad": -0.002,
            "yaw_error_rad": 0.004,
        }
    )
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
        "adapter_id": "taoryx.x15_source_surface_local.v1",
        "controller_id": design.id,
        "controller_method": "lqi",
        "integral_output_names": list(design.result.output_names),
        "control_realization": "source_surface_physical_wrench_lqi_allocation",
        "physical_effector_allocation": True,
        "effector_names": list(X15_SOURCE_SURFACE_NAMES),
        "controlled_state_names": list(design.projection.state_names),
        "controlled_wrench_axes": list(design.projection.wrench_names),
        "effector_dynamics": "source position bounds only; no source actuator rate or lag data",
        "integrators_exercised": validation.integrators_exercised,
        "control_saturation_fraction": validation.saturation_fraction,
        "local_moment_balance_trim": {"status": "verified", "max_residual_rad_s2": trim.max_residual},
        "full_state_trim": plan.manifest()["full_state_trim"],
        "dt_s": plan.dt_s,
        "duration_s": plan.duration_s,
        "mass_kg": X15_MASS_KG,
        "source_effectiveness_rank": int(np.linalg.matrix_rank(plant.effectiveness(trim.state, trim.controls).array)),
        "hard_gates_passed": assessment["screen_pass"],
        "mission_graph_execution": unobserved_mission_graph_execution(
            composition,
            "The X-15 local source-surface LQI screen has no route dispatcher or transition execution.",
        ).as_dict(),
    }
    evaluation = _evaluation(assessment, plan)
    status_trace = build_committed_status_trace(composition, _status_samples(rows))
    control_trace = build_committed_control_trace(composition, _control_samples(rows))
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    runtime["resource_ledger"] = resource_ledger_summary(resource_ledger)
    result = X15LocalPhysicalSurfaceLqiScreenExecution(
        composition=composition,
        preflight=preflight,
        plan=plan,
        output_dir=destination,
        runtime=runtime,
        evaluation=evaluation,
        status_trace=status_trace,
        control_trace=control_trace,
        claim_boundary=(
            "This is one two-second nonlinear fixed-translation X-15 attitude/rate recovery. The controller requests "
            "three body moments, which are allocated to the source symmetric stabilator, differential stabilator, and "
            "rudder at every step. It does not establish full trim, translation, propulsion/RCS, robustness, guidance, "
            "or flight qualification."
        ),
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
    _write_json(
        destination / "evaluation.json",
        build_composition_trajectory_evaluation(
            composition,
            preflight,
            evaluation,
            runtime=runtime,
            envelope={"local_moment_balance_trim": runtime["local_moment_balance_trim"]},
            claim_boundary=result.claim_boundary,
            status_trace=status_trace,
            control_trace=control_trace,
        ).as_dict(),
    )
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _assess(validation: PhysicalWrenchLqiValidation) -> dict[str, object]:
    """Apply narrow local-recovery gates without recasting them as flight tests."""

    final_fraction = validation.final_normalized_feedback_error_norm / max(
        validation.initial_normalized_feedback_error_norm,
        1.0e-12,
    )
    statuses = set(validation.allocation_statuses)
    checks = {
        "derivative_consistent": validation.design.projection.source_linearization.provenance.derivative_consistent,
        "attitude_rate_recovery": final_fraction <= _RECOVERY_LIMIT,
        "integrators_exercised": validation.integrators_exercised,
        "no_allocation_saturation": validation.saturation_fraction == 0.0,
        "allocation_status": not bool(statuses & {"partially_achievable", "infeasible", "numerically_singular", "solver_failure"}),
        "finite_final_state": all(math.isfinite(value) for value in validation.final_state.values()),
    }
    return {
        "screen_pass": all(checks.values()),
        "checks": checks,
        "allocation_statuses": list(validation.allocation_statuses),
        "initial_normalized_feedback_error": validation.initial_normalized_feedback_error_norm,
        "final_normalized_feedback_error": validation.final_normalized_feedback_error_norm,
        "final_feedback_error_fraction": final_fraction,
        "saturation_fraction": validation.saturation_fraction,
        "maximum_controlled_actual_residual_nm": validation.maximum_controlled_actual_residual,
    }
    ####


def _evaluation(assessment: Mapping[str, object], plan: X15LocalPhysicalSurfaceLqiScreenPlan) -> dict[str, object]:
    checks = assessment.get("checks")
    if not isinstance(checks, Mapping):
        raise ValueError("X-15 local physical LQI assessment is missing checks")
    return {
        "schema": "taoryx.x15-local-physical-surface-lqi-screen-evaluation/v1alpha1",
        "kind": "local_physical_control_screen",
        "mission_pass": assessment["screen_pass"],
        "results": [
            {"id": identifier, "status": "pass" if value is True else "fail", "required": True}
            for identifier, value in checks.items()
        ],
        "metrics": {key: value for key, value in assessment.items() if key not in {"screen_pass", "checks"}},
        "screen_duration_s": plan.manifest()["duration_s"],
        "controller_method": "lqi",
        "control_realization": "source_surface_physical_wrench_lqi_allocation",
        "claim_boundary": plan.manifest()["claim_boundary"],
    }
    ####


def _rows(
    validation: PhysicalWrenchLqiValidation,
    plant: object,
) -> list[dict[str, float | int | str]]:
    """Flatten nonlinear feedback, allocation, and source-table loads into truth rows."""

    if not hasattr(plant, "source_plant"):
        raise ValueError("X-15 local source-surface screen plant lacks source table access")
    source_plant = plant.source_plant
    rows: list[dict[str, float | int | str]] = []
    for sample in validation.samples:
        payload = sample.as_dict()
        state = _mapping(payload.get("state"), "sample state")
        requested = _mapping(payload.get("requested_wrench"), "requested wrench")
        achieved = _mapping(payload.get("achieved_wrench"), "achieved wrench")
        residual = _mapping(payload.get("achieved_residual"), "achieved residual")
        effectors = _mapping(payload.get("actual_effectors"), "actual effectors")
        local_state = {
            **{
                name: float(source_plant.reference_state[name])
                for name in ("u_m_s", "v_m_s", "w_m_s")
            },
            **{name: _number(state, name) for name in ("p_rad_s", "q_rad_s", "r_rad_s")},
        }
        actual = {name: _number(effectors, name) for name in X15_SOURCE_SURFACE_NAMES}
        _, loads = source_plant.source_surface_loads(local_state, actual)
        rows.append(
            {
                "time_s": _number(payload, "time_s"),
                "u_m_s": local_state["u_m_s"],
                "v_m_s": local_state["v_m_s"],
                "w_m_s": local_state["w_m_s"],
                **{name: _number(state, name) for name in _STATE_NAMES},
                **{f"requested_moment_{axis}_nm": _number(requested, f"moment_{axis}_nm") for axis in ("x", "y", "z")},
                **{f"achieved_moment_{axis}_nm": _number(achieved, f"moment_{axis}_nm") for axis in ("x", "y", "z")},
                **{f"residual_moment_{axis}_nm": _number(residual, f"moment_{axis}_nm") for axis in ("x", "y", "z")},
                **{f"surface_{name}_deg": actual[name] for name in X15_SOURCE_SURFACE_NAMES},
                "source_mach": float(loads["mach"]),
                "source_alpha_deg": float(loads["alpha_deg"]),
                "source_beta_deg": float(loads["beta_deg"]),
                "allocation_status": _text(payload, "allocation_status"),
                "saturation_count": len(_strings(payload.get("position_saturated"), "position saturation")) + len(_strings(payload.get("rate_limited"), "rate limitation")),
                "allocation_controlled_residual_norm": _number(payload, "achieved_controlled_residual_norm"),
                "mass_kg": X15_MASS_KG,
                "source_effectiveness_rank": 3,
            }
        )
    return rows
    ####


def _status_samples(rows: list[dict[str, float | int | str]]) -> tuple[BatchTruthSample, ...]:
    """Project the local control truth into the X-15 advertised status contract."""

    source_velocity = build_x15_source_surface_local_plant().source_plant.reference_state
    samples: list[BatchTruthSample] = []
    for index, row in enumerate(rows):
        raw: dict[str, object] = dict(row)
        raw.update(
            {
                "local_attitude_rad": [_number(row, name) for name in ("roll_error_rad", "pitch_error_rad", "yaw_error_rad")],
                "body_rate_rad_s": [_number(row, name) for name in ("p_rad_s", "q_rad_s", "r_rad_s")],
                "body_velocity_m_s": [float(source_velocity[name]) for name in ("u_m_s", "v_m_s", "w_m_s")],
                "requested_moment_body_nm": [_number(row, f"requested_moment_{axis}_nm") for axis in ("x", "y", "z")],
                "achieved_moment_body_nm": [_number(row, f"achieved_moment_{axis}_nm") for axis in ("x", "y", "z")],
                "residual_moment_body_nm": [_number(row, f"residual_moment_{axis}_nm") for axis in ("x", "y", "z")],
                "wrench_status": _text(row, "allocation_status"),
                "wrench_saturated": int(row["saturation_count"]) > 0,
                "physical_effector_allocation": True,
                "control_realization": "source_surface_physical_wrench_lqi_allocation",
                "controller_method": "lqi",
                "full_state_trim_status": "not_available",
                "local_moment_balance_trim_status": "verified",
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
    """Retain actual named source-surface positions for each committed interval."""

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
                    f"effector.surface.{name}.position": _number(row, f"surface_{name}_deg")
                    for name in X15_SOURCE_SURFACE_NAMES
                },
            )
        )
        previous_time = time_s
    return tuple(samples)
    ####


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"X-15 local physical LQI {label} must be a mapping")
    return value
    ####


def _strings(value: object, label: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ValueError(f"X-15 local physical LQI {label} must be a list of strings")
    return tuple(value)
    ####


def _number(values: Mapping[str, object], name: str) -> float:
    value = values.get(name)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"X-15 local physical LQI value {name!r} must be finite numeric")
    return float(value)
    ####


def _text(values: Mapping[str, object], name: str) -> str:
    value = values.get(name)
    if not isinstance(value, str) or not value:
        raise ValueError(f"X-15 local physical LQI value {name!r} must be nonempty text")
    return value
    ####


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    if not rows:
        raise ValueError("X-15 local physical LQI screen produced no telemetry rows")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    ####


__all__ = [
    "X15LocalPhysicalSurfaceLqiScreenCapabilityAdapter",
    "X15LocalPhysicalSurfaceLqiScreenExecution",
    "X15LocalPhysicalSurfaceLqiScreenPlan",
    "compile_x15_local_physical_surface_lqi_screen",
    "execute_x15_local_physical_surface_lqi_screen",
    "preflight_x15_local_physical_surface_lqi_screen",
]
