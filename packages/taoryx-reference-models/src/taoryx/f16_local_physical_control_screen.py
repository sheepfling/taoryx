"""Composition-owned local physical-control screens for the F-16 S-119.

The F-16 source package already supports a bounded local wrench controller and
physical surface allocation.  This module makes that existing evidence an
explicit Vehicle Composition endpoint without misrepresenting its one-second
controlled entry screen as a completed racetrack or flight qualification.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from taoryx_reference_models.resources import model_resource_root

from .composition_control_trace import (
    BatchControlSample,
    build_committed_control_trace,
    control_trace_summary,
)
from .composition_evaluation import build_composition_trajectory_evaluation
from .composition_graph_evidence import unobserved_mission_graph_execution
from .composition_resource_ledger import (
    build_committed_resource_ledger,
    resource_ledger_summary,
)
from .composition_sensor_trace import BatchTruthSample
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .mission_capability import MissionCapabilityEstimate, estimate_mission_capability
from .powered_fixed_wing_mission_compiler import (
    CapabilityScaledRacetrack,
    compile_powered_fixed_wing_racetrack,
    resolve_powered_fixed_wing_mission_profile,
)
from .source_f16 import (
    F16_SOURCE_SIDECAR,
    build_f16_local_physical_wrench_design,
    build_f16_local_physical_wrench_lqi_design,
    build_f16_source_physical_plant,
)
from .trajectory import F16RacetrackRunner
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    VehicleExecutionPreflight,
    build_concrete_capability_preflight_evidence,
    preflight_vehicle_composition,
)

_ROOT = model_resource_root()
_MISSION_ID = "f16_local_physical_control_screen_v1"
_INITIALIZATION_ID = "source_physical_control_local_point"
_SEGMENT_ID = "local_physical_control_screen"
_CAPABILITY_ADAPTER_ID = "taoryx.f16_local_physical_control_screen.capability.v1"
_SCREEN_DURATION_S = 1.0
_SCREEN_DT_S = 0.2
_Mode = Literal["direct_wrench", "surface_allocated"]


@dataclass(frozen=True, slots=True)
class F16LocalPhysicalControlScreenPlan:
    """Exact parameter-free lowering for one F-16 physical control screen."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_instance_id: str
    mode: _Mode
    duration_s: float
    dt_s: float

    def manifest(self) -> dict[str, object]:
        """Return the public lowering record without inventing mission scope."""

        return {
            "schema": "taoryx.f16-local-physical-control-screen-plan/v1alpha1",
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
            "mode": self.mode,
            "duration_s": self.duration_s,
            "dt_s": self.dt_s,
            "claim_boundary": (
                "This selects a pinned F-16 local controlled-entry screen. It does not execute the full racetrack, "
                "a scheduled flight controller, an envelope sweep, or a qualified vehicle mission."
            ),
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class F16LocalPhysicalControlScreenExecution:
    """Result of one source-backed F-16 direct or allocated-surface screen."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: F16LocalPhysicalControlScreenPlan
    proposal: CapabilityScaledRacetrack
    output_dir: Path
    runtime: dict[str, object]
    envelope: dict[str, object]
    evaluation: dict[str, object]
    status_trace: dict[str, object]
    control_trace: dict[str, object]
    claim_boundary: str

    @property
    def screen_pass(self) -> bool:
        """Return only the narrow local-screen disposition."""

        return bool(self.evaluation["mission_pass"])
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a compact public execution record."""

        return {
            "schema": "taoryx.f16-local-physical-control-screen-execution/v1alpha1",
            "status": "development_local_screen_pass" if self.screen_pass else "development_local_screen_failed",
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "plan": self.plan.manifest(),
            "proposal": self.proposal.manifest(),
            "output_dir": str(self.output_dir),
            "runtime": self.runtime,
            "envelope": self.envelope,
            "control_screen": self.evaluation,
            "screen_pass": self.screen_pass,
            "status_trace": status_trace_summary(self.status_trace),
            "control_trace": control_trace_summary(self.control_trace),
            "claim_boundary": self.claim_boundary,
        }
        ####

    ####


def compile_f16_local_physical_control_screen(
    composition: CompiledVehicleComposition,
) -> F16LocalPhysicalControlScreenPlan:
    """Validate the intentionally fixed source-local physical-control screen."""

    if composition.family_id != "f16_s119":
        raise ValueError("F-16 physical control screen requires the f16_s119 family")
    if composition.mission != _MISSION_ID:
        raise ValueError(f"F-16 physical control screen requires mission {_MISSION_ID!r}")
    modes: dict[str, _Mode] = {
        "rigid_body_6dof_direct_wrench": "direct_wrench",
        "rigid_body_6dof_surface_allocated": "surface_allocated",
    }
    try:
        mode = modes[composition.fidelity]
    except KeyError as error:
        raise ValueError("F-16 physical control screen requires a direct-wrench or surface-allocated fidelity") from error
    if composition.initialization.id != _INITIALIZATION_ID:
        raise ValueError(f"F-16 physical control screen requires initialization {_INITIALIZATION_ID!r}")
    if composition.initialization.inputs:
        raise ValueError("F-16 physical control screen does not accept initialization overrides")
    if len(composition.segments) != 1 or composition.segments[0].id != _SEGMENT_ID:
        raise ValueError(f"F-16 physical control screen requires exactly one {_SEGMENT_ID!r} segment")
    segment = composition.segments[0]
    if segment.inputs:
        raise ValueError("F-16 physical control screen does not accept segment overrides")
    return F16LocalPhysicalControlScreenPlan(
        family_id=composition.family_id,
        mission_id=composition.mission,
        fidelity=composition.fidelity,
        initialization_id=composition.initialization.id,
        segment_instance_id=segment.instance_id,
        mode=mode,
        duration_s=_SCREEN_DURATION_S,
        dt_s=_SCREEN_DT_S,
    )
    ####


class F16LocalPhysicalControlScreenCapabilityAdapter:
    """Expose the concrete source plant/controller screen for discovery and preflight."""

    id = _CAPABILITY_ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether this exact F-16 local screen is selected."""

        return (
            composition.family_id == "f16_s119"
            and composition.mission == _MISSION_ID
            and composition.fidelity in {"rigid_body_6dof_direct_wrench", "rigid_body_6dof_surface_allocated"}
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Describe bounded local screen authority without promoting it to flight capability."""

        plan = compile_f16_local_physical_control_screen(composition)
        plant = build_f16_source_physical_plant()
        design = build_f16_local_physical_wrench_design()
        manifest = plan.manifest()
        capability: dict[str, object] = {
            "control_realization": "direct_wrench" if plan.mode == "direct_wrench" else "surface_allocated",
            "participating_nonlinear_plant": True,
            "source_physical_trim": True,
            "navigation_guidance": False,
            "screen_duration_s": plan.duration_s,
            "integration_dt_s": plan.dt_s,
            "controller_id": design.id,
            "controller_hurwitz": design.result.hurwitz,
            "effector_names": list(design.projection.effector_names),
            "source_mass_kg": plant.source.mass_kg,
            "physical_effector_allocation": plan.mode == "surface_allocated",
        }
        if plan.mode == "surface_allocated":
            lqi_design = build_f16_local_physical_wrench_lqi_design()
            capability["offset_free_tuning_candidate"] = {
                "campaign_id": "f16-source-surface-local-lqi-v1",
                "method": "lqi",
                "availability": "available_through_model_tune",
                "controller_id": lqi_design.id,
                "controller_hurwitz": lqi_design.result.hurwitz,
                "controlled_state_names": list(lqi_design.projection.state_names),
                "controlled_wrench_axes": list(lqi_design.projection.wrench_names),
                "integral_output_names": list(lqi_design.result.output_names),
                "integral_q_diagonal": list(lqi_design.integral_q_diagonal),
                "physical_allocation_baseline": "focused_bounded_source_local_recovery_passed",
                "physical_screen_status": "not_executed_by_this_lqr_screen",
                "physical_screen_execution": {
                    "status": "executed_by_paired_lqi_screen",
                    "mission_id": "f16_local_physical_surface_lqi_screen_v1",
                    "capability_adapter_id": "taoryx.f16_local_physical_surface_lqi_screen.capability.v1",
                    "operations": ["validate", "batch"],
                    "control_realization": "surface_allocated",
                },
                "persistent_disturbance_status": "not_executable_without_a_declared_source_wind_or_mass_derivative_environment",
                "claim_boundary": (
                    "The public batch screen executes its separately validated LQR controller. The named velocity "
                    "LQI design passes a focused bounded-allocation source-local recovery baseline through the "
                    "declared engineering surface/throttle overlay. It does not establish a wind, mass, or "
                    "persistent-disturbance rejection result, a scheduled controller, an F-16 racetrack, or flight "
                    "qualification."
                ),
            }
        manifest["capability"] = capability
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                "pinned F-16 source trim, local wrench LQR, and one-second controlled entry are available; "
                "full racetrack and scheduled-envelope execution remain separate gates",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####


def preflight_f16_local_physical_control_screen(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Preflight the F-16's exact local physical-control Composition route."""

    plan = compile_f16_local_physical_control_screen(composition)
    estimate = estimate_mission_capability(composition)
    if estimate is None or estimate.adapter_id != _CAPABILITY_ADAPTER_ID:
        raise ValueError("F-16 local physical control screen has no matching installed capability adapter")
    capability = estimate.manifest.get("capability")
    if not isinstance(capability, Mapping):
        raise ValueError("F-16 local physical control capability record is missing")
    controller_hurwitz = capability.get("controller_hurwitz") is True
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status="translation_ready" if controller_hurwitz else "blocked",
        translator_id=_CAPABILITY_ADAPTER_ID,
        checks=(
            ExecutionPreflightCheck(
                "f16.semantic_local_physical_control_screen",
                [_INITIALIZATION_ID, _SEGMENT_ID],
                [plan.initialization_id, plan.segment_instance_id],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                "f16.local_physical_wrench_controller_hurwitz",
                True,
                controller_hurwitz,
                None,
                controller_hurwitz,
            ),
        ),
        diagnostics=(
            "composition lowers exactly to the F-16 source-trim local physical-control screen; it is not a full route or scheduled-envelope translator",
            *estimate.diagnostics,
        ),
        derived_mission=estimate.manifest,
        capability_estimate=build_concrete_capability_preflight_evidence(composition, estimate),
    )
    ####


def execute_f16_local_physical_control_screen(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None = None,
) -> F16LocalPhysicalControlScreenExecution:
    """Run the pinned F-16 local source control screen through Composition."""

    if max_steps is not None:
        raise ValueError("--max-steps is unavailable for the fixed-cadence F-16 physical-control screen")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        details = "; ".join(preflight.diagnostics) or "no translation-ready local physical screen"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {details}")
    plan = compile_f16_local_physical_control_screen(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    proposal = _screen_proposal(plan)
    plant = build_f16_source_physical_plant()
    design = build_f16_local_physical_wrench_design()
    run = F16RacetrackRunner(
        plant.source,
        plant.trim_result,
        design,
        proposal.route,
        plan.mode,
        plant if plan.mode == "surface_allocated" else None,
        dt_s=plan.dt_s,
    ).run(duration_s=plan.duration_s)
    rows = [dict(row) for row in run.rows]
    for row in rows:
        row["mass_kg"] = float(plant.source.mass_kg)
        row["schedule_node_id"] = "f16-sea-level-152mps"
        row["controller_selection"] = "fixed_source_trim_node"
    envelope = _f16_envelope(rows)
    finite = _finite_rows(rows)
    allocation_pass = plan.mode == "direct_wrench" or all(
        row.get("allocation_status") in {"feasible", "feasible_near_limit"} and int(row["saturation_count"]) == 0
        for row in rows
    )
    screen_pass = bool(run.numerical_valid and finite and envelope["pass"] and allocation_pass)
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.fixed_wing.daveml.v1",
        "controller_id": design.id,
        "controller_method": "lqr",
        "mode": plan.mode,
        "control_realization": "direct_wrench" if plan.mode == "direct_wrench" else "surface_allocated",
        "physical_effector_allocation": plan.mode == "surface_allocated",
        "controller_selection": "fixed_source_trim_node",
        "dt_s": plan.dt_s,
        "duration_s": float(rows[-1]["time_s"]) if rows else 0.0,
        "numerical_valid": run.numerical_valid,
        "failure": run.failure,
        "allocation_pass": allocation_pass,
        "hard_gates_passed": screen_pass,
        "mission_graph_execution": unobserved_mission_graph_execution(
            composition,
            "The local physical-control screen has no route graph dispatcher and makes no mission-transition claim.",
        ).as_dict(),
    }
    evaluation = _screen_evaluation(run.numerical_valid, finite, envelope, allocation_pass, screen_pass, plan)
    status_trace = build_committed_status_trace(
        composition,
        _status_samples(rows, plan.mode, plant.source.mass_kg),
    )
    control_trace = build_committed_control_trace(
        composition,
        _control_samples(rows, plan.mode),
    )
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    runtime["resource_ledger"] = resource_ledger_summary(resource_ledger)
    result = F16LocalPhysicalControlScreenExecution(
        composition=composition,
        preflight=preflight,
        plan=plan,
        proposal=proposal,
        output_dir=destination,
        runtime=runtime,
        envelope=envelope,
        evaluation=evaluation,
        status_trace=status_trace,
        control_trace=control_trace,
        claim_boundary=(
            "This is a F-16 source-trim local controlled-entry screen. Direct-wrench results are a bounded generalized-force comparison. "
            "Surface results report the declared engineering actuator overlay and allocator at this operating point only; neither is scheduled-envelope or flight qualification."
        ),
    )
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_json(destination / "proposal.json", proposal.manifest())
    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "mission_graph_execution.json", runtime["mission_graph_execution"])
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


def _screen_proposal(plan: F16LocalPhysicalControlScreenPlan) -> CapabilityScaledRacetrack:
    """Derive the local screen's fixed first-entry reference from the F-16 profile."""

    fidelity = (
        "rigid_body_6dof_direct_wrench"
        if plan.mode == "direct_wrench"
        else "rigid_body_6dof_surface_allocated"
    )
    binding_id = "f16-s119-direct-wrench" if plan.mode == "direct_wrench" else "f16-s119-surfaces"
    capability, intent = resolve_powered_fixed_wing_mission_profile(
        _ROOT / "verification/powered_fixed_wing_mission_profiles.yaml",
        "f16-subsonic",
        fidelity,
    )
    return compile_powered_fixed_wing_racetrack(
        capability,
        intent,
        binding_id=binding_id,
        fidelity=fidelity,
        source_realization=(
            "f16_source_local_direct_wrench_control_screen"
            if plan.mode == "direct_wrench"
            else "f16_source_local_surface_allocated_control_screen"
        ),
    )
    ####


def _f16_envelope(rows: list[dict[str, float | int | str]]) -> dict[str, object]:
    """Check every local screen row against the source package domain."""

    payload = json.loads(F16_SOURCE_SIDECAR.read_text(encoding="utf-8"))
    validity = payload["package"]["validity_envelope"]
    bounds = (
        ("altitude_m", float(validity["altitude_min_m"]), float(validity["altitude_max_m"])),
        ("mach", float(validity["mach_min"]), float(validity["mach_max"])),
        ("aero_alpha_deg", math.degrees(float(validity["alpha_min_rad"])), math.degrees(float(validity["alpha_max_rad"]))),
        ("aero_sideslip_deg", math.degrees(float(validity["beta_min_rad"])), math.degrees(float(validity["beta_max_rad"]))),
    )
    checks: list[dict[str, object]] = []
    for identifier, lower, upper in bounds:
        tolerance = 1.0e-9 * max(1.0, abs(lower), abs(upper))
        violations = [
            {"time_s": row.get("time_s"), "value": row.get(identifier), "lower": lower, "upper": upper}
            for row in rows
            if not _in_range(row.get(identifier), lower, upper, tolerance)
        ]
        checks.append({"channel": identifier, "minimum": lower, "maximum": upper, "violations": violations, "pass": not violations})
    return {"schema_version": 1, "checks": checks, "pass": all(bool(item["pass"]) for item in checks)}
    ####


def _screen_evaluation(
    numerical_valid: bool,
    finite: bool,
    envelope: Mapping[str, object],
    allocation_pass: bool,
    screen_pass: bool,
    plan: F16LocalPhysicalControlScreenPlan,
) -> dict[str, object]:
    """State the deliberately local pass rule independently from route objectives."""

    return {
        "schema": "taoryx.f16-local-physical-control-screen-evaluation/v1alpha1",
        "kind": "local_physical_control_screen",
        "mission_pass": screen_pass,
        "results": [
            {"id": "numerical_valid", "status": "pass" if numerical_valid else "fail", "required": True},
            {"id": "finite_telemetry", "status": "pass" if finite else "fail", "required": True},
            {"id": "source_envelope", "status": "pass" if envelope.get("pass") is True else "fail", "required": True},
            {"id": "allocation", "status": "pass" if allocation_pass else "fail", "required": True},
        ],
        "screen_duration_s": plan.duration_s,
        "controller_method": "lqr",
        "control_realization": "direct_wrench" if plan.mode == "direct_wrench" else "surface_allocated",
        "claim_boundary": (
            "All gates apply only to the pinned one-second local controlled entry. This is not a racetrack truth-objective, "
            "gain-schedule, robustness, or flight-qualification evaluation."
        ),
    }
    ####


def _status_samples(
    rows: list[dict[str, float | int | str]],
    mode: _Mode,
    mass_kg: float,
) -> tuple[BatchTruthSample, ...]:
    """Project source-owned control and plant telemetry through the portable interface."""

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
                "wrench_status": str(row["allocation_status"]),
                "wrench_saturated": int(row["saturation_count"]) > 0,
                "physical_effector_allocation": mode == "surface_allocated",
                "control_realization": "direct_wrench" if mode == "direct_wrench" else "surface_allocated",
                "controller_method": "lqr",
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


def _control_samples(
    rows: list[dict[str, float | int | str]],
    mode: _Mode,
) -> tuple[BatchControlSample, ...]:
    """Retain physical achieved-effectors only where the selected tier owns them."""

    samples: list[BatchControlSample] = []
    previous_time: float | None = None
    for row in rows:
        time_s = _number(row, "time_s")
        effectors: dict[str, object] = {}
        if mode == "surface_allocated":
            effectors = {
                "effector.elevator.position": _number(row, "elevator_deg"),
                "effector.aileron.position": _number(row, "aileron_deg"),
                "effector.rudder.position": _number(row, "rudder_deg"),
                "effector.throttle.position": _number(row, "throttle_fraction"),
            }
        samples.append(
            BatchControlSample(
                interval_start_time_s=time_s if previous_time is None else previous_time,
                committed_truth_time_s=time_s,
                requested_actions={},
                achieved_effectors=effectors,
            )
        )
        previous_time = time_s
    return tuple(samples)
    ####


def _finite_rows(rows: list[dict[str, float | int | str]]) -> bool:
    """Reject non-finite numeric source telemetry."""

    return bool(rows) and all(
        math.isfinite(float(value))
        for row in rows
        for value in row.values()
        if isinstance(value, int | float) and not isinstance(value, bool)
    )
    ####


def _in_range(value: object, lower: float, upper: float, tolerance: float) -> bool:
    """Return whether one source telemetry value remains inside a bound."""

    return (
        isinstance(value, int | float)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
        and lower - tolerance <= float(value) <= upper + tolerance
    )
    ####


def _number(row: Mapping[str, float | int | str], identifier: str) -> float:
    """Read a required finite F-16 telemetry scalar."""

    value = row.get(identifier)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"F-16 physical-control telemetry {identifier!r} must be finite numeric")
    return float(value)
    ####


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    """Write complete source telemetry with stable columns."""

    fields = sorted({identifier for row in rows for identifier in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    ####


def _write_json(path: Path, value: object) -> None:
    """Write deterministic readable artifacts."""

    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


__all__ = [
    "F16LocalPhysicalControlScreenCapabilityAdapter",
    "F16LocalPhysicalControlScreenExecution",
    "F16LocalPhysicalControlScreenPlan",
    "compile_f16_local_physical_control_screen",
    "execute_f16_local_physical_control_screen",
    "preflight_f16_local_physical_control_screen",
]
