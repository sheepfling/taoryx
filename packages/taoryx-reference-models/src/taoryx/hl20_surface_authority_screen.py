"""Public Composition endpoint for a bounded HL-20 source-surface authority probe.

The retained DAVE-ML data proves a Mach-1 scalar pitch-coefficient trim
fragment, and evaluates all seven named surface inputs.  It does *not* supply
the attitude, gravity, position, or full-equilibrium bindings required for a
flight-control or glide screen.  This endpoint makes the available evidence
useful without widening that claim: a fixed source fixture, a requested pitch
moment increment, bounded surface allocation with declared lag, and a fresh
nonlinear source-load evaluation at every committed sample.
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
from .control_allocation import allocate_and_advance_wrench
from .hl20_adapter import HL20_TRIM_EVIDENCE, HL20SourceSurfacePlant
from .hl20_controls import HL20_SOURCE_SURFACE_BOUNDS_DEG, HL20_SURFACE_NAMES
from .hl20_source_aerodynamics import HL20_SOURCE_SPEED_OF_SOUND_M_S
from .mission_capability import MissionCapabilityEstimate, estimate_mission_capability
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    VehicleExecutionPreflight,
    build_concrete_capability_preflight_evidence,
    preflight_vehicle_composition,
)

_INITIALIZATION_ID = "source_mach1_pitch_trim_anchor"
_MISSION_ID = "hl20_source_surface_pitch_authority_screen_v1"
_SEGMENT_ID = "source_surface_pitch_authority_allocation_screen"
_ADAPTER_ID = "taoryx.hl20_source_surface_pitch_authority_screen.capability.v1"
_DT_S = 0.05
_DURATION_S = 2.0
_STEP_COUNT = int(_DURATION_S / _DT_S)
_REQUESTED_PITCH_INCREMENT_NM = 1_000.0
_TRIM_ALPHA_DEG = 6.457652069


@dataclass(frozen=True, slots=True)
class HL20SurfaceAuthorityScreenPlan:
    """The fixed semantic lowering record for the source-surface probe."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_instance_id: str

    def manifest(self) -> dict[str, object]:
        """Return exact scope, including the important full-trim nonclaim."""

        return {
            "schema": "taoryx.hl20-source-surface-authority-screen-plan/v1alpha1",
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
                "source_mach": 1.0,
                "alpha_deg": _TRIM_ALPHA_DEG,
                "altitude_m": 0.0,
                "body_rates_rad_s": [0.0, 0.0, 0.0],
                "state_propagation": "not_performed",
            },
            "duration_s": _DURATION_S,
            "dt_s": _DT_S,
            "requested_pitch_moment_increment_nm": _REQUESTED_PITCH_INCREMENT_NM,
            "control_realization": "source_surface_pitch_authority_allocation",
            "full_state_trim": {
                "status": "not_available",
                "reason": "the retained source evidence verifies only a scalar pitch-coefficient trim fragment",
            },
            "claim_boundary": (
                "This is a frozen-source-fixture pitch-authority and surface-allocation screen. It does not propagate "
                "state, solve a six-DOF equilibrium, synthesize feedback, navigate, guide a glide, or qualify flight."
            ),
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class HL20SurfaceAuthorityScreenExecution:
    """Result and persisted evidence for the narrow nonlinear source probe."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: HL20SurfaceAuthorityScreenPlan
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
        return {
            "schema": "taoryx.hl20-source-surface-authority-screen-execution/v1alpha1",
            "status": "development_source_surface_authority_pass" if self.screen_pass else "development_source_surface_authority_failed",
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


def compile_hl20_source_surface_authority_screen(
    composition: CompiledVehicleComposition,
) -> HL20SurfaceAuthorityScreenPlan:
    """Fail closed unless Composition selected this exact authority screen."""

    if composition.family_id != "hl20_mod_k":
        raise ValueError("HL-20 source-surface authority screen requires the hl20_mod_k family")
    if composition.mission != _MISSION_ID:
        raise ValueError(f"HL-20 source-surface authority screen requires mission {_MISSION_ID!r}")
    if composition.fidelity != "rigid_body_6dof_surface_allocated":
        raise ValueError("HL-20 source-surface authority screen requires surface-allocated rigid-body fidelity")
    if composition.initialization.id != _INITIALIZATION_ID:
        raise ValueError(f"HL-20 source-surface authority screen requires initialization {_INITIALIZATION_ID!r}")
    if composition.initialization.inputs:
        raise ValueError("HL-20 source-surface authority screen does not accept initialization overrides")
    if len(composition.segments) != 1 or composition.segments[0].id != _SEGMENT_ID:
        raise ValueError(f"HL-20 source-surface authority screen requires exactly one {_SEGMENT_ID!r} segment")
    if composition.segments[0].inputs:
        raise ValueError("HL-20 source-surface authority screen does not accept segment overrides")
    return HL20SurfaceAuthorityScreenPlan(
        composition.family_id,
        composition.mission,
        composition.fidelity,
        composition.initialization.id,
        composition.segments[0].instance_id,
    )
    ####


class HL20SourceSurfacePitchAuthorityScreenCapabilityAdapter:
    """Advertise the exact source-backed seven-surface probe before execution."""

    id = _ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "hl20_mod_k"
            and composition.mission == _MISSION_ID
            and composition.fidelity == "rigid_body_6dof_surface_allocated"
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        plan = compile_hl20_source_surface_authority_screen(composition)
        plant = HL20SourceSurfacePlant()
        state = _fixture_state()
        effectiveness = plant.effectiveness(state, _zero_effectors())
        trim = _pitch_trim_fragment()
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": "source_surface_pitch_authority_allocation",
            "participating_nonlinear_source_load_evaluation": True,
            "physical_effector_allocation": True,
            "effector_names": list(HL20_SURFACE_NAMES),
            "effector_limits_deg": {name: list(HL20_SOURCE_SURFACE_BOUNDS_DEG[name]) for name in HL20_SURFACE_NAMES},
            "controlled_wrench_axes": ["moment_y_nm"],
            "uncontrolled_wrench_axes": [
                "force_x_n",
                "force_y_n",
                "force_z_n",
                "moment_x_nm",
                "moment_z_nm",
            ],
            "source_effectiveness_rank": int(np.linalg.matrix_rank(effectiveness.array)),
            "source_pitch_trim_fragment": trim,
            "full_state_trim": plan.manifest()["full_state_trim"],
            "controller": {
                "status": "not_available_without_full_state_trim",
                "claim_boundary": "This screen proves source-surface authority only; it does not select, tune, or execute a feedback controller.",
            },
            "navigation_guidance": False,
            "allocation_scope": "source_pitch_authority_only",
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                "pinned DAVE-ML Mach-1 pitch-coefficient trim fragment, seven bounded source surfaces, and a nonlinear pitch-authority allocation screen are available",
                "full-state trim, feedback-controller synthesis, state propagation, guidance, and glide arrival remain unavailable from the retained source binding",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####

    ####


def preflight_hl20_source_surface_authority_screen(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Prove exact semantic lowering and the source evidence needed by this probe."""

    plan = compile_hl20_source_surface_authority_screen(composition)
    estimate = estimate_mission_capability(composition)
    if estimate is None or estimate.adapter_id != _ADAPTER_ID:
        raise ValueError("HL-20 source-surface authority screen has no matching capability adapter")
    capability = estimate.manifest.get("capability")
    if not isinstance(capability, Mapping):
        raise ValueError("HL-20 source-surface authority screen is missing its capability record")
    trim = capability.get("source_pitch_trim_fragment")
    trim_ready = isinstance(trim, Mapping) and trim.get("status") == "verified"
    rank = capability.get("source_effectiveness_rank")
    rank_ready = isinstance(rank, int) and rank >= 1
    controls = capability.get("effector_names")
    controls_ready = controls == list(HL20_SURFACE_NAMES)
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status="translation_ready" if trim_ready and rank_ready and controls_ready else "blocked",
        translator_id=_ADAPTER_ID,
        checks=(
            ExecutionPreflightCheck(
                "hl20.semantic_source_surface_pitch_authority_screen",
                [_INITIALIZATION_ID, _SEGMENT_ID],
                [plan.initialization_id, plan.segment_instance_id],
                None,
                True,
            ),
            ExecutionPreflightCheck("hl20.source_pitch_trim_fragment", "verified", None if not isinstance(trim, Mapping) else trim.get("status"), None, trim_ready),
            ExecutionPreflightCheck("hl20.source_surface_effectiveness_rank", ">= 1", rank, None, rank_ready),
            ExecutionPreflightCheck("hl20.source_surface_set", list(HL20_SURFACE_NAMES), controls, None, controls_ready),
        ),
        diagnostics=(
            "composition lowers exactly to the frozen DAVE-ML pitch-authority allocation screen; it is neither a full 6-DOF trim nor a glide controller",
            *estimate.diagnostics,
        ),
        derived_mission=estimate.manifest,
        capability_estimate=build_concrete_capability_preflight_evidence(composition, estimate),
    )
    ####


def execute_hl20_source_surface_authority_screen(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None = None,
) -> HL20SurfaceAuthorityScreenExecution:
    """Allocate a pitch-moment increment through actual bounded source surfaces."""

    if max_steps is not None:
        raise ValueError("--max-steps is unavailable for the fixed-cadence HL-20 source-surface authority screen")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        raise ValueError("cannot execute HL-20 source-surface authority screen: semantic preflight is not translation_ready")
    plan = compile_hl20_source_surface_authority_screen(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    plant = HL20SourceSurfacePlant()
    state = _fixture_state()
    effectiveness = plant.effectiveness(state, _zero_effectors())
    baseline_wrench = dict(effectiveness.reference_wrench)
    desired_wrench = dict(baseline_wrench)
    desired_wrench["moment_y_nm"] = baseline_wrench["moment_y_nm"] + _REQUESTED_PITCH_INCREMENT_NM
    actual = _zero_effectors()
    allocation_status: str
    rows: list[dict[str, float | int | str]] = []
    for index in range(_STEP_COUNT + 1):
        if index:
            step = allocate_and_advance_wrench(
                effectiveness,
                plant.effector_limits,
                desired_wrench,
                actual,
                _DT_S,
                preferred_effectors=_zero_effectors(),
                wrench_weights={name: 1.0 if name == "moment_y_nm" else 0.0 for name in effectiveness.wrench_names},
                regularization=1.0e-8,
            )
            actual = {name: float(step.actuator.actual_positions[name]) for name in HL20_SURFACE_NAMES}
            allocation_status = step.allocation.status
            allocation_residual = step.achieved_controlled_residual_norm
            saturation_count = len(set(step.allocation.position_saturated) | set(step.allocation.rate_limited) | set(step.actuator.position_saturated) | set(step.actuator.rate_limited))
        else:
            allocation_status = "reference"
            allocation_residual = 0.0
            saturation_count = 0
        loads = plant.source.evaluate(
            (state["u_m_s"], state["v_m_s"], state["w_m_s"]),
            state["altitude_m"],
            (state["p_rad_s"], state["q_rad_s"], state["r_rad_s"]),
            controls=actual,
        )
        source_pitch_moment = float(loads.moment_body_nm[1])
        row: dict[str, float | int | str] = {
            "time_s": index * _DT_S,
            "u_m_s": state["u_m_s"],
            "v_m_s": state["v_m_s"],
            "w_m_s": state["w_m_s"],
            "p_rad_s": state["p_rad_s"],
            "q_rad_s": state["q_rad_s"],
            "r_rad_s": state["r_rad_s"],
            "source_pitch_coefficient": float(dict(loads.coefficients)["cm"]),
            "reference_pitch_moment_nm": baseline_wrench["moment_y_nm"],
            "requested_pitch_moment_nm": desired_wrench["moment_y_nm"],
            "achieved_pitch_moment_nm": source_pitch_moment,
            "nonlinear_pitch_increment_nm": source_pitch_moment - baseline_wrench["moment_y_nm"],
            "pitch_moment_residual_nm": desired_wrench["moment_y_nm"] - source_pitch_moment,
            "allocation_controlled_residual_norm": allocation_residual,
            "allocation_status": allocation_status,
            "saturation_count": saturation_count,
            "source_effectiveness_rank": int(np.linalg.matrix_rank(effectiveness.array)),
            "mass_kg": plant.mass_kg,
            **{f"surface_{name}_deg": actual[name] for name in HL20_SURFACE_NAMES},
        }
        rows.append(row)
    assessment = _assess(rows)
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.lifting_body.daveml.v1",
        "control_realization": "source_surface_pitch_authority_allocation",
        "physical_effector_allocation": True,
        "effector_names": list(HL20_SURFACE_NAMES),
        "controlled_wrench_axes": ["moment_y_nm"],
        "uncontrolled_wrench_axes": [name for name in effectiveness.wrench_names if name != "moment_y_nm"],
        "source_effectiveness_rank": int(np.linalg.matrix_rank(effectiveness.array)),
        "source_pitch_trim_fragment": _pitch_trim_fragment(),
        "full_state_trim": plan.manifest()["full_state_trim"],
        "dt_s": _DT_S,
        "duration_s": _DURATION_S,
        "mass_kg": plant.mass_kg,
        "numerical_valid": assessment["numerical_valid"],
        "hard_gates_passed": assessment["screen_pass"],
        "mission_graph_execution": unobserved_mission_graph_execution(
            composition,
            "The frozen HL-20 source-surface authority screen declares no route or transition execution.",
        ).as_dict(),
    }
    evaluation = _evaluation(assessment)
    status_trace = build_committed_status_trace(composition, _status_samples(rows))
    control_trace = build_committed_control_trace(composition, _control_samples(rows))
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    runtime["resource_ledger"] = resource_ledger_summary(resource_ledger)
    result = HL20SurfaceAuthorityScreenExecution(
        composition,
        preflight,
        plan,
        destination,
        runtime,
        evaluation,
        status_trace,
        control_trace,
        "This screen proves only a pinned nonlinear DAVE-ML source pitch response after bounded allocation to all seven named source surfaces. It does not establish full-state trim, feedback control, state propagation, navigation, glide guidance, arrival behavior, or flight qualification.",
    )
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "mission_graph_execution.json", runtime["mission_graph_execution"])
    _write_json(destination / "surface_authority.json", {"assessment": assessment, "rows": rows})
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
            envelope={"pass": assessment["numerical_valid"]},
            claim_boundary=result.claim_boundary,
            status_trace=status_trace,
            control_trace=control_trace,
        ).as_dict(),
    )
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _fixture_state() -> dict[str, float]:
    alpha = math.radians(_TRIM_ALPHA_DEG)
    speed = HL20_SOURCE_SPEED_OF_SOUND_M_S
    return {
        "u_m_s": speed * math.cos(alpha),
        "v_m_s": 0.0,
        "w_m_s": -speed * math.sin(alpha),
        "p_rad_s": 0.0,
        "q_rad_s": 0.0,
        "r_rad_s": 0.0,
        "altitude_m": 0.0,
    }
    ####


def _zero_effectors() -> dict[str, float]:
    return {name: 0.0 for name in HL20_SURFACE_NAMES}
    ####


def _pitch_trim_fragment() -> dict[str, object]:
    evidence = json.loads(HL20_TRIM_EVIDENCE.read_text(encoding="utf-8"))
    trim = evidence.get("trim")
    if not isinstance(trim, Mapping):
        raise ValueError("HL-20 pitch trim evidence has no trim record")
    return {
        "status": "verified" if evidence.get("status") == "verified" and trim.get("success") is True else "failed",
        "state": dict(trim.get("state", {})),
        "controls": dict(trim.get("controls", {})),
        "residuals": dict(trim.get("residuals", {})),
        "claim_boundary": evidence.get("claim_boundary"),
        "evidence_artifact": "verification/daveml_hl20_trim_evidence.json",
    }
    ####


def _assess(rows: list[dict[str, float | int | str]]) -> dict[str, object]:
    final = rows[-1]
    statuses = {str(row["allocation_status"]) for row in rows[1:]}
    nonlinear_increment = float(final["nonlinear_pitch_increment_nm"])
    finite = all(
        math.isfinite(float(value))
        for row in rows
        for value in row.values()
        if isinstance(value, int | float) and not isinstance(value, bool)
    )
    checks = {
        "scalar_pitch_trim_fragment_verified": _pitch_trim_fragment()["status"] == "verified",
        "seven_source_surfaces_available_to_allocator": all(
            all(
                HL20_SOURCE_SURFACE_BOUNDS_DEG[name][0] <= float(row[f"surface_{name}_deg"]) <= HL20_SOURCE_SURFACE_BOUNDS_DEG[name][1]
                for row in rows
            )
            for name in HL20_SURFACE_NAMES
        ),
        "nonlinear_source_pitch_increment": abs(nonlinear_increment - _REQUESTED_PITCH_INCREMENT_NM) <= 0.10 * _REQUESTED_PITCH_INCREMENT_NM,
        "controlled_allocation_status": not bool(statuses & {"infeasible", "numerically_singular", "solver_failure"}),
        "finite_committed_source_truth": finite,
    }
    return {
        "screen_pass": all(checks.values()),
        "checks": checks,
        "allocation_statuses": sorted(statuses),
        "final_nonlinear_pitch_increment_nm": nonlinear_increment,
        "requested_pitch_increment_nm": _REQUESTED_PITCH_INCREMENT_NM,
        "final_pitch_increment_error_fraction": abs(nonlinear_increment - _REQUESTED_PITCH_INCREMENT_NM) / _REQUESTED_PITCH_INCREMENT_NM,
        "numerical_valid": finite,
    }
    ####


def _evaluation(assessment: Mapping[str, object]) -> dict[str, object]:
    checks = assessment["checks"]
    if not isinstance(checks, Mapping):
        raise ValueError("HL-20 authority assessment did not retain checks")
    return {
        "schema": "taoryx.hl20-source-surface-authority-screen-evaluation/v1alpha1",
        "kind": "source_surface_pitch_authority_screen",
        "mission_pass": assessment["screen_pass"],
        "results": [
            {"id": name, "status": "pass" if value is True else "fail", "required": True}
            for name, value in checks.items()
        ],
        "metrics": {name: value for name, value in assessment.items() if name not in {"screen_pass", "checks"}},
        "control_realization": "source_surface_pitch_authority_allocation",
        "claim_boundary": "This is a fixed-fixture source authority screen, not a closed-loop or flight-mission evaluation.",
    }
    ####


def _status_samples(rows: list[dict[str, float | int | str]]) -> tuple[BatchTruthSample, ...]:
    samples: list[BatchTruthSample] = []
    for index, row in enumerate(rows):
        raw: dict[str, object] = dict(row)
        raw.update(
            {
                "local_attitude_rad": [0.0, 0.0, 0.0],
                "body_velocity_m_s": [float(row[name]) for name in ("u_m_s", "v_m_s", "w_m_s")],
                "body_rate_rad_s": [float(row[name]) for name in ("p_rad_s", "q_rad_s", "r_rad_s")],
                "requested_pitch_moment_nm": float(row["requested_pitch_moment_nm"]),
                "achieved_pitch_moment_nm": float(row["achieved_pitch_moment_nm"]),
                "pitch_moment_residual_nm": float(row["pitch_moment_residual_nm"]),
                "wrench_status": str(row["allocation_status"]),
                "wrench_saturated": int(row["saturation_count"]) > 0,
                "physical_effector_allocation": True,
                "control_realization": "source_surface_pitch_authority_allocation",
                "controller_method": "not_applicable",
                "full_state_trim_status": "not_available",
            }
        )
        samples.append(
            BatchTruthSample(
                time_s=float(row["time_s"]),
                raw_values=raw,
                execution_status="completed" if index == len(rows) - 1 else "active",
            )
        )
    return tuple(samples)
    ####


def _control_samples(rows: list[dict[str, float | int | str]]) -> tuple[BatchControlSample, ...]:
    samples: list[BatchControlSample] = []
    previous = 0.0
    for row in rows:
        time_s = float(row["time_s"])
        samples.append(
            BatchControlSample(
                previous if time_s else 0.0,
                time_s,
                {},
                {f"effector.surface.{name}.position": float(row[f"surface_{name}_deg"]) for name in HL20_SURFACE_NAMES},
            )
        )
        previous = time_s
    return tuple(samples)
    ####


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    if not rows:
        raise ValueError("HL-20 source-surface authority screen produced no telemetry rows")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    ####


__all__ = [
    "HL20SourceSurfacePitchAuthorityScreenCapabilityAdapter",
    "HL20SurfaceAuthorityScreenExecution",
    "HL20SurfaceAuthorityScreenPlan",
    "compile_hl20_source_surface_authority_screen",
    "execute_hl20_source_surface_authority_screen",
    "preflight_hl20_source_surface_authority_screen",
]
