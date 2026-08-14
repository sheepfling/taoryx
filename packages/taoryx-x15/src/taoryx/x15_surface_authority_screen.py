"""Public Composition endpoint for the X-15 source-table surface authority.

The retained X-15 six-axis deck supplies three bounded aerodynamic control
coordinates: symmetric stabilator, differential stabilator, and rudder.  The
deck does not by itself close the full vehicle trim, propulsion, reaction
control, navigation, or high-energy mission problem.  This endpoint exposes
the useful part honestly: a frozen source-release fixture, bounded allocation
through all three source controls, and nonlinear source-table load evaluation
at every committed sample.
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
from .control_allocation import EffectorEffectiveness, EffectorLimits, allocate_and_advance_wrench
from .mission_capability import MissionCapabilityEstimate, estimate_mission_capability
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    VehicleExecutionPreflight,
    build_concrete_capability_preflight_evidence,
    preflight_vehicle_composition,
)
from .x15_adapter import (
    X15_MASS_KG,
    X15_SOURCE_SURFACE_BOUNDS_DEG,
    X15_SOURCE_SURFACE_NAMES,
    build_x15_source_direct_wrench_plant,
)

_INITIALIZATION_ID = "source_release_glide_surface_authority_anchor"
_MISSION_ID = "x15_source_surface_authority_screen_v1"
_SEGMENT_ID = "source_surface_three_axis_authority_allocation_screen"
_ADAPTER_ID = "taoryx.x15_source_surface_authority_screen.capability.v1"
_DT_S = 0.05
_DURATION_S = 0.20
_STEP_COUNT = int(_DURATION_S / _DT_S)
_WRENCH_NAMES = ("moment_x_nm", "moment_y_nm", "moment_z_nm")
_REQUESTED_INCREMENT_NM = {
    "moment_x_nm": 4_000.0,
    "moment_y_nm": -100_000.0,
    "moment_z_nm": 40_000.0,
}


@dataclass(frozen=True, slots=True)
class X15SurfaceAuthorityScreenPlan:
    """The fixed semantic lowering record for the source table probe."""

    family_id: str
    mission_id: str
    fidelity: str
    initialization_id: str
    segment_instance_id: str

    def manifest(self) -> dict[str, object]:
        """Return exact scope and the physical-control nonclaims."""

        return {
            "schema": "taoryx.x15-source-surface-authority-screen-plan/v1alpha1",
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
                "state": "frozen_source_release_glide_local_state",
                "state_propagation": "not_performed",
            },
            "duration_s": _DURATION_S,
            "dt_s": _DT_S,
            "requested_moment_increment_nm": dict(_REQUESTED_INCREMENT_NM),
            "control_realization": "source_surface_three_axis_authority_allocation",
            "full_state_trim": {
                "status": "not_available",
                "reason": "the source fragment is exercised at a frozen release fixture, not a solved full vehicle equilibrium",
            },
            "claim_boundary": (
                "This is a frozen source-release, three-surface authority screen. It does not propagate the state, "
                "solve full X-15 trim, allocate propulsion or reaction controls, synthesize feedback, navigate, "
                "guide a mission, or qualify flight."
            ),
        }
        ####

    ####


@dataclass(frozen=True, slots=True)
class X15SurfaceAuthorityScreenExecution:
    """The evidence packet returned by the narrow source-table probe."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: X15SurfaceAuthorityScreenPlan
    output_dir: Path
    runtime: dict[str, object]
    evaluation: dict[str, object]
    status_trace: dict[str, object]
    control_trace: dict[str, object]
    claim_boundary: str

    @property
    def screen_pass(self) -> bool:
        """Return whether every declared authority check passed."""

        return self.evaluation["mission_pass"] is True
        ####

    def as_dict(self) -> dict[str, object]:
        """Serialize the public execution result."""

        return {
            "schema": "taoryx.x15-source-surface-authority-screen-execution/v1alpha1",
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


def compile_x15_source_surface_authority_screen(
    composition: CompiledVehicleComposition,
) -> X15SurfaceAuthorityScreenPlan:
    """Fail closed unless Composition selected the exact source-surface probe."""

    if composition.family_id != "x15":
        raise ValueError("X-15 source-surface authority screen requires the x15 family")
    if composition.mission != _MISSION_ID:
        raise ValueError(f"X-15 source-surface authority screen requires mission {_MISSION_ID!r}")
    if composition.fidelity != "rigid_body_6dof_surface_allocated":
        raise ValueError("X-15 source-surface authority screen requires surface-allocated rigid-body fidelity")
    if composition.initialization.id != _INITIALIZATION_ID:
        raise ValueError(f"X-15 source-surface authority screen requires initialization {_INITIALIZATION_ID!r}")
    if composition.initialization.inputs:
        raise ValueError("X-15 source-surface authority screen does not accept initialization overrides")
    if len(composition.segments) != 1 or composition.segments[0].id != _SEGMENT_ID:
        raise ValueError(f"X-15 source-surface authority screen requires exactly one {_SEGMENT_ID!r} segment")
    if composition.segments[0].inputs:
        raise ValueError("X-15 source-surface authority screen does not accept segment overrides")
    return X15SurfaceAuthorityScreenPlan(
        composition.family_id,
        composition.mission,
        composition.fidelity,
        composition.initialization.id,
        composition.segments[0].instance_id,
    )
    ####


class X15SourceSurfaceAuthorityScreenCapabilityAdapter:
    """Advertise the exact bounded X-15 source-surface probe before execution."""

    id = _ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Report whether this adapter owns the selected exact composition."""

        return (
            composition.family_id == "x15"
            and composition.mission == _MISSION_ID
            and composition.fidelity == "rigid_body_6dof_surface_allocated"
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Return an explicit, non-flight capability advertisement."""

        plan = compile_x15_source_surface_authority_screen(composition)
        plant = build_x15_source_direct_wrench_plant()
        state = dict(plant.reference_state)
        effectiveness, baseline = _effectiveness(plant, state)
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": "source_surface_three_axis_authority_allocation",
            "participating_nonlinear_source_load_evaluation": True,
            "physical_effector_allocation": True,
            "effector_names": list(X15_SOURCE_SURFACE_NAMES),
            "effector_limits_deg": {name: list(X15_SOURCE_SURFACE_BOUNDS_DEG[name]) for name in X15_SOURCE_SURFACE_NAMES},
            "controlled_wrench_axes": list(_WRENCH_NAMES),
            "uncontrolled_wrench_axes": ["force_x_n", "force_y_n", "force_z_n"],
            "source_effectiveness_rank": int(np.linalg.matrix_rank(effectiveness.array)),
            "source_fixture": {
                "mach": baseline["mach"],
                "alpha_deg": baseline["alpha_deg"],
                "beta_deg": baseline["beta_deg"],
                "source_loads": {name: baseline[name] for name in (*_WRENCH_NAMES, "force_x_n", "force_y_n", "force_z_n")},
            },
            "full_state_trim": plan.manifest()["full_state_trim"],
            "controller": {
                "status": "not_available_without_full_state_trim",
                "claim_boundary": "This screen proves X-15 source-surface authority only; it does not select, tune, or execute a feedback controller.",
            },
            "navigation_guidance": False,
            "allocation_scope": "frozen_source_three_axis_moment_authority_only",
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                "the frozen X-15 source release fixture exposes three bounded table-bound aerodynamic surfaces with full local moment rank",
                "full-state trim, feedback-controller synthesis, state propagation, propulsion/RCS allocation, guidance, and flight qualification remain outside this retained source binding",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####

    ####


def preflight_x15_source_surface_authority_screen(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Prove exact lowering and the source controls needed by this probe."""

    plan = compile_x15_source_surface_authority_screen(composition)
    estimate = estimate_mission_capability(composition)
    if estimate is None or estimate.adapter_id != _ADAPTER_ID:
        raise ValueError("X-15 source-surface authority screen has no matching capability adapter")
    capability = estimate.manifest.get("capability")
    if not isinstance(capability, Mapping):
        raise ValueError("X-15 source-surface authority screen is missing its capability record")
    rank = capability.get("source_effectiveness_rank")
    rank_ready = isinstance(rank, int) and rank == 3
    controls = capability.get("effector_names")
    controls_ready = controls == list(X15_SOURCE_SURFACE_NAMES)
    fixture = capability.get("source_fixture")
    fixture_ready = isinstance(fixture, Mapping) and all(
        isinstance(fixture.get(name), (int, float)) and math.isfinite(float(fixture[name]))
        for name in ("mach", "alpha_deg", "beta_deg")
    )
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status="translation_ready" if rank_ready and controls_ready and fixture_ready else "blocked",
        translator_id=_ADAPTER_ID,
        checks=(
            ExecutionPreflightCheck(
                "x15.semantic_source_surface_authority_screen",
                [_INITIALIZATION_ID, _SEGMENT_ID],
                [plan.initialization_id, plan.segment_instance_id],
                None,
                True,
            ),
            ExecutionPreflightCheck("x15.source_surface_effectiveness_rank", 3, rank, None, rank_ready),
            ExecutionPreflightCheck("x15.source_surface_set", list(X15_SOURCE_SURFACE_NAMES), controls, None, controls_ready),
            ExecutionPreflightCheck("x15.source_release_fixture", "finite Mach/alpha/beta", fixture, None, fixture_ready),
        ),
        diagnostics=(
            "composition lowers exactly to the frozen X-15 source-surface allocation screen; it is neither full vehicle trim nor a flight controller",
            *estimate.diagnostics,
        ),
        derived_mission=estimate.manifest,
        capability_estimate=build_concrete_capability_preflight_evidence(composition, estimate),
    )
    ####


def execute_x15_source_surface_authority_screen(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    max_steps: int | None = None,
) -> X15SurfaceAuthorityScreenExecution:
    """Allocate a three-axis moment increment through actual X-15 source surfaces."""

    if max_steps is not None:
        raise ValueError("--max-steps is unavailable for the fixed-cadence X-15 source-surface authority screen")
    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        raise ValueError("cannot execute X-15 source-surface authority screen: semantic preflight is not translation_ready")
    plan = compile_x15_source_surface_authority_screen(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    plant = build_x15_source_direct_wrench_plant()
    state = dict(plant.reference_state)
    effectiveness, baseline = _effectiveness(plant, state)
    desired = {name: baseline[name] + _REQUESTED_INCREMENT_NM[name] for name in _WRENCH_NAMES}
    limits = {
        name: EffectorLimits(name, *X15_SOURCE_SURFACE_BOUNDS_DEG[name], "deg")
        for name in X15_SOURCE_SURFACE_NAMES
    }
    actual = _zero_effectors()
    allocation_status: str
    rows: list[dict[str, float | int | str]] = []
    for index in range(_STEP_COUNT + 1):
        if index:
            step = allocate_and_advance_wrench(
                effectiveness,
                limits,
                desired,
                actual,
                _DT_S,
                preferred_effectors=_zero_effectors(),
                regularization=1.0e-8,
            )
            actual = {name: float(step.actuator.actual_positions[name]) for name in X15_SOURCE_SURFACE_NAMES}
            allocation_status = step.allocation.status
            allocation_residual = step.achieved_controlled_residual_norm
            saturation_count = len(
                set(step.allocation.position_saturated)
                | set(step.allocation.rate_limited)
                | set(step.actuator.position_saturated)
                | set(step.actuator.rate_limited)
            )
        else:
            allocation_status = "reference"
            allocation_residual = 0.0
            saturation_count = 0
        _, loads = plant.source_surface_loads(state, actual)
        row: dict[str, float | int | str] = {
            "time_s": index * _DT_S,
            **{name: state[name] for name in state},
            "source_mach": loads["mach"],
            "source_alpha_deg": loads["alpha_deg"],
            "source_beta_deg": loads["beta_deg"],
            **{f"reference_{name}": baseline[name] for name in _WRENCH_NAMES},
            **{f"requested_{name}": desired[name] for name in _WRENCH_NAMES},
            **{f"achieved_{name}": loads[name] for name in _WRENCH_NAMES},
            **{f"nonlinear_increment_{name}": loads[name] - baseline[name] for name in _WRENCH_NAMES},
            **{f"residual_{name}": desired[name] - loads[name] for name in _WRENCH_NAMES},
            "allocation_controlled_residual_norm": allocation_residual,
            "allocation_status": allocation_status,
            "saturation_count": saturation_count,
            "source_effectiveness_rank": int(np.linalg.matrix_rank(effectiveness.array)),
            "mass_kg": X15_MASS_KG,
            **{f"surface_{name}_deg": actual[name] for name in X15_SOURCE_SURFACE_NAMES},
        }
        rows.append(row)
    assessment = _assess(rows)
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.high_energy.fixed_wing.v1",
        "control_realization": "source_surface_three_axis_authority_allocation",
        "physical_effector_allocation": True,
        "effector_names": list(X15_SOURCE_SURFACE_NAMES),
        "controlled_wrench_axes": list(_WRENCH_NAMES),
        "uncontrolled_wrench_axes": ["force_x_n", "force_y_n", "force_z_n"],
        "source_effectiveness_rank": int(np.linalg.matrix_rank(effectiveness.array)),
        "source_fixture": {name: baseline[name] for name in ("mach", "alpha_deg", "beta_deg")},
        "full_state_trim": plan.manifest()["full_state_trim"],
        "dt_s": _DT_S,
        "duration_s": _DURATION_S,
        "mass_kg": X15_MASS_KG,
        "numerical_valid": assessment["numerical_valid"],
        "hard_gates_passed": assessment["screen_pass"],
        "mission_graph_execution": unobserved_mission_graph_execution(
            composition,
            "The frozen X-15 source-surface authority screen declares no route or transition execution.",
        ).as_dict(),
    }
    evaluation = _evaluation(assessment)
    status_trace = build_committed_status_trace(composition, _status_samples(rows))
    control_trace = build_committed_control_trace(composition, _control_samples(rows))
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    runtime["resource_ledger"] = resource_ledger_summary(resource_ledger)
    result = X15SurfaceAuthorityScreenExecution(
        composition,
        preflight,
        plan,
        destination,
        runtime,
        evaluation,
        status_trace,
        control_trace,
        "This screen proves only a frozen nonlinear X-15 source-table response after bounded allocation to the symmetric stabilator, differential stabilator, and rudder. It does not establish full-state trim, feedback control, state propagation, propulsion/RCS allocation, navigation, a high-energy mission, or flight qualification.",
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


def _zero_effectors() -> dict[str, float]:
    """Return the declared neutral source-surface position vector."""

    return {name: 0.0 for name in X15_SOURCE_SURFACE_NAMES}
    ####


def _effectiveness(plant: object, state: Mapping[str, float]) -> tuple[EffectorEffectiveness, dict[str, float]]:
    """Differentiate the source table path in its declared degree coordinates."""

    source_plant = plant
    if not hasattr(source_plant, "source_surface_loads"):
        raise ValueError("X-15 source plant does not expose source-surface table evaluation")
    _, baseline = source_plant.source_surface_loads(state, _zero_effectors())
    columns: list[tuple[float, ...]] = []
    for name in X15_SOURCE_SURFACE_NAMES:
        plus = _zero_effectors()
        minus = _zero_effectors()
        plus[name] = 1.0
        minus[name] = -1.0
        _, plus_loads = source_plant.source_surface_loads(state, plus)
        _, minus_loads = source_plant.source_surface_loads(state, minus)
        columns.append(tuple((plus_loads[axis] - minus_loads[axis]) / 2.0 for axis in _WRENCH_NAMES))
    return (
        EffectorEffectiveness(
            wrench_names=_WRENCH_NAMES,
            effector_names=X15_SOURCE_SURFACE_NAMES,
            matrix=tuple(tuple(columns[column][row] for column in range(len(columns))) for row in range(len(_WRENCH_NAMES))),
            reference_wrench={name: baseline[name] for name in _WRENCH_NAMES},
            reference_effectors=_zero_effectors(),
            source="centered-x15-source-table-difference-per-degree",
        ),
        baseline,
    )
    ####


def _assess(rows: list[dict[str, float | int | str]]) -> dict[str, object]:
    """Apply the narrow hard gates for a source-surface authority result."""

    final = rows[-1]
    statuses = {str(row["allocation_status"]) for row in rows[1:]}
    requested_norm = math.sqrt(sum(value * value for value in _REQUESTED_INCREMENT_NM.values()))
    residual_norm = math.sqrt(sum(float(final[f"residual_{name}"]) ** 2 for name in _WRENCH_NAMES))
    finite = all(
        math.isfinite(float(value))
        for row in rows
        for value in row.values()
        if isinstance(value, int | float) and not isinstance(value, bool)
    )
    checks = {
        "three_source_surfaces_available_to_allocator": all(
            all(
                X15_SOURCE_SURFACE_BOUNDS_DEG[name][0] <= float(row[f"surface_{name}_deg"]) <= X15_SOURCE_SURFACE_BOUNDS_DEG[name][1]
                for row in rows
            )
            for name in X15_SOURCE_SURFACE_NAMES
        ),
        "three_axis_source_effectiveness": int(final["source_effectiveness_rank"]) == 3,
        "nonlinear_source_moment_increment": residual_norm <= 0.02 * requested_norm,
        "controlled_allocation_status": not bool(statuses & {"infeasible", "numerically_singular", "solver_failure"}),
        "finite_committed_source_truth": finite,
    }
    return {
        "screen_pass": all(checks.values()),
        "checks": checks,
        "allocation_statuses": sorted(statuses),
        "final_nonlinear_moment_residual_norm_nm": residual_norm,
        "requested_moment_increment_norm_nm": requested_norm,
        "final_moment_increment_error_fraction": residual_norm / requested_norm,
        "numerical_valid": finite,
    }
    ####


def _evaluation(assessment: Mapping[str, object]) -> dict[str, object]:
    """Translate narrow screen gates into the common evaluation shape."""

    checks = assessment["checks"]
    if not isinstance(checks, Mapping):
        raise ValueError("X-15 authority assessment did not retain checks")
    return {
        "schema": "taoryx.x15-source-surface-authority-screen-evaluation/v1alpha1",
        "kind": "source_surface_three_axis_authority_screen",
        "mission_pass": assessment["screen_pass"],
        "results": [
            {"id": name, "status": "pass" if value is True else "fail", "required": True}
            for name, value in checks.items()
        ],
        "metrics": {name: value for name, value in assessment.items() if name not in {"screen_pass", "checks"}},
        "control_realization": "source_surface_three_axis_authority_allocation",
        "claim_boundary": "This is a fixed-fixture source authority screen, not a closed-loop or X-15 flight-mission evaluation.",
    }
    ####


def _status_samples(rows: list[dict[str, float | int | str]]) -> tuple[BatchTruthSample, ...]:
    """Build committed source truth samples for the common status contract."""

    samples: list[BatchTruthSample] = []
    for index, row in enumerate(rows):
        raw: dict[str, object] = dict(row)
        raw.update(
            {
                "body_velocity_m_s": [float(row[name]) for name in ("u_m_s", "v_m_s", "w_m_s")],
                "body_rate_rad_s": [float(row[name]) for name in ("p_rad_s", "q_rad_s", "r_rad_s")],
                "requested_moment_body_nm": [float(row[f"requested_{name}"]) for name in _WRENCH_NAMES],
                "achieved_moment_body_nm": [float(row[f"achieved_{name}"]) for name in _WRENCH_NAMES],
                "residual_moment_body_nm": [float(row[f"residual_{name}"]) for name in _WRENCH_NAMES],
                "wrench_status": str(row["allocation_status"]),
                "wrench_saturated": int(row["saturation_count"]) > 0,
                "physical_effector_allocation": True,
                "control_realization": "source_surface_three_axis_authority_allocation",
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
    """Expose actual named surface positions through the semantic action trace."""

    samples: list[BatchControlSample] = []
    previous = 0.0
    for row in rows:
        time_s = float(row["time_s"])
        samples.append(
            BatchControlSample(
                previous if time_s else 0.0,
                time_s,
                {},
                {f"effector.surface.{name}.position": float(row[f"surface_{name}_deg"]) for name in X15_SOURCE_SURFACE_NAMES},
            )
        )
        previous = time_s
    return tuple(samples)
    ####


def _write_json(path: Path, payload: object) -> None:
    """Write one deterministic JSON evidence artifact."""

    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    """Write the committed truth rows as a CSV artifact."""

    if not rows:
        raise ValueError("X-15 source-surface authority screen produced no telemetry rows")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    ####


__all__ = [
    "X15SourceSurfaceAuthorityScreenCapabilityAdapter",
    "X15SurfaceAuthorityScreenExecution",
    "X15SurfaceAuthorityScreenPlan",
    "compile_x15_source_surface_authority_screen",
    "execute_x15_source_surface_authority_screen",
    "preflight_x15_source_surface_authority_screen",
]
