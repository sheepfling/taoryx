"""Public batch execution for source-local direct-wrench controller screens.

The local screen is intentionally narrower than a trajectory mission.  It is
still an important composition endpoint: it binds the selected source plant,
derives the LQR from that plant, executes its bounded requested-to-achieved
wrench loop, and writes the standard preflight, execution, and status-trace
artifacts.  It never creates missing navigation or physical-effector physics.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from .composition_control_trace import BatchControlSample, build_committed_control_trace, control_trace_summary
from .composition_graph_evidence import unobserved_mission_graph_execution
from .composition_resource_ledger import build_committed_resource_ledger
from .composition_sensor_trace import BatchTruthSample
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .local_direct_wrench import LocalDirectWrenchScreenExecution, run_local_direct_wrench_screen
from .local_direct_wrench_mission_translation import (
    LocalDirectWrenchScreenMissionPlan,
    compile_local_direct_wrench_screen_mission,
)
from .local_direct_wrench_screen_registry import resolve_local_direct_wrench_screen_definition
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import VehicleExecutionPreflight, preflight_vehicle_composition


@dataclass(frozen=True, slots=True)
class LocalDirectWrenchCompositionExecution:
    """One public local source-load controller-screen execution result."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    plan: LocalDirectWrenchScreenMissionPlan
    output_dir: Path
    screen: LocalDirectWrenchScreenExecution
    mission_graph_execution: dict[str, object]
    status_trace: dict[str, object]
    semantic_action_trace: dict[str, object]
    claim_boundary: str

    @property
    def screen_pass(self) -> bool:
        """Return the narrow local-screen verdict, not a mission outcome."""

        return self.screen.mission_pass
        ####

    ####

    def as_dict(self) -> dict[str, object]:
        """Return the common public execution record without relabeling scope."""

        return {
            "schema": "taoryx.local-direct-wrench-composition-execution/v1alpha1",
            "status": "development_local_screen_pass" if self.screen_pass else "development_local_screen_failed",
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "plan": self.plan.manifest(),
            "output_dir": str(self.output_dir),
            "control_screen": {
                "screen_pass": self.screen_pass,
                "initial_error_norm": self.screen.initial_error_norm,
                "final_error_norm": self.screen.final_error_norm,
                "observed_wrench_statuses": list(self.screen.observed_statuses),
                "control_realization": "direct_wrench_screen",
                "physical_effector_allocation": False,
            },
            "mission_graph_execution": self.mission_graph_execution,
            "status_trace": status_trace_summary(self.status_trace),
            "semantic_action_trace": control_trace_summary(self.semantic_action_trace),
            "claim_boundary": self.claim_boundary,
        }
        ####

    ####


def execute_local_direct_wrench_composition(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
) -> LocalDirectWrenchCompositionExecution:
    """Execute the selected source-local direct-wrench screen without fallback."""

    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        details = "; ".join(preflight.diagnostics) or "no local direct-wrench translator"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {details}")
    definition = resolve_local_direct_wrench_screen_definition(composition)
    if definition is None:
        raise ValueError("no source-local direct-wrench screen factory is registered for this composition")

    config = definition.config_factory()
    plan = compile_local_direct_wrench_screen_mission(
        composition,
        family_id=definition.family_id,
        mission_id=definition.mission_id,
        initialization_id=definition.initialization_id,
        screen_config_id=config.id,
    )
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    screen = run_local_direct_wrench_screen(config)
    mission_graph_execution = unobserved_mission_graph_execution(
        composition,
        "The local direct-wrench screen has no mission graph dispatcher; it is not a navigation execution.",
    ).as_dict()
    status_trace = build_committed_status_trace(composition, _status_samples(screen))
    semantic_action_trace = _direct_wrench_control_trace(composition, screen)
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    result = LocalDirectWrenchCompositionExecution(
        composition=composition,
        preflight=preflight,
        plan=plan,
        output_dir=destination,
        screen=screen,
        mission_graph_execution=mission_graph_execution,
        status_trace=status_trace,
        semantic_action_trace=semantic_action_trace,
        claim_boundary=definition.claim_boundary,
    )
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "plan.json", plan.manifest())
    _write_json(destination / "local_screen.json", screen.as_dict())
    _write_json(destination / "truth_telemetry.json", list(screen.rows))
    _write_json(destination / "objective_report.json", _screen_evaluation(screen))
    _write_json(destination / "mission_graph_execution.json", mission_graph_execution)
    _write_json(destination / "status_trace.json", status_trace)
    _write_json(destination / "semantic_action_trace.json", semantic_action_trace)
    _write_json(destination / "resource_ledger.json", resource_ledger)
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _screen_evaluation(screen: LocalDirectWrenchScreenExecution) -> dict[str, object]:
    """Make the local pass condition independently inspectable in artifacts."""

    final_fraction = screen.final_error_norm / max(screen.initial_error_norm, 1.0e-12)
    statuses = screen.observed_statuses
    return {
        "schema": "taoryx.local-direct-wrench-screen-evaluation/v1alpha1",
        "kind": "local_controller_recovery_screen",
        "screen_pass": screen.mission_pass,
        "requirements": [
            {
                "id": "equilibrium_wrench_feasible",
                "actual": screen.equilibrium_projection.status,
                "limit": "feasible",
                "passed": screen.equilibrium_projection.status == "feasible",
            },
            {
                "id": "equilibrium_reference_derivative",
                "actual": screen.equilibrium_derivative_norm,
                "limit": screen.config.equilibrium_derivative_norm_limit,
                "passed": screen.equilibrium_derivative_norm <= screen.config.equilibrium_derivative_norm_limit,
            },
            {"id": "closed_loop_hurwitz", "passed": screen.lqr.hurwitz},
            {
                "id": "final_error_fraction",
                "actual": final_fraction,
                "limit": screen.config.final_error_fraction_limit,
                "passed": final_fraction < screen.config.final_error_fraction_limit,
            },
            {"id": "all_requests_feasible", "actual": list(statuses), "passed": statuses == ("feasible",)},
        ],
        "claim_boundary": ("The result assesses one local screen only. It is not an independent route, terminal, or physical-effector mission evaluation."),
    }
    ####


def _direct_wrench_control_trace(
    composition: CompiledVehicleComposition,
    screen: LocalDirectWrenchScreenExecution,
) -> dict[str, object]:
    """Project the actual held total-wrench requests from the local screen.

    This is intentionally an action trace for the explicitly declared direct
    wrench bridge, not evidence of an actuator allocation. The local screen's
    own accepted rows are the authority for both requested values and timing.
    """

    samples: list[BatchControlSample] = []
    previous_time: float | None = None
    for index, row in enumerate(screen.rows):
        time_s = _number(row, "time_s")
        wrench = _mapping(row.get("wrench"), "screen wrench")
        requested = _mapping(wrench.get("requested_wrench"), "requested wrench")
        samples.append(
            BatchControlSample(
                interval_start_time_s=time_s if previous_time is None else previous_time,
                committed_truth_time_s=time_s,
                requested_actions={
                    "wrench.force.command": [_number(requested, name) for name in ("force_x_n", "force_y_n", "force_z_n")],
                    "wrench.moment.command": [
                        _number(requested, name) for name in ("moment_x_nm", "moment_y_nm", "moment_z_nm")
                    ],
                },
                achieved_effectors={},
            )
        )
        previous_time = time_s
    return build_committed_control_trace(composition, samples)
    ####


def _status_samples(screen: LocalDirectWrenchScreenExecution) -> tuple[BatchTruthSample, ...]:
    """Flatten screen telemetry for the exact resolved batch-status contract."""

    samples: list[BatchTruthSample] = []
    for index, row in enumerate(screen.rows):
        state = _mapping(row.get("state"), "screen state")
        wrench = _mapping(row.get("wrench"), "screen wrench")
        requested = _mapping(wrench.get("requested_wrench"), "requested wrench")
        achieved = _mapping(wrench.get("achieved_wrench"), "achieved wrench")
        residual = _mapping(wrench.get("residual_wrench"), "residual wrench")
        raw = {
            "body_velocity_m_s": [_number(state, name) for name in ("u_m_s", "v_m_s", "w_m_s")],
            "body_rate_rad_s": [_number(state, name) for name in ("p_rad_s", "q_rad_s", "r_rad_s")],
            "requested_force_body_n": [_number(requested, name) for name in ("force_x_n", "force_y_n", "force_z_n")],
            "requested_moment_body_nm": [_number(requested, name) for name in ("moment_x_nm", "moment_y_nm", "moment_z_nm")],
            "achieved_force_body_n": [_number(achieved, name) for name in ("force_x_n", "force_y_n", "force_z_n")],
            "achieved_moment_body_nm": [_number(achieved, name) for name in ("moment_x_nm", "moment_y_nm", "moment_z_nm")],
            "residual_force_body_n": [_number(residual, name) for name in ("force_x_n", "force_y_n", "force_z_n")],
            "residual_moment_body_nm": [_number(residual, name) for name in ("moment_x_nm", "moment_y_nm", "moment_z_nm")],
            "wrench_status": str(wrench["status"]),
            "wrench_saturated": bool(wrench["position_saturated"] or wrench["rate_limited"]),
            "control_realization": "direct_wrench_screen",
            "physical_effector_allocation": False,
        }
        samples.append(
            BatchTruthSample(
                time_s=_number(row, "time_s"),
                execution_status="completed" if index == len(screen.rows) - 1 else "active",
                raw_values=raw,
            )
        )
    return tuple(samples)
    ####


def _mapping(value: object, label: str) -> Mapping[str, object]:
    """Require expected screen telemetry mappings before projection."""

    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value
    ####


def _number(values: Mapping[str, object], name: str) -> float:
    """Read one finite numeric screen channel without coercing malformed data."""

    value = values.get(name)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"screen telemetry {name!r} must be finite numeric")
    return float(value)
    ####


def _write_json(path: Path, payload: object) -> None:
    """Write one deterministic execution artifact."""

    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


__all__ = ["LocalDirectWrenchCompositionExecution", "execute_local_direct_wrench_composition"]
