"""Source-owned execution for composed language-backed fixed-wing missions.

This is the runtime half of the initial vehicle-composition vertical slice.
It accepts an immutable semantic composition, insists on its capability-route
preflight, materializes disposable native inputs, and emits truth telemetry
plus independent objective and envelope results.  It intentionally does not
render a qualification board or promote an evidence tier; those remain
separate consumers of this normal execution artifact.
"""

from __future__ import annotations

import csv
import json
import math
import shutil
import tempfile
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import yaml

from .composition_control_trace import BatchControlSample, build_committed_control_trace, control_trace_summary
from .composition_evaluation import build_composition_trajectory_evaluation
from .composition_graph_evidence import unobserved_mission_graph_execution
from .composition_resource_ledger import build_committed_resource_ledger
from .composition_sensor_trace import BatchTruthSample, build_declared_sensor_trace
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .language.grammar_contracts import GrammarProfile
from .language_backed_racetrack import materialize_powered_fixed_wing_composition
from .mission_objectives import TruthObjectiveSpec, evaluate_truth_objectives
from .racetrack_template import load_racetrack_template_catalog
from .runtime.runner import RunReport, run_files
from .vehicle_composition import CompiledVehicleComposition, resolve_vehicle_composition_interface_contract
from .vehicle_execution_bindings import resolve_vehicle_execution_binding
from .vehicle_execution_preflight import VehicleExecutionPreflight, preflight_vehicle_composition

_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True, slots=True)
class LanguageBackedCompositionExecution:
    """Normalized result of one composed source-table mission execution."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    source_mission_id: str
    materialized_mission_id: str
    output_dir: Path
    runtime: RunReport
    numerical_valid: bool
    envelope: dict[str, object]
    truth_evaluation: dict[str, object]
    mission_graph_execution: dict[str, object]
    sensor_trace: dict[str, object] | None
    status_trace: dict[str, object] | None
    control_provenance: dict[str, object]
    semantic_action_trace: dict[str, object] | None

    @property
    def mission_pass(self) -> bool:
        """Return the hard conjunction of runtime, envelope, and truth gates."""

        return self.numerical_valid and bool(self.envelope["pass"]) and bool(self.truth_evaluation["mission_pass"])
        ####

    @property
    def execution_limit_reason(self) -> str | None:
        """Return a non-error runtime cap that left an otherwise inspectable prefix.

        The generic runtime uses exit code one when a case is incomplete and
        exit code two for a diagnostic error.  A caller-supplied step cap is
        therefore a distinct execution disposition, not numerical failure.
        """

        incomplete = tuple(result for result in self.runtime.results if not result.completed)
        if self.runtime.exit_code == 1 and incomplete and all(result.stop_reason == "max_steps" for result in incomplete):
            return "max_steps"
        return None
        ####

    def runtime_summary(self) -> dict[str, object]:
        """Return compact runtime provenance without duplicating telemetry."""

        return {
            "exit_code": self.runtime.exit_code,
            "execution_limit_reason": self.execution_limit_reason,
            "cases": self.runtime.cases,
            "results": [
                {
                    "completed": result.completed,
                    "stop_reason": result.stop_reason,
                    "vehicles": sorted(result.states),
                }
                for result in self.runtime.results
            ],
            "diagnostics": [item.model_dump(mode="json") for item in self.runtime.diagnostics],
            "runtime_outputs": list(self.runtime.outputs),
        }
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the stable public CLI/result payload."""

        return {
            "schema": "taoryx.language-backed-composition-execution/v1alpha1",
            "status": "nominal_case_pass" if self.mission_pass else "mission_failed",
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "source_mission_id": self.source_mission_id,
            "materialized_mission_id": self.materialized_mission_id,
            "output_dir": str(self.output_dir),
            "runtime": self.runtime_summary(),
            "numerical_valid": self.numerical_valid,
            "execution_limit_reason": self.execution_limit_reason,
            "envelope": self.envelope,
            "truth_evaluation": self.truth_evaluation,
            "mission_graph_execution": self.mission_graph_execution,
            "sensor_trace": _sensor_trace_summary(self.sensor_trace),
            "status_trace": None if self.status_trace is None else status_trace_summary(self.status_trace),
            "control_provenance": self.control_provenance,
            "semantic_action_trace": None if self.semantic_action_trace is None else control_trace_summary(self.semantic_action_trace),
            "mission_pass": self.mission_pass,
            "claim_boundary": (
                "This is one composed, language-backed nominal mission execution. It does not promote a family "
                "evidence tier, establish physical-effector behavior at a lower fidelity, or replace robustness, "
                "convergence, batch/step-parity, and qualification-pack gates."
            ),
        }
        ####

    ####


def execute_powered_fixed_wing_composition(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    *,
    max_steps: int | None = None,
) -> LanguageBackedCompositionExecution:
    """Execute one X8/B747 composition without a family or route fallback."""

    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        diagnostics = "; ".join(preflight.diagnostics) or "no translation-ready geometry"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {diagnostics}")
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="taoryx-language-backed-composition-") as temporary:
        materialized = materialize_powered_fixed_wing_composition(composition, Path(temporary))
        mission = _mission(materialized.mission_config, materialized.materialized_mission_id)
        tables = _mission_tables(mission)
        _copy_inputs(destination / "inputs", (materialized.problem, materialized.mission_config, materialized.racetrack_config, *tables))
        runtime = run_files(
            materialized.problem,
            tables,
            output_dir=destination / "runtime",
            max_steps=int(mission["max_steps"]) if max_steps is None else max_steps,
            integrator=str(mission.get("integrator", "rk4")),
            profile=GrammarProfile.TAORYX,
            control_provenance="intervals",
        )
        control_provenance_path = destination / "runtime" / "control_provenance.json"
        control_provenance_available = control_provenance_path.is_file()
        control_provenance = _load_control_provenance(control_provenance_path) if control_provenance_available else _unavailable_control_provenance(runtime)
        semantic_action_trace = (
            _language_backed_semantic_action_trace(composition, control_provenance)
            if control_provenance_available and _emits_language_backed_semantic_action_trace(composition)
            else None
        )

        states = tuple(next(iter(runtime.results[0].states.values()), ())) if runtime.results else ()
        rows = _local_truth_rows(states)
        route = load_racetrack_template_catalog(materialized.racetrack_config).get(str(mission["racetrack_binding"]))
        objective_specs = _objective_specs(mission, route)
        numerical_valid = runtime.exit_code == 0 and all(result.completed for result in runtime.results)
        envelope = _evaluate_envelope(mission, rows)
        truth_evaluation = evaluate_truth_objectives(
            objective_specs,
            rows,
            hard_gates_passed=numerical_valid and bool(envelope["pass"]),
        )
        samples = _fixed_wing_sensor_samples(rows)
        sensor_trace = build_declared_sensor_trace(composition, samples) if samples else None
        status_trace = build_committed_status_trace(composition, samples) if samples else None
        resource_ledger = build_committed_resource_ledger(composition, status_trace) if status_trace is not None else None
        mission_graph_execution = unobserved_mission_graph_execution(
            composition,
            "The language-backed batch runner emits truth telemetry and independent objectives but no committed "
            "controller segment-transition dispatches for this composition.",
        ).as_dict()

    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "proposal.json", materialized.proposal.manifest())
    _write_json(destination / "envelope_report.json", envelope)
    _write_json(destination / "objective_report.json", truth_evaluation)
    _write_json(destination / "mission_graph_execution.json", mission_graph_execution)
    _write_json(destination / "control_provenance.json", control_provenance)
    if semantic_action_trace is not None:
        _write_json(destination / "semantic_action_trace.json", semantic_action_trace)
    if sensor_trace is not None:
        _write_json(destination / "sensor_observations.json", sensor_trace)
    if status_trace is not None:
        _write_json(destination / "status_trace.json", status_trace)
    if resource_ledger is not None:
        _write_json(destination / "resource_ledger.json", resource_ledger)
    result = LanguageBackedCompositionExecution(
        composition=composition,
        preflight=preflight,
        source_mission_id=materialized.source_mission_id,
        materialized_mission_id=materialized.materialized_mission_id,
        output_dir=destination,
        runtime=runtime,
        numerical_valid=numerical_valid,
        envelope=envelope,
        truth_evaluation=truth_evaluation,
        mission_graph_execution=mission_graph_execution,
        sensor_trace=sensor_trace,
        status_trace=status_trace,
        control_provenance=control_provenance,
        semantic_action_trace=semantic_action_trace,
    )
    _write_json(destination / "runtime_report.json", result.runtime_summary())
    _write_json(
        destination / "evaluation.json",
        build_composition_trajectory_evaluation(
            composition,
            preflight,
            truth_evaluation,
            runtime={
                **result.runtime_summary(),
                "numerical_valid": numerical_valid,
                "execution_limit_reason": result.execution_limit_reason,
                "hard_gates_passed": result.mission_pass,
                "mission_graph_execution": mission_graph_execution,
            },
            envelope=envelope,
            claim_boundary=str(result.as_dict()["claim_boundary"]),
            status_trace=status_trace,
        ).as_dict(),
    )
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _load_control_provenance(path: Path) -> dict[str, object]:
    """Load the runtime-owned control ledger without turning it into an action trace."""

    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("runtime control provenance is not a JSON object")
    if payload.get("schema") != "taoryx.runtime-control-provenance/v1alpha1":
        raise ValueError("runtime control provenance has an unsupported schema")
    return cast(dict[str, object], payload)
    ####


def _unavailable_control_provenance(runtime: RunReport) -> dict[str, object]:
    """Preserve an early source-runtime failure as a result, not an exception.

    A malformed or failed native problem can return a valid ``RunReport``
    before lowering reaches the accepted-interval ledger writer.  Composition
    must retain that runtime failure and its diagnostics instead of treating a
    missing optional ledger as a second, unrelated executor failure.
    """

    return {
        "schema": "taoryx.runtime-control-provenance/v1alpha1",
        "status": "not_available",
        "detail": "not_available",
        "cases": [],
        "runtime_diagnostics": [item.model_dump(mode="json") for item in runtime.diagnostics],
        "claim_boundary": (
            "No accepted-interval control provenance was produced because the source runtime did not complete "
            "the lowering/execution path. No semantic action trace or controller-action claim is available."
        ),
    }
    ####


def _emits_language_backed_semantic_action_trace(composition: CompiledVehicleComposition) -> bool:
    """Return the binding-declared action-trace capability for this run.

    The execution-binding registry—not a family-specific conditional in this
    executor—is the promotion authority. The trace builder still fail-closes
    if an interval omits a public action or a solver stage changes it.
    """

    return resolve_vehicle_execution_binding(composition, "batch").batch_action_trace == "emits_committed_interval_trace"
    ####


def _language_backed_semantic_action_trace(
    composition: CompiledVehicleComposition,
    provenance: dict[str, object],
) -> dict[str, object]:
    """Project held native bridge controls into the exact composition contract.

    The autonomous racetrack reference remains mission guidance, not an
    invented external action.  This trace contains only the public bridge
    actions (for example X8 throttle and elevon coordinates) actually held by
    the native runtime across each accepted integration interval.
    """

    if provenance.get("detail") not in {"intervals", "full"}:
        raise ValueError("language-backed semantic action trace requires accepted interval controls")
    cases = provenance.get("cases")
    if not isinstance(cases, list) or len(cases) != 1 or not isinstance(cases[0], dict):
        raise ValueError("language-backed semantic action trace requires exactly one runtime case")
    vehicles = cases[0].get("vehicles")
    if not isinstance(vehicles, dict) or len(vehicles) != 1:
        raise ValueError("language-backed semantic action trace requires exactly one runtime vehicle")
    vehicle = next(iter(vehicles.values()))
    if not isinstance(vehicle, dict):
        raise ValueError("language-backed runtime control provenance has an invalid vehicle record")
    intervals = vehicle.get("control_interval_records")
    if not isinstance(intervals, list) or not intervals:
        raise ValueError("language-backed runtime control provenance has no accepted intervals")
    contract = resolve_vehicle_composition_interface_contract(composition)
    action_bindings = {
        channel.id: channel
        for channel in contract.action_channels
        if channel.availability in {"available", "available_in_batch"}
    }
    if any(not isinstance(channel.binding.get("native_action"), str) or not channel.binding.get("native_action") for channel in action_bindings.values()):
        raise ValueError("language-backed semantic actions require declared native-control bindings")
    if not action_bindings:
        samples_without_public_actions: list[BatchControlSample] = []
        for index, interval in enumerate(intervals):
            if not isinstance(interval, dict):
                raise ValueError(f"language-backed control interval {index} is not an object")
            samples_without_public_actions.append(
                BatchControlSample(
                    float(interval["interval_start_time_s"]),
                    float(interval["committed_truth_time_s"]),
                    {},
                    {},
                )
            )
        return build_committed_control_trace(
            composition,
            samples_without_public_actions,
        )
    samples: list[BatchControlSample] = []
    for index, interval in enumerate(intervals):
        if not isinstance(interval, dict):
            raise ValueError(f"language-backed control interval {index} is not an object")
        if interval.get("solver_stage_control_mutation_detected") is True:
            raise ValueError("language-backed runtime changed a public bridge control during a solver stage; semantic action tracing is forbidden")
        controls = interval.get("controls_at_interval_start")
        if not isinstance(controls, dict):
            raise ValueError(f"language-backed control interval {index} has no native control mapping")
        requested: dict[str, object] = {}
        for action_id, channel in action_bindings.items():
            native_name = channel.binding.get("native_action")
            assert isinstance(native_name, str)
            value = controls.get(native_name)
            if isinstance(value, bool) or not isinstance(value, int | float):
                raise ValueError(f"language-backed control interval {index} omits declared bridge action {action_id!r}")
            requested[action_id] = float(value) >= 0.5 if channel.value_type == "boolean" else float(value)
        samples.append(
            BatchControlSample(
                float(interval["interval_start_time_s"]),
                float(interval["committed_truth_time_s"]),
                requested,
                {},
            )
        )
    return build_committed_control_trace(composition, samples)
    ####


def _mission(path: Path, mission_id: str) -> dict[str, Any]:
    """Return the exact materialized mission contract."""

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    candidates = payload.get("missions", ()) if isinstance(payload, dict) else ()
    mission = next((item for item in candidates if isinstance(item, dict) and item.get("id") == mission_id), None)
    if mission is None:
        raise KeyError(f"materialized mission {mission_id!r} is absent from {path}")
    return mission
    ####


def _mission_tables(mission: dict[str, Any]) -> tuple[Path, ...]:
    """Resolve the immutable table inputs declared by one mission contract."""

    return tuple(_ROOT / str(item) for item in mission.get("tables", ()))
    ####


def _objective_specs(mission: dict[str, Any], route: Any) -> tuple[TruthObjectiveSpec, ...]:
    """Resolve route-owned gates while retaining mission-owned acceptance."""

    resolved: list[TruthObjectiveSpec] = []
    for item in [*mission.get("objectives", ()), mission.get("terminal", {})]:
        if not isinstance(item, dict):
            continue
        values = dict(item)
        values.pop("racetrack_phase", None)
        gate_id = values.pop("racetrack_gate_id", None)
        if gate_id is not None:
            gate = next((candidate for candidate in route.gates if candidate.id == gate_id), None)
            if gate is None:
                raise KeyError(f"mission objective references unknown racetrack gate {gate_id!r}")
            values["target"] = gate.target(route.speed_m_s)
            values["gate_normal"] = gate.gate_normal()
        if values.get("id") is None:
            values["id"] = "terminal-contract"
        resolved.append(TruthObjectiveSpec(**values))
    return tuple(resolved)
    ####


def _local_truth_rows(states: tuple[Any, ...]) -> tuple[dict[str, object], ...]:
    """Convert language-runtime truth states into local NED evidence rows."""

    if not states:
        return ()
    first = dict(states[0].named)
    latitude_0 = _number(first.get("latitude_deg", first.get("lat")), 0.0)
    longitude_0 = _number(first.get("longitude_deg", first.get("long")), 0.0)
    east_scale = 111_320.0 * math.cos(math.radians(latitude_0))
    rows: list[dict[str, object]] = []
    for state in states:
        named = dict(state.named)
        latitude = _number(named.get("latitude_deg", named.get("lat")), latitude_0)
        longitude = _number(named.get("longitude_deg", named.get("long")), longitude_0)
        altitude_m = named.get("altitude_m")
        if altitude_m is None and "alt" in named:
            altitude_m = float(named["alt"]) * 0.3048
        speed_m_s = named.get("speed_m_s")
        if speed_m_s is None and "vel" in named:
            speed_m_s = abs(float(named["vel"])) * 0.3048
        named.setdefault("latitude_deg", latitude)
        named.setdefault("longitude_deg", longitude)
        if altitude_m is not None:
            named.setdefault("altitude_m", float(altitude_m))
        if speed_m_s is not None:
            named.setdefault("speed_m_s", float(speed_m_s))
        if "flight_path_angle_deg" not in named and "gama" in named:
            named["flight_path_angle_deg"] = float(named["gama"])
        if "heading_deg" not in named and "psi" in named:
            named["heading_deg"] = float(named["psi"])
        rows.append(
            {
                **named,
                "time_s": float(state.time),
                "north_m": (latitude - latitude_0) * 111_320.0,
                "east_m": (longitude - longitude_0) * east_scale,
            }
        )
    return tuple(rows)
    ####


def _fixed_wing_sensor_samples(rows: tuple[dict[str, object], ...]) -> tuple[BatchTruthSample, ...]:
    """Map language runtime rows back to the exact interface-native status view."""

    samples: list[BatchTruthSample] = []
    for index, row in enumerate(rows):
        kinematic_attitude_deg = _kinematic_attitude_from_row(row)
        kinematic_body_rate_rad_s = _kinematic_body_rate_from_row(row)
        raw = {
            "1": {
                "alt": row.get("alt"),
                "vel": row.get("vel"),
                "gama": row.get("gama"),
                "psi": row.get("psi"),
                "mass": row.get("mass"),
                "guidance_override_active": row.get("guidance_override_active"),
                "kinematic_roll_deg": kinematic_attitude_deg[0],
                "kinematic_pitch_deg": kinematic_attitude_deg[1],
                "kinematic_yaw_deg": kinematic_attitude_deg[2],
                "kinematic_body_rate_p_rad_s": kinematic_body_rate_rad_s[0],
                "kinematic_body_rate_q_rad_s": kinematic_body_rate_rad_s[1],
                "kinematic_body_rate_r_rad_s": kinematic_body_rate_rad_s[2],
            }
        }
        samples.append(
            BatchTruthSample(
                time_s=_number(row.get("time_s"), 0.0),
                raw_values=raw,
                execution_status="completed" if index == len(rows) - 1 else "active",
            )
        )
    return tuple(samples)
    ####


def _sensor_trace_summary(trace: dict[str, object] | None) -> dict[str, object] | None:
    """Keep the execution manifest small while the trace remains a sibling artifact."""

    if trace is None:
        return None
    samples = trace.get("samples")
    return {
        "schema": trace["schema"],
        "observation_profile_id": trace["observation_profile_id"],
        "sample_count": len(samples) if isinstance(samples, list) else 0,
        "artifact": "sensor_observations.json",
    }
    ####


def _evaluate_envelope(mission: dict[str, Any], rows: tuple[dict[str, object], ...]) -> dict[str, object]:
    """Evaluate hard mission channel bounds independently of route guidance."""

    checks: list[dict[str, object]] = []
    for bound in mission.get("envelope", ()):
        if not isinstance(bound, dict):
            continue
        channel = str(bound["channel"])
        minimum = float(bound["minimum"])
        maximum = float(bound["maximum"])
        violations = [{"time_s": row.get("time_s"), "value": row.get(channel)} for row in rows if _outside_envelope(row.get(channel), minimum, maximum)]
        checks.append({"channel": channel, "minimum": minimum, "maximum": maximum, "violations": violations, "pass": not violations})
    return {"schema_version": 1, "checks": checks, "pass": all(bool(check["pass"]) for check in checks)}
    ####


def _number(value: object | None, default: float) -> float:
    """Convert one optional runtime scalar without weakening typed artifacts."""

    return default if value is None else float(cast(Any, value))
    ####


def _kinematic_attitude_from_row(row: Mapping[str, object]) -> tuple[float, float, float]:
    """Return emitted Euler attitude or reconstruct the initial kinematic sidecar."""

    direct = tuple(row.get(name) for name in ("kinematic_roll_deg", "kinematic_pitch_deg", "kinematic_yaw_deg"))
    if all(isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(float(value)) for value in direct):
        return cast(tuple[float, float, float], tuple(float(cast(float, value)) for value in direct))
    quaternion = tuple(row.get(name) for name in ("qw", "qx", "qy", "qz"))
    if not all(isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(float(value)) for value in quaternion):
        return (0.0, 0.0, 0.0)
    qw, qx, qy, qz = (float(cast(float, value)) for value in quaternion)
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
    ####


def _kinematic_body_rate_from_row(row: Mapping[str, object]) -> tuple[float, float, float]:
    """Return emitted kinematic body rates or the initial zero-rate sidecar."""

    values = tuple(
        row.get(name)
        for name in (
            "kinematic_body_rate_p_rad_s",
            "kinematic_body_rate_q_rad_s",
            "kinematic_body_rate_r_rad_s",
        )
    )
    if all(isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(float(value)) for value in values):
        return cast(tuple[float, float, float], tuple(float(cast(float, value)) for value in values))
    return (0.0, 0.0, 0.0)
    ####


def _outside_envelope(value: object | None, minimum: float, maximum: float) -> bool:
    """Return whether a declared numeric channel is absent, invalid, or out of bounds."""

    if value is None:
        return True
    try:
        numeric = float(cast(Any, value))
    except (TypeError, ValueError):
        return True
    return not math.isfinite(numeric) or not minimum <= numeric <= maximum
    ####


def _copy_inputs(destination: Path, paths: tuple[Path, ...]) -> None:
    """Copy every actual execution input into the normal run artifact."""

    destination.mkdir(parents=True, exist_ok=True)
    for path in paths:
        shutil.copy2(path, destination / path.name)
    ####


def _write_csv(path: Path, rows: tuple[dict[str, object], ...]) -> None:
    """Write canonical truth telemetry with a stable union of channels."""

    keys = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    ####


def _write_json(path: Path, value: object) -> None:
    """Write a reproducible human-readable JSON artifact."""

    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


__all__ = ["LanguageBackedCompositionExecution", "execute_powered_fixed_wing_composition"]
