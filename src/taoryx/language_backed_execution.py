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
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import yaml

from .composition_sensor_trace import BatchTruthSample, build_declared_sensor_trace
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .language.grammar_contracts import GrammarProfile
from .language_backed_racetrack import materialize_powered_fixed_wing_composition
from .mission_objectives import TruthObjectiveSpec, evaluate_truth_objectives
from .racetrack_template import load_racetrack_template_catalog
from .runtime.runner import RunReport, run_files
from .vehicle_composition import CompiledVehicleComposition
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
    sensor_trace: dict[str, object] | None
    status_trace: dict[str, object]

    @property
    def mission_pass(self) -> bool:
        """Return the hard conjunction of runtime, envelope, and truth gates."""

        return self.numerical_valid and bool(self.envelope["pass"]) and bool(self.truth_evaluation["mission_pass"])
        ####

    def runtime_summary(self) -> dict[str, object]:
        """Return compact runtime provenance without duplicating telemetry."""

        return {
            "exit_code": self.runtime.exit_code,
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
            "envelope": self.envelope,
            "truth_evaluation": self.truth_evaluation,
            "sensor_trace": _sensor_trace_summary(self.sensor_trace),
            "status_trace": status_trace_summary(self.status_trace),
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
        sensor_trace = build_declared_sensor_trace(composition, _fixed_wing_sensor_samples(rows))
        status_trace = build_committed_status_trace(composition, _fixed_wing_sensor_samples(rows))

    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "proposal.json", materialized.proposal.manifest())
    _write_json(destination / "envelope_report.json", envelope)
    _write_json(destination / "objective_report.json", truth_evaluation)
    if sensor_trace is not None:
        _write_json(destination / "sensor_observations.json", sensor_trace)
    _write_json(destination / "status_trace.json", status_trace)
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
        sensor_trace=sensor_trace,
        status_trace=status_trace,
    )
    _write_json(destination / "runtime_report.json", result.runtime_summary())
    _write_json(destination / "execution.json", result.as_dict())
    return result
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
        raw = {
            "1": {
                "alt": row.get("alt"),
                "vel": row.get("vel"),
                "gama": row.get("gama"),
                "psi": row.get("psi"),
                "mass": row.get("mass"),
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
        violations = [
            {"time_s": row.get("time_s"), "value": row.get(channel)}
            for row in rows
            if _outside_envelope(row.get(channel), minimum, maximum)
        ]
        checks.append({"channel": channel, "minimum": minimum, "maximum": maximum, "violations": violations, "pass": not violations})
    return {"schema_version": 1, "checks": checks, "pass": all(bool(check["pass"]) for check in checks)}
    ####


def _number(value: object | None, default: float) -> float:
    """Convert one optional runtime scalar without weakening typed artifacts."""

    return default if value is None else float(cast(Any, value))
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
