"""Source-owned execution for composed reduced powered-fixed-wing missions.

This module is deliberately a family-dispatch seam, not a generic flight
plant.  It binds the common capability-scaled racetrack to the declared A320
or F-16 reduced model, retains family-specific trim/provenance, and returns
the same independent truth-gate artifacts for both vehicles.  It never
substitutes another family or promotes response-law controls into physical
effector evidence.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, cast

from .composition_control_trace import BatchControlSample, build_committed_control_trace, control_trace_summary
from .composition_evaluation import build_composition_trajectory_evaluation
from .composition_graph_evidence import unobserved_mission_graph_execution
from .composition_resource_ledger import build_committed_resource_ledger, resource_ledger_summary
from .composition_sensor_trace import BatchTruthSample
from .composition_status_trace import build_committed_status_trace, status_trace_summary
from .mission_objectives import TruthObjectiveSpec, evaluate_truth_objectives
from .powered_fixed_wing_mission_compiler import CapabilityScaledRacetrack
from .racetrack_template import ResolvedRacetrack
from .trajectory import (
    A320OpenAPModel,
    A320OpenAPOperatingPoint,
    A320Pseudo6DOFModel,
    A320RacetrackRunner,
    F16AttitudeResponsePseudo6DOFModel,
    F16PointMass3DOFModel,
    F16ReducedRacetrackRunner,
    load_f16_reference_plant,
    load_pseudo6dof_catalog,
)
from .trim import TrimResult, TrimSpec
from .variant_runtime_evidence import build_variant_runtime_evidence
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_preflight import VehicleExecutionPreflight, compile_powered_fixed_wing_racetrack_from_composition, preflight_vehicle_composition

_ROOT = Path(__file__).resolve().parents[2]
_F16_STATE_NAMES = ("u_m_s", "v_m_s", "w_m_s", "p_rad_s", "q_rad_s", "r_rad_s")
_F16_CONTROL_NAMES = ("elevator_deg", "aileron_deg", "rudder_deg", "throttle_fraction")
_ReducedMode = Literal["point_mass_3dof", "pseudo_6dof_kinematic_bridge"]


@dataclass(frozen=True, slots=True)
class ReducedFixedWingCompositionExecution:
    """Immutable result of one source-owned reduced-fidelity run."""

    composition: CompiledVehicleComposition
    preflight: VehicleExecutionPreflight
    proposal: CapabilityScaledRacetrack
    output_dir: Path
    runtime: dict[str, object]
    envelope: dict[str, object]
    truth_evaluation: dict[str, object]
    trim: dict[str, object]
    provenance: dict[str, object]
    status_trace: dict[str, object]
    control_trace: dict[str, object]
    claim_boundary: str

    @property
    def mission_pass(self) -> bool:
        """Return the hard conjunction for this nominal reduced run."""

        return bool(self.runtime["hard_gates_passed"]) and bool(self.envelope["pass"]) and bool(self.truth_evaluation["mission_pass"])
        ####

    def as_dict(self) -> dict[str, object]:
        """Return the compact public result without duplicating telemetry."""

        return {
            "schema": "taoryx.reduced-fixed-wing-composition-execution/v1alpha1",
            "status": "nominal_case_pass" if self.mission_pass else "mission_failed",
            "composition": self.composition.model_dump(mode="json", by_alias=True),
            "preflight": self.preflight.as_dict(),
            "proposal": self.proposal.manifest(),
            "output_dir": str(self.output_dir),
            "runtime": self.runtime,
            "envelope": self.envelope,
            "truth_evaluation": self.truth_evaluation,
            "trim": self.trim,
            "provenance": self.provenance,
            "status_trace": status_trace_summary(self.status_trace),
            "control_trace": control_trace_summary(self.control_trace),
            "mission_pass": self.mission_pass,
            "claim_boundary": self.claim_boundary,
        }
        ####

    ####


def execute_reduced_fixed_wing_composition(
    composition: CompiledVehicleComposition,
    output_dir: str | Path,
    *,
    dt_s: float = 0.2,
) -> ReducedFixedWingCompositionExecution:
    """Run a preflighted A320/F-16 3DOF or pseudo-6DOF composition.

    The selected response profile is part of the model-specific execution
    setup.  Direct-wrench and surface-allocated runs deliberately remain
    outside this reduced executor.
    """

    preflight = preflight_vehicle_composition(composition)
    if preflight.status != "translation_ready":
        details = "; ".join(preflight.diagnostics) or "no translation-ready route"
        raise ValueError(f"cannot execute composition {composition.id!r}: {preflight.status}: {details}")
    mode = _reduced_mode(composition)
    destination = Path(output_dir)
    if destination.exists() and any(destination.iterdir()):
        raise ValueError(f"execution output directory must be empty: {destination}")
    destination.mkdir(parents=True, exist_ok=True)
    proposal = compile_powered_fixed_wing_racetrack_from_composition(composition)

    if composition.family_id == "a320_openap_3dof":
        rows, runtime, envelope, trim, provenance, claim_boundary = _run_a320(composition, proposal.route, mode, dt_s)
    elif composition.family_id == "f16_s119":
        rows, runtime, envelope, trim, provenance, claim_boundary = _run_f16(composition, proposal.route, mode, dt_s)
    else:
        raise ValueError(f"reduced fixed-wing executor has no adapter for family {composition.family_id!r}")

    status_trace = build_committed_status_trace(composition, _status_samples(rows))
    resource_ledger = build_committed_resource_ledger(composition, status_trace)
    control_trace = build_committed_control_trace(composition, _control_samples(rows))
    runtime["mission_graph_execution"] = unobserved_mission_graph_execution(
        composition,
        "The reduced fixed-wing batch runner emits truth telemetry and independent objectives but no committed controller graph dispatches.",
    ).as_dict()
    variant_runtime_evidence = build_variant_runtime_evidence(
        composition,
        status_trace,
        consumed_native_inputs=_variant_native_inputs(runtime),
    )
    runtime["variant_runtime_evidence"] = variant_runtime_evidence
    runtime["resource_ledger"] = resource_ledger_summary(resource_ledger)
    runtime["hard_gates_passed"] = bool(runtime["hard_gates_passed"]) and variant_runtime_evidence["status"] in {
        "pass",
        "not_applicable",
    }
    truth_evaluation = evaluate_truth_objectives(
        _objective_specs(proposal.route),
        rows,
        hard_gates_passed=bool(runtime["hard_gates_passed"]) and bool(envelope["pass"]),
    )
    result = ReducedFixedWingCompositionExecution(
        composition=composition,
        preflight=preflight,
        proposal=proposal,
        output_dir=destination,
        runtime=runtime,
        envelope=envelope,
        truth_evaluation=truth_evaluation,
        trim=trim,
        provenance=provenance,
        status_trace=status_trace,
        control_trace=control_trace,
        claim_boundary=claim_boundary,
    )
    _write_csv(destination / "truth_telemetry.csv", rows)
    _write_json(destination / "composition.json", composition.model_dump(mode="json", by_alias=True))
    _write_json(destination / "preflight.json", preflight.as_dict())
    _write_json(destination / "proposal.json", proposal.manifest())
    _write_json(destination / "runtime_report.json", runtime)
    _write_json(destination / "mission_graph_execution.json", runtime["mission_graph_execution"])
    _write_json(destination / "variant_runtime_evidence.json", runtime["variant_runtime_evidence"])
    _write_json(destination / "envelope_report.json", envelope)
    _write_json(destination / "objective_report.json", truth_evaluation)
    _write_json(destination / "trim.json", trim)
    _write_json(destination / "model_provenance.json", provenance)
    _write_json(destination / "status_trace.json", status_trace)
    _write_json(destination / "resource_ledger.json", resource_ledger)
    _write_json(destination / "semantic_action_trace.json", control_trace)
    _write_json(
        destination / "evaluation.json",
        build_composition_trajectory_evaluation(
            composition,
            preflight,
            truth_evaluation,
            runtime=runtime,
            envelope=envelope,
            claim_boundary=claim_boundary,
            status_trace=status_trace,
            control_trace=control_trace,
        ).as_dict(),
    )
    _write_json(destination / "execution.json", result.as_dict())
    return result
    ####


def _variant_native_inputs(runtime: Mapping[str, object]) -> dict[str, object]:
    """Return the exact native adapter inputs this executor consumed.

    The mapping is deliberately explicit.  It is not reconstructed from a
    semantic initialization after the run, since that would only prove that
    the request contained a value—not that the adapter received it.
    """

    operating_point = runtime.get("native_operating_point")
    if not isinstance(operating_point, Mapping):
        return {}
    mass_kg = operating_point.get("mass_kg")
    if isinstance(mass_kg, int | float) and not isinstance(mass_kg, bool):
        return {"A320OpenAPOperatingPoint.mass_kg": mass_kg}
    return {}
    ####


def _run_a320(
    composition: CompiledVehicleComposition,
    route: ResolvedRacetrack,
    mode: _ReducedMode,
    dt_s: float,
) -> tuple[list[dict[str, float | int | str]], dict[str, object], dict[str, object], dict[str, object], dict[str, object], str]:
    """Run the OpenAP or declared A320 response-law realization."""

    initialization = composition.initialization.inputs
    altitude_m = _input_number(initialization, "altitude_m")
    mass_kg = _input_number(initialization, "mass_kg", default=60000.0)
    base_model = A320OpenAPModel.from_repository(_ROOT)
    operating_point = _a320_operating_point_for_speed(base_model, altitude_m, mass_kg, route.speed_m_s)
    model: A320OpenAPModel | A320Pseudo6DOFModel
    if mode == "point_mass_3dof":
        model = base_model
        trim = model.trim_level_flight(operating_point)
        response_profile = None
    else:
        model = A320Pseudo6DOFModel.from_repository(_ROOT)
        trim = model.trim_pseudo6dof(operating_point)
        _, response_profile = load_pseudo6dof_catalog(_ROOT / "verification/pseudo6dof_profiles.yaml").for_family("a320_openap_3dof")
    actual_speed = (
        model.evaluate(operating_point).true_airspeed_mps if isinstance(model, A320OpenAPModel) else model.openap.evaluate(operating_point).true_airspeed_mps
    )
    if not math.isclose(actual_speed, route.speed_m_s, abs_tol=1.0e-6):
        raise ValueError(f"A320 native operating point does not reproduce the capability-route speed: {actual_speed:.9g} != {route.speed_m_s:.9g} m/s")
    run = A320RacetrackRunner(
        model,
        trim,
        route,
        mode,
        dt_s=dt_s,
        response_profile=response_profile,
        operating_point=operating_point,
    ).run()
    rows = [dict(row) for row in run.rows]
    finite = _finite_rows(rows)
    envelope = _simple_envelope(rows, (("altitude_m", 0.0, 13000.0), ("speed_m_s", 0.0, 300.0)))
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.fixed_wing.openap.v1",
        "mode": mode,
        "dt_s": dt_s,
        "duration_s": float(rows[-1]["time_s"]) if rows else 0.0,
        "numerical_valid": run.numerical_valid,
        "failure": run.failure,
        "hard_gates_passed": run.numerical_valid and finite and bool(envelope["pass"]),
        "native_operating_point": {
            "altitude_m": altitude_m,
            "mach": operating_point.mach,
            "mass_kg": mass_kg,
        },
    }
    claim = (
        "OpenAP point-mass performance with a bounded kinematic navigation realization; no physical attitude, "
        "moment, surface, actuator, or manufacturer-aircraft claim."
        if mode == "point_mass_3dof"
        else "OpenAP performance plus the declared JSBSim rotational surrogate and Taoryx response law; policy "
        "surface commands are diagnostic only and do not establish physical actuator realization."
    )
    return rows, runtime, envelope, cast(dict[str, object], trim.as_dict()), cast(dict[str, object], dict(model.provenance)), claim
    ####


def _a320_operating_point_for_speed(
    model: A320OpenAPModel,
    altitude_m: float,
    mass_kg: float,
    target_speed_m_s: float,
) -> A320OpenAPOperatingPoint:
    """Solve the OpenAP Mach input that realizes one semantic speed target.

    The public composition exposes speed in the common powered-fixed-wing
    vocabulary; OpenAP's native independent variable is Mach.  This small
    deterministic adapter solve makes that conversion explicit instead of
    silently starting at the historical fixed Mach used by an older tool.
    """

    lower = 0.10
    upper = 0.82
    for _ in range(64):
        middle = 0.5 * (lower + upper)
        candidate = A320OpenAPOperatingPoint(altitude_m, middle, mass_kg)
        speed = model.evaluate(candidate).true_airspeed_mps
        if speed < target_speed_m_s:
            lower = middle
        else:
            upper = middle
    point = A320OpenAPOperatingPoint(altitude_m, 0.5 * (lower + upper), mass_kg)
    achieved_speed = model.evaluate(point).true_airspeed_mps
    if not math.isclose(achieved_speed, target_speed_m_s, abs_tol=1.0e-6):
        raise ValueError(f"A320 semantic speed {target_speed_m_s:.9g} m/s is outside the current OpenAP Mach conversion domain")
    return point
    ####


def _run_f16(
    composition: CompiledVehicleComposition,
    route: ResolvedRacetrack,
    mode: _ReducedMode,
    dt_s: float,
) -> tuple[list[dict[str, float | int | str]], dict[str, object], dict[str, object], dict[str, object], dict[str, object], str]:
    """Run the source-local F-16 point or response-law reduction."""

    source, trim, trim_pitch_rad = _f16_source_trim()
    observed_mach = float(source.evaluate_loads(trim.state, trim.controls, altitude_m=0.0)["mach"])
    requested_mach = _input_number(composition.initialization.inputs, "mach")
    if not math.isclose(requested_mach, observed_mach, abs_tol=1.0e-9):
        raise ValueError(f"F-16 reduced racetrack is pinned to its source trim Mach; requested {requested_mach:.12g}, expected {observed_mach:.12g}")
    model: F16PointMass3DOFModel | F16AttitudeResponsePseudo6DOFModel
    if mode == "point_mass_3dof":
        model = F16PointMass3DOFModel(source, trim, trim_pitch_rad)
        response_profile = None
    else:
        linearization = source.linearize_local(
            trim.state,
            trim.controls,
            trim_pitch_rad=trim_pitch_rad,
            altitude_m=0.0,
            state_step=1.0e-5,
            control_step=1.0e-5,
        )
        model = F16AttitudeResponsePseudo6DOFModel(source, trim, linearization, trim_pitch_rad)
        _, response_profile = load_pseudo6dof_catalog(_ROOT / "verification/pseudo6dof_profiles.yaml").for_family("f16_s119")
    run = F16ReducedRacetrackRunner(model, trim, route, mode, dt_s=dt_s, response_profile=response_profile).run()
    rows = [dict(row) for row in run.rows]
    for row in rows:
        row["mass_kg"] = source.mass_kg
    finite = _finite_rows(rows)
    envelope = _f16_envelope(rows)
    runtime: dict[str, object] = {
        "adapter_id": "taoryx.fixed_wing.daveml.v1",
        "mode": mode,
        "dt_s": dt_s,
        "duration_s": float(rows[-1]["time_s"]) if rows else 0.0,
        "numerical_valid": run.numerical_valid,
        "failure": run.failure,
        "hard_gates_passed": run.numerical_valid and finite and bool(envelope["pass"]),
        "native_operating_point": {"altitude_m": 0.0, "mach": observed_mach, "trim_pitch_rad": trim_pitch_rad},
    }
    provenance = {
        "model_id": "f16-s119-source-reduced-racetrack",
        "source_import": "families/reference_f16_s119/plant/daveml-import.json",
        "atmosphere": "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
        "response_profile_id": response_profile.id if response_profile is not None else "not_applicable_point_mass",
    }
    claim = (
        "Source-force local point-mass reduction with bounded translational route response; no physical moments, "
        "surface allocation, actuator, or full aircraft controller claim."
        if mode == "point_mass_3dof"
        else "Source-derived local attitude/rate response bridge with quaternion kinematics; no physical moment "
        "balance, surface allocation, actuator, or source-controller-equivalence claim."
    )
    return rows, runtime, envelope, cast(dict[str, object], trim.as_dict()), cast(dict[str, object], provenance), claim
    ####


def _f16_source_trim() -> tuple[Any, TrimResult, float]:
    """Load the immutable source operating point retained by the F-16 reduction."""

    evidence = json.loads((_ROOT / "verification/f16_runtime_linearization_evidence.json").read_text(encoding="utf-8"))
    state = {name: float(value) for name, value in dict(evidence["trim_state"]).items()}
    controls = {name: float(value) for name, value in dict(evidence["trim_controls"]).items()}
    spec = TrimSpec(
        state_names=_F16_STATE_NAMES,
        control_names=_F16_CONTROL_NAMES,
        residual_names=_F16_STATE_NAMES,
        state_initial=state,
        control_initial=controls,
        operating_point={"trim_pitch_rad": float(evidence["metadata"]["trim_pitch_rad"])},
    )
    trim = TrimResult(spec, state, controls, {name: 0.0 for name in _F16_STATE_NAMES}, 0.0, True, 1, "source trim", 0, 0.0)
    source = load_f16_reference_plant(
        _ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        _ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    return source, trim, float(evidence["metadata"]["trim_pitch_rad"])
    ####


def _reduced_mode(composition: CompiledVehicleComposition) -> _ReducedMode:
    """Map only the two explicitly supported reduced tiers."""

    mapping: dict[str, _ReducedMode] = {
        "point_mass_3dof": "point_mass_3dof",
        "pseudo_6dof": "pseudo_6dof_kinematic_bridge",
    }
    try:
        return mapping[composition.fidelity]
    except KeyError as error:
        raise ValueError(f"reduced fixed-wing executor supports only 3DOF/pseudo-6DOF, not {composition.fidelity!r}") from error
    ####


def _objective_specs(route: ResolvedRacetrack) -> tuple[TruthObjectiveSpec, ...]:
    """Evaluate all route gates independently from truth telemetry."""

    windows = {window.name: window for window in route.phase_windows}
    return tuple(
        TruthObjectiveSpec(
            id=gate.id,
            objective_type="fly_by_gate",
            target=gate.target(route.speed_m_s),
            tolerance={
                "corridor_m": route.gate_corridor_m,
                "altitude_m": route.gate_altitude_tolerance_m,
                "speed_m_s": route.gate_speed_tolerance_mps,
            },
            gate_normal=cast(tuple[float, float, float], tuple(gate.gate_normal())),
            crossing_direction=1,
            window_start_s=max(0.0, windows[gate.phase].start_s - 12.0),
            window_end_s=route.horizon_s if gate.id == "terminal-start-finish-gate" else windows[gate.phase].end_s + 12.0,
        )
        for gate in route.gates
    )
    ####


def _simple_envelope(rows: list[dict[str, float | int | str]], bounds: tuple[tuple[str, float, float], ...]) -> dict[str, object]:
    """Check declared scalar bounds, allowing only numerical round-off slack."""

    checks: list[dict[str, object]] = []
    for channel, minimum, maximum in bounds:
        comparison_tolerance = 1.0e-9 * max(1.0, abs(minimum), abs(maximum))
        violations = [
            {
                "time_s": row.get("time_s"),
                "value": row.get(channel),
                "lower": minimum,
                "upper": maximum,
                "comparison_tolerance": comparison_tolerance,
            }
            for row in rows
            if channel not in row or not _in_range(row[channel], minimum, maximum, comparison_tolerance)
        ]
        checks.append(
            {
                "channel": channel,
                "minimum": minimum,
                "maximum": maximum,
                "comparison_tolerance": comparison_tolerance,
                "violations": violations,
                "pass": not violations,
            }
        )
    return {"schema_version": 1, "checks": checks, "pass": all(bool(item["pass"]) for item in checks)}
    ####


def _f16_envelope(rows: list[dict[str, float | int | str]]) -> dict[str, object]:
    """Evaluate the F-16 source package's declared local validity bounds."""

    payload = json.loads((_ROOT / "families/reference_f16_s119/plant/daveml-import.json").read_text(encoding="utf-8"))
    validity = payload["package"]["validity_envelope"]
    bounds = (
        ("altitude_m", float(validity["altitude_min_m"]), float(validity["altitude_max_m"])),
        ("mach", float(validity["mach_min"]), float(validity["mach_max"])),
        ("aero_alpha_deg", math.degrees(float(validity["alpha_min_rad"])), math.degrees(float(validity["alpha_max_rad"]))),
        ("aero_beta_deg", math.degrees(float(validity["beta_min_rad"])), math.degrees(float(validity["beta_max_rad"]))),
    )
    return _simple_envelope(rows, bounds)
    ####


def _finite_rows(rows: list[dict[str, float | int | str]]) -> bool:
    """Reject a run with a non-finite numeric evidence channel."""

    return all(math.isfinite(float(value)) for row in rows for value in row.values() if isinstance(value, int | float))
    ####


def _status_samples(rows: list[dict[str, float | int | str]]) -> tuple[BatchTruthSample, ...]:
    """Expose committed reduced-model rows through the portable status seam."""

    return tuple(
        BatchTruthSample(
            time_s=_status_time(row),
            raw_values=_reduced_status_values(row),
            execution_status="completed" if index == len(rows) - 1 else "active",
        )
        for index, row in enumerate(rows)
    )
    ####


def _control_samples(rows: list[dict[str, float | int | str]]) -> tuple[BatchControlSample, ...]:
    """Bind racetrack guidance references to their committed truth intervals.

    The common reduced fixed-wing interface exposes speed, flight-path,
    heading, and bank as semantic guidance. Neither the point-mass nor named
    attitude-response tiers declare physical effectors, so the trace records
    no achieved surface allocation even where a source model exposes internal
    surrogate coordinates.
    """

    samples: list[BatchControlSample] = []
    interval_start_time_s: float | None = None
    for row in rows:
        committed_truth_time_s = _status_time(row)
        if interval_start_time_s is None:
            interval_start_time_s = committed_truth_time_s
        samples.append(
            BatchControlSample(
                interval_start_time_s=interval_start_time_s,
                committed_truth_time_s=committed_truth_time_s,
                requested_actions={
                    "guidance.speed.command": _row_number(row, "route_speed_command_m_s"),
                    "guidance.flight_path_angle.command": _row_number(row, "route_flight_path_command_deg"),
                    "guidance.heading.command": _canonical_heading_deg(_row_number(row, "route_heading_command_deg")),
                    "guidance.bank.command": _row_number(row, "route_bank_command_deg"),
                },
                achieved_effectors={},
            )
        )
        interval_start_time_s = committed_truth_time_s
    return tuple(samples)
    ####


def _reduced_status_values(row: dict[str, float | int | str]) -> dict[str, object]:
    """Add only explicit response-law vectors required by the portable contract."""

    values: dict[str, object] = dict(row)
    attitude_keys = ("route_bank_achieved_deg", "route_pitch_achieved_deg", "route_heading_achieved_deg")
    rate_keys = ("p_rad_s", "q_rad_s", "r_rad_s")
    if all(key in row for key in attitude_keys):
        values["attitude_euler_deg"] = [row[key] for key in attitude_keys]
    if all(key in row for key in rate_keys):
        values["body_rate_rad_s"] = [row[key] for key in rate_keys]
    return values
    ####


def _status_time(row: dict[str, float | int | str]) -> float:
    """Read one finite reduced-model telemetry timestamp."""

    value = row.get("time_s")
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError("reduced fixed-wing status telemetry time_s must be finite numeric")
    return float(value)
    ####


def _row_number(row: Mapping[str, float | int | str], key: str) -> float:
    """Read one finite numeric guidance value without coercing text telemetry."""

    value = row.get(key)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"reduced fixed-wing telemetry {key!r} must be finite numeric")
    return float(value)
    ####


def _canonical_heading_deg(value: float) -> float:
    """Project one periodic heading into the interface's [0, 360) chart.

    This changes representation only: ``-0.1`` and ``359.9`` degrees remain
    the same point on the declared circular value space. It must not be used
    for non-periodic flight-path, bank, or rate channels.
    """

    return value % 360.0
    ####


def _in_range(value: float | int | str, minimum: float, maximum: float, comparison_tolerance: float = 0.0) -> bool:
    """Return whether a telemetry scalar is finite and inside an envelope."""

    if isinstance(value, str):
        return False
    numeric = float(value)
    return math.isfinite(numeric) and minimum - comparison_tolerance <= numeric <= maximum + comparison_tolerance
    ####


def _input_number(inputs: dict[str, Any], name: str, *, default: float | None = None) -> float:
    """Read one canonical numeric composition initialization value."""

    if name not in inputs:
        if default is None:
            raise ValueError(f"composition initialization is missing {name!r}")
        return default
    value = inputs[name].value
    if isinstance(value, bool):
        raise ValueError(f"composition initialization {name!r} must be numeric")
    return float(value)
    ####


def _write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    """Write a stable union of source telemetry channels."""

    fields = sorted({name for row in rows for name in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    ####


def _write_json(path: Path, value: object) -> None:
    """Write deterministic human-readable evidence metadata."""

    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


__all__ = ["ReducedFixedWingCompositionExecution", "execute_reduced_fixed_wing_composition"]
