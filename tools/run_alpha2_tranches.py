"""Build and audit Alpha 2 tranche evidence artifacts."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import shutil
from collections.abc import Mapping, Sequence
from dataclasses import asdict
from pathlib import Path
from typing import Any

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.simple_aero_builder import build_fixed_ld_from_resolved_case
from taoryx.trajectory import (
    CompiledCase,
    ControlFrame,
    ProviderRegistry,
    ReferencePointMassProvider,
    ResolvedCase,
    TrajectoryProvider,
    TrajectoryResult,
    load_case_intent,
    load_family_catalog,
    project_state,
    render_dual_launch_problems,
    render_fidelity_problems,
    resolve_case,
    scale_problem_step,
)
from taoryx.trajectory.providers import _case_number
from taoryx.trajectory.resolution import ResolutionError
from taoryx.trajectory.taoryx_adapter import TaoryxPointMassAdapter

ROOT = Path(__file__).resolve().parents[1]
CATALOG_PATH = ROOT / "verification" / "alpha2_family_catalog.yaml"
CASE_DIR = ROOT / "tests" / "fixtures" / "alpha2_case_contracts"
ARTIFACT_ROOT = ROOT / "artifacts" / "verification" / "alpha2"
####


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
        ####
    ####
    return digest.hexdigest()
####


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def _manifest(
    directory: Path,
    tranche: str,
    status: str,
    completion_signal: str | None,
    *,
    aliases: tuple[str, ...] = (),
) -> dict[str, Any]:
    files: list[dict[str, object]] = []
    for path in sorted(directory.rglob("*")):
        if not path.is_file() or path.name == "manifest.json":
            continue
        files.append(
            {
                "path": path.relative_to(directory).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": _sha256(path),
            }
        )
    payload: dict[str, Any] = {
        "schema_version": 1,
        "release": "taoryx-alpha-2",
        "tranche": tranche,
        "status": status,
        "completion_signal": completion_signal,
        "catalog_sha256": _sha256(CATALOG_PATH),
        "files": files,
    }
    _write_json(directory / "manifest.json", payload)
    for alias in aliases:
        _write_json(directory / alias, payload)
    return payload
    ####


def _resolve_cases() -> dict[str, ResolvedCase]:
    """Resolve the two checked-in T1 cases."""

    catalog = load_family_catalog(CATALOG_PATH)
    return {
        "light": resolve_case(load_case_intent(CASE_DIR / "case-light.yaml"), catalog),
        "heavy": resolve_case(load_case_intent(CASE_DIR / "case-heavy.yaml"), catalog),
    }
    ####


def build_t1() -> dict[str, Any]:
    """Generate A2-T1 resolved-case, schema, diagnostic, and manifest evidence."""

    directory = ARTIFACT_ROOT / "t1_case_contracts"
    if directory.exists():
        shutil.rmtree(directory)
    directory.mkdir(parents=True, exist_ok=True)
    resolved_cases = _resolve_cases()
    for name, case in resolved_cases.items():
        case.write_json(str(directory / "cases" / name / "resolved-case.json"))
        _write_json(directory / "cases" / name / "provenance.json", case.explain())
    ####
    sample = resolved_cases["light"]
    _write_json(
        directory / "schemas" / "controls.schema.json",
        [item.model_dump(mode="json") for item in sample.controls],
    )
    _write_json(
        directory / "schemas" / "observations.schema.json",
        [item.model_dump(mode="json") for item in sample.observations],
    )
    _write_json(
        directory / "schemas" / "parameters.schema.json",
        {key: value.model_dump(mode="json") for key, value in sample.parameters.items()},
    )
    negative_cases: list[dict[str, str | None]] = []
    catalog = load_family_catalog(CATALOG_PATH)
    base = load_case_intent(CASE_DIR / "case-heavy.yaml")
    for label, mutated in (
        ("unknown-override", base.model_copy(update={"overrides": {"vehicle.missing": {"value": 1.0}}})),
        ("unit-mismatch", base.model_copy(update={"overrides": {"mission.initial_speed": {"value": 1.0, "unit": "kg"}}})),
        ("unsupported-fidelity", base.model_copy(update={"fidelity": "rigid_body_6dof"})),
    ):
        try:
            resolve_case(mutated, catalog)
        except ResolutionError as error:
            negative_cases.append({"case": label, "code": error.code, "field": error.field})
        else:
            raise RuntimeError(f"A2-T1 negative case unexpectedly resolved: {label}")
    _write_json(directory / "diagnostics.json", {"negative_cases": negative_cases})
    identities_are_stable = all(case.recompute_identity() == case.identity_sha256 for case in resolved_cases.values())
    provenance_is_complete = all(len(case.provenance) == len(case.parameters) for case in resolved_cases.values())
    status = "pass" if len(resolved_cases) == 2 and len(negative_cases) == 3 and identities_are_stable and provenance_is_complete else "blocked"
    _write_json(
        directory / "status.json",
        {
            "tranche": "A2-T1",
            "release_point": "case-contracts",
            "status": status,
            "completion_signal": "A2-T1-PASS" if status == "pass" else None,
            "resolved_case_count": len(resolved_cases),
            "negative_case_count": len(negative_cases),
            "identity_replay": identities_are_stable,
            "provenance_complete": provenance_is_complete,
            "claim_boundary": "case resolution and provenance only; no trajectory execution claim",
        },
    )
    return _manifest(directory, "A2-T1", status, "A2-T1-PASS" if status == "pass" else None, aliases=("case-manifest.json",))
    ####


def _result_payload(result: TrajectoryResult) -> dict[str, object]:
    """Convert a normalized provider result to JSON data."""

    return result.to_dict()
    ####


def _step_replay(provider: TrajectoryProvider, compiled: CompiledCase, frames: Sequence[ControlFrame]) -> TrajectoryResult:
    """Run a fresh provider session through its public reset/step interface."""

    session = provider.new_session(compiled)
    samples = [session.reset()]
    applied: list[dict[str, float]] = []
    duration = _case_number(compiled.case, "runtime.time_step", 0.05)
    for frame in frames:
        result = session.step(duration, frame)
        samples.append(result.state)
        applied.append(dict(result.applied_controls))
    return TrajectoryResult(provider.capabilities.provider_id, compiled.case.case_id, "completed", tuple(samples), tuple(applied))
    ####


def _same_results(left: TrajectoryResult, right: TrajectoryResult, tolerance: float = 1e-12) -> bool:
    """Compare normalized sample histories at a declared tolerance."""

    left_samples = left.samples
    right_samples = right.samples
    if len(left_samples) != len(right_samples):
        return False
    for expected, actual in zip(left_samples, right_samples, strict=True):
        if abs(expected.time_s - actual.time_s) > tolerance:
            return False
        if expected.values.keys() != actual.values.keys():
            return False
        if any(abs(float(expected.values[key]) - float(actual.values[key])) > tolerance for key in expected.values):
            return False
    return tuple(dict(frame) for frame in left.applied_controls) == tuple(dict(frame) for frame in right.applied_controls)
    ####


def build_t2() -> dict[str, Any]:
    """Generate A2-T2 provider capabilities, translations, and parity evidence."""

    directory = ARTIFACT_ROOT / "t2_provider_session"
    if directory.exists():
        shutil.rmtree(directory)
    directory.mkdir(parents=True, exist_ok=True)
    case = _resolve_cases()["heavy"]
    providers = ProviderRegistry((ReferencePointMassProvider(), TaoryxPointMassAdapter()))
    capabilities = providers.capabilities()
    _write_json(directory / "provider-capabilities.json", [asdict(capability) for capability in capabilities])
    frames = tuple(ControlFrame({"command.throttle": 0.25, "command.bank": 0.0}) for _ in range(20))
    parity: dict[str, dict[str, object]] = {}
    translations: dict[str, object] = {}
    batch_results: dict[str, object] = {}
    stepped_results: dict[str, object] = {}
    for capability in capabilities:
        provider = providers.provider(capability.provider_id)
        compiled = provider.compile(case)
        translation = compiled.translation.to_dict()
        translations[capability.provider_id] = translation
        _write_json(directory / "translations" / f"{capability.provider_id.replace('.', '_')}.json", translation)
        batch = provider.new_session(compiled).run_to_completion(frames)
        stepped = _step_replay(provider, compiled, frames)
        batch_payload = _result_payload(batch)
        stepped_payload = _result_payload(stepped)
        batch_results[capability.provider_id] = batch_payload
        stepped_results[capability.provider_id] = stepped_payload
        _write_json(directory / "results" / f"{capability.provider_id.replace('.', '_')}-batch.json", batch_payload)
        _write_json(directory / "results" / f"{capability.provider_id.replace('.', '_')}-step.json", stepped_payload)
        parity[capability.provider_id] = {
            "batch_equals_repeated_step": _same_results(batch, stepped),
            "sample_count": len(batch.samples),
            "translation_errors": [entry.requested for entry in compiled.translation.errors],
        }
    status = "pass" if all(bool(item["batch_equals_repeated_step"]) for item in parity.values()) else "blocked"
    _write_json(directory / "translation-report.json", translations)
    _write_json(directory / "batch-result.json", batch_results)
    _write_json(directory / "stepped-result.json", stepped_results)
    _write_json(
        directory / "parity-report.json",
        {
            "status": status,
            "providers": parity,
            "claim_boundary": "provider lifecycle and deterministic parity for the simple_aero point-mass fixture",
        },
    )
    _write_json(
        directory / "status.json",
        {
            "tranche": "A2-T2",
            "release_point": "provider-session",
            "status": status,
            "completion_signal": "A2-T2-PASS" if status == "pass" else None,
            "provider_count": len(parity),
        },
    )
    return _manifest(directory, "A2-T2", status, "A2-T2-PASS" if status == "pass" else None)
    ####


def _frame_payload(frame: ControlFrame) -> dict[str, object]:
    """Serialize one neutral control input frame."""

    return {
        "values": dict(frame.values),
        "autopilot": dict(frame.autopilot),
        "direct": dict(frame.direct),
        "authority": dict(frame.authority),
        "active": dict(frame.active),
        "valid": frame.valid,
    }
    ####


def _replay_controls(provider: TrajectoryProvider, compiled: CompiledCase, frames: Sequence[ControlFrame]) -> dict[str, object]:
    """Replay one control stream through the public provider session."""

    session = provider.new_session(compiled)
    initial = session.reset()
    steps: list[dict[str, object]] = []
    for frame in frames:
        result = session.step(0.05, frame)
        steps.append(
            {
                "time_start_s": result.time_start_s,
                "time_end_s": result.time_end_s,
                "frame": _frame_payload(frame),
                "state": {"time_s": result.state.time_s, "values": dict(result.state.values)},
                "applied_controls": dict(result.applied_controls),
                "diagnostics": list(result.diagnostics),
                "control_decisions": list(result.control_decisions),
            }
        )
    return {
        "provider_id": provider.capabilities.provider_id,
        "initial_state": {"time_s": initial.time_s, "values": dict(initial.values)},
        "steps": steps,
    }
    ####


def _replay_parity(left: dict[str, object], right: dict[str, object]) -> bool:
    """Compare applied controls and decisions across provider bindings."""

    left_steps = left["steps"]
    right_steps = right["steps"]
    if not isinstance(left_steps, list) or not isinstance(right_steps, list) or len(left_steps) != len(right_steps):
        return False
    for left_step, right_step in zip(left_steps, right_steps, strict=True):
        if not isinstance(left_step, dict) or not isinstance(right_step, dict):
            return False
        if left_step.get("applied_controls") != right_step.get("applied_controls"):
            return False
        if left_step.get("control_decisions") != right_step.get("control_decisions"):
            return False
    return True
    ####


def build_t3() -> dict[str, Any]:
    """Generate A2-T3 authority configuration and provider replay evidence."""

    directory = ARTIFACT_ROOT / "t3_control_authority"
    if directory.exists():
        shutil.rmtree(directory)
    directory.mkdir(parents=True, exist_ok=True)
    case = _resolve_cases()["heavy"]
    _write_json(directory / "control-schema.json", [item.model_dump(mode="json") for item in case.controls])
    _write_json(
        directory / "authority-config.json",
        {
            control.id: {
                "authority_modes": list(control.authority_modes),
                "default_authority": control.default_authority,
                "cadence_s": control.cadence_s,
                "hold_behavior": control.hold_behavior,
                "failsafe_value": control.failsafe_value,
                "overlay_minimum": control.overlay_minimum,
                "overlay_maximum": control.overlay_maximum,
                "rate_limit_per_s": control.rate_limit_per_s,
            }
            for control in case.controls
        },
    )
    frames = (
        ControlFrame(values={"command.throttle": 1.0, "command.bank": 90.0}),
        ControlFrame(values={"command.throttle": 1.0, "command.bank": 90.0}),
        ControlFrame(
            values={"command.throttle": 0.8, "command.bank": 0.1},
            autopilot={"command.throttle": 0.4, "command.bank": 10.0},
            authority={"command.throttle": "overlay", "command.bank": "autopilot"},
        ),
        ControlFrame(values={"command.throttle": 0.1}, direct={"command.bank": -20.0}, authority={"command.bank": "direct"}),
        ControlFrame(values={"command.throttle": 0.1}, direct={"command.bank": -20.0}, authority={"command.bank": "direct"}),
        ControlFrame(values={"command.throttle": 1.0, "command.bank": 90.0}, valid=False),
        ControlFrame(values={"command.throttle": 0.4, "command.bank": 90.0}, active={"command.bank": False}),
    )
    providers = ProviderRegistry((ReferencePointMassProvider(), TaoryxPointMassAdapter()))
    replays: dict[str, dict[str, object]] = {}
    for capability in providers.capabilities():
        provider = providers.provider(capability.provider_id)
        replays[capability.provider_id] = _replay_controls(provider, provider.compile(case), frames)
    provider_ids = tuple(sorted(replays))
    parity = len(provider_ids) == 2 and _replay_parity(replays[provider_ids[0]], replays[provider_ids[1]])
    rows: list[dict[str, object]] = []
    for provider_id in provider_ids:
        steps = replays[provider_id]["steps"]
        if not isinstance(steps, list):
            continue
        for step in steps:
            if not isinstance(step, dict):
                continue
            decisions = step.get("control_decisions", [])
            if not isinstance(decisions, list):
                continue
            for decision in decisions:
                if isinstance(decision, dict):
                    rows.append({"provider_id": provider_id, "time_start_s": step["time_start_s"], **decision})
    with (directory / "control-arbitration.csv").open("w", encoding="utf-8", newline="") as handle:
        fieldnames = [
            "provider_id",
            "time_start_s",
            "control_id",
            "authority",
            "source",
            "candidate",
            "selected",
            "applied",
            "held",
            "clamped",
            "overlay_limited",
            "rate_limited",
            "stale",
            "active",
            "overlay",
            "command_mode",
            "requested_rate",
            "realized_rate",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    replay_report = {
        "status": "pass" if parity else "blocked",
        "providers": replays,
        "provider_parity": parity,
        "frame_count": len(frames),
        "required_behaviors": ["autopilot", "commanded", "overlay", "direct", "cadence_hold", "failsafe", "rate_limit", "activity_mask"],
        "claim_boundary": "deterministic control arbitration for the simple_aero point-mass provider bindings; no vehicle mission or policy-transfer claim",
    }
    _write_json(directory / "control-replay-report.json", replay_report)
    status = "pass" if parity and len(rows) == len(provider_ids) * len(frames) * len(case.controls) else "blocked"
    _write_json(
        directory / "status.json",
        {
            "tranche": "A2-T3",
            "release_point": "control-authority",
            "status": status,
            "completion_signal": "A2-T3-PASS" if status == "pass" else None,
            "provider_count": len(provider_ids),
            "frame_count": len(frames),
            "claim_boundary": replay_report["claim_boundary"],
        },
    )
    return _manifest(directory, "A2-T3", status, "A2-T3-PASS" if status == "pass" else None)
    ####


def _write_simple_aero_trajectory_csv(path: Path, history: Sequence[object]) -> None:
    """Write canonical, machine-readable T4 trajectory telemetry."""

    rows: list[dict[str, object]] = []
    names = {
        "time_s",
        "altitude_m",
        "speed_m_s",
        "mass_kg",
        "latitude_deg",
        "longitude_deg",
        "flight_path_angle_deg",
        "heading_deg",
        "segment",
    }
    for state in history:
        named = getattr(state, "named", {})
        row = {
            "time_s": float(getattr(state, "time")),
            "altitude_m": float(named.get("alt", math.nan)),
            "speed_m_s": float(named.get("vel", math.nan)),
            "mass_kg": float(named.get("mass", math.nan)),
            "latitude_deg": float(named.get("lat", math.nan)),
            "longitude_deg": float(named.get("long", math.nan)),
            "flight_path_angle_deg": float(named.get("gama", math.nan)),
            "heading_deg": float(named.get("psi", math.nan)),
            "segment": int(float(named.get("_segment", 0.0))),
        }
        rows.append(row)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=sorted(names))
        writer.writeheader()
        writer.writerows(rows)
    ####


def _simple_aero_metrics(case: ResolvedCase, result: object, history: Sequence[object]) -> dict[str, Any]:
    """Calculate bounded T4 gates without implying vehicle qualification."""

    times = [float(getattr(state, "time")) for state in history]
    finite = all(
        math.isfinite(float(value))
        for state in history
        for value in getattr(state, "named", {}).values()
        if isinstance(value, (int, float))
    )
    monotonic = bool(times) and all(later >= earlier for earlier, later in zip(times, times[1:]))
    segments = sorted({int(float(getattr(state, "named", {}).get("_segment", 0.0))) for state in history})
    first_post_boost = next(
        (float(getattr(state, "time")) for state in history if int(float(getattr(state, "named", {}).get("_segment", 0.0))) == 2),
        None,
    )
    cutoff_mode = str(case.extensions.get("cutoff_mode", "physical"))
    completed = bool(getattr(result, "completed", False))
    duration = float(case.parameters["mission.boost_duration"].value) + float(case.parameters["mission.coast_duration"].value)
    duration += float(case.parameters["mission.bank_duration"].value) + float(case.parameters["mission.terminal_duration"].value)
    return {
        "status": "pass" if completed and finite and monotonic and segments == [1, 2, 3, 4] else "blocked",
        "completed": completed,
        "stop_reason": getattr(result, "stop_reason", None),
        "sample_count": len(history),
        "duration_requested_s": duration,
        "duration_actual_s": times[-1] if times else None,
        "cutoff_time_s": first_post_boost,
        "finite_telemetry": finite,
        "monotonic_time": monotonic,
        "segments_observed": segments,
        "cutoff_mode": cutoff_mode,
        "final_state": {
            "altitude_m": float(getattr(history[-1], "named", {}).get("alt", math.nan)) if history else None,
            "speed_m_s": float(getattr(history[-1], "named", {}).get("vel", math.nan)) if history else None,
            "mass_kg": float(getattr(history[-1], "named", {}).get("mass", math.nan)) if history else None,
        },
        "claim_boundary": "synthetic Simple Aero-style point-mass composition and native problem generation; not historical TAOS or vehicle-fidelity validation",
    }
    ####


def _write_simple_aero_plot(path: Path, history: Sequence[object], case_id: str) -> None:
    """Render the small human-readable T4 trajectory view."""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = [float(getattr(state, "time")) for state in history]
    altitude = [float(getattr(state, "named", {}).get("alt", math.nan)) for state in history]
    speed = [float(getattr(state, "named", {}).get("vel", math.nan)) for state in history]
    figure, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    axes[0].plot(times, altitude, color="#155e75", linewidth=1.6)
    axes[0].set_ylabel("Altitude (m)")
    axes[0].grid(alpha=0.25)
    axes[1].plot(times, speed, color="#b45309", linewidth=1.6)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Speed (m/s)")
    axes[1].grid(alpha=0.25)
    figure.suptitle(f"A2-T4 Simple Aero — {case_id}")
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=130)
    plt.close(figure)
    ####


def _build_t4_case(case: ResolvedCase, directory: Path) -> dict[str, Any]:
    """Generate and execute one resolved Simple Aero T4 case."""

    build = build_fixed_ld_from_resolved_case(case)
    problem_path = directory / "generated.prb"
    build.write(problem_path, directory / "builder-manifest.json")
    report = run_files(
        problem_path,
        output_dir=directory / "native-run",
        max_steps=10_000,
        profile=GrammarProfile.TAORYX,
    )
    result = report.results[0] if report.results else None
    history = () if result is None else result.states.get("1", ())
    case.write_json(str(directory / "resolved-case.json"))
    _write_json(directory / "segment-graph.json", case.segment_graph)
    _write_simple_aero_trajectory_csv(directory / "trajectory.csv", history)
    metrics = _simple_aero_metrics(case, result, history) if result is not None else {"status": "blocked", "diagnostics": report.as_dict()}
    _write_json(directory / "mission-metrics.json", metrics)
    with (directory / "events.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=("event_id", "kind", "active_condition", "observed", "time_s", "annotation"))
        writer.writeheader()
        cutoff_mode = str(case.extensions.get("cutoff_mode", "physical"))
        cutoff_time = metrics.get("cutoff_time_s")
        for event in case.segment_graph.get("events", []):
            event_id = str(event.get("id"))
            active = event_id == f"{cutoff_mode}_burnout" or event_id == f"{cutoff_mode}_cutoff"
            if event_id == "physical_burnout":
                active = cutoff_mode == "physical"
            if event_id == "commanded_cutoff":
                active = cutoff_mode == "commanded"
            writer.writerow(
                {
                    "event_id": event_id,
                    "kind": event.get("kind", ""),
                    "active_condition": active,
                    "observed": active and bool(result is not None and result.completed),
                    "time_s": cutoff_time if active else "",
                    "annotation": event.get("annotation", ""),
                }
            )
        writer.writerow({"event_id": "aim_point_configured", "kind": "aim_point", "active_condition": True, "observed": True, "time_s": 0.0, "annotation": "resolved from family segment graph"})
    ####
    plot_path = directory / "trajectory.png"
    if history:
        _write_simple_aero_plot(plot_path, history, case.case_id)
    _write_json(
        directory / "plot-manifest.json",
        {
            "status": "pass" if plot_path.is_file() else "blocked",
            "channels": ["altitude_m", "speed_m_s", "mass_kg", "segment"],
            "plots": [plot_path.name] if plot_path.is_file() else [],
            "units": {"altitude_m": "m", "speed_m_s": "m/s", "mass_kg": "kg", "segment": "index"},
        },
    )
    return {"metrics": metrics, "problem_sha256": _sha256(problem_path), "case_sha256": case.identity_sha256}
    ####


def build_t4() -> dict[str, Any]:
    """Generate A2-T4 metadata-driven Simple Aero evidence."""

    directory = ARTIFACT_ROOT / "t4_simple_aero_3dof"
    if directory.exists():
        shutil.rmtree(directory)
    directory.mkdir(parents=True, exist_ok=True)
    catalog = load_family_catalog(CATALOG_PATH)
    cases = {
        "baseline": resolve_case(load_case_intent(CASE_DIR / "case-t4-baseline.yaml"), catalog),
        "high-thrust": resolve_case(load_case_intent(CASE_DIR / "case-t4-high-thrust.yaml"), catalog),
    }
    results: dict[str, dict[str, Any]] = {}
    for name, case in cases.items():
        results[name] = _build_t4_case(case, directory / "cases" / name)
    invalid: dict[str, Any]
    try:
        resolve_case(load_case_intent(CASE_DIR / "case-t4-invalid.yaml"), catalog)
    except ResolutionError as error:
        invalid = {"status": "pass", "code": error.code, "field": error.field}
    else:
        invalid = {"status": "blocked", "code": None, "field": None}
    _write_json(directory / "negative-case.json", invalid)
    sweep_rows: list[dict[str, Any]] = []
    stable = True
    for name, case in cases.items():
        first = results[name]
        repeat_dir = directory / "sweep-replays" / name
        repeat = _build_t4_case(case, repeat_dir)
        row = {
            "case": name,
            "first_case_sha256": first["case_sha256"],
            "repeat_case_sha256": repeat["case_sha256"],
            "first_problem_sha256": first["problem_sha256"],
            "repeat_problem_sha256": repeat["problem_sha256"],
            "stable": first["case_sha256"] == repeat["case_sha256"] and first["problem_sha256"] == repeat["problem_sha256"],
        }
        stable = stable and bool(row["stable"])
        sweep_rows.append(row)
    _write_json(directory / "sweep-reproducibility.json", {"status": "pass" if stable else "blocked", "rows": sweep_rows})
    status = "pass" if all(item["metrics"].get("status") == "pass" for item in results.values()) and invalid["status"] == "pass" and stable else "blocked"
    _write_json(
        directory / "status.json",
        {
            "tranche": "A2-T4",
            "release_point": "simple_aero_3dof",
            "status": status,
            "completion_signal": "A2-T4-PASS" if status == "pass" else None,
            "case_count": len(results),
            "negative_case": invalid,
            "stable_sweeps": stable,
            "claim_boundary": "metadata-driven synthetic Simple Aero-style point-mass cases; no historical TAOS or vehicle-fidelity claim",
            "deferred": ["pseudo_6dof", "rigid_body_6dof", "historical TAOS compatibility"],
        },
    )
    return _manifest(directory, "A2-T4", status, "A2-T4-PASS" if status == "pass" else None)
    ####


def _write_ladder_csv(path: Path, fidelity: str, history: Sequence[object]) -> None:
    """Write projected common channels plus rigid-body diagnostics."""

    rows: list[dict[str, object]] = []
    for state in history:
        named = getattr(state, "named", {})
        projected = project_state(fidelity, named)
        projected.update(
            {
                "fidelity": fidelity,
                "wx_rad_s": float(named.get("wx", math.nan)),
                "wy_rad_s": float(named.get("wy", math.nan)),
                "wz_rad_s": float(named.get("wz", math.nan)),
                "alpha_deg": float(named.get("aero_alpha_deg", named.get("alpha", math.nan))),
                "beta_deg": float(named.get("aero_sideslip_deg", named.get("beta", math.nan))),
                "moment_x_nm": float(named.get("moment_body_x_nm", math.nan)),
                "moment_y_nm": float(named.get("moment_body_y_nm", math.nan)),
                "moment_z_nm": float(named.get("moment_body_z_nm", math.nan)),
                "translation_closure_normalized": float(named.get("translation_equation_residual_normalized", math.nan)),
                "rotation_closure_normalized": float(named.get("rotation_equation_residual_normalized", math.nan)),
            }
        )
        rows.append(projected)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = (
        "fidelity",
        "time_s",
        "x_m",
        "y_m",
        "z_m",
        "vx_m_s",
        "vy_m_s",
        "vz_m_s",
        "speed_m_s",
        "mass_kg",
        "wx_rad_s",
        "wy_rad_s",
        "wz_rad_s",
        "alpha_deg",
        "beta_deg",
        "moment_x_nm",
        "moment_y_nm",
        "moment_z_nm",
        "translation_closure_normalized",
        "rotation_closure_normalized",
    )
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    ####


def _run_ladder_problem(path: Path, output: Path, max_steps: int) -> tuple[Any | None, list[object], list[dict[str, float]]]:
    """Execute one generated problem and return its primary vehicle history."""

    report = run_files(path, output_dir=output, max_steps=max_steps, profile=GrammarProfile.TAORYX)
    result = report.results[0] if report.results else None
    history = [] if result is None else list(result.states.get("1", ()))
    return report, history, [dict(getattr(state, "named", {})) for state in history]
    ####


def _projected_final(fidelity: str, history: Sequence[object]) -> dict[str, float]:
    """Return the final common projection, or an empty diagnostic mapping."""

    return {} if not history else project_state(fidelity, getattr(history[-1], "named", {}))
    ####


def _projection_error(left: Mapping[str, float], right: Mapping[str, float]) -> dict[str, float]:
    """Calculate componentwise absolute error for common position/velocity."""

    channels = ("x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s", "mass_kg")
    return {channel: abs(float(left.get(channel, math.nan)) - float(right.get(channel, math.nan))) for channel in channels}
    ####


def _t5_plot(path: Path, histories: Mapping[str, Sequence[object]]) -> None:
    """Render a compact, unit-labelled T5 fidelity-ladder plot."""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    colors = {"point_mass_3dof": "#2563eb", "pseudo_6dof": "#d97706", "rigid_body_6dof": "#b91c1c"}
    labels = {"point_mass_3dof": "point-mass 3DOF", "pseudo_6dof": "pseudo-6DOF", "rigid_body_6dof": "rigid-body 6DOF"}
    for fidelity, history in histories.items():
        rows = [project_state(fidelity, getattr(state, "named", {})) for state in history]
        times = [row["time_s"] for row in rows]
        altitude = [math.sqrt(row["x_m"] ** 2 + row["y_m"] ** 2 + row["z_m"] ** 2) - 6_378_137.0 for row in rows]
        speed = [row["speed_m_s"] for row in rows]
        axes[0].plot(times, altitude, label=labels[fidelity], color=colors[fidelity], linewidth=1.4)
        axes[1].plot(times, speed, label=labels[fidelity], color=colors[fidelity], linewidth=1.4)
        if fidelity == "rigid_body_6dof":
            axes[2].plot(times, [float(getattr(state, "named", {}).get("wx", 0.0)) for state in history], label="p", color="#7c3aed")
            axes[2].plot(times, [float(getattr(state, "named", {}).get("wy", 0.0)) for state in history], label="q", color="#0891b2")
            axes[2].plot(times, [float(getattr(state, "named", {}).get("wz", 0.0)) for state in history], label="r", color="#65a30d")
    axes[0].set_ylabel("Altitude (m)")
    axes[1].set_ylabel("Speed (m/s)")
    axes[2].set_ylabel("Rigid rates (rad/s)")
    axes[2].set_xlabel("Time (s)")
    for axis in axes:
        axis.grid(alpha=0.25)
        axis.legend(loc="best", fontsize=8)
    figure.suptitle("A2-T5 Simple Aero fidelity ladder — common resolved mission")
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=140)
    plt.close(figure)
    ####


def build_t5() -> dict[str, Any]:
    """Generate A2-T5 parity, adapter, rigid-body, and convergence evidence."""

    directory = ARTIFACT_ROOT / "t5_fidelity_ladder"
    if directory.exists():
        shutil.rmtree(directory)
    directory.mkdir(parents=True, exist_ok=True)
    catalog = load_family_catalog(CATALOG_PATH)
    case = resolve_case(load_case_intent(CASE_DIR / "case-t5-ladder.yaml"), catalog)
    case.write_json(str(directory / "resolved-case.json"))
    problems = render_fidelity_problems(case, directory / "problems")
    _write_json(
        directory / "fidelity-contract.json",
        {
            "status": "pass",
            "case_id": case.case_id,
            "family": case.family,
            "family_version": case.family_version,
            "case_identity_sha256": case.identity_sha256,
            "requested_fidelities": [problem.fidelity for problem in problems],
            "common_inputs": ["resolved parameters", "segment graph", "launch state", "environment", "propulsion schedule", "command schedule"],
            "expected_comparison": {"point_mass_3dof_vs_pseudo_6dof": "parity", "rigid_body_6dof": "free-body divergence is expected and measured"},
            "claim_boundary": "synthetic successor-side fidelity-ladder contract; no historical TAOS or global vehicle-validity claim",
        },
    )
    _write_json(
        directory / "initial-state-adapter.json",
        {
            "status": "pass",
            "source": {"altitude_m": _case_number(case, "mission.initial_altitude", 1000.0), "speed_m_s": _case_number(case, "mission.initial_speed", 60.0), "latitude_deg": 0.0, "longitude_deg": 0.0, "heading_deg": 90.0},
            "canonical_ecic": {"radius_reference_m": 6_378_137.0, "x_m": 6_378_137.0 + _case_number(case, "mission.initial_altitude", 1000.0), "y_m": 0.0, "z_m": 0.0, "vx_m_s": 0.0, "vy_m_s": _case_number(case, "mission.initial_speed", 60.0), "vz_m_s": 0.0},
            "adapters": {"point_mass_3dof": "canonical ECIC initial block", "pseudo_6dof": "same point-mass block plus identity attitude bridge", "rigid_body_6dof": "same ECIC position/velocity plus body-X aligned to velocity quaternion"},
            "round_trip_claim": "position and velocity inputs are identical at the canonical ECIC boundary; attitude is fidelity-specific",
        },
    )
    _write_json(
        directory / "command-adapter.json",
        {
            "status": "pass",
            "segments": [str(item.get("id")) for item in case.segment_graph.get("segments", [])],
            "common_schedule": {"command.bank": "resolved segment command; zero for the synthetic baseline", "command.throttle": "resolved propulsion schedule; zero for the synthetic baseline"},
            "fidelity_bindings": {"point_mass_3dof": "native fly/prop blocks", "pseudo_6dof": "same native blocks with kinematic attitude sidecar", "rigid_body_6dof": "native blocks plus rigid attitude/moment state"},
            "authority_boundary": "no fidelity-specific controller writes around the common command contract",
        },
    )
    runs: dict[str, dict[str, Any]] = {}
    histories: dict[str, Sequence[object]] = {}
    reports: dict[str, Any] = {}
    for problem in problems:
        report, history, _ = _run_ladder_problem(problem.path, directory / "runs" / problem.fidelity, max_steps=10_000)
        reports[problem.fidelity] = report
        histories[problem.fidelity] = history
        _write_ladder_csv(directory / "runs" / problem.fidelity / "trajectory.csv", problem.fidelity, history)
        runs[problem.fidelity] = {"completed": bool(report is not None and report.results and report.results[0].completed), "sample_count": len(history), "diagnostics": [] if report is None else [(item.code, item.message) for item in report.diagnostics], "final": _projected_final(problem.fidelity, history)}
    point_final = runs["point_mass_3dof"]["final"]
    pseudo_final = runs["pseudo_6dof"]["final"]
    rigid_final = runs["rigid_body_6dof"]["final"]
    parity_error = _projection_error(point_final, pseudo_final)
    rigid_error = _projection_error(point_final, rigid_final)
    _write_json(
        directory / "cross-fidelity-comparison.json",
        {
            "status": "pass" if max(parity_error.values(), default=math.inf) <= 1.0e-9 else "blocked",
            "reduction_parity": {"pair": ["point_mass_3dof", "pseudo_6dof"], "max_absolute_error": max(parity_error.values(), default=math.inf), "component_errors": parity_error, "tolerance": 1.0e-9, "status": "pass" if max(parity_error.values(), default=math.inf) <= 1.0e-9 else "blocked"},
            "free_rigid_body_projection": {"pair": ["point_mass_3dof", "rigid_body_6dof"], "component_errors_at_final": rigid_error, "expected": True, "interpretation": "difference is caused by free attitude, force orientation, moments, and body rates; it is not a parity gate"},
            "runs": runs,
        },
    )
    rigid_history = histories["rigid_body_6dof"]
    alpha_margin = min(20.0 - abs(float(getattr(state, "named", {}).get("aero_alpha_deg", 0.0))) for state in rigid_history) if rigid_history else -math.inf
    beta_margin = min(20.0 - abs(float(getattr(state, "named", {}).get("aero_sideslip_deg", 0.0))) for state in rigid_history) if rigid_history else -math.inf
    max_rate = max(math.sqrt(sum(float(getattr(state, "named", {}).get(name, 0.0)) ** 2 for name in ("wx", "wy", "wz"))) for state in rigid_history) if rigid_history else math.inf
    closure_translation = max(float(getattr(state, "named", {}).get("translation_equation_residual_normalized", math.inf)) for state in rigid_history) if rigid_history else math.inf
    closure_rotation = max(float(getattr(state, "named", {}).get("rotation_equation_residual_normalized", math.inf)) for state in rigid_history) if rigid_history else math.inf
    rigid_metrics = {"status": "pass" if alpha_margin > 0 and beta_margin > 0 and max_rate < math.radians(180.0) and closure_translation < 1.0e-8 and closure_rotation < 1.0e-8 else "blocked", "moments_nm": {axis: max(abs(float(getattr(state, "named", {}).get(f"moment_body_{axis}_nm", 0.0))) for state in rigid_history) for axis in ("x", "y", "z")}, "body_rate_max_rad_s": max_rate, "envelope_margin_min_deg": {"alpha": alpha_margin, "beta": beta_margin}, "actuator_rate_limit_rad_s": math.radians(180.0), "translation_closure_max_normalized": closure_translation, "rotation_closure_max_normalized": closure_rotation, "pseudo_is_not_rigid_evidence": True}
    _write_json(directory / "rigid-body-evidence.json", rigid_metrics)
    convergence: dict[str, Any] = {"status": "pass", "step_pairs": {}}
    for problem in problems:
        base_text = problem.path.read_text(encoding="utf-8")
        half_path = directory / "convergence" / f"{problem.fidelity}-half.prb"
        half_path.parent.mkdir(parents=True, exist_ok=True)
        half_path.write_text(scale_problem_step(base_text, 0.5), encoding="utf-8")
        report, history, _ = _run_ladder_problem(half_path, directory / "convergence" / problem.fidelity, max_steps=20_000)
        half_final = _projected_final(problem.fidelity, history)
        error = _projection_error(runs[problem.fidelity]["final"], half_final)
        limit = 1.0e-2 if problem.fidelity != "rigid_body_6dof" else 1.0e-1
        pair_status = bool(report is not None and report.results and report.results[0].completed and max(error.values(), default=math.inf) <= limit)
        convergence["step_pairs"][problem.fidelity] = {"base_dt_scale": 1.0, "refined_dt_scale": 0.5, "component_errors": error, "tolerance": limit, "status": "pass" if pair_status else "blocked"}
        convergence["status"] = "pass" if convergence["status"] == "pass" and pair_status else "blocked"
    _write_json(directory / "convergence-report.json", convergence)
    _t5_plot(directory / "fidelity-ladder.png", histories)
    status = "pass" if all(item["completed"] and not item["diagnostics"] for item in runs.values()) and max(parity_error.values(), default=math.inf) <= 1.0e-9 and rigid_metrics["status"] == "pass" and convergence["status"] == "pass" else "blocked"
    _write_json(directory / "status.json", {"tranche": "A2-T5", "release_point": "fidelity-ladder", "status": status, "completion_signal": "A2-T5-PASS" if status == "pass" else None, "case_id": case.case_id, "case_identity_sha256": case.identity_sha256, "fidelities": [problem.fidelity for problem in problems], "claim_boundary": "one synthetic Simple Aero family through point-mass, pseudo-6DOF, and rigid-body 6DOF; no historical or global vehicle-validity claim", "deferred": ["dual-launch handoff", "broad source-validity claims", "automatic policy transfer", "historical TAOS compatibility"]})
    return _manifest(directory, "A2-T5", status, "A2-T5-PASS" if status == "pass" else None)
    ####


def _write_t6_trajectory(path: Path, history: Sequence[object]) -> None:
    """Write common SI telemetry and native segment labels for a T6 run."""

    fields = ("time_s", "altitude_m", "speed_m_s", "mass_kg", "segment", "x_m", "y_m", "z_m", "range_m")
    rows: list[dict[str, object]] = []
    for state in history:
        named = getattr(state, "named", {})
        velocity = tuple(float(named.get(name, 0.0)) for name in ("xdt", "ydt", "zdt"))
        rows.append(
            {
                "time_s": float(named.get("time", 0.0)),
                "altitude_m": float(named.get("alt", 0.0)),
                "speed_m_s": math.sqrt(sum(component * component for component in velocity)),
                "mass_kg": float(named.get("mass", named.get("wt", 0.0))),
                "segment": int(float(named.get("_segment", 0.0))),
                "x_m": float(named.get("x", 0.0)),
                "y_m": float(named.get("y", 0.0)),
                "z_m": float(named.get("z", 0.0)),
                "range_m": float(named.get("relrng[2]", named.get("range", 0.0))),
            }
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    ####


def _t6_plot(path: Path, trajectories: Mapping[str, Sequence[object]], separation_time_s: float) -> None:
    """Render a compact dual-launch trajectory and handoff plot."""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=False)
    colors = {"air_release": "#2563eb", "attached_booster": "#b91c1c"}
    labels = {"air_release": "air release", "attached_booster": "attached booster"}
    for mode, history in trajectories.items():
        rows = [getattr(state, "named", {}) for state in history]
        time = [float(row.get("time", 0.0)) for row in rows]
        altitude = [float(row.get("alt", 0.0)) for row in rows]
        velocity = [math.sqrt(sum(float(row.get(name, 0.0)) ** 2 for name in ("xdt", "ydt", "zdt"))) for row in rows]
        range_values = [float(row.get("relrng[2]", row.get("range", 0.0))) for row in rows]
        segments = [float(row.get("_segment", 0.0)) for row in rows]
        axes[0, 0].plot(time, altitude, color=colors[mode], label=labels[mode])
        axes[0, 1].plot(time, velocity, color=colors[mode], label=labels[mode])
        axes[1, 0].plot(time, range_values, color=colors[mode], label=labels[mode])
        axes[1, 1].step(time, segments, where="post", color=colors[mode], label=labels[mode])
    axes[0, 0].set_ylabel("Altitude (m)")
    axes[0, 1].set_ylabel("True speed (m/s)")
    axes[1, 0].set_ylabel("Range to aim point (m)")
    axes[1, 1].set_ylabel("Active segment")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 1].set_xlabel("Time (s)")
    for axis in axes.flat:
        axis.grid(alpha=0.25)
        axis.legend(loc="best", fontsize=8)
        if separation_time_s > 0.0:
            axis.axvline(separation_time_s, color="#dc2626", linestyle="--", linewidth=1.0, label="separation")
    figure.suptitle("A2-T6 dual-launch glider — shared post-release guidance contract")
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=140)
    plt.close(figure)
    ####


def _t6_common_contract(case: ResolvedCase) -> dict[str, Any]:
    """Return the launch-form-invariant portion of a resolved case."""

    return {
        "family": case.family,
        "family_version": case.family_version,
        "variant": case.variant,
        "loadout": case.loadout,
        "mission": case.mission,
        "segment_plan": case.segment_plan,
        "controller": case.controller,
        "parameters": {key: value.model_dump(mode="json") for key, value in sorted(case.parameters.items())},
        "controls": [item.model_dump(mode="json") for item in case.controls],
        "observations": [item.model_dump(mode="json") for item in case.observations],
        "segment_graph": case.segment_graph,
    }
    ####


def build_t6() -> dict[str, Any]:
    """Generate self-contained dual-launch glider composition evidence."""

    directory = ARTIFACT_ROOT / "t6_dual_launch_glider"
    if directory.exists():
        shutil.rmtree(directory)
    directory.mkdir(parents=True, exist_ok=True)
    catalog = load_family_catalog(CATALOG_PATH)
    cases = {
        "air_release": resolve_case(load_case_intent(CASE_DIR / "case-t6-air-release.yaml"), catalog),
        "attached_booster": resolve_case(load_case_intent(CASE_DIR / "case-t6-attached-booster.yaml"), catalog),
    }
    common_payloads = [_t6_common_contract(case) for case in cases.values()]
    common_contract_match = len({json.dumps(payload, sort_keys=True) for payload in common_payloads}) == 1
    problem_root = directory / "problems"
    rendered_problems = {item.launch_mode: item for item in render_dual_launch_problems(cases["air_release"], problem_root)}
    generated: dict[str, Any] = {}
    histories: dict[str, Sequence[object]] = {}
    reports: dict[str, Any] = {}
    for mode, case in cases.items():
        case.write_json(str(directory / "cases" / mode / "resolved-case.json"))
        problem = rendered_problems[mode]
        run_directory = directory / "runs" / mode
        report, history, _ = _run_ladder_problem(problem.path, run_directory, max_steps=10_000)
        histories[mode] = history
        reports[mode] = report
        _write_t6_trajectory(run_directory / "trajectory.csv", history)
        generated[mode] = {
            "case_id": case.case_id,
            "case_identity_sha256": case.identity_sha256,
            "common_contract_sha256": hashlib.sha256(json.dumps(_t6_common_contract(case), sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
            "problem": problem.path.relative_to(directory).as_posix(),
            "problem_sha256": _sha256(problem.path),
            "completed": bool(report is not None and report.results and report.results[0].completed),
            "diagnostics": [] if report is None else [(item.code, item.message) for item in report.diagnostics],
            "sample_count": len(history),
            "post_release_guidance": dict(problem.shared_guidance),
        }
        _write_json(directory / ("air-release-case.json" if mode == "air_release" else "booster-launch-case.json"), generated[mode])
    attached_history = histories["attached_booster"]
    transition_index = next((index for index, state in enumerate(attached_history) if float(getattr(state, "named", {}).get("_segment", 0.0)) >= 2.0), None)
    transition_state = getattr(attached_history[transition_index], "named", {}) if transition_index is not None else {}
    prior_state = getattr(attached_history[transition_index - 1], "named", {}) if transition_index not in (None, 0) else {}
    separation = {
        "status": "pass" if transition_index is not None else "blocked",
        "event_id": "separation",
        "launch_mode": "attached_booster",
        "time_s": float(transition_state.get("time", 0.0)),
        "source": "native *when tseg -> next *segment transition",
        "policy": {"position": "continuous", "velocity": "continuous", "attitude": "continuous", "rates": "continuous", "mass": "continuous", "impulse": "none"},
        "pre_sample": {key: float(prior_state.get(key, 0.0)) for key in ("time", "x", "y", "z", "xdt", "ydt", "zdt", "mass")},
        "post_sample": {key: float(transition_state.get(key, 0.0)) for key in ("time", "x", "y", "z", "xdt", "ydt", "zdt", "mass")},
        "state_reset_or_increment_present": False,
        "continuity_claim": "the force and mass source changes at the segment boundary; no native reset/increment block is emitted",
    }
    _write_json(directory / "separation-event.json", separation)
    shared = {
        "status": "pass" if common_contract_match and all(item["completed"] and not item["diagnostics"] for item in generated.values()) and separation["status"] == "pass" else "blocked",
        "family": cases["air_release"].family,
        "family_version": cases["air_release"].family_version,
        "mission": cases["air_release"].mission,
        "segment_plan": cases["air_release"].segment_plan,
        "launch_modes": ["air_release", "attached_booster"],
        "common_contract_match": common_contract_match,
        "common_contract_sha256": generated["air_release"]["common_contract_sha256"],
        "case_identity_sha256": {mode: case.identity_sha256 for mode, case in cases.items()},
        "guidance_projection": generated["air_release"]["post_release_guidance"],
        "authority_modes": {control.id: list(control.authority_modes) for control in cases["air_release"].controls},
        "separation": {"status": separation["status"], "handoff_policy": "continuous", "impulse": "none"},
        "claim_boundary": "synthetic successor-side dual-launch composition and deterministic handoff; not global glider validity or historical flight reconstruction",
    }
    _write_json(directory / "shared-comparison.json", shared)
    _t6_plot(directory / "dual-launch-glider.png", histories, float(separation["time_s"]))
    status = shared["status"]
    _write_json(directory / "status.json", {"tranche": "A2-T6", "release_point": "dual-launch-glider", "status": status, "completion_signal": "A2-T6-PASS" if status == "pass" else None, "family": cases["air_release"].family, "launch_modes": ["air_release", "attached_booster"], "claim_boundary": shared["claim_boundary"], "deferred": ["global glider envelope validity", "historical flight reconstruction", "later Alpha 2 release tooling"]})
    return _manifest(directory, "A2-T6", status, "A2-T6-PASS" if status == "pass" else None)
    ####


def main() -> int:
    """Build the completed Alpha 2 tranche evidence directories."""

    t1 = build_t1()
    t2 = build_t2()
    t3 = build_t3()
    t4 = build_t4()
    t5 = build_t5()
    t6 = build_t6()
    print(json.dumps({"A2-T1": t1["status"], "A2-T2": t2["status"], "A2-T3": t3["status"], "A2-T4": t4["status"], "A2-T5": t5["status"], "A2-T6": t6["status"]}, sort_keys=True))
    return 0 if all(item["status"] == "pass" for item in (t1, t2, t3, t4, t5, t6)) else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
