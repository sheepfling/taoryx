"""Build a hash-bound evidence packet for the four-family fidelity ladder."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import uuid
import zipfile
from pathlib import Path
from typing import Any, Literal, cast

import yaml

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.objectives import ObjectiveSpec, score_objectives
from taoryx.runtime.runner import run_files
from taoryx.scenario_contract import ScenarioContract, compare_contracts
from taoryx.trajectory import EvaluationMetric, EvidenceChannel, objective_report_to_evaluation
from taoryx.trajectory.evaluation import OutcomeStatus, ValidityStatus
from taoryx.validation import independent_force_closure, independent_moment_closure
from taoryx.visualization import render_run_artifact_plots

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "verification/fidelity_ladder.yaml"
LONG_CONFIG = ROOT / "verification/long_validation_trajectories.yaml"
CONTROLLER_MISSION_CONFIG = ROOT / "verification/controller_missions.yaml"
CLAIM_INPUTS = (
    ROOT / "verification/claims.md",
    ROOT / "verification/controller_scenarios.yaml",
    ROOT / "verification/family_validation_execution.yaml",
    ROOT / "verification/vehicle_models.yaml",
    ROOT / "verification/staged_completion_matrix.yaml",
    ROOT / "verification/fidelity_parity.yaml",
    LONG_CONFIG,
    CONTROLLER_MISSION_CONFIG,
    ROOT / "verification/acceptance/README.md",
    ROOT / "verification/acceptance/robustness_matrix_v1.yaml",
)
CLOSURE_METRIC_DICTIONARY = {
    "schema_version": 1,
    "metrics": {
        "rhs_algebraic_translation": {
            "channel": "translation_equation_residual_normalized",
            "equation": "m*(v_dot_body + omega_body cross v_body) - F_total_body",
            "frame": "body",
            "normalization": "max(m*g, norm(F_total))",
            "sampling": "integrator RHS at every saved sample",
            "events": "runtime event samples retained and separately marked",
        },
        "rhs_algebraic_rotation": {
            "channel": "rotation_equation_residual_normalized",
            "equation": "I*omega_dot + omega cross (I*omega) - M_total",
            "frame": "body",
            "normalization": "max(1 N m, norm(M_total))",
            "sampling": "integrator RHS at every saved sample",
            "events": "runtime event samples retained and separately marked",
        },
        "independent_finite_difference_translation": {
            "summary_field": "independent_closure.independent_translation",
            "equation": "finite-difference saved velocity plus rotating-frame correction minus force sum",
            "frame": "body/ECIC adapter declared by the source telemetry contract",
            "normalization": "family closure contract",
            "sampling": "smooth interior samples only",
            "events": "declared event times excluded from p99 and max statistics",
        },
        "independent_finite_difference_rotation": {
            "summary_field": "independent_closure.independent_rotation",
            "equation": "finite-difference angular rate plus Euler moment balance",
            "frame": "body",
            "normalization": "family closure contract",
            "sampling": "smooth interior samples only",
            "events": "declared event times excluded from p99 and max statistics",
        },
        "active_waypoint_error": {
            "channel": "route_target_error_m",
            "definition": "distance from the vehicle to the currently active waypoint",
            "units": "m",
            "not_final_route_error": True,
        },
        "route_cross_track_error": {
            "channel": "route_cross_track_error_m",
            "definition": "signed lateral distance from the active route leg",
            "units": "m",
        },
        "route_along_track_error": {
            "channel": "route_along_track_error_m",
            "definition": "signed distance along the active route tangent to the reference point",
            "units": "m",
        },
        "route_leg_index": {
            "channel": "route_leg_index",
            "definition": "zero-based active waypoint/leg index",
            "units": "index",
        },
        "route_heading_error": {
            "channel": "route_heading_error_deg",
            "definition": "local heading error relative to the active route leg",
            "units": "deg",
        },
        "route_bank_tracking_error": {
            "channel": "route_bank_tracking_error_deg",
            "definition": "commanded route bank minus achieved local roll",
            "units": "deg",
        },
        "command_achieved_error": {
            "definition": "commanded actuator or guidance value minus achieved value",
            "units": "channel-specific; source telemetry names carry units",
            "required_channels": [
                "*_command_*",
                "*_achieved_*",
                "*_saturated",
            ],
        },
    },
}
SCORE_DEFINITION = {
    "schema_version": 1,
    "gate_status": "required objectives must pass; blocked required objectives prevent a pass",
    "quality_score": {
        "formula": "100 * weighted_mean(max(0, 1 - quality_normalized_error))",
        "quality_limit": "declared desired-performance boundary, distinct from the hard gate target/tolerance",
        "interpretation": "a result near a hard failure limit may pass the gate but receives a low quality score",
        "advisory_objectives": "reported separately and never override required gate status",
    },
    "trajectory_resource_metrics": {
        "table_margin_min_normalized": "minimum distance to any queried table boundary divided by that axis span; dimensionless",
        "table_margin_average_normalized": "sample mean of the normalized table margin; dimensionless",
        "table_margin_min_absolute": "minimum raw table margin in mixed source-axis units; diagnostic only",
        "table_margin_average_absolute": "sample mean of raw table margins in mixed source-axis units; diagnostic only",
        "control_saturation_fraction": "fraction of saved samples with any saturation flag active; dimensionless",
        "control_saturation_average": "sample mean of aggregate saturation flags; dimensionless",
        "control_saturation_max_abs": "maximum absolute aggregate saturation flag; dimensionless",
        "control_derivative_abs_average": "mean absolute actuator-command derivative; declared command unit per second",
        "control_derivative_abs_max": "maximum absolute actuator-command derivative; declared command unit per second",
        "per_control_breakdown": "retained in trajectory_rollup.control_derivative_abs_*_by_channel",
    },
    "legacy_score": "gate_compliance_legacy is retained only when any objective lacks quality_limit",
}
CLOSURE_CONTRACT = {
    "b747": {"independent_force_p99_max": 1.0e-4, "independent_moment_p99_max": 1.0e-3, "test": "tests/e2e/test_vehicle_family_validation.py"},
    "skywalker_x8": {"independent_force_p99_max": 5.0e-3, "independent_moment_p99_max": 5.0e-3, "test": "tests/e2e/test_x8_family_validation.py"},
    "hummingbird": {"independent_force_p99_max": 1.0e-6, "independent_moment_p99_max": 1.0e-6, "test": "tests/e2e/test_hummingbird_family_validation.py"},
    "x15": {"independent_force_p99_max": 1.0e-3, "independent_moment_p99_max": 2.0e-2, "test": "tests/e2e/test_glider_family_validation.py"},
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
    ####


def _payload_sha256(payload: object) -> str:
    """Hash canonical JSON metadata for a scenario contract field."""

    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
    ####


def _scenario_contract(
    case: dict[str, Any],
    tier: str,
    source_problem: Path,
    tables: tuple[Path, ...],
    summary: dict[str, Any],
    *,
    unit_system: str,
    base_problem: Path,
) -> ScenarioContract | None:
    """Build a contract from the resolved packet inputs and initial audit."""

    metrics = dict(summary.get("telemetry_metrics", {}))
    duration = metrics.get("duration_s")
    initial = dict(summary.get("initial_condition_audit", {}).get("channels", {}))
    if duration is None or not initial:
        return None
    table_hashes = tuple(_sha256(path) for path in tables)
    physical_initial = {
        key: value
        for key, value in initial.items()
        if key not in {"qw", "qx", "qy", "qz", "local_roll_deg", "local_pitch_deg", "local_heading_deg"}
    }
    # Contracts describe declared experiment inputs, not incidental runtime
    # telemetry such as bridge-generated attitude status events.
    event_hash = _payload_sha256({"source_problem": _sha256(base_problem)})
    termination_hash = _payload_sha256({"source_problem": _sha256(base_problem)})
    initial_hash = _payload_sha256(physical_initial)
    physical_problem = base_problem if tier in {"3dof", "pseudo_6dof"} else source_problem
    model_hash = _payload_sha256({"problem": _sha256(physical_problem), "tables": table_hashes})
    shared_model_hash = _payload_sha256({"tables": table_hashes})
    return ScenarioContract(
        scenario_id=str(case["id"]),
        vehicle=str(case["id"]),
        family=str(case["display_name"]),
        dynamics_tier=cast(Literal["3dof", "pseudo_6dof", "6dof"], tier),
        initial_state_sha256=initial_hash,
        environment_sha256=_payload_sha256({"unit_system": unit_system, "source": _sha256(base_problem)}),
        vehicle_model_sha256=model_hash,
        propulsion_model_sha256=shared_model_hash,
        mass_model_sha256=_sha256(base_problem),
        command_history_sha256=_sha256(base_problem),
        event_schedule_sha256=event_hash,
        termination_policy_sha256=termination_hash,
        duration_s=float(duration),
        unit_system="si",
        integrator="rk4",
        output_rate_hz=1.0,
    )
    ####


def _files(root: Path) -> tuple[Path, ...]:
    return tuple(path for path in sorted(root.rglob("*")) if path.is_file())
    ####


def _relativeize(value: Any) -> Any:
    """Remove workstation-specific absolute paths from packet metadata."""

    if isinstance(value, dict):
        return {key: _relativeize(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_relativeize(item) for item in value]
    if isinstance(value, str):
        try:
            path = Path(value)
        except (TypeError, ValueError):
            return value
        if path.is_absolute():
            try:
                return path.relative_to(ROOT).as_posix()
            except ValueError:
                return path.name
    return value
    ####


def _write_telemetry_csv(output: Path, histories: list[Any]) -> str | None:
    """Persist all numeric state channels, including command/actuator channels."""

    if not histories:
        return None
    names = sorted({name for state in histories for name, value in state.named.items() if isinstance(value, (int, float))})
    path = output / "telemetry.csv"
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["time_s", *names])
        writer.writeheader()
        for state in histories:
            row: dict[str, object] = {"time_s": state.time}
            row.update({name: state.named.get(name) for name in names})
            writer.writerow(row)
    return path.name
    ####


def _write_bundle_metadata(packet: Path, *, reproduction_command: str | None = None) -> None:
    """Write reproducibility metadata that travels with the packet."""

    evidence = packet / "evidence"
    evidence.mkdir(parents=True, exist_ok=True)
    (evidence / "metric_dictionary.json").write_text(
        json.dumps(CLOSURE_METRIC_DICTIONARY, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (evidence / "score_definition.json").write_text(
        json.dumps(SCORE_DEFINITION, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, encoding="utf-8").strip()
    except (OSError, subprocess.CalledProcessError):
        commit = "unavailable"
    (packet / "software_commit.txt").write_text(f"git_commit={commit}\n", encoding="utf-8")
    try:
        diff = subprocess.check_output(["git", "diff", "--binary", "HEAD"], cwd=ROOT, text=True, encoding="utf-8")
        status = subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True, encoding="utf-8")
    except (OSError, subprocess.CalledProcessError):
        diff = "# working tree diff unavailable\n"
        status = "working tree status unavailable\n"
    (packet / "working_tree.patch").write_text(diff, encoding="utf-8")
    (packet / "working_tree.status").write_text(status, encoding="utf-8")
    untracked_root = packet / "working_tree_untracked"
    untracked_manifest: list[dict[str, str]] = []
    try:
        raw_untracked = subprocess.check_output(
            ["git", "ls-files", "--others", "--exclude-standard", "-z"],
            cwd=ROOT,
            stderr=subprocess.DEVNULL,
        )
        untracked_paths = tuple(Path(item) for item in raw_untracked.decode("utf-8").split("\0") if item)
    except (OSError, subprocess.CalledProcessError, UnicodeError):
        untracked_paths = ()
    for relative in untracked_paths:
        source = ROOT / relative
        if not source.is_file():
            continue
        destination = untracked_root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        untracked_manifest.append(
            {
                "path": relative.as_posix(),
                "packet_path": destination.relative_to(packet).as_posix(),
                "sha256": _sha256(source),
            }
        )
    (packet / "working_tree.untracked.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "base_commit": commit,
                "files": untracked_manifest,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    (packet / "dependency.lock").write_text(
        "# Reproducibility profile captured from the packet-producing environment.\n"
        f"python_version=3.12\npyproject_sha256={_sha256(ROOT / 'pyproject.toml')}\n"
        + _pip_freeze(),
        encoding="utf-8",
    )
    reproduce = packet / "reproduce.sh"
    command = reproduction_command or "python tools/build_fidelity_ladder_packet.py --output artifacts/verification/fidelity_ladder"
    reproduce.write_text(
        "#!/bin/sh\n"
        "set -eu\n"
        "# Run from the TAORYX repository checkout that produced this packet.\n"
        f"{command}\n",
        encoding="utf-8",
    )
    reproduce.chmod(0o755)
    ####


def _pip_freeze() -> str:
    """Return installed package versions without making the packet depend on pip."""

    try:
        output = subprocess.check_output(
            [str(ROOT / ".venv/bin/python"), "-m", "pip", "freeze", "--disable-pip-version-check"],
            cwd=ROOT,
            text=True,
            stderr=subprocess.STDOUT,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        return f"pip_freeze=unavailable:{type(error).__name__}\n"
    return "pip_freeze_begin\n" + output + "pip_freeze_end\n"
    ####


def _kinematic_problem(source: Path, destination: Path) -> Path:
    """Derive the bridge problem from the checked-in 3-DOF source problem."""

    lines = source.read_text(encoding="utf-8").splitlines()
    title_index = next(index for index, line in enumerate(lines) if line.startswith("*title"))
    lines[title_index + 1:title_index + 1] = [
        "*mode kinematic-6dof",
        "*runtime status attitude mode=lag roll-deg=0 pitch-deg=0 yaw-deg=0 lag-s=0.25 max-rate-deg-s=360",
    ]
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return destination
    ####


def _scaled_problem(source: Path, destination: Path, factor: float) -> Path:
    """Create a deterministic fixed-step refinement of a problem file."""

    if factor <= 0.0:
        raise ValueError("step refinement factor must be positive")
    text = source.read_text(encoding="utf-8")
    text = re.sub(
        r"(\bdt=)([0-9.eE+-]+)",
        lambda match: f"{match.group(1)}{float(match.group(2)) * factor:.16g}",
        text,
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(text, encoding="utf-8")
    return destination
    ####


def _timed_problem(source: Path, destination: Path, duration_s: float) -> Path:
    """Create a bounded convergence window without changing the source case."""

    if duration_s <= 0.0:
        raise ValueError("convergence window must be positive")
    text = source.read_text(encoding="utf-8")
    text, replacements = re.subn(
        r"\*when time>[0-9.eE+-]+ stop",
        f"*when time>{duration_s:.16g} stop",
        text,
    )
    if replacements != 1:
        raise ValueError(f"expected one time stop condition in {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(text, encoding="utf-8")
    return destination
    ####


def _load_cases() -> tuple[dict[str, Any], ...]:
    payload = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    return tuple(dict(item) for item in payload["families"])
    ####


def _load_long_cases() -> tuple[dict[str, Any], ...]:
    """Load the long-trajectory evidence catalog."""

    payload = yaml.safe_load(LONG_CONFIG.read_text(encoding="utf-8"))
    return tuple(dict(item) for item in payload["vehicles"])
    ####


def _load_controller_missions() -> tuple[dict[str, Any], ...]:
    """Load generic controller-mission contracts from catalog metadata."""

    payload = yaml.safe_load(CONTROLLER_MISSION_CONFIG.read_text(encoding="utf-8"))
    return tuple(dict(item) for item in payload["missions"])
    ####


def _family_key(identifier: str) -> str:
    """Normalize catalog identifiers used by the ladder and long-run catalogs."""

    return identifier.casefold().replace("-", "_")
    ####


def _event_times(report: Any) -> tuple[float, ...]:
    """Return declared runtime event times for discontinuity-aware closure."""

    values: list[float] = []
    for artifact in report.artifacts:
        for raw_event in artifact.events:
            event: Any = raw_event
            if isinstance(event, dict) and "time" in event:
                values.append(float(event["time"]))
    return tuple(values)
    ####


def _telemetry_metrics(report: Any) -> dict[str, Any]:
    """Summarize review-critical telemetry without replacing raw artifacts."""

    histories = [
        state
        for result in report.results
        for states in result.states.values()
        for state in states
    ]
    if not histories:
        return {"sample_count": 0, "duration_s": None, "final": {}, "max": {}, "max_abs": {}}

    numeric_channels: dict[str, list[float]] = {}
    for state in histories:
        for name, value in state.named.items():
            if isinstance(value, (int, float)) and math.isfinite(float(value)):
                numeric_channels.setdefault(name, []).append(float(value))

    def numeric_series(channel: str) -> list[float]:
        """Return a cached finite numeric channel without rescanning history."""

        return numeric_channels.get(channel, [])

    def is_control_channel(channel: str) -> bool:
        """Identify actuator commands, excluding guidance/reference telemetry."""

        lowered = channel.casefold()
        if lowered in {"throttle", "rotor_speed"} or lowered.startswith("rotor-") and lowered.endswith("-speed"):
            return True
        return any(token in lowered for token in ("elevator-deg", "aileron-deg", "rudder-deg", "elevon-deg", "stabilator-deg"))

    saturation_channels = sorted(
        {
            name
            for state in histories
            for name in state.named
            if name.casefold().endswith("_saturated") and isinstance(state.named[name], (int, float))
        }
    )
    saturation_values = [
        max((float(state.named[name]) for name in saturation_channels if name in state.named), default=0.0)
        for state in histories
    ]
    control_channels = sorted(
        {name for state in histories for name in state.named if is_control_channel(name) and numeric_series(name)}
    )
    control_derivatives: dict[str, list[float]] = {name: [] for name in control_channels}
    for previous, current in zip(histories, histories[1:], strict=False):
        dt = float(current.time - previous.time)
        if dt <= 0.0:
            continue
        for name in control_channels:
            if name not in previous.named or name not in current.named:
                continue
            previous_value = float(previous.named[name])
            current_value = float(current.named[name])
            if math.isfinite(previous_value) and math.isfinite(current_value):
                control_derivatives[name].append(abs(current_value - previous_value) / dt)
    all_control_derivatives = [value for values in control_derivatives.values() for value in values]
    normalized_margin_values = numeric_series("aero_table_min_normalized_margin")
    absolute_margin_values = numeric_series("aero_table_min_margin")
    telemetry_rollup = {
        "table_margin_min_normalized": min(normalized_margin_values, default=None),
        "table_margin_average_normalized": (
            sum(normalized_margin_values) / len(normalized_margin_values) if normalized_margin_values else None
        ),
        "table_margin_min_absolute": min(absolute_margin_values, default=None),
        "table_margin_average_absolute": (
            sum(absolute_margin_values) / len(absolute_margin_values) if absolute_margin_values else None
        ),
        "control_saturation_fraction": (
            sum(value > 0.5 for value in saturation_values) / len(saturation_values) if saturation_values else 0.0
        ),
        "control_saturation_average": (
            sum(saturation_values) / len(saturation_values) if saturation_values else 0.0
        ),
        "control_saturation_max_abs": max((abs(value) for value in saturation_values), default=0.0),
        "control_derivative_abs_average": (
            sum(all_control_derivatives) / len(all_control_derivatives) if all_control_derivatives else None
        ),
        "control_derivative_abs_max": max(all_control_derivatives, default=None),
        "control_channels": control_channels,
        "control_derivative_abs_max_by_channel": {
            name: max(values) for name, values in control_derivatives.items() if values
        },
        "control_derivative_abs_average_by_channel": {
            name: sum(values) / len(values) for name, values in control_derivatives.items() if values
        },
        "saturation_channels": saturation_channels,
    }
    known_channels = (
        "altitude_m",
        "speed_m_s",
        "mach",
        "aero_alpha_deg",
        "aero_sideslip_deg",
        "wx",
        "wy",
        "wz",
        "attitude_controller_saturated",
        "pro_nav_active",
        "route_target_error_m",
        "range_to_target_m",
        "motor_shutdown",
    )
    final_state = histories[-1]
    all_channels = sorted(numeric_channels)
    final = {
        channel: float(final_state.named[channel])
        for channel in known_channels
        if channel in final_state.named
    }
    maximums: dict[str, float] = {}
    maxima: dict[str, float] = {}
    minimums: dict[str, float] = {}
    for channel in all_channels:
        values = numeric_channels[channel]
        if values:
            maximums[channel] = max(abs(value) for value in values)
            maxima[channel] = max(values)
            minimums[channel] = min(values)
    final.update({channel: float(final_state.named[channel]) for channel in all_channels if channel in final_state.named and channel not in final})
    return {
        "sample_count": len(histories),
        "duration_s": float(final_state.time - histories[0].time),
        "final": final,
        "max": maxima,
        "max_abs": maximums,
        "min": minimums,
        "channel_names": all_channels,
        "channel_statistics": {
            channel: {
                "final": final.get(channel),
                "minimum": minimums.get(channel),
                "maximum": maxima.get(channel),
                "maximum_absolute": maximums.get(channel),
            }
            for channel in all_channels
        },
        "trajectory_rollup": telemetry_rollup,
        **telemetry_rollup,
    }
    ####


def _initial_condition_audit(report: Any) -> dict[str, Any]:
    """Capture the first resolved state used by a run in packet-friendly form."""

    if not report.results:
        return {"status": "unavailable", "reason": "run returned no results"}
    states = next(iter(report.results[0].states.values()), ())
    if not states:
        return {"status": "unavailable", "reason": "run returned no states"}
    state = states[0]
    channels = (
        "x",
        "y",
        "z",
        "xdt",
        "ydt",
        "zdt",
        "altitude_m",
        "speed_m_s",
        "mass_kg",
        "aero_alpha_deg",
        "aero_sideslip_deg",
        "qw",
        "qx",
        "qy",
        "qz",
    )
    return {
        "status": "available",
        "time_s": float(state.time),
        "channels": {
            name: float(state.named[name])
            for name in channels
            if name in state.named and float(state.named[name]) == float(state.named[name])
        },
    }
    ####


def _event_timeline(report: Any) -> list[dict[str, Any]]:
    """Serialize runtime events without losing their event kind or source data."""

    events: list[dict[str, Any]] = []
    for artifact in report.artifacts:
        for raw_event in artifact.events:
            if hasattr(raw_event, "model_dump"):
                value = raw_event.model_dump(mode="json")
            elif isinstance(raw_event, dict):
                value = dict(raw_event)
            else:
                value = {"value": str(raw_event)}
            events.append(value)
    return events
    ####


def _event_continuity_audit(events: list[dict[str, Any]]) -> dict[str, Any]:
    """Audit runtime event records without treating every event as harmless."""

    if not events:
        return {
            "status": "not_run",
            "event_count": 0,
            "reason": "runtime emitted no event records",
        }
    missing_state = [
        str(event.get("name", "unnamed"))
        for event in events
        if "state_discontinuity" not in event
    ]
    discontinuities = [
        {
            "name": event.get("name"),
            "time": event.get("time"),
            "action": event.get("action"),
            "reason": "runtime event declared a physical state discontinuity",
            "source": event.get("source"),
        }
        for event in events
        if bool(event.get("state_discontinuity"))
    ]
    if missing_state:
        status = "blocked"
    elif discontinuities:
        status = "diagnostic"
    else:
        status = "pass"
    return {
        "status": status,
        "event_count": len(events),
        "missing_state_discontinuity_field": missing_state,
        "declared_discontinuities": discontinuities,
        "unannotated_discontinuities": [],
        "claim_boundary": "runtime event annotation audit; channel-level policy audit remains required for declared jumps",
    }
    ####


def _expectation_evaluation(expected: dict[str, Any], metrics: dict[str, Any]) -> dict[str, Any]:
    """Evaluate catalog expectations against the packet's summarized telemetry.

    This is deliberately a reporting layer: runtime execution remains responsible
    for producing the raw history, while this function makes the catalog contract
    auditable in the evidence packet itself.
    """

    checks: list[dict[str, Any]] = []
    final = dict(metrics.get("final", {}))
    maxima = dict(metrics.get("max", {}))
    max_abs = dict(metrics.get("max_abs", {}))

    def check(name: str, actual: Any, comparator: str, limit: float, passed: bool) -> None:
        checks.append(
            {
                "name": name,
                "actual": actual,
                "comparator": comparator,
                "limit": limit,
                "passed": bool(passed),
            }
        )

    if "min_duration_s" in expected:
        actual = metrics.get("duration_s")
        limit = float(expected["min_duration_s"])
        check("min_duration_s", actual, ">=", limit, actual is not None and float(actual) + 1.0e-12 >= limit)
    for name, channel in (("max_alpha_deg", "aero_alpha_deg"), ("max_beta_deg", "aero_sideslip_deg")):
        if name in expected:
            actual = max_abs.get(channel)
            limit = float(expected[name])
            check(name, actual, "<=", limit, actual is not None and float(actual) <= limit)
    if "max_body_rate_deg_s" in expected:
        actual = max(max_abs.get(channel, 0.0) for channel in ("wx", "wy", "wz")) * 180.0 / 3.141592653589793
        limit = float(expected["max_body_rate_deg_s"])
        check(name="max_body_rate_deg_s", actual=actual, comparator="<=", limit=limit, passed=actual <= limit)
    for name, channel in (("max_altitude_m", "altitude_m"), ("max_speed_m_s", "speed_m_s")):
        if name in expected:
            actual = maxima.get(channel)
            limit = float(expected[name])
            check(name, actual, "<=", limit, actual is not None and float(actual) <= limit)
    for name, channel in (("final_range_m", "range_to_target_m"), ("final_altitude_m", "altitude_m")):
        if name in expected:
            actual = final.get(channel)
            limit = float(expected[name])
            check(name, actual, "<=", limit, actual is not None and float(actual) <= limit)
    if "require_guidance" in expected:
        actual = max_abs.get("pro_nav_active", 0.0)
        required = float(expected["require_guidance"]) > 0.0
        check("require_guidance", actual, ">= 1", 1.0, (actual >= 1.0) == required)
    if "require_unsaturated" in expected:
        actual = max_abs.get("attitude_controller_saturated", 0.0)
        required = float(expected["require_unsaturated"]) > 0.0
        check("require_unsaturated", actual, "== 0", 0.0, (actual == 0.0) == required)
    if "require_motor_shutdown" in expected:
        actual = final.get("motor_shutdown")
        required = float(expected["require_motor_shutdown"]) > 0.0
        check("require_motor_shutdown", actual, "== 1", 1.0, actual is not None and (float(actual) >= 1.0) == required)
    return {
        "status": "pass" if all(bool(item["passed"]) for item in checks) else "fail",
        "check_count": len(checks),
        "checks": checks,
    }
    ####


def _objective_evaluation(declarations: tuple[dict[str, Any], ...], summary: dict[str, Any]) -> dict[str, Any]:
    """Score external objective declarations against one packet summary."""

    if not declarations:
        return {"status": "not_run", "reason": "no objective declarations"}
    try:
        specs = tuple(ObjectiveSpec(**declaration) for declaration in declarations)
    except (TypeError, ValueError) as error:
        return {"status": "blocked", "reason": f"invalid objective declaration: {error}"}
    metrics = dict(summary.get("telemetry_metrics", {}))
    observed: dict[str, object] = {}
    for spec in specs:
        if spec.source == "summary":
            observed[spec.channel] = summary.get(spec.channel, metrics.get(spec.channel))
        else:
            source_values = dict(metrics.get(spec.source, {}))
            observed[spec.channel] = source_values.get(spec.channel)
    events: set[str] = set()
    for event in summary.get("event_timeline", ()):
        if isinstance(event, dict):
            for key in ("id", "name", "signal", "action"):
                value = event.get(key)
                if isinstance(value, str):
                    events.add(value)
    return score_objectives(
        specs,
        observed,
        completed_events=events,
        termination={
            "reason": summary.get("results", [{}])[0].get("stop_reason") if summary.get("results") else None,
            "completed": all(bool(result.get("completed")) for result in summary.get("results", ())),
        },
        scenario_contract_sha256=(
            str(summary.get("scenario_contract", {}).get("contract_sha256"))
            if isinstance(summary.get("scenario_contract"), dict)
            and summary["scenario_contract"].get("contract_sha256")
            else None
        ),
    )
    ####


def _long_convergence_report(
    problem: Path,
    tables: tuple[Path, ...],
    output: Path,
    max_steps: int,
    specification: dict[str, Any],
) -> dict[str, Any]:
    """Run fixed-step refinements for a long rigid-body evidence case."""

    factors = tuple(float(value) for value in specification.get("factors", (1.0, 0.5, 0.25)))
    if not factors or factors[0] != 1.0:
        raise ValueError("long-run convergence factors must begin with 1.0")
    window_s = float(specification.get("window_s", 0.0))
    source_problem = problem
    if window_s > 0.0:
        source_problem = _timed_problem(problem, output / "inputs" / "convergence-window.prb", window_s)
    runs: list[dict[str, Any]] = []
    for factor in factors:
        refined_problem = _scaled_problem(
            source_problem,
            output / "inputs" / f"dt-{factor:g}.prb",
            factor,
        )
        runs.append(
            _run_case(
                refined_problem,
                tables,
                output / "runs" / f"dt-{factor:g}",
                max_steps * max(1, int(round(1.0 / factor))),
                plots=False,
            )
        )
    channels = tuple(str(channel) for channel in specification.get("channels", ("altitude_m", "speed_m_s")))
    tolerances = {str(key): float(value) for key, value in dict(specification.get("tolerances", {})).items()}
    comparisons: list[dict[str, Any]] = []
    for left, right in zip(runs, runs[1:], strict=False):
        left_final = dict(left.get("telemetry_metrics", {}).get("final", {}))
        right_final = dict(right.get("telemetry_metrics", {}).get("final", {}))
        differences = {
            channel: abs(float(left_final[channel]) - float(right_final[channel]))
            for channel in channels
            if channel in left_final and channel in right_final
        }
        comparisons.append(
            {
                "left_factor": factors[len(comparisons)],
                "right_factor": factors[len(comparisons) + 1],
                "final_difference": differences,
                "gate": {
                    channel: {
                        "actual": differences.get(channel),
                        "limit": tolerances.get(channel),
                        "passed": channel in differences
                        and channel in tolerances
                        and differences[channel] <= tolerances[channel],
                    }
                    for channel in channels
                },
            }
        )
    complete = all(
        bool(item.get("results")) and all(bool(result.get("completed")) for result in item["results"])
        for item in runs
    )
    gate_passed = all(
        bool(check["passed"])
        for comparison in comparisons
        for check in comparison["gate"].values()
    )
    return {
        "status": "pass" if complete and gate_passed else "fail",
        "factors": list(factors),
        "window_s": window_s if window_s > 0.0 else None,
        "channels": list(channels),
        "tolerances": tolerances,
        "runs": runs,
        "comparisons": comparisons,
    }
    ####


def _parity_convergence_report(
    family_id: str,
    specification: dict[str, Any],
    *,
    report_path: Path | None = None,
) -> dict[str, Any]:
    """Project the existing catalog-driven parity report into a family packet."""

    source_report = report_path or ROOT / "artifacts/verification/fidelity_parity_v1/report.json"
    if not source_report.exists():
        return {
            "status": "unavailable",
            "evidence_source": "fidelity-parity-report.json",
            "reason": "parity report is not present; run tools/run_fidelity_parity.py",
        }
    payload = json.loads(source_report.read_text(encoding="utf-8"))
    record = next(
        (item for item in payload.get("families", ()) if _family_key(str(item.get("id"))) == _family_key(family_id)),
        None,
    )
    if record is None:
        return {"status": "unavailable", "evidence_source": "fidelity-parity-report.json", "reason": "family is absent"}
    six_dof = dict(record.get("runs", {}).get("6dof", {}))
    completed = bool(six_dof.get("completed"))
    parity_passed = str(record.get("parity_gate", "")).startswith("pass_")
    return {
        "status": "pass" if completed and parity_passed else "fail",
        "evidence_source": "evidence/fidelity-parity-report.json",
        "window_s": specification.get("window_s"),
        "parity_gate": record.get("parity_gate"),
        "rigid_window_max_difference": record.get("rigid_window_max_difference"),
        "convergence": record.get("convergence"),
        "claim_boundary": "fixed-step parity-window convergence; not adaptive reference convergence",
    }
    ####


def _closure_evaluation(family_id: str, summary: dict[str, Any]) -> dict[str, Any]:
    """Evaluate independent closure residuals against the family contract."""

    contract = CLOSURE_CONTRACT[family_id]
    closure = summary.get("independent_closure")
    if not isinstance(closure, dict) or "independent_translation" not in closure:
        return {"status": "unavailable", "reason": "independent closure was not available for this tier"}
    translation = dict(closure["independent_translation"])
    rotation = dict(closure.get("independent_rotation", {}))
    checks = {
        "translation_p99": {
            "actual": translation.get("p99_normalized_residual"),
            "limit": contract["independent_force_p99_max"],
        },
        "rotation_p99": {
            "actual": rotation.get("p99_normalized_residual"),
            "limit": contract["independent_moment_p99_max"],
        },
    }
    for check in checks.values():
        actual = check["actual"]
        check["passed"] = actual is not None and float(cast(float, actual)) <= float(cast(float, check["limit"]))
    return {
        "status": "pass" if all(bool(check["passed"]) for check in checks.values()) else "fail",
        "checks": checks,
        "contract_test": contract["test"],
    }
    ####


def _channel_unit(channel: str) -> str | None:
    """Infer only units encoded unambiguously in a telemetry channel name."""

    lowered = channel.casefold()
    for token, unit in (
        ("rad_s2", "rad/s^2"),
        ("rad_s", "rad/s"),
        ("m_s2", "m/s^2"),
        ("m_s", "m/s"),
        ("deg_s", "deg/s"),
        ("deg", "deg"),
        ("_nm", "N m"),
        ("_n", "N"),
        ("_pa", "Pa"),
        ("_kg", "kg"),
        ("_m", "m"),
        ("_s", "s"),
    ):
        if lowered.endswith(token) or token in lowered:
            return unit
    if lowered.endswith("_active") or lowered.endswith("_saturated"):
        return "1"
    return None
    ####


def _telemetry_evidence_channels(summary: dict[str, Any]) -> tuple[tuple[EvidenceChannel, ...], tuple[EvidenceChannel, ...], tuple[EvidenceChannel, ...]]:
    """Extract final requested, achieved, and resource channels with units."""

    metrics = dict(summary.get("telemetry_metrics", {}))
    final = dict(metrics.get("final", {}))
    time_s = metrics.get("duration_s")
    controls = set(str(name) for name in metrics.get("trajectory_rollup", {}).get("control_channels", ()))
    requested: list[EvidenceChannel] = []
    achieved: list[EvidenceChannel] = []
    for name, value in final.items():
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            continue
        lowered = name.casefold()
        source: Literal["requested", "achieved"] | None = None
        target: list[EvidenceChannel] | None = requested if "command" in lowered or lowered.endswith("_request") else achieved if name in controls or "achieved" in lowered else None
        if target is requested:
            source = "requested"
        elif target is achieved:
            source = "achieved"
        if source is None or target is None:
            continue
        unit = _channel_unit(name)
        target.append(
            EvidenceChannel(
                id=f"{source}:{name}",
                value=float(value) if unit is not None else None,
                unit=unit,
                source=source,
                status="available" if unit is not None else "unavailable",
                time_s=float(time_s) if isinstance(time_s, (int, float)) else None,
                provenance=f"telemetry.final:{name}",
            )
        )
    resource_names = {
        "mass_kg",
        "fuel_remaining_kg",
        "propellant_remaining_kg",
        "battery_soc",
        "time_to_burnout_s",
        "burnout_time_available_s",
    }
    resources: list[EvidenceChannel] = []
    for name in sorted(resource_names & set(final)):
        value = final[name]
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            continue
        unit = "1" if name == "battery_soc" else _channel_unit(name)
        resources.append(
            EvidenceChannel(
                id=f"resource:{name}",
                value=float(value),
                unit=unit,
                source="resource",
                time_s=float(time_s) if isinstance(time_s, (int, float)) else None,
                provenance=f"telemetry.final:{name}",
            )
        )
    return tuple(requested), tuple(achieved), tuple(resources)
    ####


def _event_evidence_channels(summary: dict[str, Any]) -> tuple[EvidenceChannel, ...]:
    """Represent runtime events as explicit boolean evidence channels."""

    channels: list[EvidenceChannel] = []
    for index, event in enumerate(summary.get("event_timeline", ())):
        if not isinstance(event, dict):
            continue
        name = str(event.get("name") or event.get("id") or event.get("action") or f"event-{index}")
        raw_time = event.get("time")
        time_s = float(raw_time) if isinstance(raw_time, (int, float)) else None
        channels.append(
            EvidenceChannel(
                id=f"event:{index}:{name}",
                value=True,
                source="event",
                time_s=time_s,
                provenance="runtime.event_timeline",
            )
        )
    return tuple(channels)
    ####


def _closure_metrics(summary: dict[str, Any]) -> tuple[EvaluationMetric, ...]:
    """Convert the independent closure gate into required evidence metrics."""

    evaluation = summary.get("closure_evaluation")
    if not isinstance(evaluation, dict) or evaluation.get("status") not in {"pass", "fail"}:
        return (
            EvaluationMetric(
                id="closure-availability",
                actual=None,
                target=1.0,
                tolerance=0.5,
                unit="1",
                status="blocked",
                source="independent-closure",
            ),
        )
    metrics: list[EvaluationMetric] = []
    for name, check in dict(evaluation.get("checks", {})).items():
        if not isinstance(check, dict):
            continue
        raw_actual = check.get("actual")
        raw_limit = check.get("limit")
        actual = float(raw_actual) if isinstance(raw_actual, (int, float)) else None
        limit = float(raw_limit) if isinstance(raw_limit, (int, float)) else None
        available = actual is not None and limit is not None
        metrics.append(
            EvaluationMetric(
                id=f"closure-{name}",
                actual=actual,
                target=0.0,
                tolerance=limit if limit is not None else 1.0,
                slack=limit - actual if available and limit is not None and actual is not None else None,
                normalized_error=actual / limit if available and limit is not None and limit > 0.0 and actual is not None else None,
                unit="1",
                status="pass" if check.get("passed") else "fail" if available else "blocked",
                source="independent-closure",
            )
        )
    return tuple(metrics)
    ####


def _convergence_metrics(report: Any) -> tuple[EvaluationMetric, ...]:
    """Represent a convergence report as one explicit required gate."""

    status = report.get("status") if isinstance(report, dict) else None
    if status not in {"pass", "fail"}:
        return (
            EvaluationMetric(
                id="convergence-availability",
                actual=None,
                target=1.0,
                tolerance=0.5,
                unit="1",
                status="blocked",
                source="convergence-report",
            ),
        )
    passed = status == "pass"
    return (
        EvaluationMetric(
            id="convergence-gate",
            actual=1.0 if passed else 0.0,
            target=1.0,
            tolerance=0.5,
            slack=0.5 if passed else -0.5,
            normalized_error=0.0 if passed else 2.0,
            unit="1",
            status="pass" if passed else "fail",
            source="convergence-report",
        ),
    )
    ####


def _trajectory_evaluation(
    summary: dict[str, Any],
    *,
    scenario_id: str,
    claim_boundary: str,
    objective_report: dict[str, Any] | None = None,
    convergence_report: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build the neutral evaluation envelope from an existing packet summary."""

    results = tuple(item for item in summary.get("results", ()) if isinstance(item, dict))
    outcome: OutcomeStatus
    validity: ValidityStatus
    if not results:
        outcome = "not_run"
        validity = "not_run"
    elif int(summary.get("exit_code", 1)) != 0:
        outcome = "numerical_failure"
        validity = "invalid"
    elif all(bool(item.get("completed")) for item in results):
        outcome = "completed"
        validity = "valid"
    else:
        outcome = "time_limited"
        validity = "valid"
    closure = _closure_metrics(summary)
    if any(metric.status == "fail" for metric in closure):
        validity = "invalid"
    elif any(metric.status == "blocked" for metric in closure) and validity == "valid":
        validity = "not_run"
    requested, achieved, resources = _telemetry_evidence_channels(summary)
    contract = summary.get("scenario_contract")
    contract_hash = contract.get("contract_sha256") if isinstance(contract, dict) else None
    report = objective_report or {"status": "not_run", "objectives": ()}
    evaluation = objective_report_to_evaluation(
        report,
        scenario_id=scenario_id,
        scenario_contract_sha256=str(contract_hash) if contract_hash else None,
        validity=validity,
        qualification="extended",
        feasibility="unknown",
        outcome=outcome,
        requested_controls=requested,
        achieved_controls=achieved,
        resources=resources,
        events=_event_evidence_channels(summary),
        closure=closure,
        convergence=_convergence_metrics(convergence_report),
        claim_boundary=claim_boundary,
    )
    return evaluation.as_dict()
    ####


PLOT_CHANNELS = (
    "taos.altitude_m",
    "taos.speed_m_s",
    "taos.mach",
    "taos.aero_alpha_deg",
    "taos.aero_sideslip_deg",
    "taos.local_roll_deg",
    "taos.local_pitch_deg",
    "taos.local_heading_deg",
    "taos.route_bank_command_deg",
    "taos.route_bank_achieved_deg",
    "taos.route_target_error_m",
    "taos.route_leg_index",
    "taos.route_cross_track_error_m",
    "taos.route_along_track_error_m",
    "taos.route_heading_error_deg",
    "taos.route_bank_tracking_error_deg",
    "taos.route_phase_index",
    "taos.route_corner_0_error_m",
    "taos.route_corner_1_error_m",
    "taos.route_corner_2_error_m",
    "taos.route_corner_3_error_m",
    "taos.force_body_x_n",
    "taos.force_body_y_n",
    "taos.force_body_z_n",
    "taos.moment_body_x_nm",
    "taos.moment_body_y_nm",
    "taos.moment_body_z_nm",
    "taos.alpha_command_deg",
    "taos.bank_command_deg",
    "taos.pro_nav_command_ecfc_x_m_s2",
    "taos.pro_nav_command_ecfc_y_m_s2",
    "taos.pro_nav_command_ecfc_z_m_s2",
    "taos.pro_nav_acceleration_response_residual_m_s2",
    "taos.elevator-deg",
    "taos.rudder-deg",
    "taos.collective-elevon-deg",
    "taos.differential-elevon-deg",
    "taos.symmetric-stabilator-deg",
    "taos.differential-stabilator-deg",
    "taos.rotor_command_saturated",
    "taos.aero_query_rotor-1-speed",
    "taos.aero_query_rotor-2-speed",
    "taos.aero_query_rotor-3-speed",
    "taos.aero_query_rotor-4-speed",
    "taos.translation_equation_residual_normalized",
    "taos.rotation_equation_residual_normalized",
)


def _run_case(
    problem: Path,
    tables: tuple[Path, ...],
    output: Path,
    max_steps: int,
    *,
    plots: bool = False,
) -> dict[str, Any]:
    report = run_files(problem, tables, output_dir=output, max_steps=max_steps, integrator="rk4", profile=GrammarProfile.TAORYX)
    results = []
    telemetry_histories: list[Any] = []
    for result in report.results:
        states = next(iter(result.states.values()), ())
        telemetry_histories.extend(states)
        final = states[-1] if states else None
        results.append(
            {
                "completed": result.completed,
                "stop_reason": result.stop_reason,
                "state_count": len(states),
                "final_time_s": None if final is None else final.time,
                "dynamics": None if not report.artifacts else report.artifacts[0].vehicles["1"].dynamics.value,
            }
        )
    plot_paths: tuple[Path, ...] = ()
    if plots and report.artifacts:
        plot_paths = render_run_artifact_plots(
            report.artifacts[0],
            output.parent / "plots",
            vehicle_id="1",
            channels=PLOT_CHANNELS,
        )
    closure: dict[str, Any] | None = None
    event_times = _event_times(report)
    if report.results and report.results[0].states:
        states = next(iter(report.results[0].states.values()))
        history = tuple({"time_s": state.time, **dict(state.named)} for state in states)
        required_force = {
            "mass_kg",
            "xdt",
            "ydt",
            "zdt",
            "total_force_ecic_x_n",
            "total_force_ecic_y_n",
            "total_force_ecic_z_n",
        }
        required_moment = {
            "wx",
            "wy",
            "wz",
            "total_moment_body_x_nm",
            "total_moment_body_y_nm",
            "total_moment_body_z_nm",
        }
        if history and required_force.issubset(history[0]) and required_moment.issubset(history[0]):
            metadata = report.metadata[0] if report.metadata else {}
            vehicle_value = metadata.get("vehicle", {}) if isinstance(metadata, dict) else {}
            vehicle: dict[str, Any] = dict(vehicle_value) if isinstance(vehicle_value, dict) else {}
            inertia = tuple(
                float(vehicle[name])
                for name in ("inertia-x", "inertia-y", "inertia-z")
                if name in vehicle
            )
            if len(inertia) == 3:
                enriched = tuple(
                    {
                        **sample,
                        "inertia_x_kg_m2": inertia[0],
                        "inertia_y_kg_m2": inertia[1],
                        "inertia_z_kg_m2": inertia[2],
                    }
                    for sample in history
                )
                try:
                    closure = {
                        "independent_translation": independent_force_closure(
                            enriched,
                            event_times=event_times,
                        ),
                        "independent_rotation": independent_moment_closure(enriched, event_times=event_times),
                    }
                except (ValueError, AssertionError) as error:
                    closure = {"status": "unavailable", "reason": str(error)}
    event_timeline = _event_timeline(report)
    telemetry_artifact = _write_telemetry_csv(output, telemetry_histories)
    event_timeline = _relativeize(_event_timeline(report))
    return {
        "exit_code": report.exit_code,
        "diagnostics": [{"code": item.code, "message": item.message} for item in report.diagnostics],
        "results": results,
        "plots": [path.relative_to(output.parent).as_posix() for path in plot_paths],
        "initial_condition_audit": _initial_condition_audit(report),
        "event_timeline": event_timeline,
        "event_continuity_audit": _event_continuity_audit(event_timeline),
        "telemetry_metrics": _telemetry_metrics(report),
        "telemetry_artifact": telemetry_artifact,
        "independent_closure": closure,
    }
    ####


def build(
    output: Path,
    *,
    plots: bool = False,
    family_ids: frozenset[str] | None = None,
    controller_ids: frozenset[str] | None = None,
    include_long_validation: bool = True,
    include_controller_missions: bool = True,
) -> Path:
    """Build and return the UUID-named ZIP packet."""

    if plots:
        mpl_config = output / ".mplconfig"
        mpl_config.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault("MPLCONFIGDIR", str(mpl_config))
    run_id = str(uuid.uuid4())
    packet = output / run_id
    packet.mkdir(parents=True, exist_ok=False)
    selected_family_args = "" if family_ids is None else " ".join(
        f"--family {family}" for family in sorted(family_ids)
    )
    selected_controller_args = "" if controller_ids is None else " ".join(
        f"--controller {controller}" for controller in sorted(controller_ids)
    )
    reproduction_command = " ".join(
        item
        for item in (
            "python tools/build_fidelity_ladder_packet.py --output artifacts/verification/fidelity_ladder",
            selected_family_args,
            selected_controller_args,
            "--skip-long-validation" if not include_long_validation else "",
            "--skip-controller-missions" if not include_controller_missions else "",
        )
        if item
    )
    _write_bundle_metadata(packet, reproduction_command=reproduction_command)
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "run_id": run_id,
        "source_config": CONFIG.relative_to(ROOT).as_posix(),
        "long_validation_config": LONG_CONFIG.relative_to(ROOT).as_posix(),
        "controller_mission_config": CONTROLLER_MISSION_CONFIG.relative_to(ROOT).as_posix(),
        "claim_boundary": "source-bounded research-surrogate evidence; not flight qualification",
        "tiers": ["point-mass-3dof", "kinematic-3-plus-3-dof", "rigid-body-6dof"],
        "families": [],
        "claim_inputs": [path.relative_to(ROOT).as_posix() for path in CLAIM_INPUTS],
        "closure_contract": CLOSURE_CONTRACT,
        "metric_dictionary": "evidence/metric_dictionary.json",
        "score_definition": "evidence/score_definition.json",
        "hashes_file": "SHA256SUMS",
        "working_tree_patch": "working_tree.patch",
        "working_tree_status": "working_tree.status",
        "reproduction": {
            "script": "reproduce.sh",
            "command": reproduction_command,
        },
    }
    evidence_dir = packet / "evidence"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    for source in CLAIM_INPUTS:
        shutil.copy2(source, evidence_dir / source.name)
    parity_report = ROOT / "artifacts/verification/fidelity_parity_v1/report.json"
    parity_archive = ROOT / "artifacts/verification/fidelity_parity_v1/fidelity-parity-evidence.zip"
    if not parity_report.exists():
        # A clean source snapshot does not carry ignored generated artifacts.
        # Generate the parity report in the packet workspace so M3 remains a
        # reproducible property of the source checkout, not of a developer's
        # prior artifact cache.
        import sys

        sys.path.insert(0, str(ROOT / "tools"))
        from run_fidelity_parity import run as run_fidelity_parity

        parity_report = run_fidelity_parity(output=packet / "parity-work")
    if parity_report.exists():
        shutil.copy2(parity_report, evidence_dir / "fidelity-parity-report.json")
        manifest["parity_report"] = "evidence/fidelity-parity-report.json"
    if parity_archive.exists():
        shutil.copy2(parity_archive, evidence_dir / parity_archive.name)
        manifest["parity_archive"] = f"evidence/{parity_archive.name}"
    import sys

    sys.path.insert(0, str(ROOT))
    from tests.e2e.test_golden_source_differential import source_differential_reports

    source_differential_path = evidence_dir / "source-differential-report.json"
    source_differential_path.write_text(
        json.dumps(source_differential_reports(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    manifest["source_differential_report"] = "evidence/source-differential-report.json"
    cases = _load_cases()
    if family_ids is not None:
        cases = tuple(case for case in cases if _family_key(str(case["id"])) in family_ids)
        if not cases:
            raise ValueError("family selection did not match the fidelity ladder catalog")
    for case in cases:
        family = str(case["id"])
        print(f"[fidelity-packet] {family}: ladder", flush=True)
        family_dir = packet / "families" / family
        input_dir = family_dir / "inputs"
        evidence_dir = family_dir / "evidence"
        input_dir.mkdir(parents=True, exist_ok=True)
        evidence_dir.mkdir(parents=True, exist_ok=True)
        point_problem = ROOT / str(case["point_mass_problem"])
        rigid_problem = ROOT / str(case["rigid_body_problem"])
        tables = tuple(ROOT / str(path) for path in case["tables"])
        rigid_tables = tuple(ROOT / str(path) for path in case["rigid_tables"])
        source_validation = ROOT / str(case["source_validation"])
        diagnostic_cases = tuple(dict(item) for item in case.get("diagnostic_cases", ()))
        bridge_problem = _kinematic_problem(point_problem, input_dir / f"{point_problem.stem}_kinematic.prb")
        copied_inputs = [point_problem, rigid_problem, *tables, *rigid_tables]
        copied_inputs.extend(ROOT / str(item["problem"]) for item in diagnostic_cases)
        copied_inputs.extend(ROOT / str(table) for item in diagnostic_cases for table in item.get("tables", ()))
        for source in copied_inputs:
            shutil.copy2(source, input_dir / source.name)
        shutil.copy2(source_validation, evidence_dir / source_validation.name)
        source_validation_payload = json.loads(source_validation.read_text(encoding="utf-8"))
        tiers = (
            ("point-mass-3dof", point_problem, tables),
            ("kinematic-3-plus-3-dof", bridge_problem, tables),
            ("rigid-body-6dof", rigid_problem, rigid_tables),
        )
        family_report: dict[str, Any] = {
            "id": family,
            "display_name": case["display_name"],
            "source_validation": {
                "path": str(Path(str(case["source_validation"]))),
                "status": source_validation_payload.get("status"),
                "dataset": source_validation_payload.get("dataset"),
                "failures": source_validation_payload.get("failures", []),
            },
            "tiers": {},
            "diagnostics": {},
            "long_validation": {},
        }
        for tier, problem, bound_tables in tiers:
            tier_dir = family_dir / tier
            summary = _run_case(
                problem,
                bound_tables,
                tier_dir / "run",
                int(case["max_steps"]),
                plots=plots,
            )
            summary["closure_evaluation"] = _closure_evaluation(family, summary)
            contract_tier = {
                "point-mass-3dof": "3dof",
                "kinematic-3-plus-3-dof": "pseudo_6dof",
                "rigid-body-6dof": "6dof",
            }[tier]
            contract = _scenario_contract(
                case,
                contract_tier,
                problem,
                bound_tables,
                summary,
                unit_system=str(case["point_mass_unit_system"] if tier != "rigid-body-6dof" else case["rigid_body_unit_system"]),
                base_problem=point_problem,
            )
            summary["scenario_contract"] = None if contract is None else {
                **contract.model_dump(mode="json"),
                "contract_sha256": contract.digest(),
                "parity_sha256": contract.digest(include_tier=False),
            }
            summary["evaluation"] = _trajectory_evaluation(
                summary,
                scenario_id=f"{family}:{contract_tier}:ladder",
                claim_boundary="fidelity-ladder execution evidence; unsupported tiers remain diagnostic",
            )
            (tier_dir / "summary.json").parent.mkdir(parents=True, exist_ok=True)
            (tier_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            family_report["tiers"][tier] = summary
        contracts: dict[str, ScenarioContract] = {
            ScenarioContract.model_validate(
                {
                    key: value
                    for key, value in dict(summary["scenario_contract"]).items()
                    if key not in {"contract_sha256", "parity_sha256"}
                }
            ).dynamics_tier: ScenarioContract.model_validate(
                {
                    key: value
                    for key, value in dict(summary["scenario_contract"]).items()
                    if key not in {"contract_sha256", "parity_sha256"}
                }
            )
            for tier, summary in family_report["tiers"].items()
            if summary.get("scenario_contract") is not None
        }
        family_report["scenario_contracts"] = {
            tier: dict(summary["scenario_contract"])
            for tier, summary in family_report["tiers"].items()
            if summary.get("scenario_contract") is not None
        }
        family_report["contract_comparisons"] = {
            f"{left}_vs_{right}": compare_contracts(contracts[left], contracts[right])
            for left, right in (("3dof", "pseudo_6dof"), ("3dof", "6dof"))
            if left in contracts and right in contracts
        }
        for diagnostic in diagnostic_cases:
            diagnostic_problem = ROOT / str(diagnostic["problem"])
            diagnostic_tables = tuple(ROOT / str(path) for path in diagnostic.get("tables", ()))
            diagnostic_id = str(diagnostic["id"])
            diagnostic_dir = family_dir / "diagnostics" / diagnostic_id
            summary = _run_case(
                diagnostic_problem,
                diagnostic_tables,
                diagnostic_dir / "run",
                int(diagnostic.get("max_steps", case["max_steps"])),
                plots=plots,
            )
            summary["closure_evaluation"] = _closure_evaluation(family, summary)
            (diagnostic_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            family_report["diagnostics"][diagnostic_id] = summary
            family_report["diagnostics"][diagnostic_id]["expected_status"] = diagnostic.get("expected_status", "candidate")

        long_case = next(
            (
                item
                for item in (_load_long_cases() if include_long_validation else ())
                if _family_key(str(item["id"])) == _family_key(family)
            ),
            None,
        )
        if long_case is not None:
            print(f"[fidelity-packet] {family}: long validation", flush=True)
            long_dir = family_dir / "long-validation"
            long_input = ROOT / str(long_case["nominal_problem"])
            shutil.copy2(long_input, input_dir / long_input.name)
            nominal = _run_case(
                long_input,
                rigid_tables,
                long_dir / "nominal",
                50_000,
                plots=plots,
            )
            nominal_contract = _scenario_contract(
                case,
                "6dof",
                long_input,
                rigid_tables,
                nominal,
                unit_system=str(case["rigid_body_unit_system"]),
                base_problem=point_problem,
            )
            nominal["scenario_contract"] = None if nominal_contract is None else {
                **nominal_contract.model_dump(mode="json"),
                "contract_sha256": nominal_contract.digest(),
                "parity_sha256": nominal_contract.digest(include_tier=False),
            }
            nominal["closure_evaluation"] = _closure_evaluation(family, nominal)
            long_report: dict[str, Any] = {
                "catalog": {
                    "path": LONG_CONFIG.relative_to(ROOT).as_posix(),
                    "id": long_case["id"],
                    "semantic_family": long_case["semantic_family"],
                    "duration_s": long_case["duration_s"],
                    "phases": long_case["phases"],
                    "primary_metrics": long_case["primary_metrics"],
                    "gates": long_case["gates"],
                    "claim_boundary": "sustained research-surrogate semantics; not flight qualification",
                },
                "nominal": nominal,
            }
            long_report["objective_evaluation"] = _objective_evaluation(
                tuple(dict(item) for item in long_case.get("objectives", ())),
                nominal,
            )
            convergence_specification = dict(long_case.get("convergence", {}))
            if convergence_specification:
                if convergence_specification.get("source") == "fidelity_parity":
                    long_report["convergence"] = _parity_convergence_report(
                        family,
                        convergence_specification,
                        report_path=parity_report,
                    )
                else:
                    long_report["convergence"] = _long_convergence_report(
                        long_input,
                        rigid_tables,
                        long_dir / "convergence",
                        50_000,
                        convergence_specification,
                    )
            nominal["evaluation"] = _trajectory_evaluation(
                nominal,
                scenario_id=f"{family}:6dof:long-validation",
                claim_boundary=str(long_case.get("claim_boundary", "sustained research-surrogate semantics; not flight qualification")),
                objective_report=dict(long_report["objective_evaluation"]),
                convergence_report=long_report.get("convergence"),
            )
            follow_on = long_case.get("follow_on_maneuver")
            if follow_on:
                follow_on_input = ROOT / str(follow_on)
                shutil.copy2(follow_on_input, input_dir / follow_on_input.name)
                long_report["follow_on"] = _run_case(
                    follow_on_input,
                    rigid_tables,
                    long_dir / "follow-on",
                    50_000,
                    plots=plots,
                )
                long_report["follow_on"]["closure_evaluation"] = _closure_evaluation(
                    family,
                    long_report["follow_on"],
                )
                long_report["follow_on"]["objective_evaluation"] = _objective_evaluation(
                    tuple(dict(item) for item in long_case.get("follow_on_objectives", ())),
                    long_report["follow_on"],
                )
                long_report["follow_on"]["evaluation"] = _trajectory_evaluation(
                    long_report["follow_on"],
                    scenario_id=f"{family}:6dof:follow-on",
                    claim_boundary=str(long_case.get("claim_boundary", "sustained research-surrogate semantics; not flight qualification")),
                    objective_report=dict(long_report["follow_on"]["objective_evaluation"]),
                )
            (long_dir / "summary.json").parent.mkdir(parents=True, exist_ok=True)
            (long_dir / "summary.json").write_text(
                json.dumps(long_report, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            family_report["long_validation"] = long_report
            family_report["closure_gate"] = nominal.get("closure_evaluation", {"status": "unavailable"})
        elif not include_long_validation:
            family_report["long_validation"] = {"status": "skipped-by-request"}
            family_report["closure_gate"] = {"status": "skipped-by-request"}
        manifest["families"].append(family_report)
    controller_catalog = yaml.safe_load(
        (ROOT / "verification/controller_scenarios.yaml").read_text(encoding="utf-8")
    )
    controller_reports: list[dict[str, Any]] = []
    controllers = tuple(dict(item) for item in controller_catalog.get("vehicles", ()))
    if family_ids is not None:
        # Controller catalogs historically used semantic families (for example
        # ``fixed-wing``), so an explicit controller selection remains the
        # precise filter.  Family filtering is applied only when the catalog
        # entry carries a vehicle-family id matching the ladder.
        controllers = tuple(
            controller
            for controller in controllers
            if "vehicle_id" not in controller
            or _family_key(str(controller["vehicle_id"])) in family_ids
        )
    if controller_ids is not None:
        controllers = tuple(controller for controller in controllers if str(controller["id"]) in controller_ids)
    for controller in controllers:
        controller_id = str(controller["id"])
        controller_status = str(controller.get("status", "ready"))
        if controller_status == "blocked":
            controller_reports.append(
                {
                    "id": controller_id,
                    "status": "blocked",
                    "reason": str(controller.get("reason", "blocked by controller catalog")),
                }
            )
            continue
        controller_dir = packet / "controllers" / controller_id
        input_dir = controller_dir / "inputs"
        input_dir.mkdir(parents=True, exist_ok=True)
        problem = ROOT / str(controller["problem"])
        tables = tuple(ROOT / str(path) for path in controller.get("tables", ()))
        for source in (problem, *tables):
            shutil.copy2(source, input_dir / source.name)
        summary = _run_case(
            problem,
            tables,
            controller_dir / "run",
            int(controller["max_steps"]),
            plots=plots,
        )
        (controller_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        controller_reports.append(
            {
                "id": controller_id,
                "status": "executed",
                "problem": str(controller["problem"]),
                "summary": (controller_dir / "summary.json").relative_to(packet).as_posix(),
                "expected": dict(controller.get("expectations", {})),
                "expectation_evaluation": _expectation_evaluation(
                    dict(controller.get("expectations", {})),
                    dict(summary.get("telemetry_metrics", {})),
                ),
                "result": summary,
            }
        )
    manifest["controller_cases"] = controller_reports
    controller_mission_catalog = yaml.safe_load(CONTROLLER_MISSION_CONFIG.read_text(encoding="utf-8"))
    controller_mission_claim_boundary = str(controller_mission_catalog["claim_boundary"])
    controller_mission_reports: list[dict[str, Any]] = []
    missions = _load_controller_missions() if include_controller_missions else ()
    for mission in missions:
        family = _family_key(str(mission["family"]))
        if family_ids is not None and family not in family_ids:
            continue
        mission_id = str(mission["id"])
        mission_dir = packet / "controller-missions" / mission_id
        input_dir = mission_dir / "inputs"
        input_dir.mkdir(parents=True, exist_ok=True)
        problem = ROOT / str(mission["problem"])
        tables = tuple(ROOT / str(path) for path in mission.get("tables", ()))
        for source in (problem, *tables):
            shutil.copy2(source, input_dir / source.name)
        summary = _run_case(
            problem,
            tables,
            mission_dir / "run",
            int(mission["max_steps"]),
            plots=plots,
        )
        summary["objective_evaluation"] = _objective_evaluation(
            tuple(dict(item) for item in mission.get("objectives", ())),
            summary,
        )
        summary["closure_evaluation"] = _closure_evaluation(family, summary)
        convergence_specification = dict(mission.get("convergence", {}))
        summary["convergence"] = (
            _long_convergence_report(
                problem,
                tables,
                mission_dir / "convergence",
                int(mission["max_steps"]),
                convergence_specification,
            )
            if convergence_specification
            else {"status": "not_run", "reason": "controller mission has no convergence declaration"}
        )
        summary["evaluation"] = _trajectory_evaluation(
            summary,
            scenario_id=f"{mission_id}:controller-mission",
            claim_boundary=controller_mission_claim_boundary,
            objective_report=dict(summary["objective_evaluation"]),
            convergence_report=summary["convergence"],
        )
        (mission_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        controller_mission_reports.append(
            {
                "id": mission_id,
                "family": family,
                "display_name": str(mission["display_name"]),
                "status": "executed",
                "summary": (mission_dir / "summary.json").relative_to(packet).as_posix(),
                "claim_boundary": controller_mission_claim_boundary,
                "objective_evaluation": summary["objective_evaluation"],
                "closure_evaluation": summary["closure_evaluation"],
                "convergence": summary["convergence"],
                "evaluation": summary["evaluation"],
                "event_continuity_audit": summary.get("event_continuity_audit", {"status": "unavailable"}),
            }
        )
    manifest["controller_missions"] = controller_mission_reports
    manifest["controller_missions_status"] = "executed" if include_controller_missions else "skipped-by-request"
    _write_case_manifests(packet)
    manifest_path = packet / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest["files"] = {
        path.relative_to(packet).as_posix(): _sha256(path)
        for path in _files(packet)
        if path not in {manifest_path, packet / "SHA256SUMS"}
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    hashes_path = packet / "SHA256SUMS"
    hashes_path.write_text(
        "".join(
            f"{_sha256(path)}  {path.relative_to(packet).as_posix()}\n"
            for path in _files(packet)
            if path != hashes_path
        ),
        encoding="utf-8",
    )
    archive = output / f"fidelity-ladder-evidence-{run_id}.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as handle:
        for path in _files(packet):
            handle.write(path, path.relative_to(packet).as_posix())
    return archive
    ####


def _write_case_manifests(packet: Path) -> None:
    """Add a local manifest beside every case summary before bundle hashing."""

    for summary_path in sorted(packet.rglob("summary.json")):
        case_dir = summary_path.parent
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        files = {
            path.relative_to(case_dir).as_posix(): _sha256(path)
            for path in _files(case_dir)
            if path.name != "case_manifest.json"
        }
        contract = summary.get("scenario_contract")
        case_manifest = {
            "schema_version": 1,
            "case_id": case_dir.relative_to(packet).as_posix(),
            "claim_boundary": "source-bounded research-surrogate evidence; not flight qualification",
            "summary": "summary.json",
            "telemetry": summary.get("telemetry_artifact"),
            "scenario_contract": contract,
            "files": files,
        }
        (case_dir / "case_manifest.json").write_text(
            json.dumps(case_manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    ####


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/verification/fidelity_ladder")
    parser.add_argument("--no-plots", action="store_true", help="accepted for parity with other packet builders")
    parser.add_argument(
        "--skip-controller-missions",
        action="store_true",
        help="build plant/ladder evidence without long controller-mission cases",
    )
    parser.add_argument(
        "--skip-long-validation",
        action="store_true",
        help="build the three-tier ladder without the expensive sustained-run refinement",
    )
    parser.add_argument(
        "--family",
        action="append",
        dest="families",
        help="run one ladder family; repeat for multiple families (default: all)",
    )
    parser.add_argument(
        "--controller",
        action="append",
        dest="controllers",
        help="run one controller catalog entry; repeat for multiple entries (default: all selected)",
    )
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=True)
    families = None if not arguments.families else frozenset(_family_key(item) for item in arguments.families)
    controllers = None if not arguments.controllers else frozenset(arguments.controllers)
    print(
        build(
            arguments.output,
            plots=not arguments.no_plots,
            family_ids=families,
            controller_ids=controllers,
            include_long_validation=not arguments.skip_long_validation,
            include_controller_missions=not arguments.skip_controller_missions,
        )
    )
    ####


if __name__ == "__main__":
    main()
