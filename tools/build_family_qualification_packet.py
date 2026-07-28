"""Build one family qualification packet using independent truth objectives."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import shutil
import zipfile
from dataclasses import asdict
from pathlib import Path
from typing import Any

import yaml

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.mission_objectives import ControllerTransition, TruthObjectiveSpec, evaluate_truth_objectives
from taoryx.racetrack_template import load_racetrack_template_catalog
from taoryx.racetrack_timing import estimate_racetrack_timing
from taoryx.runtime.runner import run_files
from taoryx.visualization import render_run_artifact_plots

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "verification/family_qualification_missions.yaml"
RACETRACK_CONFIG = ROOT / "verification/racetrack_templates.yaml"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _files(root: Path) -> tuple[Path, ...]:
    return tuple(path for path in sorted(root.rglob("*")) if path.is_file())


def _local_rows(states: tuple[Any, ...]) -> tuple[dict[str, object], ...]:
    """Convert geodetic truth channels to local-NED metres for evaluation."""

    if not states:
        return ()
    first = dict(states[0].named)
    lat0 = float(first.get("latitude_deg", first.get("lat", 0.0)))
    lon0 = float(first.get("longitude_deg", first.get("long", 0.0)))
    scale_east = 111_320.0 * math.cos(math.radians(lat0))
    rows: list[dict[str, object]] = []
    for state in states:
        named = dict(state.named)
        latitude = float(named.get("latitude_deg", named.get("lat", lat0)))
        longitude = float(named.get("longitude_deg", named.get("long", lon0)))
        # Point-mass historical scalars use feet and feet/second internally;
        # rigid-body adapters already publish canonical SI channels. Resolve
        # both at this evidence boundary so one truth evaluator can serve the
        # same racetrack template at multiple fidelities.
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
        row: dict[str, object] = {
            **named,
            "time_s": float(state.time),
            "north_m": (latitude - lat0) * 111_320.0,
            "east_m": (longitude - lon0) * scale_east,
        }
        rows.append(row)
    return tuple(rows)


def _write_csv(path: Path, rows: tuple[dict[str, object], ...]) -> None:
    keys = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _evaluate_envelope(mission: dict[str, Any], rows: tuple[dict[str, object], ...]) -> dict[str, object]:
    """Evaluate declared hard channel bounds independently of guidance."""

    checks: list[dict[str, object]] = []
    for bound in mission.get("envelope", ()):
        channel = str(bound["channel"])
        minimum = float(bound["minimum"])
        maximum = float(bound["maximum"])
        violations = [
            {"time_s": row.get("time_s"), "value": row.get(channel)}
            for row in rows
            if channel not in row or not math.isfinite(float(row[channel])) or not minimum <= float(row[channel]) <= maximum
        ]
        checks.append({"channel": channel, "minimum": minimum, "maximum": maximum, "violations": violations, "pass": not violations})
    return {"schema_version": 1, "checks": checks, "pass": all(bool(check["pass"]) for check in checks)}


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _resolve_racetrack_objective(
    item: dict[str, Any],
    resolved_racetrack: Any | None,
) -> dict[str, Any]:
    """Resolve a mission objective's shared racetrack gate reference.

    Mission YAML owns acceptance tolerances and timing windows.  The common
    template owns the phase-boundary coordinates, altitude targets, speed, and
    oriented gate normals.  Keeping those concerns separate prevents X8 and
    B747 realizations from drifting apart while still allowing vehicle-specific
    tolerances and windows.
    """

    resolved = dict(item)
    gate_id = resolved.pop("racetrack_gate_id", None)
    if gate_id is None:
        return resolved
    if resolved_racetrack is None:
        raise ValueError(f"objective {item.get('id', '<unknown>')!r} references a racetrack gate without a binding")
    gate = next((candidate for candidate in resolved_racetrack.gates if candidate.id == gate_id), None)
    if gate is None:
        raise ValueError(f"unknown racetrack gate {gate_id!r} in objective {item.get('id', '<unknown>')!r}")
    resolved["target"] = gate.target(resolved_racetrack.speed_m_s)
    resolved["gate_normal"] = gate.gate_normal()
    return resolved


def _validate_racetrack_problem_binding(problem: Path, resolved_racetrack: Any) -> None:
    """Reject a problem file whose runtime route differs from its binding."""

    text = problem.read_text(encoding="utf-8")
    route_lines = tuple(
        line
        for line in text.splitlines()
        if line.lstrip().startswith("*runtime status route ")
    )
    if len(route_lines) != 1:
        raise ValueError(f"{problem} must contain exactly one *runtime status route declaration")
    route_text = route_lines[0]
    for key, expected in resolved_racetrack.route_attributes().items():
        match = re.search(rf"(?:^|\s){re.escape(key)}=([^\s]+)", route_text)
        if match is None:
            raise ValueError(f"{problem} is missing racetrack route attribute {key!r}")
        actual = match.group(1)
        if key == "mode":
            if actual != expected:
                raise ValueError(f"{problem} has {key}={actual!r}; expected {expected!r}")
        elif not math.isclose(float(actual), float(expected), rel_tol=1.0e-9, abs_tol=1.0e-9):
            raise ValueError(f"{problem} has {key}={actual}; expected {expected}")
    ####


def _render_mission_sequence(packet: Path, truth_evaluation: dict[str, object], mission: dict[str, Any]) -> None:
    """Render a sequence-diagram page comparing controller and truth timing."""

    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    results = list(truth_evaluation["results"])
    columns = min(7, max(1, len(results)))
    rows = (len(results) + columns - 1) // columns
    figure, axes = plt.subplots(rows, 1, figsize=(16, max(5.0, 3.4 * rows)), squeeze=False)
    axes_flat = list(axes[:, 0])
    results_by_row = [results[index : index + columns] for index in range(0, len(results), columns)]
    for row_index, row_results in enumerate(results_by_row):
        axis = axes_flat[row_index]
        axis.set_xlim(0.0, float(columns))
        axis.set_ylim(0.0, 1.0)
        axis.axis("off")
        placements = list(range(len(row_results)))
        centers: dict[int, float] = {result_index: float(column) + 0.5 for column, result_index in enumerate(placements)}
        for result_index, item in enumerate(row_results):
            center = centers[result_index]
            passed = item.get("status") == "pass"
            color = "#dcfce7" if passed else "#fee2e2"
            edge = "#16a34a" if passed else "#dc2626"
            truth_time = item.get("truth_time_s")
            controller_time = item.get("controller_time_s")
            truth_line = "truth: " + (f"{float(truth_time):.1f} s / PASS" if truth_time is not None and passed else (f"{float(truth_time):.1f} s / FAIL" if truth_time is not None else "not satisfied"))
            controller_line = "controller: " + (f"{float(controller_time):.1f} s" if controller_time is not None else "—")
            reason = str(item.get("controller_reason") or "—")
            sequence_index = row_index * columns + result_index + 1
            text = f"{sequence_index}. {item['id']}\n{item['objective_type']}\n{controller_line}\nreason: {reason}\n{truth_line}"
            axis.text(
                center,
                0.52,
                text,
                ha="center",
                va="center",
                fontsize=8,
                color="#0f172a",
                bbox={"boxstyle": "round,pad=0.55", "facecolor": color, "edgecolor": edge, "linewidth": 1.3},
            )
            if result_index < len(row_results) - 1:
                next_center = centers[result_index + 1]
                axis.annotate("", xy=(next_center - (0.22 if next_center > center else -0.22), 0.52), xytext=(center + (0.22 if next_center > center else -0.22), 0.52), arrowprops={"arrowstyle": "->", "color": "#64748b", "linewidth": 1.2})
        if row_index < len(results_by_row) - 1:
            last_center = centers[len(row_results) - 1]
            axis.annotate("next row", xy=(0.5, 0.10), xytext=(last_center, 0.28), ha="center", fontsize=7, color="#64748b", arrowprops={"arrowstyle": "->", "color": "#64748b", "linewidth": 1.0})
    realization = mission.get("realization", {})
    realization_label = f"{realization.get('actuator_realization', 'undeclared')} / {realization.get('moment_source', 'undeclared')}"
    figure.suptitle(f"{mission['display_name']} — mission sequence and independent truth adjudication", fontsize=18, fontweight="bold")
    figure.text(0.01, 0.015, f"Control realization: {realization_label}. Controller transitions are diagnostic. Only the truth result determines physical objective completion. Green = pass; red = fail or unsatisfied.", fontsize=9, color="#334155")
    figure.savefig(packet / "mission_sequence.png", dpi=180, bbox_inches="tight", pad_inches=0.25)
    plt.close(figure)


def _diagnostic_controller_transitions(mission: dict[str, Any], rows: tuple[dict[str, object], ...]) -> tuple[ControllerTransition, ...]:
    """Extract route-leg changes as diagnostic records, never as truth evidence."""

    objective_ids = [str(item["id"]) for item in mission.get("objectives", ()) if item.get("objective_type") == "fly_by_gate"]
    transitions: list[ControllerTransition] = []
    previous_leg = 0
    for row in rows:
        # A smooth route phase is a guidance reference, not a controller
        # transition. Only discrete waypoint-leg telemetry is eligible for
        # this diagnostic adapter; otherwise an early phase change would be
        # mistaken for a claimed truth capture and invalidate the mission.
        if "route_leg_index" not in row:
            continue
        leg = int(float(row.get("route_leg_index", 0.0)))
        if leg <= previous_leg or leg > len(objective_ids):
            continue
        transitions.append(
            ControllerTransition(
                objective_id=objective_ids[leg - 1],
                time_s=float(row["time_s"]),
                reason="UNCLASSIFIED",
                source="telemetry.route_leg_index",
            )
        )
        previous_leg = leg
    return tuple(transitions)


def _truth_events(mission: dict[str, Any], rows: tuple[dict[str, object], ...]) -> tuple[set[str], dict[str, float]]:
    """Detect declared truth events directly from accepted telemetry."""

    events: set[str] = set()
    event_times: dict[str, float] = {}
    for declaration in mission.get("truth_events", ()):
        event_id = str(declaration["id"])
        channel = str(declaration["channel"])
        predicate = str(declaration.get("predicate", "at_least"))
        threshold = float(declaration["threshold"])
        start = float(declaration.get("window_start_s", -math.inf))
        end = float(declaration.get("window_end_s", math.inf))
        for row in rows:
            time_s = float(row["time_s"])
            value = row.get(channel)
            if value is None or not start <= time_s <= end:
                continue
            numeric = float(value)
            passed = {
                "at_least": numeric >= threshold,
                "at_most": numeric <= threshold,
            }.get(predicate)
            if passed is None:
                raise ValueError(f"unsupported truth event predicate: {predicate!r}")
            if passed:
                events.add(event_id)
                event_times[event_id] = time_s
                break
    return events, event_times


def _render_board(
    packet: Path,
    rows: tuple[dict[str, object], ...],
    truth_evaluation: dict[str, object],
    status: str,
    mission: dict[str, Any],
) -> None:
    """Render a qualification board from the same truth results as the JSON."""

    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    realization = mission.get("realization", {})
    realization_label = f"{realization.get('actuator_realization', 'undeclared')} / {realization.get('moment_source', 'undeclared')}"

    stride = max(1, len(rows) // 5000)
    plot_rows = rows[::stride]
    if not rows:
        figure, axis = plt.subplots(figsize=(16, 9), layout="constrained")
        axis.axis("off")
        axis.text(
            0.02,
            0.92,
            f"{mission['display_name']} — {status.replace('_', ' ').upper()}",
            fontsize=20,
            fontweight="bold",
            color="#991b1b",
        )
        axis.text(
            0.02,
            0.78,
            "No truth telemetry was produced. See summary.json and runtime diagnostics for the failure.",
            fontsize=14,
            color="#334155",
        )
        figure.savefig(packet / "qualification_board.png", dpi=180, bbox_inches="tight", pad_inches=0.25)
        plt.close(figure)
        return
    if plot_rows[-1] is not rows[-1]:
        plot_rows = (*plot_rows, rows[-1])
    figure, axes = plt.subplots(3, 2, figsize=(16, 12), layout="constrained")
    times = [float(row["time_s"]) for row in plot_rows]
    axes[0, 0].plot([float(row["east_m"]) for row in plot_rows], [float(row["north_m"]) for row in plot_rows], color="#2563eb", label="truth path")
    axes[0, 0].scatter([float(rows[0]["east_m"])], [float(rows[0]["north_m"])], color="#16a34a", label="start")
    axes[0, 0].scatter([float(rows[-1]["east_m"])], [float(rows[-1]["north_m"])], color="#dc2626", label="terminal")
    results_by_id = {str(item["id"]): item for item in truth_evaluation["results"]}
    objective_times: list[tuple[str, float]] = []
    for objective_index, objective in enumerate(mission.get("objectives", ()), start=1):
        target = objective.get("target", {})
        result = results_by_id.get(str(objective["id"]), {})
        if result.get("truth_time_s") is not None:
            event_time = float(result["truth_time_s"])
            objective_times.append((f"{objective_index} {objective['id']}", event_time))
        if "north_m" not in target or "east_m" not in target:
            continue
        passed = result.get("status") == "pass"
        marker_color = "#16a34a" if passed else "#dc2626"
        gate_normal = objective.get("gate_normal")
        corridor = float(objective.get("tolerance", {}).get("corridor_m", 0.0))
        if gate_normal is not None and corridor > 0.0:
            normal_n, normal_e = float(gate_normal[0]), float(gate_normal[1])
            normal_norm = math.hypot(normal_n, normal_e)
            if normal_norm > 0.0:
                tangent_e, tangent_n = -normal_n / normal_norm, normal_e / normal_norm
                axes[0, 0].plot(
                    [float(target["east_m"]) - tangent_e * corridor, float(target["east_m"]) + tangent_e * corridor],
                    [float(target["north_m"]) - tangent_n * corridor, float(target["north_m"]) + tangent_n * corridor],
                    color=marker_color,
                    linestyle="--",
                    linewidth=1.0,
                )
        axes[0, 0].scatter([float(target["east_m"])], [float(target["north_m"])], color=marker_color, marker="x", s=70)
        label_suffix = " gate plane" if objective.get("objective_type") == "fly_by_gate" else ""
        axes[0, 0].annotate(f"{objective_index}: {objective['id']}{label_suffix}", (float(target["east_m"]), float(target["north_m"])), fontsize=7, xytext=(4, 4), textcoords="offset points")
        if result.get("truth_time_s") is not None:
            event_time = float(result["truth_time_s"])
            actual = min(rows, key=lambda row: abs(float(row["time_s"]) - event_time))
            axes[0, 0].scatter([float(actual["east_m"])], [float(actual["north_m"])], color=marker_color, marker="o", facecolors="none", s=55)
    axes[0, 0].set_title("Truth trajectory, oriented gate planes, and objective sequence")
    axes[0, 0].set_xlabel("east (m)")
    axes[0, 0].set_ylabel("north (m)")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True, color="#cbd5e1")
    axes[0, 0].legend(fontsize="small")
    status_line = " | ".join(
        f"{index} {str(results_by_id.get(str(objective['id']), {}).get('status', 'blocked')).upper()}"
        for index, objective in enumerate(mission.get("objectives", ()), start=1)
    )
    objective_ids = {str(objective["id"]) for objective in mission.get("objectives", ())}
    maneuver_description = (
        "altitude-gated lobe crossings, bilateral bank reversal, and pitch reversal"
        if any("pitch" in objective_id for objective_id in objective_ids)
        else "isolated climb/descent gates, bilateral turn-bank reversal, and terminal crossing"
    )
    axes[0, 0].text(
        0.01,
        0.98,
        f"MISSION STATUS\n{status_line}",
        transform=axes[0, 0].transAxes,
        va="top",
        ha="left",
        fontsize=7,
        color="#166534" if all(results_by_id.get(str(objective["id"]), {}).get("status") == "pass" for objective in mission.get("objectives", ())) else "#991b1b",
        bbox={"facecolor": "white", "edgecolor": "#94a3b8", "alpha": 0.90, "pad": 3.0},
    )
    axes[0, 0].text(
        0.01,
        0.02,
        f"Mission: {maneuver_description}\n"
        "Purpose: exercise fixed-wing fly-by geometry, altitude/speed management, and response observability\n"
        "Dashed segments = oriented gate-plane footprints; open circles = truth crossings",
        transform=axes[0, 0].transAxes,
        va="bottom",
        ha="left",
        fontsize=6,
        color="#334155",
        bbox={"facecolor": "white", "edgecolor": "#cbd5e1", "alpha": 0.90, "pad": 3.0},
    )
    terminal_result = results_by_id.get("terminal-contract", {})
    if terminal_result.get("status") != "pass":
        terminal_metric = terminal_result.get("critical_metric") or {}
        terminal_text = ["TERMINAL GATE: FAIL"]
        if terminal_metric:
            units = terminal_metric.get("units", "model units")
            terminal_text.append(
                f"critical {terminal_metric.get('channel', 'metric')}: "
                f"{float(terminal_metric.get('actual', math.nan)):.3g} {units} > "
                f"{float(terminal_metric.get('limit', math.nan)):.3g} {units}"
            )
        if terminal_result.get("truth_time_s") is not None:
            terminal_text.append(f"best/terminal truth sample: {float(terminal_result['truth_time_s']):.1f} s")
        axes[0, 0].text(
            0.99,
            0.98,
            "\n".join(terminal_text),
            transform=axes[0, 0].transAxes,
            va="top",
            ha="right",
            fontsize=7,
            color="#991b1b",
            bbox={"facecolor": "#fff7ed", "edgecolor": "#dc2626", "alpha": 0.92, "pad": 3.0},
        )
    axes[0, 1].plot(times, [float(row.get("altitude_m", math.nan)) for row in plot_rows], color="#2563eb", label="altitude truth")
    speed_axis = axes[0, 1].twinx()
    speed_axis.plot(times, [float(row.get("speed_m_s", math.nan)) for row in plot_rows], color="#7c3aed", label="speed truth")
    altitude_targets: dict[float, list[str]] = {}
    for objective in mission.get("objectives", ()):
        target_altitude = objective.get("target", {}).get("altitude_m")
        if target_altitude is not None:
            altitude_targets.setdefault(float(target_altitude), []).append(str(objective["id"]))
    for altitude, objective_ids in altitude_targets.items():
        axes[0, 1].axhline(altitude, color="#b91c1c", linestyle=":", linewidth=0.9, alpha=0.75, label=f"gate {', '.join(objective_ids)}: {altitude:g} m")
        for objective_id in objective_ids:
            result = results_by_id.get(objective_id, {})
            truth_time = result.get("truth_time_s")
            if truth_time is None:
                continue
            actual_row = min(rows, key=lambda row: abs(float(row["time_s"]) - float(truth_time)))
            actual_altitude = float(actual_row.get("altitude_m", math.nan))
            passed = result.get("status") == "pass"
            marker_color = "#16a34a" if passed else "#dc2626"
            axes[0, 1].scatter([float(truth_time)], [actual_altitude], color=marker_color, edgecolors="white", linewidths=0.6, zorder=5)
            axes[0, 1].annotate(
                f"{objective_id}: {actual_altitude:.1f} m @ {float(truth_time):.1f} s",
                (float(truth_time), actual_altitude),
                xytext=(5, 5),
                textcoords="offset points",
                fontsize=6,
                color=marker_color,
            )
    axes[0, 1].set_title("Altitude and speed truth with explicit altitude gates")
    axes[0, 1].set_xlabel("time (s)")
    axes[0, 1].set_ylabel("altitude (m)")
    speed_axis.set_ylabel("speed (m/s)", color="#7c3aed")
    speed_axis.tick_params(axis="y", labelcolor="#7c3aed")
    axes[0, 1].grid(True, color="#cbd5e1")
    altitude_handles, altitude_labels = axes[0, 1].get_legend_handles_labels()
    speed_handles, speed_labels = speed_axis.get_legend_handles_labels()
    axes[0, 1].legend(altitude_handles + speed_handles, altitude_labels + speed_labels, fontsize="small", loc="best")

    direct_moment_plotted = any("direct_moment_active" in row and float(row.get("direct_moment_active", 0.0)) > 0.5 for row in plot_rows)
    if direct_moment_plotted:
        # A direct-moment realization does not have physical surface
        # commands.  Plot the actual control seam instead of comparing a
        # route attitude reference with a body-moment command as if they
        # were the same quantity.
        moment_axes = (
            ("x", "roll", "#2563eb"),
            ("y", "pitch", "#dc2626"),
            ("z", "yaw", "#16a34a"),
        )
        for axis, label, color in moment_axes:
            for suffix, linestyle, line_label in (
                ("controller_request", ":", "controller request"),
                ("commanded", "-", "limited command"),
                ("applied", "-.", "applied direct moment"),
            ):
                name = f"direct_moment_{suffix}_{axis}_nm"
                if any(name in row for row in plot_rows):
                    axes[1, 0].plot(
                        times,
                        [float(row.get(name, math.nan)) for row in plot_rows],
                        color=color,
                        linestyle=linestyle,
                        label=f"{label} {line_label}",
                    )
        for suffix, linestyle, color, label in (
            ("aero", "--", "#64748b", "aerodynamic load"),
            ("total", ":", "#111827", "total moment"),
        ):
            for axis, axis_label, _axis_color in moment_axes:
                name = f"direct_moment_{suffix}_{axis}_nm"
                if any(name in row for row in plot_rows):
                    axes[1, 0].plot(
                        times,
                        [float(row.get(name, math.nan)) for row in plot_rows],
                        color=color,
                        linestyle=linestyle,
                        alpha=0.55,
                        label=f"{axis_label} {label}",
                    )
        axes[1, 0].set_title("Direct moment realization — request, injection, aero load, total")
        axes[1, 0].set_ylabel("body moment (N m)")
        axes[1, 0].text(
            0.01,
            0.98,
            "This is a direct body-moment path; no surface allocation is claimed.",
            transform=axes[1, 0].transAxes,
            va="top",
            fontsize=6,
            color="#991b1b",
            bbox={"facecolor": "#fff7ed", "edgecolor": "#fecaca", "alpha": 0.9, "pad": 2.0},
        )
        axes[1, 0].legend(fontsize="xx-small", loc="best", ncol=2)
    else:
        allocation_pairs = (
            ("surface_allocation_collective_elevon_commanded_deg", "surface_allocation_collective_elevon_achieved_deg", "collective elevon", "#0891b2"),
            ("surface_allocation_differential_elevon_commanded_deg", "surface_allocation_differential_elevon_achieved_deg", "differential elevon", "#16a34a"),
            ("surface_allocation_symmetric_stabilator_commanded_deg", "surface_allocation_symmetric_stabilator_achieved_deg", "symmetric stabilator", "#ea580c"),
            ("surface_allocation_differential_stabilator_commanded_deg", "surface_allocation_differential_stabilator_achieved_deg", "differential stabilator", "#9333ea"),
            ("surface_allocation_rudder_commanded_deg", "surface_allocation_rudder_achieved_deg", "rudder", "#dc2626"),
        )
        allocation_plotted = False
        for commanded_name, achieved_name, label, color in allocation_pairs:
            if any(commanded_name in row or achieved_name in row for row in plot_rows):
                if any(commanded_name in row for row in plot_rows):
                    axes[1, 0].plot(times, [float(row.get(commanded_name, math.nan)) for row in plot_rows], color=color, linestyle=":", label=f"{label} commanded")
                if any(achieved_name in row for row in plot_rows):
                    axes[1, 0].plot(times, [float(row.get(achieved_name, math.nan)) for row in plot_rows], color=color, linestyle="-", label=f"{label} achieved")
                allocation_plotted = True
        residual_names = (
            ("surface_allocation_actual_residual_nm", "actual aero residual", "-"),
            ("surface_allocation_residual_nm", "linearized residual", "--"),
        )
        residual_axis = None
        for residual_name, residual_label, linestyle in residual_names:
            if any(residual_name in row for row in plot_rows):
                if residual_axis is None:
                    residual_axis = axes[1, 0].twinx()
                    residual_axis.set_ylabel("moment residual (N m)", color="#111827")
                    residual_axis.tick_params(axis="y", labelcolor="#111827")
                residual_axis.plot(times, [float(row.get(residual_name, math.nan)) for row in plot_rows], color="#111827", linestyle=linestyle, label=residual_label)
                allocation_plotted = True
        axes[1, 0].set_title("Physical surface allocation and residual")
        axes[1, 0].set_ylabel("surface deflection (deg)")
        if not allocation_plotted:
            axes[1, 0].text(0.5, 0.5, "not applicable in this run:\ndirect canonical moment realization", ha="center", va="center", transform=axes[1, 0].transAxes, color="#991b1b", fontsize=10, bbox={"facecolor": "#fff7ed", "edgecolor": "#fecaca"})
        else:
            axes[1, 0].legend(fontsize="small", loc="best")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].grid(True, color="#cbd5e1")
    for name, color in (("local_roll_deg", "#2563eb"), ("local_pitch_deg", "#dc2626"), ("local_heading_deg", "#16a34a")):
        values = [float(row.get(name, math.nan)) for row in plot_rows]
        axes[1, 1].plot(times, values, label=name.removesuffix("_deg"), color=color)
    axes[1, 1].set_title(
        "Attitude truth — bank and pitch reversal"
        if any("pitch" in objective_id for objective_id in objective_ids)
        else "Attitude truth — bank and heading reversal"
    )
    axes[1, 1].set_xlabel("time (s)")
    axes[1, 1].set_ylabel("deg")
    axes[1, 1].grid(True, color="#cbd5e1")
    axes[1, 1].legend(fontsize="small")
    actuator_channels = (
        ("route_bank_command_deg", "bank command", "#7c3aed", ":"),
        ("route_bank_achieved_deg", "bank achieved", "#7c3aed", "-"),
        ("route_pitch_command_deg", "pitch command", "#0891b2", ":"),
        ("route_pitch_achieved_deg", "pitch achieved", "#0891b2", "-"),
    )
    for name, label, color, linestyle in actuator_channels:
        if any(name in row for row in plot_rows):
            axes[2, 0].plot(times, [float(row.get(name, math.nan)) for row in plot_rows], label=label, color=color, linestyle=linestyle)
    axes[2, 0].set_title("Guidance commands versus achieved attitude response")
    axes[2, 0].set_xlabel("time (s)")
    axes[2, 0].grid(True, color="#cbd5e1")
    axes[2, 0].legend(fontsize="small")
    axes[2, 0].text(
        0.99,
        0.98,
        "Same color = command/achieved pair; dotted = commanded, solid = achieved.\n"
        "Physical allocation is shown only when the run declares a surface allocator.",
        transform=axes[2, 0].transAxes,
        va="top",
        ha="right",
        fontsize=6,
        color="#7f1d1d",
        bbox={"facecolor": "white", "edgecolor": "#fecaca", "alpha": 0.90, "pad": 3.0},
    )
    for axis in (axes[0, 1], speed_axis, axes[1, 0], axes[1, 1], axes[2, 0]):
        for objective_id, event_time in objective_times:
            axis.axvline(event_time, color="#b91c1c", linestyle="--", linewidth=0.7, alpha=0.35)
            if axis is axes[1, 0]:
                axis.text(event_time, axis.get_ylim()[1], objective_id, rotation=90, va="top", ha="right", fontsize=6, color="#991b1b")
        if objective_times:
            axis.text(0.99, 0.02, "dashed lines: truth objective completion", transform=axis.transAxes, ha="right", va="bottom", fontsize=7, color="#991b1b")
    table_rows = []
    for item in truth_evaluation["results"]:
        metric = item.get("critical_metric") or {}
        controller_time = item.get("controller_time_s")
        controller = "—" if controller_time is None else f"{float(controller_time):.1f} {item.get('controller_reason', '')}"
        table_rows.append(
            [
                item["id"],
                item["objective_type"],
                item["status"].upper(),
                "—" if item.get("truth_time_s") is None else f"{float(item['truth_time_s']):.1f}",
                controller,
                str(metric.get("channel", "—")),
                "—" if metric.get("actual") is None else f"{float(metric['actual']):.3g}",
                "—" if metric.get("limit") is None else f"{float(metric['limit']):.3g}",
                "—" if metric.get("margin") is None else f"{float(metric['margin']):+.3g} {metric.get('units', '')}",
            ]
        )
    axes[2, 1].table(
        cellText=table_rows,
        colLabels=["objective", "type", "truth", "truth t(s)", "controller", "critical", "actual", "limit", "margin"],
        loc="center",
        cellLoc="center",
    ).scale(1.0, 1.7)
    axes[2, 1].set_title("Independent truth objective results")
    figure.suptitle(f"{mission['display_name']} — {status.replace('_', ' ').upper()}", fontsize=20, fontweight="bold")
    figure.text(
        0.01,
        0.005,
        f"CONTROL REALIZATION: {realization_label}. "
        f"CLAIM: {mission['claim']} "
        "NONCLAIMS: source-validated controller/actuator fidelity, robustness, and any unsupported terminal behavior. "
        "Truth objective status is independent of controller transitions.",
        fontsize=8,
        color="#334155",
    )
    figure.savefig(packet / "qualification_board.png", dpi=180, bbox_inches="tight", pad_inches=0.25)
    plt.close(figure)


def build(output: Path, mission_id: str) -> Path:
    catalog = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    mission = next(item for item in catalog["missions"] if item["id"] == mission_id)
    packet = output / mission_id
    run_dir = packet / "run"
    inputs = packet / "inputs"
    run_dir.mkdir(parents=True, exist_ok=True)
    inputs.mkdir(parents=True, exist_ok=True)
    problem = ROOT / mission["problem"]
    tables = tuple(ROOT / path for path in mission["tables"])
    resolved_racetrack = None
    if "racetrack_binding" in mission:
        racetrack_catalog = load_racetrack_template_catalog(RACETRACK_CONFIG)
        resolved_racetrack = racetrack_catalog.get(str(mission["racetrack_binding"]))
        _validate_racetrack_problem_binding(problem, resolved_racetrack)
    shutil.copy2(problem, inputs / problem.name)
    for table in tables:
        shutil.copy2(table, inputs / table.name)
    report = run_files(
        problem,
        tables,
        output_dir=run_dir,
        max_steps=int(mission["max_steps"]),
        integrator=str(mission.get("integrator", "rk4")),
        profile=GrammarProfile.TAORYX,
    )
    states = tuple(next(iter(report.results[0].states.values()), ())) if report.results else ()
    rows = _local_rows(states)
    _write_csv(packet / "truth_telemetry.csv", rows)
    controller_transitions = _diagnostic_controller_transitions(mission, rows)
    objective_specs = tuple(
        TruthObjectiveSpec(**_resolve_racetrack_objective(item, resolved_racetrack))
        for item in mission["objectives"]
    )
    terminal = dict(mission["terminal"])
    terminal = _resolve_racetrack_objective(terminal, resolved_racetrack)
    terminal_objective_type = str(terminal.get("objective_type", "terminal_state_gate"))
    objective_specs += (
        TruthObjectiveSpec(
            id="terminal-contract",
            objective_type=terminal_objective_type,
            target=dict(terminal["target"]),
            tolerance=dict(terminal["tolerance"]),
            dwell_s=float(terminal["dwell_s"]),
            window_start_s=terminal.get("window_start_s"),
            window_end_s=terminal.get("window_end_s"),
            gate_normal=None if terminal.get("gate_normal") is None else tuple(terminal["gate_normal"]),
            crossing_direction=int(terminal.get("crossing_direction", 1)),
        ),
    )
    truth_events, truth_event_times = _truth_events(mission, rows)
    if any(float(row.get("motor_shutdown", 0.0)) >= 0.5 for row in rows):
        truth_events.add("motor-shutdown")
        truth_event_times["motor-shutdown"] = next(float(row["time_s"]) for row in rows if float(row.get("motor_shutdown", 0.0)) >= 0.5)
    numerical_valid = report.exit_code == 0 and all(result.completed for result in report.results)
    envelope_report = _evaluate_envelope(mission, rows)
    hard_gates_passed = numerical_valid and bool(envelope_report["pass"])
    truth_evaluation = evaluate_truth_objectives(
        objective_specs,
        rows,
        controller_transitions=controller_transitions,
        truth_events=truth_events,
        truth_event_times=truth_event_times,
        hard_gates_passed=hard_gates_passed,
    )
    mission_pass = bool(truth_evaluation["mission_pass"])
    timing_estimate = None
    if resolved_racetrack is not None:
        timing_estimate = asdict(resolved_racetrack.timing)
        _write_json(
            packet / "resolved_racetrack.json",
            {
                "schema_version": 1,
                "template_id": resolved_racetrack.template_id,
                "binding_id": resolved_racetrack.binding_id,
                "vehicle_id": resolved_racetrack.vehicle_id,
                "fidelity": resolved_racetrack.fidelity,
                "source_realization": resolved_racetrack.source_realization,
                "binding_status": resolved_racetrack.status,
                "route_attributes": resolved_racetrack.route_attributes(),
                "timing": resolved_racetrack.timing_manifest(),
                "phase_windows": [asdict(window) for window in resolved_racetrack.phase_windows],
                "gates": [
                    {
                        "id": gate.id,
                        "phase": gate.phase,
                        "target": gate.target(resolved_racetrack.speed_m_s),
                        "gate_normal": gate.gate_normal(),
                    }
                    for gate in resolved_racetrack.gates
                ],
            },
        )
    elif "timing_estimate" in mission:
        timing_estimate = asdict(estimate_racetrack_timing(**mission["timing_estimate"]))
    if timing_estimate is not None:
        _write_json(packet / "preflight_timing_estimate.json", timing_estimate)
    summary = {
        "schema_version": 1,
        "mission_id": mission_id,
        "family": mission["family"],
        "display_name": mission["display_name"],
        "claim": mission["claim"],
        "nonclaims": mission["nonclaims"],
        "evidence_level": mission["evidence_level"],
        "fidelity_tier": None if resolved_racetrack is None else resolved_racetrack.fidelity,
        "realization": mission.get("realization", {}),
        "status": (
            "nominal_case_pass_overall_qualification_pending"
            if mission_pass
            else (
                "integration_failure_qualification_blocked"
                if not numerical_valid
                else (
                    "nominal_objective_failure_qualification_pending"
                    if truth_evaluation["required_passed"] < truth_evaluation["required_objectives"]
                    else "nominal_objective_pass_terminal_pending"
                )
            )
        ),
        "mission_pass": mission_pass,
        "numerical_valid": numerical_valid,
        "envelope_report": envelope_report,
        "truth_evaluation": truth_evaluation,
        "preflight_timing_estimate": timing_estimate,
        "run": {
            "exit_code": report.exit_code,
            "completed": [result.completed for result in report.results],
            "stop_reasons": [result.stop_reason for result in report.results],
            "sample_count": len(rows),
            "duration_s": None if not rows else rows[-1]["time_s"],
        },
    }
    _write_json(packet / "summary.json", summary)
    _write_json(packet / "objective_report.json", truth_evaluation)
    _write_json(packet / "envelope_report.json", envelope_report)
    _write_json(
        packet / "controller_transitions.json",
        {
            "schema_version": 1,
            "source": "controller",
            "transitions": [
                {
                    "objective_id": transition.objective_id,
                    "time_s": transition.time_s,
                    "reason": transition.reason,
                    "source": transition.source,
                }
                for transition in controller_transitions
            ],
            "note": "These route-leg changes are diagnostic telemetry records without causal controller reasons; truth results remain independent and an early transition invalidates that objective for mission pass.",
        },
    )
    _write_json(
        packet / "events.json",
        {
            "schema_version": 1,
            "truth_events": sorted(truth_events),
            "truth_event_times_s": truth_event_times,
            "note": "Events are detected from truth telemetry channels; controller event flags are not used to pass objectives.",
        },
    )
    (packet / "README.md").write_text(
        "\n".join(
            [
                f"# {mission['display_name']}",
                "",
                f"Status: **{summary['status']}**",
                "",
                "This packet is evaluated by `independent_truth_telemetry`. Controller guidance transitions do not certify objectives.",
                "",
                f"Claim: {mission['claim']}",
                "",
                "Nonclaims:",
                *[f"- {item}" for item in mission["nonclaims"]],
                "",
                "The strict qualification evaluator computes required objectives from truth telemetry. A nominal pass does not imply source validation, robustness, or hardware fidelity beyond the claim above.",
                "",
                "Reproduce with the command in `reproduction.txt`.",
                "",
            ]
        ),
        encoding="utf-8",
    )
    (packet / "mission.yaml").write_text(yaml.safe_dump(mission, sort_keys=False), encoding="utf-8")
    (packet / "reproduction.txt").write_text(
        f"PYTHONPATH=src python3 tools/build_family_qualification_packet.py --output {output} --mission {mission_id}\n",
        encoding="utf-8",
    )
    if report.artifacts:
        render_run_artifact_plots(
            report.artifacts[0],
            packet / "plots",
            vehicle_id="1",
            channels=(
                "taos.altitude_m",
                "taos.speed_m_s",
                "taos.local_roll_deg",
                "taos.local_pitch_deg",
                "taos.local_heading_deg",
                "taos.route_cross_track_error_m",
                "taos.translation_equation_residual_normalized",
                "taos.rotation_equation_residual_normalized",
            ),
        )
    _render_board(packet, rows, truth_evaluation, str(summary["status"]), mission)
    _render_mission_sequence(packet, truth_evaluation, mission)
    manifest = {
        "schema_version": 1,
        "claim_boundary": catalog["claim_boundary"],
        "mission_id": mission_id,
        "fidelity_tier": None if resolved_racetrack is None else resolved_racetrack.fidelity,
        "realization": mission.get("realization", {}),
        "summary": "summary.json",
        "truth_telemetry": "truth_telemetry.csv",
        "files": {},
    }
    manifest_path = packet / "manifest.json"
    manifest["files"] = {str(path.relative_to(packet)): _sha256(path) for path in _files(packet) if path != manifest_path}
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    archive = output / f"{mission_id}.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as handle:
        for path in _files(packet):
            handle.write(path, path.relative_to(packet))
    return archive


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("/tmp/taoryx-family-qualification"))
    parser.add_argument("--mission", default="hummingbird-hover-to-touchdown-strict-v1")
    args = parser.parse_args()
    print(build(args.output, args.mission))


if __name__ == "__main__":
    main()
