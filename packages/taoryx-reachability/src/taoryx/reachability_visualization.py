"""Matplotlib views for Alpha 3 reachability envelope artifacts."""

from __future__ import annotations

import csv
import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .reachability_envelope import ReachabilityEnvelope

Payload = Mapping[str, Any]


@dataclass(frozen=True, slots=True)
class ReachabilityPlotReport:
    """Paths emitted by one deterministic reachability plot bundle."""

    plot_paths: tuple[Path, ...]
    manifest_path: Path
    omitted: tuple[dict[str, str], ...] = ()
    artifact_paths: tuple[Path, ...] = ()
    ####


def load_reachability_artifact(path: str | Path) -> dict[str, Any]:
    """Load and minimally validate a serialized reachability result."""

    source = Path(path)
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("schema") != "trajectory.reachability-envelope/v1alpha1":
        raise ValueError(f"unsupported reachability artifact: {source}")
    return payload
    ####


def plot_flown_trajectories(
    source: ReachabilityEnvelope | Payload,
    path: str | Path,
    *,
    dpi: int = 140,
) -> Path:
    """Plot every retained flight path in top-down and altitude views."""

    plt = _get_matplotlib()
    payload = _payload(source)
    samples = _samples(payload)
    figure, axes = plt.subplots(1, 2, figsize=(12.0, 5.2), layout="constrained")
    try:
        phase_colors = {"boost": "#d97706", "glide": "#0f766e"}
        seen_phases: set[str] = set()
        seen_statuses: set[str] = set()
        for sample in samples:
            trajectory = sample.get("trajectory")
            if not isinstance(trajectory, Mapping):
                continue
            fields = trajectory.get("fields")
            rows = trajectory.get("rows")
            if not isinstance(fields, list) or not isinstance(rows, list):
                continue
            columns = _columns(fields, rows)
            status = str(sample.get("classification", "unknown"))
            status_color = _status_color(status)
            linewidth = 1.8 if status == "feasible" else 0.8
            alpha = 0.82 if status == "feasible" else 0.25
            for first, second in zip(rows[:-1], rows[1:], strict=True):
                left = _row_columns(fields, first)
                right = _row_columns(fields, second)
                phase = str(left.get("phase", "unknown"))
                color = phase_colors.get(phase, status_color)
                seen_phases.add(phase)
                axes[0].plot(
                    [left["x_m"], right["x_m"]],
                    [left["y_m"], right["y_m"]],
                    color=color,
                    linewidth=linewidth,
                    alpha=alpha,
                )
                axes[1].plot(
                    [left["x_m"], right["x_m"]],
                    [left["z_m"], right["z_m"]],
                    color=color,
                    linewidth=linewidth,
                    alpha=alpha,
                )
            if columns:
                axes[0].scatter(columns["x_m"][0], columns["y_m"][0], color="#334155", marker="o", s=15, alpha=0.7)
                axes[0].scatter(columns["x_m"][-1], columns["y_m"][-1], color=status_color, marker="x", s=28)
                axes[1].scatter(columns["x_m"][-1], columns["z_m"][-1], color=status_color, marker="x", s=28)
            seen_statuses.add(status)
            child_payloads = sample.get("spawned_bodies", [])
            if isinstance(child_payloads, list):
                for child in child_payloads:
                    if not isinstance(child, Mapping):
                        continue
                    child_trajectory = child.get("trajectory")
                    if not isinstance(child_trajectory, Mapping):
                        continue
                    child_fields = child_trajectory.get("fields")
                    child_rows = child_trajectory.get("rows")
                    if not isinstance(child_fields, list) or not isinstance(child_rows, list):
                        continue
                    for first, second in zip(child_rows[:-1], child_rows[1:], strict=True):
                        left = _row_columns(child_fields, first)
                        right = _row_columns(child_fields, second)
                        if not {"x_m", "y_m", "z_m"}.issubset(left) or not {"x_m", "y_m", "z_m"}.issubset(right):
                            continue
                        axes[0].plot(
                            [left["x_m"], right["x_m"]],
                            [left["y_m"], right["y_m"]],
                            color="#475569",
                            linewidth=1.0,
                            linestyle="--",
                            alpha=0.65,
                        )
                        axes[1].plot(
                            [left["x_m"], right["x_m"]],
                            [left["z_m"], right["z_m"]],
                            color="#475569",
                            linewidth=1.0,
                            linestyle="--",
                            alpha=0.65,
                        )
                    if child_rows:
                        first = _row_columns(child_fields, child_rows[0])
                        last = _row_columns(child_fields, child_rows[-1])
                        if {"x_m", "y_m", "z_m"}.issubset(first) and {"x_m", "y_m", "z_m"}.issubset(last):
                            axes[0].scatter(first["x_m"], first["y_m"], color="#475569", marker="o", s=12, alpha=0.7)
                            axes[0].scatter(last["x_m"], last["y_m"], color="#475569", marker="x", s=24)
                            axes[1].scatter(last["x_m"], last["z_m"], color="#475569", marker="x", s=24)
        axes[0].set_title("Flown trajectories: top-down", loc="left")
        axes[0].set_xlabel("downrange x (m)")
        axes[0].set_ylabel("crossrange y (m)")
        axes[0].axis("equal")
        axes[1].set_title("Flown trajectories: vertical profile", loc="left")
        axes[1].set_xlabel("downrange x (m)")
        axes[1].set_ylabel("altitude z (m)")
        for axis in axes:
            axis.grid(True, color="#cbd5e1", linewidth=0.7)
        handles = []
        from matplotlib.lines import Line2D

        for phase in ("boost", "glide"):
            if phase in seen_phases:
                handles.append(Line2D([0], [0], color=phase_colors[phase], linewidth=2.2, label=phase))
        if any(sample.get("spawned_bodies") for sample in samples):
            handles.append(Line2D([0], [0], color="#475569", linewidth=1.2, linestyle="--", label="spawned body"))
        for status in ("feasible", "infeasible", "invalid"):
            if status in seen_statuses:
                handles.append(Line2D([0], [0], color=_status_color(status), linewidth=2.0, label=status))
        if handles:
            axes[0].legend(handles=handles, loc="best", fontsize=8)
        return _save_figure(figure, path, dpi)
    finally:
        plt.close(figure)
    ####


def plot_deployment_timeline(
    source: ReachabilityEnvelope | Payload,
    path: str | Path,
    *,
    dpi: int = 140,
) -> Path:
    """Plot accepted deployment times and parent/child identities."""

    plt = _get_matplotlib()
    events = [event for sample in _samples(_payload(source)) for event in sample.get("deployment_events", []) if isinstance(event, Mapping)]
    figure, axis = plt.subplots(figsize=(10.0, max(2.8, 0.55 * len(events) + 1.5)), layout="constrained")
    try:
        if not events:
            axis.text(0.5, 0.5, "No deployment events recorded", ha="center", va="center", transform=axis.transAxes)
            axis.set_axis_off()
        else:
            for index, event in enumerate(events):
                time = float(event.get("accepted_time_s", event.get("time_s", 0.0)))
                label = f"{event.get('parent_model_id', 'parent')} -> {event.get('child_model_id', 'child')}"
                axis.scatter(time, index, color="#0f766e", s=42, zorder=2)
                axis.text(time, index + 0.12, label, fontsize=8, va="bottom")
            axis.set_xlabel("accepted event time (s)")
            axis.set_yticks([])
            axis.set_title("Deployment event timeline", loc="left")
            axis.grid(True, axis="x", color="#cbd5e1", linewidth=0.7)
        return _save_figure(figure, path, dpi)
    finally:
        plt.close(figure)
    ####


def plot_projected_area(
    source: ReachabilityEnvelope | Payload,
    path: str | Path,
    *,
    dpi: int = 140,
) -> Path:
    """Plot projected drag area and drag load for every spawned body."""

    plt = _get_matplotlib()
    children = [
        child
        for sample in _samples(_payload(source))
        for child in sample.get("spawned_bodies", [])
        if isinstance(child, Mapping)
    ]
    figure, axes = plt.subplots(2, 1, figsize=(10.0, 6.5), sharex=True, layout="constrained")
    try:
        legend_labels: set[str] = set()
        for child in children:
            telemetry = child.get("telemetry", [])
            if not isinstance(telemetry, list):
                continue
            rows = [row for row in telemetry if isinstance(row, Mapping)]
            if not rows:
                continue
            label = f"{child.get('body_id', 'child')} ({child.get('shape', 'unknown')})"
            plotted_label = label if label not in legend_labels else "_nolegend_"
            legend_labels.add(label)
            times = [float(row["time_s"]) for row in rows]
            axes[0].plot(times, [float(row["projected_area_m2"]) for row in rows], linewidth=1.4, label=plotted_label)
            axes[1].plot(times, [float(row["drag_force_n"]) for row in rows], linewidth=1.4, label=plotted_label)
        axes[0].set_ylabel("projected area (m2)")
        axes[1].set_ylabel("drag force (N)")
        axes[1].set_xlabel("time (s)")
        axes[0].set_title("Detached-body aerodynamic history", loc="left")
        for axis in axes:
            axis.grid(True, color="#cbd5e1", linewidth=0.7)
            if legend_labels:
                axis.legend(loc="best", fontsize=8)
        return _save_figure(figure, path, dpi)
    finally:
        plt.close(figure)
    ####


def _parent_only_payload(payload: Payload) -> dict[str, Any]:
    """Build a plotting payload with child traces removed."""

    result = dict(payload)
    result["samples"] = [
        {**sample, "spawned_bodies": []}
        for sample in _samples(payload)
    ]
    return result


def _children_only_payload(payload: Payload) -> dict[str, Any]:
    """Build a plotting payload containing only spawned-body traces."""

    result = dict(payload)
    result["samples"] = [
        {
            **sample,
            "trajectory": {"fields": [], "rows": []},
            "spawned_bodies": sample.get("spawned_bodies", []),
        }
        for sample in _samples(payload)
    ]
    return result


def plot_parent_trajectory(source: ReachabilityEnvelope | Payload, path: str | Path, *, dpi: int = 140) -> Path:
    """Write the canonical parent-only trajectory view."""

    return plot_flown_trajectories(_parent_only_payload(_payload(source)), path, dpi=dpi)
    ####


def plot_children_trajectories(source: ReachabilityEnvelope | Payload, path: str | Path, *, dpi: int = 140) -> Path:
    """Write the canonical spawned-child trajectory view."""

    return plot_flown_trajectories(_children_only_payload(_payload(source)), path, dpi=dpi)
    ####


def _write_telemetry_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> Path:
    """Write stable flattened telemetry rows while retaining vector fields as JSON."""

    path.parent.mkdir(parents=True, exist_ok=True)
    keys = sorted({str(key) for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=("query_id", *keys), extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({
                "query_id": row.get("query_id", ""),
                **{
                    key: json.dumps(row[key], sort_keys=True) if isinstance(row.get(key), (list, dict)) else row.get(key, "")
                    for key in keys
                },
            })
    return path
    ####


def _deployment_telemetry_artifacts(payload: Payload, destination: Path) -> tuple[Path, ...]:
    """Write aggregate parent and one CSV per spawned-body identity."""

    parent_rows: list[dict[str, Any]] = []
    child_rows: dict[str, list[dict[str, Any]]] = {}
    for sample in _samples(payload):
        query_id = str(sample.get("query_id", ""))
        for row in sample.get("telemetry", []):
            if isinstance(row, Mapping):
                parent_rows.append({"query_id": query_id, **dict(row)})
        for child in sample.get("spawned_bodies", []):
            if not isinstance(child, Mapping):
                continue
            child_id = str(child.get("body_id", "child"))
            for row in child.get("telemetry", []):
                if isinstance(row, Mapping):
                    child_rows.setdefault(child_id, []).append({"query_id": query_id, **dict(row)})
    paths = [_write_telemetry_csv(destination / "telemetry_parent.csv", parent_rows)]
    paths.extend(_write_telemetry_csv(destination / f"telemetry_{child_id}.csv", rows) for child_id, rows in sorted(child_rows.items()))
    return tuple(paths)
    ####


def plot_search_coverage(
    source: ReachabilityEnvelope | Payload,
    path: str | Path,
    *,
    dpi: int = 140,
) -> Path:
    """Plot realized candidates against the declared azimuth/elevation grid."""

    plt = _get_matplotlib()
    payload = _payload(source)
    axes_spec = _axis_map(payload)
    azimuth = axes_spec.get("launch.azimuth_rad")
    elevation = axes_spec.get("launch.elevation_rad")
    bank = axes_spec.get("glide.bank_rad")
    if not azimuth or not elevation or not bank:
        raise ValueError("search coverage requires azimuth, elevation, and bank axes")
    banks = [float(value) for value in bank["values"]]
    columns = min(3, max(1, len(banks)))
    rows = math.ceil(len(banks) / columns)
    figure, axes = plt.subplots(rows, columns, figsize=(4.6 * columns, 3.8 * rows), squeeze=False, layout="constrained")
    try:
        samples = _samples(payload)
        observed = {(float(sample["search_coordinates"]["launch.azimuth_rad"]), float(sample["search_coordinates"]["launch.elevation_rad"]), float(sample["search_coordinates"]["glide.bank_rad"])) for sample in samples}
        for index, bank_value in enumerate(banks):
            axis = axes[index // columns][index % columns]
            for x_value in azimuth["values"]:
                for y_value in elevation["values"]:
                    axis.scatter(float(x_value), float(y_value), color="#cbd5e1", marker="s", s=34, zorder=1)
            for sample in samples:
                coordinates = sample.get("search_coordinates")
                if not isinstance(coordinates, Mapping) or not math.isclose(float(coordinates["glide.bank_rad"]), bank_value):
                    continue
                status = str(sample.get("classification", "unknown"))
                axis.scatter(
                    float(coordinates["launch.azimuth_rad"]),
                    float(coordinates["launch.elevation_rad"]),
                    color=_status_color(status),
                    edgecolor="#0f172a",
                    linewidth=0.35,
                    s=50,
                    zorder=2,
                )
            axis.set_title(f"bank = {math.degrees(bank_value):g} deg", loc="left")
            axis.set_xlabel("launch azimuth (rad)")
            axis.set_ylabel("launch elevation (rad)")
            axis.set_xticks([float(value) for value in azimuth["values"]])
            axis.set_yticks([float(value) for value in elevation["values"]])
            axis.grid(True, color="#cbd5e1", linewidth=0.7)
        for index in range(len(banks), rows * columns):
            axes[index // columns][index % columns].axis("off")
        from matplotlib.lines import Line2D

        handles = [
            Line2D([0], [0], marker="s", color="w", markerfacecolor="#cbd5e1", label="declared candidate", markersize=7),
            Line2D([0], [0], marker="o", color="w", markerfacecolor=_status_color("feasible"), label="feasible", markersize=7),
            Line2D([0], [0], marker="o", color="w", markerfacecolor=_status_color("infeasible"), label="infeasible", markersize=7),
        ]
        axes[0][0].legend(handles=handles, loc="best", fontsize=8)
        candidate_count = payload.get("summary", {}).get("candidate_count", len(observed))
        figure.suptitle(f"Search coverage: {candidate_count} realized candidates", x=0.02, ha="left")
        return _save_figure(figure, path, dpi)
    finally:
        plt.close(figure)
    ####


def plot_terminal_capability(
    source: ReachabilityEnvelope | Payload,
    path: str | Path,
    *,
    dpi: int = 140,
) -> Path:
    """Plot terminal capability and terminal-speed variation."""

    plt = _get_matplotlib()
    payload = _payload(source)
    samples = _samples(payload)
    figure, axes = plt.subplots(1, 2, figsize=(11.5, 4.8), layout="constrained")
    try:
        for sample in samples:
            terminal = sample.get("terminal_state", {})
            if not isinstance(terminal, Mapping):
                continue
            position = terminal.get("position_m", [])
            if not isinstance(position, list) or len(position) < 3:
                continue
            status = str(sample.get("classification", "unknown"))
            color = _status_color(status)
            axes[0].scatter(float(position[0]), float(position[1]), color=color, s=44, alpha=0.85, edgecolor="#0f172a", linewidth=0.3)
            axes[1].scatter(float(position[0]), float(position[2]), color=color, s=44, alpha=0.85, edgecolor="#0f172a", linewidth=0.3)
        axes[0].set_title("Terminal footprint", loc="left")
        axes[0].set_xlabel("downrange x (m)")
        axes[0].set_ylabel("crossrange y (m)")
        axes[1].set_title("Terminal altitude capability", loc="left")
        axes[1].set_xlabel("downrange x (m)")
        axes[1].set_ylabel("terminal altitude z (m)")
        for axis in axes:
            axis.grid(True, color="#cbd5e1", linewidth=0.7)
        from matplotlib.lines import Line2D

        handles = [
            Line2D([0], [0], marker="o", color="w", markerfacecolor=_status_color("feasible"), label="feasible", markersize=7),
            Line2D([0], [0], marker="o", color="w", markerfacecolor=_status_color("infeasible"), label="infeasible", markersize=7),
            Line2D([0], [0], marker="o", color="w", markerfacecolor=_status_color("invalid"), label="invalid", markersize=7),
        ]
        axes[0].legend(handles=handles, loc="best", fontsize=8)
        return _save_figure(figure, path, dpi)
    finally:
        plt.close(figure)
    ####


def plot_fidelity_progression(
    sources: Sequence[ReachabilityEnvelope | Payload],
    path: str | Path,
    *,
    dpi: int = 140,
) -> Path:
    """Show terminal movement and feasible-set changes across fidelities."""

    if len(sources) < 2:
        raise ValueError("fidelity progression requires at least two envelope results")
    plt = _get_matplotlib()
    payloads = [_payload(source) for source in sources]
    labels = [str(payload.get("fidelity", payload.get("study", {}).get("fidelity", f"result-{index}"))) for index, payload in enumerate(payloads)]
    figure, axes = plt.subplots(1, 3, figsize=(14.0, 4.8), layout="constrained")
    try:
        for first, second in zip(payloads[:-1], payloads[1:], strict=True):
            first_samples = _samples(first)
            second_samples = _samples(second)
            if len(first_samples) != len(second_samples):
                continue
            for left, right in zip(first_samples, second_samples, strict=True):
                left_position = left.get("terminal_state", {}).get("position_m", [])
                right_position = right.get("terminal_state", {}).get("position_m", [])
                if len(left_position) < 2 or len(right_position) < 2:
                    continue
                axes[0].plot(
                    [float(left_position[0]), float(right_position[0])],
                    [float(left_position[1]), float(right_position[1])],
                    color="#94a3b8",
                    linewidth=0.7,
                    alpha=0.35,
                )
        for index, payload in enumerate(payloads):
            samples = _samples(payload)
            color = _series_color(index)
            positions = [sample.get("terminal_state", {}).get("position_m", []) for sample in samples]
            feasible = [sample for sample in samples if sample.get("classification") == "feasible"]
            axes[0].scatter(
                [float(position[0]) for position in positions if len(position) >= 2],
                [float(position[1]) for position in positions if len(position) >= 2],
                color=color,
                s=38,
                alpha=0.72,
                label=labels[index],
            )
            axes[1].bar(index, len(feasible), color=color, alpha=0.85)
            max_range = max((_range_of(sample) for sample in feasible), default=0.0)
            axes[2].bar(index, max_range, color=color, alpha=0.85)
        axes[0].set_title("Terminal movement across fidelity", loc="left")
        axes[0].set_xlabel("downrange x (m)")
        axes[0].set_ylabel("crossrange y (m)")
        axes[0].legend(loc="best", fontsize=8)
        axes[1].set_title("Feasible candidate count", loc="left")
        axes[1].set_ylabel("count")
        axes[2].set_title("Maximum feasible terminal range", loc="left")
        axes[2].set_ylabel("range (m)")
        axes[1].set_xticks(range(len(labels)), labels, rotation=25, ha="right")
        axes[2].set_xticks(range(len(labels)), labels, rotation=25, ha="right")
        for axis in axes:
            axis.grid(True, axis="y", color="#cbd5e1", linewidth=0.7)
        return _save_figure(figure, path, dpi)
    finally:
        plt.close(figure)
    ####


def render_reachability_plot_bundle(
    source: ReachabilityEnvelope | Payload,
    directory: str | Path,
    *,
    comparison_sources: Sequence[ReachabilityEnvelope | Payload] = (),
    dpi: int = 140,
) -> ReachabilityPlotReport:
    """Render the standard Alpha 3 reachability plot bundle."""

    if dpi <= 0:
        raise ValueError("plot dpi must be positive")
    destination = Path(directory)
    destination.mkdir(parents=True, exist_ok=True)
    payload = _payload(source)
    paths: list[Path] = []
    omitted: list[dict[str, str]] = []
    all_sources = (source, *comparison_sources)
    if _has_trajectories(payload):
        paths.append(plot_flown_trajectories(payload, destination / "flown-trajectories.png", dpi=dpi))
    else:
        omitted.append({"plot": "flown-trajectories", "reason": "trajectory tables are not present"})
    paths.append(plot_search_coverage(payload, destination / "search-coverage.png", dpi=dpi))
    paths.append(plot_terminal_capability(payload, destination / "terminal-capability.png", dpi=dpi))
    if _has_deployment_events(payload):
        paths.append(plot_deployment_timeline(payload, destination / "deployment-timeline.png", dpi=dpi))
        paths.append(plot_projected_area(payload, destination / "projected-area.png", dpi=dpi))
        paths.extend(
            (
                plot_parent_trajectory(payload, destination / "trajectory_parent.png", dpi=dpi),
                plot_children_trajectories(payload, destination / "trajectory_children.png", dpi=dpi),
                plot_deployment_timeline(payload, destination / "event_timeline.png", dpi=dpi),
                plot_terminal_capability(payload, destination / "capability_footprint.png", dpi=dpi),
                plot_search_coverage(payload, destination / "exploration_coverage.png", dpi=dpi),
                plot_projected_area(payload, destination / "projected_area.png", dpi=dpi),
            )
        )
        if len(all_sources) >= 2:
            paths.append(plot_fidelity_progression(all_sources, destination / "fidelity_comparison.png", dpi=dpi))
        artifact_paths = _deployment_telemetry_artifacts(payload, destination)
    else:
        artifact_paths = ()
    if len(all_sources) >= 2:
        paths.append(plot_fidelity_progression(all_sources, destination / "fidelity-progression.png", dpi=dpi))
    manifest = {
        "schema": "trajectory.reachability-plot-bundle/v1alpha1",
        "source_schema": payload.get("schema"),
        "fidelity": payload.get("fidelity", payload.get("study", {}).get("fidelity")),
        "dpi": dpi,
        "plots": [path.name for path in paths],
        "artifacts": [path.name for path in artifact_paths],
        "deployment_event_count": sum(len(sample.get("deployment_events", [])) for sample in _samples(payload)),
        "source_configuration_hash": _configuration_hash(payload),
        "search_space": payload.get("search_space", {}),
        "execution": payload.get("execution", {}),
        "omitted": omitted,
    }
    manifest_path = destination / "plot-manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return ReachabilityPlotReport(tuple(paths), manifest_path, tuple(omitted), artifact_paths)
    ####


def _payload(source: ReachabilityEnvelope | Payload) -> dict[str, Any]:
    if isinstance(source, ReachabilityEnvelope):
        return source.as_dict(include_trajectories=True)
    return dict(source)
    ####


def _samples(payload: Payload) -> list[Mapping[str, Any]]:
    samples = payload.get("samples", [])
    if not isinstance(samples, list):
        raise ValueError("reachability artifact samples must be a list")
    return [sample for sample in samples if isinstance(sample, Mapping)]
    ####


def _axis_map(payload: Payload) -> dict[str, Mapping[str, Any]]:
    search_space = payload.get("search_space", {})
    axes = search_space.get("axes", []) if isinstance(search_space, Mapping) else []
    return {str(axis["name"]): axis for axis in axes if isinstance(axis, Mapping) and "name" in axis}
    ####


def _has_trajectories(payload: Payload) -> bool:
    return any(isinstance(sample.get("trajectory"), Mapping) for sample in _samples(payload))
    ####


def _has_deployment_events(payload: Payload) -> bool:
    """Return whether at least one candidate recorded a committed deployment."""

    return any(bool(sample.get("deployment_events")) for sample in _samples(payload))
    ####


def _configuration_hash(payload: Payload) -> str:
    """Hash resolved study, vehicle, search, and execution configuration."""

    selected = {
        "study": payload.get("study", {}),
        "search_space": payload.get("search_space", {}),
        "execution": payload.get("execution", {}),
        "provenance": payload.get("provenance", {}),
    }
    encoded = json.dumps(selected, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
    ####


def _columns(fields: list[Any], rows: list[Any]) -> dict[str, list[Any]]:
    columns: dict[str, list[Any]] = {str(field): [] for field in fields}
    for row in rows:
        row_columns = _row_columns(fields, row)
        for field, values in columns.items():
            values.append(row_columns.get(field))
    return columns
    ####


def _row_columns(fields: list[Any], row: Any) -> dict[str, Any]:
    if not isinstance(row, list):
        return {}
    return {str(field): value for field, value in zip(fields, row, strict=False)}
    ####


def _status_color(status: str) -> str:
    return {
        "feasible": "#15803d",
        "infeasible": "#dc2626",
        "invalid": "#7c3aed",
    }.get(status, "#64748b")
    ####


def _series_color(index: int) -> str:
    return ("#2563eb", "#ea580c", "#0f766e", "#7c3aed", "#be123c", "#475569")[index % 6]
    ####


def _range_of(sample: Mapping[str, Any]) -> float:
    terminal = sample.get("terminal_state", {})
    position = terminal.get("position_m", []) if isinstance(terminal, Mapping) else []
    return math.hypot(float(position[0]), float(position[1])) if len(position) >= 2 else 0.0
    ####


def _save_figure(figure: Any, path: str | Path, dpi: int) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, format="png", dpi=dpi, bbox_inches="tight")
    return destination
    ####


def _get_matplotlib() -> Any:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    return plt
    ####


__all__ = [
    "ReachabilityPlotReport",
    "load_reachability_artifact",
    "plot_fidelity_progression",
    "plot_flown_trajectories",
    "plot_deployment_timeline",
    "plot_children_trajectories",
    "plot_projected_area",
    "plot_parent_trajectory",
    "plot_search_coverage",
    "plot_terminal_capability",
    "render_reachability_plot_bundle",
]
####
