"""Matplotlib views for Alpha 3 reachability envelope artifacts."""

from __future__ import annotations

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
        for status in ("feasible", "infeasible", "invalid"):
            if status in seen_statuses:
                handles.append(Line2D([0], [0], color=_status_color(status), linewidth=2.0, label=status))
        if handles:
            axes[0].legend(handles=handles, loc="best", fontsize=8)
        return _save_figure(figure, path, dpi)
    finally:
        plt.close(figure)
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
    if _has_trajectories(payload):
        paths.append(plot_flown_trajectories(payload, destination / "flown-trajectories.png", dpi=dpi))
    else:
        omitted.append({"plot": "flown-trajectories", "reason": "trajectory tables are not present"})
    paths.append(plot_search_coverage(payload, destination / "search-coverage.png", dpi=dpi))
    paths.append(plot_terminal_capability(payload, destination / "terminal-capability.png", dpi=dpi))
    all_sources = (source, *comparison_sources)
    if len(all_sources) >= 2:
        paths.append(plot_fidelity_progression(all_sources, destination / "fidelity-progression.png", dpi=dpi))
    manifest = {
        "schema": "trajectory.reachability-plot-bundle/v1alpha1",
        "source_schema": payload.get("schema"),
        "fidelity": payload.get("fidelity", payload.get("study", {}).get("fidelity")),
        "dpi": dpi,
        "plots": [path.name for path in paths],
        "omitted": omitted,
    }
    manifest_path = destination / "plot-manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return ReachabilityPlotReport(tuple(paths), manifest_path, tuple(omitted))
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
    "plot_search_coverage",
    "plot_terminal_capability",
    "render_reachability_plot_bundle",
]
####
