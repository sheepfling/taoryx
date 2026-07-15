"""Matplotlib debug artifacts for family-specific run telemetry."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from taoryx.family_debugging import DebugFamily, FamilyDebugPlan, build_family_debug_plan
from taoryx.outputs import RunArtifact, VehicleTelemetry


@dataclass(frozen=True, slots=True)
class FamilyDebugRenderReport:
    """Files emitted by one family debug render pass."""

    family: DebugFamily
    vehicle_id: str
    plot_paths: tuple[Path, ...]
    log_path: Path
    plan: FamilyDebugPlan


def render_family_debug_artifacts(
    artifact: RunArtifact,
    output_dir: str | Path,
    family: DebugFamily | str,
    *,
    vehicle_id: str | None = None,
    dpi: int = 150,
) -> tuple[FamilyDebugRenderReport, ...]:
    """Render family panels and a structured debug log for an artifact.

    Missing required channels are recorded in the log and do not produce a
    misleading plot.  Optional channels are plotted when present and are
    retained as diagnostics when absent.
    """

    selected_family = DebugFamily(family)
    selected_ids = (vehicle_id,) if vehicle_id is not None else tuple(artifact.vehicles)
    reports: list[FamilyDebugRenderReport] = []
    for selected_id in selected_ids:
        vehicle = artifact.vehicles[selected_id]
        plan = build_family_debug_plan(artifact, selected_family, vehicle_id=selected_id)
        vehicle_dir = Path(output_dir) / selected_family.value / _slugify(selected_id)
        vehicle_dir.mkdir(parents=True, exist_ok=True)
        plot_paths = tuple(
            _render_panel(vehicle, panel, plan, vehicle_dir, dpi=dpi)
            for panel in plan.panels
            if panel.renderable
            and (
                _panel_has_values(vehicle, panel.spec.required_channels + panel.spec.optional_channels)
                or _panel_has_events(vehicle, panel.spec.event_kinds, plan)
            )
        )
        log_path = vehicle_dir / "debug-log.json"
        log_path.write_text(_debug_log(artifact, vehicle, plan, plot_paths), encoding="utf-8")
        reports.append(FamilyDebugRenderReport(selected_family, selected_id, plot_paths, log_path, plan))
    return tuple(reports)
####


def _render_panel(vehicle: VehicleTelemetry, panel: Any, plan: FamilyDebugPlan, output_dir: Path, *, dpi: int) -> Path:
    plt = _get_matplotlib()
    figure, axis = plt.subplots(figsize=(8.0, 4.5), constrained_layout=True)
    plotted = False
    names = panel.spec.required_channels + panel.spec.optional_channels
    for name in names:
        channel = vehicle.channels.get(name)
        if channel is None:
            continue
        samples = [(time, value) for time, value in zip(vehicle.times, channel.values, strict=True) if value is not None]
        if not samples:
            continue
        axis.plot([sample[0] for sample in samples], [sample[1] for sample in samples], label=_channel_label(channel))
        plotted = True
    for event in plan.events:
        if event.vehicle == vehicle.vehicle_id:
            axis.axvline(event.time, color="0.45", linewidth=0.8, linestyle="--", alpha=0.7)
            axis.text(event.time, 1.01, event.name, rotation=90, transform=axis.get_xaxis_transform(), fontsize=7, va="bottom")
    axis.set_title(panel.spec.title)
    axis.set_xlabel("time (s)")
    axis.grid(True, alpha=0.25)
    if plotted:
        axis.legend(loc="best", fontsize=8)
    path = output_dir / f"{_slugify(panel.spec.panel_id)}.png"
    figure.savefig(path, dpi=dpi, format="png", bbox_inches="tight")
    plt.close(figure)
    return path
####


def _debug_log(artifact: RunArtifact, vehicle: VehicleTelemetry, plan: FamilyDebugPlan, plot_paths: tuple[Path, ...]) -> str:
    rendered = {path.stem for path in plot_paths}
    payload = {
        "schema_version": 1,
        "problem": artifact.problem,
        "family": plan.family.value,
        "vehicle_id": vehicle.vehicle_id,
        "vehicle_name": vehicle.name,
        "panels": [
            {
                "panel_id": panel.spec.panel_id,
                "title": panel.spec.title,
                "renderable": panel.renderable,
                "rendered": panel.spec.panel_id in rendered,
                "required_channels": list(panel.spec.required_channels),
                "optional_channels": list(panel.spec.optional_channels),
                "available_channels": list(panel.available_channels),
                "missing_required": list(panel.missing_required),
                "missing_optional": list(panel.missing_optional),
            }
            for panel in plan.panels
        ],
        "events": [event.model_dump(mode="json") for event in plan.events if event.vehicle == vehicle.vehicle_id],
        "diagnostics": list(plan.diagnostics),
        "plots": [path.name for path in plot_paths],
    }
    return json.dumps(payload, indent=2, sort_keys=True) + "\n"
####


def _panel_has_values(vehicle: VehicleTelemetry, names: tuple[str, ...]) -> bool:
    return any(any(value is not None for value in vehicle.channels[name].values) for name in names if name in vehicle.channels)
####


def _panel_has_events(vehicle: VehicleTelemetry, event_kinds: tuple[str, ...], plan: FamilyDebugPlan) -> bool:
    return bool(event_kinds) and any(event.vehicle == vehicle.vehicle_id and event.kind in event_kinds for event in plan.events)
####


def _channel_label(channel: Any) -> str:
    return f"{channel.semantic_name} ({channel.unit})" if channel.unit else channel.semantic_name
####


def _get_matplotlib() -> Any:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    return plt
####


def _slugify(value: str) -> str:
    pieces = [character if character.isalnum() or character in {"-", "_"} else "-" for character in value.casefold()]
    slug = "".join(pieces).strip("-")
    while "--" in slug:
        slug = slug.replace("--", "-")
    return slug or "vehicle"
####
