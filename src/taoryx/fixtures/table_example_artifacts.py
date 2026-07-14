"""Plot-series helpers for checked-in table example run artifacts."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from taoryx.outputs import RunArtifact


@dataclass(frozen=True, slots=True)
class TableArtifactPlotSeries:
    """A deterministic 1D plot series built from a run artifact channel."""

    filename: str
    title: str
    axis_label: str
    value_label: str
    axis: tuple[float, ...]
    values: tuple[float, ...]


def build_run_plot_series(artifact: RunArtifact) -> list[TableArtifactPlotSeries]:
    """Build one plot series per populated telemetry channel."""

    series: list[TableArtifactPlotSeries] = []
    problem_name = Path(artifact.problem).name
    problem_slug = _slugify(Path(problem_name).stem)
    for vehicle_id, vehicle in artifact.vehicles.items():
        vehicle_slug = _slugify(f"{problem_slug}-{vehicle_id}-{vehicle.name}")
        for channel in sorted(vehicle.channels.values(), key=lambda item: (item.semantic_name, item.source_name)):
            samples = tuple(
                (time, value)
                for time, value in zip(vehicle.times, channel.values, strict=True)
                if value is not None
            )
            if not samples:
                continue
            axis = tuple(time for time, _ in samples)
            values = tuple(value for _, value in samples)
            unit_label = f" ({channel.unit})" if channel.unit else ""
            series.append(
                TableArtifactPlotSeries(
                    filename=f"{vehicle_slug}-{_slugify(channel.semantic_name)}.png",
                    title=f"{problem_name} / {vehicle.name} / {channel.semantic_name}",
                    axis_label="time (s)",
                    value_label=f"{channel.semantic_name}{unit_label}",
                    axis=axis,
                    values=values,
                )
            )
    return series
####


def _slugify(value: str) -> str:
    pieces = [character if character.isalnum() or character in {"-", "_"} else "-" for character in value.casefold()]
    slug = "".join(pieces).strip("-")
    while "--" in slug:
        slug = slug.replace("--", "-")
    return slug or "series"
####
