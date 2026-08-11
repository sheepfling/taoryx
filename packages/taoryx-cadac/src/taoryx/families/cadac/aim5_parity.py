"""AIM5-specific bridge from CADAC ``plot*.asc`` output to Python parity metrics."""

from __future__ import annotations

from pathlib import Path

from pydantic import Field

from .aim5 import aim5_plot_projection, load_aim5_source_definition, run_aim5_source_compatibility
from .bundle import CadacSourceArtifact
from .input_ast import CadacModel
from .parity import CadacParityReport, compare_cadac_plot_channels, parse_cadac_plot_file


class Aim5PlotParityResult(CadacModel):
    """A source-grounded parity comparison for one AIM5 plot stream."""

    schema_id: str = "taoryx.cadac.aim5-plot-parity/v0alpha1"
    input_artifacts: tuple[CadacSourceArtifact, ...] = Field(min_length=3)
    plot_source_name: str = Field(min_length=1)
    source_sample_count: int = Field(ge=0)
    python_sample_count: int = Field(ge=0)
    compared_channels: tuple[str, ...] = Field(min_length=1)
    report: CadacParityReport


####


def compare_aim5_source_plot(
    input_path: str | Path,
    plot_path: str | Path,
    *,
    absolute_tolerance: float = 1.0e-6,
    relative_tolerance: float = 1.0e-6,
) -> Aim5PlotParityResult:
    """Run the Python compatibility slice and compare common CADAC plot channels."""

    definition = load_aim5_source_definition(input_path)
    source_plot = parse_cadac_plot_file(plot_path)
    source_times = tuple(value for value in source_plot.column("time") if value >= 0.0)
    if not source_times:
        raise ValueError("CADAC plot contains no nonnegative source time samples")
    ####
    sample_step = definition.plot_step_s or _first_positive_spacing(source_times) or definition.integration_step_s
    run = run_aim5_source_compatibility(definition, sample_step_s=sample_step)
    projection = aim5_plot_projection(run, times_s=source_times)
    source_names = {name.casefold(): name for name in source_plot.channels}
    expected = {name: values for name, values in projection.items() if name.casefold() in source_names}
    if not expected:
        raise ValueError("CADAC plot and AIM5 compatibility projection share no channels")
    ####
    normalized_expected = {source_names[name.casefold()]: values for name, values in expected.items()}
    report = compare_cadac_plot_channels(
        source_plot,
        normalized_expected,
        absolute_tolerance=absolute_tolerance,
        relative_tolerance=relative_tolerance,
    )
    return Aim5PlotParityResult(
        input_artifacts=definition.source_artifacts,
        plot_source_name=source_plot.source_name,
        source_sample_count=len(source_times),
        python_sample_count=len(run.samples),
        compared_channels=tuple(normalized_expected),
        report=report,
    )


####


def _first_positive_spacing(values: tuple[float, ...]) -> float | None:
    for left, right in zip(values, values[1:], strict=False):
        if right > left:
            return right - left
        ####
    ####
    return None


####


__all__ = ["Aim5PlotParityResult", "compare_aim5_source_plot"]
