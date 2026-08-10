"""Lazy fixture facade spanning core and optional model packages."""

from __future__ import annotations

from importlib import import_module
from pkgutil import extend_path
from typing import Final

__path__ = extend_path(__path__, __name__)

_EXPORTS: Final[dict[str, tuple[str, str]]] = {
    "SimpleAeroProblemSpec": (".simple_aero_dumps", "SimpleAeroProblemSpec"),
    "SimpleAeroSegmentSpec": (".simple_aero_dumps", "SimpleAeroSegmentSpec"),
    "SimpleAeroSolutionMetadata": (".simple_aero_trajectories", "SimpleAeroSolutionMetadata"),
    "SimpleAeroTrajectoryProblemSpec": (".simple_aero_trajectories", "SimpleAeroTrajectoryProblemSpec"),
    "SimpleAeroTrajectorySpec": (".simple_aero_trajectories", "SimpleAeroTrajectorySpec"),
    "SimpleAeroTrajectoryWorkspaceSpec": (".simple_aero_trajectories", "SimpleAeroTrajectoryWorkspaceSpec"),
    "TableArtifactPlotSeries": (".table_example_artifacts", "TableArtifactPlotSeries"),
    "build_run_plot_series": (".table_example_artifacts", "build_run_plot_series"),
    "load_spec": (".simple_aero_dumps", "load_spec"),
    "load_trajectory_spec": (".simple_aero_trajectories", "load_spec"),
    "render_problem_file": (".simple_aero_dumps", "render_problem_file"),
    "render_trajectory_problem_file": (".simple_aero_trajectories", "render_problem_file"),
    "render_trajectory_workspace": (".simple_aero_trajectories", "render_workspace"),
    "render_workspace": (".simple_aero_dumps", "render_workspace"),
    "write_trajectory_workspace": (".simple_aero_trajectories", "write_workspace"),
    "write_workspace": (".simple_aero_dumps", "write_workspace"),
}


def __getattr__(name: str) -> object:
    """Resolve a fixture helper from its owning package on demand."""

    target = _EXPORTS.get(name)
    if target is None:
        raise AttributeError(name)
    module_name, attribute = target
    value = getattr(import_module(module_name, __name__), attribute)
    globals()[name] = value
    return value
    ####


def __dir__() -> list[str]:
    """Include lazy fixture exports in introspection."""

    return sorted({*globals(), *_EXPORTS})
    ####


__all__ = sorted(_EXPORTS)
####
