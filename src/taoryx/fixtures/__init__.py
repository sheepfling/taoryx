"""Fixture-generation helpers for checked-in problem-file dumps."""

from .simple_aero_dumps import (
    SimpleAeroProblemSpec,
    SimpleAeroSegmentSpec,
    load_spec,
    render_problem_file,
    render_workspace,
    write_workspace,
)
from .simple_aero_trajectories import (
    SimpleAeroSolutionMetadata,
    SimpleAeroTrajectoryProblemSpec,
    SimpleAeroTrajectorySpec,
    SimpleAeroTrajectoryWorkspaceSpec,
)
from .simple_aero_trajectories import (
    load_spec as load_trajectory_spec,
)
from .simple_aero_trajectories import (
    render_problem_file as render_trajectory_problem_file,
)
from .simple_aero_trajectories import (
    render_workspace as render_trajectory_workspace,
)
from .simple_aero_trajectories import (
    write_workspace as write_trajectory_workspace,
)
from .table_example_artifacts import TableArtifactPlotSeries, build_run_plot_series

__all__ = [
    "SimpleAeroProblemSpec",
    "SimpleAeroSegmentSpec",
    "SimpleAeroTrajectoryProblemSpec",
    "SimpleAeroTrajectorySpec",
    "SimpleAeroTrajectoryWorkspaceSpec",
    "SimpleAeroSolutionMetadata",
    "load_spec",
    "load_trajectory_spec",
    "render_problem_file",
    "render_workspace",
    "render_trajectory_problem_file",
    "render_trajectory_workspace",
    "TableArtifactPlotSeries",
    "build_run_plot_series",
    "write_workspace",
    "write_trajectory_workspace",
]
####
