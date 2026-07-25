"""Construction of the resolved runtime vehicle graph."""

from __future__ import annotations

from collections.abc import Iterable

from .common import RuntimeProblem, RuntimeVehicle
from .sensor_clock import SensorClockSpec


def build_runtime_problem(
    vehicles: Iterable[RuntimeVehicle],
    *,
    print_times: Iterable[float] = (),
    table_knots: Iterable[float] = (),
    required_truth_times: Iterable[float] = (),
    sensor_clocks: Iterable[SensorClockSpec] = (),
    final_time: float | None = None,
) -> RuntimeProblem:
    """Build and validate a name-indexed graph of direct/dependent vehicles."""

    records = tuple(vehicles)
    names = [vehicle.name for vehicle in records]
    if len(set(names)) != len(names):
        raise ValueError("vehicle names must be unique")
    graph = {vehicle.name: vehicle for vehicle in records}
    for vehicle in records:
        unknown = set(vehicle.dependencies) - graph.keys()
        if unknown:
            raise ValueError(f"unknown vehicle dependency: {sorted(unknown)!r}")
    _assert_acyclic(graph)
    return RuntimeProblem(
        graph,
        tuple(sorted(print_times)),
        tuple(sorted(table_knots)),
        final_time,
        required_truth_times=tuple(sorted(required_truth_times)),
        sensor_clocks=tuple(sensor_clocks),
    )
####


def _assert_acyclic(graph: dict[str, RuntimeVehicle]) -> None:
    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(name: str) -> None:
        if name in visiting:
            raise ValueError("vehicle dependency cycle")
        if name in visited:
            return
        visiting.add(name)
        for dependency in graph[name].dependencies:
            visit(dependency)
        visiting.remove(name)
        visited.add(name)

    for name in graph:
        visit(name)
    ####
####
