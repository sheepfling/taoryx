"""Mapping of optimize blocks to numerical callbacks."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass

from taoryx.optimization import OptimizationResult, han_powell_rqp


@dataclass(frozen=True, slots=True)
class OptimizeRuntime:
    objective: Callable[[tuple[float, ...]], float]
    bounds: tuple[tuple[float, float], ...]
    equality_constraints: tuple[Callable[[tuple[float, ...]], float], ...] = ()
    inequality_constraints: tuple[Callable[[tuple[float, ...]], float], ...] = ()

    def run(self, initial: Sequence[float], *, max_iterations: int = 100) -> OptimizationResult:
        return han_powell_rqp(self.objective, initial, self.bounds, equality_constraints=self.equality_constraints, inequality_constraints=self.inequality_constraints, max_iterations=max_iterations)
    ####
####


def resolve_optimize_block(objective: Callable[[tuple[float, ...]], float], bounds: Sequence[tuple[float, float]], *, equality_constraints: Sequence[Callable[[tuple[float, ...]], float]] = (), inequality_constraints: Sequence[Callable[[tuple[float, ...]], float]] = ()) -> OptimizeRuntime:
    """Resolve an optimize block into a repeatable optimization runner."""

    return OptimizeRuntime(objective, tuple(bounds), tuple(equality_constraints), tuple(inequality_constraints))
####
