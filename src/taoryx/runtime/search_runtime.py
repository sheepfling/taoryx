"""Search-loop orchestration over a restartable scalar callback."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from taoryx.searches import secant_bracketed_root


@dataclass(frozen=True, slots=True)
class SearchLoopResult:
    parameter: float
    residual: float
    evaluations: int
    converged: bool


def execute_search_loops(function: Callable[[float], float], bounds: tuple[float, float], *, tolerance: float = 1e-8, max_iterations: int = 100, restart: Callable[[float], None] | None = None) -> SearchLoopResult:
    """Drive bounded secant search and notify the trajectory after updates."""

    result = secant_bracketed_root(function, bounds, tolerance, max_iterations=max_iterations)
    if restart is not None:
        restart(result.root)
    return SearchLoopResult(result.root, result.residual, result.iterations + 1, result.converged)
####
