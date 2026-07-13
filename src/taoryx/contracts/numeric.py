"""Numerical policy shared by deterministic algorithm implementations."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class NumericalTolerances:
    """Absolute/relative tolerances and iteration limits for one run."""

    absolute: float = 1e-12
    relative: float = 1e-10
    zero: float = 1e-15
    max_iterations: int = 100

    def __post_init__(self) -> None:
        if self.absolute < 0.0 or self.relative < 0.0 or self.zero < 0.0:
            raise ValueError("numerical tolerances must be nonnegative")
        if self.max_iterations < 1:
            raise ValueError("max_iterations must be positive")
        ####
####
