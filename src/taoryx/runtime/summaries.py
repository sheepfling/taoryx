"""History summary operations."""

from __future__ import annotations

from collections.abc import Sequence


def evaluate_summary(values: Sequence[float], operation: str, *, argument: float | None = None) -> float:
    """Evaluate one first/last/min/max/interpolation summary operation."""

    if not values:
        raise ValueError("summary requires at least one sample")
    numbers = tuple(float(value) for value in values)
    name = operation.casefold()
    if name == "first":
        return numbers[0]
    if name == "last":
        return numbers[-1]
    if name in {"min", "minfit"}:
        return min(numbers)
    if name in {"max", "maxfit"}:
        return max(numbers)
    if name == "interpolate":
        if argument is None or not 0.0 <= argument <= 1.0:
            raise ValueError("interpolation argument must be a normalized value")
        index = argument * (len(numbers) - 1)
        lower = min(len(numbers) - 1, int(index))
        upper = min(len(numbers) - 1, lower + 1)
        return numbers[lower] + (index - lower) * (numbers[upper] - numbers[lower])
    raise ValueError(f"unsupported summary operation: {operation}")
####
