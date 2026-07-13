"""Survey-case expansion contracts."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from itertools import product


def generate_survey_cases(settings: Mapping[str, Sequence[float] | tuple[float, float, float]]) -> tuple[dict[str, float], ...]:
    """Expand explicit values or ``(low, high, increment)`` ranges."""

    axes: list[tuple[str, tuple[float, ...]]] = []
    for name, raw in settings.items():
        values = _expand(raw)
        if not values:
            raise ValueError(f"survey axis {name!r} is empty")
        axes.append((name, values))
    return tuple(dict(zip((name for name, _ in axes), values, strict=True)) for values in product(*(values for _, values in axes)))
####


def _expand(raw: Sequence[float] | tuple[float, float, float]) -> tuple[float, ...]:
    if len(raw) != 3:
        return tuple(float(value) for value in raw)
    low, high, increment = (float(value) for value in raw)
    if increment == 0.0 or (high - low) * increment < 0.0:
        raise ValueError("survey increment has the wrong sign")
    values: list[float] = []
    current = low
    compare = (lambda value: value <= high + 1e-12) if increment > 0 else (lambda value: value >= high - 1e-12)
    while compare(current):
        values.append(current)
        current += increment
        if len(values) > 100000:
            raise ValueError("survey range is too large")
    if values and abs(values[-1] - high) <= 1e-12:
        values[-1] = high
    return tuple(values)
####
