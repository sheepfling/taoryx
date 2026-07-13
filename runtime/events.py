"""Event detection, state discontinuity, and ordered when-condition helpers."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass

from .common import EventCondition, RuntimeState


@dataclass(frozen=True, slots=True)
class EventCrossing:
    name: str
    time: float
    residual: float


def refine_segment_final_condition(
    start: RuntimeState,
    end: RuntimeState,
    conditions: Iterable[EventCondition],
    *,
    tolerance: float = 1e-9,
    max_iterations: int = 64,
) -> tuple[EventCrossing, ...]:
    """Bracket and secant-refine all sign-changing event residuals."""

    if end.time < start.time or tolerance <= 0.0:
        raise ValueError("event interval and tolerance must be valid")
    crossings: list[EventCrossing] = []
    for condition in conditions:
        left, right = condition.function(start), condition.function(end)
        if abs(left) <= tolerance:
            crossings.append(EventCrossing(condition.name, start.time, left))
            continue
        if left * right > 0.0:
            continue
        lo, hi = start, end
        lo_value, hi_value = left, right
        for _ in range(max_iterations):
            fraction = 0.5 if hi_value == lo_value else -lo_value / (hi_value - lo_value)
            fraction = min(1.0, max(0.0, fraction))
            mid = _interpolate_state(lo, hi, fraction)
            value = condition.function(mid)
            if abs(value) <= tolerance or hi.time - lo.time <= tolerance:
                crossings.append(EventCrossing(condition.name, mid.time, value))
                break
            if lo_value * value <= 0.0:
                hi, hi_value = mid, value
            else:
                lo, lo_value = mid, value
        else:
            crossings.append(EventCrossing(condition.name, mid.time, value))
    return tuple(sorted(crossings, key=lambda event: (event.time, event.name)))
####


def apply_state_discontinuity(state: RuntimeState, increments: Sequence[float], *, named_updates: dict[str, float] | None = None) -> RuntimeState:
    """Apply an instantaneous state increment while preserving the time/frame."""

    if len(increments) != len(state.values):
        raise ValueError("state increment dimension does not match state")
    named = dict(state.named)
    named.update(named_updates or {})
    return RuntimeState(state.time, tuple(a + b for a, b in zip(state.values, increments, strict=True)), state.frame, named)
####


def evaluate_when_conditions(state: RuntimeState, conditions: Iterable[EventCondition]) -> EventCondition | None:
    """Return the first true condition in source order."""

    for condition in conditions:
        if condition.function(state) >= 0.0:
            return condition
    return None
####


def _interpolate_state(start: RuntimeState, end: RuntimeState, fraction: float) -> RuntimeState:
    values = tuple(a + fraction * (b - a) for a, b in zip(start.values, end.values, strict=True))
    return RuntimeState(start.time + fraction * (end.time - start.time), values, start.frame, start.named)
####
