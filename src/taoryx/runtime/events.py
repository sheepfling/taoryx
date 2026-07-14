"""Event detection and state-discontinuity contracts."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass

from .common import EventCondition, RuntimeState


@dataclass(frozen=True, slots=True)
class EventCrossing:
    name: str
    time: float
    residual: float
    action: str = "stop"


def refine_segment_final_condition(start: RuntimeState, end: RuntimeState, conditions: Iterable[EventCondition], *, tolerance: float = 1e-9, max_iterations: int = 64) -> tuple[EventCrossing, ...]:
    """Refine sign-changing final conditions by a safeguarded secant bracket."""

    if end.time < start.time or tolerance <= 0.0:
        raise ValueError("event interval and tolerance must be valid")
    crossings: list[tuple[int, EventCrossing]] = []
    for condition_index, condition in enumerate(conditions):
        left_value, right_value = condition.function(start), condition.function(end)
        if abs(left_value) <= tolerance:
            if right_value <= left_value and abs(right_value) <= tolerance:
                continue
            crossings.append((condition_index, EventCrossing(condition.name, start.time, left_value, condition.action)))
            continue
        if left_value * right_value > 0.0:
            continue
        lo, hi = start, end
        lo_value, hi_value = left_value, right_value
        mid, mid_value = lo, lo_value
        for _ in range(max_iterations):
            fraction = 0.5 if hi_value == lo_value else -lo_value / (hi_value - lo_value)
            mid = _interpolate_state(lo, hi, min(1.0, max(0.0, fraction)))
            mid_value = condition.function(mid)
            if abs(mid_value) <= tolerance or hi.time - lo.time <= tolerance:
                break
            if lo_value * mid_value <= 0.0:
                hi, hi_value = mid, mid_value
            else:
                lo, lo_value = mid, mid_value
        crossings.append((condition_index, EventCrossing(condition.name, round(mid.time, 12), mid_value, condition.action)))
    return tuple(event for _, event in sorted(crossings, key=lambda item: (item[1].time, item[0])))
####


def apply_state_discontinuity(state: RuntimeState, increments: Sequence[float], *, named_updates: dict[str, float] | None = None) -> RuntimeState:
    """Apply an instantaneous increment while preserving time and frame."""

    if len(increments) != len(state.values):
        raise ValueError("state increment dimension does not match state")
    values = tuple(a + b for a, b in zip(state.values, increments, strict=True))
    named = dict(state.named)
    for name, value in zip(state.value_names, values, strict=False):
        named[name] = value
    named.update(named_updates or {})
    return RuntimeState(state.time, values, state.frame, named, state.value_names, state.segment_endpoints)
####


def evaluate_when_conditions(state: RuntimeState, conditions: Iterable[EventCondition]) -> EventCondition | None:
    """Return the first true condition in source order."""

    return next((condition for condition in conditions if condition.function(state) >= 0.0), None)
####


def _interpolate_state(start: RuntimeState, end: RuntimeState, fraction: float) -> RuntimeState:
    return start.with_values(tuple(a + fraction * (b - a) for a, b in zip(start.values, end.values, strict=True)), time=start.time + fraction * (end.time - start.time))
####
