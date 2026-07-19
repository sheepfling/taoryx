"""Reusable phase-window checks for long vehicle validation scenarios."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Mapping, Sequence

Direction = Literal["increasing", "decreasing"]


@dataclass(frozen=True, slots=True)
class PhaseWindow:
    """Named interval used to scope a trajectory acceptance check."""

    name: str
    start_s: float
    end_s: float

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("phase name must not be empty")
        if self.start_s < 0.0 or self.end_s <= self.start_s:
            raise ValueError("phase window must have positive ordered bounds")
    ####

    def select(self, history: Sequence[Mapping[str, float]]) -> tuple[Mapping[str, float], ...]:
        """Return telemetry samples whose time lies within this phase."""

        samples = tuple(
            sample
            for sample in history
            if self.start_s <= float(sample.get("time_s", -1.0)) <= self.end_s
        )
        if not samples:
            raise AssertionError(f"phase {self.name!r} has no telemetry samples")
        return samples
    ####


def require_channel(samples: Sequence[Mapping[str, float]], channel: str) -> tuple[float, ...]:
    """Return one finite channel or raise an actionable validation error."""

    values: list[float] = []
    for sample in samples:
        if channel not in sample:
            raise AssertionError(f"required validation channel {channel!r} is missing")
        value = float(sample[channel])
        if value != value or value in (float("inf"), float("-inf")):
            raise AssertionError(f"validation channel {channel!r} is not finite")
        values.append(value)
    return tuple(values)
####


def require_bounded(
    samples: Sequence[Mapping[str, float]],
    channel: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> None:
    """Require every value in a phase to remain inside an inclusive band."""

    values = require_channel(samples, channel)
    if minimum is not None and min(values) < minimum:
        raise AssertionError(f"{channel} fell below {minimum}: {min(values)}")
    if maximum is not None and max(values) > maximum:
        raise AssertionError(f"{channel} exceeded {maximum}: {max(values)}")
####


def require_monotonic(
    samples: Sequence[Mapping[str, float]],
    channel: str,
    direction: Direction,
    *,
    tolerance: float = 0.0,
) -> None:
    """Require a channel to move in one direction within a numeric tolerance."""

    values = require_channel(samples, channel)
    deltas = tuple(right - left for left, right in zip(values, values[1:], strict=False))
    violation = max((delta for delta in deltas if (delta < -tolerance if direction == "increasing" else delta > tolerance)), default=0.0)
    if violation:
        raise AssertionError(f"{channel} is not {direction} within tolerance {tolerance}")
####


def require_net_change(
    samples: Sequence[Mapping[str, float]],
    channel: str,
    direction: Direction,
    *,
    minimum: float = 0.0,
) -> None:
    """Require the net phase displacement to have the expected direction."""

    values = require_channel(samples, channel)
    change = values[-1] - values[0]
    signed_change = change if direction == "increasing" else -change
    if signed_change < minimum:
        raise AssertionError(f"{channel} net change was {change}, expected {direction} by {minimum}")
####


def require_change_of_sign(
    samples: Sequence[Mapping[str, float]],
    channel: str,
    *,
    minimum_before: float = 0.0,
    minimum_after: float = 0.0,
) -> None:
    """Require a signed response with nontrivial values on both sides."""

    values = require_channel(samples, channel)
    midpoint = len(values) // 2
    before = values[:midpoint]
    after = values[midpoint:]
    if not before or not after:
        raise AssertionError(f"{channel} did not exhibit the required sign change")
    forward = min(before) <= -minimum_before and max(after) >= minimum_after
    reverse = max(before) >= minimum_before and min(after) <= -minimum_after
    if not (forward or reverse):
        raise AssertionError(f"{channel} did not exhibit the required sign change")
####


__all__ = ["Direction", "PhaseWindow", "require_bounded", "require_change_of_sign", "require_channel", "require_monotonic", "require_net_change"]
