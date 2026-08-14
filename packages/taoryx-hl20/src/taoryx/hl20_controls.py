"""Explicit HL-20 surface command and actuator contracts."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

HL20_SURFACE_NAMES = (
    "upper_left_body_flap",
    "lower_left_body_flap",
    "upper_right_body_flap",
    "lower_right_body_flap",
    "left_wing_flap",
    "right_wing_flap",
    "rudder",
)

HL20_SOURCE_SURFACE_BOUNDS_DEG = {
    "upper_left_body_flap": (-60.0, 0.0),
    "lower_left_body_flap": (0.0, 60.0),
    "upper_right_body_flap": (-60.0, 0.0),
    "lower_right_body_flap": (0.0, 60.0),
    "left_wing_flap": (-30.0, 30.0),
    "right_wing_flap": (-30.0, 30.0),
    "rudder": (-30.0, 30.0),
}


@dataclass(frozen=True, slots=True)
class HL20ActuatorTrace:
    """One requested-to-achieved surface realization."""

    requested_deg: tuple[tuple[str, float], ...]
    achieved_deg: tuple[tuple[str, float], ...]
    rate_deg_s: tuple[tuple[str, float], ...]
    saturated: tuple[str, ...]
    rate_limited: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        return {
            "requested_deg": dict(self.requested_deg),
            "achieved_deg": dict(self.achieved_deg),
            "rate_deg_s": dict(self.rate_deg_s),
            "saturated": list(self.saturated),
            "rate_limited": list(self.rate_limited),
        }


@dataclass(frozen=True, slots=True)
class HL20ActuatorProfile:
    """Bounded first-order or direct surface realization policy."""

    profile_id: str = "hl20.reference_first_order.v1"
    mode: str = "first_order"
    rate_limit_deg_s: float = 60.0
    time_constant_s: float = 0.15
    bounds_deg: tuple[tuple[str, tuple[float, float]], ...] = tuple(
        (name, bounds) for name, bounds in HL20_SOURCE_SURFACE_BOUNDS_DEG.items()
    )

    def __post_init__(self) -> None:
        if self.mode not in {"direct", "first_order"}:
            raise ValueError("HL-20 actuator mode must be 'direct' or 'first_order'")
        if not math.isfinite(self.rate_limit_deg_s) or self.rate_limit_deg_s <= 0.0:
            raise ValueError("HL-20 actuator rate limit must be positive and finite")
        if not math.isfinite(self.time_constant_s) or self.time_constant_s <= 0.0:
            raise ValueError("HL-20 actuator time constant must be positive and finite")
        names = tuple(name for name, _ in self.bounds_deg)
        if names != HL20_SURFACE_NAMES:
            raise ValueError("HL-20 actuator bounds must declare all seven surfaces in canonical order")
        for name, (lower, upper) in self.bounds_deg:
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise ValueError(f"invalid HL-20 actuator bounds for {name}")

    def realize(
        self,
        requested_deg: Mapping[str, float],
        previous_achieved_deg: Mapping[str, float] | None = None,
        *,
        duration_s: float,
    ) -> HL20ActuatorTrace:
        """Apply position and rate limits and return complete actuator telemetry."""

        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("actuator realization duration must be positive and finite")
        previous = previous_achieved_deg or {}
        requested: dict[str, float] = {}
        achieved: dict[str, float] = {}
        rates: dict[str, float] = {}
        saturated: list[str] = []
        rate_limited: list[str] = []
        for name, (lower, upper) in self.bounds_deg:
            value = float(requested_deg.get(name, 0.0))
            if not math.isfinite(value):
                raise ValueError(f"requested HL-20 actuator command is not finite: {name}")
            bounded = min(upper, max(lower, value))
            requested[name] = value
            if bounded != value:
                saturated.append(name)
            prior = float(previous.get(name, 0.0))
            if self.mode == "direct":
                result = bounded
            else:
                first_order = prior + (bounded - prior) * (1.0 - math.exp(-duration_s / self.time_constant_s))
                maximum_delta = self.rate_limit_deg_s * duration_s
                result = min(prior + maximum_delta, max(prior - maximum_delta, first_order))
                if abs(result - bounded) > 1.0e-9:
                    rate_limited.append(name)
            achieved[name] = result
            rates[name] = (result - prior) / duration_s
        return HL20ActuatorTrace(
            tuple(requested.items()),
            tuple(achieved.items()),
            tuple(rates.items()),
            tuple(saturated),
            tuple(rate_limited),
        )


__all__ = [
    "HL20ActuatorProfile",
    "HL20ActuatorTrace",
    "HL20_SOURCE_SURFACE_BOUNDS_DEG",
    "HL20_SURFACE_NAMES",
]
