"""Auditable first-pass timing estimates for fixed-wing racetrack courses.

The estimate is deliberately kinematic.  It is a planning aid for selecting a
simulation horizon and phase windows before a vehicle-specific run; it is not a
performance guarantee and does not replace the truth-state evaluator.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class RacetrackTimingEstimate:
    """Derived phase durations for a closed, two-turn racetrack."""

    straight_time_s: float
    turn_time_s: float
    climb_time_s: float
    descent_time_s: float
    outbound_level_time_s: float
    inbound_level_time_s: float
    total_time_s: float

    def phase_durations(self) -> tuple[float, ...]:
        """Return durations in route order: climb, level, turns, descent, level."""

        return (
            self.climb_time_s,
            self.outbound_level_time_s,
            self.turn_time_s,
            self.descent_time_s,
            self.inbound_level_time_s,
            self.turn_time_s,
        )
        ####


def estimate_racetrack_timing(
    *,
    straight_length_m: float,
    turn_radius_m: float,
    speed_m_s: float,
    altitude_delta_m: float,
    climb_rate_m_s: float,
    descent_rate_m_s: float,
) -> RacetrackTimingEstimate:
    """Estimate a racetrack horizon from geometry and vehicle-level rates.

    The climb and descent are placed on the first portions of the outbound
    and inbound straights.  The function rejects a geometry whose straight is
    too short to contain the requested vertical maneuver; this prevents a
    caller from silently compressing an altitude gate into a turn.
    """

    values = {
        "straight_length_m": straight_length_m,
        "turn_radius_m": turn_radius_m,
        "speed_m_s": speed_m_s,
        "climb_rate_m_s": climb_rate_m_s,
        "descent_rate_m_s": descent_rate_m_s,
    }
    for name, value in values.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if not math.isfinite(altitude_delta_m) or altitude_delta_m < 0.0:
        raise ValueError("altitude_delta_m must be finite and non-negative")

    straight_time = straight_length_m / speed_m_s
    turn_time = math.pi * turn_radius_m / speed_m_s
    climb_time = altitude_delta_m / climb_rate_m_s
    descent_time = altitude_delta_m / descent_rate_m_s
    if climb_time > straight_time or descent_time > straight_time:
        raise ValueError("straight length is too short for the requested vertical phases")
    outbound_level_time = straight_time - climb_time
    inbound_level_time = straight_time - descent_time
    total_time = 2.0 * straight_time + 2.0 * turn_time
    return RacetrackTimingEstimate(
        straight_time_s=straight_time,
        turn_time_s=turn_time,
        climb_time_s=climb_time,
        descent_time_s=descent_time,
        outbound_level_time_s=outbound_level_time,
        inbound_level_time_s=inbound_level_time,
        total_time_s=total_time,
    )
    ####


__all__ = ["RacetrackTimingEstimate", "estimate_racetrack_timing"]
####
