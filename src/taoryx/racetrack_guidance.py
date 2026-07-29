"""Pure guidance references for the shared powered-fixed-wing racetrack.

This module owns only semantic route geometry.  It does not trim a vehicle,
apply forces or moments, allocate effectors, or claim that a vehicle can track
the returned reference.  Those responsibilities remain with the fidelity-
specific controller and plant adapter.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .racetrack_template import ResolvedRacetrack


@dataclass(frozen=True, slots=True)
class RacetrackGuidanceReference:
    """One kinematic reference sample on a resolved racetrack."""

    time_s: float
    phase_index: int
    phase: str
    route_leg_index: int
    north_m: float
    east_m: float
    altitude_m: float
    speed_m_s: float
    heading_rad: float
    flight_path_angle_rad: float
    bank_rad: float


def _heading_from_velocity(north_rate_m_s: float, east_rate_m_s: float) -> float:
    """Return navigation heading measured clockwise from north."""

    return math.atan2(east_rate_m_s, north_rate_m_s)
    ####


def _wrap_heading(angle_rad: float) -> float:
    """Wrap a heading to the interval [-pi, pi)."""

    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    ####


def racetrack_reference_at_time(
    route: ResolvedRacetrack,
    time_s: float,
    *,
    trim_pitch_rad: float = 0.0,
) -> RacetrackGuidanceReference:
    """Resolve the shared course into a bounded kinematic reference.

    The local frame is NED for position, with positive ``altitude_m`` upward
    for the mission contract.  The outbound leg starts eastbound at the low
    altitude.  The left semicircle ends westbound at the high altitude; the
    inbound straight descends; and the right semicircle returns eastbound to
    the start/finish crossing.  Values after the declared route duration are
    clamped to the terminal crossing.

    This is deliberately a guidance reference rather than an achieved-state
    calculator.  A vehicle may fail to track it; the independent objective
    evaluator must adjudicate the resulting truth telemetry.
    """

    if not math.isfinite(time_s) or time_s < 0.0:
        raise ValueError("racetrack reference time must be finite and non-negative")
    if not math.isfinite(trim_pitch_rad):
        raise ValueError("trim_pitch_rad must be finite")

    timing = route.timing
    durations = timing.phase_durations()
    phase_ends = route._cumulative_phase_ends()
    t = min(time_s, route.declared_duration_s)
    speed = route.speed_m_s
    length = route.straight_length_m
    radius = route.turn_radius_m
    altitude = route.low_altitude_m
    phase_index = 0
    local_time = t
    for index, end_time in enumerate(phase_ends):
        if t <= end_time or index == len(phase_ends) - 1:
            phase_index = index
            phase_start = 0.0 if index == 0 else phase_ends[index - 1]
            local_time = t - phase_start
            break

    phase = route.phase_windows[phase_index].name
    climb_rate = 0.0
    north = 0.0
    east = 0.0
    north_rate = 0.0
    east_rate = speed
    bank = 0.0

    if phase_index == 0:
        east = speed * local_time
        altitude = route.low_altitude_m + route.climb_rate_m_s * local_time
        climb_rate = route.climb_rate_m_s
    elif phase_index == 1:
        east = speed * timing.climb_time_s + speed * local_time
        altitude = route.high_altitude_m
    elif phase_index == 2:
        theta = min(math.pi, speed * local_time / radius)
        north = radius * (1.0 - math.cos(theta))
        east = length + radius * math.sin(theta)
        north_rate = speed * math.sin(theta)
        east_rate = speed * math.cos(theta)
        altitude = route.high_altitude_m
        bank = math.radians(route.left_turn_bank_deg)
    elif phase_index == 3:
        east = length - speed * local_time
        north = 2.0 * radius
        north_rate = 0.0
        east_rate = -speed
        altitude = route.high_altitude_m - route.descent_rate_m_s * local_time
        climb_rate = -route.descent_rate_m_s
    elif phase_index == 4:
        east = length - speed * timing.descent_time_s - speed * local_time
        north = 2.0 * radius
        altitude = route.low_altitude_m
        north_rate = 0.0
        east_rate = -speed
    else:
        theta = min(math.pi, speed * local_time / radius)
        north = radius * (1.0 + math.cos(theta))
        east = -radius * math.sin(theta)
        north_rate = -speed * math.sin(theta)
        east_rate = -speed * math.cos(theta)
        altitude = route.low_altitude_m
        bank = math.radians(route.right_turn_bank_deg)

    heading = _wrap_heading(_heading_from_velocity(north_rate, east_rate))
    horizontal_speed = math.hypot(north_rate, east_rate)
    flight_path_angle = math.atan2(climb_rate, max(horizontal_speed, 1.0e-12))
    if time_s >= route.declared_duration_s:
        north = 0.0
        east = 0.0
        altitude = route.low_altitude_m
        heading = 0.0
        flight_path_angle = 0.0
        bank = 0.0
        phase_index = len(durations) - 1
        phase = route.phase_windows[-1].name
        local_time = durations[-1]

    return RacetrackGuidanceReference(
        time_s=float(time_s),
        phase_index=phase_index,
        phase=phase,
        route_leg_index=phase_index + 1,
        north_m=float(north),
        east_m=float(east),
        altitude_m=float(altitude),
        speed_m_s=float(speed),
        heading_rad=float(heading),
        flight_path_angle_rad=float(flight_path_angle),
        bank_rad=float(bank),
    )
    ####


__all__ = ["RacetrackGuidanceReference", "racetrack_reference_at_time"]
####
