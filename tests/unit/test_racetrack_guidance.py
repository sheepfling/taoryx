"""Tests for the reusable powered-fixed-wing racetrack reference."""

from __future__ import annotations

import math
from pathlib import Path

from taoryx.racetrack_guidance import racetrack_reference_at_time
from taoryx.racetrack_template import load_racetrack_template_catalog

ROOT = Path(__file__).resolve().parents[2]
ROUTE = load_racetrack_template_catalog(ROOT / "verification/racetrack_templates.yaml").get("f16-s119-point-mass")


def test_reference_has_isolated_vertical_phases_and_signed_turns() -> None:
    timing = ROUTE.timing
    climb = racetrack_reference_at_time(ROUTE, timing.climb_time_s * 0.5)
    outbound_level = racetrack_reference_at_time(ROUTE, timing.climb_time_s + timing.outbound_level_time_s * 0.5)
    left = racetrack_reference_at_time(ROUTE, sum(timing.phase_durations()[:2]) + timing.turn_time_s * 0.5)
    descent_time = sum(timing.phase_durations()[:3]) + timing.descent_time_s * 0.5
    descent = racetrack_reference_at_time(ROUTE, descent_time)
    right_time = sum(timing.phase_durations()[:5]) + timing.turn_time_s * 0.5
    right = racetrack_reference_at_time(ROUTE, right_time)

    assert climb.phase == "outbound-climb"
    assert climb.flight_path_angle_rad > 0.0
    assert climb.altitude_m < ROUTE.high_altitude_m
    assert outbound_level.phase == "outbound-level"
    assert outbound_level.flight_path_angle_rad == 0.0
    assert left.phase == "left-turn"
    assert left.bank_rad == math.radians(ROUTE.left_turn_bank_deg)
    assert descent.phase == "inbound-descent"
    assert descent.flight_path_angle_rad < 0.0
    assert descent.heading_rad == math.radians(-90.0)
    assert right.phase == "right-turn"
    assert right.bank_rad == math.radians(ROUTE.right_turn_bank_deg)
    ####


def test_reference_closes_at_start_finish_without_wrapping_into_another_lap() -> None:
    start = racetrack_reference_at_time(ROUTE, 0.0)
    finish = racetrack_reference_at_time(ROUTE, ROUTE.declared_duration_s)
    after_finish = racetrack_reference_at_time(ROUTE, ROUTE.declared_duration_s + 100.0)

    assert (start.north_m, start.east_m) == (0.0, 0.0)
    assert (finish.north_m, finish.east_m) == (0.0, 0.0)
    assert finish.heading_rad == 0.0
    assert after_finish == after_finish.__class__(
        time_s=ROUTE.declared_duration_s + 100.0,
        phase_index=finish.phase_index,
        phase=finish.phase,
        route_leg_index=finish.route_leg_index,
        north_m=0.0,
        east_m=0.0,
        altitude_m=ROUTE.low_altitude_m,
        speed_m_s=ROUTE.speed_m_s,
        heading_rad=0.0,
        flight_path_angle_rad=0.0,
        bank_rad=0.0,
    )
    ####
