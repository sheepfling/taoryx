from __future__ import annotations

import math
from pathlib import Path

import pytest

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.language import GrammarProfile, parse_problem_file
from taoryx.modes import Quaternion
from taoryx.racetrack_timing import estimate_racetrack_timing
from taoryx.rigid_body import RigidBody6DofState
from taoryx.runtime.lowering import (
    _runtime_coordinated_turn_enabled,
    _runtime_figure_eight_pitch_angle,
    _runtime_figure_eight_waypoint_position,
    _runtime_point_mass_route_commands,
    _runtime_rectangle_bank_angle,
    _runtime_rectangle_waypoint_position,
    _runtime_route_tracking_geometry,
    _runtime_route_velocity,
)

ROUTE = {
    "mode": "figure-eight",
    "start-latitude-deg": "0.0",
    "start-longitude-deg": "0.0",
    "start-altitude-m": "100.0",
    "duration-s": "100.0",
    "figure-eight-length-m": "600.0",
    "figure-eight-width-m": "300.0",
    "figure-eight-speed-mps": "20.0",
    "figure-eight-bank-deg": "12.0",
}

RACETRACK = {
    "mode": "racetrack",
    "start-latitude-deg": "0.0",
    "start-longitude-deg": "0.0",
    "racetrack-length-m": "1400.0",
    "racetrack-turn-radius-m": "250.0",
    "racetrack-speed-mps": "20.0",
    "racetrack-low-altitude-m": "178.0",
    "racetrack-high-altitude-m": "218.0",
    "racetrack-climb-rate-mps": "2.0",
    "racetrack-descent-rate-mps": "2.0",
    "duration-s": "218.5398163397",
    "racetrack-left-bank-deg": "12.0",
    "racetrack-right-bank-deg": "-12.0",
}


@pytest.mark.parametrize(
    "path",
    [
        Path("examples/mission_families/slower_b747/SV01_figure_eight_route_6dof.prb"),
        Path("examples/mission_families/slower_x8/SV03_figure_eight_route_6dof.prb"),
        Path("examples/mission_families/slower_x8/SV03_racetrack_altitude_turns_6dof.prb"),
        Path("examples/mission_families/slower_hummingbird/SV05_figure_eight_route_6dof.prb"),
    ],
)
def test_fixed_wing_figure_eight_fixtures_parse_as_taoryx(path: Path) -> None:
    document = parse_problem_file(path, profile=GrammarProfile.TAORYX)
    assert not [item for item in document.diagnostics if item.severity == "error"]
    assert document.problems[0].ended
    ####


def _state(time_s: float) -> RigidBody6DofState:
    return RigidBody6DofState(
        time_s,
        FrameVector3(Vector3(6_378_237.0, 0.0, 0.0), Frame.ECIC),
        FrameVector3(Vector3(0.0, 20.0, 0.0), Frame.ECIC),
        Quaternion.identity(),
        Vector3(0.0, 0.0, 0.0),
        100.0,
        0.0,
    )
    ####


def test_figure_eight_reference_is_continuous_at_start_and_finish() -> None:
    start = _runtime_figure_eight_waypoint_position(ROUTE, _state(0.0))
    finish = _runtime_figure_eight_waypoint_position(ROUTE, _state(100.0))

    assert start is not None and finish is not None
    assert (finish - start).norm() < 1.0e-9
    ####


def test_figure_eight_bank_reverses_sign_between_lobes() -> None:
    first_lobe = _runtime_rectangle_bank_angle(ROUTE, _state(25.0))
    second_lobe = _runtime_rectangle_bank_angle(ROUTE, _state(75.0))

    assert first_lobe is not None and second_lobe is not None
    assert first_lobe > 0.0
    assert second_lobe < 0.0
    assert math.degrees(abs(first_lobe)) == pytest.approx(12.0)
    ####


def test_figure_eight_pitch_reverses_sign_between_lobes() -> None:
    route = {**ROUTE, "figure-eight-pitch-deg": "4.0"}
    first_lobe = _runtime_figure_eight_pitch_angle(route, math.pi / 2.0)
    second_lobe = _runtime_figure_eight_pitch_angle(route, 3.0 * math.pi / 2.0)

    assert math.degrees(first_lobe) == pytest.approx(4.0)
    assert math.degrees(second_lobe) == pytest.approx(-4.0)
    ####


def test_figure_eight_altitude_reference_returns_to_start() -> None:
    route = {**ROUTE, "figure-eight-altitude-amplitude-m": "8.0"}
    start = _runtime_figure_eight_waypoint_position(route, _state(0.0))
    peak = _runtime_figure_eight_waypoint_position(route, _state(25.0))
    finish = _runtime_figure_eight_waypoint_position(route, _state(100.0))

    assert start is not None and peak is not None and finish is not None
    assert (finish - start).norm() < 1.0e-9
    assert peak.norm() - start.norm() == pytest.approx(8.0, abs=1.0e-2)
    ####


def test_figure_eight_bank_feedback_is_bounded_by_route_authority() -> None:
    feedback_route = {**ROUTE, "route-cross-track-bank-gain-deg-per-m": "0.5", "route-max-bank-deg": "8.0"}
    command = _runtime_rectangle_bank_angle(feedback_route, _state(25.0))

    assert command is not None
    assert abs(math.degrees(command)) <= 8.0 + 1.0e-12
    ####


def test_figure_eight_route_velocity_is_finite_and_uses_declared_speed() -> None:
    velocity = _runtime_route_velocity(ROUTE, {}, _state(25.0), 0.0)

    assert velocity is not None
    assert all(math.isfinite(value) for value in (velocity.x, velocity.y, velocity.z))
    assert velocity.norm() == 20.0
    ####


def test_figure_eight_terminal_route_preserves_declared_course_and_speed() -> None:
    route = {
        **ROUTE,
        "terminal-capture": "true",
        "terminal-start-s": "100.0",
        "terminal-recovery-mode": "gate",
        "terminal-intercept-distance-m": "0.0",
        "terminal-target-altitude-m": "100.0",
        "terminal-speed-mps": "20.0",
        "terminal-heading-deg": "90.0",
        "terminal-command-speed-mps": "25.0",
    }
    state = RigidBody6DofState(
        110.0,
        FrameVector3(Vector3(6_378_237.0, 0.0, 0.0), Frame.ECIC),
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECIC),
        Quaternion.identity(),
        Vector3(0.0, 0.0, 0.0),
        100.0,
        0.0,
    )

    velocity = _runtime_route_velocity(route, {}, state, 0.0)

    assert velocity is not None
    assert velocity.x == pytest.approx(0.0, abs=1.0e-9)
    assert velocity.y == pytest.approx(20.0, abs=1.0e-9)
    assert velocity.z == pytest.approx(0.0, abs=1.0e-9)
    ####


def test_figure_eight_reports_tangent_errors_separately_from_range() -> None:
    state = _state(0.0)
    target = _runtime_figure_eight_waypoint_position(ROUTE, state)
    assert target is not None

    errors = _runtime_route_tracking_geometry(ROUTE, target, state, 0.0)

    assert errors["route_cross_track_error_m"] == pytest.approx(0.0, abs=1.0e-8)
    assert errors["route_along_track_error_m"] == pytest.approx(0.0, abs=1.0e-8)
    assert errors["route_heading_error_deg"] == pytest.approx(-math.degrees(math.atan2(150.0, 300.0)), abs=1.0e-8)
    ####


def test_coordinated_turn_switch_is_shared_by_route_shapes() -> None:
    assert _runtime_coordinated_turn_enabled({"figure-eight-coordinated-turn": "true"}, ROUTE)
    assert _runtime_coordinated_turn_enabled({"rectangle-coordinated-turn": "true"}, {**ROUTE, "mode": "rectangle"})
    assert not _runtime_coordinated_turn_enabled({}, ROUTE)
    ####


def test_racetrack_timing_keeps_vertical_phases_inside_straights() -> None:
    estimate = estimate_racetrack_timing(
        straight_length_m=1400.0,
        turn_radius_m=250.0,
        speed_m_s=20.0,
        altitude_delta_m=40.0,
        climb_rate_m_s=2.0,
        descent_rate_m_s=2.0,
    )

    assert estimate.straight_time_s == pytest.approx(70.0)
    assert estimate.turn_time_s == pytest.approx(math.pi * 12.5)
    assert estimate.total_time_s == pytest.approx(2.0 * 70.0 + 2.0 * math.pi * 12.5)
    assert sum(estimate.phase_durations()) == pytest.approx(estimate.total_time_s)
    ####


def test_racetrack_reference_closes_and_reverses_turn_bank() -> None:
    start = _runtime_rectangle_waypoint_position(RACETRACK, _state(0.0))
    finish = _runtime_rectangle_waypoint_position(RACETRACK, _state(218.5398163397))
    left_bank = _runtime_rectangle_bank_angle(RACETRACK, _state(90.0))
    right_bank = _runtime_rectangle_bank_angle(RACETRACK, _state(205.0))
    velocity = _runtime_route_velocity(RACETRACK, {}, _state(25.0), 0.0)

    assert start is not None and finish is not None
    assert (finish - start).norm() < 1.0e-6
    assert left_bank is not None and right_bank is not None
    assert left_bank > 0.0 and right_bank < 0.0
    assert velocity is not None and all(math.isfinite(value) for value in (velocity.x, velocity.y, velocity.z))
    ####


def test_point_mass_racetrack_commands_share_the_rigid_body_phase_reference() -> None:
    route = {
        "mode": "racetrack",
        "start-latitude-deg": "0.0",
        "start-longitude-deg": "0.0",
        "duration-s": "165.9662752399384",
        "racetrack-length-m": "700.0",
        "racetrack-turn-radius-m": "250.0",
        "racetrack-speed-mps": "17.9",
        "racetrack-low-altitude-m": "178.0",
        "racetrack-high-altitude-m": "188.0",
        "racetrack-climb-rate-mps": "0.5",
        "racetrack-descent-rate-mps": "0.3",
        "racetrack-altitude-capture-gain-per-s": "0.1",
        "racetrack-altitude-capture-max-mps": "2.0",
        "position-capture-gain": "0.01",
        "position-capture-max-correction-mps": "2.0",
    }
    command = _runtime_point_mass_route_commands(
        route,
        {},
        {"lat": 0.0, "long": 0.0, "alt": 178.0 / 0.3048, "vel": 17.9 / 0.3048, "time": 0.0},
    )

    assert command["_command_vel"] == pytest.approx(17.9 / 0.3048)
    assert command["_command_psi"] == pytest.approx(90.0)
    assert command["_command_gamgd"] > 0.0
    ####
