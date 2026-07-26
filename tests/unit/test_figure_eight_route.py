from __future__ import annotations

import math
from pathlib import Path

import pytest

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.language import GrammarProfile, parse_problem_file
from taoryx.modes import Quaternion
from taoryx.rigid_body import RigidBody6DofState
from taoryx.runtime.lowering import (
    _runtime_coordinated_turn_enabled,
    _runtime_figure_eight_pitch_angle,
    _runtime_figure_eight_waypoint_position,
    _runtime_rectangle_bank_angle,
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


@pytest.mark.parametrize(
    "path",
    [
        Path("examples/mission_families/slower_b747/SV01_figure_eight_route_6dof.prb"),
        Path("examples/mission_families/slower_x8/SV03_figure_eight_route_6dof.prb"),
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
