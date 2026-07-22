from __future__ import annotations

from tools.render_evidence_showcase import ANGLE_SERIES, _objective_markers, _series_from_rows


def test_objective_markers_include_evaluation_bounds_and_declared_time() -> None:
    rows = [{"time_s": 0.0}, {"time_s": 12.0}]
    payload = {"objective_evaluation": {"objectives": [{"id": "heading-step", "time_s": 6.0}]}}

    assert _objective_markers(payload, rows) == [
        (0.0, "objective evaluation start"),
        (6.0, "heading-step"),
        (12.0, "objective evaluation finish"),
    ]
    ####


def test_objective_markers_include_family_specific_route_and_pronav_events() -> None:
    rows = [
        {"time_s": 0.0, "route_leg_index": 0.0, "pro_nav_active": 0.0},
        {"time_s": 4.0, "route_leg_index": 1.0, "pro_nav_active": 0.0, "_segment": 1.0},
        {"time_s": 8.0, "route_leg_index": 1.0, "pro_nav_active": 1.0},
        {"time_s": 12.0, "route_leg_index": 2.0, "pro_nav_active": 1.0, "_segment": 2.0},
    ]

    markers = _objective_markers({"objective_evaluation": {"objectives": []}}, rows)

    assert markers == [
        (0.0, "objective evaluation start"),
        (4.0, "waypoint leg 1"),
        (8.0, "ProNav activation"),
        (12.0, "objective evaluation finish / waypoint leg 2 / segment 2"),
    ]
    ####


def test_angle_panel_prefers_local_physical_bank_over_query_coordinate() -> None:
    rows = [
        {"time_s": 0.0, "local_roll_deg": 0.0, "aero_query_bank-deg": 180.0},
        {"time_s": 1.0, "local_roll_deg": 0.0, "aero_query_bank-deg": 180.0},
    ]

    assert _series_from_rows(rows, ANGLE_SERIES["aero"][1][1]) == (
        "local_roll_deg",
        [0.0, 1.0],
        [0.0, 0.0],
    )
    ####


def test_figure_eight_phase_markers_are_not_called_waypoints() -> None:
    rows = [
        {"time_s": 0.0, "route_phase_index": 0.0},
        {"time_s": 5.0, "route_phase_index": 1.0},
        {"time_s": 10.0, "route_phase_index": 2.0},
    ]

    markers = _objective_markers({"objective_evaluation": {"objectives": []}}, rows)

    assert markers[1:] == [
        (5.0, "figure-eight phase 1"),
        (10.0, "objective evaluation finish / figure-eight phase 2"),
    ]
    ####
