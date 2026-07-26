from taoryx.mission_objectives import (
    ControllerTransition,
    TruthObjectiveSpec,
    evaluate_truth_objectives,
)


def test_truth_evaluator_rejects_controller_advance_before_physical_capture() -> None:
    telemetry = (
        {"time_s": 0.0, "north_m": 0.0, "east_m": 0.0, "altitude_m": 2.0},
        {"time_s": 0.5, "north_m": 0.5, "east_m": 0.0, "altitude_m": 2.0},
        {"time_s": 1.0, "north_m": 1.0, "east_m": 0.0, "altitude_m": 2.0},
    )
    result = evaluate_truth_objectives(
        (
            TruthObjectiveSpec(
                id="wp-1",
                objective_type="fly_over",
                target={"north_m": 1.0, "east_m": 0.0, "altitude_m": 2.0},
                tolerance={"north_m": 0.1, "east_m": 0.1, "altitude_m": 0.1},
            ),
        ),
        telemetry,
        controller_transitions=(ControllerTransition("wp-1", 0.5, "CAPTURED"),),
    )
    record = result["results"][0]
    assert record["status"] == "pass"
    assert record["truth_time_s"] == 1.0
    assert record["controller_transition_valid"] is False
    assert result["mission_pass"] is False


def test_truth_evaluator_requires_dwell_and_reports_margin() -> None:
    telemetry = tuple(
        {"time_s": time_s, "north_m": 0.0, "east_m": 0.0, "altitude_m": 2.0}
        for time_s in (0.0, 0.5, 1.0, 1.5, 2.0)
    )
    result = evaluate_truth_objectives(
        (
            TruthObjectiveSpec(
                id="hover",
                objective_type="dwell",
                target={"north_m": 0.0, "east_m": 0.0, "altitude_m": 2.0},
                tolerance={"north_m": 0.1, "east_m": 0.1, "altitude_m": 0.05},
                dwell_s=1.0,
            ),
        ),
        telemetry,
    )
    record = result["results"][0]
    assert result["mission_pass"] is True
    assert record["status"] == "pass"
    assert record["dwell_actual_s"] == 2.0
    assert record["margin"] == 0.05


def test_truth_evaluator_supports_gate_and_truth_event() -> None:
    telemetry = (
        {"time_s": 0.0, "north_m": -1.0, "east_m": 0.0, "altitude_m": 2.0},
        {"time_s": 1.0, "north_m": 1.0, "east_m": 0.0, "altitude_m": 2.0},
    )
    result = evaluate_truth_objectives(
        (
            TruthObjectiveSpec(
                id="gate",
                objective_type="fly_by_gate",
                target={"north_m": 0.0, "east_m": 0.0, "altitude_m": 2.0},
                tolerance={"east_m": 0.2},
                gate_normal=(1.0, 0.0, 0.0),
            ),
            TruthObjectiveSpec(id="touchdown-event", objective_type="event", event_id="touchdown"),
        ),
        telemetry,
        truth_events={"touchdown"},
    )
    assert result["mission_pass"] is True
    assert [item["status"] for item in result["results"]] == ["pass", "pass"]


def test_hard_gate_failure_blocks_mission_even_when_objectives_pass() -> None:
    objective = TruthObjectiveSpec(
        id="terminal",
        objective_type="fly_over",
        target={"north_m": 0.0},
        tolerance={"north_m": 1.0},
    )
    result = evaluate_truth_objectives(
        (objective,),
        ({"time_s": 0.0, "north_m": 0.0},),
        hard_gates_passed=False,
    )
    assert result["objective_pass"] is True
    assert result["mission_pass"] is False


def test_gate_negative_control_rejects_wrong_crossing_direction() -> None:
    telemetry = (
        {"time_s": 0.0, "north_m": 1.0, "east_m": 0.0, "altitude_m": 2.0},
        {"time_s": 1.0, "north_m": -1.0, "east_m": 0.0, "altitude_m": 2.0},
    )
    result = evaluate_truth_objectives(
        (
            TruthObjectiveSpec(
                id="north-gate",
                objective_type="fly_by_gate",
                target={"north_m": 0.0, "east_m": 0.0, "altitude_m": 2.0},
                tolerance={"corridor_m": 0.2},
                gate_normal=(1.0, 0.0, 0.0),
                crossing_direction=-1,
            ),
        ),
        telemetry,
    )
    assert result["mission_pass"] is True
    assert result["results"][0]["critical_metric"]["units"] == "m"

    wrong_way = evaluate_truth_objectives(
        (
            TruthObjectiveSpec(
                id="north-gate",
                objective_type="fly_by_gate",
                target={"north_m": 0.0, "east_m": 0.0, "altitude_m": 2.0},
                tolerance={"corridor_m": 0.2},
                gate_normal=(1.0, 0.0, 0.0),
                crossing_direction=1,
            ),
        ),
        telemetry,
    )
    assert wrong_way["mission_pass"] is False


def test_gate_reports_altitude_as_a_separate_required_metric() -> None:
    telemetry = (
        {"time_s": 0.0, "north_m": -1.0, "east_m": 0.0, "altitude_m": 10.0},
        {"time_s": 1.0, "north_m": 1.0, "east_m": 0.0, "altitude_m": 14.0},
    )
    result = evaluate_truth_objectives(
        (
            TruthObjectiveSpec(
                id="altitude-gate",
                objective_type="fly_by_gate",
                target={"north_m": 0.0, "east_m": 0.0, "altitude_m": 10.0},
                tolerance={"corridor_m": 0.5, "altitude_m": 1.0},
                gate_normal=(1.0, 0.0, 0.0),
            ),
        ),
        telemetry,
    )
    record = result["results"][0]
    assert record["status"] == "fail"
    assert record["critical_metric"]["channel"] == "gate_altitude_error"
    assert record["critical_metric"]["actual"] == 4.0
    assert record["critical_metric"]["limit"] == 1.0
    assert record["critical_metric"]["units"] == "m"


def test_gate_pass_requires_both_lateral_and_altitude_limits() -> None:
    telemetry = (
        {"time_s": 0.0, "north_m": -1.0, "east_m": 0.0, "altitude_m": 10.0},
        {"time_s": 1.0, "north_m": 1.0, "east_m": 0.0, "altitude_m": 10.9},
    )
    result = evaluate_truth_objectives(
        (
            TruthObjectiveSpec(
                id="altitude-gate",
                objective_type="fly_by_gate",
                target={"north_m": 0.0, "east_m": 0.0, "altitude_m": 10.0},
                tolerance={"corridor_m": 0.5, "altitude_m": 1.0},
                gate_normal=(1.0, 0.0, 0.0),
            ),
        ),
        telemetry,
    )
    record = result["results"][0]
    assert record["status"] == "pass"
    assert record["critical_metric"]["channel"] == "gate_altitude_error"


def test_gate_pass_requires_speed_limit_when_declared() -> None:
    telemetry = (
        {"time_s": 0.0, "north_m": -1.0, "east_m": 0.0, "altitude_m": 10.0, "speed_m_s": 20.0},
        {"time_s": 1.0, "north_m": 1.0, "east_m": 0.0, "altitude_m": 10.0, "speed_m_s": 25.0},
    )
    result = evaluate_truth_objectives(
        (
            TruthObjectiveSpec(
                id="speed-gate",
                objective_type="fly_by_gate",
                target={"north_m": 0.0, "east_m": 0.0, "altitude_m": 10.0, "speed_m_s": 20.0},
                tolerance={"corridor_m": 0.5, "altitude_m": 1.0, "speed_m_s": 1.0},
                gate_normal=(1.0, 0.0, 0.0),
            ),
        ),
        telemetry,
    )
    record = result["results"][0]
    assert record["status"] == "fail"
    assert record["critical_metric"]["channel"] == "gate_speed_error"
    assert record["critical_metric"]["actual"] == 5.0
    assert record["critical_metric"]["limit"] == 1.0


def test_negative_control_missing_dwell_fails_required_objective() -> None:
    result = evaluate_truth_objectives(
        (
            TruthObjectiveSpec(
                id="hover",
                objective_type="dwell",
                target={"north_m": 0.0, "speed_m_s": 0.0},
                tolerance={"north_m": 0.1, "speed_m_s": 0.1},
                dwell_s=2.0,
            ),
        ),
        (
            {"time_s": 0.0, "north_m": 0.0, "speed_m_s": 0.0},
            {"time_s": 1.0, "north_m": 0.0, "speed_m_s": 0.0},
        ),
    )
    assert result["results"][0]["status"] == "fail"
    assert result["mission_pass"] is False


def test_negative_control_terminal_heading_failure_is_visible_with_units() -> None:
    result = evaluate_truth_objectives(
        (
            TruthObjectiveSpec(
                id="terminal",
                objective_type="terminal_state_gate",
                target={"north_m": 0.0, "heading_deg": 90.0},
                tolerance={"north_m": 1.0, "heading_deg": 2.0},
                dwell_s=0.5,
            ),
        ),
        (
            {"time_s": 0.0, "north_m": 0.0, "heading_deg": 0.0},
            {"time_s": 1.0, "north_m": 0.0, "heading_deg": 0.0},
        ),
    )
    record = result["results"][0]
    assert record["status"] == "fail"
    assert record["critical_metric"]["channel"] == "heading_deg"
    assert record["critical_metric"]["units"] == "deg"
