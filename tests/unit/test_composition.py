from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.composition import (
    RuntimeEvaluationOptions,
    SegmentCompositionRegistry,
    TrajectoryBuilder,
    WaypointSpec,
    evaluate_composition,
    evaluate_composition_runtime,
    evaluate_segment,
)
from taoryx.outputs import DynamicsKind, EventRecord, RunArtifact, SegmentSpan, TelemetryChannel, VehicleKind, VehicleTelemetry
from taoryx.segmentation import GoalSpec, SegmentEntry, SegmentExit, SegmentSpec


def test_standard_registry_is_discoverable() -> None:
    registry = SegmentCompositionRegistry.standard()

    assert registry.names() == (
        "alpha_profile",
        "altitude_capture",
        "ballistic_coast",
        "bank_maneuver",
        "heading_capture",
        "hover",
        "moving_target_intercept",
        "powered_ascent",
        "skip_maneuver",
        "terminal_pronav",
        "trim_hold",
        "waypoint",
    )
    assert registry.describe("waypoint").requires_target
    assert registry.describe("waypoint").requires_tolerance
    assert registry.describe("moving_target_intercept").requires_reference
    assert registry.describe("terminal_pronav").goal_kind == "terminal_guidance"


def test_waypoint_spec_rejects_invalid_dwell_and_tolerance() -> None:
    with pytest.raises(ValueError, match="tolerances must be finite and positive"):
        WaypointSpec(
            id="north",
            target={"north_m": 100.0},
            tolerance={"north_m": 0.0},
            duration_s=10.0,
        )
    with pytest.raises(ValueError, match="dwell time cannot exceed"):
        WaypointSpec(
            id="north",
            target={"north_m": 100.0},
            tolerance={"north_m": 5.0},
            duration_s=10.0,
            dwell_time_s=11.0,
        )


def test_builder_composes_waypoint_course_and_evaluates_before_compile() -> None:
    builder = TrajectoryBuilder(
        "demo-course",
        vehicle="demo",
        family="fixed-wing",
        source_problem="source.prb",
        mode="point_mass_3dof",
    )
    builder.use(
        "trim_hold",
        "trim",
        duration_s=10.0,
        target={"speed_m_s": 20.0},
        tolerance={"speed_m_s": 1.0},
    )
    builder.waypoint(
        WaypointSpec(
            id="north",
            target={"north_m": 100.0, "altitude_m": 50.0},
            tolerance={"north_m": 10.0, "altitude_m": 3.0},
            duration_s=20.0,
            dwell_time_s=2.0,
        )
    )
    builder.waypoint_course(
        (
            WaypointSpec(
                id="east",
                target={"east_m": 100.0},
                tolerance={"east_m": 10.0},
                duration_s=15.0,
            ),
        )
    )

    scenario = builder.build()
    report = evaluate_composition(scenario)

    assert builder.duration_s == 45.0
    assert report.ok
    assert report.status == "warning"
    assert [segment.id for segment in scenario.segments] == ["trim", "north", "east"]
    assert scenario.segments[0].exit.target == "north"
    assert scenario.segments[1].exit.target == "east"
    assert scenario.segments[-1].exit.action == "stop"
    assert scenario.segments[-1].exit.target is None
    assert scenario.segments[1].goal is not None
    assert scenario.segments[1].goal.kind == "waypoint"
    assert scenario.segments[1].entry.condition == "time >= 10"
    assert scenario.segments[1].exit.condition == "time > 30"


def test_builder_exposes_moving_target_intercept_contract() -> None:
    builder = TrajectoryBuilder(
        "moving-target-course",
        vehicle="x15",
        family="unpowered-hypersonic-glider",
        source_problem="source.prb",
    )

    builder.moving_target_intercept(
        "terminal-intercept",
        duration_s=10.0,
        reference="target-2",
        target={"pro_nav_los_range_m": 25.0, "pro_nav_closing_velocity_m_s": 0.0},
        tolerance={"pro_nav_los_range_m": 25.0, "pro_nav_closing_velocity_m_s": 5.0},
    )

    segment = builder.build().segments[0]
    assert segment.goal is not None
    assert segment.goal.kind == "intercept_geometry"
    assert segment.goal.reference == "target-2"

    with pytest.raises(ValueError, match="moving-target reference"):
        builder.use(
            "moving_target_intercept",
            "missing-reference",
            duration_s=1.0,
            target={"pro_nav_los_range_m": 25.0},
            tolerance={"pro_nav_los_range_m": 5.0},
        )


def test_composed_scenario_compiles_through_existing_segment_lowering(tmp_path: Path) -> None:
    source = tmp_path / "source.prb"
    source.write_text(
        """(composition-demo)
*trajectory 1 demo start on 1
  *initial geodetic alt=10 long=0 lat=0 vel=20 gama=0 psi=90 time=0 mass=1
  *segment 1 source
    *integ dtprnt=0.1 dt=0.05
    *when time>100 stop
*end
""",
        encoding="utf-8",
    )
    builder = TrajectoryBuilder(
        "compiled-course",
        vehicle="demo",
        family="point-mass",
        source_problem="source.prb",
        output_problem="compiled.prb",
        output_manifest="compiled.json",
        output_audit="compiled-audit.json",
        mode="point_mass_3dof",
    )
    builder.waypoint(
        WaypointSpec(
            id="target",
            target={"north_m": 100.0},
            tolerance={"north_m": 5.0},
            duration_s=2.0,
        )
    )
    problem, manifest, audit = builder.compile(tmp_path)

    assert problem.is_file()
    assert manifest.is_file()
    assert audit.is_file()
    rendered = problem.read_text(encoding="utf-8")
    assert "*segment 1 target" in rendered
    assert "*when time > 2 stop" in rendered


def test_segment_evaluation_flags_missing_capture_contract() -> None:
    segment = SegmentSpec(
        id="incomplete",
        mode="point_mass_3dof",
        entry=SegmentEntry(condition="time >= 0"),
        exit=SegmentExit(condition="time > 1"),
        goal=GoalSpec(id="incomplete-goal", kind="waypoint"),
    )

    evaluation = evaluate_segment(segment)

    assert evaluation.status == "fail"
    assert "capture segment requires a non-empty target" in evaluation.messages
    assert "capture segment requires a non-empty tolerance" in evaluation.messages


def _runtime_artifact(*, altitude: list[float], include_transition: bool = True) -> RunArtifact:
    channel = TelemetryChannel(
        source_name="alt",
        semantic_name="position.altitude.geodetic",
        values=altitude,
    )
    spans = [SegmentSpan(number=1, title="segment 1", start_time=0.0, end_time=float(len(altitude) - 1))]
    if len(altitude) > 2:
        spans = [
            SegmentSpan(number=1, title="segment 1", start_time=0.0, end_time=1.0),
            SegmentSpan(number=2, title="segment 2", start_time=1.0, end_time=float(len(altitude) - 1)),
        ]
    vehicle = VehicleTelemetry(
        vehicle_id="demo",
        name="demo",
        kind=VehicleKind.GENERIC,
        dynamics=DynamicsKind.POINT_MASS_3DOF,
        times=[float(index) for index in range(len(altitude))],
        channels={"position.altitude.geodetic": channel},
        segments=spans,
        events=[
            EventRecord(time=1.0, vehicle="demo", name="segment-transition", kind="segment_transition", segment_from=1, segment_to=2),
        ] if include_transition else [],
    )
    return RunArtifact(problem="demo.prb", vehicles={"demo": vehicle})


def test_runtime_composition_scores_goal_dwell_and_handoff() -> None:
    builder = TrajectoryBuilder(
        "runtime-course",
        vehicle="demo",
        family="point-mass",
        source_problem="source.prb",
        mode="point_mass_3dof",
    )
    builder.use("trim_hold", "trim", duration_s=1.0, target={"position.altitude.geodetic": 0.0}, tolerance={"position.altitude.geodetic": 0.5})
    builder.use("waypoint", "target", duration_s=2.0, target={"position.altitude.geodetic": 10.0}, tolerance={"position.altitude.geodetic": 0.5}, dwell_time_s=1.0)

    report = evaluate_composition_runtime(builder.build(), _runtime_artifact(altitude=[0.0, 0.0, 10.0, 10.0]))

    assert report.ok
    assert report.status == "warning"  # terminal stop reason is not part of RunArtifact yet
    assert [segment.status for segment in report.segments] == ["pass", "warning"]
    assert any(check.name == "goal:target-goal:dwell" and check.status == "pass" for check in report.segments[1].goal_checks)
    assert any(check.name == "transition:event" and check.status == "pass" for check in report.segments[1].transition_checks)


def test_runtime_composition_blocks_missing_goal_channel_instead_of_guessing() -> None:
    builder = TrajectoryBuilder(
        "blocked-course",
        vehicle="demo",
        family="point-mass",
        source_problem="source.prb",
        mode="point_mass_3dof",
    )
    builder.use("waypoint", "target", duration_s=1.0, target={"altitude_m": 10.0}, tolerance={"altitude_m": 1.0})

    report = evaluate_composition_runtime(
        builder.build(),
        _runtime_artifact(altitude=[0.0, 0.0]),
        options=RuntimeEvaluationOptions(require_explicit_segment_spans=True),
    )

    assert report.status == "blocked"
    assert not report.ok
    assert any("altitude_m" in (check.message or "") for check in report.segments[0].goal_checks)


def test_runtime_transition_tolerance_is_a_real_quality_gate() -> None:
    builder = TrajectoryBuilder(
        "handoff-course",
        vehicle="demo",
        family="point-mass",
        source_problem="source.prb",
        mode="point_mass_3dof",
    )
    builder.use("trim_hold", "trim", duration_s=1.0, target={"position.altitude.geodetic": 0.0}, tolerance={"position.altitude.geodetic": 0.5})
    builder.use("waypoint", "target", duration_s=2.0, target={"position.altitude.geodetic": 10.0}, tolerance={"position.altitude.geodetic": 0.5})

    report = evaluate_composition_runtime(
        builder.build(),
        _runtime_artifact(altitude=[0.0, 0.0, 10.0, 10.0]),
        options=RuntimeEvaluationOptions(transition_tolerances={"position.altitude.geodetic": 0.1}),
    )

    assert report.status == "fail"
    assert any(check.name == "transition:position.altitude.geodetic" and check.status == "fail" for check in report.segments[1].transition_checks)


def test_runtime_required_channels_are_explicit_and_never_guessed() -> None:
    builder = TrajectoryBuilder(
        "guidance-course",
        vehicle="demo",
        family="point-mass",
        source_problem="source.prb",
        mode="point_mass_3dof",
    )
    builder.use(
        "waypoint",
        "intercept",
        duration_s=1.0,
        target={"position.altitude.geodetic": 0.0},
        tolerance={"position.altitude.geodetic": 0.5},
    )

    report = evaluate_composition_runtime(
        builder.build(),
        _runtime_artifact(altitude=[0.0, 0.0]),
        options=RuntimeEvaluationOptions(required_channels=("pro_nav_los_range_m",)),
    )

    assert report.status == "blocked"
    assert any(
        check.name == "quality:required-channel:pro_nav_los_range_m" and check.status == "blocked"
        for check in report.segments[0].quality_checks
    )


def test_runtime_terminal_gate_rejects_incomplete_execution() -> None:
    builder = TrajectoryBuilder(
        "incomplete-course",
        vehicle="demo",
        family="point-mass",
        source_problem="source.prb",
        mode="point_mass_3dof",
    )
    builder.use("waypoint", "target", duration_s=1.0, target={"position.altitude.geodetic": 0.0}, tolerance={"position.altitude.geodetic": 0.5})

    artifact = _runtime_artifact(altitude=[0.0, 0.0]).model_copy(update={"termination": {"completed": False, "stop_reason": "max_steps"}})
    report = evaluate_composition_runtime(builder.build(), artifact)

    assert report.status == "fail"
    assert any(check.name == "exit:termination-reason" and check.status == "fail" for check in report.segments[0].exit_checks)
