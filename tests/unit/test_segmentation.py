from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.control import (
    ControlDemand,
    SegmentController,
    SegmentPlan,
    SegmentSchedule,
    SegmentTransition,
    VehicleObservation,
)
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.problem_parser import parse_problem_file
from taoryx.runtime.runner import run_files
from taoryx.segmentation import (
    GoalSpec,
    SegmentationCompileError,
    SegmentationScenario,
    SegmentSpec,
    TransitionEventSpec,
    TransitionPolicy,
    compile_catalog,
    compile_scenario,
    load_catalog,
    time_schedule,
    transition_audit,
)

ROOT = Path(__file__).resolve().parents[2]


def test_repository_segmentation_catalog_compiles_all_vehicle_families(tmp_path: Path) -> None:
    outputs = compile_catalog(ROOT / "verification/segmentation_catalog.yaml", ROOT)
    assert len(outputs) == 6
    for problem, manifest, audit in outputs:
        assert problem.is_file()
        assert manifest.is_file()
        assert audit.is_file()
        assert "*segment 1" in problem.read_text(encoding="utf-8")
        assert "historical_syntax" in manifest.read_text(encoding="utf-8")
        assert json.loads(audit.read_text(encoding="utf-8"))["status"] == "compiled"
        manifest_payload = json.loads(manifest.read_text(encoding="utf-8"))
        assert f"*segment {len(manifest_payload['segments'])}" in problem.read_text(encoding="utf-8")
        assert manifest_payload["historical_syntax"] == "unchanged; segmentation metadata is external"
        assert not parse_problem_file(problem, profile=GrammarProfile.TAORYX).diagnostics
    ####


def test_catalog_contains_typed_goals_and_external_events() -> None:
    scenarios = load_catalog(ROOT / "verification/segmentation_catalog.yaml").scenarios
    assert all(segment.goal is not None for scenario in scenarios for segment in scenario.segments)
    x15 = next(scenario for scenario in scenarios if scenario.vehicle == "x15")
    assert x15.events[0].kind == "separation"
    assert x15.events[0].target == "descent"
    assert x15.events[0].transition.velocity == "continuous"
    ####


def test_goal_and_event_contracts_reject_invalid_values() -> None:
    with pytest.raises(ValueError, match="tolerances must be positive"):
        GoalSpec(id="bad-goal", kind="waypoint", tolerance={"north_m": 0.0})
    with pytest.raises(ValueError, match="requires a target"):
        TransitionEventSpec(id="bad-event", kind="signal", condition="time > 1", action="goto")
    ####


def test_catalog_rejects_missing_target(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text(
        """
schema_version: 1
id: bad
scenarios:
  - id: case
    vehicle: demo
    family: point-mass
    source_problem: demo.prb
    output_problem: out.prb
    output_manifest: out.json
    output_audit: audit.json
    segments:
      - id: first
        mode: point_mass_3dof
        entry: {condition: 'time >= 0'}
        exit: {condition: 'time > 1', target: nowhere}
""",
        encoding="utf-8",
    )
    with pytest.raises(SegmentationCompileError, match="missing segment"):
        load_catalog(path)
    ####


def test_catalog_rejects_cycles_and_stop_targets(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text(
        """
schema_version: 1
id: bad
scenarios:
  - id: case
    vehicle: demo
    family: point-mass
    source_problem: demo.prb
    output_problem: out.prb
    output_manifest: out.json
    output_audit: audit.json
    segments:
      - id: first
        mode: point_mass_3dof
        entry: {condition: 'time >= 0'}
        exit: {condition: 'time > 1', target: second}
      - id: second
        mode: point_mass_3dof
        entry: {condition: 'time > 1'}
        exit: {condition: 'time > 2', target: first, action: stop}
""",
        encoding="utf-8",
    )
    with pytest.raises(SegmentationCompileError, match="stopping segment"):
        load_catalog(path)
    ####


class _LifecycleController:
    def __init__(self) -> None:
        self.resets: list[str] = []
        self.transitions: list[SegmentTransition] = []

    def reset(self, segment: SegmentPlan) -> None:
        self.resets.append(segment.identifier)
        ####

    def on_transition(self, transition: SegmentTransition) -> None:
        self.transitions.append(transition)
        ####

    def update(self, observation: VehicleObservation, target: dict[str, float], dt_s: float) -> ControlDemand:
        return ControlDemand({"throttle": target.get("throttle", 0.0)}, source="test")
        ####
    ####


def test_segment_controller_calls_lifecycle_once_per_transition() -> None:
    controller = _LifecycleController()
    schedule = SegmentSchedule(
        (
            SegmentPlan("a", 0.0, 1.0, {"throttle": 0.2}),
            SegmentPlan("b", 1.0, 2.0, {"throttle": 0.4}),
        )
    )
    runner = SegmentController(controller, schedule)
    runner.step(VehicleObservation(0.5, {}), 0.1)
    runner.step(VehicleObservation(0.75, {}), 0.1)
    runner.step(VehicleObservation(1.0, {}), 0.1)
    assert controller.resets == ["a", "b"]
    assert [(item.previous_identifier, item.current_identifier) for item in controller.transitions] == [
        (None, "a"),
        ("a", "b"),
    ]
    assert len(runner.transition_history) == 2
    ####


def test_transition_audit_requires_declared_discontinuities() -> None:
    continuous = transition_audit(
        TransitionPolicy(),
        {"position": 1.0, "velocity": 2.0, "mass": 3.0},
        {"position": 1.0, "velocity": 2.1, "mass": 3.0},
    )
    assert continuous["status"] == "fail"
    assert continuous["violations"] == ["velocity"]
    explicit = transition_audit(
        TransitionPolicy(velocity="impulse", mass="mass_change"),
        {"velocity": 2.0, "mass": 3.0},
        {"velocity": 2.1, "mass": 2.5},
    )
    assert explicit["status"] == "pass"
    ####


def test_time_schedule_adapts_catalog_segments_to_existing_controller_contract() -> None:
    scenario = load_catalog(ROOT / "verification/segmentation_catalog.yaml").scenarios[0]
    schedule = time_schedule(scenario)
    assert [segment.identifier for segment in schedule.segments] == ["trim", "heading-step"]
    assert schedule.segments[-1].end_time_s == 120.0
    ####


def test_declared_increment_lowers_through_native_transition_syntax(tmp_path: Path) -> None:
    source = tmp_path / "source.prb"
    source.write_text(
        """(demo)\n*trajectory 1 demo start on 1\n  *initial geodetic alt=0 long=0 lat=0 vel=1 gama=0 psi=0 time=0 mass=10\n  *segment 1 source\n    *integ dt=0.1 dtprnt=0.1\n    *when time>1 goto 2\n  *segment 2 source\n    *integ dt=0.1 dtprnt=0.1\n    *when time>2 stop\n*end\n""",
        encoding="utf-8",
    )
    scenario = SegmentationScenario(
        id="increment-case",
        vehicle="demo",
        family="point-mass",
        source_problem="source.prb",
        output_problem="out.prb",
        output_manifest="out.json",
        output_audit="audit.json",
        segments=(
            SegmentSpec(
                id="first",
                mode="point_mass_3dof",
                entry={"condition": "time >= 0", "frame": "geodetic", "units": "si"},
                exit={"condition": "time > 1", "target": "second"},
                transition=TransitionPolicy(velocity="impulse"),
                transition_values={"xdt": 2.0},
            ),
            SegmentSpec(
                id="second",
                mode="point_mass_3dof",
                entry={"condition": "time > 1", "frame": "geodetic", "units": "si"},
                exit={"condition": "time > 2", "action": "stop"},
                initial_state={"mass": 9.0},
            ),
        ),
    )
    problem, _, _ = compile_scenario(scenario, tmp_path)
    assert "*increment xdt=2.0" in problem.read_text(encoding="utf-8")
    assert "*reset mass=9.0" in problem.read_text(encoding="utf-8")
    ####


def test_compiled_segments_execute_through_standard_runtime(tmp_path: Path) -> None:
    source = tmp_path / "source.prb"
    source.write_text(
        """(runtime-demo)\n*title segmentation runtime demo\n*mode point-mass\n*atmos standard\n*earth wgs-84 omega=0\n*trajectory 1 demo start on 1\n  *initial geodetic alt=10 long=0 lat=0 vel=10 gama=0 psi=90 time=0 mass=1\n  *segment 1 source\n    *integ dtprnt=0.1 dt=0.05\n    *when time>0.5 goto 2\n  *segment 2 source\n    *integ dtprnt=0.1 dt=0.05\n    *when time>1.0 stop\n*end\n""",
        encoding="utf-8",
    )
    scenario = SegmentationScenario(
        id="runtime-demo",
        vehicle="demo",
        family="point-mass",
        source_problem="source.prb",
        output_problem="compiled.prb",
        output_manifest="compiled.json",
        output_audit="compiled-audit.json",
        segments=(
            SegmentSpec(
                id="first",
                mode="point_mass_3dof",
                entry={"condition": "time >= 0", "frame": "geodetic", "units": "si"},
                exit={"condition": "time > 0.5", "target": "second"},
            ),
            SegmentSpec(
                id="second",
                mode="point_mass_3dof",
                entry={"condition": "time > 0.5", "frame": "geodetic", "units": "si"},
                exit={"condition": "time > 1.0", "action": "stop"},
            ),
        ),
    )
    problem, _, _ = compile_scenario(scenario, tmp_path)
    report = run_files(problem, output_dir=tmp_path / "run", max_steps=100, profile=GrammarProfile.TAORYX)
    assert report.exit_code == 0, report.as_dict()
    assert report.results and report.results[0].completed
    assert report.artifacts
    transitions = [event for event in report.artifacts[0].events if event.get("action") == "goto"]
    assert transitions and transitions[0]["state_discontinuity"] is False
