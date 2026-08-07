from __future__ import annotations

import pytest

from taoryx.contracts import Frame, FrameVector3, Quantity, Unit, Vector3
from taoryx.language.expressions import parse_expression
from taoryx.outputs import build_run_artifact
from taoryx.runtime.common import (
    DeploymentValidationError,
    EventCondition,
    RuntimeProblem,
    RuntimeState,
    RuntimeVehicle,
    SpawnRequest,
)
from taoryx.runtime.engine import compute_trajectories, get_next_step_boundary, get_next_time_step
from taoryx.runtime.environment_runtime import evaluate_wind
from taoryx.runtime.events import apply_state_discontinuity, refine_segment_final_condition
from taoryx.runtime.expressions import evaluate_definition_program, evaluate_expression
from taoryx.runtime.optimization_runtime import (
    OptimizerBackend,
    available_optimizers,
    resolve_optimize_block,
    select_optimizer,
)
from taoryx.runtime.runtime_model import build_runtime_problem
from taoryx.runtime.surveys import generate_survey_cases
from taoryx.runtime.units import from_internal, resolve_units_and_formats, selected_setting, to_internal, unit_scale
from taoryx.state import PointMassRates, PointMassState


def test_spawn_event_commits_child_at_accepted_boundary_and_steps_active_collection() -> None:
    def spawn_child(state: RuntimeState) -> tuple[SpawnRequest, ...]:
        child = RuntimeVehicle(
            "spent-stage",
            RuntimeState(state.time, state.values, state.frame),
            lambda _: (2.0,),
            step_size=0.1,
            events=(EventCondition("child-terminal", lambda current: current.values[0] - 1.1),),
        )
        return (SpawnRequest("parent-stage-separation", child, metadata={"shape": "cylinder"}),)

    parent = RuntimeVehicle(
        "parent",
        RuntimeState(0.0, (0.0,), value_names=("x",)),
        lambda _: (1.0,),
        step_size=1.0,
        events=(EventCondition("separate", lambda state: state.values[0] - 0.5, action="spawn"),),
        spawn_provider=spawn_child,
    )
    problem = RuntimeProblem({parent.name: parent}, final_time=0.8)

    result = compute_trajectories(problem)

    assert result.completed
    assert [state.time for state in problem.vehicles["spent-stage"].history] == pytest.approx([0.5, 0.6, 0.7, 0.8])
    event = problem.event_history[0]
    assert event["event_id"] == "parent-stage-separation"
    assert event["model_id"] == "parent"
    assert event["child_model_ids"] == ["spent-stage"]
    assert event["child_initial_states"][0]["time"] == pytest.approx(0.5)
    assert event["status"] == "committed"
    assert problem.vehicles["spent-stage"].parent_model_id == "parent"
    assert problem.vehicles["spent-stage"].active is False
    assert problem.event_history[-1]["name"] == "child-terminal"


def test_spawn_event_accepts_an_arbitrary_propulsive_child() -> None:
    def spawn_payload(state: RuntimeState) -> tuple[SpawnRequest, ...]:
        payload = RuntimeVehicle(
            "payload-vehicle",
            RuntimeState(state.time, (0.0,), state.frame),
            lambda _: (3.0,),
            step_size=0.1,
        )
        return (
            SpawnRequest(
                "payload-deployment",
                payload,
                metadata={"vehicle_class": "propulsive_vehicle", "aero_ballistic": False},
            ),
        )

    parent = RuntimeVehicle(
        "carrier",
        RuntimeState(0.0, (0.0,)),
        lambda _: (1.0,),
        step_size=1.0,
        events=(EventCondition("deploy", lambda state: state.values[0] - 0.5, action="spawn"),),
        spawn_provider=spawn_payload,
    )
    problem = RuntimeProblem({parent.name: parent}, final_time=0.8)

    result = compute_trajectories(problem)

    assert result.completed
    assert problem.vehicles["payload-vehicle"].state.values[0] == pytest.approx(0.9)
    assert problem.event_history[0]["spawn_metadata"][0]["vehicle_class"] == "propulsive_vehicle"
    assert problem.event_history[0]["spawn_metadata"][0]["aero_ballistic"] is False


def test_spawn_batch_validation_is_atomic_and_records_failure() -> None:
    def invalid_spawn(state: RuntimeState) -> tuple[SpawnRequest, ...]:
        first = RuntimeVehicle("child", RuntimeState(state.time, state.values), lambda _: (0.0,))
        duplicate = RuntimeVehicle("child", RuntimeState(state.time, state.values), lambda _: (0.0,))
        return (
            SpawnRequest("invalid-separation", first),
            SpawnRequest("invalid-separation", duplicate),
        )

    parent = RuntimeVehicle(
        "parent",
        RuntimeState(0.0, (0.0,)),
        lambda _: (1.0,),
        step_size=1.0,
        events=(EventCondition("separate", lambda state: state.values[0] - 0.5, action="spawn"),),
        spawn_provider=invalid_spawn,
    )
    problem = RuntimeProblem({parent.name: parent}, final_time=1.0)

    with pytest.raises(DeploymentValidationError, match="already exists"):
        compute_trajectories(problem)

    assert tuple(problem.vehicles) == ("parent",)
    assert problem.event_history[-1]["status"] == "deployment_failed"
    assert problem.event_history[-1]["event_id"] == "separate"


def test_runtime_graph_rejects_cycles_and_steps_at_boundaries() -> None:
    first = RuntimeVehicle("a", RuntimeState(0.0, (0.0,)), lambda state: (1.0,), step_size=2.0)
    problem = build_runtime_problem((first,), print_times=(0.5,), final_time=1.0)
    assert get_next_time_step(problem, 2.0) == 0.5
    assert get_next_step_boundary(problem, 2.0).reason == "print_cadence"
    assert get_next_step_boundary(problem, 2.0).time == pytest.approx(0.5)
    truth_timed = build_runtime_problem((first,), required_truth_times=(0.25, 0.75), final_time=1.0)
    assert get_next_time_step(truth_timed, 2.0) == 0.25
    result = compute_trajectories(problem)
    assert result.completed
    assert problem.vehicles["a"].state.time == pytest.approx(1.0)
    with pytest.raises(ValueError, match="cycle"):
        build_runtime_problem((RuntimeVehicle("a", RuntimeState(0.0, ()), dependencies=("b",)), RuntimeVehicle("b", RuntimeState(0.0, ()), dependencies=("a",))))
####


def test_expression_program_and_events_are_resolved() -> None:
    values = evaluate_definition_program({"a": parse_expression("2"), "b": parse_expression("a * 3")})
    assert values["b"] == 6.0
    state = RuntimeState(0.0, (0.0,), named={"x": 0.0}, value_names=("x",))
    end = RuntimeState(2.0, (2.0,))
    crossing = refine_segment_final_condition(state, end, (EventCondition("zero", lambda item: item.values[0] - 1.0),))
    assert crossing[0].time == pytest.approx(1.0)
    updated = apply_state_discontinuity(state, (2.0,))
    assert updated.values == (2.0,)
    assert updated.named["x"] == 2.0
    assert updated.value_names == ("x",)
    with pytest.raises(ValueError, match="circular"):
        evaluate_definition_program({"a": parse_expression("b"), "b": parse_expression("a")})
####


def test_expression_min_and_max_accept_variadic_arguments() -> None:
    assert evaluate_expression(parse_expression("max(1, 4, 2)"), {}) == pytest.approx(4.0)
    assert evaluate_expression(parse_expression("min(1, 4, 2)"), {}) == pytest.approx(1.0)
    ####


def test_engine_integrates_typed_point_mass_derivative() -> None:
    state = PointMassState(
        0.0,
        FrameVector3(Vector3(0.0, 0.0, 0.0), Frame.ECFC),
        FrameVector3(Vector3(1.0, 0.0, 0.0), Frame.ECFC),
        2.0,
    )

    def derivative(item: PointMassState) -> PointMassRates:
        return PointMassRates(
            FrameVector3(item.earth_relative_velocity.vector, Frame.ECFC),
            FrameVector3(Vector3(2.0, 0.0, 0.0), Frame.ECFC),
            0.0,
            item.earth_relative_velocity.vector.norm(),
            item.earth_relative_velocity.vector.norm(),
        )
    ####

    vehicle = RuntimeVehicle(
        "typed",
        RuntimeState.from_point_mass_state(state),
        step_size=0.1,
        integrator="rk4",
        point_mass_derivative=derivative,
    )
    result = compute_trajectories(RuntimeProblem({"typed": vehicle}, final_time=0.1))

    assert result.completed
    final = result.states["typed"][-1].to_point_mass_state()
    assert final.position.vector.x == pytest.approx(0.11)
    assert final.earth_relative_velocity.vector.x == pytest.approx(1.2)
####


def test_engine_can_integrate_uncoupled_vehicles_without_shared_step_throttling() -> None:
    fast = RuntimeVehicle("fast", RuntimeState(0.0, (0.0,)), lambda state: (1.0,), step_size=1.0)
    slow = RuntimeVehicle("slow", RuntimeState(0.0, (0.0,)), lambda state: (1.0,), step_size=0.1)

    result = compute_trajectories(RuntimeProblem({"fast": fast, "slow": slow}, final_time=1.0), synchronize_vehicles=False)

    assert result.completed
    assert len(result.states["fast"]) == 2
    assert len(result.states["slow"]) == 11
####


def test_engine_reports_a_declared_state_stall_before_consuming_step_budget() -> None:
    vehicle = RuntimeVehicle(
        "stalled",
        RuntimeState(0.0, (0.0,)),
        lambda state: (0.0,),
        stall_detector=lambda state: True,
    )

    result = compute_trajectories(RuntimeProblem({"stalled": vehicle}), max_steps=10000)

    assert not result.completed
    assert result.stop_reason == "state_stall"
    assert len(result.states["stalled"]) == 1
####


def test_units_and_formats_resolve_canonical_and_historical_aliases() -> None:
    settings = {"x": "km", "xecfcdt": "m/sec"}
    formats = {"x": "f.2", "xecfcdt": "f.3"}

    assert selected_setting("xecfc", formats) == "f.2"
    assert to_internal(1.0, "xecfc", settings) == pytest.approx(3280.839895013123)
    assert from_internal(3280.839895013123, "x", settings) == pytest.approx(1.0)
    assert selected_setting("xdt", formats) == "f.3"
    ####


@pytest.mark.parametrize(
    ("unit", "dimension"),
    [
        ("m/min", "speed"),
        ("rpm", "angular_rate"),
        ("g", "acceleration"),
        ("kg/sec", "mass_rate"),
        ("psi", "pressure"),
        ("1/ft", "inverse_length"),
        ("m2/sec", "kinematic_viscosity"),
        ("kg/m3", "density"),
    ],
)
def test_runtime_units_cover_the_documented_dimension_families(unit: str, dimension: str) -> None:
    assert unit_scale(unit, dimension) > 0.0


def test_runtime_units_convert_acceleration_and_pressure_with_dimension_checks() -> None:
    assert to_internal(1.0, "nx", {"nx": "g"}) == pytest.approx(32.17404855643044)
    assert to_internal(1.0, "dynprs", {"dynprs": "psi"}) == pytest.approx(144.0)
    assert from_internal(1.0, "rho", {"rho": "kg/m3"}) == pytest.approx(16.01846337396)
    with pytest.raises(ValueError, match="incompatible"):
        unit_scale("sec", "speed")
    assert to_internal(144.0, "sref", {"sref": "in"}) == pytest.approx(1.0)
    ####


def test_simultaneous_event_crossings_preserve_source_order() -> None:
    state = RuntimeState(0.0, (0.0,))
    end = RuntimeState(1.0, (1.0,))
    crossing = refine_segment_final_condition(
        state,
        end,
        (
            EventCondition("listed-first", lambda item: item.values[0] - 0.5),
            EventCondition("listed-second", lambda item: item.values[0] - 0.5),
        ),
    )

    assert [item.name for item in crossing] == ["listed-first", "listed-second"]
####


def test_event_restart_rewinds_synchronized_vehicles() -> None:
    stopping = RuntimeVehicle(
        "stopping",
        RuntimeState(0.0, (0.0,), value_names=("x",)),
        lambda state: (1.0,),
        step_size=1.0,
        events=(EventCondition("stop", lambda state: state.values[0] - 0.5),),
    )
    continuing = RuntimeVehicle(
        "continuing",
        RuntimeState(0.0, (0.0,), value_names=("x",)),
        lambda state: (2.0,),
        step_size=1.0,
    )

    result = compute_trajectories(build_runtime_problem((stopping, continuing), final_time=1.0))

    assert result.completed
    assert [state.time for state in result.states["continuing"]] == pytest.approx([0.0, 0.5, 1.0])
    assert result.states["continuing"][1].values == pytest.approx((1.0,))


def test_batch_signal_and_stop_events_are_recorded_once_with_provenance() -> None:
    vehicle = RuntimeVehicle(
        "signal-vehicle",
        RuntimeState(0.0, (0.0,), value_names=("x",)),
        lambda state: (1.0,),
        step_size=0.2,
        events=(
            EventCondition("midpoint", lambda state: state.values[0] - 0.15, "signal", signal="midpoint-reached", source="test.prb:4"),
            EventCondition("stop", lambda state: state.values[0] - 0.25, "stop", signal="terminal", source="test.prb:5"),
        ),
    )

    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=1.0)
    result = compute_trajectories(problem)

    assert result.completed
    assert [event["name"] for event in problem.event_history] == ["midpoint", "stop"]
    assert problem.event_history[0]["action"] == "signal"
    assert problem.event_history[0]["signal"] == "midpoint-reached"
    assert problem.event_history[0]["source"] == "test.prb:4"
    assert problem.event_history[1]["time"] == pytest.approx(0.25)
    assert len(problem.transition_history) == 2
    midpoint = problem.transition_history[0]
    assert midpoint.pre.state.time == pytest.approx(0.15)
    assert midpoint.post.state.time == pytest.approx(0.15)
    assert midpoint.pre.segment_number == midpoint.post.segment_number == 1
    assert midpoint.pre.active is True
    assert midpoint.post.active is True
    assert midpoint.state_discontinuity is False
    terminal = problem.transition_history[1]
    assert terminal.pre.state.values == pytest.approx((0.25,))
    assert terminal.post.state.values == pytest.approx((0.25,))
    assert terminal.post.active is False
    assert terminal.state_discontinuity is False
    assert problem.event_history[0]["pre_truth"]["time"] == pytest.approx(0.15)
    assert problem.event_history[1]["post_truth"]["active"] is False
    artifact = build_run_artifact("test.prb", problem, result, events=problem.event_history)
    assert artifact.events[0]["signal"] == "midpoint-reached"
    ####


def test_transition_truth_pair_captures_declared_state_reset_and_controls() -> None:
    vehicle = RuntimeVehicle(
        "reset-vehicle",
        RuntimeState(0.0, (0.0,), value_names=("x",)),
        lambda state: (1.0,),
        step_size=0.5,
        control_values={"throttle": 0.4},
        events=(EventCondition("release", lambda state: state.values[0] - 0.5, "signal"),),
        event_handlers={"release": lambda state: apply_state_discontinuity(state, (2.0,))},
    )
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=1.0)

    assert compute_trajectories(problem).completed
    pair = problem.transition_history[0]
    assert pair.pre.state.values == pytest.approx((0.5,))
    assert pair.post.state.values == pytest.approx((2.5,))
    assert pair.pre.achieved_controls == (("throttle", 0.4),)
    assert pair.post.achieved_controls == (("throttle", 0.4),)
    assert pair.state_discontinuity is True
    assert pair.to_metadata()["pre_truth"]["values"] == pytest.approx([0.5])
    ####


def test_stopped_dependent_vehicle_is_not_reactivated() -> None:
    parent = RuntimeVehicle(
        "parent",
        RuntimeState(0.0, (0.0,)),
        lambda state: (0.0,),
        step_size=0.1,
    )
    child = RuntimeVehicle(
        "child",
        RuntimeState(0.0, (0.0,)),
        lambda state: (0.0,),
        step_size=0.1,
        dependencies=("parent",),
        active=False,
        events=(EventCondition("stop", lambda state: 0.0),),
    )

    result = compute_trajectories(build_runtime_problem((parent, child), final_time=0.2))

    assert result.completed
    assert not child.active
    assert len(result.states["child"]) == 1
####


def test_unreachable_dependent_vehicle_reports_incomplete_execution() -> None:
    parent = RuntimeVehicle(
        "parent",
        RuntimeState(0.0, (0.0,)),
        lambda state: (0.0,),
        step_size=0.1,
        events=(EventCondition("stop", lambda state: 0.0),),
    )
    child = RuntimeVehicle(
        "child",
        RuntimeState(0.0, (0.0,)),
        lambda state: (0.0,),
        step_size=0.1,
        dependencies=("parent",),
        dependency_segments={"parent": 2},
        active=False,
    )

    result = compute_trajectories(build_runtime_problem((parent, child), final_time=1.0))

    assert not result.completed
    assert result.stop_reason == "dependency_unresolved"
    assert child.activation_pending
####


def test_survey_units_and_wind_contracts() -> None:
    cases = generate_survey_cases({"x": (0.0, 1.0, 0.5), "y": (2.0, 3.0)})
    assert len(cases) == 6
    assert cases[0] == {"x": 0.0, "y": 2.0}
    formats = resolve_units_and_formats({"alt": ("km", "f.3")})
    assert formats["alt"].unit is Unit.KILOMETER
    with pytest.raises(ValueError, match="unsupported unit"):
        resolve_units_and_formats({"alt": ("furlong", None)})
    wind = evaluate_wind(magnitude=10.0, heading=0.0, longitude=0.0, latitude=0.0)
    assert wind.ecfc.frame is Frame.ECFC
    assert wind.ecfc.vector == Vector3(0.0, 0.0, 10.0)
    assert Quantity(1.0, Unit.KILOMETER).to(Unit.METER).value == 1000.0
####


def test_optimizer_selection_prefers_scipy_when_available(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("TAORYX_OPTIMIZER", raising=False)

    assert OptimizerBackend.BUILTIN_RQP in available_optimizers()
    assert select_optimizer(OptimizerBackend.BUILTIN_RQP, has_constraints=True) is OptimizerBackend.BUILTIN_RQP
    if OptimizerBackend.SCIPY_SLSQP in available_optimizers():
        assert select_optimizer(OptimizerBackend.AUTO, has_constraints=True) is OptimizerBackend.SCIPY_SLSQP
        assert select_optimizer(OptimizerBackend.AUTO, has_constraints=False) is OptimizerBackend.SCIPY_LBFGSB


def test_selectable_scipy_backend_normalizes_result() -> None:
    pytest.importorskip("scipy")
    runner = resolve_optimize_block(
        lambda point: (point[0] - 3.0) ** 2,
        ((-5.0, 5.0),),
        backend=OptimizerBackend.SCIPY_SLSQP,
    )

    result = runner.run((0.0,))

    assert result.converged
    assert result.parameters == pytest.approx((3.0,), abs=1e-5)
####


def test_optimizer_environment_override(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("TAORYX_OPTIMIZER", "builtin-rqp")

    assert select_optimizer(None, has_constraints=False) is OptimizerBackend.BUILTIN_RQP
####
