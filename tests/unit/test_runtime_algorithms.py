from __future__ import annotations

import pytest

from taoryx.contracts import Frame, Quantity, Unit, Vector3
from taoryx.language.expressions import parse_expression
from taoryx.runtime.common import EventCondition, RuntimeState, RuntimeVehicle
from taoryx.runtime.engine import compute_trajectories, get_next_time_step
from taoryx.runtime.environment_runtime import evaluate_wind
from taoryx.runtime.events import apply_state_discontinuity, refine_segment_final_condition
from taoryx.runtime.expressions import evaluate_definition_program
from taoryx.runtime.runtime_model import build_runtime_problem
from taoryx.runtime.surveys import generate_survey_cases
from taoryx.runtime.units import resolve_units_and_formats


def test_runtime_graph_rejects_cycles_and_steps_at_boundaries() -> None:
    first = RuntimeVehicle("a", RuntimeState(0.0, (0.0,)), lambda state: (1.0,), step_size=2.0)
    problem = build_runtime_problem((first,), print_times=(0.5,), final_time=1.0)
    assert get_next_time_step(problem, 2.0) == 0.5
    result = compute_trajectories(problem)
    assert result.completed
    assert problem.vehicles["a"].state.time == pytest.approx(1.0)
    with pytest.raises(ValueError, match="cycle"):
        build_runtime_problem((RuntimeVehicle("a", RuntimeState(0.0, ()), dependencies=("b",)), RuntimeVehicle("b", RuntimeState(0.0, ()), dependencies=("a",))))
####


def test_expression_program_and_events_are_resolved() -> None:
    values = evaluate_definition_program({"a": parse_expression("2"), "b": parse_expression("a * 3")})
    assert values["b"] == 6.0
    state = RuntimeState(0.0, (0.0,))
    end = RuntimeState(2.0, (2.0,))
    crossing = refine_segment_final_condition(state, end, (EventCondition("zero", lambda item: item.values[0] - 1.0),))
    assert crossing[0].time == pytest.approx(1.0)
    assert apply_state_discontinuity(state, (2.0,)).values == (2.0,)
    with pytest.raises(ValueError, match="circular"):
        evaluate_definition_program({"a": parse_expression("b"), "b": parse_expression("a")})
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
