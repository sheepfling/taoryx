import pytest

from taoryx.simulation import SimulationRunner, SimulationState, TerminationReason


def test_runner_advances_constant_rate_and_records_history() -> None:
    initial = SimulationState(time=0.0, values=(2.0,), frame="test")

    result = SimulationRunner().run(
        initial,
        lambda state: (3.0,),
        step_size=0.5,
        max_steps=2,
    )

    assert result.reason is TerminationReason.MAX_STEPS
    assert [state.time for state in result.states] == [0.0, 0.5, 1.0]
    assert result.states[-1].values == (5.0,)
    assert result.states[-1].frame == "test"


def test_runner_stops_after_a_state_satisfies_condition() -> None:
    result = SimulationRunner().run(
        SimulationState(0.0, (0.0,)),
        lambda state: (1.0,),
        step_size=1.0,
        max_steps=10,
        stop_when=lambda state: state.time >= 2.0,
    )

    assert result.reason is TerminationReason.STOP_CONDITION
    assert result.states[-1].time == 2.0


def test_runner_rejects_invalid_step_and_limit() -> None:
    runner = SimulationRunner()
    initial = SimulationState(0.0, (0.0,))

    with pytest.raises(ValueError, match="step_size"):
        runner.run(initial, lambda state: (0.0,), step_size=0.0, max_steps=1)
    with pytest.raises(ValueError, match="max_steps"):
        runner.run(initial, lambda state: (0.0,), step_size=1.0, max_steps=0)
