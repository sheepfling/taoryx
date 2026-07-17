import numpy as np
import pytest

from taoryx.runtime import LqrController, solve_continuous_lqr


def test_continuous_lqr_solves_and_stabilizes_double_integrator() -> None:
    result = solve_continuous_lqr(
        ((0.0, 1.0), (0.0, 0.0)),
        ((0.0,), (1.0,)),
        ((1.0, 0.0), (0.0, 1.0)),
        ((1.0,),),
        state_names=("position", "velocity"),
        control_names=("acceleration",),
    )

    assert result.gain.shape == (1, 2)
    assert result.controllable is True
    assert np.all(np.real(result.closed_loop_eigenvalues) < 0.0)
    assert result.state_names == ("position", "velocity")


def test_lqr_controller_closes_double_integrator_with_named_bounded_command() -> None:
    result = solve_continuous_lqr(
        ((0.0, 1.0), (0.0, 0.0)),
        ((0.0,), (1.0,)),
        ((1.0, 0.0), (0.0, 1.0)),
        ((1.0,),),
        state_names=("position", "velocity"),
        control_names=("acceleration",),
    )
    controller = LqrController(result, upper={"acceleration": 0.5}, lower={"acceleration": -0.5})

    command = controller.command({"position": 2.0, "velocity": 0.0})

    assert command.controls["acceleration"] == pytest.approx(-0.5)
    assert command.unsaturated["acceleration"] < -0.5
    assert command.saturated == ("acceleration",)


def test_lqr_controller_rejects_missing_named_state() -> None:
    result = solve_continuous_lqr(
        ((0.0, 1.0), (0.0, 0.0)),
        ((0.0,), (1.0,)),
        ((1.0, 0.0), (0.0, 1.0)),
        ((1.0,),),
        state_names=("position", "velocity"),
        control_names=("acceleration",),
    )
    with pytest.raises(KeyError, match="velocity"):
        LqrController(result).command({"position": 1.0})


@pytest.mark.parametrize(
    "matrices, message",
    [
        ((((0.0,),), ((1.0,), (1.0,)), ((1.0,),), ((1.0,),)), "same row count"),
        ((((0.0, 1.0),), ((1.0,),), ((1.0,),), ((1.0,),)), "square"),
        ((((0.0,),), ((1.0,),), ((1.0,),), ((0.0,),)), "positive definite"),
    ],
)
def test_lqr_rejects_invalid_matrix_contract(matrices: tuple[object, ...], message: str) -> None:
    with pytest.raises(ValueError, match=message):
        solve_continuous_lqr(*matrices)  # type: ignore[arg-type]
