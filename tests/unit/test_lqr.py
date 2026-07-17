import numpy as np
import pytest

from taoryx.runtime import solve_continuous_lqr


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
