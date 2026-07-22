from __future__ import annotations

import numpy as np

from taoryx.trim import TrimSpec, finite_difference_linearization, solve_trim


def _plant(state: dict[str, float], controls: dict[str, float]) -> dict[str, float]:
    x = state["x"]
    u = controls["u"]
    return {"force": x + u - 1.0, "moment": 2.0 * x - u}


def _spec() -> TrimSpec:
    return TrimSpec(
        state_names=("x",),
        control_names=("u",),
        residual_names=("force", "moment"),
        state_initial={"x": 0.0},
        control_initial={"u": 0.0},
        state_lower={"x": -10.0},
        state_upper={"x": 10.0},
        control_lower={"u": -10.0},
        control_upper={"u": 10.0},
        residual_scales={"force": 1.0, "moment": 1.0},
    )


def test_solve_trim_returns_named_plant_bound_solution() -> None:
    result = solve_trim(_spec(), _plant)

    assert result.success
    assert result.state["x"] == 1.0 / 3.0
    assert result.controls["u"] == 2.0 / 3.0
    assert result.max_residual < 1e-10
    assert result.iterations > 0


def test_trim_finite_difference_linearization_is_named_and_local() -> None:
    spec = _spec()
    result = solve_trim(spec, _plant)

    a, b = finite_difference_linearization(spec, _plant, result)

    np.testing.assert_allclose(a, [[1.0], [2.0]], rtol=1e-6, atol=1e-8)
    np.testing.assert_allclose(b, [[1.0], [-1.0]], rtol=1e-6, atol=1e-8)


def test_trim_rejects_duplicate_state_and_control_names() -> None:
    try:
        TrimSpec(
            state_names=("x",),
            control_names=("x",),
            residual_names=("force",),
            state_initial={"x": 0.0},
            control_initial={"x": 0.0},
        )
    except ValueError as error:
        assert "unique" in str(error)
    else:
        raise AssertionError("duplicate trim names should be rejected")
