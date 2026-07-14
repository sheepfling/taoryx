from __future__ import annotations

import pytest

import taoryx.optimization as optimization_module
from taoryx.numeric import DifferenceMode
from taoryx.optimization import (
    OptimizationStatus,
    build_optimization_problem,
    han_powell_rqp,
    path_violation_integral,
    redistribute_control_history,
)


def test_build_optimization_problem_preserves_constraint_kinds() -> None:
    problem = build_optimization_problem(
        "range",
        ("vel=4",),
        ("qmin<0",),
        parameters=("par-1",),
        bounds=((0.0, 10.0),),
        references=(100.0,),
    )

    assert problem.objective == "range"
    assert problem.equality_constraints == ("vel=4",)
    assert problem.inequality_constraints == ("qmin<0",)
    assert problem.parameters == ("par-1",)
    assert problem.bounds == ((0.0, 10.0),)
    assert problem.references == (100.0,)


def test_projected_rqp_converges_on_scalar_problem() -> None:
    result = han_powell_rqp(lambda point: (point[0] - 3.0) ** 2, (0.0,), ((-5.0, 5.0),))

    assert result.status is OptimizationStatus.CONVERGED
    assert result.parameters == pytest.approx((3.0,), abs=1e-4)


def test_rqp_propagates_the_configured_difference_mode(monkeypatch: pytest.MonkeyPatch) -> None:
    real = optimization_module.finite_difference_jacobian
    modes: list[DifferenceMode] = []

    def capture(*args, **kwargs):
        modes.append(kwargs["mode"])
        return real(*args, **kwargs)
    ####

    monkeypatch.setattr(optimization_module, "finite_difference_jacobian", capture)
    han_powell_rqp(
        lambda point: (point[0] - 3.0) ** 2,
        (0.0,),
        ((-5.0, 5.0),),
        difference_mode=DifferenceMode.CENTRAL,
        max_iterations=1,
    )

    assert modes
    assert all(mode is DifferenceMode.CENTRAL for mode in modes)
####


def test_path_violation_integral_and_control_redistribution() -> None:
    assert path_violation_integral((-2.0, 0.0, 2.0), lower=-1.0, upper=1.0) == pytest.approx(2.0)
    result = redistribute_control_history((0.0, 1.0), ((0.0, 10.0), (10.0, 20.0)), (0.25, 0.75))

    assert result[0] == pytest.approx((2.5, 12.5))
    assert result[1] == pytest.approx((7.5, 17.5))
