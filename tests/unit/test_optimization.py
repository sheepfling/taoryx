from __future__ import annotations

import pytest

from taoryx.optimization import (
    OptimizationStatus,
    build_optimization_problem,
    han_powell_rqp,
    path_violation_integral,
    redistribute_control_history,
)


def test_build_optimization_problem_preserves_constraint_kinds() -> None:
    problem = build_optimization_problem("range", ("vel=4",), ("qmin<0",))

    assert problem.objective == "range"
    assert problem.equality_constraints == ("vel=4",)
    assert problem.inequality_constraints == ("qmin<0",)


def test_projected_rqp_converges_on_scalar_problem() -> None:
    result = han_powell_rqp(lambda point: (point[0] - 3.0) ** 2, (0.0,), ((-5.0, 5.0),))

    assert result.status is OptimizationStatus.CONVERGED
    assert result.parameters == pytest.approx((3.0,), abs=1e-4)


def test_path_violation_integral_and_control_redistribution() -> None:
    assert path_violation_integral((-2.0, 0.0, 2.0), lower=-1.0, upper=1.0) == pytest.approx(2.0)
    result = redistribute_control_history((0.0, 1.0), ((0.0, 10.0), (10.0, 20.0)), (0.25, 0.75))

    assert result[0] == pytest.approx((2.5, 12.5))
    assert result[1] == pytest.approx((7.5, 17.5))
