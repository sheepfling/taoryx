from __future__ import annotations

import math

import pytest

from taoryx.equations import (
    Matrix,
    active_set_equality_problem,
    general_nonlinear_program,
    golden_section_x1,
    golden_section_x2,
    maximum_altitude_constraint,
    maximum_altitude_integral,
    newton_linearization,
    newton_update_search,
    optimization_constraint_linearization,
    optimization_quadratic_approximation,
    optimization_quadratic_subproblem,
    parabolic_minimum,
    parabolic_search_a,
    parabolic_search_b,
    parabolic_search_c,
    parabolic_search_polynomial,
    parabolic_search_roots,
    secant_root_estimate,
)


def test_newton_and_secant_search_helpers_follow_the_manual_formulas() -> None:
    assert newton_linearization(6.0, 2.0, 5.0, 4.0) == pytest.approx(8.0)
    assert newton_update_search(4.0, 6.0, 2.0) == pytest.approx(1.0)
    assert secant_root_estimate(0.0, 2.0, 2.0, -2.0) == pytest.approx(1.0)
####


def test_parabolic_search_helpers_follow_the_manual_formulas() -> None:
    coefficient_b = parabolic_search_b(0.0, 6.0, 1.0, 2.0, 2.0, 0.0)
    coefficient_a = parabolic_search_a(1.0, 2.0, 2.0, 0.0, coefficient_b)
    coefficient_c = parabolic_search_c(1.0, 2.0, coefficient_a, coefficient_b)

    assert coefficient_b == pytest.approx(-5.0)
    assert coefficient_a == pytest.approx(1.0)
    assert coefficient_c == pytest.approx(6.0)
    assert parabolic_search_polynomial(4.0, 1.0, -5.0, 6.0) == pytest.approx(2.0)
    assert parabolic_search_roots(1.0, -5.0, 6.0) == (2.0, 3.0)
    assert parabolic_minimum(1.0, -5.0) == pytest.approx(2.5)
####


def test_golden_section_helpers_follow_the_manual_formulas() -> None:
    assert golden_section_x1(0.0, 1.0) == pytest.approx((3.0 - math.sqrt(5.0)) / 2.0)
    assert golden_section_x2(0.0, 1.0) == pytest.approx((math.sqrt(5.0) - 1.0) / 2.0)
####


def test_optimization_helpers_follow_the_manual_formulas() -> None:
    matrix = Matrix(((1.0, 0.0), (0.0, 1.0)))
    vector = (1.0, 1.0)

    assert optimization_quadratic_approximation((1.0, 2.0), matrix, vector) == pytest.approx(4.0)
    assert optimization_constraint_linearization((10.0, 20.0), Matrix(((1.0, 2.0), (3.0, 4.0))), vector) == (13.0, 27.0)
    objective, constraints = optimization_quadratic_subproblem((1.0, 2.0), matrix, vector, (10.0, 20.0), Matrix(((1.0, 2.0), (3.0, 4.0))))
    assert objective == pytest.approx(4.0)
    assert constraints == (13.0, 27.0)
    assert maximum_altitude_integral(100001.0) == pytest.approx(1.0)
    assert maximum_altitude_integral(99999.0) == pytest.approx(0.0)
    assert maximum_altitude_constraint(-0.5) is True
    assert maximum_altitude_constraint(0.0) is False
    assert general_nonlinear_program("f", ("c1",), ("g1",)).objective == "f"
    assert active_set_equality_problem("f", ("c1",)).constraints == ("c1",)
####
