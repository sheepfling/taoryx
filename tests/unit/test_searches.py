from __future__ import annotations

import math

import pytest

from taoryx.searches import (
    MinimizationStatus,
    RootSearchStatus,
    golden_section_minimize,
    newton_root,
    parabolic_minimize,
    parabolic_root,
    secant_bracketed_root,
)


def test_newton_root_solves_linear_quadratic_and_transcendental_roots() -> None:
    linear = newton_root(lambda x: 2.0 * x - 6.0, 0.0, (-10.0, 10.0), 1e-5, 1e-10)
    quadratic = newton_root(lambda x: x * x - 2.0, 1.0, (0.0, 2.0), 1e-5, 1e-10)
    transcendental = newton_root(math.sin, 3.0, (0.0, 4.0), 1e-5, 1e-10)

    assert linear.status is RootSearchStatus.CONVERGED
    assert linear.root == pytest.approx(3.0)
    assert quadratic.root == pytest.approx(math.sqrt(2.0))
    assert transcendental.root == pytest.approx(math.pi)
####


def test_newton_root_reports_zero_derivative_boundary_and_iteration_failures() -> None:
    zero_derivative = newton_root(lambda x: 1.0, 0.0, (-1.0, 1.0), 1e-4, 1e-10)
    boundary = newton_root(lambda x: x - 2.0, 1.0, (0.0, 1.0), 1e-4, 1e-10)
    exhausted = newton_root(lambda x: x * x - 2.0, 1.0, (0.0, 2.0), 1e-5, 1e-30, max_iterations=1)

    assert zero_derivative.status is RootSearchStatus.ZERO_DERIVATIVE
    assert boundary.status is RootSearchStatus.BOUNDARY_STALL
    assert exhausted.status is RootSearchStatus.MAX_ITERATIONS
####


def test_newton_root_rejects_invalid_bounds_and_step() -> None:
    with pytest.raises(ValueError, match="lower bound"):
        newton_root(lambda x: x, 0.0, (1.0, 0.0), 1e-4, 1e-8)
    with pytest.raises(ValueError, match="derivative step"):
        newton_root(lambda x: x, 0.0, (-1.0, 1.0), 0.0, 1e-8)
    with pytest.raises(ValueError, match="initial estimate"):
        newton_root(lambda x: x, 2.0, (-1.0, 1.0), 1e-4, 1e-8)
####


def test_bracketed_secant_solves_root_and_preserves_sign_change() -> None:
    result = secant_bracketed_root(lambda x: x * x - 2.0, (0.0, 2.0), 1e-10)

    assert result.status is RootSearchStatus.CONVERGED
    assert result.root == pytest.approx(math.sqrt(2.0))
    assert result.bracket[0] <= result.root <= result.bracket[1]
    assert (result.bracket[0] * result.bracket[0] - 2.0) * (result.bracket[1] * result.bracket[1] - 2.0) <= 0.0
####


def test_bracketed_secant_handles_endpoint_and_invalid_bracket_cases() -> None:
    endpoint = secant_bracketed_root(lambda x: x - 1.0, (1.0, 3.0), 1e-12)
    invalid = secant_bracketed_root(lambda x: x + 1.0, (1.0, 3.0), 1e-12)

    assert endpoint.status is RootSearchStatus.CONVERGED
    assert endpoint.iterations == 0
    assert invalid.status is RootSearchStatus.INVALID_BRACKET
    assert invalid.bracket == (1.0, 3.0)
####


def test_golden_section_minimizes_convex_and_boundary_objectives() -> None:
    interior = golden_section_minimize(lambda x: (x - 2.5) ** 2 + 1.0, (-1.0, 7.0), 1e-8)
    boundary = golden_section_minimize(lambda x: x * x, (0.0, 4.0), 1e-8)

    assert interior.status is MinimizationStatus.CONVERGED
    assert interior.minimum == pytest.approx(2.5, abs=1e-7)
    assert interior.value == pytest.approx(1.0, abs=1e-12)
    assert boundary.minimum == pytest.approx(0.0, abs=1e-7)
    assert boundary.value == pytest.approx(0.0, abs=1e-12)
    assert interior.interval[1] - interior.interval[0] <= 1e-8
####


def test_golden_section_reports_iteration_limit_and_rejects_bad_tolerance() -> None:
    limited = golden_section_minimize(lambda x: (x - 1.0) ** 2, (-2.0, 3.0), 1e-20, max_iterations=2)

    assert limited.status is MinimizationStatus.MAX_ITERATIONS
    assert limited.iterations == 2
    with pytest.raises(ValueError, match="interval tolerance"):
        golden_section_minimize(lambda x: x * x, (0.0, 1.0), 0.0)
####


def test_parabolic_searches_solve_exact_quadratics() -> None:
    root = parabolic_root(lambda x: (x - 2.0) * (x + 1.0), 1.0, 0.5, (-3.0, 3.0), 1e-12)
    minimum = parabolic_minimize(lambda x: (x - 1.5) ** 2 + 4.0, 1.0, 0.5, (-3.0, 3.0), 1e-12)

    assert root.status is RootSearchStatus.CONVERGED
    assert root.root == pytest.approx(2.0)
    assert minimum.status is MinimizationStatus.CONVERGED
    assert minimum.minimum == pytest.approx(1.5)
    assert minimum.value == pytest.approx(4.0)
####


def test_parabolic_searches_report_degenerate_startup_sets() -> None:
    with pytest.raises(ValueError, match="three distinct"):
        parabolic_root(lambda x: x, 0.0, 1.0, (-0.5, 0.0), 1e-8)
