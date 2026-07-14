from __future__ import annotations

import math

import pytest

from taoryx.integration import IntegratorName, RK4Integrator, available_integrators, euler_step, normalize_integrator, rk4_step, rkf45_step, scipy_ivp_step
from taoryx.simulation import SimulationState


def test_rk4_step_integrates_constant_derivative_and_preserves_frame() -> None:
    state = SimulationState(2.0, (1.0, -3.0), "ecfc")
    calls: list[SimulationState] = []

    def derivative(current: SimulationState) -> tuple[float, float]:
        calls.append(current)
        return (4.0, -2.0)

    result = rk4_step(derivative, state, 0.25)

    assert result.time == pytest.approx(2.25)
    assert result.values == pytest.approx((2.0, -3.5))
    assert result.frame == "ecfc"
    assert len(calls) == 4
    assert [call.time for call in calls] == pytest.approx([2.0, 2.125, 2.125, 2.25])
####


def test_euler_step_is_fast_first_order_reference_and_preserves_frame() -> None:
    state = SimulationState(2.0, (1.0, -3.0), "ecfc")

    result = euler_step(lambda _: (4.0, -2.0), state, 0.25)

    assert result.time == pytest.approx(2.25)
    assert result.values == pytest.approx((2.0, -3.5))
    assert result.frame == "ecfc"
####


def test_rk4_step_has_fourth_order_accuracy_for_exponential_growth() -> None:
    state = SimulationState(0.0, (1.0,))

    def derivative(current: SimulationState) -> tuple[float]:
        return (current.values[0],)

    coarse = rk4_step(derivative, state, 0.2).values[0]
    fine_state = rk4_step(derivative, rk4_step(derivative, state, 0.1), 0.1)
    coarse_error = abs(coarse - math.exp(0.2))
    fine_error = abs(fine_state.values[0] - math.exp(0.2))

    assert coarse == pytest.approx(math.exp(0.2), rel=1e-5)
    assert coarse_error / fine_error == pytest.approx(16.0, rel=0.1)
####


def test_rk4_integrator_adapter_and_validation() -> None:
    state = SimulationState(0.0, (1.0,))
    integrator = RK4Integrator()
    with pytest.raises(ValueError, match="step_size"):
        integrator.step(lambda _: (1.0,), state, 0.0)
    with pytest.raises(ValueError, match="dimension"):
        rk4_step(lambda _: (1.0, 2.0), state, 0.1)
    with pytest.raises(ValueError, match="finite"):
        rk4_step(lambda _: (float("nan"),), state, 0.1)
####


def test_rkf45_step_adapts_and_preserves_state_frame() -> None:
    state = SimulationState(0.0, (1.0,), "iip")
    result = rkf45_step(lambda current: (current.values[0],), state, 0.5, 1e-10, 1e-8)

    assert result.state.frame == "iip"
    assert result.state.time == pytest.approx(result.accepted_step)
    assert 0.0 < result.accepted_step <= 0.5
    assert result.state.values[0] == pytest.approx(math.exp(result.accepted_step), rel=1e-6)
    assert result.error_norm <= 1.0
    assert result.next_step > 0.0
####


def test_integrator_selection_names_are_explicit() -> None:
    assert normalize_integrator("RKF45") is IntegratorName.RKF45
    assert normalize_integrator(IntegratorName.RK4) is IntegratorName.RK4
    assert IntegratorName.RK4 in available_integrators()
    assert IntegratorName.EULER in available_integrators()
    with pytest.raises(ValueError, match="unknown integrator"):
        normalize_integrator("not-an-integrator")
####


def test_scipy_ivp_dop853_matches_reference_for_exponential_growth() -> None:
    pytest.importorskip("scipy")
    state = SimulationState(0.0, (1.0,), "ecfc")
    result = scipy_ivp_step(
        lambda current: (current.values[0],),
        state,
        0.5,
        1e-10,
        1e-8,
        method="DOP853",
    )

    assert result.frame == state.frame
    assert result.time == pytest.approx(0.5)
    assert result.values[0] == pytest.approx(math.exp(0.5), rel=1e-8)
####
