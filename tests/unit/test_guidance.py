from __future__ import annotations

import math

import pytest

from taoryx.equations import CartesianVector3
from taoryx.guidance import cubic_guidance_correction, parabolic_guidance_correction, predictive_intercept
from taoryx.numeric import NewtonSystemStatus, newton_system


def test_parabolic_guidance_correction_matches_terminal_constraints() -> None:
    result = parabolic_guidance_correction(0.0, 10.0, 2.0, 0.0, 5.0)

    assert result == pytest.approx(2.0)
####


def test_cubic_guidance_correction_is_zero_for_matched_constant_state() -> None:
    assert cubic_guidance_correction(0.0, 5.0, 3.0, 3.0, 0.0, 0.0) == pytest.approx(0.0)
####


def test_guidance_corrections_reject_zero_interval() -> None:
    with pytest.raises(ValueError, match="interval"):
        parabolic_guidance_correction(0.0, 1.0, 0.0, 0.0, 0.0)


def test_newton_system_solves_bounded_residuals() -> None:
    result = newton_system(
        lambda controls: (controls[0] - 2.0, controls[1] + 1.0),
        (0.0, 0.0),
        ((-5.0, 5.0), (-5.0, 5.0)),
        1e-5,
        1e-10,
    )

    assert result.status is NewtonSystemStatus.CONVERGED
    assert result.controls == pytest.approx((2.0, -1.0))
    assert result.residuals == pytest.approx((0.0, 0.0), abs=1e-10)


def test_newton_system_clips_control_updates() -> None:
    result = newton_system(
        lambda controls: (controls[0] - 5.0,),
        (0.0,),
        ((0.0, 1.0),),
        1e-5,
        1e-10,
    )

    assert result.controls == pytest.approx((1.0,))
    assert result.status is NewtonSystemStatus.MAX_ITERATIONS


def test_predictive_intercept_returns_time_point_and_line_of_sight_angles() -> None:
    result = predictive_intercept(
        CartesianVector3(0.0, 0.0, 0.0),
        CartesianVector3(1.0, 0.0, 0.0),
        CartesianVector3(0.0, 10.0, 0.0),
        CartesianVector3(0.0, 0.0, 0.0),
    )

    assert result.time_to_intercept_seconds == pytest.approx(10.0)
    assert result.intercept_point == CartesianVector3(0.0, 10.0, 0.0)
    assert result.heading_radians == pytest.approx(math.pi / 2.0)
    assert result.flight_path_angle_radians == pytest.approx(0.0)
