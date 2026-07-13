from __future__ import annotations

import pytest

from taoryx.aerodynamics import aerodynamic_performance_metrics, maximum_lift_to_drag


def test_aerodynamic_performance_metrics_match_catalog_equations() -> None:
    result = aerodynamic_performance_metrics(100.0, 0.8, 0.2, 5.0)

    assert result.ballistic_coefficient == pytest.approx(100.0)
    assert result.lift_to_drag_ratio == pytest.approx(4.0)
####


def test_maximum_lift_to_drag_finds_bounded_quadratic_peak() -> None:
    result = maximum_lift_to_drag(lambda angle: (4.0 - (angle - 0.25) ** 2, 1.0), (-1.0, 1.0), 1e-8)

    assert result.angle == pytest.approx(0.25, abs=1e-6)
    assert result.lift_to_drag_ratio == pytest.approx(4.0, abs=1e-8)
####


def test_aerodynamic_metrics_reject_zero_drag() -> None:
    with pytest.raises(ValueError, match="zero drag"):
        aerodynamic_performance_metrics(1.0, 1.0, 0.0, 1.0)
