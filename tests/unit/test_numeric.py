from __future__ import annotations

import pytest

from taoryx.numeric import DifferenceMode, Jacobian, finite_difference_jacobian


def test_forward_and_central_differences_compute_scalar_gradient() -> None:
    function = lambda point: point[0] ** 2 + 3.0 * point[1]

    forward = finite_difference_jacobian(function, (2.0, 4.0), (1e-5, 1e-5), mode=DifferenceMode.FORWARD)
    central = finite_difference_jacobian(function, (2.0, 4.0), (1e-5, 1e-5), mode=DifferenceMode.CENTRAL)

    assert forward.gradient() == pytest.approx((4.00001, 3.0), rel=1e-6)
    assert central.gradient() == pytest.approx((4.0, 3.0))
####


def test_vector_function_returns_output_by_input_jacobian() -> None:
    def function(point: tuple[float, ...]) -> tuple[float, float]:
        x, y = point
        return x * y, x + y * y

    jacobian = finite_difference_jacobian(function, (2.0, 3.0), 1e-5)

    assert jacobian.output_dimension == 2
    assert jacobian.input_dimension == 2
    assert jacobian.rows[0] == pytest.approx((3.0, 2.0))
    assert jacobian.rows[1] == pytest.approx((1.0, 6.0))
####


def test_finite_difference_rejects_ambiguous_or_invalid_inputs() -> None:
    with pytest.raises(ValueError, match="one finite nonzero"):
        finite_difference_jacobian(lambda point: point[0], (1.0, 2.0), (1.0,))
    with pytest.raises(ValueError, match="one finite nonzero"):
        finite_difference_jacobian(lambda point: point[0], (1.0,), 0.0)
    with pytest.raises(ValueError, match="equal width"):
        Jacobian(((1.0,), (1.0, 2.0)))
####
