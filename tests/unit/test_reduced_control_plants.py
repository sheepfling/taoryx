"""Regression tests for the shared lower-tier trim/linearization seam."""

from __future__ import annotations

import math

import pytest

from taoryx.trajectory.hummingbird_adapter import HummingbirdPointMassControlPlant, HummingbirdPseudo6DOFControlPlant


@pytest.mark.parametrize(
    ("plant_type", "expected_state_count", "expected_control_count"),
    (
        (HummingbirdPointMassControlPlant, 7, 3),
        (HummingbirdPseudo6DOFControlPlant, 13, 4),
    ),
)
def test_hummingbird_reduced_plants_expose_real_lower_tier_trim_and_derivatives(
    plant_type: type[HummingbirdPointMassControlPlant] | type[HummingbirdPseudo6DOFControlPlant],
    expected_state_count: int,
    expected_control_count: int,
) -> None:
    plant = plant_type()

    trim = plant.trim({}, {})
    derivative = plant.state_derivative(trim.state, trim.controls, {})
    linearization = plant.linearize(trim, {})

    assert trim.success
    assert trim.max_residual < 1.0e-9
    assert len(derivative) == expected_state_count
    assert all(math.isfinite(value) for value in derivative.values())
    assert linearization.primary.a_matrix.shape == (expected_state_count, expected_state_count)
    assert linearization.primary.b_matrix.shape == (expected_state_count, expected_control_count)
    assert linearization.primary.metadata["control_realization"] == "reduced_force_or_response_law"


def test_hummingbird_reduced_plants_reject_physical_effector_promotion() -> None:
    plant = HummingbirdPseudo6DOFControlPlant()
    trim = plant.trim({}, {})

    with pytest.raises(NotImplementedError, match="no physical effector effectiveness"):
        plant.effectiveness(trim.state, trim.controls)
    with pytest.raises(NotImplementedError, match="no physical effector allocator"):
        plant.allocate(trim.state, {}, trim.controls, 0.01)
