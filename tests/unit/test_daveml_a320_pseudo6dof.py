"""Qualification-boundary checks for the A320 surrogate-composite lane."""

from __future__ import annotations

import pytest

from taoryx.trajectory import A320OpenAPOperatingPoint, A320Pseudo6DOFModel, A320Pseudo6DOFOperatingPoint


@pytest.fixture(scope="module")
def pseudo() -> A320Pseudo6DOFModel:
    return A320Pseudo6DOFModel.from_repository()


def test_pseudo6dof_preserves_single_translational_authority(pseudo: A320Pseudo6DOFModel) -> None:
    point = A320Pseudo6DOFOperatingPoint(
        A320OpenAPOperatingPoint(11000.0, 0.78, 60000.0),
        alpha_rad=0.04,
        beta_rad=0.02,
        aileron_rad=0.01,
        elevator_rad=-0.01,
        rudder_rad=0.01,
    )
    result = pseudo.evaluate(point)

    assert result.qualification_class == "surrogate_composite"
    assert result.performance.drag_n == pytest.approx(32807.716586087394, abs=1.0e-9)
    assert result.as_dict()["disabled_contributions"] == [
        "jsbsim.aerodynamic_drag",
        "jsbsim.propulsion_thrust",
        "jsbsim.fuel_flow",
    ]
    assert result.side_force_n != 0.0
    assert result.roll_moment_nm != 0.0
    assert result.pitch_moment_nm != 0.0
    assert result.yaw_moment_nm != 0.0


def test_pseudo6dof_inertia_is_explicitly_mass_scheduled(pseudo: A320Pseudo6DOFModel) -> None:
    light = pseudo.inertia_for_mass(50000.0)
    heavy = pseudo.inertia_for_mass(60000.0)

    assert all(value > 0.0 for value in light)
    assert all(heavy_value > light_value for heavy_value, light_value in zip(heavy, light, strict=True))
    assert pseudo.provenance["source_exact"] is False
    assert pseudo.provenance["inertia_policy"] == "linear_mass_rescale_from_jsbsim_empty_mass"
