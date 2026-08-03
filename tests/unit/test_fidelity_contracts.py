import pytest

from taoryx.fidelity_contracts import (
    CANONICAL_FIDELITY_TIERS,
    canonicalize_fidelity,
    control_realization_for,
    parent_fidelity,
    runtime_fidelity_for,
)


def test_canonical_tier_order_is_the_horizontal_ladder() -> None:
    assert CANONICAL_FIDELITY_TIERS == (
        "point_mass_3dof",
        "pseudo_6dof",
        "rigid_body_6dof_direct_wrench",
        "rigid_body_6dof_surface_allocated",
    )


def test_parent_and_control_realization_are_deterministic() -> None:
    assert parent_fidelity("point_mass_3dof") is None
    assert parent_fidelity("pseudo_6dof") == "point_mass_3dof"
    assert parent_fidelity("rigid_body_6dof_direct_wrench") == "pseudo_6dof"
    assert parent_fidelity("rigid_body_6dof_surface_allocated") == "rigid_body_6dof_direct_wrench"
    assert control_realization_for("pseudo_6dof") == "response_law"
    assert control_realization_for("rigid_body_6dof_direct_wrench") == "direct_wrench"
    assert runtime_fidelity_for("rigid_body_6dof_direct_wrench") == "rigid_body_6dof"
    assert runtime_fidelity_for("rigid_body_6dof_surface_allocated") == "rigid_body_6dof"


def test_legacy_rigid_body_name_requires_an_explicit_realization() -> None:
    assert canonicalize_fidelity("rigid_body_6dof", control_realization="direct_wrench") == "rigid_body_6dof_direct_wrench"
    assert canonicalize_fidelity("rigid_body_6dof", control_realization="surface_allocated") == "rigid_body_6dof_surface_allocated"
    with pytest.raises(ValueError, match="ambiguous"):
        canonicalize_fidelity("rigid_body_6dof")
