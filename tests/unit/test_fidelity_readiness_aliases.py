from __future__ import annotations

import pytest

from taoryx.fidelity_readiness import canonical_readiness_tier, validate_fidelity_readiness


def test_readiness_accepts_canonical_and_legacy_pseudo_names() -> None:
    assert canonical_readiness_tier("pseudo_6dof") == "pseudo_6dof"
    assert canonical_readiness_tier("pseudo_6dof_kinematic_bridge") == "pseudo_6dof"
    assert validate_fidelity_readiness("x15", "pseudo_6dof").tier == "pseudo_6dof"
    assert validate_fidelity_readiness("x15", "pseudo_6dof_kinematic_bridge").tier == "pseudo_6dof"


def test_readiness_rejects_ambiguous_rigid_body_name() -> None:
    with pytest.raises(ValueError, match="requires direct_wrench or surface_allocated"):
        canonical_readiness_tier("rigid_body_6dof")
