from __future__ import annotations

from taoryx.fidelity_readiness import validate_fidelity_readiness


def test_b747_direct_wrench_readiness_is_distinct_from_surface_readiness() -> None:
    direct = validate_fidelity_readiness("b747", "rigid_body_6dof_direct_wrench")
    surface = validate_fidelity_readiness("b747", "rigid_body_6dof_surface_allocated")

    assert not direct.errors
    assert surface.status == "blocked"
    assert any(item.requirement_id == "allocator_declaration" for item in surface.errors)
    ####


def test_hummingbird_has_a_declared_rotor_allocation_contract() -> None:
    report = validate_fidelity_readiness("hummingbird", "rigid_body_6dof_surface_allocated")

    assert not any(item.requirement_id == "allocator_declaration" for item in report.errors)
    assert not any(item.requirement_id == "rotor_controls" for item in report.errors)
    ####


def test_readiness_does_not_claim_runtime_qualification() -> None:
    report = validate_fidelity_readiness("skywalker_x8", "point_mass_3dof")

    assert report.runtime_proof_status == "not_evaluated"
    assert "runtime_proof_status" in report.as_dict()
    ####
