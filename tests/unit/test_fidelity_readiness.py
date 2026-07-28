from __future__ import annotations

import importlib.util
from pathlib import Path

from taoryx.fidelity_readiness import validate_fidelity_readiness

ROOT = Path(__file__).parents[2]
SPEC = importlib.util.spec_from_file_location("validate_fidelity_readiness", ROOT / "tools/validate_fidelity_readiness.py")
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

def test_b747_direct_wrench_readiness_is_distinct_from_surface_readiness() -> None:
    direct = validate_fidelity_readiness("b747", "rigid_body_6dof_direct_wrench")
    surface = validate_fidelity_readiness("b747", "rigid_body_6dof_surface_allocated")

    assert not direct.errors
    assert surface.status == "partial"
    assert not surface.errors
    assert any(item.requirement_id == "residual_telemetry" for item in surface.warnings)
    ####


def test_hummingbird_has_a_declared_rotor_allocation_contract() -> None:
    report = validate_fidelity_readiness("hummingbird", "rigid_body_6dof_surface_allocated")

    assert not any(item.requirement_id == "allocator_declaration" for item in report.errors)
    assert not any(item.requirement_id == "rotor_controls" for item in report.errors)
    assert not any(item.requirement_id == "rotor_geometry" for item in report.errors)
    ####


def test_x8_aerodynamic_convention_is_complete_for_direct_wrench() -> None:
    report = validate_fidelity_readiness("skywalker_x8", "rigid_body_6dof_direct_wrench")

    assert not any(item.requirement_id == "aero_convention" for item in report.errors)
    ####


def test_x15_direct_wrench_authority_is_explicitly_bounded() -> None:
    report = validate_fidelity_readiness("x15", "rigid_body_6dof_direct_wrench")

    assert not any(item.requirement_id == "direct_authority" for item in report.errors)
    assert not any(item.requirement_id == "attitude_limits" for item in report.errors)
    ####


def test_readiness_does_not_claim_runtime_qualification() -> None:
    report = validate_fidelity_readiness("skywalker_x8", "point_mass_3dof")

    assert report.runtime_proof_status == "not_evaluated"
    assert "runtime_proof_status" in report.as_dict()
    ####


def test_ordered_blocker_is_the_first_blocked_tier() -> None:
    reports = tuple(validate_fidelity_readiness("x15", tier) for tier in MODULE.TIERS)

    blocker = MODULE._ordered_blocker(reports)

    assert blocker is not None
    assert blocker.tier == "pseudo_6dof_kinematic_bridge"
