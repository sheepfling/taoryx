from __future__ import annotations

import pytest
from pydantic import ValidationError
from taoryx.families.cadac.manifest import (
    CADAC_MANIFEST_CATALOG,
    CadacAllocationGranularity,
    CadacControlRealization,
    CadacPackageId,
    CadacPhaseFidelity,
    manifest_for,
)


def test_catalog_classifies_all_source_packages() -> None:
    assert {package.package_id for package in CADAC_MANIFEST_CATALOG.packages} == set(CadacPackageId)


####


def test_aim5_is_pseudo_and_falcon6_is_physical_effector() -> None:
    aim5 = manifest_for("AIM5").actors[0].phases[0]
    falcon6 = manifest_for("falcon6").actors[0].phases[0]

    assert aim5.taoryx_tier == "pseudo_6dof"
    assert aim5.control_realization is CadacControlRealization.RESPONSE_LAW
    assert falcon6.taoryx_tier == "rigid_body_6dof_surface_allocated"
    assert falcon6.control_realization is CadacControlRealization.EFFECTOR_ALLOCATED


####


def test_rocket6g_retains_mixed_phase_realization() -> None:
    phases = {phase.phase_id: phase for phase in manifest_for(CadacPackageId.ROCKET6G).actors[0].phases}

    assert phases["aggregate_rcs"].taoryx_tier == "rigid_body_6dof_direct_wrench"
    assert phases["physical_tvc"].taoryx_tier == "rigid_body_6dof_surface_allocated"
    assert phases["mixed_tvc_rcs"].allocation_granularity is CadacAllocationGranularity.MIXED


####


def test_highest_tier_rejects_missing_physical_effector_evidence() -> None:
    with pytest.raises(ValidationError, match="identify physical effectors"):
        CadacPhaseFidelity(
            phase_id="invalid",
            description="Invalid claim",
            taoryx_tier="rigid_body_6dof_surface_allocated",
            runtime_fidelity="rigid_body_6dof",
            control_realization="effector_allocated",
            allocation_granularity="fixed_mixing",
            actuator_dynamics="second_order",
        )
    ####


####


def test_sraam6_manifest_preserves_exact_source_actor_names_and_optional_tvc() -> None:
    manifest = manifest_for(CadacPackageId.SRAAM6)
    missile, target = manifest.actors

    assert missile.source_model == "MISSILE6"
    assert target.source_model == "TARGET3"
    phases = {phase.phase_id: phase for phase in missile.phases}
    assert phases["fin_control"].taoryx_tier == "rigid_body_6dof_surface_allocated"
    assert phases["optional_tvc"].control_realization is CadacControlRealization.EFFECTOR_ALLOCATED


####


def test_ghame6_manifest_corrects_tvc_assumption_and_preserves_phase_fidelity() -> None:
    manifest = manifest_for(CadacPackageId.GHAME6)
    vehicle, satellite, radar = manifest.actors
    phases = {phase.phase_id: phase for phase in vehicle.phases}

    assert vehicle.source_model == "HYPER6"
    assert satellite.source_model == "SAT3"
    assert radar.source_model == "RADAR0"
    assert phases["atmospheric_surfaces"].taoryx_tier == "rigid_body_6dof_surface_allocated"
    assert phases["atmospheric_surfaces"].control_realization is CadacControlRealization.EFFECTOR_ALLOCATED
    assert {phase.taoryx_tier for name, phase in phases.items() if name != "atmospheric_surfaces"} == {"rigid_body_6dof_direct_wrench"}
    assert all("tvc" not in module.casefold() for phase in vehicle.phases for module in phase.source_modules)
    assert any("no TVC" in note for note in manifest.notes)


####
