"""Tests for existing-family and new-topology intake generation."""

from __future__ import annotations

import pytest

from taoryx.vehicle_integration_intake import (
    ExistingFamilyIntakeRequest,
    NewTopologyIntakeRequest,
    StrategySelectionError,
    build_existing_family_intake_blueprint,
    build_new_topology_intake_scaffold,
    compatible_strategies,
)


def test_existing_fixed_wing_intake_generates_all_four_tier_work_packages() -> None:
    blueprint = build_existing_family_intake_blueprint(
        ExistingFamilyIntakeRequest(
            family_id="test_uav",
            physical_family="powered_fixed_wing",
            mission_overlay="fixed_wing_racetrack",
            adapter_id="taoryx.fixed_wing.test_uav.v1",
            strategy_id="powered_fixed_wing.v1",
        )
    )

    payload = blueprint.as_dict()
    assert payload["status"] == "ready_for_source_and_adapter_binding"
    assert payload["strategy_id"] == "powered_fixed_wing.v1"
    raw_tiers = payload["tiers"]
    assert isinstance(raw_tiers, list)
    tiers = [item for item in raw_tiers if isinstance(item, dict)]
    assert len(tiers) == 4
    direct = next(item for item in tiers if item["tier"] == "rigid_body_6dof_direct_wrench")
    surface = next(item for item in tiers if item["tier"] == "rigid_body_6dof_surface_allocated")
    assert direct["operating_point_campaign_required"] is True
    assert surface["operating_point_campaign_required"] is True
    assert surface["required_operations"] == ["state_derivative", "trim", "linearize", "effectiveness", "allocate"]
    registry = payload["registry_entry_template"]
    assert isinstance(registry, dict)
    registry_tiers = registry["tiers"]
    assert isinstance(registry_tiers, dict)
    pseudo = registry_tiers["pseudo_6dof"]
    assert isinstance(pseudo, dict)
    assert pseudo["promotion_status"] == "planned"
    composition = payload["mission_composition"]
    assert isinstance(composition, dict)
    assert composition["template_id"] == "powered_fixed_wing_racetrack_v1"
    assert "maximum_bank_deg" in composition["capability_profile_required"]
    assert "does not prove" in composition["claim_boundary"]
    ####


def test_ambiguous_powered_fixed_wing_requires_an_explicit_strategy_choice() -> None:
    request = ExistingFamilyIntakeRequest(
        family_id="test_ambiguous",
        physical_family="powered_fixed_wing",
        mission_overlay="unknown",
        adapter_id="taoryx.test.v1",
    )

    with pytest.raises(StrategySelectionError, match="ambiguous"):
        build_existing_family_intake_blueprint(request)
    assert {strategy.id for strategy in compatible_strategies("powered_fixed_wing")} == {
        "powered_fixed_wing.v1",
        "rocket_aircraft_high_energy.v1",
    }
    ####


def test_new_topology_scaffold_demands_all_four_fidelity_decisions_without_inventing_data() -> None:
    scaffold = build_new_topology_intake_scaffold(
        NewTopologyIntakeRequest(
            family_id="test_tailsitter",
            physical_family="tailsitter_vtol",
            proposed_strategy_id="tailsitter_transition_vtol.v1",
            topology_summary="Propeller-driven vertical takeoff with a pitch-over transition to wing-borne flight.",
        )
    )

    payload = scaffold.as_dict()
    assert payload["status"] == "strategy_definition_required"
    raw_tiers = payload["tiers"]
    assert isinstance(raw_tiers, list)
    tiers = [item for item in raw_tiers if isinstance(item, dict)]
    assert len(tiers) == 4
    assert tiers[0]["control_realization"] == "force_model"
    assert tiers[-1]["control_realization"] == "surface_allocated"
    claim_boundary = payload["claim_boundary"]
    assert isinstance(claim_boundary, str)
    assert "not a vehicle model" in claim_boundary
    ####
