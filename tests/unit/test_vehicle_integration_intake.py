"""Tests for existing-family and new-topology intake generation."""

from __future__ import annotations

import json

import pytest

from taoryx.runtime.cli import main
from taoryx.vehicle_integration_intake import (
    ExistingFamilyIntakeRequest,
    NewTopologyIntakeRequest,
    StrategySelectionError,
    build_existing_family_intake_blueprint,
    build_new_topology_intake_scaffold,
    compatible_strategies,
    validate_integration_intake_payload,
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
    capability_contracts = composition["capability_profile_parameter_contracts"]
    assert isinstance(capability_contracts, list)
    operating_point = next(item for item in capability_contracts if item["id"] == "operating_point_id")
    assert operating_point["value_type"] == "enum"
    assert operating_point["value_type_source"] == "declared"
    assert operating_point["value_space"]["topology"] == "finite_set"
    assert operating_point["source_value_status"] == "authoring_required"
    mission_contracts = composition["semantic_intent_parameter_contracts"]
    assert isinstance(mission_contracts, list)
    requested_bank = next(item for item in mission_contracts if item["id"] == "requested_bank_deg")
    assert requested_bank["value_space"]["topology"] == "bounded_interval"
    assert requested_bank["hard_upper"] == 89.0
    assert all(item["default_declared"] is False for item in mission_contracts)
    assert "does not prove" in composition["claim_boundary"]
    interface = payload["interface_contract_required"]
    assert isinstance(interface, dict)
    parameter_contract = interface["parameter_contract"]
    assert isinstance(parameter_contract, dict)
    assert "value_space_topology_and_representation" in parameter_contract["required_per_parameter"]
    control_contract = interface["control_contract"]
    assert isinstance(control_contract, dict)
    assert "commanded_vs_achieved_telemetry" in control_contract["required_per_action_or_effector"]
    execution_mode_contract = interface["execution_mode_contract"]
    assert isinstance(execution_mode_contract, dict)
    assert "closed_loop_controller" in execution_mode_contract["allowed_values"]
    assert "source_scheduled_replay" in execution_mode_contract["allowed_values"]
    witnesses = payload["synthetic_conformance_witness_plan"]
    assert isinstance(witnesses, list)
    assert [item["tier"] for item in witnesses] == [
        "point_mass_3dof",
        "pseudo_6dof",
        "rigid_body_6dof_direct_wrench",
        "rigid_body_6dof_surface_allocated",
    ]
    execution = interface["execution_and_evaluation_contract"]
    assert isinstance(execution, list)
    assert "execution_mode_and_batch_episode_disposition" in execution
    assert "batch_action_trace_disposition" in execution
    mission_binding = payload["mission_binding_contract_required"]
    assert isinstance(mission_binding, dict)
    assert mission_binding["supported_kinds"] == ["legacy_racetrack", "semantic_composition"]
    semantic_binding = mission_binding["semantic_composition"]
    assert isinstance(semantic_binding, dict)
    assert any("composition_family_id" in item for item in semantic_binding["required_fields"])
    assert any("expected_preflight_status" in item for item in semantic_binding["required_per_realization"])
    assert payload["intake_validation"]["status"] == "pass"
    assert validate_integration_intake_payload(payload) == ()
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
    interface = payload["interface_contract_required"]
    assert isinstance(interface, dict)
    execution = interface["execution_and_evaluation_contract"]
    assert isinstance(execution, list)
    assert "committed_truth_boundary_and_sensor_cadence_policy" in execution
    assert "execution_mode_and_batch_episode_disposition" in execution
    execution_mode_contract = interface["execution_mode_contract"]
    assert isinstance(execution_mode_contract, dict)
    assert execution_mode_contract["status"] == "required_before_execution_binding"
    source_manifest = payload["source_manifest_template"]
    assert isinstance(source_manifest, dict)
    assert source_manifest["status"] == "authoring_required"
    assert source_manifest["authoring_values"] is None
    decision = payload["new_family_decision_record"]
    assert isinstance(decision, dict)
    assert decision["status"] == "decision_required"
    assert "why no existing family strategy is suitable" in decision["required_decisions"]
    witnesses = payload["synthetic_conformance_witness_plan"]
    assert isinstance(witnesses, list)
    assert len(witnesses) == 4
    assert witnesses[-1]["control_realization"] == "surface_allocated"
    claim_boundary = payload["claim_boundary"]
    assert isinstance(claim_boundary, str)
    assert "not a vehicle model" in claim_boundary
    assert payload["intake_validation"]["status"] == "pass"
    assert validate_integration_intake_payload(payload) == ()
    ####


def test_intake_validator_rejects_missing_canonical_witness_tier() -> None:
    payload = build_new_topology_intake_scaffold(
        NewTopologyIntakeRequest(
            family_id="test_invalid_witness_plan",
            physical_family="tandem_rotor_vtol",
            proposed_strategy_id="tandem_rotor_vtol.v1",
            topology_summary="Two lift rotors with differential collective and longitudinal cyclic authority.",
        )
    ).as_dict()
    witnesses = payload["synthetic_conformance_witness_plan"]
    assert isinstance(witnesses, list)
    witnesses.pop()

    errors = validate_integration_intake_payload(payload)

    assert "synthetic_conformance_witness_plan must cover canonical tiers in canonical order" in errors
    ####


def test_intake_validator_rejects_missing_execution_provenance_contract() -> None:
    payload = build_new_topology_intake_scaffold(
        NewTopologyIntakeRequest(
            family_id="test_missing_execution_contract",
            physical_family="tandem_rotor_vtol",
            proposed_strategy_id="tandem_rotor_vtol.v1",
            topology_summary="Two lift rotors with declared transition authority.",
        )
    ).as_dict()
    interface = payload["interface_contract_required"]
    assert isinstance(interface, dict)
    execution = interface["execution_and_evaluation_contract"]
    assert isinstance(execution, list)
    execution.remove("execution_mode_and_batch_episode_disposition")

    errors = validate_integration_intake_payload(payload)

    assert any("execution_mode_and_batch_episode_disposition" in error for error in errors)
    ####


def test_intake_validator_rejects_a_semantic_mission_binding_contract_without_identity_mapping() -> None:
    payload = build_new_topology_intake_scaffold(
        NewTopologyIntakeRequest(
            family_id="test_missing_composition_identity",
            physical_family="tandem_rotor_vtol",
            proposed_strategy_id="tandem_rotor_vtol.v1",
            topology_summary="Two lift rotors with declared transition authority.",
        )
    ).as_dict()
    binding = payload["mission_binding_contract_required"]
    assert isinstance(binding, dict)
    semantic = binding["semantic_composition"]
    assert isinstance(semantic, dict)
    fields = semantic["required_fields"]
    assert isinstance(fields, list)
    semantic["required_fields"] = [item for item in fields if "composition_family_id" not in item]

    errors = validate_integration_intake_payload(payload)

    assert any("must require composition_family_id" in error for error in errors)
    ####


def test_intake_validator_rejects_a_noncanonical_execution_mode_vocabulary() -> None:
    payload = build_new_topology_intake_scaffold(
        NewTopologyIntakeRequest(
            family_id="test_noncanonical_execution_mode",
            physical_family="tandem_rotor_vtol",
            proposed_strategy_id="tandem_rotor_vtol.v1",
            topology_summary="Two lift rotors with declared transition authority.",
        )
    ).as_dict()
    interface = payload["interface_contract_required"]
    assert isinstance(interface, dict)
    contract = interface["execution_mode_contract"]
    assert isinstance(contract, dict)
    contract["allowed_values"] = ["closed_loop_controller", "invented_mode"]

    errors = validate_integration_intake_payload(payload)

    assert any("canonical execution-mode vocabulary" in error for error in errors)
    ####


def test_product_three_vehicle_intake_cli_exports_existing_family_blueprint(
    capsys: pytest.CaptureFixture[str],
) -> None:
    exit_code = main(
        [
            "vehicle",
            "intake",
            "existing-family",
            "--family-id",
            "test_cli_uav",
            "--physical-family",
            "powered_fixed_wing",
            "--mission-overlay",
            "fixed_wing_racetrack",
            "--adapter-id",
            "taoryx.fixed_wing.test_cli_uav.v1",
            "--strategy-id",
            "powered_fixed_wing.v1",
        ]
    )

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["schema"] == "taoryx.vehicle-integration-intake/v1alpha1"
    assert payload["kind"] == "existing_family"
    assert payload["family_id"] == "test_cli_uav"
    assert payload["mission_composition"]["template_id"] == "powered_fixed_wing_racetrack_v1"
    ####


def test_product_three_vehicle_intake_cli_exports_new_topology_scaffold(
    capsys: pytest.CaptureFixture[str],
) -> None:
    exit_code = main(
        [
            "vehicle",
            "intake",
            "new-topology",
            "--family-id",
            "test_cli_tailsitter",
            "--physical-family",
            "tailsitter_vtol",
            "--strategy-id",
            "tailsitter_transition_vtol.v1",
            "--topology-summary",
            "Vertical propeller takeoff followed by wing-borne transition.",
        ]
    )

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["schema"] == "taoryx.vehicle-integration-intake/v1alpha1"
    assert payload["kind"] == "new_topology"
    assert payload["status"] == "strategy_definition_required"
    assert payload["new_family_decision_record"]["status"] == "decision_required"
    ####
