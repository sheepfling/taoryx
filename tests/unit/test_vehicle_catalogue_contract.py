"""Fast coverage contract between the vehicle catalogue and vertical slices."""

from __future__ import annotations

from pathlib import Path
from typing import cast

from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider

import tools.dev as dev
from taoryx.fidelity_contracts import FidelityTier
from taoryx.mission_workflow_endpoint import load_mission_workflow_endpoint_catalog
from taoryx.plugins import discover_plugins
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_discovery import (
    DeclaredValidityEnvelope,
    NotApplicableEffectors,
    NotRepresentedMassProperties,
    PseudoSixDofTierProfile,
    ScheduledMassProperties,
    VariantGeometry,
)
from taoryx.vehicle_endpoint_spec import load_vehicle_endpoint_spec_catalog
from taoryx.vehicle_interface import VehicleControlAuthorityAdvertisement

ROOT = Path(__file__).resolve().parents[2]


def test_every_registered_controller_campaign_has_one_runnable_vehicle_slice() -> None:
    """Every runnable vehicle, workflow, and controller campaign has a focused proof."""

    registry = discover_plugins(include_external=False).build_controller_tuning_campaign_registry()
    campaign_families = {registration.family_id for registration in registry.registrations}
    catalog = load_resolved_vehicle_composition_catalog()
    runnable_vehicle_families = {
        vehicle.family.family_id
        for vehicle in catalog.vehicles
        if any(
            vehicle.authoring_tier_assessment(mission, tier).endpoint_maturity
            in {"batch_and_episode_ready", "batch_ready", "episode_ready"}
            for mission in vehicle.declaration.mission_templates
            for tier in mission.compatible_fidelities
        )
    }
    runnable_workflow_models = {
        model.id
        for model in RegistryMissionCompositionProvider().list_models()
        if model.model_kind != "canonical_vehicle_family" and "batch" in model.common_runner_operations
    }

    assert campaign_families <= set(dev.VEHICLE_VERTICAL_TEST_PATHS)
    assert set(dev.VEHICLE_VERTICAL_TEST_PATHS) == runnable_vehicle_families | runnable_workflow_models
    for family_id, paths in dev.VEHICLE_VERTICAL_TEST_PATHS.items():
        assert paths
        assert all((ROOT / path.split("::", maxsplit=1)[0]).is_file() for path in paths), family_id
    ####


def test_every_advertised_vehicle_tier_reports_explicit_operational_maturity() -> None:
    """A declared tier must distinguish endpoint readiness from qualification status."""

    catalog = load_resolved_vehicle_composition_catalog()
    for vehicle in catalog.vehicles:
        for mission in vehicle.declaration.mission_templates:
            for tier in mission.compatible_fidelities:
                assessment = vehicle.authoring_tier_assessment(mission, tier)
                assert assessment.endpoint_maturity in {
                    "batch_and_episode_ready",
                    "batch_ready",
                    "episode_ready",
                    "declared_execution_gap",
                    "unbound",
                }
                assert assessment.operation_scope in {"mission", "local_controller_screen"}
                assert assessment.fidelity.validation_status == "pass"
    ####


def test_every_registry_model_has_the_appropriate_focused_endpoint_contract() -> None:
    """Physical families and nonphysical workflows cannot silently lose vertical proof coverage."""

    models = RegistryMissionCompositionProvider().list_models()
    physical_models = {item.id for item in models if item.model_kind == "canonical_vehicle_family"}
    nonphysical_runnable_models = {
        item.id
        for item in models
        if item.model_kind in {"trajectory_workflow", "composition_proof_family"}
        and "batch" in item.common_runner_operations
    }
    physical_endpoint_models = {item.model_id for item in load_vehicle_endpoint_spec_catalog().endpoints}
    workflow_endpoint_models = {item.model_id for item in load_mission_workflow_endpoint_catalog().endpoints}

    assert physical_endpoint_models == physical_models
    assert workflow_endpoint_models == nonphysical_runnable_models
    assert physical_endpoint_models.isdisjoint(workflow_endpoint_models)
    ####


def test_every_vehicle_has_complete_discovery_metadata_and_explicit_characteristic_gaps() -> None:
    """The catalogue must be useful without turning missing plant facts into guesses."""

    catalog = load_resolved_vehicle_composition_catalog()
    summary = catalog.as_dict(detail="summary")
    summary_records = cast(list[dict[str, object]], summary["vehicles"])
    assert len(summary_records) == len(catalog.vehicles)
    for summary_record, vehicle in zip(summary_records, catalog.vehicles, strict=True):
        metadata = vehicle.declaration.metadata
        characteristics = vehicle.physical_characteristics()
        serialized_characteristics = characteristics.public_dict()
        assert metadata.summary
        assert metadata.vehicle_class
        assert metadata.operating_domains
        assert metadata.propulsion
        assert metadata.roles
        assert metadata.claim_boundary
        assert summary_record["summary"] == metadata.summary
        assert summary_record["vehicle_class"] == metadata.vehicle_class
        assert serialized_characteristics["status"] in {
            "source_owned_values_available",
            "not_represented_in_common_catalog",
        }
        coverage = characteristics.availability_by_field()
        available = {identifier for identifier, status in coverage.items() if status != "not_represented"}
        unavailable = {identifier for identifier, status in coverage.items() if status == "not_represented"}
        assert available.isdisjoint(unavailable)
        assert available | unavailable == {
            "reference_geometry",
            "mass_properties",
            "validity_envelope",
            "effectors",
        }
        assert characteristics.claim_boundary
        if isinstance(characteristics.validity_envelope, DeclaredValidityEnvelope):
            assert "not demonstrated maximum vehicle performance" in characteristics.claim_boundary
    ####


def test_every_vehicle_tier_advertises_model_identity_and_control_authority() -> None:
    """Lower and rigid-body tiers expose comparable names, limits, and authority."""

    catalog = load_resolved_vehicle_composition_catalog()
    for vehicle in catalog.vehicles:
        descriptor = vehicle.as_dict()
        fidelities = cast(dict[str, dict[str, object]], descriptor["fidelities"])
        interfaces = cast(dict[str, dict[str, object]], descriptor["interfaces"])
        assert set(fidelities) == {
            "point_mass_3dof",
            "pseudo_6dof",
            "rigid_body_6dof_direct_wrench",
            "rigid_body_6dof_surface_allocated",
        }
        for tier, record in fidelities.items():
            tier_metadata = cast(dict[str, object], record["tier_metadata"])
            profile = cast(dict[str, object], record["model_profile"])
            authority = cast(dict[str, object], record["control_authority"])
            advertisement = vehicle.fidelity_advertisement(cast(FidelityTier, tier))
            typed_authority = advertisement.interface.authority_advertisement()
            assert tier_metadata["display_name"]
            assert tier_metadata["model_kind"]
            assert tier_metadata["control_boundary"]
            assert profile == advertisement.model_profile.model_dump(mode="json")
            assert profile["profile_id"] == advertisement.binding.profile_id
            assert authority["interface_id"] == interfaces[tier]["interface_id"]
            assert authority["details_path"] == f"interfaces.{tier}.control_authority"
            assert isinstance(typed_authority, VehicleControlAuthorityAdvertisement)
            assert typed_authority.summary_dict(details_path=f"interfaces.{tier}.control_authority") == authority
            authority_detail = cast(dict[str, object], interfaces[tier]["control_authority"])
            assert authority_detail["interface_id"] == typed_authority.interface_id
        pseudo_profile = vehicle.tier_model_profile("pseudo_6dof")
        assert isinstance(pseudo_profile, PseudoSixDofTierProfile)
        if pseudo_profile.model_kind != "rigid_body_reuse":
            assert {limit.axis for limit in pseudo_profile.default_response} == {"roll", "pitch", "yaw"}
    ####


def test_vehicle_physical_characteristics_use_explicit_source_specific_alternatives() -> None:
    """Known source gaps and variable forms are typed rather than null or ad hoc keys."""

    catalog = load_resolved_vehicle_composition_catalog()

    f16 = catalog.vehicle("f16_s119").physical_characteristics()
    nesc = catalog.vehicle("reference_nesc_two_stage_rocket").physical_characteristics()
    tumbling = catalog.vehicle("tumbling_body").physical_characteristics()

    assert isinstance(f16.mass_properties, NotRepresentedMassProperties)
    assert isinstance(nesc.mass_properties, ScheduledMassProperties)
    assert nesc.mass_properties.includes_staging_events
    assert isinstance(tumbling.reference_geometry, VariantGeometry)
    assert tumbling.reference_geometry.selector_parameter_id == "body_shape"
    assert isinstance(tumbling.effectors, NotApplicableEffectors)
    assert tumbling.availability_by_field()["effectors"] == "not_applicable"
    ####


def test_authoring_kit_serializes_the_typed_assessment_without_a_descriptor_round_trip() -> None:
    """The kit and worklist expose one stable assessment contract at their API boundary."""

    vehicle = load_resolved_vehicle_composition_catalog().vehicle("x15")
    mission = next(
        item
        for item in vehicle.declaration.mission_templates
        if item.id == "x15_local_direct_wrench_screen_v1"
    )
    assessment = vehicle.authoring_tier_assessment(mission, "rigid_body_6dof_direct_wrench")
    kit = vehicle.authoring_kit_dict(mission.id, assessment.fidelity.tier)

    selection = cast(dict[str, object], kit["selection"])
    assert selection["control_realization"] == assessment.fidelity.control_realization
    assert selection["runtime_fidelity"] == assessment.fidelity.runtime_fidelity
    assert selection["interface_validation"] == assessment.fidelity.validation_status
    assert kit["runtime_and_evidence_worklist"] == assessment.as_dict()
    ####


def test_every_composition_input_and_segment_has_common_semantics() -> None:
    """Current families cannot fall back to opaque argument or segment bags."""

    catalog = load_resolved_vehicle_composition_catalog()
    for vehicle in catalog.vehicles:
        descriptor = vehicle.as_dict()
        planning = cast(dict[str, object], descriptor["segment_planning"])
        configuration = cast(dict[str, object], descriptor["configuration_surface"])
        assert planning["status"] == "complete"
        assert planning["missing_categories"] == []
        assert planning["unreferenced_segment_ids"] == []
        assert configuration["semantic_role_counts"]
        parameters = cast(list[dict[str, object]], vehicle.parameter_dict()["parameters"])
        assert all(parameter["semantic_role"] != "custom" for parameter in parameters)
        segments = cast(list[dict[str, object]], descriptor["segment_contracts"])
        for segment in segments:
            taxonomy = cast(dict[str, object], segment["taxonomy"])
            assert taxonomy["category"]
            assert taxonomy["execution_style"]
            assert taxonomy["lifecycle_phase"]
            assert taxonomy["description"]
    ####
