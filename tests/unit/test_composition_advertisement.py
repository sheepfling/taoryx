"""Closed-contract tests for the normalized composition advertisement."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from taoryx import trajectory as trajectory_api
from taoryx.plugins import discover_plugins
from taoryx.trajectory.configuration_contract import (
    COMPOSITION_FEATURE_VOCABULARY,
    TrajectoryModelMetadata,
)
from taoryx.trajectory.execution_contract import audit_provider_advertisement
from taoryx.trajectory.mission_composition import TrajectoryCompositionAdvertisement
from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider


def _expected_feature_keys() -> tuple[tuple[str, str], ...]:
    return tuple((category, identifier) for category, identifiers in COMPOSITION_FEATURE_VOCABULARY for identifier in identifiers)
    ####


def test_composition_advertisement_contract_is_on_the_public_trajectory_surface() -> None:
    assert trajectory_api.TrajectoryCompositionAdvertisement is TrajectoryCompositionAdvertisement
    assert trajectory_api.COMPOSITION_FEATURE_VOCABULARY is COMPOSITION_FEATURE_VOCABULARY
    ####


def test_composition_advertisement_exports_a_closed_standalone_json_schema() -> None:
    schema = TrajectoryCompositionAdvertisement.model_json_schema(by_alias=True)

    assert schema["additionalProperties"] is False
    assert schema["properties"]["schema"]["const"] == "taoryx.trajectory-composition-advertisement/v1"
    feature = schema["$defs"]["TrajectoryCompositionFeatureMetadata"]
    assert feature["additionalProperties"] is False
    assert feature["properties"]["category"]["enum"] == [item[0] for item in COMPOSITION_FEATURE_VOCABULARY]
    ####


def test_every_registry_model_publishes_one_exhaustive_composition_partition() -> None:
    provider = RegistryMissionCompositionProvider()

    for model in provider.list_models():
        advertisement = model.composition_advertisement
        assert isinstance(advertisement, TrajectoryCompositionAdvertisement)
        assert tuple((item.category, item.id) for item in advertisement.features) == _expected_feature_keys()
        assert TrajectoryCompositionAdvertisement.model_validate_json(advertisement.model_dump_json(by_alias=True)) == advertisement
        assert TrajectoryModelMetadata.model_validate_json(model.model_dump_json(by_alias=True)) == model
    ####


def test_every_installed_provider_model_uses_the_same_additive_safe_contract() -> None:
    providers = discover_plugins().build_mission_composition_provider_registry()

    for provider in providers.providers:
        for model in provider.list_models():
            advertisement = model.composition_advertisement
            assert tuple((item.category, item.id) for item in advertisement.features) == _expected_feature_keys()
            round_tripped = TrajectoryModelMetadata.model_validate_json(model.model_dump_json(by_alias=True))
            assert round_tripped.model_dump(mode="json", by_alias=True) == model.model_dump(mode="json", by_alias=True)
            if model.output_schema.entity_output.supports_multiple_entities:
                multi_entity = advertisement.feature("execution_mode", "multi_entity")
                assert multi_entity.status == "conditional"
                assert multi_entity.operations
                if not model.output_schema.entity_output.supports_dynamic_spawning:
                    assert advertisement.feature("entity_topology", "spawn").status == "not_available"
    ####


def test_composition_advertisement_distinguishes_sequence_edits_from_graph_mutation() -> None:
    model = RegistryMissionCompositionProvider().model("simple_aero")
    advertisement = model.composition_advertisement

    assert advertisement.feature("authoring_mode", "caller_ordered_sequence").status == "conditional"
    assert advertisement.feature("rearrangement", "reorder").status == "conditional"
    assert advertisement.feature("rearrangement", "parameter_patch").status == "available"
    assert advertisement.feature("authoring_mode", "caller_authored_graph").status == "not_available"
    assert advertisement.feature("graph_form", "conditional_branching").status == "not_available"
    assert advertisement.feature("rearrangement", "rewire").status == "not_available"
    assert advertisement.feature("runtime_transition", "graph_patch").status == "not_available"
    ####


def test_composition_advertisement_exposes_exact_runtime_and_entity_boundaries() -> None:
    provider = RegistryMissionCompositionProvider()
    x8 = provider.model("skywalker_x8").composition_advertisement
    nesc = provider.model("reference_nesc_two_stage_rocket").composition_advertisement

    assert x8.feature("execution_mode", "stateful_session").operations == ("step",)
    assert x8.feature("runtime_transition", "fidelity_switch").status == "blocked"
    assert x8.feature("state_transfer", "fidelity_projection").status == "blocked"

    spawn = nesc.feature("entity_topology", "spawn")
    assert spawn.status == "conditional"
    assert spawn.operations == ("batch",)
    assert spawn.mutation_timing == ("accepted_boundary",)
    assert nesc.feature("entity_topology", "split").status == "conditional"
    assert nesc.feature("entity_topology", "merge").status == "not_available"
    assert nesc.feature("entity_topology", "recursive_spawn").status == "not_available"
    ####


def test_serialized_model_cannot_overclaim_a_derived_composition_feature() -> None:
    model = RegistryMissionCompositionProvider().model("skywalker_x8")
    payload = model.model_dump(mode="json", by_alias=True)
    features = payload["composition_advertisement"]["features"]
    graph = next(item for item in features if item["category"] == "graph_form" and item["id"] == "conditional_branching")
    graph.update(
        {
            "status": "available",
            "operations": ["validate"],
            "claim_boundary": "Forged branching claim.",
        }
    )

    with pytest.raises(ValidationError, match="composition advertisement is stale or overclaims"):
        TrajectoryModelMetadata.model_validate(payload)
    ####


def test_provider_audit_counts_the_complete_composition_partition() -> None:
    provider = RegistryMissionCompositionProvider()
    report = audit_provider_advertisement(provider, provider.build_runner())

    assert report.status == "pass"
    for model in report.models:
        assert model.composition_schema_id == "taoryx.trajectory-composition-advertisement/v1"
        assert model.composition_feature_count == len(_expected_feature_keys())
        assert (
            model.available_composition_feature_count
            + model.conditional_composition_feature_count
            + model.declared_composition_feature_count
            + model.blocked_composition_feature_count
            + model.unavailable_composition_feature_count
            == model.composition_feature_count
        )
    ####


def test_provider_audit_accepts_an_explicit_uncontrolled_zero_action_step() -> None:
    """An open-loop session is interactive in time, not necessarily in controls."""

    catalog = discover_plugins(include_external=False, selected=("taoryx.debug-models",))
    provider = catalog.build_mission_composition_provider_registry().provider("taoryx.reference.mission-composition")

    report = audit_provider_advertisement(provider, provider.build_runner())

    assert report.status == "pass", report.diagnostics
    ballistic = next(item for item in report.models if item.model_id == "reference_ballistic_3dof")
    assert ballistic.action_control_channel_count == 0
    assert ballistic.common_runner_operations == ("batch", "step")
    ####
