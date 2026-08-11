from __future__ import annotations

import json

import pytest
from pydantic import ValidationError
from taoryx.mission_composition_completion import (
    MissionCompositionCompletionReport,
    build_mission_composition_completion_report,
    render_mission_composition_completion_markdown,
)
from taoryx.trajectory.native_mission_composition import build_registry_mission_composition_runner
from taoryx.trajectory.reference_mission_composition import ReferenceMissionCompositionProvider
from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider

from taoryx.mission_composition_inventory import (
    MissionCompositionInventory,
    MissionCompositionInventoryAudit,
    audit_mission_composition_inventory,
    load_mission_composition_inventory,
)
from taoryx.trajectory.configuration_contract import ConfigurationContractError
from taoryx.trajectory.contract_probe_mission_composition import (
    ContractProbeMissionCompositionProvider,
    build_contract_probe_configuration,
)
from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    ProviderAdvertisementConformanceReport,
    audit_provider_advertisement,
    diagnostic_from_exception,
)
from taoryx.vehicle_registry import ROOT


def test_authoritative_inventory_reconciles_production_debug_and_realization_discovery() -> None:
    inventory = load_mission_composition_inventory()
    provider = RegistryMissionCompositionProvider()
    audit = audit_mission_composition_inventory(
        inventory,
        provider=provider,
        debug_providers=(ReferenceMissionCompositionProvider(), ContractProbeMissionCompositionProvider()),
    )

    assert audit.status == "pass", audit.errors
    assert audit.asset_count == 46
    assert audit.source_identity_count == 51
    assert audit.published_model_count == 11
    assert audit.debug_model_count == 3
    assert MissionCompositionInventory.model_validate_json(inventory.model_dump_json(by_alias=True)) == inventory
    assert MissionCompositionInventoryAudit.model_validate_json(audit.model_dump_json(by_alias=True)) == audit

    production_ids = {item.id for item in provider.list_models()}
    debug_ids = {
        item.id
        for debug_provider in (ReferenceMissionCompositionProvider(), ContractProbeMissionCompositionProvider())
        for item in debug_provider.list_models()
    }
    assert production_ids.isdisjoint(debug_ids)
    assert {"simple_aero", "dual_launch_glider", "a320_openap_3dof", "tumbling_body"} <= production_ids
    ####


def test_production_advertisement_and_generated_completion_artifacts_are_current() -> None:
    provider = RegistryMissionCompositionProvider()
    runner = build_registry_mission_composition_runner(provider)
    audit = audit_provider_advertisement(provider, runner)
    report = build_mission_composition_completion_report(provider)

    assert audit.status == "pass", audit.diagnostics
    assert audit.runner_checked
    assert all(item.status == "pass" for item in audit.models)
    assert sum(item.control_channel_count for item in audit.models) == 133
    assert sum(item.active_control_channel_count for item in audit.models) == 88
    assert sum(item.native_bound_control_channel_count for item in audit.models) == 121
    assert sum(item.control_authority_count for item in audit.models) == 57
    assert sum(item.control_intent_count for item in audit.models) == 155
    assert ProviderAdvertisementConformanceReport.model_validate_json(audit.model_dump_json(by_alias=True)) == audit

    assert report.status == "pass", report.diagnostics
    assert report.inventory_status == "pass"
    assert report.advertisement_status == "pass"
    assert report.family_count == 11
    assert report.realization_count == 45
    assert report.registered_batch_tuple_count == 66
    assert report.registered_interactive_tuple_count == 12
    assert MissionCompositionCompletionReport.model_validate_json(report.model_dump_json(by_alias=True)) == report

    expected_json = report.model_dump_json(indent=2, by_alias=True) + "\n"
    expected_markdown = render_mission_composition_completion_markdown(report)
    assert (ROOT / "verification/mission_composition_completion.json").read_text(encoding="utf-8") == expected_json
    assert (ROOT / "docs/architecture/mission-composition-coverage-matrix.md").read_text(encoding="utf-8") == expected_markdown
    ####


def test_model_level_readiness_is_only_an_index_over_exact_registered_tuples() -> None:
    provider = RegistryMissionCompositionProvider()
    assert provider.metadata.session_contract == "taoryx.mission-composition-session/v1"
    for model in provider.list_models():
        exact = {
            operation.operation
            for mission in model.mission_templates
            for operation in mission.operations
            if operation.operation in {"batch", "step"} and operation.status == "available" and operation.common_runner_status == "registered"
        }
        assert set(model.common_runner_operations) == exact
        assert (model.status == "common_runner_ready") == bool(exact)

        for realization in model.realizations:
            if realization.status == "available":
                assert realization.operations
            if realization.status != "available":
                assert not {"batch", "step"} & set(realization.operations)

    source_refs = {source for model in provider.list_models() for realization in model.realizations for source in realization.source_refs}
    assert "verification/horizontal_fidelity_registry.yaml" in source_refs
    assert "verification/horizontal_fidelity.yaml" not in source_refs
    ####


def test_contract_probe_round_trips_typed_selected_outputs_and_child_lineage() -> None:
    provider = ContractProbeMissionCompositionProvider()
    prepared = provider.validate_configuration(build_contract_probe_configuration(provider))
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="typed-contract-probe",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all"),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    result = response.result
    assert len(result.objects) == 3
    assert len(result.relationships) == 2
    entity_output = provider.get_model_output_schema(prepared.configuration.model_id).entity_output
    assert entity_output.supports_recursive_spawning
    assert entity_output.maximum_descendant_depth == 2
    primary = next(item for item in result.objects if item.object_id == result.primary_object_id)
    metadata = {item.id: item for item in primary.channels}
    assert {item.data_type for item in metadata.values()} >= {"float64", "int64", "boolean", "string", "json"}
    assert any(item.shape for item in metadata.values())
    assert any(item.sampling_semantics == "event" for item in metadata.values())
    child = next(item for item in result.objects if item.parent_object_id == result.primary_object_id)
    relationship = next(item for item in result.relationships if item.child_object_id == child.object_id)
    assert child.realization_id
    assert child.parent_object_id == relationship.parent_object_id
    assert child.samples[0].values == relationship.initial_state.values
    descendant = next(item for item in result.objects if item.parent_object_id == child.object_id)
    descendant_relationship = next(item for item in result.relationships if item.child_object_id == descendant.object_id)
    assert descendant.samples[0].values == descendant_relationship.initial_state.values

    serialized = json.loads(response.model_dump_json(by_alias=True))
    assert (
        provider.build_runner()
        .run(
            MissionCompositionRunRequest.model_validate(
                {
                    "request_id": "typed-contract-probe-replay",
                    "provider_id": provider.metadata.id,
                    "provider_version": provider.metadata.version,
                    "prepared_configuration": prepared.model_dump(mode="json", by_alias=True),
                    "output": {"mode": "core"},
                }
            )
        )
        .kind
        == "trajectory"
    )
    assert serialized["kind"] == "trajectory"

    bounded = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="typed-contract-probe-bounded-lineage",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core", maximum_objects=2),
        )
    )
    assert bounded.kind == "trajectory"
    assert len(bounded.result.objects) == 2
    assert len(bounded.result.relationships) == 1

    root_only = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="typed-contract-probe-root-only",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core", include_spawned_objects=False),
        )
    )
    assert root_only.kind == "trajectory"
    assert len(root_only.result.objects) == 1
    assert root_only.result.relationships == ()
    ####


def test_stateless_run_request_cannot_claim_interactive_step_semantics() -> None:
    provider = ContractProbeMissionCompositionProvider()
    prepared = provider.validate_configuration(build_contract_probe_configuration(provider))

    with pytest.raises(ValidationError, match="batch"):
        MissionCompositionRunRequest.model_validate(
            {
                "request_id": "invalid-stateless-step",
                "provider_id": provider.metadata.id,
                "provider_version": provider.metadata.version,
                "operation": "step",
                "prepared_configuration": prepared.model_dump(mode="json", by_alias=True),
            }
        )
    ####


def test_discovery_configuration_preflight_execution_and_projection_diagnostics_are_stable() -> None:
    provider = ContractProbeMissionCompositionProvider()
    prepared = provider.validate_configuration(build_contract_probe_configuration(provider))
    request = MissionCompositionRunRequest(
        request_id="diagnostic-phases",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="core"),
    )

    missing = MissionCompositionRunnerRegistry().run(request)
    assert missing.kind == "failure"
    assert missing.failure.phase == "preflight"
    assert missing.failure.diagnostics[0].code == "executor-not-registered"

    successful = provider.build_runner().run(request)
    assert successful.kind == "trajectory"
    mismatched_runner = MissionCompositionRunnerRegistry(
        {(provider.metadata.id, request.model_id): lambda _: successful.result.model_copy(update={"request_id": "another-request"})}
    )
    mismatched = mismatched_runner.run(request)
    assert mismatched.kind == "failure"
    assert mismatched.failure.phase == "projection"
    assert mismatched.failure.diagnostics[0].code == "provider-result-identity-mismatch"

    configuration, category = diagnostic_from_exception(
        ConfigurationContractError("unit-mismatch", "expected metres", path="configuration.altitude"),
        phase="execution",
        provider_id=provider.metadata.id,
        model_id=request.model_id,
    )
    assert configuration.phase == "configuration"
    assert category == "invalid_request"

    execution, category = diagnostic_from_exception(
        RuntimeError("private provider detail"),
        phase="execution",
        provider_id=provider.metadata.id,
        model_id=request.model_id,
    )
    assert execution.phase == "execution"
    assert execution.message == "The provider failed without a structured public diagnostic."
    assert category == "provider_error"
    ####
