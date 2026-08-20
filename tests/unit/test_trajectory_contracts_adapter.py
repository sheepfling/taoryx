"""TAORYX's adapter coverage for the standalone trajectory contracts."""

from __future__ import annotations

from taoryx_trajectory_contracts import (
    BatchRunRequest,
    CloseStreamingSessionRequest,
    DefaultConfigurationProvider,
    InspectStreamingSessionRequest,
    OpenStreamingSessionRequest,
    ResetStreamingSessionRequest,
    StreamingCompositionProvider,
    StreamingStepRequest,
    audit_batch_provider,
    audit_batch_result,
    audit_default_configuration_provider,
    audit_streaming_descriptor,
    audit_streaming_provider,
    audit_streaming_step,
)

from taoryx.trajectory.contracts_adapter import TaoryxTrajectoryContractsAdapter
from taoryx.trajectory.native_mission_composition import configuration_instance_from_vehicle_request
from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider
from taoryx.vehicle_composition import load_vehicle_composition_request
from taoryx.vehicle_execution_witnesses import load_vehicle_execution_witness_catalog
from taoryx.vehicle_registry import ROOT


def _prepared_adapter() -> tuple[TaoryxTrajectoryContractsAdapter, object]:
    """Create one public prepared configuration from an installed registry witness."""

    provider = RegistryMissionCompositionProvider()
    witness = next(item for item in load_vehicle_execution_witness_catalog().witnesses if item.operation == "episode")
    request = load_vehicle_composition_request(ROOT / witness.composition)
    configuration = configuration_instance_from_vehicle_request(provider, request)
    adapter = TaoryxTrajectoryContractsAdapter(provider)
    return adapter, adapter.prepare_configuration(adapter.wrap_configuration(configuration))
    ####


def test_registry_adapter_preserves_universal_discovery_and_ecef_batch_data() -> None:
    adapter, prepared = _prepared_adapter()
    descriptor = adapter.descriptor
    result = adapter.run_batch(BatchRunRequest(request_id="trajectory-contracts-batch", prepared_configuration=prepared))

    assert isinstance(adapter, StreamingCompositionProvider)
    assert audit_batch_provider(adapter).status == "pass"
    assert descriptor.id == "taoryx.registry.mission-composition"
    assert descriptor.models
    assert all(model.operations for model in descriptor.models)
    assert all(
        any(operation.operation == "batch" and operation.availability == "available" for operation in model.operations)
        for model in descriptor.models
    )
    assert all(
        any(operation.operation == "step" and operation.availability == "available" for operation in model.operations)
        for model in descriptor.models
    )
    assert audit_batch_result(result).status == "pass"
    assert result.configuration_fingerprint == prepared.fingerprint
    assert all(sample.standard_ecef.frame_id == "ecfc" for entity in result.entities for sample in entity.samples)
    assert all(
        sample.standard_ecef.angular_velocity_body_radps
        for entity in result.entities
        for sample in entity.samples
    )
    ####


def test_registry_adapter_provides_preparable_defaults_for_every_advertised_model() -> None:
    """Required mission choices never leak to a generic host as fake defaults."""

    adapter = TaoryxTrajectoryContractsAdapter(RegistryMissionCompositionProvider())

    assert isinstance(adapter, DefaultConfigurationProvider)
    assert audit_default_configuration_provider(adapter).status == "pass"
    for model in adapter.descriptor.models:
        configuration = adapter.build_default_configuration(model.id)
        prepared = adapter.prepare_configuration(configuration)

        assert configuration.configuration_id == model.default_configuration_id
        assert prepared.configuration == configuration
    ####


def test_registry_adapter_exercises_streaming_lifecycle_through_public_types() -> None:
    adapter, prepared = _prepared_adapter()
    opened = adapter.open_stream(
        OpenStreamingSessionRequest(
            session_id="trajectory-contracts-stream",
            prepared_configuration=prepared,
            seed=19,
        )
    )
    inspected = adapter.inspect_stream(InspectStreamingSessionRequest(session_id=opened.session_id))
    stepped = adapter.step_stream(
        StreamingStepRequest(
            session_id=opened.session_id,
            duration_s=0.02,
            expected_sequence=0,
        )
    )
    reset = adapter.reset_stream(ResetStreamingSessionRequest(session_id=opened.session_id, seed=19))
    closed = adapter.close_stream(CloseStreamingSessionRequest(session_id=opened.session_id))

    assert audit_streaming_provider(adapter).status == "pass"
    assert audit_streaming_descriptor(opened).status == "pass"
    assert audit_streaming_step(stepped).status == "pass"
    assert inspected == opened.initial_observation
    assert stepped.sequence == 1
    assert stepped.observation.standard_ecef.frame_id == "ecfc"
    assert reset.sequence == 0
    assert reset.standard_ecef.ecef_from_body_wxyz == opened.initial_observation.standard_ecef.ecef_from_body_wxyz
    assert closed.lifecycle == "closed"
    ####
