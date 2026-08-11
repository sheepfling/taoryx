"""Focused tests for the common plug-in execution envelope."""

from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.vehicle_batch_execution import VehicleBatchExecutionRequest, _invoke_batch_factory, batch_factory_request_v1
from taoryx.vehicle_composition import (
    compile_vehicle_composition,
    load_vehicle_composition_request,
    resolve_vehicle_composition_interface_contract,
)
from taoryx.vehicle_execution_artifact import (
    VEHICLE_EXECUTION_PACKET_SCHEMA,
    VehicleExecutionArtifact,
    VehicleExecutionPacket,
    VehicleExecutionRequestRecord,
    read_vehicle_execution_packet,
)
from taoryx.vehicle_execution_bindings import resolve_vehicle_execution_binding

ROOT = Path(__file__).resolve().parents[2]


def _composition():
    request = load_vehicle_composition_request(
        ROOT / "examples/vehicle_composition/x15_local_direct_wrench_screen_compose.yaml"
    )
    return compile_vehicle_composition(request)


def test_execution_artifact_binds_provider_payload_to_exact_composition() -> None:
    composition = _composition()
    payload = {
        "schema": "example.execution/v1",
        "status": "development_local_screen_pass",
        "composition": composition.model_dump(mode="json", by_alias=True),
        "screen_pass": True,
        "claim_boundary": "bounded controller screen only",
        "runtime": {"provider_field": "preserved"},
    }

    artifact = VehicleExecutionArtifact.from_payload(payload, composition=composition)

    assert artifact.passed is True
    assert artifact.schema_id == "example.execution/v1"
    assert artifact.runtime == {"provider_field": "preserved"}
    assert artifact.as_dict()["screen_pass"] is True


def test_execution_artifact_rejects_missing_common_disposition() -> None:
    composition = _composition()
    payload = {
        "schema": "example.execution/v1",
        "status": "incomplete",
        "composition": composition.model_dump(mode="json", by_alias=True),
        "claim_boundary": "bounded controller screen only",
    }

    with pytest.raises(ValueError, match="mission_pass or screen_pass"):
        VehicleExecutionArtifact.from_payload(payload, composition=composition)


def test_execution_artifact_rejects_stale_composition_identity() -> None:
    composition = _composition()
    payload = {
        "schema": "example.execution/v1",
        "status": "development_local_screen_pass",
        "composition": {"id": "stale", "identity_sha256": "0" * 64},
        "screen_pass": True,
        "claim_boundary": "bounded controller screen only",
    }

    with pytest.raises(ValueError, match="composition"):
        VehicleExecutionArtifact.from_payload(payload, composition=composition)


def test_typed_batch_factory_receives_the_complete_execution_request(tmp_path: Path) -> None:
    composition = _composition()
    request = VehicleBatchExecutionRequest(
        composition=composition,
        binding=resolve_vehicle_execution_binding(composition, "batch"),
        output_dir=tmp_path / "typed-factory",
        max_steps=12,
    )
    observed: list[VehicleBatchExecutionRequest] = []

    class FactoryExecution:
        @property
        def output_dir(self) -> Path:
            return request.output_dir

        def as_dict(self) -> dict[str, object]:
            return {}

    @batch_factory_request_v1
    def typed_factory(value: VehicleBatchExecutionRequest) -> FactoryExecution:
        observed.append(value)
        return FactoryExecution()

    assert _invoke_batch_factory(typed_factory, request) is not None
    assert observed == [request]
    assert request.as_dict()["factory_id"] == "local_direct_wrench_screen.v1"
    assert request.as_dict()["max_steps"] == 12


def test_execution_request_records_a_sealed_tuning_application_identity(tmp_path: Path) -> None:
    """Persisted packets retain candidate identity without serializing gains."""

    composition = _composition()
    request = VehicleBatchExecutionRequest(
        composition=composition,
        binding=resolve_vehicle_execution_binding(composition, "batch"),
        output_dir=tmp_path / "tuned-factory",
    )
    payload = request.as_dict()
    payload["tuning_application_context"] = {
        "campaign_id": "x15-direct-wrench-lqr",
        "node_id": "screen",
        "candidate_profile_id": "lqr-001",
        "candidate_configuration_fingerprint_sha256": "a" * 64,
        "resolved_gain_fingerprint_sha256": "b" * 64,
    }

    recorded = VehicleExecutionRequestRecord.model_validate(payload)

    assert recorded.tuning_application_context is not None
    assert recorded.tuning_application_context.campaign_id == "x15-direct-wrench-lqr"
    assert recorded.tuning_application_context.resolved_gain_fingerprint_sha256 == "b" * 64

    malformed = dict(payload["tuning_application_context"])
    malformed["gain_matrix"] = [[1.0]]
    payload["tuning_application_context"] = malformed
    with pytest.raises(ValueError, match="gain_matrix"):
        VehicleExecutionRequestRecord.model_validate(payload)


def test_execution_request_records_a_complete_schedule_tuning_selection(tmp_path: Path) -> None:
    """A persisted schedule request cannot omit or duplicate a selected node."""

    composition = _composition()
    request = VehicleBatchExecutionRequest(
        composition=composition,
        binding=resolve_vehicle_execution_binding(composition, "batch"),
        output_dir=tmp_path / "scheduled-tuned-factory",
    )
    payload = request.as_dict()
    payload["tuning_application_context_set"] = {
        "campaign_id": "f16-schedule-lqi",
        "controller_method": "lqi",
        "node_ids": ["sea-level", "high-altitude"],
        "contexts": [
            {
                "campaign_id": "f16-schedule-lqi",
                "node_id": "sea-level",
                "candidate_profile_id": "balanced-sea-level",
                "candidate_configuration_fingerprint_sha256": "a" * 64,
                "resolved_gain_fingerprint_sha256": "b" * 64,
            },
            {
                "campaign_id": "f16-schedule-lqi",
                "node_id": "high-altitude",
                "candidate_profile_id": "balanced-high-altitude",
                "candidate_configuration_fingerprint_sha256": "c" * 64,
                "resolved_gain_fingerprint_sha256": "d" * 64,
            },
        ],
    }

    recorded = VehicleExecutionRequestRecord.model_validate(payload)

    assert recorded.tuning_application_context_set is not None
    assert recorded.tuning_application_context_set.node_ids == ("sea-level", "high-altitude")

    payload["tuning_application_context_set"]["node_ids"] = ["sea-level", "sea-level"]
    with pytest.raises(ValueError, match="node IDs"):
        VehicleExecutionRequestRecord.model_validate(payload)


def test_host_execution_packet_preserves_provider_extensions_and_binds_request(tmp_path: Path) -> None:
    composition = _composition()
    request = VehicleBatchExecutionRequest(
        composition=composition,
        binding=resolve_vehicle_execution_binding(composition, "batch"),
        output_dir=tmp_path / "packet",
    )
    provider = VehicleExecutionArtifact.from_payload(
        {
            "schema": "example.execution/v1",
            "status": "development_local_screen_pass",
            "composition": composition.model_dump(mode="json", by_alias=True),
            "screen_pass": True,
            "claim_boundary": "bounded controller screen only",
            "runtime": {"provider_field": "preserved"},
        },
        composition=composition,
    )
    interface = resolve_vehicle_composition_interface_contract(composition)

    packet = VehicleExecutionPacket.from_artifact(
        provider,
        request=request.as_dict(),
        interface_id=interface.id,
        interface_fingerprint_sha256=interface.fingerprint,
    )
    path = packet.write_json(request.output_dir / "execution.json")
    restored = read_vehicle_execution_packet(path)

    assert restored.schema_id == VEHICLE_EXECUTION_PACKET_SCHEMA
    assert restored.outcome.scope == "local_screen"
    assert restored.outcome.passed is True
    assert restored.as_dict()["runtime"] == {"provider_field": "preserved"}
    assert restored.provider_execution_schema == "example.execution/v1"
    assert restored.host_execution.request.factory_id == request.binding.factory_id
