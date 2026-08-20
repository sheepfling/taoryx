"""Standalone trajectory-contract package boundaries and protocol conformance."""

from __future__ import annotations

import ast
import tomllib
from pathlib import Path

from taoryx_trajectory_contracts import (
    BatchCompositionProvider,
    BatchRunRequest,
    BatchRunResult,
    CompositionConfiguration,
    CompositionOperationDescriptor,
    DefaultConfigurationProvider,
    PreparedCompositionConfiguration,
    StandardEcefState,
    StreamingCompositionProvider,
    TrajectoryEntity,
    TrajectoryModelDescriptor,
    TrajectoryProviderDescriptor,
    TrajectorySample,
    audit_batch_provider,
    audit_batch_result,
    audit_default_configuration_provider,
    canonical_json_sha256,
)

ROOT = Path(__file__).resolve().parents[2]
CONTRACT_PROJECT = ROOT / "packages" / "taoryx-trajectory-contracts"


def _state() -> StandardEcefState:
    return StandardEcefState(
        position_ecef_m=(6_378_137.0, 0.0, 0.0),
        velocity_ecef_mps=(0.0, 150.0, 0.0),
        acceleration_ecef_mps2=(0.0, 0.0, 0.0),
        angular_velocity_body_radps=(0.0, 0.0, 0.0),
        angular_velocity_kind="native_body_rate",
        ecef_from_body_wxyz=(1.0, 0.0, 0.0, 0.0),
        position_projection="native_ecfc",
        source_frame="ecfc",
        orientation_kind="native_ecef_from_body",
    )
    ####


def _configuration() -> CompositionConfiguration:
    return CompositionConfiguration(
        configuration_id="batch-only-example",
        provider_id="example.batch-only",
        provider_version="1.0.0",
        model_id="example_vehicle",
        model_version="1.0.0",
        configuration_schema_id="example.configuration/v1",
        fidelity="point_mass",
        mission_template_id="straight_line",
        payload={"speed_mps": 150.0},
    )
    ####


class _BatchOnlyProvider:
    """A non-TAORYX provider witness: batch support does not imply streaming."""

    @property
    def descriptor(self) -> TrajectoryProviderDescriptor:
        return TrajectoryProviderDescriptor(
            id="example.batch-only",
            version="1.0.0",
            name="Example Batch-only Provider",
            description="Independent batch-only conformance witness.",
            models=(
                TrajectoryModelDescriptor(
                    id="example_vehicle",
                    version="1.0.0",
                    name="Example Vehicle",
                    description="Minimal provider-owned batch model.",
                    model_kind="point_mass",
                    status="available",
                    configuration_schema_id="example.configuration/v1",
                    output_schema_id="example.output/v1",
                    operations=(
                        CompositionOperationDescriptor(
                            mission_template_id="straight_line",
                            fidelity="point_mass",
                            operation="validate",
                            availability="available",
                            claim_boundary="Structural configuration validation only.",
                        ),
                        CompositionOperationDescriptor(
                            mission_template_id="straight_line",
                            fidelity="point_mass",
                            operation="batch",
                            availability="available",
                            execution_kind="native",
                            claim_boundary="Deterministic straight-line batch witness only.",
                        ),
                    ),
                    claim_boundary="Synthetic interoperability witness only.",
                ),
            ),
            claim_boundary="Test provider only; no vehicle claim.",
        )
        ####

    def prepare_configuration(self, configuration: CompositionConfiguration) -> PreparedCompositionConfiguration:
        payload = {"native_prepared": configuration.payload}
        return PreparedCompositionConfiguration(
            configuration=configuration,
            provider_payload=payload,
            fingerprint=canonical_json_sha256(
                {
                    "configuration": configuration.model_dump(mode="json", by_alias=True),
                    "provider_payload": payload,
                }
            ),
        )
        ####

    def run_batch(self, request: BatchRunRequest) -> BatchRunResult:
        return BatchRunResult(
            provider_id=request.provider_id,
            provider_version="1.0.0",
            request_id=request.request_id,
            configuration_fingerprint=request.prepared_configuration.fingerprint,
            primary_entity_id="example:primary",
            status="completed",
            entities=(
                TrajectoryEntity(
                    id="example:primary",
                    model_id="example_vehicle",
                    realization_id="point_mass",
                    fidelity="point_mass",
                    status="completed",
                    role="primary",
                    samples=(
                        TrajectorySample(
                            time_s=0.0,
                            values={"speed_mps": 150.0},
                            standard_ecef=_state(),
                            extensions={"foreign_provider_quality": "nominal"},
                        ),
                    ),
                    claim_boundary="Synthetic batch witness only.",
                    extensions={"drag_model": "provider-specific"},
                ),
            ),
            claim_boundary="Synthetic batch witness only.",
            extensions={"provider_revision": "example-native-v1"},
        )
        ####

    ####


def test_batch_only_provider_is_a_first_class_contract_shape() -> None:
    provider = _BatchOnlyProvider()
    configuration = _configuration()
    prepared = provider.prepare_configuration(configuration)
    result = provider.run_batch(BatchRunRequest(request_id="batch-only", prepared_configuration=prepared))

    assert isinstance(provider, BatchCompositionProvider)
    assert not isinstance(provider, DefaultConfigurationProvider)
    assert not isinstance(provider, StreamingCompositionProvider)
    assert audit_batch_provider(provider).status == "pass"
    assert audit_default_configuration_provider(provider).status == "fail"
    assert audit_batch_result(result).status == "pass"
    assert provider.descriptor.model("example_vehicle").supports(
        "batch",
        mission_template_id="straight_line",
        fidelity="point_mass",
    )
    assert not provider.descriptor.model("example_vehicle").supports(
        "step",
        mission_template_id="straight_line",
        fidelity="point_mass",
    )
    assert result.entities[0].samples[0].standard_ecef.frame_id == "ecfc"
    assert result.entities[0].extensions["drag_model"] == "provider-specific"
    assert result.extensions["provider_revision"] == "example-native-v1"
    ####


def test_contract_distribution_does_not_import_the_taoryx_runtime() -> None:
    """Keep the package usable by a foreign provider or host on its own."""

    project = tomllib.loads((CONTRACT_PROJECT / "pyproject.toml").read_text(encoding="utf-8"))
    assert project["project"]["dependencies"] == ["pydantic>=2.8"]

    imports: set[str] = set()
    for path in (CONTRACT_PROJECT / "src" / "taoryx_trajectory_contracts").glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module is not None:
                imports.add(node.module)
    assert not {item for item in imports if item == "taoryx" or item.startswith("taoryx.")}
    ####
