"""Fast, implementation-neutral conformance checks for interface adopters."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, ConfigDict, Field

from .composition import BatchRunResult, TrajectoryProviderDescriptor
from .protocols import BatchCompositionProvider, DefaultConfigurationProvider, StreamingCompositionProvider
from .streaming import StreamingSessionDescriptor, StreamingStepResult


class ConformanceFinding(BaseModel):
    """One actionable nonconformance finding."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    code: str = Field(pattern=r"^[a-z0-9]+(?:[-.][a-z0-9]+)*$")
    message: str = Field(min_length=1)
    path: str | None = None


class ConformanceReport(BaseModel):
    """Portable result of auditing a provider or returned contract artifact."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    subject: str = Field(min_length=1)
    status: str
    findings: tuple[ConformanceFinding, ...] = ()
    facts: dict[str, Any] = Field(default_factory=dict)


def audit_provider_descriptor(descriptor: TrajectoryProviderDescriptor) -> ConformanceReport:
    """Audit discovery data without imposing TAORYX-specific capability rules.

    In particular, a provider that truthfully advertises batch-only support is
    conformant. Streaming is an optional extension, not an inferred promise.
    """

    findings: list[ConformanceFinding] = []
    exact_operations = tuple(
        operation
        for model in descriptor.models
        for operation in model.operations
    )
    available = tuple(item for item in exact_operations if item.availability == "available")
    batch_count = sum(item.operation == "batch" for item in available)
    step_count = sum(item.operation == "step" for item in available)
    if descriptor.contract_version != "1":
        findings.append(
            ConformanceFinding(
                code="unsupported-contract-version",
                message=f"contract version {descriptor.contract_version!r} is not supported by this v1 audit",
                path="contract_version",
            )
        )
    return ConformanceReport(
        subject=f"provider:{descriptor.id}",
        status="pass" if not findings else "fail",
        findings=tuple(findings),
        facts={
            "model_count": len(descriptor.models),
            "available_batch_tuple_count": batch_count,
            "available_step_tuple_count": step_count,
            "capability_shapes": {
                "batch_only_or_batch_and_step": sum(
                    any(item.operation == "batch" and item.availability == "available" for item in model.operations)
                    for model in descriptor.models
                ),
                "step_only": sum(
                    not any(item.operation == "batch" and item.availability == "available" for item in model.operations)
                    and any(item.operation == "step" and item.availability == "available" for item in model.operations)
                    for model in descriptor.models
                ),
            },
        },
    )
    ####


def audit_batch_provider(provider: object) -> ConformanceReport:
    """Check that a provider implements the batch protocol structurally."""

    if not isinstance(provider, BatchCompositionProvider):
        return ConformanceReport(
            subject=f"provider:{type(provider).__name__}",
            status="fail",
            findings=(
                ConformanceFinding(
                    code="missing-batch-protocol",
                    message="provider does not implement descriptor, prepare_configuration, and run_batch",
                ),
            ),
        )
    return audit_provider_descriptor(provider.descriptor)
    ####


def audit_streaming_provider(provider: object) -> ConformanceReport:
    """Check the optional streaming extension without requiring it of all providers."""

    if not isinstance(provider, StreamingCompositionProvider):
        return ConformanceReport(
            subject=f"provider:{type(provider).__name__}",
            status="fail",
            findings=(
                ConformanceFinding(
                    code="missing-streaming-protocol",
                    message="provider does not implement the optional streaming lifecycle extension",
                ),
            ),
        )
    return audit_provider_descriptor(provider.descriptor)
    ####


def audit_default_configuration_provider(provider: object) -> ConformanceReport:
    """Check the optional provider-selected runnable-default extension.

    This remains deliberately separate from the base batch protocol: a foreign
    provider may require caller-authored mission inputs and still be an honest
    batch provider. A host can use this audit before asking for a deterministic
    provider-selected starting configuration.
    """

    if not isinstance(provider, DefaultConfigurationProvider):
        return ConformanceReport(
            subject=f"provider:{type(provider).__name__}",
            status="fail",
            findings=(
                ConformanceFinding(
                    code="missing-default-configuration-protocol",
                    message="provider does not implement build_default_configuration(model_id)",
                ),
            ),
        )
    missing = tuple(model.id for model in provider.descriptor.models if model.default_configuration_id is None)
    findings = tuple(
        ConformanceFinding(
            code="missing-default-configuration-advertisement",
            message=f"model {model_id!r} does not advertise a default configuration ID",
            path=f"models.{model_id}.default_configuration_id",
        )
        for model_id in missing
    )
    return ConformanceReport(
        subject=f"provider:{provider.descriptor.id}",
        status="pass" if not findings else "fail",
        findings=findings,
        facts={
            "model_count": len(provider.descriptor.models),
            "advertised_default_configuration_count": len(provider.descriptor.models) - len(missing),
        },
    )
    ####


def audit_batch_result(result: BatchRunResult) -> ConformanceReport:
    """Confirm the standard ECEF state is present on every returned sample."""

    sample_count = sum(len(entity.samples) for entity in result.entities)
    return ConformanceReport(
        subject=f"batch-result:{result.request_id}",
        status="pass",
        facts={"entity_count": len(result.entities), "sample_count": sample_count},
    )
    ####


def audit_streaming_descriptor(descriptor: StreamingSessionDescriptor) -> ConformanceReport:
    """Confirm a session exposes a coherent zero-boundary descriptor."""

    return ConformanceReport(
        subject=f"stream-session:{descriptor.session_id}",
        status="pass",
        facts={
            "action_channel_count": len(descriptor.action_channels),
            "observation_channel_count": len(descriptor.observation_channels),
            "authority_profile_count": len(descriptor.authority_profiles),
        },
    )
    ####


def audit_streaming_step(result: StreamingStepResult) -> ConformanceReport:
    """Confirm one step exposes an accepted ECEF observation boundary."""

    return ConformanceReport(
        subject=f"stream-step:{result.session_id}:{result.sequence}",
        status="pass",
        facts={"time_start_s": result.time_start_s, "time_end_s": result.time_end_s},
    )
    ####


__all__ = [
    "ConformanceFinding",
    "ConformanceReport",
    "audit_batch_provider",
    "audit_batch_result",
    "audit_default_configuration_provider",
    "audit_provider_descriptor",
    "audit_streaming_descriptor",
    "audit_streaming_provider",
    "audit_streaming_step",
]
