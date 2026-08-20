"""Structural provider protocols implemented without a TAORYX runtime dependency."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from .composition import (
    BatchRunRequest,
    BatchRunResult,
    CompositionConfiguration,
    PreparedCompositionConfiguration,
    TrajectoryProviderDescriptor,
)
from .streaming import (
    AuthorityTransition,
    ClosedStreamingSession,
    CloseStreamingSessionRequest,
    InspectStreamingSessionRequest,
    OpenStreamingSessionRequest,
    ResetStreamingSessionRequest,
    StreamingObservation,
    StreamingSessionDescriptor,
    StreamingStepRequest,
    StreamingStepResult,
    SwitchAuthorityRequest,
)


@runtime_checkable
class BatchCompositionProvider(Protocol):
    """Minimum protocol for a provider that can prepare and run a batch request."""

    @property
    def descriptor(self) -> TrajectoryProviderDescriptor:
        """Return the complete discoverable provider descriptor."""
        ...

    def prepare_configuration(self, configuration: CompositionConfiguration) -> PreparedCompositionConfiguration:
        """Validate and canonicalize one provider-owned configuration payload."""
        ...

    def run_batch(self, request: BatchRunRequest) -> BatchRunResult:
        """Run one prepared configuration without requiring streaming support."""
        ...


@runtime_checkable
class DefaultConfigurationProvider(BatchCompositionProvider, Protocol):
    """Optional provider-selected, runnable default-configuration extension.

    A default configuration is a batteries-included starting point selected by
    the provider. It is not a claim that its values are schema defaults or a
    physical vehicle's canonical operating condition. Hosts discover it from
    :attr:`TrajectoryModelDescriptor.default_configuration_id`, then still
    prepare it through the normal validation handoff before execution.
    """

    def build_default_configuration(self, model_id: str) -> CompositionConfiguration:
        """Return the advertised runnable default for one exact model."""
        ...


@runtime_checkable
class StreamingCompositionProvider(BatchCompositionProvider, Protocol):
    """Optional extension for a provider with stateful control streaming."""

    def open_stream(self, request: OpenStreamingSessionRequest) -> StreamingSessionDescriptor:
        """Open a provider-owned streaming session from prepared configuration."""
        ...

    def inspect_stream(self, request: InspectStreamingSessionRequest) -> StreamingObservation:
        """Inspect a session without advancing its state."""
        ...

    def step_stream(self, request: StreamingStepRequest) -> StreamingStepResult:
        """Advance a session through one accepted action duration."""
        ...

    def switch_stream_authority(self, request: SwitchAuthorityRequest) -> AuthorityTransition:
        """Transfer to a declared authority profile without advancing plant time."""
        ...

    def reset_stream(self, request: ResetStreamingSessionRequest) -> StreamingObservation:
        """Reset a session and return its new zero-boundary observation."""
        ...

    def close_stream(self, request: CloseStreamingSessionRequest) -> ClosedStreamingSession:
        """Release state held by one streaming session."""
        ...


__all__ = ["BatchCompositionProvider", "DefaultConfigurationProvider", "StreamingCompositionProvider"]
